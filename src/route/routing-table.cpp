/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2024,  The University of Memphis,
 *                           Regents of the University of California
 */

#include "routing-table.hpp"
#include "name-map.hpp"
#include "routing-calculator.hpp"
#include "routing-table-entry.hpp"
#include "load-aware-routing-calculator.hpp"
#include "ml-adaptive-calculator.hpp"  // 注意：文件名要与实际文件名一致

#include "conf-parameter.hpp"
#include "logger.hpp"
#include "nlsr.hpp"
#include "tlv-nlsr.hpp"

namespace nlsr {

INIT_LOGGER(route.RoutingTable);

RoutingTable::RoutingTable(ndn::Scheduler& scheduler, Lsdb& lsdb, ConfParameter& confParam)
  // ✅ 教学要点：初始化列表顺序必须与头文件中成员声明顺序完全一致
  // 这是C++的基本要求，违反会导致编译警告甚至未定义行为
  : m_scheduler(scheduler)
  , m_lsdb(lsdb)
  , m_confParam(confParam)
  , m_hyperbolicState(m_confParam.getHyperbolicState())
  , m_routingCalcInterval{confParam.getRoutingCalcInterval()}
  , m_isRoutingTableCalculating(false)
  , m_isRouteCalculationScheduled(false)
  , m_ownAdjLsaExist(false)
  // m_afterLsdbModified 会在构造函数体中设置
  , m_linkCostManager(nullptr)
  // ✅ 智能指针初始化：nullptr是现代C++的最佳实践
  , m_loadAwareCalculator(nullptr)
  , m_mlAdaptiveCalculator(nullptr)
{
  // ✅ 教学要点：信号连接在所有成员初始化完成后进行
  // 这确保了回调函数中引用的所有成员都已正确初始化
  m_afterLsdbModified = lsdb.onLsdbModified.connect(
    [this] (std::shared_ptr<Lsa> lsa, LsdbUpdate updateType,
            const auto& namesToAdd, const auto& namesToRemove) {
      auto type = lsa->getType();
      bool updateForOwnAdjacencyLsa = lsa->getOriginRouter() == m_confParam.getRouterPrefix() &&
                                      type == Lsa::Type::ADJACENCY;
      bool scheduleCalculation = false;

      if (updateType == LsdbUpdate::REMOVED && updateForOwnAdjacencyLsa) {
        NLSR_LOG_DEBUG("No Adj LSA of router itself, routing table can not be calculated :(");
        clearRoutingTable();
        clearDryRoutingTable();
        NLSR_LOG_DEBUG("Calling Update NPT With new Route");
        afterRoutingChange(m_rTable);
        NLSR_LOG_DEBUG(*this);
        m_ownAdjLsaExist = false;
      }

      if (updateType == LsdbUpdate::INSTALLED && updateForOwnAdjacencyLsa) {
        m_ownAdjLsaExist = true;
      }

      if (updateType == LsdbUpdate::INSTALLED || updateType == LsdbUpdate::UPDATED) {
        if ((type == Lsa::Type::ADJACENCY  && m_hyperbolicState != HYPERBOLIC_STATE_ON) ||
            (type == Lsa::Type::COORDINATE && m_hyperbolicState != HYPERBOLIC_STATE_OFF)) {
          scheduleCalculation = true;
        }
      }

      if (scheduleCalculation) {
        scheduleRoutingTableCalculation();
      }
    }
  );
}

// ✅ 教学要点：析构函数在cpp文件中定义的重要性
// 此时编译器已知完整的类定义，可以正确生成unique_ptr的析构代码
RoutingTable::~RoutingTable()
{
  m_afterLsdbModified.disconnect();
  // unique_ptr会自动管理LoadAwareRoutingCalculator和MLAdaptiveCalculator的生命周期
}

void RoutingTable::calculate()
{
  m_lsdb.writeLog();
  NLSR_LOG_TRACE("Calculating routing table");

  if (m_isRoutingTableCalculating == false) {
    m_isRoutingTableCalculating = true;//开启算法计算标志位

    // ✅ 教学要点：算法优先级设计的考虑
    // ML自适应算法优先级最高，因为它能学习和适应网络变化
    // 负载感知算法次之，因为它能动态响应网络负载
    // 标准算法优先级最低，作为基础备选方案
    if (m_confParam.getMLAdaptiveRouting()) {
      NLSR_LOG_INFO("Using ML-adaptive routing algorithm");
      calculateMLAdaptiveRoutingTable();
    }
    else if (m_confParam.getLoadAwareRouting()) {
      NLSR_LOG_INFO("Using load-aware routing algorithm");
      //NLSR_LOG_INFO("Starting!")
      calculateLoadAwareRoutingTable();
    }
    else if (m_hyperbolicState == HYPERBOLIC_STATE_OFF) {
      NLSR_LOG_INFO("Using standard link-state routing algorithm");
      calculateLsRoutingTable();
    }
    else if (m_hyperbolicState == HYPERBOLIC_STATE_DRY_RUN) {
      NLSR_LOG_INFO("Using hyperbolic routing (dry-run mode)");
      calculateLsRoutingTable();
      calculateHypRoutingTable(true);
    }
    else if (m_hyperbolicState == HYPERBOLIC_STATE_ON) {
      NLSR_LOG_INFO("Using hyperbolic routing algorithm");
      calculateHypRoutingTable(false);
    }

    m_isRouteCalculationScheduled = false;
    m_isRoutingTableCalculating = false;
  }
  else {
    scheduleRoutingTableCalculation();
  }
}

// ✅ 保持负载感知算法的成功实现完全不变
void
RoutingTable::calculateLoadAwareRoutingTable()
{
  NLSR_LOG_TRACE("CalculateLoadAwareRoutingTable Called");

  if (m_lsdb.getIsBuildAdjLsaScheduled()) {
    NLSR_LOG_DEBUG("Adjacency build is scheduled, routing table can not be calculated :(");
    return;
  }

  if (!m_ownAdjLsaExist) {
    return;
  }

  clearRoutingTable();
  
  if (m_linkCostManager == nullptr) {
    NLSR_LOG_WARN("LinkCostManager not available, falling back to standard routing");
    calculateLsRoutingTable();
    return;
  }

  auto lsaRange = m_lsdb.getLsdbIterator<AdjLsa>();
  auto map = NameMap::createFromAdjLsdb(lsaRange.first, lsaRange.second);
  NLSR_LOG_DEBUG(map);

  // ✅ 教学要点：懒加载模式的优势
  // 只在第一次使用时创建对象，避免不必要的资源消耗
  // 同时确保对象生命周期与RoutingTable一致，避免回调丢失
  if (!m_loadAwareCalculator) {
    NLSR_LOG_INFO("Creating persistent LoadAwareRoutingCalculator (first time)");
    m_loadAwareCalculator = std::make_unique<LoadAwareRoutingCalculator>(*m_linkCostManager);
  }

  m_loadAwareCalculator->calculatePath(map, *this, m_confParam, m_lsdb);

  NLSR_LOG_DEBUG("Calling Update NPT With new Route");
  afterRoutingChange(m_rTable);
  NLSR_LOG_DEBUG(*this);
}

// ✅ 新增：ML自适应路由表计算方法
// 完全模仿负载感知算法的成功模式，确保架构一致性
void
RoutingTable::calculateMLAdaptiveRoutingTable()
{
  NLSR_LOG_TRACE("CalculateMLAdaptiveRoutingTable Called");

  // ✅ 复用负载感知算法验证过的前置检查逻辑
  if (m_lsdb.getIsBuildAdjLsaScheduled()) {
    NLSR_LOG_DEBUG("Adjacency build is scheduled, routing table can not be calculated :(");
    return;
  }
  
  if (!m_ownAdjLsaExist) {
    return;
  }

  clearRoutingTable();
  
  // 如果关键依赖不可用，自动降级到可靠的备选方案
  if (m_linkCostManager == nullptr) {
    NLSR_LOG_WARN("LinkCostManager not available, falling back to standard routing");
    calculateLsRoutingTable();
    return;
  }

  auto lsaRange = m_lsdb.getLsdbIterator<AdjLsa>();
  auto map = NameMap::createFromAdjLsdb(lsaRange.first, lsaRange.second);
  NLSR_LOG_DEBUG(map);

  // 严格遵循负载感知算法的成功模式
  // 使用相同的懒加载策略，确保ML学习状态的持久性
  if (!m_mlAdaptiveCalculator) {
    NLSR_LOG_INFO("Creating persistent MLAdaptiveCalculator (first time)");
    m_mlAdaptiveCalculator = std::make_unique<MLAdaptiveCalculator>(*m_linkCostManager);
    // ✅ 关键：建立ML反馈连接
    if (m_linkCostManager) {
      m_linkCostManager->setMLFeedbackCallback(
        [this](const ndn::Name& neighbor, double performance) {
          if (m_mlAdaptiveCalculator) {
            m_mlAdaptiveCalculator->reportPathPerformance(neighbor, performance);
            NLSR_LOG_TRACE("🔄 ML learning cycle: " << neighbor 
                          << " performance=" << performance);
          }
        });
      
      NLSR_LOG_INFO("🔗 ML feedback loop established between LinkCostManager and MLAdaptiveCalculator");
    }
  }

  // ✅ 关键设计：直接调用持久化对象方法，避免临时对象陷阱
  m_mlAdaptiveCalculator->calculatePath(map, *this, m_confParam, m_lsdb);

  NLSR_LOG_DEBUG("Calling Update NPT With new Route");
  afterRoutingChange(m_rTable);
  NLSR_LOG_DEBUG(*this);
}

// ✅ 其他方法保持完全不变
void
RoutingTable::calculateLsRoutingTable()
{
  NLSR_LOG_TRACE("CalculateLsRoutingTable Called");

  if (m_lsdb.getIsBuildAdjLsaScheduled()) {
    NLSR_LOG_DEBUG("Adjacency build is scheduled, routing table can not be calculated :(");
    return;
  }

  if (!m_ownAdjLsaExist) {
    return;
  }

  clearRoutingTable();

  auto lsaRange = m_lsdb.getLsdbIterator<AdjLsa>();
  auto map = NameMap::createFromAdjLsdb(lsaRange.first, lsaRange.second);
  NLSR_LOG_DEBUG(map);

  calculateLinkStateRoutingPath(map, *this, m_confParam, m_lsdb);

  NLSR_LOG_DEBUG("Calling Update NPT With new Route");
  afterRoutingChange(m_rTable);
  NLSR_LOG_DEBUG(*this);
}

void
RoutingTable::calculateHypRoutingTable(bool isDryRun)
{
  if (isDryRun) {
    clearDryRoutingTable();
  }
  else {
    clearRoutingTable();
  }

  auto lsaRange = m_lsdb.getLsdbIterator<CoordinateLsa>();
  auto map = NameMap::createFromCoordinateLsdb(lsaRange.first, lsaRange.second);
  NLSR_LOG_DEBUG(map);

  calculateHyperbolicRoutingPath(map, *this, m_lsdb, m_confParam.getAdjacencyList(),
                                 m_confParam.getRouterPrefix(), isDryRun);

  if (!isDryRun) {
    NLSR_LOG_DEBUG("Calling Update NPT With new Route");
    afterRoutingChange(m_rTable);
    NLSR_LOG_DEBUG(*this);
  }
}

void
RoutingTable::scheduleRoutingTableCalculation()
{
  if (!m_isRouteCalculationScheduled) {
    NLSR_LOG_DEBUG("Scheduling routing table calculation in " << m_routingCalcInterval);
    m_scheduler.schedule(m_routingCalcInterval, [this] { calculate(); });
    m_isRouteCalculationScheduled = true;
  }
}

static bool
routingTableEntryCompare(RoutingTableEntry& rte, ndn::Name& destRouter)
{
  return rte.getDestination() == destRouter;
}

void
RoutingTable::addNextHop(const ndn::Name& destRouter, NextHop& nh)
{
  NLSR_LOG_DEBUG("Adding " << nh << " for destination: " << destRouter);

  RoutingTableEntry* rteChk = findRoutingTableEntry(destRouter);
  if (rteChk == nullptr) {
    RoutingTableEntry rte(destRouter);
    rte.getNexthopList().addNextHop(nh);
    m_rTable.push_back(rte);
  }
  else {
    rteChk->getNexthopList().addNextHop(nh);
  }
  m_wire.reset();
}

RoutingTableEntry*
RoutingTable::findRoutingTableEntry(const ndn::Name& destRouter)
{
  auto it = std::find_if(m_rTable.begin(), m_rTable.end(),
                         std::bind(&routingTableEntryCompare, _1, destRouter));
  if (it != m_rTable.end()) {
    return &(*it);
  }
  return nullptr;
}

void
RoutingTable::addNextHopToDryTable(const ndn::Name& destRouter, NextHop& nh)
{
  NLSR_LOG_DEBUG("Adding " << nh << " to dry table for destination: " << destRouter);

  auto it = std::find_if(m_dryTable.begin(), m_dryTable.end(),
                         std::bind(&routingTableEntryCompare, _1, destRouter));
  if (it == m_dryTable.end()) {
    RoutingTableEntry rte(destRouter);
    rte.getNexthopList().addNextHop(nh);
    m_dryTable.push_back(rte);
  }
  else {
    it->getNexthopList().addNextHop(nh);
  }
  m_wire.reset();
}

void
RoutingTable::clearRoutingTable()
{
  m_rTable.clear();
  m_wire.reset();
}

void
RoutingTable::clearDryRoutingTable()
{
  m_dryTable.clear();
  m_wire.reset();
}

// 其余方法保持不变...
template<ndn::encoding::Tag TAG>
size_t
RoutingTableStatus::wireEncode(ndn::EncodingImpl<TAG>& block) const
{
  size_t totalLength = 0;

  for (auto it = m_dryTable.rbegin(); it != m_dryTable.rend(); ++it) {
    totalLength += it->wireEncode(block);
  }

  for (auto it = m_rTable.rbegin(); it != m_rTable.rend(); ++it) {
    totalLength += it->wireEncode(block);
  }

  totalLength += block.prependVarNumber(totalLength);
  totalLength += block.prependVarNumber(nlsr::tlv::RoutingTable);

  return totalLength;
}

NDN_CXX_DEFINE_WIRE_ENCODE_INSTANTIATIONS(RoutingTableStatus);

const ndn::Block&
RoutingTableStatus::wireEncode() const
{
  if (m_wire.hasWire()) {
    return m_wire;
  }

  ndn::EncodingEstimator estimator;
  size_t estimatedSize = wireEncode(estimator);

  ndn::EncodingBuffer buffer(estimatedSize, 0);
  wireEncode(buffer);

  m_wire = buffer.block();

  return m_wire;
}

void
RoutingTableStatus::wireDecode(const ndn::Block& wire)
{
  m_rTable.clear();

  m_wire = wire;

  if (m_wire.type() != nlsr::tlv::RoutingTable) {
    NDN_THROW(Error("RoutingTable", m_wire.type()));
  }

  m_wire.parse();
  auto val = m_wire.elements_begin();

  std::set<ndn::Name> destinations;
  for (; val != m_wire.elements_end() && val->type() == nlsr::tlv::RoutingTableEntry; ++val) {
    auto entry = RoutingTableEntry(*val);

    if (destinations.emplace(entry.getDestination()).second) {
      m_rTable.push_back(entry);
    }
    else {
      m_dryTable.push_back(entry);
    }
  }

  if (val != m_wire.elements_end()) {
    NDN_THROW(Error("Unrecognized TLV of type " + ndn::to_string(val->type()) + " in RoutingTable"));
  }
}

std::ostream&
operator<<(std::ostream& os, const RoutingTableStatus& rts)
{
  os << "Routing Table:\n";
  for (const auto& rte : rts.getRoutingTableEntry()) {
    os << rte;
  }

  if (!rts.getDryRoutingTableEntry().empty()) {
    os << "Dry-Run Hyperbolic Routing Table:\n";
    for (const auto& rte : rts.getDryRoutingTableEntry()) {
      os << rte;
    }
  }
  return os;
}

} // namespace nlsr