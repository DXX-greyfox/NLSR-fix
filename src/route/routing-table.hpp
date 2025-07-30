/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2023,  The University of Memphis,
 *                           Regents of the University of California
 */

#ifndef NLSR_ROUTING_TABLE_HPP
#define NLSR_ROUTING_TABLE_HPP

#include "conf-parameter.hpp"
#include "routing-table-entry.hpp"
#include "signals.hpp"
#include "lsdb.hpp"
#include "route/fib.hpp"
#include "test-access-control.hpp"
#include "route/name-prefix-table.hpp"

#include <ndn-cxx/util/scheduler.hpp>
#include <memory>

namespace nlsr {

class NextHop;
// ✅ 关键修正：严格使用前向声明，避免头文件包含复杂性
class LoadAwareRoutingCalculator;
class MLAdaptiveCalculator;  // 注意：类名要与ml-adaptive-calculator.hpp中一致
class Nlsr;
class LinkCostManager;

class RoutingTableStatus
{
public:
  using Error = ndn::tlv::Error;

  RoutingTableStatus() = default;

  RoutingTableStatus(const ndn::Block& block)
  {
    wireDecode(block);
  }

  const std::list<RoutingTableEntry>&
  getRoutingTableEntry() const
  {
    return m_rTable;
  }

  const std::list<RoutingTableEntry>&
  getDryRoutingTableEntry() const
  {
    return m_dryTable;
  }

  const ndn::Block&
  wireEncode() const;

private:
  void
  wireDecode(const ndn::Block& wire);

  template<ndn::encoding::Tag TAG>
  size_t
  wireEncode(ndn::EncodingImpl<TAG>& block) const;

PUBLIC_WITH_TESTS_ELSE_PROTECTED:
  std::list<RoutingTableEntry> m_dryTable;
  std::list<RoutingTableEntry> m_rTable;
  mutable ndn::Block m_wire;
};

std::ostream&
operator<<(std::ostream& os, const RoutingTableStatus& rts);

class RoutingTable : public RoutingTableStatus
{
public:
  explicit
  RoutingTable(ndn::Scheduler& scheduler, Lsdb& lsdb, ConfParameter& confParam);

  // ✅ 显式声明析构函数，在cpp文件中定义
  ~RoutingTable();

  void
  calculate();

  void
  addNextHop(const ndn::Name& destRouter, NextHop& nh);

  void
  addNextHopToDryTable(const ndn::Name& destRouter, NextHop& nh);

  bool isMLAdaptiveEnabled() const { return m_confParam.getMLAdaptiveRouting(); }

  RoutingTableEntry*
  findRoutingTableEntry(const ndn::Name& destRouter);

  void
  scheduleRoutingTableCalculation();

private:
  void
  calculateLsRoutingTable();

  void
  calculateHypRoutingTable(bool isDryRun);

  void
  clearRoutingTable();

  void
  clearDryRoutingTable();

  // ✅ 负载感知路由计算方法（已有）
  void
  calculateLoadAwareRoutingTable();

  // ✅ ML自适应路由计算方法（新增）
  void
  calculateMLAdaptiveRoutingTable();

public:
  AfterRoutingChange afterRoutingChange;

  void setLinkCostManager(LinkCostManager* linkCostManager) {
    m_linkCostManager = linkCostManager;
  }

private:
  // ✅ 成员变量顺序：严格按照初始化依赖关系排列
  ndn::Scheduler& m_scheduler;
  Lsdb& m_lsdb;
  ConfParameter& m_confParam;
  
  int32_t m_hyperbolicState;
  ndn::time::seconds m_routingCalcInterval; 
  bool m_isRoutingTableCalculating;
  bool m_isRouteCalculationScheduled;
  bool m_ownAdjLsaExist;
  
  ndn::signal::Connection m_afterLsdbModified;
  LinkCostManager* m_linkCostManager;
  
  // ✅ 关键：两个算法使用完全相同的持久化对象模式
  std::unique_ptr<LoadAwareRoutingCalculator> m_loadAwareCalculator;
  std::unique_ptr<MLAdaptiveCalculator> m_mlAdaptiveCalculator;  // 注意类名

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  // 测试访问控制成员保持不变
};

} // namespace nlsr

#endif // NLSR_ROUTING_TABLE_HPP