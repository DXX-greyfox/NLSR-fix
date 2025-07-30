#ifndef NLSR_LOAD_AWARE_ROUTING_CALCULATOR_HPP
#define NLSR_LOAD_AWARE_ROUTING_CALCULATOR_HPP

#include "route/routing-table.hpp"
#include "link-cost-manager.hpp"
#include "common.hpp"

#include <ndn-cxx/util/time.hpp>
#include <unordered_map>
#include <deque>

#include "adjacency-list.hpp"  // ✅ 添加缺失的包含
#include "lsdb.hpp"           // ✅ 添加缺失的包含

namespace nlsr {

// 前向声明
class NameMap;
class ConfParameter;
class Lsdb;

/**
 * @brief 负载感知路由计算函数（主要对外接口）
 */
void calculateLoadAwareRoutingPath(NameMap& map, RoutingTable& rt, 
                                  ConfParameter& confParam, const Lsdb& lsdb, 
                                  LinkCostManager& linkCostManager);

/**
 * @brief 负载感知路由计算器辅助类（不继承任何类）
 */
class LoadAwareRoutingCalculator {
public:
  // 匹配你的实现文件
  explicit LoadAwareRoutingCalculator(LinkCostManager& linkCostManager);
  ~LoadAwareRoutingCalculator();
  
  // 匹配你的实现文件
  void calculatePath(NameMap& map, RoutingTable& rt, 
                    ConfParameter& confParam, const Lsdb& lsdb);

private:
  //void applyLoadAwareAdjustments(NameMap& map, RoutingTable& rt,ConfParameter& confParam, const Lsdb& lsdb);
                                
  
  //核心：负载感知成本计算方法
  double calculateLoadAwareCost(const ndn::Name& neighbor, double rttBasedCost, 
                               const LinkCostManager::LinkMetrics& metrics);
  
  double getRttFactor(const LinkCostManager::LinkMetrics& metrics);
  double getLoadFactor(const LinkCostManager::LinkMetrics& metrics);
  double getStabilityFactor(const LinkCostManager::LinkMetrics& metrics);
  
  //double getOriginalLinkCost(const ndn::Name& sourceRouter,const ndn::Name& targetRouter);
  
  void updateRttHistory(const ndn::Name& neighbor, double currentRttMs);
  //bool shouldSuppressUpdate(const ndn::Name& neighbor, double newCost);
  //void recordCostUpdate(const ndn::Name& neighbor, double newCost);
  //void adjustRoutingTableCosts(RoutingTable& rt, const ndn::Name& neighbor,double originalCost, double newCost);
                              

private:
  LinkCostManager& m_linkCostManager;
  double m_rttWeight = 0.3;
  double m_loadWeight = 0.4;
  double m_stabilityWeight = 0.3;
  
  std::unordered_map<ndn::Name, std::deque<double>> m_rttHistory;
  static constexpr size_t MAX_RTT_HISTORY = 10;
  
  struct CostUpdateRecord {
    double cost;
    ndn::time::steady_clock::time_point timestamp;
  };
  //std::unordered_map<ndn::Name, std::deque<CostUpdateRecord>> m_costUpdateHistory;
  static constexpr size_t MAX_UPDATE_HISTORY = 5;
  static constexpr auto MIN_UPDATE_INTERVAL = ndn::time::seconds(5);
  static constexpr double MIN_COST_CHANGE_RATIO = 0.05;
  
  uint64_t m_calculationCount = 0;
  uint64_t m_costAdjustmentCount = 0;
  //uint64_t m_suppressedUpdates;
  //添加引用
  /*先暂时禁用
  AdjacencyList& m_adjacencyList;
  Lsdb& m_lsdb;
  RoutingTable& m_routingTable;
  */
};

} // namespace nlsr

#endif // NLSR_LOAD_AWARE_ROUTING_CALCULATOR_HPP