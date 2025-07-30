/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2025,  The University of Memphis,
 *                           Regents of the University of California,
 *                           Arizona Board of Regents.
 */

#ifndef NLSR_ML_ADAPTIVE_CALCULATOR_HPP
#define NLSR_ML_ADAPTIVE_CALCULATOR_HPP

// ✅ 现在可以安全地包含 common.hpp
// 因为当前版本已经清除了所有命名空间污染源
#include "common.hpp"

// 标准库头文件
#include <vector>
#include <unordered_map>
#include <deque>
#include <functional>
#include <array>

// NDN-CXX库头文件 
#include <ndn-cxx/name.hpp>
#include <ndn-cxx/util/time.hpp>

// 本地头文件（只包含必要的前向声明）
#include "route/routing-table.hpp"
#include "link-cost-manager.hpp"

namespace nlsr {

// 前向声明（避免不必要的头文件包含）
class NameMap;
class ConfParameter; 
class Lsdb;

/**
 * @brief 机器学习自适应路由计算器
 * 
 * ✅ 教学要点：现在我们可以安全地使用 common.hpp 中定义的类型和常量
 * 比如时间字面量（TIME_ALLOWED_FOR_CANONIZATION）和 NDN 类型别名
 */
class MLAdaptiveCalculator {
public:
  /**
   * @brief 构造函数
   * @param linkCostManager LinkCostManager的引用，用于智能成本计算集成
   */
  explicit MLAdaptiveCalculator(LinkCostManager& linkCostManager);
  
  /**
   * @brief 析构函数，自动清理回调注册
   */
  ~MLAdaptiveCalculator();
  
  /**
   * @brief 执行路由路径计算
   */
  void calculatePath(NameMap& map, RoutingTable& rt, 
                    ConfParameter& confParam, const Lsdb& lsdb);

  /**
   * @brief 报告路径的实际性能（用于在线学习）
   * @param neighbor 邻居节点名称
   * @param actualPerformance 实际性能值 (0-1，越低越好)
   */
  void reportPathPerformance(const ndn::Name& neighbor, double actualPerformance);

  /**
   * @brief ML算法统计信息
   */
  struct Statistics {
    uint64_t predictionCount = 0;
    uint64_t modelUpdateCount = 0;
    uint64_t patternDetectionCount = 0;
    double averagePredictionError = 0.0;
  };
  
  const Statistics& getStatistics() const { return m_statistics; }

private:
  /**
   * @brief 轻量级线性回归模型
   */
  class LinearRegressionModel {
  public:
    explicit LinearRegressionModel(size_t featureCount);
    
    double predict(const std::vector<double>& features) const;
    
    void updateOnline(const std::vector<double>& features, 
                     double target, double learningRate);
    
    const std::vector<double>& getWeights() const { return m_weights; }
    
  private:
    std::vector<double> m_weights;
    double m_bias;
    size_t m_updateCount;
  };

  /**
   * @brief 时间模式学习器
   */
  class TemporalPatternLearner {
  public:
    struct TimeSlot {
      int hour;
      int minute; 
      double averagePerformance;
      int sampleCount;
      ndn::time::steady_clock::time_point lastUpdate;
    };
    
    void updatePattern(const ndn::Name& neighbor, double performance);
    double getTimeFeature(const ndn::Name& neighbor) const;
    
  private:
    std::unordered_map<ndn::Name, std::unordered_map<int, TimeSlot>> m_timePatterns;
    
    int getTimeSlotKey(int hour, int minute) const {
      return hour * 60 + minute;
    }
  };

  // ✅ 核心算法接口
  std::vector<double> extractCoreFeatures(const ndn::Name& neighbor);
  double predictLinkQuality(const ndn::Name& neighbor, 
                           const LinkCostManager::LinkMetrics& metrics);
  double predictWithFixedWeights(const std::vector<double>& features);

  // ✅ 特征工程函数
  double calculateRttTrend(const ndn::Name& neighbor);
  double calculateRttVariationCoefficient(const ndn::Name& neighbor);
  double calculateSuccessRate(const ndn::Name& neighbor);
  double calculateLoadIndicator(const ndn::Name& neighbor);

  // ✅ 在线学习机制
  void updateModelWithFeedback(const ndn::Name& neighbor,
                              const std::vector<double>& features, 
                              double actualPerformance);
  bool shouldTriggerModelUpdate(double predictionError);
  void adaptLearningRate();

private:
  // ✅ 关键：核心依赖关系
  LinkCostManager& m_linkCostManager;
  std::unique_ptr<LinearRegressionModel> m_model;
  std::unique_ptr<TemporalPatternLearner> m_patternLearner;

  // ✅ 学习参数
  double m_learningRate;
  double m_adaptationThreshold;
  
  // ✅ 算法常量
  static constexpr size_t FEATURE_COUNT = 5;
  static constexpr std::array<double, 4> FIXED_WEIGHTS{{0.4, 0.3, 0.2, 0.1}};
  
  // ✅ 数据缓存
  struct PerformanceRecord {
    double predictedScore;
    double actualPerformance;
    ndn::time::steady_clock::time_point timestamp;
  };
  
  std::unordered_map<ndn::Name, std::deque<PerformanceRecord>> m_performanceHistory;
  std::unordered_map<ndn::Name, std::deque<double>> m_rttHistory;
  
  static constexpr size_t MAX_PERFORMANCE_HISTORY = 100;
  static constexpr size_t MAX_RTT_HISTORY = 20;
  
  // ✅ 运行时状态
  mutable Statistics m_statistics;
  bool m_isModelReady;
  ndn::time::steady_clock::time_point m_lastModelUpdate;
  
  // ✅ 现在可以安全地使用 common.hpp 中的时间常量
  static constexpr auto MIN_UPDATE_INTERVAL = 30_s;  // 使用时间字面量
  
  // ✅ 枚举类型定义
  enum class LinkQuality { EXCELLENT, GOOD, FAIR, POOR };
  LinkQuality categorizeLinkQuality(const std::vector<double>& features);
};

} // namespace nlsr

#endif // NLSR_ML_ADAPTIVE_CALCULATOR_HPP