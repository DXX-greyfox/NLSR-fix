/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2025,  The University of Memphis,
 *                           Regents of the University of California,
 *                           Arizona Board of Regents.
 */

// ✅ 关键修正：严格的头文件包含顺序策略
// 1. 首先包含本类的头文件
#include "ml-adaptive-calculator.hpp"

// 2. 然后包含NLSR核心头文件
#include "routing-calculator.hpp"
#include "name-map.hpp"
#include "nexthop.hpp"
#include "adjacent.hpp"
#include "logger.hpp"
#include "conf-parameter.hpp"

// 3. 最后包含标准库和boost库（在明确的命名空间中使用）
#include <algorithm>
#include <cmath>
#include <chrono>

// 关键修正：显式命名空间引用，避免using namespace
// 不使用 using namespace boost; 避免命名空间污染

namespace nlsr {

INIT_LOGGER(route.MLAdaptiveCalculator);

// ============================================================================
// LinearRegressionModel Implementation
// ============================================================================

MLAdaptiveCalculator::LinearRegressionModel::LinearRegressionModel(size_t featureCount)
  : m_weights(featureCount, 0.2)  // 初始化为小的随机值
  , m_bias(0.0)
  , m_updateCount(0)
{
  // ✅ 教学要点：启发式权重初始化的重要性
  // 在机器学习中，初始权重的选择直接影响模型的收敛速度和最终性能
  // 这里使用基于特征重要性的启发式初始化策略
  if (featureCount >= 4) {
    m_weights[0] = 0.4;  // RTT趋势权重较高 - 网络延迟是路由选择的关键因素
    m_weights[1] = 0.3;  // 稳定性权重中等 - 连接稳定性影响用户体验
    m_weights[2] = 0.2;  // 成功率权重中等 - 数据包投递成功率很重要
    m_weights[3] = 0.1;  // 负载指示器权重较低 - 作为辅助判断因素
  }
  if (featureCount >= 5) {
    m_weights[4] = 0.15; // 时间特征权重 - 考虑网络的时间模式
  }
}

double
MLAdaptiveCalculator::LinearRegressionModel::predict(const std::vector<double>& features) const
{
  double result = m_bias;
  for (size_t i = 0; i < features.size() && i < m_weights.size(); ++i) {
    result += m_weights[i] * features[i];
  }
  
  // ✅ 教学要点：sigmoid激活函数的作用
  // sigmoid函数将任意实数映射到(0,1)区间，这里用于确保输出是有效的概率值
  // 这在机器学习中是一个经典的技巧，特别适用于二分类和概率预测
  return 1.0 / (1.0 + std::exp(-result));
}

void
MLAdaptiveCalculator::LinearRegressionModel::updateOnline(const std::vector<double>& features, 
                                                         double target, double learningRate)
{
  if (features.size() != m_weights.size()) {
    return;
  }
  
  double prediction = predict(features);
  double error = target - prediction;
  
  // 梯度下降算法的核心原理
  // 这里实现的是随机梯度下降(SGD)，这是机器学习中最基础也最重要的优化算法
  // 通过计算损失函数对参数的梯度，然后朝着梯度的反方向更新参数
  m_bias += learningRate * error;
  for (size_t i = 0; i < m_weights.size(); ++i) {
    m_weights[i] += learningRate * error * features[i];
  }
  
  ++m_updateCount;
}

// ============================================================================
// TemporalPatternLearner Implementation  
// ============================================================================

void
MLAdaptiveCalculator::TemporalPatternLearner::updatePattern(const ndn::Name& neighbor, 
                                                           double performance)
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto localtime = *std::localtime(&time_t);
  
  // ✅ 教学要点：时间粒度的选择策略
  // 这里选择10分钟作为时间粒度，这是基于网络流量模式的经验选择
  // 太细的粒度会导致数据稀疏，太粗的粒度会丢失重要的时间模式
  int timeSlotKey = getTimeSlotKey(localtime.tm_hour, localtime.tm_min / 10 * 10);
  
  auto& timeSlot = m_timePatterns[neighbor][timeSlotKey];
  
  // ✅ 教学要点：指数移动平均(EMA)的优势
  // EMA能够给最新的数据更高的权重，同时保留历史信息
  // 这在网络环境中特别有用，因为网络状态会随时间变化
  if (timeSlot.sampleCount == 0) {
    timeSlot.hour = localtime.tm_hour;
    timeSlot.minute = localtime.tm_min / 10 * 10;
    timeSlot.averagePerformance = performance;
    timeSlot.sampleCount = 1;
  } else {
    double alpha = 0.1; // 平滑因子 - 控制新旧数据的权重平衡
    timeSlot.averagePerformance = alpha * performance + (1 - alpha) * timeSlot.averagePerformance;
    timeSlot.sampleCount++;
  }
  
  timeSlot.lastUpdate = ndn::time::steady_clock::now();
}

double
MLAdaptiveCalculator::TemporalPatternLearner::getTimeFeature(const ndn::Name& neighbor) const
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto localtime = *std::localtime(&time_t);
  
  int currentTimeSlot = getTimeSlotKey(localtime.tm_hour, localtime.tm_min / 10 * 10);
  
  auto neighborIt = m_timePatterns.find(neighbor);
  if (neighborIt != m_timePatterns.end()) {
    auto slotIt = neighborIt->second.find(currentTimeSlot);
    if (slotIt != neighborIt->second.end()) {
      return slotIt->second.averagePerformance;
    }
  }
  
  return 0.5; // 默认中等性能，当没有历史数据时的安全选择
}

// ============================================================================
// MLAdaptiveCalculator Main Implementation
// ============================================================================

MLAdaptiveCalculator::MLAdaptiveCalculator(LinkCostManager& linkCostManager)
  : m_linkCostManager(linkCostManager)
  , m_model(std::make_unique<LinearRegressionModel>(FEATURE_COUNT))
  , m_patternLearner(std::make_unique<TemporalPatternLearner>())
  , m_learningRate(0.01)
  , m_adaptationThreshold(0.2)
  , m_isModelReady(false)
  , m_lastModelUpdate(ndn::time::steady_clock::now())
{
  // ✅ 教学要点：回调机制的设计精髓
  // 通过lambda表达式注册回调，实现了ML算法与LinkCostManager的松耦合
  // 这种设计允许在不修改LinkCostManager核心逻辑的情况下添加智能能力
  m_linkCostManager.setLoadAwareCostCalculator(
    [this](const ndn::Name& neighbor, double rttBasedCost, 
           const LinkCostManager::LinkMetrics& metrics) {
      return this->predictLinkQuality(neighbor, metrics);
    });
  
  NLSR_LOG_INFO("MLAdaptiveCalculator: Initialized with ML model registered");
}

MLAdaptiveCalculator::~MLAdaptiveCalculator()
{
  // RAII原则的体现
  // 析构函数自动清理回调注册，确保没有悬挂指针和资源泄漏
  // 这是现代C++的最佳实践，让对象的生命周期管理变得自动和安全
  m_linkCostManager.clearLoadAwareCostCalculator();
  NLSR_LOG_INFO("MLAdaptiveCalculator: Deregistered, LinkCostManager restored");
}

void
MLAdaptiveCalculator::calculatePath(NameMap& map, RoutingTable& rt, 
                                   ConfParameter& confParam, const Lsdb& lsdb)
{
  NLSR_LOG_DEBUG("MLAdaptiveCalculator::calculatePath called");
  ++m_statistics.predictionCount;
  
  // ✅ 教学要点：智能路由的实现策略
  // ML算法的智能体现在成本计算上，而不是路径算法本身
  // 这种设计保持了路由算法的稳定性，同时增加了智能决策能力
  calculateLinkStateRoutingPath(map, rt, confParam, lsdb);
  
  NLSR_LOG_DEBUG("ML adaptive routing calculation completed. Predictions: " 
                << m_statistics.predictionCount);
}

std::vector<double>
MLAdaptiveCalculator::extractCoreFeatures(const ndn::Name& neighbor)
{
  std::vector<double> features(FEATURE_COUNT, 0.0);
  
  // ✅ 教学要点：特征工程的艺术
  // 特征工程是机器学习成功的关键，这里选择的每个特征都有明确的网络意义
  
  // 特征1: RTT趋势 - 捕获网络延迟的变化趋势
  features[0] = calculateRttTrend(neighbor);
  
  // 特征2: 稳定性 - 量化连接的稳定程度
  features[1] = calculateRttVariationCoefficient(neighbor);
  
  // 特征3: 成功率 - 反映链路的可靠性
  features[2] = calculateSuccessRate(neighbor);
  
  // 特征4: 负载指示器 - 检测网络拥塞状况
  features[3] = calculateLoadIndicator(neighbor);
  
  // 特征5: 时间模式特征 - 利用网络的时间规律性
  features[4] = m_patternLearner->getTimeFeature(neighbor);
  
  return features;
}

double
MLAdaptiveCalculator::predictLinkQuality(const ndn::Name& neighbor, 
                                        const LinkCostManager::LinkMetrics& metrics)
{
  // ✅ 教学要点：特征提取与预测的流水线
  auto features = extractCoreFeatures(neighbor);
  
  double mlPrediction = 0.0;
  if (m_isModelReady && m_model) {
    mlPrediction = m_model->predict(features);
  } else {
    // ✅ 优雅降级：当ML模型未就绪时使用固定权重
    mlPrediction = predictWithFixedWeights(features);
  }
  
  // ✅ 教学要点：ML预测与基础成本的融合策略
  // 这里使用乘法融合，保持了原有成本的基础结构
  // 同时让ML预测能够动态调整成本倍数
  double finalCost = metrics.originalCost * (1.0 + mlPrediction);
  
  // ✅ 更新特征计算所需的历史数据
  if (metrics.currentRtt) {
    auto rttMs = ndn::time::duration_cast<ndn::time::milliseconds>(*metrics.currentRtt).count();
    auto& history = m_rttHistory[neighbor];
    history.push_back(rttMs);
    if (history.size() > MAX_RTT_HISTORY) {
      history.pop_front();
    }
  }
  
  NLSR_LOG_TRACE("ML prediction for " << neighbor
                << ": features=[" << features[0] << "," << features[1] 
                << "," << features[2] << "," << features[3] << "," << features[4] << "]"
                << ", ml_score=" << mlPrediction 
                << ", final_cost=" << finalCost);
  
  return finalCost;
}

// ============================================================================
// 其余方法实现保持不变，但移除boost相关的直接引用
// ============================================================================

double
MLAdaptiveCalculator::predictWithFixedWeights(const std::vector<double>& features)
{
  double score = 0.0;
  for (size_t i = 0; i < features.size() && i < FIXED_WEIGHTS.size(); ++i) {
    score += FIXED_WEIGHTS[i] * features[i];
  }
  return std::max(0.0, std::min(score, 1.0));
}

MLAdaptiveCalculator::LinkQuality 
MLAdaptiveCalculator::categorizeLinkQuality(const std::vector<double>& features)
{
  // ✅ 教学要点：基于规则的智能分类
  // 这种分类方法结合了机器学习特征和专家知识
  if (features[0] < 0.1 && features[1] < 0.2 && features[2] > 0.8) {
    return LinkQuality::EXCELLENT;
  }
  if (features[0] < 0.3 && features[1] < 0.4 && features[2] > 0.6) {
    return LinkQuality::GOOD;
  }
  if (features[0] < 0.6 && features[2] > 0.4) {
    return LinkQuality::FAIR;
  }
  return LinkQuality::POOR;
}

// ============================================================================
// 特征计算函数实现
// ============================================================================

double
MLAdaptiveCalculator::calculateRttTrend(const ndn::Name& neighbor)
{
  auto it = m_rttHistory.find(neighbor);
  if (it == m_rttHistory.end() || it->second.size() < 10) {
    return 0.0; // 数据不足时返回中性值
  }
  
  const auto& history = it->second;
  size_t size = history.size();
  
  // ✅ 教学要点：滑动窗口趋势分析
  // 比较最近5个样本与之前5个样本，捕获短期趋势变化
  double recentAvg = 0.0, oldAvg = 0.0;
  for (size_t i = size - 5; i < size; ++i) {
    recentAvg += history[i];
  }
  for (size_t i = size - 10; i < size - 5; ++i) {
    oldAvg += history[i];
  }
  
  recentAvg /= 5.0;
  oldAvg /= 5.0;
  
  if (oldAvg > 0) {
    double trend = (recentAvg / oldAvg) - 1.0;
    return std::max(-1.0, std::min(trend, 1.0)); // 限制在[-1,1]区间
  }
  
  return 0.0;
}

double
MLAdaptiveCalculator::calculateRttVariationCoefficient(const ndn::Name& neighbor)
{
  auto it = m_rttHistory.find(neighbor);
  if (it == m_rttHistory.end() || it->second.size() < 3) {
    return 0.0;
  }
  
  const auto& history = it->second;
  
  // ✅ 教学要点：变异系数作为稳定性指标
  // 变异系数 = 标准差/均值，是一个归一化的离散程度度量
  // 在网络分析中，它能很好地反映连接的稳定性
  double sum = 0.0;
  for (double rtt : history) {
    sum += rtt;
  }
  double mean = sum / history.size();
  
  if (mean <= 0) return 1.0; // 异常情况处理
  
  double variance = 0.0;
  for (double rtt : history) {
    variance += (rtt - mean) * (rtt - mean);
  }
  double stddev = std::sqrt(variance / history.size());
  
  double cv = stddev / mean;
  return std::min(cv, 1.0); // 限制最大值，避免极端情况
}

double
MLAdaptiveCalculator::calculateSuccessRate(const ndn::Name& neighbor)
{
  // ✅ 教学要点：基于RTT历史估算成功率
  // 这是一个实用的近似方法，假设RTT过高表示网络拥塞或不稳定
  auto it = m_rttHistory.find(neighbor);
  if (it == m_rttHistory.end() || it->second.empty()) {
    return 0.5; // 默认中等成功率
  }
  
  const auto& history = it->second;
  int successCount = 0;
  for (double rtt : history) {
    if (rtt < 500.0) { // 500ms作为成功阈值
      successCount++;
    }
  }
  
  return static_cast<double>(successCount) / history.size();
}

double
MLAdaptiveCalculator::calculateLoadIndicator(const ndn::Name& neighbor)
{
  auto it = m_rttHistory.find(neighbor);
  if (it == m_rttHistory.end() || it->second.size() < 5) {
    return 0.0;
  }
  
  const auto& history = it->second;
  size_t size = history.size();
  
  // ✅ 教学要点：二阶导数作为负载指示器
  // 通过计算RTT的加速度变化，我们可以检测网络负载的突然变化
  // 这是一个创新的网络分析方法，比单纯的平均值更敏感
  if (size >= 3) {
    double recent = history[size-1];
    double middle = history[size-2]; 
    double old = history[size-3];
    
    double acceleration = (recent - middle) - (middle - old);
    return std::max(-1.0, std::min(acceleration / 100.0, 1.0)); // 归一化处理
  }
  
  return 0.0;
}

// ============================================================================
// 在线学习和反馈机制
// ============================================================================

void
MLAdaptiveCalculator::reportPathPerformance(const ndn::Name& neighbor, double actualPerformance)
{
  // ✅ 教学要点：完整的学习循环
  // 这个方法实现了从预测到反馈到学习的完整循环，这是ML系统的核心
  
  auto features = extractCoreFeatures(neighbor);
  
  // 更新时间模式学习
  m_patternLearner->updatePattern(neighbor, actualPerformance);
  
  // 执行在线模型更新
  updateModelWithFeedback(neighbor, features, actualPerformance);
  
  // 记录性能历史，用于后续分析
  PerformanceRecord record;
  record.predictedScore = m_model->predict(features);
  record.actualPerformance = actualPerformance;
  record.timestamp = ndn::time::steady_clock::now();
  
  auto& history = m_performanceHistory[neighbor];
  history.push_back(record);
  if (history.size() > MAX_PERFORMANCE_HISTORY) {
    history.pop_front();
  }
  
  NLSR_LOG_DEBUG("Performance feedback for " << neighbor 
                << ": predicted=" << record.predictedScore
                << ", actual=" << actualPerformance);
}

void
MLAdaptiveCalculator::updateModelWithFeedback(const ndn::Name& neighbor,
                                             const std::vector<double>& features, 
                                             double actualPerformance)
{
  if (!m_model || features.size() != FEATURE_COUNT) {
    return;
  }
  
  double prediction = m_model->predict(features);
  double error = std::abs(actualPerformance - prediction);
  
  // ✅ 更新统计信息
  if (m_statistics.predictionCount > 0) {
    m_statistics.averagePredictionError = 
      (m_statistics.averagePredictionError * (m_statistics.predictionCount - 1) + error) 
      / m_statistics.predictionCount;
  } else {
    m_statistics.averagePredictionError = error;
  }
  
  // ✅ 教学要点：自适应学习策略
  // 只有在满足特定条件时才触发模型更新，避免过拟合和计算资源浪费
  if (shouldTriggerModelUpdate(error)) {
    adaptLearningRate();
    m_model->updateOnline(features, actualPerformance, m_learningRate);
    
    ++m_statistics.modelUpdateCount;
    m_lastModelUpdate = ndn::time::steady_clock::now();
    m_isModelReady = true;
    
    NLSR_LOG_DEBUG("Model updated for " << neighbor 
                  << ": error=" << error 
                  << ", learning_rate=" << m_learningRate);
  }
}

bool
MLAdaptiveCalculator::shouldTriggerModelUpdate(double predictionError)
{
  // ✅ 教学要点：智能更新触发条件
  // 这些条件平衡了学习速度和计算效率
  
  // 条件1: 预测误差超过阈值时需要纠正
  if (predictionError > m_adaptationThreshold) {
    return true;
  }
  
  // 条件2: 定期更新保持模型活跃
  auto timeSinceUpdate = ndn::time::steady_clock::now() - m_lastModelUpdate;
  if (timeSinceUpdate > MIN_UPDATE_INTERVAL) {
    return true;
  }
  
  return false;
}

void
MLAdaptiveCalculator::adaptLearningRate()
{
  // ✅ 教学要点：自适应学习率调整
  // 根据模型的当前性能动态调整学习率，这是现代ML的重要技巧
  if (m_statistics.averagePredictionError > 0.3) {
    m_learningRate = std::min(0.05, m_learningRate * 1.1); // 误差大时加速学习
  } else if (m_statistics.averagePredictionError < 0.1) {
    m_learningRate = std::max(0.001, m_learningRate * 0.9); // 误差小时稳定学习
  }
}

} // namespace nlsr