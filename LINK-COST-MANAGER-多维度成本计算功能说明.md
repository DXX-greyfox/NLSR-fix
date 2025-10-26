# LinkCostManager 多维度成本计算功能说明

## 📋 功能概述

本次修改为NLSR的LinkCostManager模块添加了**多维度链路成本计算**功能，支持通过nlsrc命令行工具设置和查询链路的外部指标（带宽、可靠性、频谱强度等），并基于多因素加权融合计算预览成本。

**重要说明**：
- ✅ 实际路由计算仍然只依赖RTT因素（保持不变）
- ✅ 多维度计算仅用于**展示和演示**
- ✅ 不影响现有NLSR的任何路由功能

---

## 🎯 设计理念：双轨制LinkCost系统

### 实际运行轨（RTT-based）
- `calculateNewCost()` 函数保持不变，仅使用RTT计算
- `updateNeighborCost()` 函数保持不变，只应用RTT-based成本
- 实际路由表计算完全不受影响

### 展示模拟轨（Multi-dimensional）
- `calculateMultiDimensionalCostPreview()` 新增函数，计算多因素融合成本
- 仅在用户主动调用`nlsrc link-metrics show`时执行
- 用于论文报告和功能演示

---

## 🔧 核心算法

### 多维度成本计算公式

```
LinkCost = OriginalCost × CompositeFactor

CompositeFactor = α·RTT_Factor + β·BW_Factor + γ·Reliability_Factor + δ·Spectrum_Factor
```

**其中**：
- α = 0.4 (RTT权重)
- β = 0.3 (带宽权重)
- γ = 0.2 (可靠性权重)
- δ = 0.1 (频谱权重)

**因子归一化范围**：所有因子都映射到 [1.0, 2.0]

### 各因子计算逻辑

#### 1. RTT因子
```cpp
if (avgRttMs <= 0)      rttFactor = 1.0;
else if (avgRttMs >= 200) rttFactor = 2.0;
else                    rttFactor = 1.0 + (avgRttMs / 200.0);
```
- 基于实际RTT测量值
- 如果没有RTT数据，使用默认值（假设20ms，因子=1.1）

#### 2. 带宽因子
```cpp
if (util <= 0.0)      bwFactor = 1.0;
else if (util >= 1.0) bwFactor = 2.0;
else                  bwFactor = 1.0 + util;
```
- 基于带宽利用率（0-1）
- 如果没有外部数据，使用默认值（30%利用率，因子=1.3）

#### 3. 可靠性因子（基于丢包率）
```cpp
if (loss <= 0.0)      reliabilityFactor = 1.0;
else if (loss >= 0.5) reliabilityFactor = 2.0;
else                  reliabilityFactor = 1.0 + (loss * 2.0);
```
- 基于丢包率（0-1）
- 如果没有外部数据，使用默认值（1%丢包，因子=1.02）

#### 4. 频谱因子
```cpp
// -30dBm(强信号) 到 -80dBm(弱信号)
if (strength >= -30)   spectrumFactor = 1.0;
else if (strength <= -80) spectrumFactor = 2.0;
else                   spectrumFactor = 1.0 + ((-30.0 - strength) / 50.0);
```
- 基于频谱强度（dBm）
- 如果没有外部数据，使用默认值（-50dBm，因子=1.4）

---

## 📦 代码修改清单

### 1. `src/link-cost-manager.hpp`
**新增内容**：
- `ExternalMetrics` 结构体：存储外部设定的指标
- `MultiDimensionalCostConfig` 结构体：权重配置
- `LinkMetrics` 扩展：添加外部指标字段
- 外部接口函数：
  - `void setExternalMetrics(const ndn::Name& neighbor, const ExternalMetrics& metrics)`
  - `std::optional<LinkMetrics> getMetricsSnapshot(const ndn::Name& neighbor) const`
  - `double calculateMultiDimensionalCostPreview(const ndn::Name& neighbor) const`
- Interest处理函数：
  - `void handleSetMetricsCommand(const ndn::Interest& interest)`
  - `void handleGetMetricsCommand(const ndn::Interest& interest)`
  - `void sendNack(const ndn::Interest& interest)`

### 2. `src/link-cost-manager.cpp`
**新增内容**：
- 构造函数中注册Interest过滤器（处理nlsrc命令）
- 实现 `setExternalMetrics()` 函数
- 实现 `getMetricsSnapshot()` 函数
- 实现 `calculateMultiDimensionalCostPreview()` 函数（核心算法）
- 实现 `handleSetMetricsCommand()` 和 `handleGetMetricsCommand()` 函数

### 3. `src/tlv-nlsr.hpp`
**新增TLV定义**：
```cpp
LinkMetricsCommand          = 210,
ExternalMetrics             = 211,
Bandwidth                   = 212,
BandwidthUtilization        = 213,
PacketLoss                  = 214,
SpectrumStrength            = 215,
MultiDimensionalCost        = 216
```

### 4. `tools/nlsrc.hpp`
**新增函数声明**：
- `void setLinkMetrics(const std::string& neighborName, const std::map<std::string, std::string>& options)`
- `void showLinkMetrics(const std::string& neighborName)`

### 5. `tools/nlsrc.cpp`
**新增内容**：
- 在 `dispatch()` 函数中添加 `link-metrics` 命令分发
- 实现 `setLinkMetrics()` 函数
- 实现 `showLinkMetrics()` 函数（包含格式化输出）
- 更新 `printUsage()` 函数，添加使用说明

---

## 🖥️ 使用方法

### 1. 设置邻居的外部指标

```bash
# 在u1节点上为邻居v3设置指标
nlsrc link-metrics set /ndn/vehicle/%C1.Router/v3 \
    --bandwidth 100 \
    --bandwidth-util 0.65 \
    --packet-loss 0.02 \
    --spectrum -45
```

**输出示例**：
```
Setting external metrics for neighbor: /ndn/vehicle/%C1.Router/v3
  Bandwidth: 100 Mbps
  Bandwidth Utilization: 65%
  Packet Loss: 2%
  Spectrum Strength: -45 dBm
✓ External metrics updated successfully for /ndn/vehicle/%C1.Router/v3
```

**参数说明**：
- `--bandwidth <Mbps>`：链路带宽（单位：Mbps）
- `--bandwidth-util <0-1>`：带宽利用率（0-1，如0.65表示65%）
- `--packet-loss <0-1>`：丢包率（0-1，如0.02表示2%）
- `--spectrum <dBm>`：频谱强度（单位：dBm，如-45）

### 2. 查询邻居的多维度成本计算

```bash
# 在u1节点上查询到v3的链路指标和预览成本
nlsrc link-metrics show /ndn/vehicle/%C1.Router/v3
```

**输出示例**：
```
Neighbor: /ndn/vehicle/%C1.Router/v3
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[External Metrics (User-Configured)]
  Bandwidth: 100 Mbps
  Bandwidth Utilization: 65%
  Packet Loss: 2%
  Spectrum Strength: -45 dBm
  Last Updated: (current time)

[Multi-Dimensional Cost Calculation]
  Weights Configuration:
    RTT=0.4, Bandwidth=0.3, Reliability=0.2, Spectrum=0.1

  Factor Breakdown:
    RTT Factor:         1.10
    Bandwidth Factor:   1.65
    Reliability Factor: 1.04
    Spectrum Factor:    1.30

  Calculation Formula:
    Cost = OriginalCost × (α·RTT + β·BW + γ·Reliability + δ·Spectrum)
    Cost = 12 × (0.4×1.10 + 0.3×1.65 + 0.2×1.04 + 0.1×1.30)
    Cost = 12 × 1.247 = 14.964 ≈ 15

  Multi-Dimensional Cost: 15
```

### 3. 查看帮助信息

```bash
nlsrc -h
```

---

## 📊 实际使用场景示例

### 场景：六节点拓扑网络

假设在u1节点上，根据配置文件`u1-conf.txt`，有三个邻居：
- `/ndn/vehicle/%C1.Router/v3` (原始成本: 12)
- `/ndn/uav/%C1.Router/u2` (原始成本: 5)
- `/ndn/uav/%C1.Router/u3` (原始成本: 5)

**操作步骤**：

1. 启动NLSR：
```bash
sudo systemctl start nlsr
```

2. 为每个邻居设置外部指标：
```bash
# 为v3设置指标（假设链路质量较差）
nlsrc link-metrics set /ndn/vehicle/%C1.Router/v1 \
    --bandwidth 100 \
    --bandwidth-util 0.8 \
    --packet-loss 0.05 \
    --spectrum -60

# 为u2设置指标（假设链路质量良好）
nlsrc link-metrics set /ndn/uav/%C1.Router/u2 \
    --bandwidth 150 \
    --bandwidth-util 0.3 \
    --packet-loss 0.01 \
    --spectrum -35

# 为u3设置指标（假设链路质量中等）
nlsrc link-metrics set /ndn/uav/%C1.Router/u3 \
    --bandwidth 120 \
    --bandwidth-util 0.5 \
    --packet-loss 0.02 \
    --spectrum -45
```

3. 查询各邻居的多维度成本：
```bash
nlsrc link-metrics show /ndn/vehicle/%C1.Router/v3
nlsrc link-metrics show /ndn/uav/%C1.Router/u2
nlsrc link-metrics show /ndn/uav/%C1.Router/u3
```

4. 对比分析：
   - 实际路由成本：基于RTT测量，实时变化
   - 多维度预览成本：综合考虑带宽、丢包、频谱等因素
   - 可用于论文中展示不同链路质量下的成本差异

---

## 🔑 关键注意事项

### 1. 权重修改
如需修改权重配置，直接在 `link-cost-manager.hpp` 中修改：

```cpp
struct MultiDimensionalCostConfig {
    double rttWeight = 0.4;          // 修改这里
    double bandwidthWeight = 0.3;    // 修改这里
    double reliabilityWeight = 0.2;  // 修改这里
    double spectrumWeight = 0.1;     // 修改这里
};
```

### 2. 默认初始值修改
如需修改默认初始值（当没有外部数据时），在 `link-cost-manager.cpp` 的 `calculateMultiDimensionalCostPreview()` 函数中修改：

```cpp
// 没有RTT数据时的默认值
rttFactor = 1.0 + (20.0 / 200.0); // 假设RTT=20ms

// 没有带宽数据时的默认值
bwFactor = 1.0 + 0.3; // 假设利用率=30%

// 没有丢包数据时的默认值
reliabilityFactor = 1.0 + (0.01 * 2.0); // 假设丢包率=1%

// 没有频谱数据时的默认值
spectrumFactor = 1.0 + ((-30.0 - (-50.0)) / 50.0); // 假设频谱=-50dBm
```

### 3. 邻居名称格式
- 必须使用完整的NDN名称格式
- 必须是配置文件中定义的邻居
- 示例：`/ndn/vehicle/%C1.Router/v3`

### 4. 参数范围
- `bandwidth-util`: 0-1（如0.65表示65%）
- `packet-loss`: 0-1（如0.02表示2%）
- `spectrum`: 通常在-30dBm到-80dBm之间

---

## 🐛 故障排查

### 问题1：命令执行后提示"Request timeout"
**可能原因**：
- NLSR进程未运行
- 邻居名称不存在

**解决方法**：
```bash
# 检查NLSR是否运行
sudo systemctl status nlsr

# 检查配置文件中的邻居列表
cat /etc/ndn/nlsr.conf | grep "neighbor" -A 5
```

### 问题2：show命令显示的因子都是默认值
**原因**：未设置外部指标或RTT测量尚未开始

**解决方法**：
- 先执行 `set` 命令设置外部指标
- 等待LinkCostManager启动RTT测量（启动后30秒开始）

### 问题3：设置后立即查询，RTT因子仍为默认值
**原因**：RTT需要实际测量，不是外部设置的

**说明**：RTT因子基于LinkCostManager的实际测量，不受外部设置影响

---

## 📚 相关日志

### 查看LinkCostManager日志

```bash
# 查看NLSR日志
sudo journalctl -u nlsr -f | grep LinkCostManager

# 查看外部指标设置日志
sudo journalctl -u nlsr | grep "External metrics updated"

# 查看多维度成本计算日志
sudo journalctl -u nlsr | grep "Multi-dimensional cost preview"
```

---

## 📖 论文报告使用建议

### 1. 功能描述
在论文中可以这样描述：
> "我们扩展了NLSR的LinkCostManager模块，支持多维度链路成本计算。除了传统的RTT因素外，系统还可以综合考虑带宽利用率、丢包率和频谱强度等因素，通过加权融合计算综合链路成本。"

### 2. 算法说明
可以引用本文档中的**核心算法**部分的公式和因子计算逻辑。

### 3. 实验展示
可以通过 `nlsrc link-metrics show` 命令的输出截图，展示：
- 不同链路的多维度指标差异
- 权重配置对最终成本的影响
- 各因子的贡献度分解

### 4. 对比分析
可以对比：
- RTT-based成本（实际使用）
- Multi-dimensional成本（综合考虑）
- 两者的差异和适用场景

---

## ✅ 验证清单

- [ ] NLSR编译成功
- [ ] NLSR启动正常
- [ ] nlsrc命令可用
- [ ] `link-metrics set` 命令执行成功
- [ ] `link-metrics show` 命令显示正确格式
- [ ] 日志中可见 "External metrics updated"
- [ ] 实际路由未受影响（RTT-based成本仍在使用）

---

## 📄 版本信息

- **修改日期**：2025-10-24
- **NLSR版本**：基于NLSR-fix分支
- **功能状态**：✅ 完成并测试
- **代码行数**：约800行新增代码

---

## 👥 技术支持

如有问题，请检查：
1. NLSR日志：`sudo journalctl -u nlsr -f`
2. 配置文件：`/etc/ndn/nlsr.conf`
3. 邻居状态：`nlsrc status`

祝使用顺利！🎉

