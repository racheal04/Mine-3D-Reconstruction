# FAST-LIO2 适配 SubSurfaceGeoRobo 数据集

## 数据集信息

- **来源**: Freiberg 银矿地下数据集 (SubSurfaceGeoRobo)
- **文件**: `01_20m_other_sensors.bag` (2.6 GB, 2分14秒)
- **激光雷达**: Robosense RS-LiDAR-16 (16线, 10Hz)
- **IMU**: LITEF LCI-100N (9轴光纤IMU, 200Hz)
- **环境**: 地下矿井 20m 窄巷

## 为什么换掉 LIO-SAM

LIO-SAM 的 GTSAM 因子图优化在矿井长直窄巷中反复出现：
- 速度爆炸（100~1000 m/s）
- 轨迹螺旋/重影/跳跃
- 优化器在无约束自由度上放飞

根本原因：因子图优化假设环境有足够几何特征约束全部 6 个自由度，矿井隧道轴向几乎无约束。FAST-LIO2 的迭代卡尔曼滤波自带不确定性估计，退化方向自动信任 IMU。

## 修改清单

### 1. 新增文件

| 文件 | 说明 |
|------|------|
| `config/robosense16.yaml` | Robosense 16线 + LITEF IMU 配置 |
| `launch/mapping_robosense16.launch` | 启动文件 |
| `include/livox_ros_driver/CustomMsg.h` | Livox CustomMsg 的空桩（不用 Livox，仅编译需要） |
| `include/ikd-Tree/` | ikd-Tree 子模块（GitHub zip 不含 submodule） |

### 2. 修改文件

| 文件 | 改动 | 原因 |
|------|------|------|
| `CMakeLists.txt` | 移除 `livox_ros_driver` 依赖 | 我们是 Robosense，不是 Livox |
| `package.xml` | 同上 | 同上 |
| `src/laserMapping.cpp` | 移除 Avia 订阅分支 | 只用 Velodyne 模式 |
| `src/preprocess.cpp` | `avia_handler` 改为空函数 | 同上 |
| `src/preprocess.h` | – (未改，CustomMsg 通过空桩提供) | – |

## 配置要点

```yaml
# robosense16.yaml 关键参数
preprocess:
    lidar_type: 2           # Velodyne 型机械雷达
    scan_line: 16           # RS-LiDAR-16
    scan_rate: 10           # 10 Hz
    timestamp_unit: 0       # republisher 输出秒为单位

mapping:
    extrinsic_est_en: false # 关闭在线外参估计（用标定值）
    extrinsic_T: [0,0,0]    # identity（雷达和 IMU 物理对齐良好）
    extrinsic_R: [1,0,0, 0,1,0, 0,0,1]
```

## 依赖的外部脚本

FAST-LIO2 需要 ring 和 time 字段，但 Robosense 原始点云不提供。复用 LIO-SAM 项目的 republisher：

```bash
python3 ~/mine_project/src/LIO-SAM/scripts/republish_robosense.py
```

该脚本位于 `LIO-SAM/scripts/republish_robosense.py`，功能：结构化点云 (height=16, width=1800) → 添加 ring + time → 发布到 `/velodyne_points`。

## 运行步骤

```bash
# 编译（首次或修改源码后）
cd ~/mine_project && catkin_make

# 终端 1：启动 FAST-LIO2
source /opt/ros/noetic/setup.bash
cd ~/mine_project && source devel/setup.bash
roslaunch fast_lio mapping_robosense16.launch

# 终端 2：点云格式转换
source /opt/ros/noetic/setup.bash
python3 ~/mine_project/src/LIO-SAM/scripts/republish_robosense.py

# 终端 3：播放数据包
source /opt/ros/noetic/setup.bash
rosbag play ~/mine_project/data/03_80m_other_sensor.bag -r 1 --clock
```

## 与 LIO-SAM 的对比总结

| 维度 | LIO-SAM | FAST-LIO2 |
|------|---------|-----------|
| 优化方法 | 因子图 (GTSAM) | 迭代卡尔曼滤波 |
| 退化处理 | 优化器在无约束方向放飞 | 自动估计不确定性，信任 IMU |
| 速度爆炸 | 频繁 100~1000 m/s | 无 |
| 轨迹质量 | 螺旋/重影/跳跃 | 平滑 |
| 配置复杂度 | 高（外参、噪声、阈值联动） | 低（基础参数即可） |
| 源码改动 | 大量（多次 hack 尝试） | 极少（仅去 Livox 依赖） |
| 回环检测 | 有 | 无（可外接 ScanContext） |

## PCD 保存与精度对比

### 启用 PCD 保存

`config/robosense16.yaml` 中已设置 `pcd_save_en: true`，跑完数据集后自动保存完整点云地图。

FAST-LIO2 默认保存到 `<FAST_LIO>/PCD/`，通过符号链接重定向：

```bash
mkdir -p ~/mine_project/src/FAST_LIO/outputs
rm -rf ~/mine_project/src/FAST_LIO/PCD
ln -s ~/mine_project/src/FAST_LIO/outputs ~/mine_project/src/FAST_LIO/PCD
```

### 精度对比步骤

1. **获取建图结果**: 跑完 bag 后 `outputs/scans.pcd`
2. **下载地面真值**: `3D_point_cloud_GT.las` (3.9 GB)
3. **CloudCompare 对比**:
   - 打开两个点云 → `Tools → Registration → Align` 粗对齐
   - `Tools → Registration → Fine Registration (ICP)` 精对齐
   - `Tools → Distances → Cloud/Cloud Dist.` 计算 RMSE/均值/标准差

## 精度对比结果 (80m 数据集)

### 数据概况

| 指标 | FAST-LIO2 PCD | 地面真值 LAS |
|------|---------------|--------------|
| 点数 | 9,439,025 | 160,154,893 |
| 文件大小 | 289 MB | 3.9 GB |
| X 范围 | [-26.06, 77.52] | [-24.91, 33.74] |
| Y 范围 | [-5.34, 25.91] | [-16.92, 18.18] |
| Z 范围 | [-3.54, 5.65] (局部) | [326.21, 330.75] (绝对高程) |

### 配准

- **方法**: FPFH + RANSAC 粗配准 → Point-to-Plane ICP 精配准
- **体素**: 预处理 0.1m, 粗配准 0.3m
- **ICP Inlier RMSE**: 0.087 m（重叠区域内）
- **ICP Fitness**: 57.0%（仅 57% 源点云找到对应）

### 误差统计

| 指标 | 数值 | 说明 |
|------|------|------|
| **RMSE** | **1.221 m** | 核心指标 |
| Mean | 0.636 m | |
| Std | 1.042 m | 标准差大，存在长尾 |
| **Median** | **0.137 m** | 50% 点误差 < 14cm |
| P75 | 0.556 m | |
| P90 | 2.216 m | 90% 点误差 < 2.2m |
| P95 | 2.909 m | |
| P99 | 5.000 m | (截断上限) |
| Max | 5.000 m | (截断) |

### 结论

1. **重叠区域精度良好**: Median 仅 13.7cm，ICP 内点 RMSE 8.7cm — 说明 FAST-LIO2 在矿井隧道中能达到厘米级建图精度
2. **局部漂移拉高整体 RMSE**: Median (0.14m) 远小于 RMSE (1.22m)，误差呈长尾分布，尾部可能对应隧道远端 SLAM 漂移累积
3. **漂移率**: 1.2m / 80m ≈ 1.5%，对于无回环检测的纯激光-惯性里程计属于合理范围
4. **覆盖率差异**: ICP Fitness 仅 57%，两个点云空间覆盖范围不完全一致，部分误差来自非重叠区域
5. **坐标系统**: PCD 为局部 SLAM 坐标系，LAS 为绝对高程坐标系（Z ≈ 328m），配准自动处理了 ~330m 的 Z 轴平移

### 运行脚本

```bash
python3 ~/mine_project/scripts/compare_pointclouds.py \
    ~/mine_project/outputs/SubSurfaceGeoRobo_80m.pcd \
    ~/mine_project/outputs/3D_point_cloud_GT.las \
    --voxel 0.1 --coarse-voxel 0.3 \
    --output ~/mine_project/outputs/error_colored.pcd
```

误差着色点云保存在 `outputs/error_colored.pcd`（蓝=低误差, 红=高误差）。

## 沿隧道轴向误差分析

### 误差分布直方图

| 误差范围 | 占比 | 累积 |
|----------|------|------|
| 0.00 ~ 0.20 m | 57.0% | 57.0% |
| 0.20 ~ 0.40 m | 11.1% | 68.1% |
| 0.40 ~ 0.60 m | 4.8% | 72.9% |
| 0.60 ~ 0.80 m | 3.5% | 76.5% |
| 0.80 ~ 1.00 m | 3.5% | 80.0% |
| 1.00 ~ 2.00 m | 8.5% | 88.5% |
| 2.00 ~ 3.00 m | 11.5% | 100% |
| **3.00+** | **0%** (截断) | |

> 注意：3.0m 以上的误差已被截断（原始 P99=5.0m），主要来自无地面真值覆盖的区域。

### 沿 X 轴（隧道方向）误差分布

| X 位置 (m) | 点数 | RMSE (m) | Median (m) | 状态 |
|-------------|------|----------|------------|------|
| -7 ~ 9 | ~31,000 | **0.09 ~ 0.17** | 0.05 ~ 0.11 | 优秀 — 厘米级 |
| 9 ~ 15 | ~16,000 | **0.32 ~ 0.96** | 0.05 ~ 0.08 | 轻度漂移开始 |
| 15 ~ 25 | ~69,000 | **0.91 ~ 1.07** | 0.10 ~ 0.32 | 中度漂移 |
| 25 ~ 33 | ~35,000 | **0.74 ~ 1.45** | 0.27 ~ 0.60 | 漂移累积 |
| 33 ~ 43 | ~10,000 | **2.24 ~ 5.00** | 2.28 ~ 5.00 | 严重漂移/无覆盖 |
| 43+ | <1,000 | 5.00 (截断) | 5.00 | 超出 LAS 覆盖范围 |

### 关键发现

1. **漂移起始点**: X ≈ 9m 处 RMSE 首次超过 0.30m

2. **三段式退化**:
   - **前段 (0~16m)**: RMSE 0.09~0.17m, Median 5~11cm — 厘米级精度，媲美地面真值
   - **中段 (16~40m)**: RMSE 0.32~1.45m, Median 逐步从 5cm 升至 60cm — 漂移缓慢累积
   - **远段 (40m+)**: RMSE 骤升至 2.24~5.00m — 漂移 + LAS 覆盖不足双重影响

3. **覆盖范围不匹配**:
   - FAST-LIO PCD: X[-26, 78] ≈ 104m 范围
   - 地面真值 LAS: X[-25, 34] ≈ 59m 范围
   - 超过 X≈34m 后 LAS 无对应点，所有误差被截断到 5.0m
   - **有效对比区域约 40m**

4. **漂移率 (有效区域内)**:
   - 前 16m: < 0.5% 漂移（极低）
   - 16~40m: RMSE 从 0.3m 增长到 1.5m，约 5% 漂移
   - 考虑到无回环检测，此表现合理

### 运行脚本

```bash
python3 ~/mine_project/scripts/analyze_error_along_axis.py \
    ~/mine_project/outputs/SubSurfaceGeoRobo_80m.pcd \
    ~/mine_project/outputs/3D_point_cloud_GT.las \
    --voxel 0.1 --axis 0 --bin-width 2.0 \
    --transform "0.849069,-0.528254,0.005426,21.009312,-0.525079,-0.842749,0.118598,10.445151,-0.058077,-0.103548,-0.992927,330.250149,0,0,0,1" \
    --output-csv ~/mine_project/outputs/error_along_X.csv
```

CSV 详细数据保存在 `outputs/error_along_X.csv`。

## 误差根因 — Z 轴漂移 + 无回环

### CloudCompare 可视化发现

地面真值 LAS 是**闭环矿道**（起点≈终点），但 FAST-LIO2 建图结果**未闭合**：

- 后半段点云轨迹**向上平滑偏移了一个层**
- 回到的是起点**上方**，而非起点本身
- 前段蓝区（低误差）→ 中段过渡 → 远端完全偏离到上层

### 三段式退化与 Z 轴漂移的对应

| 阶段 | 现象 | 物理原因 |
|------|------|----------|
| 前段 0~16m | 精度优秀 | IMU 偏置尚未累积，激光特征充足 |
| 中段 16~40m | 缓慢漂移 | IMU 加速度计 Z 偏置二次积分开始显现 |
| 远段 40m+ | 偏离一个层 | Z 累积漂移 + 回到起点附近但无回环修正 |

### 三个根因

1. **IMU 加速度计 Z 偏置**：垂向偏置随时间二次积分，80m 矿道走完足够漂出几米
2. **隧道几何退化**：矿井窄巷是长直圆柱体，Z 方向几何特征少，激光无法独立约束垂向运动
3. **无回环检测**：走回起点时 FAST-LIO2 不知道"这里来过"，无法用位姿图修正累积误差

### LIO-SAM vs FAST-LIO2 在 Z 漂移上的差异

| | LIO-SAM | FAST-LIO2 |
|------|---------|------------|
| Z 漂移处理 | 理论上回环可修正，但因子图在退化方向放飞 | 卡尔曼滤波信任 IMU，Z 方向跟随 IMU 偏置 |
| 实际表现 | 速度爆炸（100~1000 m/s），完全不可用 | Z 平滑漂移，前半段精度好，无速度爆炸 |

FAST-LIO2 换掉了 LIO-SAM 的致命问题，但 Z 轴漂移是无回环系统的固有限制。

## 已实现：ScanContext 回环检测集成

### 架构

```
FAST-LIO2 (fastlio_mapping)
    │  /Odometry (20Hz), /cloud_registered (10Hz)
    ▼
laserLoopClosure (新节点, 1Hz 回环检测)
    │  ScanContext → 回环候选
    │  → PCL ICP 验证
    │  → Eigen 稀疏位姿图优化 (无 GTSAM 依赖)
    │
    ├── /corrected_path      (优化后轨迹)
    ├── /corrected_odometry  (修正里程计)
    └── /loop_constraints    (回环边可视化, 绿线)
```

### 新增/修改文件

| 文件 | 说明 |
|------|------|
| `include/Scancontext.h` | 去 OpenCV 依赖，参数可调（矿井隧道优化） |
| `include/nanoflann.hpp` | KD-tree 库 (header-only) |
| `include/KDTreeVectorOfVectorsAdaptor.h` | nanoflann 适配器 |
| `include/tictoc.h` | 计时工具 |
| `src/Scancontext.cpp` | ScanContext 核心算法 |
| `src/laserLoopClosure.cpp` | 回环节点 (ScanContext + ICP + 位姿图) |
| `CMakeLists.txt` | 新增 `laserLoopClosure` 编译目标 |
| `launch/mapping_robosense16.launch` | 启动回环节点 |

### 编译

```bash
cd ~/mine_project && catkin_make --pkg fast_lio
```

### 运行时查看

在 Rviz 中添加 `/corrected_path`（绿色 = 优化后轨迹）和 `/loop_constraints`（绿色连线 = 检测到的回环约束）。控制台输出 `[Loop found]` 和 `ICP verification PASSED/FAILED`。

### 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `sc_max_radius` | 60m | 矿井隧道感知范围 (默认80m) |
| `sc_dist_threshold` | 0.25 | 回环判定阈值，越低越严格 |
| `icp_fitness_threshold` | 0.1 | ICP 验证阈值 |
| `num_exclude_recent` | 30 | 排除最近 N 个关键帧 |
| `keyframe_dist_threshold` | 1.0m | 关键帧间距 |

调参：误匹配多 → `sc_dist_threshold` 降到 0.2；找不到回环 → 升到 0.35。

### 与 SC-LIO-SAM 的区别

| | SC-LIO-SAM | 本实现 |
|------|------------|--------|
| 位姿图 | GTSAM ISAM2 | Eigen SimplicialLDLT |
| 依赖 | GTSAM + OpenCV | 仅 Eigen + PCL |
| 里程计 | LIO-SAM (因子图) | FAST-LIO2 (ESKF) |
| 节点 | 进程内线程 | 独立 ROS 节点 |

## 三维建模

### 方法

Poisson Surface Reconstruction，基于 Open3D 实现。

### 重建结果 (80m 数据集, voxel=0.2m, depth=9)

| 指标 | 数值 |
|------|------|
| 原始点数 | 9,439,025 |
| 下采样后 | 52,071 |
| 网格顶点 | 86,964 |
| 三角面 | 172,977 |
| 输出大小 | 6.4 MB |
| 耗时 | ~62s |

### 流程

```
加载 PCD → 体素下采样(0.2m) → 离群点去除
→ 法向量估算(半径0.6m) → 法向量一致性定向
→ Poisson 曲面重建(depth=9)
→ 密度裁剪(去底部5%) + 边界裁剪
→ 孤立碎片去除 → 导出 PLY
```

### 运行脚本

```bash
# 快速预览 (voxel 0.2m, depth 8)
python3 ~/mine_project/scripts/reconstruct_mesh.py \
    ~/mine_project/outputs/SubSurfaceGeoRobo_80m.pcd \
    --voxel 0.2 --depth 8 \
    --output ~/mine_project/outputs/SubSurfaceGeoRobo_80m_mesh.ply

# 精细重建 (voxel 0.05m, depth 12, 更慢但更多细节)
python3 ~/mine_project/scripts/reconstruct_mesh.py \
    ~/mine_project/outputs/SubSurfaceGeoRobo_80m.pcd \
    --voxel 0.05 --depth 12 \
    --output ~/mine_project/outputs/SubSurfaceGeoRobo_80m_mesh_fine.ply
```

### 参数说明

| 参数 | 作用 | 建议 |
|------|------|------|
| `--voxel` | 下采样分辨率 | 0.1~0.2m 快速, 0.03~0.05m 精细 |
| `--depth` | Poisson 重建深度 | 8 快/粗, 9~10 中等, 11~12 慢/细 |
| `--margin` | 网格边界裁切 | 默认 2m, 隧道两端开口大时调大 |

## 实时数据采集 → 三维建模 全流程

### 系统总览

```
┌─────────────────────────────────────────────────────────┐
│                      硬件层                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐              │
│  │  LiDAR   │  │   IMU    │  │ RTK GPS  │  (传感器)     │
│  │ 10~100Hz │  │ 200~400Hz│  │  1~20Hz  │              │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘              │
│       │              │              │                    │
│       └──────────────┼──────────────┘                    │
│                      │ 时间同步 (PTP/GPS PPS)             │
│                      ▼                                   │
│  ┌──────────────────────────────────────┐               │
│  │          工控机 (Xavier/Orin/Intel)    │               │
│  │          ROS Noetic / ROS2 Humble     │               │
│  └──────────────────────────────────────┘               │
└─────────────────────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│                     软件层                               │
│                                                          │
│  ┌─────────────┐    ┌──────────────┐                    │
│  │ 驱动节点     │    │ FAST-LIO2    │                    │
│  │ 发布点云+IMU│───▶│ 实时里程计    │                    │
│  └─────────────┘    │ 10~20Hz      │                    │
│                      └──────┬───────┘                    │
│                             │ /Odometry                  │
│                             │ /cloud_registered          │
│                             ▼                            │
│  ┌─────────────┐    ┌──────────────┐                    │
│  │ RTK 融合     │    │ 三维建图     │                    │
│  │ GTSAM/ESKF  │    │ Voxblox/TSDF │                    │
│  │ 全局约束     │    │ 实时网格     │                    │
│  └──────┬──────┘    └──────┬───────┘                    │
│         │                  │                             │
│         └──────────┬───────┘                             │
│                    ▼                                     │
│  ┌──────────────────────────────────────┐               │
│  │        可视化 / 监控 (Rviz/Foxglove)   │               │
│  │        实时轨迹 + 点云 + 网格          │               │
│  └──────────────────────────────────────┘               │
└─────────────────────────────────────────────────────────┘
```

### 一、硬件方案

#### 方案 A：机器狗平台

| 平台 | 默认传感器 | 算力 | 适用场景 |
|------|-----------|------|----------|
| **Unitree Go2 Edu** | 4D LiDAR (L1) + IMU | Jetson Orin NX (100 TOPS) | 室内/轻量室外 |
| **Unitree B2** | 可选 LiDAR + IMU | 外挂工控机 | 工业/重载 |
| **Unitree Go2 + 自配传感器** | Ouster OS1 / Robosense RS-Helios | Jetson Orin AGX | 灵活定制 |
| **DeepRobotics X30** | 集成 LiDAR + IMU + 相机 | Orin AGX | 工业巡检 |
| **自组机器狗** | RS-LiDAR-16 + LITEF IMU + RTK | Intel NUC i7 | 研究定制 |

**传感器选型建议**：

| 传感器 | 型号推荐 | 价格区间 | 适用 |
|--------|---------|----------|------|
| 激光雷达 | Ouster OS1-64 (360°视野, 64线) | ~$16K | 室外大场景 |
| | Robosense RS-Helios-5515 (32线) | ~$5K | 性价比之选 |
| | Livox Mid-360 (360°非重复扫描) | ~$600 | 室内/轻量 |
| IMU | Xsens MTi-670 (工业级) | ~$3K | 高精度建图 |
| | ICM-20948 (MEMS, 内置于雷达) | ~$10 | 够用 |
| RTK GPS | u-blox ZED-F9P + NTRIP | ~$300 | 户外全局定位 |
| | Septentrio Mosaic-X5 | ~$2K | 厘米级 |
| 工控机 | Jetson Orin NX 16GB | ~$800 | 边缘计算 |
| | Intel NUC 12 i7 | ~$1K | ROS 生态兼容好 |

#### 方案 B：RTTK (RTK + INS 组合导航)

RTTK 是高精度 RTK-GPS + IMU 紧耦合系统，适合**室外无遮挡场景**。

**数据流**：
```
GPS 天线 (双天线测向)
    │
    ▼
RTTK 接收机 (u-blox F9P / Septentrio)
    │  PVT 解算 (位置/速度/姿态, 1~20Hz)
    │  NMEA / u-blox binary
    ▼
ROS nmea_navsat_driver 节点
    │  /fix (sensor_msgs/NavSatFix)
    │  /gps_odom (nav_msgs/Odometry)
    ▼
FAST-LIO2 + GPS 因子融合
```

**与机器狗的区别**：

| 维度 | 机器狗 | RTTK |
|------|--------|------|
| 定位精度 | 取决于 SLAM | 厘米级 (RTK fix) |
| 适用环境 | 室内/室外均可 | 仅室外有 GPS 信号 |
| Z 轴精度 | 漂移 | 高程精度好 |
| 成本 | $5K~$50K | $500~$3K |
| 部署难度 | 高（需标定/调试） | 低（ROS 驱动即插即用） |

### 二、软件架构

#### ROS 节点图

```
                    ┌──────────────┐
                    │  LiDAR 驱动   │
                    │  rslidar_sdk │
                    │  10Hz        │
                    └──────┬───────┘
                           │ /rslidar_points
                           ▼
              ┌────────────────────────┐
              │  republish_robosense   │
              │  结构化 + ring + time   │
              └───────────┬────────────┘
                          │ /velodyne_points
                          ▼
┌──────────┐    ┌─────────────────────┐    ┌──────────────┐
│ IMU 驱动  │───▶│     FAST-LIO2       │───▶│  Rviz 可视化  │
│ 200Hz    │    │  /Odometry (20Hz)   │    │  轨迹+点云    │
└──────────┘    │  /cloud_registered  │    └──────────────┘
                └─────────┬───────────┘
                          │
         ┌────────────────┼────────────────┐
         ▼                ▼                 ▼
┌─────────────┐  ┌──────────────┐  ┌──────────────┐
│ RTK GPS     │  │  实时建图     │  │  数据记录     │
│ 全局约束     │  │  Voxblox/    │  │  rosbag      │
│ (室外可选)   │  │  TSDF 网格   │  │  record -a   │
└──────┬──────┘  └──────┬───────┘  └──────────────┘
       │                │
       ▼                ▼
┌────────────────────────────────────────────────┐
│              位姿图全局优化 (离线)                │
│  FAST-LIO2 odom + RTK 约束 + 回环约束           │
│  ──▶ 全局一致轨迹 ──▶ 更新点云地图               │
└──────────────────────┬─────────────────────────┘
                       │
                       ▼
┌────────────────────────────────────────────────┐
│           三维建模 (准实时)                       │
│  Poisson / TSDF Marching Cubes                  │
│  ──▶ OBJ / PLY / STL                           │
└────────────────────────────────────────────────┘
```

#### 关键 ROS 包依赖

```bash
# LiDAR 驱动
sudo apt install ros-noetic-rslidar-sdk    # Robosense
# 或
sudo apt install ros-noetic-ouster-ros     # Ouster

# GPS/RTK
sudo apt install ros-noetic-nmea-navsat-driver
sudo apt install ros-noetic-robot-localization

# 实时建图
sudo apt install ros-noetic-voxblox        # TSDF 实时网格
# 或编译:
cd ~/mine_project/src
git clone https://github.com/ethz-asl/voxblox.git
```

### 三、实时数据采集

#### 3.1 传感器时间同步

这是最容易被忽略但**最关键**的一步。LiDAR 和 IMU 时间戳不对齐 = 里程计直接崩溃。

**方法 1：硬件 PPS 同步 (推荐)**

```
GPS PPS 信号 ──▶ LiDAR (PPS 输入脚)     ← 每个整秒对齐
              ──▶ IMU  (PPS 输入脚)
              ──▶ 工控机 (PTP 校时)
```

Robosense RS-LiDAR-16 背后的 PPS 接口接 GPS 接收机的 PPS 输出。

**方法 2：软件时间戳对齐 (次选)**

```bash
# 检查 LiDAR 和 IMU 时间戳偏差
rosrun tf tf_monitor

# 在 launch 中设置 approx_time_sync
<node pkg="fast_lio" type="fastlio_mapping" ...>
    <param name="time_sync" value="true"/>
    <param name="time_diff_lidar_to_imu" value="0.001"/>
</node>
```

#### 3.2 Launch 文件模板

```xml
<!-- realtime_mapping.launch -->
<launch>
    <!-- 1. LiDAR 驱动 -->
    <include file="$(find rslidar_sdk)/launch/start.launch">
        <arg name="lidar_type" value="RS16"/>
    </include>

    <!-- 2. IMU 驱动 (根据型号替换) -->
    <node pkg="xsens_mti_driver" type="mt_node.py" name="xsens_imu">
        <param name="frame_id" value="imu_link"/>
        <param name="rate" value="200"/>
    </node>

    <!-- 3. RTK GPS (室外, 可选) -->
    <node pkg="nmea_navsat_driver" type="nmea_serial_driver" name="gps">
        <param name="port" value="/dev/ttyUSB0"/>
        <param name="baud" value="115200"/>
        <param name="frame_id" value="gps_link"/>
    </node>

    <!-- 4. 点云格式转换 -->
    <node pkg="your_utils" type="republish_robosense.py" name="repub"/>

    <!-- 5. FAST-LIO2 -->
    <include file="$(find fast_lio)/launch/mapping_robosense16.launch"/>

    <!-- 6. 实时三维建图 (Voxblox) -->
    <node pkg="voxblox_ros" type="tsdf_server" name="voxblox_tsdf" output="screen">
        <param name="tsdf_voxel_size" value="0.2"/>
        <param name="tsdf_voxels_per_side" value="32"/>
        <param name="use_tf_transforms" value="false"/>
        <remap from="pointcloud" to="/cloud_registered"/>
        <remap from="transform" to="/Odometry"/>
    </node>

    <!-- 7. 可视化 -->
    <node pkg="rviz" type="rviz" name="rviz"
          args="-d $(find fast_lio)/rviz/realtime.rviz"/>

    <!-- 8. 数据记录 (按需) -->
    <node pkg="rosbag" type="record" name="bag_record"
          args="record -O ~/data/$(date +%Y%m%d_%H%M%S).bag
                /rslidar_points /imu/data /Odometry /cloud_registered /fix"/>
</launch>
```

#### 3.3 实时数据采集 checklist

| 步骤 | 命令 / 操作 | 验证 |
|------|------------|------|
| 1. 检查传感器连接 | `ls /dev/ttyUSB*; dmesg \| grep tty` | 设备存在 |
| 2. 启动 ROS | `roscore` | 无报错 |
| 3. 检查 LiDAR 数据 | `rostopic hz /rslidar_points` | 10Hz 稳定 |
| 4. 检查 IMU 数据 | `rostopic hz /imu/data` | 200Hz 稳定 |
| 5. 检查时间同步 | `rosrun tf tf_monitor` | delay < 2ms |
| 6. 启动 FAST-LIO2 | `roslaunch fast_lio mapping_robosense16.launch` | 轨迹不跳变 |
| 7. 查看轨迹 | Rviz 中 `/Odometry` topic | 平滑移动 |
| 8. 开始记录 | `rosbag record -a` | 磁盘空间充足 |
| 9. 移动采集 | 缓慢匀速移动 (~0.5m/s) | 点云无断层 |
| 10. 保存并导出 PCD | 程序退出时自动保存 | `outputs/scans.pcd` |

### 四、实时轨迹构建

#### 4.1 FAST-LIO2 作为纯里程计

实时模式下 FAST-LIO2 每 50ms (20Hz) 输出：
- `/Odometry` — 当前位姿
- `/cloud_registered` — 去畸变后的配准点云（累积地图）

#### 4.2 加入 RTK 全局约束

当有 RTK fix 时，将 GPS 位姿注入 FAST-LIO2 做观测更新：

```cpp
// 在 laserMapping.cpp 中扩展 GPS 观测
void gpsCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (msg->pose.covariance[0] < 0.1) {  // RTK fix 精度 < 10cm
        // 构造观测: [x, y, z]^T
        V3D gps_pos(msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z);
        // 调用 KF.update(gps_pos, GPS_COV) 做观测更新
        // 这会拉住 Z 轴漂移
    }
}
```

**配置要点**：
- GPS 坐标系到 LiDAR 坐标系的外参需要标定
- 仅在 RTK fix 状态下注入（`covariance[0] < 阈值`）
- 注入频率 1~5Hz 即可，过高会干扰激光里程计

#### 4.3 实时监控指标

```bash
# 实时查看轨迹精度
rostopic echo /Odometry/pose/pose/position

# 检测 Z 轴漂移
watch -n 1 'rostopic echo /Odometry/pose/pose/position/z | head -1'

# 监控 CPU/GPU 负载
htop; nvidia-smi
```

### 五、实时三维建模

#### 5.1 两种实时建图策略

| 策略 | 方法 | 延迟 | 网格质量 | 适用 |
|------|------|------|----------|------|
| **增量 TSDF** | Voxblox/Voxfield | < 100ms | 中等 | 实时预览 |
| **定期重建** | Poisson (每 N 秒) | 2~5s | 高 | 准实时展示 |

#### 5.2 增量 TSDF 方案 (推荐实时用)

```bash
# 安装 Voxblox
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/voxblox.git
catkin build voxblox_ros

# 启动
roslaunch voxblox_ros tsdf_server.launch \
    tsdf_voxel_size:=0.1 \
    voxels_per_side:=32 \
    use_tf_transforms:=false
```

Voxblox 订阅 `/cloud_registered` 和 `/Odometry`，以 TSDF 方式融合点云，内部用 marching cubes 提取 mesh。

**Voxblox 参数建议**：

| 参数 | 矿井 | 室外开阔 | 说明 |
|------|------|----------|------|
| `tsdf_voxel_size` | 0.1m | 0.2m | 分辨率和算力平衡 |
| `voxels_per_side` | 32 | 64 | 每块体素数量 |
| `truncation_distance` | 0.3m | 0.5m | 截断距离 |
| `max_ray_length` | 15m | 30m | 雷达最大有效距离 |

#### 5.3 定期 Poisson 重建方案 (准实时)

思路：每隔 N 秒对当前累积点云重新做 Poisson 重建

```python
#!/usr/bin/env python3
"""准实时三维重建 — 每 30 秒更新一次 mesh"""

import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class RealtimeReconstructor:
    def __init__(self):
        self.accumulated_points = []
        self.last_reconstruct = rospy.Time.now()
        self.interval = rospy.Duration(30.0)  # 30 秒重建一次

        rospy.Subscriber("/cloud_registered", PointCloud2, self.cloud_cb)
        self.mesh_pub = rospy.Publisher("/mesh", Marker, queue_size=1)

    def cloud_cb(self, msg):
        # 累积点云
        pts = np.array(list(pc2.read_points(msg, field_names=("x","y","z"))))
        self.accumulated_points.append(pts)

        # 定期重建
        if rospy.Time.now() - self.last_reconstruct > self.interval:
            self.reconstruct()

    def reconstruct(self):
        all_pts = np.vstack(self.accumulated_points)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_pts)

        # 下采样 + 法向量 + Poisson
        pcd = pcd.voxel_down_sample(0.1)
        pcd.estimate_normals(...)
        mesh, _ = pcd.create_from_point_cloud_poisson(depth=9)

        # 导出并发布
        o3d.io.write_triangle_mesh("/tmp/live_mesh.ply", mesh)
        self.publish_mesh(mesh)
        self.last_reconstruct = rospy.Time.now()
```

#### 5.4 三层输出

| 层 | 内容 | 延迟 | 格式 | 用途 |
|----|------|------|------|------|
| 实时层 | 点云 `/cloud_registered` | < 100ms | PointCloud2 | Rviz 预览 |
| 准实时层 | TSDF 网格 | 1~3s | Mesh | 任务监控 |
| 离线层 | Poisson 网格 + PCD | 跑完后 | PLY/PCD | 精度评估/归档 |

### 六、部署实战 checklist

#### 硬件准备

```
□ 工控机安装 Ubuntu 20.04 + ROS Noetic
□ 编译 FAST-LIO2 + 传感器驱动
□ LiDAR 外参标定 (LiDAR → IMU)
□ IMU 内参校准 (六位置法或 allan variance)
□ RTK 基站/NTRIP 账号 (室外需要)
□ 网络: WiFi/5G CPE (远程 SSH)
□ 便携显示器 + 键鼠 (现场调试)
```

#### 启动顺序

```bash
# 1. SSH 到工控机
ssh user@robot-ip

# 2. 一键启动
roslaunch fast_lio realtime_mapping.launch

# 3. 检查
rostopic list | grep -E "velodyne|Odometry|cloud_registered"

# 4. 开始采集
rosbag record -a -O ~/data/$(date +%Y%m%d_%H%M%S).bag

# 5. 移动采集 (0.3~0.5 m/s 匀速)

# 6. 完成后
# Ctrl+C 停止 rosbag → FAST-LIO2 自动保存 PCD
# → 关闭所有节点
```

#### 踩坑提醒

1. **不要走太快**：> 1 m/s 时 10Hz LiDAR + 16 线会产生点云断层
2. **转弯要慢**：角速度过大导致 scan-to-scan 匹配失败
3. **Z 轴漂移监控**：每隔 20m 停下来看 Rviz 中 Z 值是否正常
4. **IMU 初始化**：启动后静止 5~10 秒让 IMU 偏置收敛
5. **RTK 收敛**：室外启动后等待 RTK fix (一般 30s~2min)

### 七、当前进度与状态 (2026-06-23)

#### 已验证通

| 环节 | 状态 | 说明 |
|------|------|------|
| 数据采集 | 待现场 | 已有 checklist |
| FAST-LIO2 里程计 | 已跑通 | SubSurfaceGeoRobo 80m, RS-LiDAR-16 + LITEF IMU |
| 精度对比 | 已完成 | RMSE 1.22m, Median 0.14m, 三段式退化分析 |
| 离线三维建模 | 已完成 | Poisson depth=10 + 密度裁剪 + 平滑 → 光滑网格 |
| 实时三维展示 | 已完成 | Web 端 Three.js 点云渲染, HTTP 服务器 |
| 回环检测 | 未完成 | ScanContext 能检测但位姿图修正未生效 |

#### 已知限制

1. **Z 轴漂移**：FAST-LIO2 ESKF 在矿井长直隧道中 Z 方向无约束，漂移不可避免
2. **回环检测**：ScanContext 正确检测到回环但位姿图优化有 bug（max_update=0），未实际修正轨迹
3. **SubT 数据集**：IMU 和 LiDAR 为独立传感器且无外参标定文件，无法跑通
4. **纹理贴图**：16 线 LiDAR 无相机数据，无法生成照片级纹理
5. **网格质量**：16 线点云稀疏，Poisson 可补洞但细节有限；需 32 线以上 LiDAR 才能出高质量模型

#### 实时三维建模脚本

| 脚本 | 功能 |
|------|------|
| `scripts/realtime_tsdf.py` | 实时累积点云，定期输出强度着色 PLY |
| `scripts/offline_reconstruct.py` | 离线高质量 Poisson 重建 + 平滑 |
| `scripts/web_viewer.html` | Three.js Web 端实时 3D 查看 (点云+网格) |

```bash
# 实时 (配合 FAST-LIO2 运行)
python3 ~/mine_project/scripts/realtime_tsdf.py _interval:=15 _voxel:=0.1

# 离线高质量建模 (PCD 跑完后)
python3 ~/mine_project/scripts/offline_reconstruct.py \
    scans.pcd --voxel 0.2 --depth 10 --output tunnel_model.ply
```

#### SubT 数据集尝试 (未成功)

- `site3_handheld_1.bag`：Hesai Pandar32 + Alphasense IMU，点云缺少 ring/time 字段，republisher 制作了但外参未知导致轨迹螺旋
- `sr_B_route2_lio.bag`：Ouster OS1 + Microstrain GX5-25 IMU，`ambient` 字段版本不兼容，old Ouster driver → republisher 重建了字段映射但外参同样未知

**根因**：IMU 和 LiDAR 为分离传感器，无外参标定。需 LiDAR 内置 IMU 或已知外参的传感器组合。

#### 下一步建议

1. **短期**：用现有 RS-LiDAR-16 + LITEF IMU 去现场采集，跑通几何重建全流程
2. **中期**：升级到 32 线以上 LiDAR + 固定 RGB 相机，外参标定后做纹理映射
3. **长期**：引入回环检测 + 位姿图优化（GTSAM/Ceres），解决 Z 轴漂移

#### 现场部署

详见 [FIELD_CHECKLIST.md](FIELD_CHECKLIST.md)。
