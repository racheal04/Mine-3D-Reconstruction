# 现场数据采集 → 三维建模 完整 Checklist

## 出发前确认

- [ ] 笔记本充满电，**带充电器**
- [ ] 硬盘空间 > 50GB
- [ ] 确认机器狗上 LiDAR 型号和线数（问厂商）
- [ ] 确认 IMU 型号（问厂商）
- [ ] 确认 LiDAR 和 IMU 是否物理对齐（刚性连接？外参已知？）
- [ ] RTK 基站/NTRIP 账号
- [ ] 笔记本已编译 FAST-LIO2：`~/mine_project/devel/lib/fast_lio/fastlio_mapping` 存在
- [ ] 笔记本有网口/USB 网口转换器（连 LiDAR 用）

---

## 现场操作

### 1. 设备上电
- [ ] 机器狗开机，放平地
- [ ] LiDAR 上电，确认旋转
- [ ] 笔记本连上传感器网络（网线/WiFi）
- [ ] RTK 基站架好/NTRIP 连上

### 2. 检查数据流
```bash
source /opt/ros/noetic/setup.bash
# LiDAR 在发吗？
rostopic hz <lidar_topic>      # 应该 10Hz
# IMU 在发吗？
rostopic hz <imu_topic>        # 应该 >=100Hz
# 如果频率不对 → 检查驱动/线缆，不要开始采集
```

### 3. 准备配置
- [ ] 根据实际 LiDAR 型号，修改 `config/robosense16.yaml` 中的：
  - `lid_topic`（LiDAR topic 名）
  - `imu_topic`（IMU topic 名）
  - `scan_line`（LiDAR 线数）
  - `scan_rate`（LiDAR 频率）
- [ ] 如果点云缺少 ring/time → 现场写转换脚本
- [ ] 如果需要，修改 launch 文件

### 4. 启动采集
```bash
# 终端 1
source /opt/ros/noetic/setup.bash
roscore

# 终端 2（点云转换，如果需要）
python3 <republisher>.py &

# 终端 3（FAST-LIO2）
source /opt/ros/noetic/setup.bash
source ~/mine_project/devel/setup.bash
roslaunch fast_lio mapping_robosense16.launch

# 终端 4（先开 rosbag，再让狗动！）
rosbag record -a -O ~/field_data/$(date +%Y%m%d_%H%M%S).bag
```

### 5. 移动采集
- [ ] **先开 rosbag，确认在录**
- [ ] 启动后静止 5~10 秒（IMU 初始化）
- [ ] 匀速 0.3~0.5 m/s
- [ ] 转弯放慢（角速度 < 30°/s）
- [ ] 如果有回环路线（走一圈回到起点）→ **最有价值**
- [ ] 结束前静止 5 秒

### 6. 结束
- [ ] Ctrl+C 停 rosbag
- [ ] Ctrl+C 停 FAST-LIO2（会自动保存 PCD 到 `PCD/scans.pcd`）
- [ ] 检查 PCD 文件大小 > 0

---

## 回来后处理

### 7. 点云构建
```bash
# 回放 bag 生成 PCD
roscore &
rosbag play ~/field_data/<bag_file> -r 1 --clock &
roslaunch fast_lio mapping_robosense16.launch
# FAST-LIO2 退出后 PCD 自动保存在 FAST_LIO/PCD/scans.pcd
```

### 8. 精度评估（如果有真值）
```bash
python3 ~/mine_project/scripts/compare_pointclouds.py \
    ~/mine_project/src/FAST_LIO/PCD/scans.pcd \
    <ground_truth.las> \
    --voxel 0.1 --output ~/outputs/error_colored.pcd
```

### 9. 三维建模
```bash
# 快速预览
python3 ~/mine_project/scripts/reconstruct_mesh.py \
    ~/mine_project/src/FAST_LIO/PCD/scans.pcd \
    --voxel 0.2 --depth 8 \
    --output ~/outputs/tunnel_mesh.ply
```

### 10. 输出来源
- [ ] `PCD/scans.pcd` — 点云地图
- [ ] `outputs/tunnel_mesh.ply` — 三维网格模型
- [ ] `outputs/error_colored.pcd` — 精度评估（如有真值）
- [ ] `field_data/*.bag` — 原始数据（存档）

---

## 常见故障速查

| 现象 | 原因 | 解决 |
|------|------|------|
| 没有轨迹 | IMU topic 不通 | `rostopic hz <imu_topic>` |
| 轨迹跳变 | 时间戳不对齐 | 检查 LiDAR 和 IMU 时间戳 |
| Z 轴飞 | IMU 初始化不够 | 启动后静止 10 秒 |
| 点云断层 | 走太快 | 慢到 0.5 m/s 以下 |
| 轨迹螺旋 | LiDAR-IMU 外参错误 | 确认传感器物理安装角度 |

---

## 核心原则

1. **传感器必须刚性连接** — LiDAR 和 IMU 不能有相对运动
2. **先录后动** — rosbag 确认在写再让狗走
3. **匀速慢走** — 快了丢帧，快了飘
4. **回环最值钱** — 走回起点一次胜过走两倍距离
5. **多录几组** — 不同速度/路线，回来挑最好的
