# RoboMaster Aim Lab ROS 2 自动瞄准

本项目实现 RoboMaster Aim Lab 的自动瞄准节点。节点只使用规定的 ROS 2
接口，订阅装甲板三维角点、云台状态和发射参数，计算带重力补偿的 yaw、
pitch，并根据目标运动状态决定开火时机。

## 环境

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- `colcon`
- `rm_aim_sim_bridge`

## 编译

先编译并加载仿真桥接工作空间：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rm_aim_sim_training/ros2_ws
colcon build --packages-select rm_aim_sim_bridge
source install/setup.bash
```

再编译本项目：

```bash
source /opt/ros/jazzy/setup.bash
source ~/rm_aim_sim_training/ros2_ws/install/setup.bash
cd ~/student_ws
colcon build --packages-select student_auto_aim --symlink-install
source install/setup.bash
```

## 运行

在一个终端启动仿真：

```bash
cd ~/rm_aim_sim_training
./run.sh
```

在另一个终端启动自动瞄准节点：

```bash
source /opt/ros/jazzy/setup.bash
source ~/rm_aim_sim_training/ros2_ws/install/setup.bash
source ~/student_ws/install/setup.bash

ros2 run student_auto_aim aim_node --ros-args \
  -p use_sim_time:=true \
  -p enable_fire:=true
```

在仿真窗口按 `R` 重置并开始新一轮。

## ROS 2 接口

节点以可靠、volatile、depth=1 的 QoS 使用以下话题：

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/aim_sim/armors` | `rm_aim_sim_bridge/msg/ArmorCornersArray` |
| 订阅 | `/aim_sim/gimbal` | `rm_aim_sim_bridge/msg/GimbalState` |
| 订阅 | `/aim_sim/telemetry` | `rm_aim_sim_bridge/msg/Telemetry` |
| 发布 | `/aim_sim/command` | `rm_aim_sim_bridge/msg/AimCommand` |

发布控制指令时，`header` 和 `session_id` 均复制自当前使用的装甲观测。
装甲列表为空、输入过期或没有可用弹道解时，节点发送
`enable=false, fire=false`。

## 算法

### 目标几何

每块装甲中心由四个三维角点求平均得到。使用左右边和上下边计算装甲切向量
与上方向，再通过叉积得到朝外的法向量。

只有一块可见装甲时，直接以该装甲中心作为目标。出现两块可见装甲时，根据
两块相邻装甲的中心、法向量和已知的 90 度夹角反推旋转中心与旋转半径。

### 运动估计

目标中心与旋转相位使用最近 7 帧观测进行短期拟合。

- 平移目标估计二维速度和加速度。
- 旋转目标通过法向量相位估计角速度。
- `session_id` 改变或仿真时间回退时，清空历史状态。
- 装甲使用预测位置做连续性匹配，不依赖数组下标。

预测位置使用二阶运动模型：

```text
p(t) = p0 + v*t + 0.5*a*t^2
theta(t) = theta0 + omega*t
```

### 弹道解算

节点从 `/aim_sim/telemetry` 读取弹速、重力、枪管长度、射击频率和开火延迟。
水平角由预测目标位置的 odom x、y 计算。俯仰角通过迭代低弹道解求解，并计入
枪口随云台旋转产生的水平与竖直偏移。

运动目标的预测时间包含：

```text
观测延迟 + 开火延迟 + 弹丸飞行时间
```

### 开火控制

节点比较指令角与实际云台反馈。只有在以下条件同时成立时发送 `fire=true`：

- 当前存在可见装甲。
- 装甲、云台和遥测数据足够新。
- 弹道解在云台俯仰限位内。
- yaw、pitch 误差均不超过默认 0.35 度。
- 预测位置处的装甲仍朝向己方。

每轮击毁后装甲持续为空，节点发送 `enable=false, fire=false`，不会继续射击。

## 可调参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `use_sim_time` | `true` | 使用仿真时间 |
| `enable_fire` | `true` | 是否允许自动开火 |
| `track_samples` | `7` | 运动拟合窗口 |
| `fire_delay_scale` | `1.0` | 开火延迟预测比例 |
| `aim_tolerance_deg` | `0.35` | 云台到位判定角误差 |

## 测试成绩

下表记录当前版本的最终验证结果。命中率以仿真窗口显示的
`score_hit_rate` 为准，击毁时间由仿真时间和装甲持续消失时刻记录。

| 任务 | 三轮命中率 | 近似击毁时间 |
|---|---|---|
| 4 m 固定靶 | 三轮均高于 94% | 3.012 / 2.879 / 3.325 s |
| 6 m 固定靶 | 90.0% | 2.492 / 2.862 / 2.371 s |
| 10 m 固定靶 | 85.7% | 2.304 / 2.304 / 2.287 s |
| 6 m 平移靶 | 62.1% / 64.3% / 46.2% | 26.046 / 4.896 / 6.458 s |
| 6 m 旋转靶 | 90.0% / 90.0% / 90.0% | 2.404 / 2.475 / 第三轮待补 |
| 6 m 平移旋转靶 | 72.0% / 66.7% / 60.0% | 4.696 / 4.496 / 5.008 s |

## 工程结构

```text
student_ws/
├── src/
│   └── student_auto_aim/
│       ├── package.xml
│       ├── setup.py
│       └── student_auto_aim/
│           └── aim_node.py
└── tools/
    └── score_monitor.py
```

`score_monitor.py` 仅用于辅助记录近似击毁时间。它订阅 `/clock` 和
`/aim_sim/armors`，不读取仿真内部真值，也不参与瞄准控制。
