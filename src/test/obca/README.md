# OBCA Planner Test - C++ 独立测试

这是一个针对OBCA（Optimization-Based Collision Avoidance）路径规划器的独立C++测试套件。

## 📋 测试概述

### 测试场景配置

- **起始点**: (0.0, 0.0)
- **目标点**: (30.0, 0.0)
- **初始路径**: 从起点到终点的直线，包含31个采样点（间距1.0m）
- **障碍物**: 方形障碍物，范围从 (11, 0.7) 到 (13, 1.7)，高度为1.0m，宽度为2.0m
- **车辆**: 长5.0m，宽2.0m，轴距2.5m

### 车辆参数

```
length: 5.0 m                  # 车身长度
width: 2.0 m                   # 车身宽度
wheel_base: 2.5 m              # 轴距（前后轴中心距離）
max_steer_angle: 0.5 rad       # 最大转向角（约28.6度）
max_acc: 2.0 m/s²              # 最大加速度
max_velocity: 5.0 m/s          # 最大速度
front_overhang: 1.25 m         # 前悬（后轴中心到前保险杠）
rear_overhang: 1.25 m          # 后悬（后轴中心到后保险杠）

注：前悬+轴距+后悬 = 1.25 + 2.5 + 1.25 = 5.0 m（车身总长）
```

## 📁 文件说明

| 文件 | 说明 |
|------|------|
| `test_obca_main.cpp` | ⭐ 主C++测试程序（300+行） |
| `visualize_obca_result.py` | Python可视化脚本 |
| `CMakeLists.txt` | CMake构建配置 |
| `README.md` | 本文件 |

## 🚀 快速开始

### 前置要求

1. 项目已编译：
```bash
cd /root/workspace/AutomatedPark
mkdir -p build && cd build
cmake ..
make -j4
```

2. 安装Python依赖：
```bash
pip install matplotlib numpy
```

### 运行测试

```bash
# 运行OBCA测试程序（生成结果CSV和结果到 /tmp/obca_test_results/）
./build/src/test/obca/test_obca_main

# 运行Python可视化脚本（生成 obca_result.png）
python3 src/test/obca/visualize_obca_result.py
```

## 📊 输出说明

### C++程序输出

程序会输出以下信息到标准输出：

```
========== OBCA Planner Test ==========
Setting up problem...
Vehicle created: length=2.0m, width=1.0m
Start position: (0, 0)
Goal position: (10, 0)
Obstacle created: Square from (4, -0.5) to (6, 0.5)
Map created with resolution 0.1m
Saving initial problem data...
Creating initial path...
Initial path created with 11 waypoints
Configuring IPOPT solver...
Creating OBCA solver...
Running OBCA optimization...
✓ OBCA solver succeeded!
Extracting results...
Number of states: 11
Number of controls: 10
...
========== Test Complete ==========
Results saved to: /tmp/obca_test_results/
```

### CSV输出文件

测试完成后会在 `/tmp/obca_test_results/` 目录生成以下CSV文件：

| 文件 | 列 | 说明 |
|------|-----|------|
| `initial_path.csv` | x, y | 初始路径的采样点 |
| `optimized_trajectory.csv` | x, y, theta, v | OBCA优化后的轨迹 |
| `optimized_controls.csv` | acceleration, steer_angle | 优化后的控制序列 |
| `start_goal.csv` | x, y | 起点和终点 |
| `obstacle_*.csv` | x, y | 障碍物顶点坐标 |

### 可视化输出

`visualize_obca_result.py` 脚本会生成一个PNG图像 `obca_result.png`，包含3个子图：

**1. 轨迹图（上方，占2行）**
- 蓝色虚线：初始路径的后轴中心轨迹
- 绿色实线：OBCA优化后的后轴中心轨迹
- 蓝色/绿色车体：沿轨迹绘制的车身包络（每3个采样点一个）
- 蓝色车体方块：起始位置车体（透明度40%）
- 红色车体方块：终止位置车体（透明度40%）
- 绿色三角形：起始点
- 红色星标：目标点
- 红色填充区域：障碍物范围

**2. 速度曲线图（左下）**
- 蓝色曲线：沿路径距离的速度变化
- 红色虚线：最大速度限制（5.0 m/s）
- x轴：沿路径的累计距离（米）
- y轴：速度（m/s）

**3. 控制输入图（右下，双y轴）**
- 绿色曲线：加速度序列，左y轴，单位m/s²
- 橙色曲线：转向角序列，右y轴，单位度
- 绿色/橙色虚线：对应的上下限

## 🔍 代码结构

### test_obca_main.cpp 主要部分

```cpp
1. 设置问题 (Setup Problem)
   ├─ 创建车辆参数
   ├─ 创建运动学模型
   ├─ 定义规划问题（起点、终点、障碍物）
   └─ 创建地图

2. 创建初始路径 (Create Initial Path)
   ├─ 生成11个采样点
   └─ 保存为CSV

3. 配置求解器 (Configure Solver)
   ├─ 设置IPOPT选项
   └─ 创建OBCA求解器

4. 运行优化 (Run OBCA Solver)
   ├─ 调用 Process() 方法
   └─ 检查求解状态

5. 提取结果 (Extract Results)
   ├─ 获取优化轨迹
   ├─ 获取控制序列
   └─ 保存为CSV
```

## 📈 预期结果

### 成功标志

✓ 程序返回0（成功）  
✓ 输出中显示 "✓ OBCA solver succeeded!"  
✓ 生成所有CSV文件  
✓ 生成可视化PNG图像（包含3个子图）  
✓ 最终位置接近目标 (dx < 0.2m, dy < 0.2m)  

### 典型输出

```
========== OBCA Planner Test ==========
Setting up problem...
Vehicle created: length=5.0m, width=2.0m
Start position: (0, 0)
Goal position: (30, 0)
Obstacle created: Square from (11, 0.7) to (13, 1.7)
Map created with resolution 0.1m
Configuring IPOPT solver...
IPOPT configured
Creating OBCA solver...
Running OBCA optimization...
✓ OBCA solver succeeded!
Extracting results...
Number of states: 31
Number of controls: 30
First state: x=0.000000 y=0.000000 theta=0.000000 v=0.000000
Last state: x=29.950000 y=-0.165000 theta=-0.050000 v=0.000000
Final position error: dx=0.050000m, dy=0.165000m
✓ Test PASSED: Final position close to goal
========== Test Complete ==========
Results saved to: /tmp/obca_test_results/
```

## 🛠️ 自定义测试

### 修改初始路径

编辑 `test_obca_main.cpp` 中的初始路径生成部分（当前：31个点，从(0,0)到(30,0)）：

```cpp
// Generate initial path: modify this section
for (int i = 0; i <= 30; ++i) {  // 改数字改变采样点数
    vehicle_model::VehiclePose pose;
    pose.x     = i * 1.0;       // 改间距或乘数改变路径长度
    pose.y     = 0.0;           // 改y坐标，例如：sin(i*0.2)*0.5 制造蛇形
    pose.theta = 0.0;           // 初始朝向
    init_path.push_back(pose);
}
```

**重要**：目标点也要修改为与最后一个路径点一致
```cpp
goal_pose_ptr->x     = 30.0;    // 改为最后一个点的x坐标
goal_pose_ptr->y     = 0.0;     // 改为最后一个点的y坐标
```

### 添加新的障碍物

在问题设置部分添加（当前障碍物：x∈[11,13], y∈[0.7,1.7]）：

```cpp
// Add obstacle 2
problem::Polygon* obs_polygon2 = plan_problem.add_obstacle_list();
obs_polygon2->set_vertex_num(4);

// Bottom-left
cost_map::Pos2D* v1 = obs_polygon2->add_vertex_pts();
v1->set_x(20.0);
v1->set_y(-1.0);

// Bottom-right
cost_map::Pos2D* v2 = obs_polygon2->add_vertex_pts();
v2->set_x(22.0);
v2->set_y(-1.0);

// Top-right
cost_map::Pos2D* v3 = obs_polygon2->add_vertex_pts();
v3->set_x(22.0);
v3->set_y(0.0);

// Top-left
cost_map::Pos2D* v4 = obs_polygon2->add_vertex_pts();
v4->set_x(20.0);
v4->set_y(0.0);

plan_problem.set_obstacle_num(2);  // 更新障碍物总数
LOG(INFO) << "Obstacle 2 created: Rectangle from (20, -1) to (22, 0)";
```

### 修改车辆参数

在车辆参数设置部分修改（当前值：长5.0m, 宽2.0m, 轴距2.5m）：

```cpp
vehicle_param.set_length(6.0);           // 车身长度
vehicle_param.set_width(2.5);            // 车身宽度
vehicle_param.set_wheel_base(3.0);       // 轴距
vehicle_param.set_front_overhang(1.5);   // 前悬
vehicle_param.set_rear_overhang(1.5);    // 后悬
vehicle_param.set_max_steer_angle(0.6);  // 最大转向角（弧度）
vehicle_param.set_max_acc(3.0);          // 最大加速度
vehicle_param.set_max_velocity(6.0);     // 最大速度
```

**重要**：修改车辆参数后，同时更新 `visualize_obca_result.py` 中的对应参数：
```python
VEHICLE_LENGTH = 6.0      # 同 set_length
VEHICLE_WIDTH = 2.5       # 同 set_width
REAR_OVERHANG = 1.5       # 同 set_rear_overhang
```

### 修改求解器参数

在IPOPT选项部分修改（当前配置基于目标函数变化收敛）：

```cpp
std::string ipopt_options;
ipopt_options += "String  sb                  yes\n";          // 启用IPOPT抓不哈法则
ipopt_options += "Integer max_iter            50\n";           // 最大迭代次数
ipopt_options += "Numeric tol                 1e-3\n";         // 一阶最优性容差
// 核心收敛参数：基于目标函数变化
ipopt_options += "Numeric obj_tol             1e-5\n";         // ⭐ 目标函数相对变化容差
ipopt_options += "Numeric acceptable_tol      1e-2\n";         // 可接受解的容差
ipopt_options += "Integer acceptable_iter     5\n";            // 满足可接受条件5次后停止
ipopt_options += "Numeric max_cpu_time        120.0\n";        // 最大求解时间
```

**IPOPT收敛参数说明：**

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `tol` | 1e-3 | 一阶最优性条件容差（影响最终精度） |
| **`obj_tol`** | **1e-5** | **⭐ 目标函数相对变化容差（核心参数）**<br>连续迭代间目标函数变化 < 1e-5 时停止 |
| `acceptable_tol` | 1e-2 | "可接受"解的容差（更松散的条件） |
| `acceptable_iter` | 5 | 满足可接受条件5次后允许提前终止 |
| `max_iter` | 50 | 最多迭代50次 |
| `max_cpu_time` | 120s | 最大求解时间（秒） |

**调参建议：**
- 目标函数变化缓慢 → 增大 `obj_tol` (如 1e-4)
- 需要更高精度 → 减小 `obj_tol` (如 1e-6)  
- 求解速度过慢 → 增大 `acceptable_tol` 或减少 `acceptable_iter`
- 提高整体精度 → 减小 `tol` (如 1e-4)

## 🐛 故障排查

### 编译错误

**错误**: `'obca_planner.h' file not found`

**解决**:
- 确保项目已编译：`cd build && cmake .. && make`
- 检查CMakeLists.txt中的include_directories配置

**错误**: `undefined reference to ...`

**解决**:
- 确保CMakeLists.txt中链接了所有必要的库
- 运行 `make clean && make` 重新编译

### 运行时错误

**错误**: 求解器失败 (OBCA solver failed)

**解决方案**:
1. 检查初始路径是否合理
2. 确保障碍物定义正确
3. 尝试增加迭代次数
4. 降低容差要求

### Python可视化错误

**错误**: `ModuleNotFoundError: No module named 'matplotlib'`

**解决**:
```bash
pip install matplotlib numpy
```

**错误**: 找不到CSV文件

**解决**:
- 确保test_obca_main已成功运行
- 检查/tmp/obca_test_results目录是否存在和包含文件

## 📚 相关文件

- 实现文件: `src/planner/obca/obca_planner.cpp/h`
- 地图类: `src/map/map.h/cpp`
- 车辆模型: `src/common/vehicle_model/kinematic_model.h`
- 数学库: `src/common/math/polygon2d.h`

##  坐标系约定

所有坐标均基于以下约定：
- **原点**: (0, 0) 在地图左下角
- **x轴**: 指向右方（正方向）
- **y轴**: 指向上方（正方向）
- **车辆状态向量**: (x, y, theta, v)
  - **(x, y)**: 后轴中心在世界坐标系中的位置
  - **theta**: 车体朝向角（弧度），0表示沿+x方向
  - **v**: 后轴中心的速度（m/s）

## ✨ 总结

这个测试套件提供了：
- ✅ 独立的C++测试程序，不依赖pybind
- ✅ 完整的OBCA规划器功能测试
- ✅ CSV格式的结果保存
- ✅ Python可视化脚本
- ✅ 易于定制和扩展的结构

---

**版本**: 1.0  
**创建时间**: 2026年1月31日  
**状态**: ✅ 完成并就绪
