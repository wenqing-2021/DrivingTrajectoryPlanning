# OBCA Planner Test - C++ 独立测试

这是一个针对OBCA（Optimization-Based Collision Avoidance）路径规划器的独立C++测试套件。

## 📋 测试概述

### 测试场景配置

- **起始点**: (0, 0)
- **目标点**: (10, 0)
- **初始路径**: 从起点到终点的直线，包含11个采样点
- **障碍物**: 中心位置的方形障碍物，范围从 (4, -0.5) 到 (6, 0.5)
- **车辆**: 长2.0m，宽1.0m，轴距1.2m

### 车辆参数

```
length: 2.0 m                  # 车身长度
width: 1.0 m                   # 车身宽度
wheel_base: 1.2 m              # 轴距
max_steer_angle: 0.5 rad       # 最大转向角
max_acc: 2.0 m/s²              # 最大加速度
max_velocity: 5.0 m/s          # 最大速度
front_overhang: 0.5 m          # 前悬
rear_overhang: 0.3 m           # 后悬
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

#### 方法1: 直接运行编译后的可执行文件
```bash
# 构建后的可执行文件位置
./build/src/test/obca/test_obca_main

# 查看结果
python3 src/test/obca/visualize_obca_result.py
```

#### 方法2: 通过CMake/CTest运行
```bash
cd /root/workspace/AutomatedPark/build

# 运行OBCA测试
ctest -R test_obca_main -VV

# 运行可视化
ctest -R visualize_obca -VV
```

#### 方法3: 使用自定义结果目录
```bash
# 指定结果目录运行可视化
python3 src/test/obca/visualize_obca_result.py /tmp/obca_test_results
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

`visualize_obca_result.py` 脚本会生成一个PNG图像 `obca_result.png`，包含：

- **蓝色虚线**: 初始路径
- **绿色实线**: OBCA优化后的轨迹
- **绿色三角形**: 起始点
- **红色星标**: 目标点
- **红色填充区域**: 障碍物

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
✓ 生成可视化PNG图像  
✓ 最终位置接近目标 (dx < 0.2m, dy < 0.2m)  

### 典型输出

```
First state: x=0.000000 y=0.000000 theta=0.000000 v=0.000000
Last state: x=9.950000 y=-0.050000 theta=-0.050000 v=0.000000
Final position error: dx=0.050000m, dy=0.050000m
✓ Test PASSED: Final position close to goal
```

## 🛠️ 自定义测试

### 修改初始路径

编辑 `test_obca_main.cpp` 中的初始路径生成部分：

```cpp
// Generate initial path: modify this section
for (int i = 0; i <= 10; ++i) {
    vehicle_model::VehiclePose pose;
    pose.x     = i * 1.0;      // 修改x坐标
    pose.y     = 0.0;          // 修改y坐标（例如：sin(i)*0.5）
    pose.theta = 0.0;
    init_path.push_back(pose);
}
```

### 添加新的障碍物

在问题设置部分添加：

```cpp
// Add another obstacle
problem::Polygon* obs_polygon2 = plan_problem.add_obstacle_list();
obs_polygon2->set_vertex_num(4);

cost_map::Pos2D* v1 = obs_polygon2->add_vertex_pts();
v1->set_x(2.0);
v1->set_y(1.0);
// ... 添加其他顶点

plan_problem.set_obstacle_num(2);
```

### 修改车辆参数

在车辆参数设置部分修改：

```cpp
vehicle_param.set_length(3.0);           // 改变长度
vehicle_param.set_width(1.5);            // 改变宽度
vehicle_param.set_wheel_base(1.5);       // 改变轴距
```

### 修改求解器参数

在IPOPT选项部分修改：

```cpp
std::string ipopt_options;
ipopt_options += "Integer print_level         2\n";    // 增加输出等级
ipopt_options += "Integer max_iter            200\n";   // 增加迭代次数
ipopt_options += "Numeric tol                 1e-4\n";  // 放宽容差
```

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

## 💡 进阶用法

### 批量测试多个场景

创建shell脚本 `run_multiple_tests.sh`:

```bash
#!/bin/bash

# 运行基本测试
./build/src/test/obca/test_obca_main

# 可视化
python3 src/test/obca/visualize_obca_result.py

# 备份结果
cp -r /tmp/obca_test_results ./results_$(date +%Y%m%d_%H%M%S)
```

### 性能分析

在C++程序中添加计时代码：

```cpp
#include <chrono>

auto start = std::chrono::high_resolution_clock::now();
bool success = obca_solver.Process(init_path, start_pose_ptr, goal_pose_ptr);
auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
LOG(INFO) << "Solve time: " << duration.count() << " ms";
```

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
