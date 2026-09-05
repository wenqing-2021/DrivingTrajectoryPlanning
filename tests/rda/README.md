# RDA Planner 单元测试说明

本目录包含 RDA（Reduced Dual-space ADMM）规划器的单元测试，验证**直线避障**场景下的正确性。

> 参考实现：[RDA-planner (Python)](https://github.com/hanruihua/RDA-planner/blob/main/RDA_planner/rda_solver.py)
> 论文：*RDA: An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments*

---

## 1. 问题构建（Problem Formulation）

### 1.1 运动学模型（无滑移角 β 的自行车模型）

状态 $X = [x, y, \theta, v]^\top$，控制 $U = [\delta, a]^\top$（转向角、加速度）：

$$
\begin{aligned}
x_{t+1}     &= x_t + v_t \cos\theta_t \, dt_t \\
y_{t+1}     &= y_t + v_t \sin\theta_t \, dt_t \\
\theta_{t+1}&= \theta_t + \frac{v_t}{L}\tan\delta_t \, dt_t \\
v_{t+1}     &= v_t + a_t \, dt_t
\end{aligned}
$$

其中 $L$ 为轴距，$dt_t$ 为第 $t$ 段的时间间隔（**也是优化变量**）。

### 1.2 线性化（在当前参考点处）

由于动力学对 $(X, U, dt)$ 是双线性的，在参考点 $(\bar v, \bar\delta, \bar\theta, \bar a, \overline{dt})$ 处一阶泰勒展开：

$$
X_{t+1} \approx A_t X_t + B_t U_t + D_t\, dt_t + C_t
$$

$$
A_t = \frac{\partial f}{\partial X},\quad
B_t = \frac{\partial f}{\partial U},\quad
D_t = \frac{\partial f}{\partial dt} = \begin{bmatrix} \bar v\cos\bar\theta \\ \bar v\sin\bar\theta \\ \frac{\bar v}{L}\tan\bar\delta \\ \bar a \end{bmatrix},\quad
C_t = f(\bar X,\bar U,\overline{dt}) - A_t\bar X - B_t\bar U - D_t\overline{dt}
$$

### 1.3 碰撞规避（对偶形式）

每个障碍物是凸多边形，用半平面表示内部：$\{p : A_{obs}\, p \le b_{obs}\}$（法向量朝外）。
车辆外形用矩形 $G_{veh}\, c \le h_{veh}$。

由对偶理论，车辆与障碍物无碰撞 ⟺ 存在对偶变量 $\lambda \ge 0, \mu \ge 0$ 使得：

$$
\begin{aligned}
\lambda^\top (A_{obs}\, s - b_{obs}) - \mu^\top h_{veh} &\ge d \quad \text{(安全距离 } d\text{)} \\
G_{veh}^\top \mu + (A_{obs} R(\theta))^\top \lambda &= 0 \\
\|A_{obs}^\top \lambda\| &\le 1
\end{aligned}
$$

其中 $s = [x, y]^\top$ 为车辆中心，$R(\theta)$ 为旋转矩阵。

### 1.4 ADMM 残差

定义两个残差（对应上述两个等式/不等式）：

$$
\begin{aligned}
I_m &= \lambda^\top (A_{obs}\, s - b_{obs}) - \mu^\top h_{veh} - d - z + \zeta \\
H_m &= \mu^\top G_{veh} + \lambda^\top A_{obs} R(\theta) + \xi
\end{aligned}
$$

$\zeta, \xi$ 为累积对偶变量，$z \ge 0$ 为松弛变量。

---

## 2. 迭代求解（ADMM 三阶段）

每次 ADMM 迭代依次求解三个子问题：

### 2.1 SU 子问题（QP，OSQP 求解）

优化变量：$[X(4(T{+}1)),\ U(2T),\ d(T),\ dt(T),\ r(n_{obs}{\cdot}T)]$

$$
\min_{X,U,d,dt,r}\quad
\underbrace{w_s\sum_t\|X_t - X_t^{ref}\|^2}_{\text{跟踪}}
+ \underbrace{w_u\sum_t\|U_t\|^2}_{\text{控制}}
+ \underbrace{(-g_{slack}\sum_t d_t)}_{\text{最大化安全距离}}
+ \underbrace{w_{dt}\sum_t (dt_t - \overline{dt})^2}_{\text{时间正则}}
+ \underbrace{\tfrac{\rho_1}{2}\sum_{o,t} r_{o,t}^2}_{\text{Im 罚项}}
+ \underbrace{\tfrac{\rho_2}{2}\sum_{o,t}\|H_{m,o,t}\|^2}_{\text{Hm 罚项}}
$$

约束：

$$
\begin{aligned}
X_{t+1} &= A_t X_t + B_t U_t + D_t\, dt_t + C_t && \text{线性化动力学} \\
r_{o,t} &\ge -I_{m,o,t},\quad r_{o,t} \ge 0 && \text{accelerated ADMM（只惩罚违反部分）} \\
X_0 &\in [X_{start} \pm \epsilon],\quad X_T \in [X_{goal} \pm \epsilon] && \text{起终点 box} \\
v &\in [-v_{max}, v_{max}],\ \delta \in [-\delta_{max}, \delta_{max}],\ a \in [-a_{max}, a_{max}] \\
d &\in [d_{min}, d_{max}],\quad dt \in [dt_{min}, dt_{max}]
\end{aligned}
$$

> **Accelerated ADMM**：引入辅助变量 $r \ge \max(0, -I_m)$，只惩罚违反部分（对应 Python 的 `sum_squares(neg(Im))`），比完整平方收敛更稳。

### 2.2 LamMuZ 子问题（SOCP，EiCOS 求解）

对每个障碍物分别求解对偶变量 $(\lambda, \mu, z)$：

$$
\min_{\lambda,\mu,z,t}\; t
\quad \text{s.t.}\quad
\left\| \begin{bmatrix} \sqrt{\rho_1/2}\, I_{m,t} \\ \sqrt{\rho_2/2}\, H_{m,t} \end{bmatrix} \right\| \le t,\quad
\|A_{obs}^\top \lambda_t\| \le 1,\quad
\lambda \ge 0,\ \mu \ge 0,\ z \ge 0
$$

第一个约束是 epigraph 二阶锥（SOC），第二个是对偶范数 SOC。

### 2.3 对偶变量更新

$$
\begin{aligned}
\xi_t   &\mathrel{+}= G_{veh}^\top \mu_t + (A_{obs} R(\theta_t))^\top \lambda_t \\
\zeta_t &\mathrel{+}= \lambda_t^\top (A_{obs}\, s_t - b_{obs}) - \mu_t^\top h_{veh} - d_t - z_t
\end{aligned}
$$

### 2.4 收敛判据

- 原始残差：$r_{pri} = \sqrt{\sum_{o,t} \|H_{m,o,t}\|^2}$
- 对偶残差：$r_{dual} = \sqrt{\sum_o \left( \|\Delta\lambda_o\|^2 + \|\Delta\mu_o\|^2 + \|\Delta z_o\|^2 \right)}$

当 $r_{pri} < \epsilon$ 且 $r_{dual} < \epsilon$（至少迭代 2 次后）提前停止。

---

## 3. 单元测试场景

测试场景与 OBCA 单元测试完全一致，便于对比两种规划器。

- **起点**：$(0, 0)$，朝向 $+x$，$v=0$
- **终点**：$(30, 0)$，朝向 $+x$，$v=0$
- **障碍物**：方块 $(16..18, 0.2..1.2)$，位于路径上方（车宽 2 m 会碰撞）
- **初始路径**：$y=0$ 直线，60 个路径点（$x = i \cdot 30/59$）
- **车辆**：长 5.0 m，宽 2.0 m，轴距 2.5 m，前/后悬 1.25 m
- **地图边界**：$x \in [-1, 31]$，$y \in [-2, 3]$

### 验证项
1. `Process()` 返回 `true`
2. 到达终点（误差 < 0.05 m）
3. 无碰撞（车辆中心在障碍物半平面外）
4. 轨迹偏航绕障（max|y| > 0.1 m）

---

## 4. 编译与运行

```bash
# 配置 + 编译（在仓库根目录）
cmake -S . -B build
cmake --build build --target test_rda_main

# 运行单元测试
cd /tmp && /root/workspace/AutomatedPark/build/tests/rda/test_rda_main

# 生成可视化对比图
python3 /root/workspace/AutomatedPark/tests/rda/visualize_rda_result.py
```

输出：
- CSV 结果：`/tmp/rda_test_results/*.csv`
- 对比图：`/tmp/rda_test_results/rda_result.png`

---

## 5. 结果对比

见 `rda_result.png`：绿色为 RDA 优化轨迹（向下绕开上方障碍物），蓝色虚线为初始参考路径（$y=0$ 直线）。

| 指标 | 结果 |
|---|---|
| 迭代次数 | 50（最大迭代） |
| 终点误差 | dx = 0.064 m, dy = 0.016 m |
| 最小安全间距 | 1.70 m |
| 最大横向偏移 | 2.00 m（向下绕障） |
| 最大速度 | 3.21 m/s |
