# DrivingTrajectoryPlanning
This repository implements a variety of trajectory planning methods for autonomous driving, with a focus on optimization-based approaches. It integrates optimal control planning models with solver interfaces for both OSQP and IPOPT. In addition, the project offers an intuitive visualization window and a streamlined execution pipeline for ease of use.

# 1. Installation
Before start, you should install [docker](https://www.docker.com/) for Linux or [docker-desktop](https://www.docker.com/products/docker-desktop/) for Windows. After that, clone this repo and run the following command to build images:
```
git clone --recurse-submodules https://github.com/wenqing-2021/DrivingTrajectoryPlanning.git
cd DrivingTrajectoryPlanning
python3 docker/run_docker.py -b
```

# 2. Run
Enter the Container and execute the following commands for start:
```
bash scripts/build.sh
bash scripts/run.sh
```

# 3. Optimization planner

## 3.1 frontend-search
The frontend search stage uses Hybrid A* to generate an initial collision-free path for the backend optimizer.
Core implementation and related modules:
- [Hybrid A* interface](src/planner/hybrid_a_star/hybrid_a_star.h)
- [Reeds-Shepp path utilities](src/planner/hybrid_a_star/rs_path.cpp)

## 3.2 backend-opt
The backend optimization stage refines the frontend path into a smoother and dynamically feasible trajectory.
This project currently supports OBCA (IPOPT-based) and piecewise-jerk speed optimization (OSQP-based), and also includes an ADMM module under development.
Core implementation and related modules:
- [Solver configuration](src/config/solver_params.yaml)
- [OBCA solver interface](src/planner/obca/obca_planner.h)
- [Piecewise jerk speed optimizer](src/planner/speed_planner/piece_wise_jerk.cpp)
- [ADMM module](src/planner/admm/admm_planner.cpp)
- [Planner backend dispatch](src/planner/trajectory_planner.cpp)

# 4. TODO
[] check obca running settings

[] dev ADMM method

[] add commonroad interface