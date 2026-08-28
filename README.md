# 基于 EGO-Planner 思想的无人机自主快递考核

这是一个面向大一学生的 Python 三维教学仿真包。学生不需要安装 ROS、PX4，也不需要复现真实 EGO-Planner；他们要完成的是公开定点顺序投放、任务状态机、局部规划器调用、窄门通过、安全恢复和实验验证。飞行、碰撞和回放使用 `x/y/z`，学生规划接口仍采用易于理解的二维局部栅格＋障碍高度图。

> **重要：** 本仓库中的 `EgoLikePlanner` 只是教学抽象。它用局部占据栅格、A* 引导路径、三次 B 样条平滑和简单时间分配来模拟“局部轨迹规划器”在系统中的角色，**不是**浙江大学 FAST Lab 的 EGO-Planner 源码、移植版或等价复现。

## 应该使用 Windows 还是 Ubuntu？

本项目是纯 Python 教学仿真，Windows、Ubuntu 和 macOS 都可以运行，不需要 ROS。

- **学生统一环境推荐：Windows 10/11 + 64位 Python 3.11 或 3.12。** 大一学生安装最简单，也方便录屏和提交作业。
- 教师或已有 Linux 环境的学生可以使用 Ubuntu 22.04/24.04，仿真结果与 Windows 相同。
- 只有以后接入真实 EGO-Planner、PX4 和 ROS 时，才建议另建 Ubuntu 环境；不要把 ROS 安装作为本题考核内容。

Windows 用户安装 Python 时须勾选 `Add Python to PATH`，然后双击 `setup_windows.bat`。Ubuntu 用户运行：

```bash
chmod +x setup_ubuntu.sh
./setup_ubuntu.sh
```

## 快速开始

```bash
git clone https://github.com/david312213/robocup-uav-exam-student.git
cd robocup-uav-exam-student
python3 -m pip install -r requirements.txt

# 起始代码：能起飞并飞向第一个投放区，但不会执行投放
python3 run_sim.py --controller student --seed 1001 --plot artifacts/student.png --events

# 需要原俯视图时显式选择 2D
python3 run_sim.py --controller student --seed 1001 --view 2d --animate

# 快速检查前三个公开场景
python3 evaluate.py --controller student --limit 3

# 三组参数、全部 20 个公开场景
python3 evaluate.py --controller student

# 标准库测试，无需 pytest
python3 -m unittest discover -s tests -v
```

单次运行可以增加 `--animate` 打开三维轨迹回放，或用 `--gif artifacts/demo.gif` 生成 GIF。学生录屏时建议使用 `--animate`；`--view 2d` 可切换为俯视图。

场地为 `10m × 10m × 5m`，全部障碍高 5m。三个公开投放区均为边长 0.5m 的地面正方形，中心依次为蓝色 `(2.0, 2.0)`、紫色 `(4.0, 5.0)`、绿色 `(5.5, 8.0)`。无人机必须按此顺序在 0.8–1.5m 高度显式投放；任务满分为100分。

## 学生需要修改的内容

- `student_controller.py`：唯一必改代码，接口为 `StudentController.step(observation, planner) -> Action`。
- `configs/adaptive.json`：学生自适应参数方案。
- `REPORT_TEMPLATE.md` 和 `sources.md`：报告与来源记录模板。

运行时只接受 `FOLLOW_TRAJECTORY`、`HOVER`、`DROP`、`LAND`、`ABORT` 五种动作。投放区通过 `Observation.mission.delivery_zones` 公开；障碍和门洞位置仍未知，只能从局部栅格推断。为兼容旧控制器，`Observation.detections` 字段仍存在，但本题中始终为空。

## 题目文件

完整公开题面见 [ASSIGNMENT.md](ASSIGNMENT.md)，报告结构见 [REPORT_TEMPLATE.md](REPORT_TEMPLATE.md)。本公开仓库不包含教师参考控制器、隐藏场景或教师评分脚本。

## 参考资料

- [EGO-Planner 论文：EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors](https://arxiv.org/abs/2008.08835)
- [EGO-Planner 官方仓库](https://github.com/ZJU-FAST-Lab/ego-planner)
