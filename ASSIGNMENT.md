# 高难综合题：基于 EGO-Planner 思想的自主快递无人机（40分）

## 一、背景与规则

你需要为一架保护外径为 0.38m 的无人机设计自主快递系统。场地尺寸为 `10m × 10m × 5m`，任务限时 600 秒。箱子、树木、边界、走廊墙和门墙均高5m，不能从上方越过；障碍和门洞位置未知。

无人机携带蓝、紫、绿三个快递盒。三个投放区是地面上的 `0.5m × 0.5m` 正方形，坐标表示中心且在任务开始前公开：

| 顺序 | 货物/区域 | 中心坐标 |
|---:|---|---|
| 1 | `blue` | `(2.0, 2.0)` |
| 2 | `purple` | `(4.0, 5.0)` |
| 3 | `green` | `(5.5, 8.0)` |

必须按蓝→紫→绿顺序飞到对应区域上方并显式执行 `DROP`。场地末端是一条宽1.5m的走廊，其中有两面带0.8m随机门洞的墙；无人机必须低空穿门。

仿真任务最高100分：自主起飞10分、无碰撞避障10分、三个投放区各15分、两扇门各10分、准确降落15分。任务分用于实验指标，不改变本综合题40分的课程评分表。

### 仿真中公开的测量与计分假设

- 无人机按水平半径0.19m进行连续碰撞检测；局部栅格分辨率0.1m、感知半径3m，未知区域值为 `-1`，占据格同时提供障碍高度。
- 三个投放区通过 `Observation.mission.delivery_zones` 给出，随机障碍距离各区域中心至少0.8m。
- 有效投放要求：当前货物顺序正确，X/Y均落在中心±0.25m内，飞行高度在0.8–1.5m，并调用匹配颜色的 `Action.drop(label)`。不满足条件时拒绝投放且货物保留。
- 穿门速度不超过0.5m/s、高度不超过1.45m；干净通过得10分，擦边通过得5分。
- 降落中心误差不超过0.45m得15分，0.45–0.70m得5分。
- 树木和纸箱接触不会立即终止仿真，但取消无碰撞避障分；边界、走廊墙或门墙碰撞立即终止。

## 二、任务要求

### 1. 系统设计与 EGO-Planner 理解（8分）

1. 画出定位、感知、建图、投放点管理、任务决策、局部规划、轨迹控制、投放和安全模块的数据流，说明主要输入、输出与故障传播路径。
2. 用不超过400字回答：
   - EGO-Planner解决自主无人机系统中的哪一层问题；
   - ESDF-free、梯度优化、B样条局部轨迹分别是什么意思；
   - 它需要什么输入、输出什么；
   - 为什么它不能独立完成本题。
3. 分析至少三个风险，例如窄门膨胀后无路、5m高度边界违规、定位或深度失效、局部最优、错误投放顺序等。

### 2. 自主任务实操（20分）

只能修改 `student_controller.py` 和 `configs/adaptive.json`。实现以下状态：

```text
TAKEOFF → APPROACH_BLUE → DROP_BLUE
        → APPROACH_PURPLE → DROP_PURPLE
        → APPROACH_GREEN → DROP_GREEN
        → CORRIDOR_ENTRY → DOOR_SCAN → DOOR_PASS → LAND
                         ↘ HOVER / ABORT ↗
```

必须满足：

- 从 `observation.mission.delivery_zones` 读取公开投放点，按蓝、紫、绿顺序导航和投放；不得通过导入或反射读取仿真器内部状态。
- 只有进入对应0.5m方形且高度处于0.8–1.5m时才能执行匹配的 `DROP`；被拒绝后必须重新接近，不能直接跳过。
- 正常飞行阶段至少每0.5秒调用一次局部规划器，并处理 `NO_PATH`、目标变化和轨迹失效。
- 从局部占据栅格计算门洞中心，使用门前、门中、门后三段航点；穿门速度不得超过0.5m/s、高度不得超过1.45m。
- 里程计无效或地图年龄超过0.5秒时立即悬停；连续五帧有效后才能恢复。失效超过两秒时安全降落或终止。
- 不得修改、复制或从 Python 反射访问 `Scenario.obstacles`、`Scenario.gates` 等真实状态。固定投放区和固定场地结构应从 `MissionSpec` 获取。

`EgoLikePlanner` 是稳定的教学接口，不是真实 EGO-Planner。飞行、碰撞和回放使用三维坐标，但学生地图仍是二维局部栅格＋障碍高度图，不要求实现三维体素规划。

### 3. 实验验证（8分）

对同一组20个公开场景分别运行：

1. `baseline`：1.0m/s、膨胀0.25m、高度上限5.0m；
2. `conservative`：0.5m/s、膨胀0.30m、高度上限1.45m；
3. `adaptive`：你的分阶段自适应方案。

执行：

```bash
python3 evaluate.py --controller student
```

报告平均任务得分、完整任务成功率、碰撞率、两门通过率、平均投放中心误差、平均任务时间、控制器和规划器计算时间。放入评测生成的对比图，并选择一次失败记录说明根因和改进方式。

### 4. 提交与可复现性（4分）

提交可运行代码、4页以内PDF报告、3分钟以内带解说录屏和填写完整的 `sources.md`。允许查阅资料和使用AI，但必须声明具体用途。无法解释所提交代码、伪造运行数据或未声明地整段复制，相关部分不得分。

## 三、公开接口摘要

```python
class StudentController:
    def __init__(self, profile: ControllerProfile): ...
    def reset(self) -> None: ...
    def step(self, observation: Observation, planner: EgoLikePlanner) -> Action: ...
```

- `Observation`：时间、带噪位姿、速度、里程计状态、地图年龄、0.1m局部栅格、剩余货物、上一规划状态和 `MissionSpec`。兼容字段 `detections` 在本题中始终为空。
- `MissionSpec.delivery_zones`：按规定顺序提供三个 `DeliveryZone(label, x, y, size)`。
- `planner.plan(start, grid, goal, params)`：返回 `PlanResult(status, trajectory, ...)`。
- `Action.follow(result)`：提交新轨迹；`Action.follow()`：继续执行当前轨迹。
- 其他动作：`Action.hover()`、`Action.drop(label)`、`Action.land()`、`Action.abort()`。

完整字段以 `uav_exam/types.py` 的类型定义为准。
