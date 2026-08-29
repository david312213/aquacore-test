# 三维自主快递无人机接口编程考核（100分）

本题是一项独立的三维无人机自主任务编程考核。你需要在教师提供的仿真环境中补全控制器，仅使用公开接口完成投放、避障、穿门和降落。程序启动后不得接受人工飞行指令。

## 一、整体任务

### 1.1 你要完成什么

你的任务是在教师提供的10m×10m×5m三维仿真场地中，完成一套能够独立运行的无人机自主控制程序。无人机从规定起点出发，携带蓝、紫、绿三件货物，必须在600秒内自主起飞，并依次飞到三个公开坐标对应的投放区上方，在满足水平范围和0.8—1.5m高度要求时执行正确的投放动作。完成三次投放后，无人机需要进入右侧走廊，根据实时局部占据栅格判断未知障碍物和两扇随机门洞的位置，在不碰撞、不越界、不从障碍物上方绕行的前提下依次穿过两扇门，并在规定速度和高度内抵达终点降落区，最后自主降落。整个任务过程中，程序只能读取Observation、MissionSpec以及局部规划器返回的公开信息，自行完成目标选择、规划调用、轨迹跟随、任务顺序判断和异常处理，不得读取完整场景、隐藏门位或接受人工控制。遇到无路可走、新障碍、轨迹失效、里程计无效或地图过期时，控制器必须及时悬停、重新规划或安全终止；传感器恢复后也要满足规定的连续有效帧数才能继续飞行。最终程序应能在不同公开和隐藏场景中重复运行，并以顺序正确、投放有效、避障安全、穿门合规和降落准确作为任务完成标准。

### 1.2 场地与基本参数

| 项目 | 公开参数 |
|---|---|
| 场地与时间 | `10m × 10m × 5m`；单场 `600s`；仿真步长 `0.1s` |
| 无人机安全尺寸 | 保护外径 `0.38m`；碰撞检测半径 `0.19m` |
| 局部地图 | 感知半径 `3.0m`；栅格分辨率 `0.1m` |
| 起降位置 | 起点 `(0.8, 5.0, 0.0)`；降落区中心 `(9.35, 5.0, 0.0)` |
| 障碍高度 | 箱子、树木、边界、走廊墙和门墙均高 `5m` |

无人机高度超过5m会立即终止任务，因此不能通过升高绕过障碍物或门墙。

### 1.3 三个投放区

投放区是地面上边长0.5m、边缘与坐标轴平行的正方形，坐标表示中心。

| 顺序与货物 | 中心坐标 `(x, y)` | 有效水平范围 |
|---|---|---|
| 1、`blue` | `(2.0, 2.0)` | 中心X/Y方向各 `±0.25m` |
| 2、`purple` | `(4.0, 5.0)` | 中心X/Y方向各 `±0.25m` |
| 3、`green` | `(5.5, 8.0)` | 中心X/Y方向各 `±0.25m` |

有效投放必须同时满足：货物和顺序正确；X/Y在对应正方形边界内；`0.8m ≤ z ≤ 1.5m`；显式返回 `Action.drop(label)`。失败时货物保留，可通过 `remaining_payloads` 或 `last_event` 确认结果。

### 1.4 走廊、窄门与降落

| 项目 | 公开信息或限制 |
|---|---|
| 走廊 | `x∈[6.5, 9.85]，y∈[4.25, 5.75]` |
| 两扇门 | 门墙 `x=7.4m`、`x=8.4m`；门洞宽 `0.8m`；中心Y坐标每场随机且不提供 |
| 穿门 | 速度不超过 `0.5m/s`；高度不超过 `1.45m` |
| 降落 | 先水平到达降落区上方，再返回 `Action.land()` |

## 二、学生可以修改什么

只允许修改并最终提交两个文件：

| 文件 | 需要完成的内容 |
|---|---|
| `student_controller.py` | 任务决策、目标选择、规划调用、投放、穿门、降落和异常处理 |
| `configs/adaptive.json` | 最大速度、最大加速度、安全距离、飞行高度和规划视野等参数 |

必须保留接口：

```python
class StudentController:
    def __init__(self, profile: ControllerProfile) -> None: ...
    def reset(self) -> None: ...
    def step(self, observation: Observation,
             planner: EgoLikePlanner) -> Action: ...
```

每个新场景开始前调用一次 `reset()`，每0.1秒调用一次 `step()`。跨帧状态应保存在控制器实例中，且 `step()` 每次都必须返回合法 `Action`。未处理异常会使当前场景按 `ABORT` 结束。

除上述两个文件外，任何修改在教师评分环境中都不会生效。最终程序必须自主完成起飞、顺序投放、局部避障、两扇随机窄门、故障安全响应和准确降落；题目不规定内部状态名称，也不提供具体实现算法。

## 三、怎么给分

### 3.1 单场100分

每个场景独立产生 `raw_score`。已获得的项目分在超时、主动终止或碰撞终止后保留，未完成项目不得分。

| 计分项目 | 满分 | 判定条件 |
|---|---:|---|
| 自主起飞 | 10 | 保持 `z>1.0m` 连续10秒 |
| 无碰撞避障 | 10 | 到达走廊入口前未接触普通障碍，且没有从上方飞越 |
| 蓝色区域投放 | 15 | 顺序、区域和高度正确，投放 `blue` |
| 紫色区域投放 | 15 | 顺序、区域和高度正确，投放 `purple` |
| 绿色区域投放 | 15 | 顺序、区域和高度正确，投放 `green` |
| 第一扇门 | 10 | 满足高度/速度限制并干净通过；擦边通过得5分 |
| 第二扇门 | 10 | 满足高度/速度限制并干净通过；擦边通过得5分 |
| 准确降落 | 15 | 中心水平误差≤0.45m；误差 `(0.45, 0.70]m` 得5分 |
| **合计** | **100** | `10 + 10 + 45 + 20 + 15` |

接触普通树木或纸箱会失去“无碰撞避障”10分；碰撞边界、走廊墙或门墙会立即终止。门洞保护圆净间距不足0.05m但未碰撞时，记为擦边通过。

### 3.2 最终成绩

公开20个场景只用于练习。教师使用20个未公开但类型和接口一致的场景评分：

```text
最终成绩 = round(sum(raw_score_i) / 20, 1)
```

成绩范围为0.0–100.0，不按班级排名，不额外奖励竞速。

### 3.3 违规和异常后果

- 高度超过5m、飞出边界或碰撞关键墙体：当前场景立即终止。
- 控制器无法加载、接口签名变化或依赖缺失：受影响的隐藏场景记0分。
- 修改规定外文件，或读取 `teacher/`、`Scenario.obstacles`、`Scenario.gates`、隐藏用例等非公开数据：整份提交无效。禁止反射、文件扫描、Monkey Patch、硬编码隐藏场景或任何人工/外部进程控制。

## 四、接口怎么接

### 4.1 每一帧的数据流

1. 仿真器生成当前帧 `Observation`，其中只包含允许学生读取的飞行状态、传感器状态、局部地图和任务信息。
2. 仿真器调用 `StudentController.step(observation, planner)`，控制器读取本帧公开数据并决定下一步动作。
3. 控制器需要生成或更新飞行轨迹时，可以调用 `planner.plan(start, grid, goal, params)`，并根据返回的 `PlanResult` 处理成功、无路或输入无效情况。
4. 控制器返回一个合法 `Action`，仿真器执行该动作、更新飞行状态与得分，再生成下一帧 `Observation`。

以上过程每0.1秒循环一次。控制器可以保存跨帧状态，但每次 `step()` 都必须在规定接口内返回动作。

### 4.2 最小接口骨架

下例只展示接线方式，不包含任务状态、目标选择、穿门或故障恢复答案。

```python
class StudentController:
    def __init__(self, profile):
        self.profile = profile
        self.reset()
    def reset(self):
        pass  # 清空跨场景状态
    def step(self, observation, planner):
        goal = Pose(goal_x, goal_y, goal_z)  # 由学生决定
        params = self.profile.planner_params(door=False)
        result = planner.plan(observation.pose, observation.local_grid,
                              goal, params)
        if result.status is PlanStatus.OK:
            return Action.follow(result)
        return Action.hover(result.message)
```

### 4.3 `Observation`：六组输入

| 输入组 | 对应字段 | 含义 |
|---|---|---|
| 飞行状态 | `time`、`pose`、`velocity` | 时间、三维位置/朝向和速度 |
| 传感器状态 | `odom_valid`、`map_age` | 里程计是否有效、地图更新年龄 |
| 局部地图 | `local_grid` | 附近占据栅格与障碍高度 |
| 公开任务信息 | `mission` | 投放/起降区、走廊、门墙X坐标和场地限制 |
| 货物与规划状态 | `remaining_payloads`、`planner_status` | 剩余货物与上次规划状态 |
| 最近事件 | `last_event` | 投放、过门、碰撞等任务反馈 |

`detections` 是兼容旧控制器的保留字段，本题始终为空。`local_grid.occupancy` 中 `-1/0/1` 分别表示未知/空闲/占据，`heights[row, col]` 是已观测障碍高度；可使用 `world_to_cell()`、`cell_to_world()` 和 `sample()`。`mission` 不包含随机门洞Y坐标。

### 4.4 局部规划器

```python
result = planner.plan(start, grid, goal, params)
```

| 内容 | 含义 |
|---|---|
| 输入 | `start`当前位姿，`grid`局部地图，`goal`三维局部目标，`params`速度/加速度/安全距离/高度/视野 |
| `PlanStatus.OK` | 生成可执行轨迹，但不代表已到达最终任务目标 |
| `PlanStatus.NO_PATH` | 当前局部地图和参数下没有找到路径 |
| `PlanStatus.INVALID_INPUT` | 目标或规划参数不合法 |

可用 `self.profile.planner_params(door=False)` 取得正常参数，用 `door=True` 取得穿门受限参数。只有 `OK` 结果可传给 `Action.follow(result)`。

### 4.5 五种合法动作

| 动作 | 调用方式 | 作用 |
|---|---|---|
| `FOLLOW_TRAJECTORY` | `Action.follow(result)` 或 `Action.follow()` | 提交新轨迹，或不替换轨迹继续执行 |
| `HOVER` | `Action.hover(reason)` | 清除当前轨迹并保持位置 |
| `DROP` | `Action.drop(label)` | 尝试投放指定标签货物 |
| `LAND` | `Action.land(reason)` | 在当前X/Y位置垂直下降 |
| `ABORT` | `Action.abort(reason)` | 主动安全终止当前场景 |

禁止绕过 `Action` 接口直接修改位姿、速度、得分、货物或仿真时间。

## 五、必须满足的验收要求

- 按蓝、紫、绿顺序，在各自0.5m正方形内且 `0.8m≤z≤1.5m` 显式投放；最后到达降落区上方才能降落。
- 仅根据局部地图处理未知障碍和门洞；安全距离不小于0.19m，不得升高越障。
- 正常飞行阶段相邻两次 `planner.plan()` 调用间隔不得超过0.5秒，并处理 `NO_PATH`、`INVALID_INPUT`、新障碍和轨迹失效。
- 穿门时速度不超过0.5m/s、高度不超过1.45m，且两扇门必须按顺序通过。
- `odom_valid=False` 或 `map_age>0.5` 时当帧必须返回 `HOVER`、`LAND` 或 `ABORT`，不得继续飞行或投放；连续5帧有效后才能恢复，异常超过2秒必须 `LAND` 或 `ABORT`。

## 六、运行和提交

### 6.1 Windows 10/11

安装64位Miniconda，安装完成后打开Anaconda Prompt。项目统一使用名为 `aquacore-test` 的Conda环境，Python版本由 `environment.yml` 固定为3.11。进入项目目录后首次运行：

```powershell
.\setup_windows.bat
```

运行一个公开场景并查看三维回放：

```powershell
conda run -n aquacore-test python .\run_sim.py --controller student --seed 1001 --animate --events
```

运行全部20个公开场景：

```powershell
conda run -n aquacore-test python .\evaluate.py --controller student --profiles adaptive
```

评测数据保存到 `results/evaluation.json`，对比图保存到 `results/comparison.png`。

### 6.2 提交清单

提交 `学号_姓名.zip`，压缩包中只包含：

```text
student_controller.py
configs/adaptive.json
```

正式评测离线运行。提交前逐项确认：

| 序号 | 确认内容 | 序号 | 确认内容 |
|---:|---|---:|---|
| 1 | 覆盖到全新学生仓库后可直接运行 | 4 | 不依赖绝对路径、额外文件、网络、ROS或人工输入 |
| 2 | 类名、构造函数、`reset()` 和 `step()` 签名未改变 | 5 | 未使用 `environment.yml` 之外的第三方库 |
| 3 | 20个公开场景可完整运行并生成结果 | — | — |
