# AquaCore Test

三维自主快递无人机接口编程考核（学生版）

这是面向学生发布的纯 Python 三维无人机接口编程考核，单场满分 100 分。你需要补全控制器，使无人机在未知障碍环境中依次完成蓝、紫、绿三个定点投放，穿过两扇随机窄门并准确降落。

本项目不需要安装 ROS、PX4 或真实 EGO-Planner。仓库中的 `EgoLikePlanner` 是用于教学的局部规划器抽象，不是浙江大学 FAST Lab 的 EGO-Planner 源码、移植版或等价复现。

完整规则可阅读 [ASSIGNMENT.md](ASSIGNMENT.md)，也可以直接下载 Word 版：[终极考核.docx](docs/终极考核.docx)。

参考完整飞行效果：[演示视频](docs/演示视频.mp4)。

## 运行环境

- 统一使用 Conda，环境名称为 `aquacore-test`，Python 版本固定为3.11。
- 推荐：Windows 10/11 + 64位Miniconda。
- 也支持 Ubuntu 22.04/24.04 和 macOS。
- 依赖以 `environment.yml` 为准；无需单独安装Python或创建 `.venv`。

## Windows 下载与安装

先按[官方说明](https://www.anaconda.com/docs/getting-started/miniconda/install/windows-gui-install)安装64位Miniconda，选择仅为当前用户安装。安装完成后打开 **Anaconda Prompt**，执行：

```powershell
git clone https://github.com/david312213/aquacore-test.git
cd aquacore-test
.\setup_windows.bat
```

脚本会根据 `environment.yml` 创建或更新 `aquacore-test` 环境，并自动运行项目测试。如果系统提示无法识别 `git`，请先安装Git for Windows，安装完成后重新打开Anaconda Prompt：

```powershell
winget install --id Git.Git -e
```

## 运行与可视化

运行一个公开场景，并在结束后打开三维回放：

```powershell
conda run -n aquacore-test python .\run_sim.py --controller student --seed 1001 --animate --events
```

生成三维轨迹图片：

```powershell
conda run -n aquacore-test python .\run_sim.py --controller student --seed 1001 --plot artifacts\student.png --events
```

生成 GIF：

```powershell
conda run -n aquacore-test python .\run_sim.py --controller student --seed 1001 --gif artifacts\student.gif --events
```

需要二维俯视图时，在命令末尾增加 `--view 2d`。

## 公开评测

快速运行前三个公开场景：

```powershell
conda run -n aquacore-test python .\evaluate.py --controller student --profiles adaptive --limit 3
```

运行全部 20 个公开场景：

```powershell
conda run -n aquacore-test python .\evaluate.py --controller student --profiles adaptive
```

评测数据保存在 `results/evaluation.json`，对比图保存在 `results/comparison.png`。

## Ubuntu与macOS

安装Miniconda并重新打开终端后执行：

```bash
chmod +x setup_ubuntu.sh
./setup_ubuntu.sh
conda run -n aquacore-test python run_sim.py --controller student --seed 1001 --animate --events
```

## 可以修改和提交的文件

只允许修改：

- `student_controller.py`
- `configs/adaptive.json`

最终提交的压缩包也只应包含这两个文件。其他文件即使被修改，也不会进入正式评分环境。

## 公开任务参数

- 场地：`10m × 10m × 5m`，限时 600 秒。
- 障碍物统一高 5m，禁止从上方飞越。
- 投放顺序：蓝 `(2.0, 2.0)`、紫 `(4.0, 5.0)`、绿 `(5.5, 8.0)`。
- 每个投放区是边长 0.5m 的正方形，投放高度必须为 0.8–1.5m。
- 穿门速度不得超过 0.5m/s，高度不得超过 1.45m。
- 控制器每 0.1 秒接收一次 `Observation`，并返回一个合法 `Action`。

仓库只包含公开场景与学生起始模板，不包含参考控制器、隐藏场景或正式评分脚本。

## 参考资料

- [EGO-Planner 论文](https://arxiv.org/abs/2008.08835)
- [EGO-Planner 官方仓库](https://github.com/ZJU-FAST-Lab/ego-planner)
