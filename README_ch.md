# SLAMForge — 单目视觉 SLAM 与稠密重建


[![License: GPL v3](https://img.shields.io/badge/License-GPL--3.0--only-blue.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue?logo=c%2B%2B)](https://isocpp.org/)
[![Docker](https://img.shields.io/badge/Docker-latest-2496ED?logo=docker)](https://github.com/JackXing875/SLAMForge/pkgs/container/slamforge)

[English Version](README.md)

**SLAMForge** 是一个使用 C++20 编写的**单目视觉 SLAM 与稠密重建**系统。它先通过
几何 SLAM 估计 6-DoF 相机运动，再将本地推理的深度融合为彩色连续表面；跟踪与建图
架构参考 **ORB-SLAM3**。

---

## 快速开始

### 桌面 Beta——无需开发环境

从 [GitHub Releases](https://github.com/JackXing875/SLAMForge/releases/tag/v3.2.0-beta.1)
下载 `SLAMForge Desktop 3.2.0-beta.1`：

- **Windows x64：**解压 ZIP，然后双击 `SLAMForge Desktop.exe`。
- **Linux x86_64：**给 AppImage 添加执行权限后直接启动。

将视频拖入窗口，选择与该相机匹配的标定 YAML，指定结果目录并开始建图。所有处理都在
本地完成；软件会显示最终稠密彩色表面和相机轨迹，并导出 `map.ply`、
`sparse_map.ply`、`trajectory.txt` 和 `run.log`。

> 单目 SLAM 无法确定绝对尺度，并且依赖准确的相机内参。桌面应用不会从任意视频中
> 自动推断标定参数。

### 开发者和容器用法

```bash
# Docker 构建（无需安装任何依赖）
docker build -t slamforge -f docker/Dockerfile .
docker run --rm -v /path/to/images:/images -v $PWD/output:/output \
    slamforge run --config /opt/slamforge/config/kitti.yaml --input /images --output /output/traj.txt

# 本地构建 (Ubuntu 22.04)
sudo apt-get install -y libopencv-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
./build/apps/slamforge_cli run --config config/kitti.yaml --input /path/to/images

# 评估结果
python3 tools/evaluate_ate.py output/traj.txt groundtruth.txt --plot
```

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                   输入：单目视频流                        │
└──────────────────────┬──────────────────────────────────┘
                       │
         ┌─────────────▼─────────────┐
         │   TRACKING 前端（实时）     │
         │   • ORB 特征提取            │
         │   • 帧间运动估计            │
         │   • 局部地图跟踪            │
         │   • 关键帧决策              │
         └─────────────┬─────────────┘
                       │ 新关键帧
         ┌─────────────▼─────────────┐
         │   LOCAL MAPPING（异步）    │
         │   • 地图点三角化            │
         │   • 局部 BA (Ceres)        │
         │   • 地图点/关键帧剔除       │
         └─────────────┬─────────────┘
                       │
         ┌─────────────▼─────────────┐
         │   LOOP CLOSING（异步）     │
         │   • BoW 回环检测           │
         │   • Sim(3) 几何验证        │
         │   • 位姿图优化 (g2o)       │
         │   • 全局 BA                │
         └─────────────┬─────────────┘
                       │ 最终位姿 + 稀疏深度锚点
         ┌─────────────▼─────────────┐
         │   DENSE RECONSTRUCTION    │
         │   • 本地学习深度推理       │
         │   • 稀疏尺度标定           │
         │   • 多视角一致性过滤       │
         │   • 彩色体素融合           │
         └───────────────────────────┘
```

## 功能特性

| 类别       | 状态 | 描述                                           |
| ---------- | ---- | ---------------------------------------------- |
| 跟踪前端   | ✅    | 双视图初始化、运动模型、局部地图跟踪、重定位   |
| 局部建图   | ✅    | 三角化、局部 BA (Ceres)、地图点/关键帧剔除     |
| 稠密建图   | 🧪    | 本地深度推理、稀疏尺度标定、多视角体素融合      |
| 回环闭合   | ✅    | FBOW 词袋、Sim(3) 验证、位姿图 (g2o)、全局 BA  |
| ORB 特征   | ✅    | 多尺度金字塔、四叉树均匀分布、自适应阈值       |
| 配置系统   | ✅    | YAML 配置 + 模式校验                           |
| 桌面 Beta  | 🧪    | Windows/Linux 视频工作流、结果查看与 PLY 导出  |
| CLI 工具   | ✅    | `run`、`eval`、`benchmark` 子命令              |
| ROS2 节点  | ✅    | 实时 SLAM，发布 PoseStamped、PointCloud2、TF   |
| Python API | ✅    | pybind11 绑定，numpy 互操作                    |
| 评估工具   | ✅    | ATE、RPE、轨迹可视化、批量基准测试             |
| Docker     | ✅    | 一键构建运行                                   |
| 文档       | ✅    | Doxygen API 文档、架构说明、快速开始、调优指南 |
| 单元测试   | ✅    | 20 项核心/CLI 测试及桌面结果查看器冒烟测试     |
| CI/CD      | ✅    | GitHub Actions: 构建、测试、lint、文档、Docker |
| 基准测试   | ✅    | Google Benchmark: ORB、PnP、三角化             |

## 发布回归基线

发布门禁包含对一段已标定、已矫正的 4757 帧 TUM MonoVO 序列进行两次完整运行；两次输出的
轨迹与稀疏地图逐字节一致。

| 位姿数 | 初始化后丢失 | 关键帧 | 刚性回环 | 稠密点数 | 部分真值 ATE RMSE |
| ------ | ------------ | ------ | -------- | -------- | ----------------- |
| 4556   | 0            | 598    | 2        | 886813   | 1.554 m           |

ATE 是在 954 个具有有限真值的位姿上进行全局 Sim(3) 对齐后的结果。这只是单个序列的回归
数据，不代表广泛数据集精度，也不构成测绘级精度声明。

## 技术栈

| 层次     | 技术                         | 用途                     |
| -------- | ---------------------------- | ------------------------ |
| 语言     | C++20                        | 核心引擎                 |
| 构建     | CMake 3.20+                  | 构建系统                 |
| 线性代数 | Eigen 3.3+ + Sophus          | 矩阵运算 + 李群李代数    |
| 视觉     | OpenCV 4.x                   | ORB、PnP、对极几何       |
| 稠密深度 | Depth Anything V2 Small + ONNX Runtime | 本地表面深度推理 |
| 局部优化 | Ceres Solver                 | 局部光束法平差           |
| 位姿图   | g2o                          | 回环闭合位姿图优化       |
| 词袋     | FBOW                         | DBoW2 兼容的视觉位置识别 |
| 日志     | spdlog                       | 结构化异步日志           |
| 配置     | yaml-cpp                     | 运行时参数               |
| CLI      | CLI11                        | 命令行参数解析           |
| 绑定     | pybind11                     | C++ → Python 桥接        |
| 测试     | GoogleTest, Google Benchmark | 单元测试 + 微基准        |
| 容器     | Docker                       | 可复现环境               |

## CLI 使用

```bash
# 对图片目录运行 SLAM
slamforge_cli run --config config/kitti.yaml --input /data/images --output traj.txt

# 对 TUM/EuRoC 图片序列保留原始时间戳
slamforge_cli run --config config/euroc.yaml --input /data/images \
    --timestamps /data/timestamps.txt --output traj.txt

# 对视频运行 SLAM，并在桌面发布包中生成稠密彩色地图
slamforge_cli run --config config/kitti.yaml --input /data/video.mp4 --fps 30 \
    --map-output sparse_map.ply --dense-output map.ply

# 评估轨迹精度
slamforge_cli eval --estimated traj.txt --groundtruth gt.txt --format kitti

# 批量基准测试
slamforge_cli benchmark --dataset-dir /data/kitti --config config/kitti.yaml
```

## Python API

```python
import numpy as np
import slamforge

cfg = slamforge.load_config("config/kitti.yaml")
camera = slamforge.Camera(cfg.camera)
tracker = slamforge.Tracker(camera, cfg.tracking, cfg.orb)

for frame in frames:
    pose = tracker.track(frame, timestamp)
    if pose is not None:
        print(f"位置: {pose.position}")

map_ = tracker.get_map()
print(f"关键帧: {map_.keyframe_count}, 地图点: {map_.map_point_count}")
```

## 从源码构建

```bash
sudo apt-get install -y build-essential cmake libopencv-dev libeigen3-dev \
    libspdlog-dev libyaml-cpp-dev libceres-dev

cmake -B build -DCMAKE_BUILD_TYPE=Release -DSLAMFORGE_BUILD_TESTS=ON
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure
```

## 文档

- [快速开始指南](docs/quick_start.md)
- [架构概览](docs/architecture.md)
- [调优指南](docs/tuning_guide.md)
- [桌面 Beta 指南](docs/desktop.md)
- [API 文档](https://JackXing875.github.io/SLAMForge/)

## 参与贡献

详见 [CONTRIBUTING.md](CONTRIBUTING.md)

## 开源协议

本项目采用 GNU General Public License v3.0 only（GPL-3.0-only）许可证。详见 [LICENSE](LICENSE)。

---

*技术栈: C++20 • Eigen • Sophus • OpenCV • Ceres • g2o • FBOW*
