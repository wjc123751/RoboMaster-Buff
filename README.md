# RoboMaster-Buff
本科竞赛
# RoboMaster 视觉识别与自动瞄准系统

![RoboMaster Logo](docs/rm_logo.png)

## 项目概述
本项目为RoboMaster机甲大师赛中的能量机关（大符）自动击打解决方案，集成YOLO目标检测、TensorRT加速、PnP测距、运动预测等算法，实现高效稳定的自动瞄准系统。系统实时计算目标角度并通过通信模块发送给机器人执行机构。

## 主要功能
- 🎯 **YOLOv5 目标检测**  
  使用改进的YOLOv5模型实现高精度能量机关扇叶识别
- ⚡ **TensorRT 加速推理**  
  实现检测速度提升300%+，640x640输入可达100+ FPS
- 📏 **PnP 距离解算**  
  基于solvePnP算法实现精确距离和姿态估计
- 🔮 **运动轨迹预测**  
  卡尔曼滤波+运动学模型预测目标运动轨迹
- 🎮 **双轴控制算法**  
  实时计算pitch和yaw角度控制量
- 📡 **串口通信模块**  
  支持CRC校验的稳定机器人通信协议
- ⏱ **帧差法测速**  
  动态计算能量机关旋转速度

## 环境依赖
- Ubuntu 18.04/20.04
- CUDA 11.1+
- TensorRT 8.0+
- OpenCV 4.5+ with contrib
- PyTorch 1.7+
- Python 3.8+

## 快速开始

### 安装步骤
```bash
# 克隆仓库
git clone https://github.com/wjc123751/RoboMaster-Buff.git
cd RoboMaster-Buff

# 安装依赖
pip install -r requirements.txt

# 转换YOLO模型到TensorRT
python tools/export_trt.py --weights best.pt --img-size 640

# 运行程序
# 主程序启动
python main.py \
    --camera-config configs/camera_params.yaml \
    --robot-config configs/robot_settings.json \
    --trt-model engine/best_fp16.engine

# 代码结构
├─src
│  ├─calculation
│  ├─HK_camera
│  ├─include
│  │  ├─my
│  │  └─yolo
│  ├─usart
│  └─yolo
└─weight

