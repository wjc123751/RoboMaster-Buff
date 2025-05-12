# RoboMaster-Buff
本科竞赛
# RoboMaster 机甲大师大符自动击打系统
![RoboMaster](https://img.shields.io/badge/RoboMaster-ICRA_AI_Challenge-red) 
![TensorRT](https://img.shields.io/badge/Inference-TensorRT-76B900) 
![YOLOv5](https://img.shields.io/badge/Detection-YOLOv5-00FFFF)

基于视觉感知的旋转大符动态击打系统，实现实时目标检测、轨迹预测与双轴云台控制。

## 🚀 核心功能
### 1. 大符识别模块
- **YOLOv5s-TensorRT**：416x416输入，150FPS实时检测  
- **PnP定位算法**：亚像素角点检测，相机到大符距离误差<3cm  
- **动态补偿机制**：卡尔曼滤波预测未来100ms轨迹

### 2. 状态感知模块
- **旋转速度估计**：帧间差分法+光流校验，精度±5°/s  
- **运动学解算**：考虑弹丸初速（18m/s）和重力补偿

### 3. 控制模块
- **双轴控制**：Pitch-Yaw双闭环PID控制器  
- **通信协议**：UDP协议传输控制指令（20ms周期）  
- **安全机制**：射击禁区检测与过热保护

# RoboMaster 机甲大师大符自动击打系统
![RoboMaster](https://img.shields.io/badge/RoboMaster-ICRA_AI_Challenge-red) 
![TensorRT](https://img.shields.io/badge/Inference-TensorRT-76B900) 
![YOLOv5](https://img.shields.io/badge/Detection-YOLOv5-00FFFF)

基于视觉感知的旋转大符动态击打系统，实现实时目标检测、轨迹预测与双轴云台控制。

## 🚀 核心功能
### 1. 大符识别模块
- **YOLOv5s-TensorRT**：416x416输入，150FPS实时检测  
- **PnP定位算法**：亚像素角点检测，相机到大符距离误差<3cm  
- **动态补偿机制**：卡尔曼滤波预测未来100ms轨迹

### 2. 状态感知模块
- **旋转速度估计**：帧间差分法+光流校验，精度±5°/s  
- **运动学解算**：考虑弹丸初速（18m/s）和重力补偿

### 3. 控制模块
- **双轴控制**：Pitch-Yaw双闭环PID控制器  
- **通信协议**：UDP协议传输控制指令（20ms周期）  
- **安全机制**：射击禁区检测与过热保护

硬件要求
组件	 规格要求
计算平台	 Jetson Xavier NX
相机	 全局快门相机（MV-SUA133GC-T）
云台	 6020电机+高精度编码器

# 环境配置
# 安装TensorRT加速环境
sudo apt-get install python3-libnvinfer-dev
pip install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cu113
# 模型部署
# 转换YOLOv5模型到TensorRT
python export.py --weights runs/train/exp/weights/best.pt --include engine --device 0 --half
# 运行系统
# 启动主控程序（带参数调试界面）
python main.py \
    --weights assets/buff.trt \
    --calibration calibration.yaml \

# 项目结构
├─src
│  ├─build
│  │  └─CMakeFiles
│  │      ├─3.10.2
│  │      │  ├─CompilerIdC
│  │      │  │  └─tmp
│  │      │  └─CompilerIdCXX
│  │      │      └─tmp
│  │      ├─buffR.dir
│  │      │  ├─calculation
│  │      │  ├─HK_camera
│  │      │  ├─usart
│  │      │  └─yolo
│  │      ├─CMakeTmp
│  │      └─myplugins.dir
│  │          └─yolo
│  ├─calculation
│  ├─HK_camera
│  ├─include
│  │  ├─my
│  │  └─yolo
│  ├─usart
│  └─yolo
└─weight
