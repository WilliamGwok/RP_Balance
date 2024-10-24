# RP_Balance

## 前言
<div align=center>
<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0%E5%BD%95/2024_1.jpg" width="710px">
</div>

## 建模与控制器设计

| 文件名 | 功能 | 文件链接 |
|---|---|---|
| HEU_Model | 基于哈工程模型的数学建模与控制器设计 | [HEU](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/HEU_Model) |
| SJTU_Model | 全身建模与控制器设计，五连杆解算，增益矩阵拟合，AB矩阵拟合 | [SJTU](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/SJTU_Model) |
| Webots_Data_Estimate | 用于分析仿真读取的数据 | [DATA](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/Webots_Data_Estimate) |

## 仿真

<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/Webots/PixPin_2024-10-22_23-50-17.png" width="410px">

[仿真主要代码](https://github.com/WilliamGwok/RP_Balance/tree/main/Webots/LinkLinkGo/controllers/Eva_Test_02)
| 文件名 | 功能 |
|---|---|
| Config | 仿真环境设置，机器人机械参数设置等 |
| Device | 电机，编码器，imu等设备 |
| Model | 机器人模型分层，包括直腿模型，虚拟腿力计算等 |
| Algorithm | pid算法，五连杆解算 |
| Robot | 机器人顶层控制代码 |

## 测试视频

<video src="https://github.com/WilliamGwok/RP_Balance/blob/main/Test_Video/Real-world_Testing/Slope.mp4" controls="controls" width="500" height="300"></video>








