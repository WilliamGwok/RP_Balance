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
[More Test Videos](https://github.com/WilliamGwok/RP_Balance/tree/main/Test_Video)
#### 仿真测试


https://github.com/user-attachments/assets/7b33e5dd-0533-4bbb-9497-3062cb68b34d

#### 实车测试
https://github.com/user-attachments/assets/42d3e697-0503-4a66-8977-5c767b99fcbf

## 实车调试细节

| 文件名 | 文件链接 |
|---|---|
| 关于LQR控制器的参数整定技巧 | [NOTE_1](https://github.com/WilliamGwok/RP_Balance/blob/main/%E5%AE%9E%E8%BD%A6%E8%B0%83%E8%AF%95%E7%BB%86%E8%8A%82/%E5%85%B3%E4%BA%8ELQR%E6%8E%A7%E5%88%B6%E5%99%A8%E7%9A%84%E5%8F%82%E6%95%B0%E6%95%B4%E5%AE%9A%E6%8A%80%E5%B7%A7.md) |
| 驱动轮电机在连杆转动时定转相对移动导致位移计算错误的分析 | [NOTE_2](https://github.com/WilliamGwok/RP_Balance/blob/main/%E5%AE%9E%E8%BD%A6%E8%B0%83%E8%AF%95%E7%BB%86%E8%8A%82/%E5%85%B3%E4%BA%8E%E9%A9%B1%E5%8A%A8%E8%BD%AE%E7%94%B5%E6%9C%BA%E5%9C%A8%E8%BF%9E%E6%9D%86%E8%BD%AC%E5%8A%A8%E6%97%B6%E5%AE%9A%E8%BD%AC%E7%9B%B8%E5%AF%B9%E7%A7%BB%E5%8A%A8%E5%AF%BC%E8%87%B4%E4%BD%8D%E7%A7%BB%E8%AE%A1%E7%AE%97%E9%94%99%E8%AF%AF%E7%9A%84%E9%97%AE%E9%A2%98.md) |
| NULL | To be updated |

## 调车血泪史

##### 贯穿始终的烧保护
24年因为实验室硬件断代，很长的一段时间里电机保护是用一个烧一个，有几次甚至烧出明火了。后面是老队员帮忙才有了较为稳定的保护。

<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0%E5%BD%95/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_20241025215802.jpg" width="260px">
<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0%E5%BD%95/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_202410252158021.jpg" width="310px">









