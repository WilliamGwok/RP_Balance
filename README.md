### English Version Please Click: [ENG_README](https://github.com/WilliamGwok/RP_Balance/blob/main/README_ENG.md)
### 日本語バージョンは: [JAP_README](https://github.com/WilliamGwok/RP_Balance/blob/main/README_JAP.md)


# RP_Balance

## 前言
<div align=center>
<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0%E5%BD%95/2024_1.jpg" width="710px">
</div>
<br>
<p align="center">本项目为本人于2023~2024年在深圳大学RobotPilots实验室完成，供实验室新队员学习参考用，将会在24年末持续更新。若有错误之处，欢迎指出。</p>

<a target="_blank" href="http://mail.qq.com/cgi-bin/qm_share?t=qm_mailme&email=VmRmZGdmb2ZnYG4WMzs3Pzp4JSwjeDMyI3g1OA" style="text-decoration:none;"><img src="http://rescdn.qqmail.com/zh_CN/htmledition/images/function/qm_open/ico_mailme_01.png"/></a>

## 建模与控制器设计

| 文件名 | 功能 | 文件链接 |
|---|---|---|
| HEU_Model | 基于哈工程模型的数学建模与控制器设计 | [HEU](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/HEU_Model) |
| SJTU_Model | 全身建模与控制器设计，五连杆解算，增益矩阵拟合，AB矩阵拟合 | [SJTU](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/SJTU_Model) |
| Webots_Data_Estimate | 用于分析仿真读取的数据 | [DATA](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/Webots_Data_Estimate) |

### 核心代码
#### 平衡底盘建模
列写动力学与运动学方程化简，计算系统A，B矩阵
| 代码 | 功能 | 文件链接 |
|---|---|---|
| Bot_Dynamics_Part1 | 列些方程并化简，得到A,B矩阵（P1~P3） | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Bot_Dynamics_Part1.m) |

#### 五连杆解算和VMC控制
角度关系与虚拟力求解
| 代码 | 功能 | 文件链接 |
|---|---|---|
| Centroid | 对连杆质心坐标，转动惯量的求解 | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Centroid.m) |
| Five_Link_Base | 角度关系解算与虚拟力计算 | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Five_Link_Base.m) |

#### Lqr控制器与K矩阵拟合
该轮足底盘采用五连杆结构，其腿长可变。因此底盘模型会随着腿长变化而改变，需要求得K矩阵关于腿长拟合的系数
| 代码 | 功能 | 文件链接 |
|---|---|---|
| Master | 拟合主函数 | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/K_L_Fitting/Master.m) |
| Func_Cal_K | K矩阵计算 | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/K_L_Fitting/Func_Cal_K.m) |


#### 系统A，B矩阵拟合与类MPC控制思想

为了解决机器人在驱动轮受阻突变（光滑或起伏路面）时失控的问题，参考了MPC控制的思想，通过机器人当前一刻的状态和输入来预测下一刻机器人的状态，根据状态预测与实际的差值对驱动轮输出转矩进行补偿。
与K矩阵需要拟合的原因相同，采用该方法需要对A，B矩阵进行拟合
| 代码 | 功能 | 文件链接 |
|---|---|---|
| A_B_Mater | A,B矩阵拟合主代码 | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/A_B_Fitting/A_B_Master.m) |



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

#### 贯穿始终的烧保护
24年因为实验室硬件断代，很长的一段时间里电机保护是用一个烧一个，有几次甚至烧出明火了。
后面是老队员帮忙才有了较为稳定的保护。国赛还烧碳板，原因是有硅胶线破皮了（即使套了蛇皮管）。

<div align=center>
<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0%E5%BD%95/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_202410252158021.jpg" width="300px">
</div>

#### 电机损坏
因为机械新队员缺乏经验，他在固定电机时夹了块打印件。
这块打印件在调试过程中因电机发热或振动产生形变，导致后续维护的时候螺丝拧进了电机线圈。

https://github.com/user-attachments/assets/464f6e4d-6a7e-44c8-b6e9-2862852114d7

电机损坏后有一边腿无法正常响应

https://github.com/user-attachments/assets/495ed83a-8a68-4ea7-8e44-cc3647adb917








