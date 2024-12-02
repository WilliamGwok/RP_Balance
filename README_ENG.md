# RP_Balance

## Introduction

<div align="center">
<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0%E5%BD%95/2024_1.jpg" width="710px">
</div>
<br>
<p align="center">This project was completed in the RobotPilots Lab at Shenzhen University during 2023-2024. It is intended for new lab members as a learning reference and will be continuously updated through the end of 2024. If there are any errors, feedback is welcome.</p>

## Modeling and Controller Design

| File Name | Description | File Link |
|---|---|---|
| HEU_Model | Mathematical modeling and controller design based on the Harbin Engineering model | [HEU](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/HEU_Model) |
| SJTU_Model | Full-body modeling and controller design, five-link solution, gain matrix fitting, AB matrix fitting | [SJTU](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/SJTU_Model) |
| Webots_Data_Estimate | Analyzing simulation data | [DATA](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/Webots_Data_Estimate) |

### Core Code
#### Balancing Chassis Modeling
Formulate and simplify dynamic and kinematic equations, calculate system A, B matrices.

| Code | Description | File Link |
|---|---|---|
| Bot_Dynamics_Part1 | Formulate equations and simplify to obtain A, B matrices (P1~P3) | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Bot_Dynamics_Part1.m) |

#### Five-Link Solution and VMC Control
Solve for angle relationships and virtual forces.

| Code | Description | File Link |
|---|---|---|
| Centroid | Solve for centroid coordinates and moments of inertia of the links | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Centroid.m) |
| Five_Link_Base | Solve for angle relationships and virtual force calculations | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Five_Link_Base.m) |

#### LQR Controller and K Matrix Fitting
The wheeled chassis uses a five-link structure with adjustable leg lengths. As the leg length changes, the chassis model changes, requiring the fitting coefficients of the K matrix relative to leg length.

| Code | Description | File Link |
|---|---|---|
| Master | Main fitting function | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/K_L_Fitting/Master.m) |
| Func_Cal_K | K matrix calculations | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/K_L_Fitting/Func_Cal_K.m) |

#### System A, B Matrix Fitting and MPC-Like Control
To address instability when the robot’s drive wheels encounter abrupt resistance (on smooth or uneven surfaces), an MPC-inspired approach is used. It predicts the robot's state at the next moment based on its current state and input, compensating the output torque of the drive wheels for the difference between the predicted and actual state. 

Since A and B matrices also require fitting, this method is implemented accordingly.

## Simulation

<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/Webots/PixPin_2024-10-22_23-50-17.png" width="410px">

[Main Simulation Code](https://github.com/WilliamGwok/RP_Balance/tree/main/Webots/LinkLinkGo/controllers/Eva_Test_02)

| File Name | Description |
|---|---|
| Config | Simulation environment settings, robot mechanical parameters, etc. |
| Device | Devices like motors, encoders, IMU, etc. |
| Model | Robot model hierarchy, including straight-leg models and virtual leg force calculations |
| Algorithm | PID algorithms, five-link solutions |
| Robot | Top-level robot control code |

## Test Videos
[More Test Videos](https://github.com/WilliamGwok/RP_Balance/tree/main/Test_Video)

#### Simulation Tests
https://github.com/user-attachments/assets/7b33e5dd-0533-4bbb-9497-3062cb68b34d

#### Real Robot Tests
https://github.com/user-attachments/assets/42d3e697-0503-4a66-8977-5c767b99fcbf

## Real Robot Debugging Details

| File Name | File Link |
|---|---|
| LQR Controller Parameter Tuning Tips | [NOTE_1](https://github.com/WilliamGwok/RP_Balance/blob/main/%E5%AE%9E%E8%BD%A6%E8%B0%83%E8%AF%95%E7%BB%86%E8%8A%82/%E5%85%B3%E4%BA%8ELQR%E6%8E%A7%E5%88%B6%E5%99%A8%E7%9A%84%E5%8F%82%E6%95%B0%E6%95%B4%E5%AE%9A%E6%8A%80%E5%B7%A7.md) |
| Analysis of Displacement Calculation Errors Due to Relative Movement During Linkage Rotation | [NOTE_2](https://github.com/WilliamGwok/RP_Balance/blob/main/%E5%AE%9E%E8%BD%A6%E8%B0%83%E8%AF%95%E7%BB%86%E8%8A%82/%E5%85%B3%E4%BA%8E%E9%A9%B1%E5%8A%A8%E8%BD%A6%E7%94%B5%E6%9C%BA%E5%9C%A8%E8%BF%9E%E6%9D%86%E8%BD%AC%E5%8A%A8%E6%97%B6%E5%AE%9A%E8%BD%AC%E7%9B%B8%E5%AF%B9%E7%A7%BB%E5%8A%A8%E5%AF%BC%E8%87%B4%E4%BD%8D%E7%A7%BB%E8%AE%A1%E7%AE%97%E9%94%99%E8%AF%AF%E7%9A%84%E9%97%AE%E9%A2%98.md) |
| NULL | To be updated |

## Debugging Challenges

#### Persistent Motor Protection Issues
Due to hardware limitations in 2024, motors were frequently burned during testing, sometimes even catching fire. A more stable protection system was eventually implemented with the help of experienced team members. Additional hardware failures occurred during competitions due to wire insulation damage.

<div align="center">
<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0%E5%BD%95/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_202410252158021.jpg" width="300px">
</div>

#### Motor Damage
Inexperience among new mechanical team members led to issues such as using printed components to mount motors, which deformed during use and caused screws to penetrate motor windings during maintenance.

https://github.com/user-attachments/assets/464f6e4d-6a7e-44c8-b6e9-2862852114d7
