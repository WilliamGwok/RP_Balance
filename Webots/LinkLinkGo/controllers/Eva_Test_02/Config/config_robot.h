#ifndef __CONFIG_ROBOT_H
#define __CONFIG_ROBOT_H

#define WHEEL_RADIUS             0.06
//杆长
#define l1    0.15
#define l2    0.27
#define l3    0.27
#define l4    0.15
#define l5    0.15

//各杆质心系数
#define l1_cen 0.5
#define l2_cen 0.5
#define l3_cen 0.5
#define l4_cen 0.5
#define l5_cen 0.5

//各杆质量
#define m_l1 0.52
#define m_l2 0.2
#define m_l3 0.2
#define m_l4 0.52

//单腿质量，四杆总和
#define m_l (m_l1 + m_l2 + m_l3 + m_l4)

//机体质量
#define mb 10.0f


//整车旋转半径
#define Rl 0.18

/*腿长平均值目标值*/
#define TAR_LEG_LENGTH_INITIAL   0.21
#define TAR_ROLL_INITIAL         0
#define LEG_LENGTH_MAX           0.34
#define LEG_LENGTH_MIN           0.11

/*离地支持力，单位为牛*/
#define OFF_GROUND_SUPPORT       30

#endif