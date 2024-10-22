#ifndef __DEV_MOTOR_H
#define __DEV_MOTOR_H

#include "config_environment.h"
#include "config_math.h"

typedef enum Motor_List
{
  R_B_Sd_Motor = 0,
  
  R_F_Sd_Motor,
  
  R_Wheel_Motor,
  
  L_B_Sd_Motor,
  
  L_F_Sd_Motor,
  
  L_Wheel_Motor,
  
  Motor_Total_Num,
}Motor_List_e;

typedef struct Motor_struct_t
{
  char name[DEVICE_NAME_LENGTH];
  
  double output_torque;
  
  double distance_now;
  
  double distance_last;
  
  double linear_velocity;
  
  double angular_velocity;

  double torque_measure;
    
}Motor_t;

extern Motor_t My_Motor[Motor_Total_Num];
extern WbDeviceTag Motor[Motor_Total_Num];

void My_Motor_Init(void);
void My_Motor_Data_Update(void);
void My_Motor_Work(void);

#endif


