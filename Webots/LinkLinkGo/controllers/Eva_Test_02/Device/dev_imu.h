#ifndef __DEV_IMU_H
#define __DEV_IMU_H

#include "config_environment.h"

typedef struct Posture_struct_t
{
  double yaw;
  
  double pitch;
  
  double roll;
  
  double yaw_v;
  
  double pitch_v;
  
  double roll_v;
  
  double x_acc;
  
  double y_acc;
  
  double z_acc;
}Posture_t;

extern Posture_t My_Posture;

void My_Imu_Init(void);
void My_Imu_Data_Update(void);

#endif


