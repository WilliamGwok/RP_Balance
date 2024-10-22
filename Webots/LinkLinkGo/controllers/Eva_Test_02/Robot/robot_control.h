#ifndef __ROBOT_CONTROL_H
#define __ROBOT_CONTROL_H

#include "config_robot.h"
#include "config_environment.h"
#include "dev_imu.h"
#include "dev_encoder.h"
#include "dev_motor.h"
#include "five_link.h"
#include "dev_all.h"
#include "virtual_leg.h"
#include "straight_leg.h"
#include "state_var.h"

#define DATA_FILE_PATH ("C:\\Users\\Lenovo\\Desktop\\Balance\\MatlabWorks\\Data_Estimate\\My_Data.csv")

typedef struct Robot_Config_struct_t
{
  double front_speed_max;
  
  double spin_speed_max;
}Robot_Config_t;

typedef struct Robot_Measure_struct_t
{
  double F_support;

  bool off_ground;

}Robot_Measure_t;

typedef struct Robot_Target_struct_t
{
  double speed_target;
  
  double position_target;
  
  double spin_target;
  
  double yaw_target;
  
  double leg_length_target;
}Robot_Target_t;

typedef struct Robot_Command_struct_t
{
  bool jump_flag;

  bool data_record_flag;
}Robot_Command_t;

typedef struct Robot_struct_t
{
  Robot_Config_t *config;

  Robot_Measure_t *l_measure;

  Robot_Measure_t *r_measure;

  Robot_Target_t *target;

  Robot_Command_t *command;

}Robot_t;

extern Robot_t My_Robot;

void My_Robot_Init(void);
void My_Robot_Work(void);

#endif