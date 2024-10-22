#ifndef __VIRTUAL_LEG_H
#define __VIRTUAL_LEG_H

#include "config_robot.h"
#include "config_environment.h"
#include "dev_imu.h"
#include "dev_encoder.h"
#include "dev_motor.h"
#include "five_link.h"

#include "state_var.h"

typedef enum Leg_List
{
  L_Leg = 0,
  
  R_Leg,
  
  Leg_Num,
}Leg_List_e;

typedef struct Virtual_Leg_struct_t
{
  double l_target;

  double F_gravity;
  
  double F_inertial;
  
  double F_roll;
  
  double F;
  
  double Fbl;

  pid_t* length_pid;
  
  pid_t* roll_pid;

}Virtual_Leg_t;

extern Virtual_Leg_t My_Virtual_Leg[Leg_Num];

void My_Virtual_Leg_Force_Cal(void);

#endif


