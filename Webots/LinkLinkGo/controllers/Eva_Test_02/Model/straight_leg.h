#ifndef __STRAIGHT_LEG_H
#define __STRAIGHT_LEG_H

#include "config_robot.h"
#include "config_environment.h"
#include "dev_imu.h"
#include "dev_encoder.h"
#include "dev_motor.h"
#include "five_link.h"
#include "state_var.h"

typedef struct Straight_Leg_Model_struct_t
{
  double sdr_K[10];
  
  double wheelr_K[10];
  
  double sdl_K[10];
  
  double wheell_K[10];
  
  double K_coefficient[4][60];
  
  double Tp_l;
  
  double Tw_l;
  
  double Tp_r;
  
  double Tw_r;
  
  double s_part;

  double phi_part;

  double thetal_l_part;

  double thetal_r_part;

  double thetab_part;
}Straight_Leg_Model_t;

extern Straight_Leg_Model_t Straight_Leg_Model;

void Straight_Leg_Model_Cal(double pos_tar, double yaw_tar, bool l_off_ground_flag, bool r_off_ground_flag);

#endif


