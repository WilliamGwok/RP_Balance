#ifndef __STATE_VAR_H
#define __STATE_VAR_H

#include "config_robot.h"
#include "config_environment.h"
#include "dev_imu.h"
#include "dev_encoder.h"
#include "dev_motor.h"
#include "five_link.h"

typedef struct State_Var_struct_t
{
  double s;
  
  double s_bias;
  
  double sd1;
  
  double phi;
  
  double phid1;
  
  double thetal_l;
  
  double thetald1_l;
  
  double thetal_r;
  
  double thetald1_r;
  
  double thetab;
  
  double thetabd1;
  
  /*中间变量用 begin*/
  
  double s_now;
  
  double s_last;
  
  double thetal_l_now;
  
  double thetal_l_last;
  
  double thetal_r_now;
  
  double thetal_r_last;
  
  /*中间变量用 end*/
}State_Var_t;

extern State_Var_t My_State_Var;

void My_State_Var_Update(void);

#endif


