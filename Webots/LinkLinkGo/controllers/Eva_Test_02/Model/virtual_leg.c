#include "virtual_leg.h"

void My_Leg_Feedforward_Cal(Link_Var_t* Link_Var, Virtual_Leg_t* Virtual_Var);
double My_Leg_Length_Strength_Cal(Virtual_Leg_t* Virtual_Var);
double My_Leg_Length_Strength_Cal(Virtual_Leg_t* Virtual_Var);
double My_Roll_Control(Virtual_Leg_t* Virtual_Var);

pid_info_t R_Leg_Length = 
{
  .kp = 850.f,//750
  .ki = 0.1f,
  .kd = 30000.f,
  .integral_max = 7.f,
  .out_max = 200.f,
};

pid_info_t L_Leg_Length = 
{
  .kp = 850.f,//750
  .ki = 0.1f,
  .kd = 30000.f,
  .integral_max = 7.f,
  .out_max = 200.f,
};

pid_info_t R_Leg_Roll = 
{
  .kp = 500.f,
  .ki = 0.1f,
  .kd = 0.f,
  .integral_max = 0.f,
  .out_max = 210.f,
};

pid_info_t L_Leg_Roll = 
{
  .kp = 500.f,
  .ki = 0.1f,
  .kd = 0.f,//4000
  .integral_max = 0.f,
  .out_max = 210.f,
};

pid_t Leg_Length_Pid[Leg_Num] = 
{
  [L_Leg] = {
    .info = &L_Leg_Length,
  },
  [R_Leg] = {
    .info = &R_Leg_Length,
  },
};

pid_t Leg_Roll_Pid[Leg_Num] = 
{
  [L_Leg] = {
    .info = &L_Leg_Roll,
  },
  [R_Leg] = {
    .info = &R_Leg_Roll,
  },
};

Virtual_Leg_t My_Virtual_Leg[Leg_Num] = 
{
  [L_Leg] = {
    .length_pid = &Leg_Length_Pid[L_Leg],
    .roll_pid = &Leg_Roll_Pid[L_Leg],
  },
  [R_Leg] = {
    .length_pid = &Leg_Length_Pid[R_Leg],
    .roll_pid = &Leg_Roll_Pid[R_Leg],
  },
};

void My_Virtual_Leg_Force_Cal(void)
{
  /* 左腿 begin */
  My_Leg_Feedforward_Cal(&My_Link_Var[L_Link], &My_Virtual_Leg[L_Leg]);

  My_Virtual_Leg[L_Leg].Fbl = My_Leg_Length_Strength_Cal(&My_Virtual_Leg[L_Leg]) + My_Virtual_Leg[L_Leg].F_gravity - \
                            My_Virtual_Leg[L_Leg].F_inertial + My_Roll_Control(&My_Virtual_Leg[L_Leg]);//注意正负
  /* 左腿 end */

  /* 右腿 begin */
  My_Leg_Feedforward_Cal(&My_Link_Var[R_Link], &My_Virtual_Leg[R_Leg]);

  My_Virtual_Leg[R_Leg].Fbl = My_Leg_Length_Strength_Cal(&My_Virtual_Leg[R_Leg]) + My_Virtual_Leg[R_Leg].F_gravity + \
                            My_Virtual_Leg[R_Leg].F_inertial - My_Roll_Control(&My_Virtual_Leg[R_Leg]);//注意正负
  /* 右腿 end */

}

/**
  * @brief  沿腿方向力前馈补偿计算
  * @param  Link_Var_t* Link_Var, Virtual_Leg_t* Virtual_Var
  * @retval None
  */
void My_Leg_Feedforward_Cal(Link_Var_t* Link_Var, Virtual_Leg_t* Virtual_Var)
{
  Virtual_Var->F_gravity = (0.5*mb + Link_Var->centroid->centriod_coefficient*m_l)*g*cos(Link_Var->angle->phi0 - PI/2);
  
  Virtual_Var->F_inertial = (0.5*mb + Link_Var->centroid->centriod_coefficient*m_l)*(Link_Var->length->l0 / (2*Rl))*My_State_Var.phid1*My_State_Var.sd1;
}

/**
  * @brief  计算不同腿长下所需沿杆方向力F
  * @param  Virtual_Leg_t* Virtual_Var
  * @retval 力F
  */
double My_Leg_Length_Strength_Cal(Virtual_Leg_t* Virtual_Var)
{
  double F = 0.f;
  
  Virtual_Var->length_pid->info->measure = (My_Link_Var[R_Link].length->l0 + My_Link_Var[L_Link].length->l0)*0.5f;
  
  Virtual_Var->length_pid->info->target = Virtual_Var->l_target;
  
  single_pid_ctrl(Virtual_Var->length_pid->info);
  
  F = Virtual_Var->length_pid->info->out;
  
  return F;
}

/**
  * @brief  roll控制
  * @param  Virtual_Leg_t* Virtual_Var
  * @retval 力F
  * @note  右腿减左腿加
  */
double My_Roll_Control(Virtual_Leg_t* Virtual_Var)
{
  double F = 0.f;
  
  Virtual_Var->roll_pid->info->measure = My_Posture.roll;
  
  Virtual_Var->roll_pid->info->target = TAR_ROLL_INITIAL;

  single_pid_ctrl(Virtual_Var->roll_pid->info);
  
  F = Virtual_Var->roll_pid->info->out;
  
  return F;
}




