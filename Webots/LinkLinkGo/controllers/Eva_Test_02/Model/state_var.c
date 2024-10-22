#include "state_var.h"

State_Var_t My_State_Var;

/**
  * @brief  ״̬��������
  * @param  None
  * @retval None
  */
void My_State_Var_Update(void)
{
  /*·�����ٶ�*/
  My_State_Var.s = 0.5 * WHEEL_RADIUS * (My_Encoder[R_Wheel_Motor].rad_now + My_Encoder[L_Wheel_Motor].rad_now);
  
  My_State_Var.s_now = My_State_Var.s;
  
  My_State_Var.sd1 = (My_State_Var.s_now - My_State_Var.s_last) / (TIME_STEP * 0.001);
  
  My_State_Var.s_last = My_State_Var.s_now;
  
  /*ƫ������ƫ�����ٶ�*/
  My_State_Var.phi = My_Posture.yaw;
  
  My_State_Var.phid1 = My_Posture.yaw_v;
  
  /*������б�Ƕ�����ٶ�*/
  My_State_Var.thetal_l = (My_Link_Var[L_Link].angle->vir_phi0 + My_Posture.pitch);
  
  My_State_Var.thetal_l_now = My_State_Var.thetal_l;
  
  My_State_Var.thetald1_l = (My_State_Var.thetal_l_now - My_State_Var.thetal_l_last) / (TIME_STEP * 0.001);
  
  My_State_Var.thetal_l_last = My_State_Var.thetal_l_now;
  
   /*������б�Ƕ�����ٶ�*/
  My_State_Var.thetal_r = (My_Link_Var[R_Link].angle->vir_phi0 + My_Posture.pitch);
  
  My_State_Var.thetal_r_now = My_State_Var.thetal_r;
  
  My_State_Var.thetald1_r = (My_State_Var.thetal_r_now - My_State_Var.thetal_r_last) / (TIME_STEP * 0.001);
  
  My_State_Var.thetal_r_last = My_State_Var.thetal_r_now;
  
  /*�������븩�����ٶ�*/
  My_State_Var.thetab = My_Posture.pitch;
  
  My_State_Var.thetabd1 = My_Posture.pitch_v;
}
















