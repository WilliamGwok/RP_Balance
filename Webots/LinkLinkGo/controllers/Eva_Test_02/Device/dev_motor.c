#include "dev_motor.h"

/*鑾峰彇闇�瑕佺敤浜庯拷?锟界畻鐨勬墍鏈夊彉锟??*/

WbDeviceTag Motor[Motor_Total_Num];

Motor_t My_Motor[Motor_Total_Num] = 
{
  [R_B_Sd_Motor] = {
        .name = "R_B_Motor",
  },
  [R_F_Sd_Motor] = {
        .name = "R_F_Motor",
  },
  [L_B_Sd_Motor] = {
        .name = "L_B_Motor",
  },
  [L_F_Sd_Motor] = {
        .name = "L_F_Motor",
  },
  [R_Wheel_Motor] = {
        .name = "R_Wheel_Motor",
  },
  [L_Wheel_Motor] = {
        .name = "L_Wheel_Motor",
  },
};

void My_Motor_Init(void)
{
  for(int i = 0; i < Motor_Total_Num; i++)
  {
    Motor[i] = wb_robot_get_device(My_Motor[i].name);

    wb_motor_enable_torque_feedback(Motor[i], TIME_STEP);//使能力矩反馈
    
    if(i == R_Wheel_Motor || i == L_Wheel_Motor)
    {
      wb_motor_set_position(Motor[i], INFINITY);
    }
  }
  
}

void My_Motor_Data_Update(void)
{
  for(int i = 0; i < Motor_Total_Num; i++)
  {
    My_Motor[i].torque_measure = wb_motor_get_torque_feedback(Motor[i]);

    printf("Current motor torque: %f Nm\n", My_Motor[i].torque_measure);
  }
}

void My_Motor_Work(void)
{
  wb_motor_set_torque(Motor[R_F_Sd_Motor], constrain(My_Motor[R_F_Sd_Motor].output_torque, -32, 32));
  wb_motor_set_torque(Motor[R_B_Sd_Motor], constrain(My_Motor[R_B_Sd_Motor].output_torque, -32, 32));
    
  wb_motor_set_torque(Motor[L_F_Sd_Motor], constrain(My_Motor[L_F_Sd_Motor].output_torque, -32, 32));
  wb_motor_set_torque(Motor[L_B_Sd_Motor], constrain(My_Motor[L_B_Sd_Motor].output_torque, -32, 32));
    
  wb_motor_set_torque(Motor[R_Wheel_Motor], My_Motor[R_Wheel_Motor].output_torque);
  wb_motor_set_torque(Motor[L_Wheel_Motor], My_Motor[L_Wheel_Motor].output_torque);
}











