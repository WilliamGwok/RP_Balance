#include "dev_encoder.h"

/*获取需要用于�?�算的所有变�??*/

WbDeviceTag Encoder[Encoder_Total_Num];
Encoder_t My_Encoder[Encoder_Total_Num] = 
{
  [R_F_Sd_Encoder] = {
        .name = "R_F_Pos",
    },
  [R_B_Sd_Encoder] = {
        .name = "R_B_Pos",
    },
  [L_F_Sd_Encoder] = {
        .name = "L_F_Pos",
    },
  [L_B_Sd_Encoder] = {
        .name = "L_B_Pos",
    },
  [R_Wheel_Encoder] = {
        .name = "R_Wheel_Pos",
    },
  [L_Wheel_Encoder] = {
        .name = "L_Wheel_Pos",
    },
};

void My_Encoder_Init(void)
{
  for(int i = 0; i < Encoder_Total_Num; i++)
  {
    Encoder[i] = wb_robot_get_device(My_Encoder[i].name);
    
    wb_position_sensor_enable(Encoder[i], TIME_STEP);
  }
}

void My_Encoder_Data_Update(void)
{
  for(int i = 0; i < Encoder_Total_Num; i++)
  {
    My_Encoder[i].rad_now = wb_position_sensor_get_value(Encoder[i]);
    
    My_Encoder[i].rad_diff = My_Encoder[i].rad_now - My_Encoder[i].rad_last;
    
    My_Encoder[i].rad_last = My_Encoder[i].rad_now;
  }
}













