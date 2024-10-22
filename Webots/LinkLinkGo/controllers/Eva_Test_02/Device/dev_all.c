#include "dev_all.h"

void My_Device_Init(void)
{
  My_Imu_Init();

  My_Motor_Init();

  My_Encoder_Init();

  My_Keyboard_Init();
}

/**
  * @brief  传感器数据更新，包括Imu,Gyro，电机编码器
  * @param  None
  * @retval None
  */
void My_Device_Data_Update(void)
{
  My_Imu_Data_Update();

  My_Encoder_Data_Update();

  My_Motor_Data_Update();
}



