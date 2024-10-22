#include "dev_imu.h"

/*获取需要用于�?�算的所有变�??*/

WbDeviceTag Imu;//测�?�度
WbDeviceTag Gyro;//测�?�速度
WbDeviceTag Acce;//测加速度

Posture_t My_Posture;

void My_Imu_Init(void)
{
  Imu = wb_robot_get_device("My_Imu");
  wb_inertial_unit_enable(Imu, TIME_STEP);
  
  Gyro = wb_robot_get_device("My_Gyro");
  wb_gyro_enable(Gyro, TIME_STEP);
  
  Acce = wb_robot_get_device("My_Accelerometer");
  wb_accelerometer_enable(Acce, TIME_STEP);
}

void My_Imu_Data_Update(void)
{
  const double *imu = wb_inertial_unit_get_roll_pitch_yaw(Imu);
  My_Posture.roll = imu[1];
  My_Posture.pitch = imu[0];
  My_Posture.yaw = imu[2];
  
  const double *gyro = wb_gyro_get_values(Gyro);
  My_Posture.roll_v = gyro[2];
  My_Posture.yaw_v = gyro[1];
  My_Posture.pitch_v = gyro[0];
  
  const double *acce = wb_accelerometer_get_values(Acce);
  My_Posture.x_acc = acce[0];//左右
  My_Posture.y_acc = acce[1];//上下
  My_Posture.z_acc = acce[2];//前后
}











