/*
 * File:          Eva_Test_0.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
#include <math.h>
#include <stdio.h> 
#include <string.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>  
#include <webots/motor.h>
#include <webots/gyro.h> 
 
 /*最小步进时长1ms*/
#define TIME_STEP            1
#define PI                   3.1415926535
#define DEVICE_NAME_LENGTH   20
#define WHEEL_RADIUS         0.01  //单位：m

#define my_abs(x)            ((x)>0? (x):(-(x)))
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))

FILE *fp = NULL;


void Imu_Gyro_Init(void);
void Motor_and_Encoder_Init(void);
void Imu_Gyro_Update(void);
void Encoder_Update(void);
void Wheel_Distance_Velocity_Update(void);
void State_Variable_Update(void);
void Printing_Task(void);
void Chassis_Ctrl(void);



/*...........................................传感器 begin...........................................*/
typedef struct Sensor_Data_Struct_t
{
  /*角度*/
  double yaw;
  double pitch;
  double roll;
  
  /*角度速度*/
  double yaw_v;
  double pitch_v;
  double roll_v;
}Sensor_Data_t;

WbDeviceTag Imu;//测角度
WbDeviceTag Gyro;//测角速度
Sensor_Data_t Sensor_Data;

/*...........................................传感器 end...........................................*/

/*...........................................电机与其编码器 begin...........................................*/
typedef enum Motor_List
{
  R_Sd_Motor = 0,
  
  R_Wheel_Motor,
  
  L_Sd_Motor,
  
  L_Wheel_Motor,
  
  Motor_Total_Num,
}Motor_List_e;

typedef enum Encoder_List
{
  R_Sd_Encoder = 0,
  
  R_Wheel_Encoder,
  
  L_Sd_Encoder,
  
  L_Wheel_Encoder,
  
  Encode_Total_Num,
}Encoder_List_e;





typedef struct Motor_struct_t
{
  char name[DEVICE_NAME_LENGTH];
  
  double velocity;  
  
  double distance_now;
  
  double distance_last;
  
}Motor_t;

WbDeviceTag My_Motor[Motor_Total_Num];
Motor_t Motor_Info[Motor_Total_Num] = 
{
  [R_Sd_Motor] = {
        .name = "R_Motor_Up",
    },
    
  [R_Wheel_Motor] = {
        .name = "R_Motor_Down",
    },
    
  [L_Sd_Motor] = {
        .name = "L_Motor_Up",
    },
    
  [L_Wheel_Motor] = {
        .name = "L_Motor_Down",
    },
};



typedef struct Encoder_struct_t
{
  char name[DEVICE_NAME_LENGTH];
  
  double rad_diff;
  
  double rad_now;
  
  double rad_last;
  
}Encoder_t;

WbDeviceTag My_Encoder[Encode_Total_Num];
Encoder_t Encoder_Info[Encode_Total_Num] = 
{
  [R_Sd_Encoder] = {
        .name = "R_Pos_Up",
    },
    
  [R_Wheel_Encoder] = {
        .name = "R_Pos_Down",
    },
    
  [L_Sd_Encoder] = {
        .name = "L_Pos_Up",
    },
    
  [L_Wheel_Encoder] = {
        .name = "L_Pos_Down",
    },
};

/*...........................................电机与其编码器 end...........................................*/



/*...........................................状态变量 begin...........................................*/

typedef struct Space_Var_struct_t
{
    double theta;
    double thetad1;
    double xd0;
    double xd1;
    double phi;
    double phid1;

    double theta_now;
    double theta_last;
    double theta_diff;
}Space_Var_t;

Space_Var_t Space_Var_L;
Space_Var_t Space_Var_R;

/*...........................................状态变量 end...........................................*/


/*...........................................PID begin...........................................*/
typedef struct 
{
	double	  target;
	double	  measure;
	double 	err;
	double 	last_err;
	double	  integral;
	double 	pout;
	double 	iout;
	double 	dout;
	double 	out;
	/* 配置 */
	double   blind_err;
	double   integral_bias;
	double	  kp;
	double 	ki;
	double 	kd;
	double 	integral_max;
	double 	out_max;
}pid_info_t;

//左右腿角度差控制器,左右两边输出相反
pid_info_t Leg_Diff_PD_Ctrl = 
{
  .kp = 0,//1.5,
  .kd = 0,//0.5,
  .out_max = 25,//输出扭矩，这里按电机可提供的最大扭矩给
};
/*...........................................PID end...........................................*/




/*...........................................底盘 begin...........................................*/

typedef struct Chassis_struct_t
{
  double target_speed_now;
  
  double target_speed_last;

  double sd_K[6];

  double wheel_K[6];

  double theta_part;

  double x_part;

  double phi_part;

  double sd_torque;

  double wheel_torque;
}Chassis_t;

//  -44.0828   -5.6364   -0.9492   -2.1450   13.7310    2.4060
  // 21.9025    2.5431    0.6293    1.3660   63.2551    6.0526

// Chassis_t Chassis =     
// {
  // .sd_K = {1,  0,  0.6293  ,  1.3660   , -63.2551 ,   -6.0526},//+ - -
  // .wheel_K = { -20.4142 , -1,-0.9492 ,  -2.1450 ,  13.7310  ,  2.4060},//- - +
// };

Chassis_t Chassis =     
{
  .sd_K = {1,  0,  0.6293  ,  1.3660   , -63.2551 ,   -6.0526},//+ - -
  .wheel_K = { -20.4142 , -1,-0.9492 ,  -2.1450 ,  13.7310  ,  2.4060},//- - +
};

/*...........................................底盘 end...........................................*/

/**
  * @brief  所有设备初始化
  * @param  None
  * @retval None
  */
void Wb_Get_All_Device(void)
{
  Imu_Gyro_Init();

  Motor_and_Encoder_Init();
}

/**
  * @brief  所有传感器数据更新（32ms更新一次）
  * @param  None
  * @retval None
  */
void Wb_Get_All_Data_Update(void)
{
  Imu_Gyro_Update();

  Encoder_Update();

  Wheel_Distance_Velocity_Update();

  State_Variable_Update();
}

int main(int argc, char **argv) {
 
  fp = fopen("C:\\Users\\Lenovo\\Desktop\\Balance\\PaperFiles\\Data\\Print_Data.txt","w");
  
  wb_robot_init();
  
  Wb_Get_All_Device();


  while (wb_robot_step(TIME_STEP) != -1) {
  
    Wb_Get_All_Data_Update();

    Chassis_Ctrl();
    
    Printing_Task();

  };

  wb_robot_cleanup();
  
  fclose(fp);
  
  return 0;
}



void single_pid_cal(pid_info_t *pid)
{
	pid->err = pid->target - pid->measure;
	if(abs(pid->err)<=(pid->blind_err))
		pid->err = 0;
	// 积分
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -pid->integral_max+pid->integral_bias, pid->integral_max+pid->integral_bias);
	// p i d 输出项计算
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->last_err);
	// 累加pid输出值
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// 记录上次误差值
	pid->last_err = pid->err;
}

void Imu_Gyro_Init(void)
{
  Imu = wb_robot_get_device("My_Imu");
  wb_inertial_unit_enable(Imu, TIME_STEP);
  
  Gyro = wb_robot_get_device("My_Gyro");
  wb_gyro_enable(Gyro, TIME_STEP);
}

void Motor_and_Encoder_Init(void)
{
  My_Motor[R_Sd_Motor] = wb_robot_get_device("R_Motor_Up");
  My_Motor[R_Wheel_Motor] = wb_robot_get_device("R_Motor_Down");
  My_Motor[L_Sd_Motor] = wb_robot_get_device("L_Motor_Up");
  My_Motor[L_Wheel_Motor] = wb_robot_get_device("L_Motor_Down");
  wb_motor_set_position(My_Motor[R_Wheel_Motor], INFINITY);
  // wb_motor_set_position(My_Motor[R_Sd_Motor], INFINITY);
  wb_motor_set_position(My_Motor[L_Wheel_Motor], INFINITY);
  // wb_motor_set_position(My_Motor[L_Sd_Motor], INFINITY);
  
  My_Encoder[R_Sd_Encoder] = wb_robot_get_device("R_Pos_Up");
  My_Encoder[R_Wheel_Encoder] = wb_robot_get_device("R_Pos_Down");
  My_Encoder[L_Sd_Encoder] = wb_robot_get_device("L_Pos_Up");
  My_Encoder[L_Wheel_Encoder] = wb_robot_get_device("L_Pos_Down");
  wb_position_sensor_enable(My_Encoder[R_Sd_Encoder], TIME_STEP);
  wb_position_sensor_enable(My_Encoder[R_Wheel_Encoder], TIME_STEP);
  wb_position_sensor_enable(My_Encoder[L_Sd_Encoder], TIME_STEP);
  wb_position_sensor_enable(My_Encoder[L_Wheel_Encoder], TIME_STEP);
}

void Imu_Gyro_Update(void)
{
  /*角度更新， 单位：rad?*/
  /*Note that the indices 0, 1 and 2 return the roll, pitch and yaw angles respectively.*/
  const double *imu = wb_inertial_unit_get_roll_pitch_yaw(Imu);
  Sensor_Data.roll = imu[0];
  Sensor_Data.pitch = imu[1];
  Sensor_Data.yaw = imu[2];

  /*角速度更新 单位： rad/s */
  /*The first element corresponds to the angular velocity about the x-axis, the second element to the y-axis, etc.*/
  /*X_axis -> roll
    Y_axis -> yaw
    Z_axis -> pitch*/
  const double *gyro = wb_gyro_get_values(Gyro);
  Sensor_Data.roll_v = gyro[0];
  Sensor_Data.yaw_v = gyro[1];
  Sensor_Data.pitch_v = gyro[2];
}

void Encoder_Update(void)
{
  /*编码器更新 单位： rad/s */
  /*弧度和更新*/
  Encoder_Info[R_Sd_Encoder].rad_now = wb_position_sensor_get_value(My_Encoder[R_Sd_Encoder]);//机体摆正时，摆杆逆时针运动时rad++,顺时针时rad--
  Encoder_Info[R_Wheel_Encoder].rad_now = -wb_position_sensor_get_value(My_Encoder[R_Wheel_Encoder]);
  Encoder_Info[L_Sd_Encoder].rad_now = wb_position_sensor_get_value(My_Encoder[L_Sd_Encoder]);//机体摆正时，摆杆逆时针运动时rad--,顺时针时rad++
  Encoder_Info[L_Wheel_Encoder].rad_now = wb_position_sensor_get_value(My_Encoder[L_Wheel_Encoder]);
  
  /*弧度差更新*/
  Encoder_Info[R_Sd_Encoder].rad_diff = Encoder_Info[R_Sd_Encoder].rad_now - Encoder_Info[R_Sd_Encoder].rad_last;//机体摆正时，摆杆逆时针运动时rad为+,顺时针时rad为-
  Encoder_Info[R_Wheel_Encoder].rad_diff = Encoder_Info[R_Sd_Encoder].rad_now - Encoder_Info[R_Wheel_Encoder].rad_last;
  Encoder_Info[L_Sd_Encoder].rad_diff = Encoder_Info[R_Sd_Encoder].rad_now - Encoder_Info[L_Sd_Encoder].rad_last;//机体摆正时，摆杆逆时针运动时rad为-,顺时针时rad为+
  Encoder_Info[L_Wheel_Encoder].rad_diff = Encoder_Info[R_Sd_Encoder].rad_now - Encoder_Info[L_Wheel_Encoder].rad_last;
  
  Encoder_Info[R_Sd_Encoder].rad_last = Encoder_Info[R_Sd_Encoder].rad_now;
  Encoder_Info[R_Wheel_Encoder].rad_last = Encoder_Info[R_Wheel_Encoder].rad_now;
  Encoder_Info[L_Sd_Encoder].rad_last = Encoder_Info[L_Sd_Encoder].rad_now;
  Encoder_Info[L_Wheel_Encoder].rad_last = Encoder_Info[L_Wheel_Encoder].rad_now;
}

void Wheel_Distance_Velocity_Update(void)
{
  /*驱动轮电机位移更新 单位： m */
  Motor_Info[R_Wheel_Encoder].distance_now = (wb_position_sensor_get_value(My_Encoder[R_Wheel_Encoder])) * WHEEL_RADIUS;
  Motor_Info[L_Wheel_Encoder].distance_now = (wb_position_sensor_get_value(My_Encoder[L_Wheel_Encoder])) * WHEEL_RADIUS;
  
  /*驱动轮电机速度更新 单位： m/s */
  Motor_Info[R_Wheel_Encoder].velocity = (Motor_Info[R_Wheel_Encoder].distance_now - Motor_Info[R_Wheel_Encoder].distance_last) / (TIME_STEP * 0.001);
  Motor_Info[L_Wheel_Encoder].velocity = (Motor_Info[L_Wheel_Encoder].distance_now - Motor_Info[L_Wheel_Encoder].distance_last) / (TIME_STEP * 0.001);
  
  Motor_Info[R_Wheel_Encoder].distance_last = Motor_Info[R_Wheel_Encoder].distance_now;
  Motor_Info[L_Wheel_Encoder].distance_last = Motor_Info[L_Wheel_Encoder].distance_now;
}

void State_Variable_Update(void)
{
  /*左腿  更新状态变量*/
  Space_Var_L.phi = Sensor_Data.pitch;//测试方向正确
  Space_Var_L.phid1 = Sensor_Data.pitch_v;//测试方向正确
  Space_Var_L.xd0 = (Motor_Info[L_Wheel_Encoder].distance_now);//测试方向正确
  
  /*驱动轮速度以右轮顺时针方向为正*/
  Space_Var_L.xd1 = Motor_Info[L_Wheel_Encoder].velocity;//测试方向正确
  Space_Var_L.theta = (Encoder_Info[L_Sd_Encoder].rad_now - Space_Var_L.phi);

  Space_Var_L.theta_now =  Space_Var_L.theta;
  Space_Var_L.theta_diff = Space_Var_L.theta_now - Space_Var_L.theta_last;
  Space_Var_L.theta_last = Space_Var_L.theta_now;

  Space_Var_L.thetad1 = Space_Var_L.theta_diff / (TIME_STEP * 0.001);



  /*更新状态变量*/
  Space_Var_R.phi = Sensor_Data.pitch;//测试方向正确
  Space_Var_R.phid1 = Sensor_Data.pitch_v;//测试方向正确
  Space_Var_R.xd0 = (Motor_Info[R_Wheel_Encoder].distance_now);//测试方向正确
  
  /*驱动轮速度以右轮顺时针方向为正*/
  Space_Var_R.xd1 = Motor_Info[R_Wheel_Encoder].velocity;//测试方向正确
  Space_Var_R.theta = (Encoder_Info[R_Sd_Encoder].rad_now - Space_Var_R.phi);

  Space_Var_R.theta_now =  Space_Var_R.theta;
  Space_Var_R.theta_diff = Space_Var_R.theta_now - Space_Var_R.theta_last;
  Space_Var_R.theta_last = Space_Var_R.theta_now;

  Space_Var_R.thetad1 = Space_Var_R.theta_diff / (TIME_STEP * 0.001);

  /*两腿角度差更新*/
  Leg_Diff_PD_Ctrl.measure = (Encoder_Info[R_Sd_Encoder].rad_now - Encoder_Info[L_Sd_Encoder].rad_now) * 57.8f;
  Leg_Diff_PD_Ctrl.target = 0.f;
  single_pid_cal(&Leg_Diff_PD_Ctrl);
  double cal_tor_test = Leg_Diff_PD_Ctrl.out;//给右腿的极性
  printf("%f \t",Space_Var_R.theta_now);
  printf("%f\n",Space_Var_R.thetad1);
}

void Printing_Task(void)
{
  // static int print_switch = 0;

  // if(print_switch == 0)
  // {
  //   fprintf(fp,"theta    \t thetad1    \t xd0    \t xd1    \t phi    \t phid1    \n");
  //   print_switch = 1;
  // }

  // /*将pitch打印到excel表格中*/
  // fprintf(fp,"%f \t", Space_Var.theta);
  // fprintf(fp,"%f \t", Space_Var.thetad1);
  // fprintf(fp,"%f \t", Space_Var.xd0);
  // fprintf(fp,"%f \t", Space_Var.xd1);
  // fprintf(fp,"%f \t", Space_Var.phi);
  // fprintf(fp,"%f \n", Space_Var.phid1);
  
}

void Chassis_Ctrl(void)
{
  /*左腿*/
  Chassis.theta_part = (Chassis.wheel_K[0] * Space_Var_L.theta + Chassis.wheel_K[1] * Space_Var_L.thetad1);
  Chassis.x_part =  (Chassis.wheel_K[2] * Space_Var_L.xd0 + Chassis.wheel_K[3] * Space_Var_L.xd1);
  Chassis.phi_part = Chassis.wheel_K[4] * Space_Var_L.phi + Chassis.wheel_K[5] * Space_Var_L.phid1;

  double Wheel_Torque = -0.5 * (Chassis.theta_part + Chassis.x_part + Chassis.phi_part);//右腿加负
  
  Wheel_Torque = constrain(Wheel_Torque, -30, 30);

  wb_motor_set_torque(My_Motor[L_Wheel_Motor], Wheel_Torque);

  /*右腿*/
  Chassis.theta_part = (Chassis.wheel_K[0] * Space_Var_R.theta + Chassis.wheel_K[1] * Space_Var_R.thetad1);
  Chassis.x_part =  (Chassis.wheel_K[2] * Space_Var_R.xd0 + Chassis.wheel_K[3] * Space_Var_R.xd1);
  Chassis.phi_part = Chassis.wheel_K[4] * Space_Var_R.phi + Chassis.wheel_K[5] * Space_Var_R.phid1;
  
  Wheel_Torque = constrain(Wheel_Torque, -30, 30);

  Wheel_Torque = -0.5 * (Chassis.theta_part + Chassis.x_part + Chassis.phi_part);

  wb_motor_set_torque(My_Motor[R_Wheel_Motor], Wheel_Torque);
  
  
  // /*现象：位移++输出的负转矩++*/
  //右视图：右轮输出负扭矩时逆时针转，验证极性正确
  // //右视图，sd电机给负扭矩时腿逆时针转动
  
  /*左腿*/
  Chassis.theta_part = (Chassis.sd_K[0] * Space_Var_L.theta + Chassis.sd_K[1] * Space_Var_L.thetad1);
  Chassis.x_part =  Chassis.sd_K[2] * Space_Var_L.xd0 + Chassis.sd_K[3] * Space_Var_L.xd1;
  Chassis.phi_part = (Chassis.sd_K[4] * Space_Var_L.phi + Chassis.sd_K[5] * Space_Var_L.phid1);
  
  double Sd_Torque = 1 * (Chassis.theta_part + Chassis.x_part + Chassis.phi_part) - 1 * Leg_Diff_PD_Ctrl.out;
  
  Sd_Torque = constrain(Sd_Torque, -30, 30);
  
  
  wb_motor_set_torque(My_Motor[L_Sd_Motor], Sd_Torque);

  /*右腿*/
  Chassis.theta_part = (Chassis.sd_K[0] * Space_Var_R.theta + Chassis.sd_K[1] * Space_Var_R.thetad1);
  Chassis.x_part =  Chassis.sd_K[2] * Space_Var_R.xd0 + Chassis.sd_K[3] * Space_Var_R.xd1;
  Chassis.phi_part = (Chassis.sd_K[4] * Space_Var_R.phi + Chassis.sd_K[5] * Space_Var_R.phid1);
  
  Sd_Torque = 1 * (Chassis.theta_part + Chassis.x_part + Chassis.phi_part) + 1 * Leg_Diff_PD_Ctrl.out;
  
  Sd_Torque = constrain(Sd_Torque, -30, 30);
  
  wb_motor_set_torque(My_Motor[R_Sd_Motor], Sd_Torque);


  
}

//23/9/17/21:43 艹！
//测量值取两腿平均值会有问题？两腿工况不同（大部分情况下不可能相同），
//一腿到达平衡点但另一腿没有，此时平均值不为0，则两腿都会有输出，导致原来到达平衡点处的腿再次偏离平衡点，系统进入无法双腿平衡的死循环
//那两腿分开控会有什么问题吗？
//
