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
#include <webots/keyboard.h>

 /*最小步进时长1ms*/
#define TIME_STEP            1
#define PI                   3.1415926535
#define DEVICE_NAME_LENGTH   20
#define WHEEL_RADIUS         0.1  //单位：m
#define WHEEL_MAX_TORQUE     6
#define SD_MAX_TORQUE        25

#define FORWARD_SPEED        2
#define BACK_SPEED           -2

#define my_abs(x)            ((x)>0? (x):(-(x)))
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define sgn(x) (((x)>0)?1:((x)<0?-1:0))

FILE *fp = NULL;


void Imu_Gyro_Init(void);
void Motor_and_Encoder_Init(void);
void Imu_Gyro_Update(void);
void Encoder_Update(void);
void Wheel_Distance_Velocity_Update(void);
void State_Variable_Update(void);
void Key_and_Command_Update(void);
void Printing_Task(void);
void Chassis_Ctrl(void);
void Wb_Get_All_Device(void);
void Wb_Get_All_Data_Update(void);



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

  double angular_velocity;
  
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

typedef struct State_Var_struct_t
{
    double s;
    double sd1;

    double phi;
    double phid1;

    double thetal_l;
    double thetald1_l;

    double thetal_r;
    double thetald1_r;

    double thetab;
    double thetabd1;


    double thetal_r_now;
    double thetal_r_last;
    double thetal_r__diff;

    double thetal_l_now;
    double thetal_l_last;
    double thetal_l__diff;
}State_Var_t;

State_Var_t State_Var;

/*...........................................状态变量 end...........................................*/


/*...........................................底盘 begin...........................................*/

typedef struct Chassis_struct_t
{
  double target_speed_now;
  
  double target_speed_last;
  
  double speed_diff;

  double rad_bias_r;

  double rad_bias_l;
  
  /*左右腿LQR的K矩阵*/
  double sdr_K[10];

  double wheelr_K[10];
  
  double sdl_K[10];

  double wheell_K[10];

  /*K矩阵各部分*/
  double s_part;

  double phi_part;

  double thetal_l_part;

  double thetal_r_part;

  double thetab_part;

  double sd_torque;

  double wheel_torque;
}Chassis_t;


Chassis_t Chassis =     
{
  .sdr_K = { -0.0311 ,  -0.0952 ,   0.6806   , 0.7925 , -13.1370,   -1.3218 ,  14.9232  ,  1.4093  , -4.3222  , -1.2976},//- + + - +
  
  .wheelr_K = {-0.7064 ,  -1.3457 ,   0.1916  ,  0.1561    ,5.1205 ,   0.4613 , -16.1479  , -1.7997 ,  -0.8692  , -0.1893},//- - - + +
  
  .sdl_K = {-0.0311  , -0.0952 ,  -0.6806 ,  -0.7925 ,  14.9232  ,  1.4093 , -13.1370  , -1.3218 ,  -4.3222  , -1.2976},//- - - + +
  
  .wheell_K = {-0.7064 ,  -1.3457  , -0.1916  , -0.1561,  -16.1479  , -1.7997  ,  5.1205 ,   0.4613 ,  -0.8692   ,-0.1893},//- + + - +
};

/*...........................................底盘 end...........................................*/




/**
  * @brief  主函数
  * @param  None
  * @retval None
  */
int main(int argc, char **argv) {
 
  fp = fopen("C:\\Users\\Lenovo\\Desktop\\Balance\\PaperFiles\\Data\\Print_Data.txt","w");
  
  wb_robot_init();
  
  Wb_Get_All_Device();


  while (wb_robot_step(TIME_STEP) != -1) {
  
    Wb_Get_All_Data_Update();
    
    

    Chassis_Ctrl();

  };

  wb_robot_cleanup();
  
  fclose(fp);
  
  return 0;
}
































/**
  * @brief  所有设备初始化
  * @param  None
  * @retval None
  */
void Wb_Get_All_Device(void)
{
  Imu_Gyro_Init();

  Motor_and_Encoder_Init();
  
  wb_keyboard_enable(TIME_STEP);
}

/**
  * @brief  所有传感器数据更新（1ms更新一次）
  * @param  None
  * @retval None
  */
void Wb_Get_All_Data_Update(void)
{
  Key_and_Command_Update();//优先级

  Imu_Gyro_Update();

  Encoder_Update();

  Wheel_Distance_Velocity_Update();

  State_Variable_Update();
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
  // wb_motor_set_position(My_Motor[R_Wheel_Motor], INFINITY);
  // wb_motor_set_position(My_Motor[R_Sd_Motor], INFINITY);
  // wb_motor_set_position(My_Motor[L_Wheel_Motor], INFINITY);
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
  Sensor_Data.roll = imu[1];
  Sensor_Data.pitch = imu[0];
  Sensor_Data.yaw = imu[2];

  /*角速度更新 单位： rad/s */
  /*The first element corresponds to the angular velocity about the x-axis, the second element to the y-axis, etc.*/
  /*X_axis -> roll
    Y_axis -> yaw
    Z_axis -> pitch*/
  const double *gyro = wb_gyro_get_values(Gyro);
  Sensor_Data.roll_v = gyro[2];
  Sensor_Data.yaw_v = gyro[1];
  Sensor_Data.pitch_v = gyro[0];
}

void Encoder_Update(void)
{
  /*编码器更新 单位： rad/s */
  /*弧度和更新*/
  Encoder_Info[R_Sd_Encoder].rad_now = wb_position_sensor_get_value(My_Encoder[R_Sd_Encoder]);//机体摆正时，摆杆逆时针运动时rad++,顺时针时rad--
  Encoder_Info[R_Wheel_Encoder].rad_now = wb_position_sensor_get_value(My_Encoder[R_Wheel_Encoder]) - Chassis.rad_bias_r;
  Encoder_Info[L_Sd_Encoder].rad_now = wb_position_sensor_get_value(My_Encoder[L_Sd_Encoder]);//机体摆正时，摆杆逆时针运动时rad--,顺时针时rad++
  Encoder_Info[L_Wheel_Encoder].rad_now = wb_position_sensor_get_value(My_Encoder[L_Wheel_Encoder]) - Chassis.rad_bias_l;
  
  /*弧度差更新*/
  Encoder_Info[R_Sd_Encoder].rad_diff = Encoder_Info[R_Sd_Encoder].rad_now - Encoder_Info[R_Sd_Encoder].rad_last;//机体摆正时，摆杆逆时针运动时rad为+,顺时针时rad为-
  Encoder_Info[R_Wheel_Encoder].rad_diff = Encoder_Info[R_Wheel_Encoder].rad_now - Encoder_Info[R_Wheel_Encoder].rad_last;
  Encoder_Info[L_Sd_Encoder].rad_diff = Encoder_Info[R_Sd_Encoder].rad_now - Encoder_Info[L_Sd_Encoder].rad_last;//机体摆正时，摆杆逆时针运动时rad为-,顺时针时rad为+
  Encoder_Info[L_Wheel_Encoder].rad_diff = Encoder_Info[L_Wheel_Encoder].rad_now - Encoder_Info[L_Wheel_Encoder].rad_last;
  
  Encoder_Info[R_Sd_Encoder].rad_last = Encoder_Info[R_Sd_Encoder].rad_now;
  Encoder_Info[R_Wheel_Encoder].rad_last = Encoder_Info[R_Wheel_Encoder].rad_now;
  Encoder_Info[L_Sd_Encoder].rad_last = Encoder_Info[L_Sd_Encoder].rad_now;
  Encoder_Info[L_Wheel_Encoder].rad_last = Encoder_Info[L_Wheel_Encoder].rad_now;
}

void Wheel_Distance_Velocity_Update(void)
{
  /*驱动轮电机位移更新 单位： m */
  Motor_Info[R_Wheel_Encoder].distance_now = Encoder_Info[R_Wheel_Encoder].rad_now * WHEEL_RADIUS;
  Motor_Info[L_Wheel_Encoder].distance_now = Encoder_Info[L_Wheel_Encoder].rad_now * WHEEL_RADIUS;
  
  /*驱动轮电机线速度更新 单位： m/s */
  Motor_Info[R_Wheel_Encoder].velocity = (Motor_Info[R_Wheel_Encoder].distance_now - Motor_Info[R_Wheel_Encoder].distance_last) / (TIME_STEP * 0.001);
  Motor_Info[L_Wheel_Encoder].velocity = (Motor_Info[L_Wheel_Encoder].distance_now - Motor_Info[L_Wheel_Encoder].distance_last) / (TIME_STEP * 0.001);
  
  Motor_Info[R_Wheel_Encoder].distance_last = Motor_Info[R_Wheel_Encoder].distance_now;
  Motor_Info[L_Wheel_Encoder].distance_last = Motor_Info[L_Wheel_Encoder].distance_now;

  /*驱动轮电机角速度更新 单位：rad/s */
  Motor_Info[R_Wheel_Encoder].angular_velocity = Encoder_Info[R_Wheel_Encoder].rad_diff / (TIME_STEP * 0.001);
  Motor_Info[L_Wheel_Encoder].angular_velocity = Encoder_Info[L_Wheel_Encoder].rad_diff / (TIME_STEP * 0.001);
}

/**
  * @brief  状态变量更新
  * @param  None
  * @retval None
  */
void State_Variable_Update(void)
{
    State_Var.s = 0.5*(Motor_Info[R_Wheel_Encoder].distance_now + Motor_Info[L_Wheel_Encoder].distance_now);//1

    State_Var.sd1 = 0.5*WHEEL_RADIUS*(Motor_Info[R_Wheel_Encoder].angular_velocity + Motor_Info[L_Wheel_Encoder].angular_velocity);//2

    State_Var.phi = Sensor_Data.yaw;//3

    State_Var.phid1 = Sensor_Data.yaw_v;//4

    State_Var.thetal_l =  (Encoder_Info[L_Sd_Encoder].rad_now + Sensor_Data.pitch);//5

    State_Var.thetal_l_now =  State_Var.thetal_l;
    State_Var.thetal_l__diff = State_Var.thetal_l_now - State_Var.thetal_l_last;
    State_Var.thetal_l_last = State_Var.thetal_l_now;

    State_Var.thetald1_l = State_Var.thetal_l__diff / (TIME_STEP * 0.001);//6

    State_Var.thetal_r = (Encoder_Info[R_Sd_Encoder].rad_now + Sensor_Data.pitch);//7

    State_Var.thetal_r_now =  State_Var.thetal_r;
    State_Var.thetal_r__diff = State_Var.thetal_r_now - State_Var.thetal_r_last;
    State_Var.thetal_r_last = State_Var.thetal_r_now;

    State_Var.thetald1_r = State_Var.thetal_r__diff / (TIME_STEP * 0.001);//8

    State_Var.thetab = Sensor_Data.pitch;

    State_Var.thetabd1 = Sensor_Data.pitch_v;
    
    if(Chassis.target_speed_now != 0)
    {
      State_Var.s = 0;
    }
}

/*键值：
  w 87 a 65 s 83 d 68
  x 88
 上 315 下 317 左 314 右 316
 -1 无输入
*/
void Key_and_Command_Update(void)
{
  int my_key = 0;
  
  my_key = wb_keyboard_get_key();
    
  

  switch (my_key)
  {
  case 87:
    Chassis.target_speed_now = FORWARD_SPEED;
    break;
  case 83:
    Chassis.target_speed_now = BACK_SPEED;
    break;
  case 88:
    wb_keyboard_disable();
    break;
  default:
    Chassis.target_speed_now = 0.f;
    break;
  }

  if(Chassis.target_speed_last != 0 && Chassis.target_speed_now == 0)
  {
    // printf("%d \n", my_key);
    

    Chassis.rad_bias_r = wb_position_sensor_get_value(My_Encoder[R_Wheel_Encoder]);
    Chassis.rad_bias_l = wb_position_sensor_get_value(My_Encoder[L_Wheel_Encoder]);
      

    Encoder_Info[R_Wheel_Encoder].rad_last = 0;
    Encoder_Info[L_Wheel_Encoder].rad_last = 0;
    
    Motor_Info[R_Wheel_Encoder].distance_last = 0;
    Motor_Info[L_Wheel_Encoder].distance_last = 0;
  }

  if(Chassis.target_speed_now != 0)
  {
    Motor_Info[R_Wheel_Encoder].distance_now = 0;
    Motor_Info[L_Wheel_Encoder].distance_now = 0;
  }
  
  printf("%f \t %f \n", Encoder_Info[R_Sd_Encoder].rad_now, Sensor_Data.pitch); 

  Chassis.target_speed_last = Chassis.target_speed_now;
}

void Chassis_Ctrl(void)
{
    Chassis.speed_diff = Chassis.target_speed_now - Chassis.wheell_K[1] * State_Var.sd1;
    // printf("%f \n",State_Var.s);
    /*驱动轮控制*/
    /*左*/
    Chassis.s_part = Chassis.wheell_K[0] * State_Var.s + Chassis.wheell_K[1] * Chassis.speed_diff;
    Chassis.phi_part = Chassis.wheell_K[2] * State_Var.phi + Chassis.wheell_K[3] * State_Var.phid1;
    Chassis.thetal_l_part = Chassis.wheell_K[4] * State_Var.thetal_l + Chassis.wheell_K[5] * State_Var.thetald1_l;
    Chassis.thetal_r_part = Chassis.wheell_K[6] * State_Var.thetal_r + Chassis.wheell_K[7] * State_Var.thetald1_r;
    Chassis.thetab_part = Chassis.wheell_K[8] * State_Var.thetab + Chassis.wheell_K[9] * State_Var.thetabd1;

    Chassis.wheel_torque = Chassis.s_part + Chassis.phi_part + Chassis.thetal_l_part + Chassis.thetal_r_part + Chassis.thetab_part;

    Chassis.wheel_torque = -constrain(Chassis.wheel_torque, -WHEEL_MAX_TORQUE, WHEEL_MAX_TORQUE);
    
    // printf("L: %f \t %f \t %f \t %f \t %f \n",Chassis.s_part,Chassis.phi_part,Chassis.thetal_l_part, Chassis.thetal_r_part ,Chassis.thetab_part);

    wb_motor_set_torque(My_Motor[L_Wheel_Motor], Chassis.wheel_torque);
    
    // printf("L_W: %f \t", Chassis.wheel_torque);

    /*右*/
    Chassis.s_part = Chassis.wheelr_K[0] * State_Var.s + Chassis.wheelr_K[1] * Chassis.speed_diff;
    Chassis.phi_part = Chassis.wheelr_K[2] * State_Var.phi + Chassis.wheelr_K[3] * State_Var.phid1;
    Chassis.thetal_l_part = Chassis.wheelr_K[4] * State_Var.thetal_l + Chassis.wheelr_K[5] * State_Var.thetald1_l;
    Chassis.thetal_r_part = Chassis.wheelr_K[6] * State_Var.thetal_r + Chassis.wheelr_K[7] * State_Var.thetald1_r;
    Chassis.thetab_part = Chassis.wheelr_K[8] * State_Var.thetab + Chassis.wheelr_K[9] * State_Var.thetabd1;

    Chassis.wheel_torque = Chassis.s_part + Chassis.phi_part + Chassis.thetal_l_part + Chassis.thetal_r_part + Chassis.thetab_part;

    Chassis.wheel_torque = -constrain(Chassis.wheel_torque, -WHEEL_MAX_TORQUE, WHEEL_MAX_TORQUE);

    wb_motor_set_torque(My_Motor[R_Wheel_Motor], Chassis.wheel_torque);
    
    // printf("R_W: %f \t", Chassis.wheel_torque);

    /*关节电机控制*/
    /*左*/
    Chassis.s_part = Chassis.sdl_K[0] * State_Var.s + Chassis.sdl_K[1] * Chassis.speed_diff;
    Chassis.phi_part = Chassis.sdl_K[2] * State_Var.phi + Chassis.sdl_K[3] * State_Var.phid1;
    Chassis.thetal_l_part = Chassis.sdl_K[4] * State_Var.thetal_l + Chassis.sdl_K[5] * State_Var.thetald1_l;
    Chassis.thetal_r_part = Chassis.sdl_K[6] * State_Var.thetal_r + Chassis.sdl_K[7] * State_Var.thetald1_r;
    Chassis.thetab_part = Chassis.sdl_K[8] * State_Var.thetab + Chassis.sdl_K[9] * State_Var.thetabd1;

    Chassis.sd_torque = (Chassis.s_part + Chassis.phi_part + Chassis.thetal_l_part + Chassis.thetal_r_part + Chassis.thetab_part);

    Chassis.sd_torque = -constrain(Chassis.sd_torque, -SD_MAX_TORQUE, SD_MAX_TORQUE);
    
    
   
    wb_motor_set_torque(My_Motor[L_Sd_Motor], Chassis.sd_torque);
    
    // printf("L_S: %f \t", Chassis.sd_torque);

    /*右*/
    Chassis.s_part = Chassis.sdr_K[0] * State_Var.s + Chassis.sdr_K[1] * Chassis.speed_diff;
    Chassis.phi_part = Chassis.sdr_K[2] * State_Var.phi + Chassis.sdr_K[3] * State_Var.phid1;
    Chassis.thetal_l_part = Chassis.sdr_K[4] * State_Var.thetal_l + Chassis.sdr_K[5] * State_Var.thetald1_l;
    // if(sgn(State_Var.thetal_l) != sgn(State_Var.thetal_r))
    // {
      // Chassis.thetal_l_part = -Chassis.thetal_l_part;
    // }
    // Chassis.thetal_l_part = 0;
    Chassis.thetal_r_part = Chassis.sdr_K[6] * State_Var.thetal_r + Chassis.sdr_K[7] * State_Var.thetald1_r;
    Chassis.thetab_part = Chassis.sdr_K[8] * State_Var.thetab + Chassis.sdr_K[9] * State_Var.thetabd1;

    Chassis.sd_torque = (Chassis.s_part + Chassis.phi_part + Chassis.thetal_l_part + Chassis.thetal_r_part + Chassis.thetab_part);

    Chassis.sd_torque = -constrain(Chassis.sd_torque, -SD_MAX_TORQUE, SD_MAX_TORQUE);

    wb_motor_set_torque(My_Motor[R_Sd_Motor], Chassis.sd_torque);  
    
    // printf("R_S: %f \n", Chassis.sd_torque);
    
    
    // wb_motor_set_torque(My_Motor[R_Wheel_Motor], 0);
    // wb_motor_set_torque(My_Motor[L_Wheel_Motor], 0);
    // wb_motor_set_torque(My_Motor[L_Sd_Motor], 2);
    // wb_motor_set_torque(My_Motor[R_Sd_Motor], 2);
}










