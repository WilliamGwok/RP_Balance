#include <webots/robot.h>
#include <math.h>
#include <stdio.h> 
#include <string.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>  
#include <webots/motor.h>
#include <webots/gyro.h> 
#include <webots/keyboard.h>
#include <webots/accelerometer.h>

#define TIME_STEP                4
#define DEVICE_NAME_LENGTH       20
#define PI                       3.1415926535
#define pi                       3.1415926535
#define abs(x)                   ((x)>0? (x):(-(x)))
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define one(x)		((x)>0? (1):(-1))

#define WHEEL_RADIUS             0.06
/*鑵块暱骞冲潎鍊肩洰鏍囧€�*/
#define TAR_LEG_LENGTH_INITIAL   0.21
#define TAR_ROLL_INITIAL         0
#define LEG_LENGTH_MAX           0.34
#define LEG_LENGTH_MIN           0.11
/*绂诲湴鏀�鎸佸姏锛屽崟浣嶄负鐗�*/
#define OFF_GROUND_SUPPORT       30

/*閿�鐩橀敭鍊煎畾涔�*/
#define GO_FORWARD      'W'
#define GO_BACKWARD     'S'
#define TURN_RIGHT      'D'
#define TURN_LEFT       'A'
#define GO_UP           'O'
#define GO_DOWN         'P'
#define TILT_LEFT       'Q'
#define TILT_RIGHT      'E'
#define TOP_MODE_ON     'F'
#define TOP_MODE_OFF    'G'
#define JUMP            ' '

#define START_RECORD    'N'
#define END_RECORD      'M'

#define KEY_CNT_MAX     50

FILE *fp = NULL;

/*...........................................鐢垫満涓庡叾缂栫爜鍣� begin...........................................*/
typedef enum Motor_List
{
  R_B_Sd_Motor = 0,
  
  R_F_Sd_Motor,
  
  R_Wheel_Motor,
  
  L_B_Sd_Motor,
  
  L_F_Sd_Motor,
  
  L_Wheel_Motor,
  
  Motor_Total_Num,
}Motor_List_e;

typedef struct Motor_struct_t
{
  char name[DEVICE_NAME_LENGTH];
  
  double output_torque;
  
  double distance_now;
  
  double distance_last;
  
  double linear_velocity;
  
  double angular_velocity;
    
}Motor_t;

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


typedef enum Encoder_List
{
  R_B_Sd_Encoder = 0,
  
  R_F_Sd_Encoder,
  
  R_Wheel_Encoder,
  
  L_B_Sd_Encoder,
  
  L_F_Sd_Encoder,
  
  L_Wheel_Encoder,
  
  Encoder_Total_Num,
}Encoder_List_e;

typedef struct Encoder_struct_t
{
  char name[DEVICE_NAME_LENGTH];
  
  double rad_now;
  
  double rad_last;
  
  double rad_diff;  
}Encoder_t;

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

/*...........................................鐢垫満涓庡叾缂栫爜鍣� begin...........................................*/


/*...........................................Imu涓� Gyro begin...........................................*/

WbDeviceTag Imu;//娴嬭�掑害
WbDeviceTag Gyro;//娴嬭�掗€熷害
WbDeviceTag Acce;//娴嬪姞閫熷害

typedef struct Posture_struct_t
{
  double yaw;
  
  double pitch;
  
  double roll;
  
  double yaw_v;
  
  double pitch_v;
  
  double roll_v;
  
  double x_acc;
  
  double y_acc;
  
  double z_acc;
}Posture_t;

Posture_t My_Posture;

/*...........................................Imu涓� Gyro end...........................................*/

/*...........................................PID begin...........................................*/
typedef enum Leg_List
{
  L_Leg = 0,
  
  R_Leg,
  
  Leg_Num,
}Leg_List_e;

typedef struct 
{
  double	target;
  double	measure;
  double 	err;
  double 	last_err;
  double	integral;
  double 	pout;
  double 	iout;
  double 	dout;
  double 	out;
/* 閰嶇疆 */
  double   blind_err;
  double   integral_bias;
  double	kp;
  double 	ki;
  double 	kd;
  double 	integral_max;
  double 	out_max;
}pid_info_t;

typedef struct
{
  pid_info_t *info;
}pid_t;

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
  .kp = 850.f,//850
  .ki = 0.1f,
  .kd = 30000.f,//45000
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

/*...........................................PID end...........................................*/

/*...........................................鐘舵€佸彉閲� begin...........................................*/

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
  
  /*涓�闂村彉閲忕敤 begin*/
  
  double s_now;
  
  double s_last;
  
  double thetal_l_now;
  
  double thetal_l_last;
  
  double thetal_r_now;
  
  double thetal_r_last;
  
  /*涓�闂村彉閲忕敤 end*/
}State_Var_t;

State_Var_t My_State_Var;

typedef enum Link_Var_List
{
  L_Link = 0,
  
  R_Link,
  
  Link_Num,
}Link_Var_List_e;

typedef struct Link_Var_struct_t
{
  double phi1;
  
  double phi2;
  
  double phi3;
  
  double phi4;
  
  double phi0;
  
  double vir_phi0;
  
  double l0; double l0_last; double l0_dot; double l0_dot_last; double l0_dot2;
  
  double l_target;
  
  double xa;  double ya;
  
  double xb;  double yb;
  
  double xc;  double yc;
  
  double xd;  double yd;
  
  double xe;  double ye;
  
  double xp;  double yp;
  
  double mx_l1;  double my_l1;
  
  double mx_l2;  double my_l2;
  
  double mx_l3;  double my_l3;
  
  double mx_l4;  double my_l4;
  
  double centriod_coefficient;
  
  double F_gravity;
  
  double F_inertial;
  
  double F_roll;
  
  double F;
  
  double Fbl;
  
  double F_support;
  
  pid_t* length_pid;
  
  pid_t* roll_pid;
  
  bool off_ground;
  
  bool landed;
  
}Link_Var_t;

Link_Var_t My_Link_Var[Link_Num] = 
{
  [L_Link] = {
    .phi1 = 0,
    .phi4 = 0,
    .length_pid = &Leg_Length_Pid[L_Leg],
    .roll_pid = &Leg_Roll_Pid[L_Leg],
  },
  
  [R_Link] = {
    .phi1 = 0,
    .phi4 = 0,
    .length_pid = &Leg_Length_Pid[R_Leg],
    .roll_pid = &Leg_Roll_Pid[R_Leg],
  },
};
/*...........................................鐘舵€佸彉閲� end...........................................*/


/*...........................................鐩磋吙妯″瀷璁＄畻 begin...........................................*/
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

Straight_Leg_Model_t Straight_Leg_Model = 
{
.wheell_K = {
   -6.9132, -5.5353, -4.9792, -1.6117, -12.7881, -2.7435, -7.7516, -0.4925, -3.1132, -1.3978
},
.wheelr_K = {
   -6.9132, -5.5353, 4.9792, 1.6117, -7.7516, -0.4925, -12.7881, -2.7435, -3.1132, -1.3978
},
.sdl_K = {
   2.9716, 2.2817, -10.0415, -3.4662, 14.3558, 2.9045, -8.3324, -1.7750, -43.2204, -13.9839
},
.sdr_K = {
   2.9716, 2.2817, 10.0415, 3.4662, -8.3324, -1.7750, 14.3558, 2.9045, -43.2204, -13.9839
},
  
  .K_coefficient = {
        {-6.624446048, -21.20046033, 19.20495344, 34.32606716, -20.99469017, -10.09762216, -6.670985518, -15.09956699, 20.21398386, 33.18509248, -33.21275191, -8.482976031, -12.97279636, 19.91353515, -11.9734648, -17.33502754, 3.773559483, 13.65030485, -1.980371367, 3.456951833, -2.303259415, -2.359234029, 0.213775902, 2.690936684, -13.17178739, -54.98608696, 41.13404265, 57.57269215, -1.09905021, -51.22690283, -2.720350527, -2.033943486, 2.246333062, 3.275510536, -2.132529988, -2.567324533, -7.383810831, 18.08664208, -24.89864874, 3.723709288, -40.36715285, 64.09968621, -0.90116328, 3.019807082, -0.699942947, -3.853365737, -0.552603537, 2.1203812, -3.027170644, 2.527637706, -3.470857472, 0.805265739, -5.766150681, 8.852144731, -2.158771016, 1.962244879, 3.122468017, 1.794658904, -8.311592078, -0.91801822},
        {-6.624446048, 19.20495344, -21.20046033, -10.09762216, -20.99469017, 34.32606716, -6.670985518, 20.21398386, -15.09956699, -8.482976031, -33.21275191, 33.18509248, 12.97279636, 11.9734648, -19.91353515, -13.65030485, -3.773559483, 17.33502754, 1.980371367, 2.303259415, -3.456951833, -2.690936684, -0.213775902, 2.359234029, -7.383810831, -24.89864874, 18.08664208, 64.09968621, -40.36715285, 3.723709288, -0.90116328, -0.699942947, 3.019807082, 2.1203812, -0.552603537, -3.853365737, -13.17178739, 41.13404265, -54.98608696, -51.22690283, -1.09905021, 57.57269215, -2.720350527, 2.246333062, -2.033943486, -2.567324533, -2.132529988, 3.275510536, -3.027170644, -3.470857472, 2.527637706, 8.852144731, -5.766150681, 0.805265739, -2.158771016, 3.122468017, 1.962244879, -0.91801822, -8.311592078, 1.794658904},
        {5.228205571, 21.47143293, -36.65262908, -51.75604473, 41.44016441, 34.18060721, 4.832754777, 17.03050994, -33.25200007, -45.94901198, 42.54879271, 30.04708013, -18.16558549, -4.838030893, -16.43896788, 12.01485787, -21.40409878, 21.11812477, -2.882844366, -1.224980018, -2.52614491, 1.980323961, -5.042450292, 2.746721234, 14.22829658, 22.92178555, -12.08350329, -45.47876101, 76.34028147, 9.65404555, 3.540194499, -7.237045077, 3.450891865, 7.129764032, 3.842422174, -5.885076968, -3.229845008, -15.72935581, -20.96943619, 2.756190121, -4.713606117, -1.186356233, -1.762305842, -3.919038748, 3.242813268, 6.301857277, -0.276244615, -4.29450962, -43.40364019, -28.22542453, 29.77475976, 36.01193023, -7.529118179, -32.64661412, -13.61775428, -9.83778803, 7.253041546, 9.497061852, 2.157997701, -7.396576681},
        {5.228205571, -36.65262908, 21.47143293, 34.18060721, 41.44016441, -51.75604473, 4.832754777, -33.25200007, 17.03050994, 30.04708013, 42.54879271, -45.94901198, 18.16558549, 16.43896788, 4.838030893, -21.11812477, 21.40409878, -12.01485787, 2.882844366, 2.52614491, 1.224980018, -2.746721234, 5.042450292, -1.980323961, -3.229845008, -20.96943619, -15.72935581, -1.186356233, -4.713606117, 2.756190121, -1.762305842, 3.242813268, -3.919038748, -4.29450962, -0.276244615, 6.301857277, 14.22829658, -12.08350329, 22.92178555, 9.65404555, 76.34028147, -45.47876101, 3.540194499, 3.450891865, -7.237045077, -5.885076968, 3.842422174, 7.129764032, -43.40364019, 29.77475976, -28.22542453, -32.64661412, -7.529118179, 36.01193023, -13.61775428, 7.253041546, -9.83778803, -7.396576681, 2.157997701, 9.497061852}
    },
};

/*...........................................鐩磋吙妯″瀷璁＄畻 end...........................................*/

/*...........................................鏁磋溅鎺у埗 begin...........................................*/
typedef struct My_Key_struct_t
{
  int w_key;
  
  int a_key;
  
  int s_key;
  
  int d_key;
  
  int o_key;
  
  int p_key;
  
  int k_key;
  
  int l_key;
  
  int n_key;
  
  int m_key;
}My_Key_t;

My_Key_t My_Key;

typedef struct My_Robot_Config_struct_t
{
  double front_speed_max;
  
  double spin_speed_max;
}My_Robot_Config_t;

My_Robot_Config_t My_Robot_Config = 
{
  .front_speed_max = 2.0f,//m/s
  
  .spin_speed_max = 4.2f,//rad/s
};

typedef struct My_Robot_Status_struct_t
{
  bool moving;
  
  bool steady;
  
  bool spinning;
}My_Robot_Status_t;

My_Robot_Status_t My_Robot_Status;

typedef struct My_Robot_struct_t
{
  bool top_mode;
  
  bool jump_flag;

  double speed_target;
  
  double position_target;
  
  double spin_target;
  
  double yaw_target;
  
  double leg_length_target;
  
  double roll_target;
  
  My_Robot_Config_t *config;
  
  My_Robot_Status_t *status;
}My_Robot_t;

My_Robot_t My_Robot = 
{
  .top_mode = false,
  
  .jump_flag = false,

  .leg_length_target = TAR_LEG_LENGTH_INITIAL,
  
  .roll_target = 0,
  
  .config = &My_Robot_Config,
  
  .status = &My_Robot_Status,
};
/*...........................................鏁磋溅鎺у埗 end...........................................*/




/*...........................................鑷�瀹氫箟鍑芥暟澹版槑 begin...........................................*/

void My_Get_All_Device(void);
void My_Raw_Data_Update(void);
void My_Link_Var_Update(void);
void My_State_Var_Update(void);
void My_Data_Update(void);
void My_Print_Debug(void);
void My_Link_Coordinate_Cal(Link_Var_t* Link_Var);
void My_link_Centroid_Coordinate_Cal(Link_Var_t* Link_Var);
double My_Leg_Length_Cal(Link_Var_t* Link_Var);
double My_Phi0_Cal(Link_Var_t* Link_Var);
double My_Virtual_Leg_Rad_Cal(Link_Var_t* Link_Var);
double My_Leg_Length_Strength_Cal(Link_Var_t* Link_Var);
double My_Back_Joint_Torque_Cal(Link_Var_t* Link_Var, double Fl, double Tp);
double My_Front_Joint_Torque_Cal(Link_Var_t* Link_Var, double Fl, double Tp);
double My_Phi2_Cal(Link_Var_t* Link_Var);
double My_Phi3_Cal(Link_Var_t* Link_Var);
void My_Link_Feedforward_Cal(Link_Var_t* Link_Var);
void Leg_Length_Stubborn_Err_Destroyer(void);
void K_Matrix_Fitting_Update(void);
void My_Leg_Length_Different_Cal(void);
void My_Test(Link_Var_t* Link_Var, double Fbl, double Tbl);
void My_Key_React(void);
double My_Roll_Control(Link_Var_t* Link_Var);
void My_Supportive_Power_Cal(Link_Var_t* Link_Var);
void My_Robot_Command_React(void);
double My_Yaw_Target_Process(double target);
double My_Yaw_Zero_Point_Process(double err);
void My_Jump_Target_Process(void);
/*...........................................鑷�瀹氫箟鍑芥暟澹版槑 end...........................................*/
double phi_to_wheel = 0.f;
double phid1_to_wheel = 0.f;
double phi_to_sd = 0.f;
double phid1_to_sd = 0.f;

double thetal_r_to_wheel = 0.f;
double thetald1_r_to_wheel = 0.f;
double thetal_r_to_sd = 0.f;
double thetald1_r_to_sd = 0.f;

double thetal_l_to_wheel = 0.f;
double thetald1_l_to_wheel = 0.f;
double thetal_l_to_sd = 0.f;
double thetald1_l_to_sd = 0.f;


void My_Get_All_Device(void)
{
  Imu = wb_robot_get_device("My_Imu");
  wb_inertial_unit_enable(Imu, TIME_STEP);
  
  Gyro = wb_robot_get_device("My_Gyro");
  wb_gyro_enable(Gyro, TIME_STEP);
  
  Acce = wb_robot_get_device("My_Accelerometer");
  wb_accelerometer_enable(Acce, TIME_STEP);

  /*鐢垫満鍒濆�嬪寲*/
  for(int i = 0; i < Motor_Total_Num; i++)
  {
    Motor[i] = wb_robot_get_device(My_Motor[i].name);
    
    if(i == R_Wheel_Motor || i == L_Wheel_Motor)
    {
      wb_motor_set_position(Motor[i], INFINITY);
    }
  }
  
  /*缂栫爜鍣ㄥ垵濮嬪寲*/
  for(int i = 0; i < Encoder_Total_Num; i++)
  {
    Encoder[i] = wb_robot_get_device(My_Encoder[i].name);
    
    wb_position_sensor_enable(Encoder[i], TIME_STEP);
  }
  
  /*閿�鐩樺垵濮嬪寲*/
  wb_keyboard_enable(TIME_STEP);
}

/**
  * @brief  浼犳劅鍣ㄦ暟鎹�鏇存柊锛屽寘鎷琁mu,Gyro锛岀數鏈虹紪鐮佸櫒
  * @param  None
  * @retval None
  */
void My_Raw_Data_Update(void)
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
  My_Posture.x_acc = acce[0];//宸﹀彸
  My_Posture.y_acc = acce[1];//涓婁笅
  My_Posture.z_acc = acce[2];//鍓嶅悗
  
  for(int i = 0; i < Encoder_Total_Num; i++)
  {
    My_Encoder[i].rad_now = wb_position_sensor_get_value(Encoder[i]);
    
    My_Encoder[i].rad_diff = My_Encoder[i].rad_now - My_Encoder[i].rad_last;
    
    My_Encoder[i].rad_last = My_Encoder[i].rad_now;
  }
}

/**
  * @brief  浜旇繛鏉嗘暟鎹�鏇存柊
  * @param  None
  * @retval None
  */
void My_Link_Var_Update(void)
{
  /*鍏堟洿鏂板悇涓�瑙掑害锛屼互渚夸簬璁＄畻鍚勭偣鍧愭爣*/
  My_Link_Var[R_Link].phi1 = PI - My_Encoder[R_F_Sd_Encoder].rad_now;//1
  
  My_Link_Var[R_Link].phi2 =  My_Phi2_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Var[R_Link].phi3 =  My_Phi3_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Var[R_Link].phi4 = -My_Encoder[R_B_Sd_Encoder].rad_now;//1
  
  My_Link_Var[L_Link].phi1 = PI - My_Encoder[L_F_Sd_Encoder].rad_now;//1
  
  My_Link_Var[L_Link].phi2 =  My_Phi2_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Var[L_Link].phi3 =  My_Phi3_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Var[L_Link].phi4 = -My_Encoder[L_B_Sd_Encoder].rad_now;//1
  
  /*鏇存柊杩炴潌鍚勭偣鍧愭爣*/
  My_Link_Coordinate_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Coordinate_Cal(&My_Link_Var[L_Link]);
  
  My_link_Centroid_Coordinate_Cal(&My_Link_Var[R_Link]);
  
  My_link_Centroid_Coordinate_Cal(&My_Link_Var[L_Link]);
  
  //璁＄畻寰楀埌鐨勮吙闀夸笌鏈熸湜鍊奸潪甯告帴杩戯紝浣嗗洜寤烘ā涓嶅噯纭�锛岃繕鏈変笉灏戣��宸�锛岄渶瑕佷紭鍖栧缓妯�   emm,鏀逛簡杩樻槸涓嶅お瀵癸紝璇�宸�鍦�1cm宸﹀彸  
  //鍙宠吙                    
  My_Link_Var[R_Link].l0 = My_Leg_Length_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Var[R_Link].l0_dot = (My_Link_Var[R_Link].l0 - My_Link_Var[R_Link].l0_last) / (TIME_STEP * 0.001);
  
  My_Link_Var[R_Link].l0_last = My_Link_Var[R_Link].l0;
  
  My_Link_Var[R_Link].l0_dot2 = (My_Link_Var[R_Link].l0_dot - My_Link_Var[R_Link].l0_dot_last) / (TIME_STEP * 0.001);
  
  My_Link_Var[R_Link].l0_dot_last = My_Link_Var[R_Link].l0_dot;
  
  //宸﹁吙
  My_Link_Var[L_Link].l0 = My_Leg_Length_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Var[L_Link].l0 = My_Leg_Length_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Var[L_Link].l0_dot = (My_Link_Var[L_Link].l0 - My_Link_Var[L_Link].l0_last) / (TIME_STEP * 0.001);
  
  My_Link_Var[L_Link].l0_last = My_Link_Var[L_Link].l0;
  
  My_Link_Var[L_Link].l0_dot2 = (My_Link_Var[L_Link].l0_dot - My_Link_Var[L_Link].l0_dot_last) / (TIME_STEP * 0.001);
  
  My_Link_Var[L_Link].l0_dot_last = My_Link_Var[L_Link].l0_dot;
                           
  
  
  
  
  
  My_Link_Var[R_Link].phi0 = My_Phi0_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Var[L_Link].phi0 = My_Phi0_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Var[R_Link].vir_phi0 = My_Virtual_Leg_Rad_Cal(&My_Link_Var[R_Link]);//1
  
  My_Link_Var[L_Link].vir_phi0 = My_Virtual_Leg_Rad_Cal(&My_Link_Var[L_Link]);//1
}

/**
  * @brief  鐘舵€佸彉閲忔洿鏂�
  * @param  None
  * @retval None
  */
void My_State_Var_Update(void)
{
  /*璺�绋嬩笌閫熷害*/
  My_State_Var.s = 0.5 * WHEEL_RADIUS * (My_Encoder[R_Wheel_Motor].rad_now + My_Encoder[L_Wheel_Motor].rad_now);
  
  My_State_Var.s_now = My_State_Var.s;
  
  My_State_Var.sd1 = (My_State_Var.s_now - My_State_Var.s_last) / (TIME_STEP * 0.001);
  
  My_State_Var.s_last = My_State_Var.s_now;
  
  /*鍋忚埅瑙掍笌鍋忚埅瑙掗€熷害*/
  My_State_Var.phi = My_Posture.yaw;
  
  My_State_Var.phid1 = My_Posture.yaw_v;
  
  /*宸﹁吙鍊炬枩瑙掑害涓庤�掗€熷害*/
  My_State_Var.thetal_l = (My_Link_Var[L_Link].vir_phi0 + My_Posture.pitch);
  
  My_State_Var.thetal_l_now = My_State_Var.thetal_l;
  
  My_State_Var.thetald1_l = (My_State_Var.thetal_l_now - My_State_Var.thetal_l_last) / (TIME_STEP * 0.001);
  
  My_State_Var.thetal_l_last = My_State_Var.thetal_l_now;
  
   /*鍙宠吙鍊炬枩瑙掑害涓庤�掗€熷害*/
  My_State_Var.thetal_r = (My_Link_Var[R_Link].vir_phi0 + My_Posture.pitch);
  
  My_State_Var.thetal_r_now = My_State_Var.thetal_r;
  
  My_State_Var.thetald1_r = (My_State_Var.thetal_r_now - My_State_Var.thetal_r_last) / (TIME_STEP * 0.001);
  
  My_State_Var.thetal_r_last = My_State_Var.thetal_r_now;
  
  /*淇�浠拌�掍笌淇�浠拌�掗€熷害*/
  My_State_Var.thetab = My_Posture.pitch;
  
  My_State_Var.thetabd1 = My_Posture.pitch_v;
}

/**
  * @brief  鏁磋溅鏁版嵁鏇存柊
  * @param  None
  * @retval None
  */
void My_Data_Update(void)
{
  My_Raw_Data_Update();
  
  My_Link_Var_Update();
  
  My_State_Var_Update();
  
  // K_Matrix_Fitting_Update();
}

/**
  * @brief  鍏宠妭鍔涚煩璁＄畻
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_Torque_Cal(void)
{
  My_Supportive_Power_Cal(&My_Link_Var[R_Link]);
  
  My_Supportive_Power_Cal(&My_Link_Var[L_Link]);
  
  // printf("%f\t %d\n", My_Link_Var[L_Link].F_support ,My_Link_Var[L_Link].off_ground);

  My_Leg_Length_Different_Cal();

  //鍏堟祴璇曞畾鑵块暱鎯呭喌涓嬬殑鏁堟灉锛屾晠鍦╩atlab鍐呰�＄畻l0鐨勮川蹇冨潗鏍�
  double Fl = 0.f, Tp = 0.f; 
  
  /*鍙宠竟鍏宠妭*/
  My_Link_Var[R_Link].F = My_Leg_Length_Strength_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Feedforward_Cal(&My_Link_Var[R_Link]);
    
  // if(My_Link_Var[R_Link].off_ground == false)
  // {
    My_Link_Var[R_Link].Fbl = My_Link_Var[R_Link].F + My_Link_Var[R_Link].F_gravity + My_Link_Var[R_Link].F_inertial - My_Roll_Control(&My_Link_Var[R_Link]);//娉ㄦ剰姝ｈ礋,渚у悜鎯�鎬у姏鏈夊緟楠岃瘉
  // }
  // else
  // {
    // My_Link_Var[R_Link].Fbl = My_Link_Var[R_Link].F + My_Link_Var[R_Link].F_gravity - My_Link_Var[R_Link].F_inertial;
  // }
  
  Tp = Straight_Leg_Model.Tp_r;
  
  My_Test(&My_Link_Var[R_Link], Fl, Tp);

  My_Motor[R_F_Sd_Motor].output_torque = My_Back_Joint_Torque_Cal(&My_Link_Var[R_Link], My_Link_Var[R_Link].Fbl, Tp);
  
  My_Motor[R_B_Sd_Motor].output_torque = My_Front_Joint_Torque_Cal(&My_Link_Var[R_Link], My_Link_Var[R_Link].Fbl, Tp);
  
  /*宸﹁竟鍏宠妭*/
  My_Link_Var[L_Link].F = My_Leg_Length_Strength_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Feedforward_Cal(&My_Link_Var[L_Link]);
  
  // if(My_Link_Var[L_Link].off_ground == false)
  // {
    My_Link_Var[L_Link].Fbl = My_Link_Var[L_Link].F + My_Link_Var[L_Link].F_gravity - My_Link_Var[L_Link].F_inertial + My_Roll_Control(&My_Link_Var[L_Link]);//娉ㄦ剰姝ｈ礋
  // }
  // else
  // {
    // My_Link_Var[L_Link].Fbl = My_Link_Var[L_Link].F + My_Link_Var[L_Link].F_gravity + My_Link_Var[L_Link].F_inertial;
  // }
  
  
  Tp = Straight_Leg_Model.Tp_l; 

  My_Motor[L_F_Sd_Motor].output_torque = My_Back_Joint_Torque_Cal(&My_Link_Var[L_Link], My_Link_Var[L_Link].Fbl, Tp);
  
  My_Motor[L_B_Sd_Motor].output_torque = My_Front_Joint_Torque_Cal(&My_Link_Var[L_Link], My_Link_Var[L_Link].Fbl, Tp);
}

/**
  * @brief  璋冭瘯鎵撳嵃鐢�
  * @param  None
  * @retval None
  */
void My_Print_Debug(void)
{
  //浜旇繛鏉哾ebug鎵撳嵃锛�
  // printf("%f\t %f\t %f\t %f\n", My_Link_Var[R_Link].xb, My_Link_Var[R_Link].yb, My_Link_Var[R_Link].xd, My_Link_Var[R_Link].yd);
  
  // printf("%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", My_Link_Var[R_Link].mx_l1, My_Link_Var[R_Link].my_l1, 
                                                    // My_Link_Var[R_Link].mx_l2, My_Link_Var[R_Link].my_l2, 
                                                    // My_Link_Var[R_Link].mx_l3, My_Link_Var[R_Link].my_l3, 
                                                    // My_Link_Var[R_Link].mx_l4, My_Link_Var[R_Link].my_l4);
                                                    
  // printf("%f\t %f\t %f\t %f\t %f\n", My_Link_Var[R_Link].xp, My_Link_Var[R_Link].yp, My_Link_Var[R_Link].xc, My_Link_Var[R_Link].yc, My_Link_Var[R_Link].centriod_coefficient);
  
  // printf("%f\t %f\n", My_Link_Var[R_Link].F_gravity, My_Link_Var[R_Link].F_inertial);
  
  // printf("%f\t %f\t %f\t %f\t %f\t %f\t %f\n",(My_Link_Var[R_Link].phi0 * 57.3), 
  // (My_Link_Var[R_Link].phi1*57.3), (My_Link_Var[R_Link].phi2*57.3), (My_Link_Var[R_Link].phi3*57.3),
  // (My_Link_Var[R_Link].phi4*57.3), My_Link_Var[R_Link].l0, My_Link_Var[R_Link].F);
  
  // printf("%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n",My_Link_Var[R_Link].phi0, 
  // My_Link_Var[R_Link].phi1, My_Link_Var[R_Link].phi2, My_Link_Var[R_Link].phi3,
  // My_Link_Var[R_Link].phi4, My_Link_Var[R_Link].l0, Straight_Leg_Model.Tp_r, My_Link_Var[R_Link].Fbl, My_Motor[R_F_Sd_Motor].output_torque, My_Motor[R_B_Sd_Motor].output_torque);
  printf("%f\n", My_Link_Var[R_Link].vir_phi0);
  // printf("%f \t %f \t %f\n", My_Motor[R_F_Sd_Motor].output_torque, My_Motor[R_B_Sd_Motor].output_torque, My_Link_Var[R_Link].l0);
  
  //鏁磋溅debug鎵撳嵃
  printf("%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", My_State_Var.s, My_State_Var.sd1, My_State_Var.phi, \
  My_State_Var.phid1, My_State_Var.thetal_l, My_State_Var.thetald1_l, My_State_Var.thetal_r, My_State_Var.thetald1_r);
  // printf("%f\t %f\t %f\t %f\n", Straight_Leg_Model.Tp_r, Straight_Leg_Model.Tw_r, Straight_Leg_Model.Tp_l, Straight_Leg_Model.Tw_l);
  // printf("%f\t %f\n",Straight_Leg_Model.Tp_r, My_State_Var.thetal_r);
  // printf("%f\t %f\n", My_Link_Var[R_Link].yp, My_Link_Var[L_Link].l0);
  
  // printf("%f\t %f\t %f\n", My_Posture.x_acc, My_Posture.y_acc, My_Posture.z_acc);
  // printf("%f\n", My_Link_Var[R_Link].l0_dot_last);
  // printf("%d\n", My_Key.w_key);
  fprintf(fp, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", My_State_Var.phi, My_Robot.yaw_target, \
                                                                             My_State_Var.phid1, 0.f, \
                                                                             phi_to_wheel, 0.f, \
                                                                             phid1_to_wheel, 0.f, \
                                                                             phi_to_sd, 0.f, \
                                                                             phid1_to_sd, 0.f,\
                                                                             
                                                                             My_State_Var.thetal_r, 0.f,\
                                                                             My_State_Var.thetald1_r, 0.f,\
                                                                             thetal_r_to_wheel, 0.f,\
                                                                             thetald1_r_to_wheel, 0.f,\
                                                                             thetal_r_to_sd, 0.f,\
                                                                             thetald1_r_to_sd, 0.f,\
                                                                             
                                                                             My_State_Var.thetal_l, 0.f,\
                                                                             My_State_Var.thetald1_l, 0.f,\
                                                                             thetal_l_to_wheel, 0.f,\
                                                                             thetald1_l_to_wheel, 0.f,\
                                                                             thetal_l_to_sd, 0.f,\
                                                                             thetald1_l_to_sd, 0.f,\
                                                                             
                                                                             My_State_Var.s, My_Robot.position_target,\
                                                                             My_Link_Var[R_Link].F_support, My_Link_Var[L_Link].F_support,\
                                                                             My_Posture.roll, 0.f,\
                                                                             My_Posture.pitch, 0.f,\
                                                                             My_Link_Var[R_Link].F_inertial, My_Link_Var[L_Link].F_inertial);
                                                    
   // printf("%f\t %f\t %f\t %f\t %f\t %f\n", My_Link_Var[R_Link].Fbl, My_Link_Var[R_Link].vir_phi0, My_Posture.y_acc, My_Link_Var[R_Link].centriod_coefficient, My_Link_Var[R_Link].l0_dot2, My_Link_Var[R_Link].l0_dot);
  // printf("%f\t %f\t %f\t %f\t %f\t %f\n", My_Encoder[R_F_Sd_Encoder].rad_now, My_Encoder[R_B_Sd_Encoder].rad_now, \
                                // My_Link_Var[R_Link].phi1, My_Link_Var[R_Link].phi2,\
                                // My_Link_Var[R_Link].phi3, My_Link_Var[R_Link].phi4);
}


/*Debug*/
/*
23/10/9/10:04 浜旇繛鏉唒hi2鏋佹€ч敊璇�,phi3鏋佹€ч敊璇�

23/10/19/20:51 鎬庝箞鍒ゆ柇鍒濆�嬬姸鎬佺殑绂诲湴鐘舵€侊紵
               鑵块暱鐩�鏍囧€肩殑闄愬箙瑕佹斁瀵�

23/10/20/10:16 椋炲潯鏃舵病鏈夋敹鑵�

23/10/24/16:38  渚у悜鎯�鎬у姏娌″紕鎳傦紝鐢ㄥソ鍍忎篃娌＄敤瀵�

23/10/24/17:04  鍚�鍔ㄦ椂鏈夎吙绂诲湴
*/







/*...........................................宸ュ叿鍑芥暟 begin...........................................*/
//鏉嗛暱
#define l1    0.15
#define l2    0.27
#define l3    0.27
#define l4    0.15
#define l5    0.15

//鍚勬潌璐ㄥ績绯绘暟
#define l1_cen 0.5
#define l2_cen 0.5
#define l3_cen 0.5
#define l4_cen 0.5
#define l5_cen 0.5

//鍚勬潌璐ㄩ噺
#define m_l1 0.52
#define m_l2 0.2
#define m_l3 0.2
#define m_l4 0.52

//鍗曡吙璐ㄩ噺锛屽洓鏉嗘€诲拰
#define m_l (m_l1 + m_l2 + m_l3 + m_l4)

//鏈轰綋璐ㄩ噺
#define mb 10.0f
#define g  9.81

//鏁磋溅鏃嬭浆鍗婂緞
#define Rl 0.18

/**
  * @brief  浜旇繛鏉嗗悇鐐瑰潗鏍囪�＄畻
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_Link_Coordinate_Cal(Link_Var_t* Link_Var)
{
  double phi1 = 0.f, phi4 = 0.f;

  phi1 = Link_Var->phi1;
  
  phi4 = Link_Var->phi4;

  Link_Var->xa = 0;
  
  Link_Var->ya = 0;
  
  Link_Var->xb = l1*cos(phi1);
  
  Link_Var->yb = l1*sin(phi1);

  Link_Var->xc = l1*cos(phi1) + l2*cos(2*atan2((pow(((pow(l1,2) - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2,2) + 2*l2*l3 \
  - pow(l3,2) + pow(l4,2) + 2*cos(phi4)*l4*l5 + pow(l5,2))*(- pow(l1,2) + 2*cos(phi1 - phi4)*l1*l4 + 2*cos(phi1)*l1*l5 + pow(l2,2) + 2*l2*l3 \
  + pow(l3,2) - pow(l4,2) - 2*cos(phi4)*l4*l5 - pow(l5,2))), 0.5) - 2*l1*l2*sin(phi1) + 2*l2*l4*sin(phi4)), (pow(l1,2) - 2*cos(phi1)*l1*l2 \
  - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 + pow(l2,2) + 2*cos(phi4)*l2*l4 + 2*l2*l5 - pow(l3,2) + pow(l4,2) + 2*cos(phi4)*l4*l5 + pow(l5,2))));
                           
  Link_Var->yc = l2*sin(2*atan2((pow(((pow(l1, 2) - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2, 2) + 2*l2*l3 - pow(l3, 2) + pow(l4, 2) \
  + 2*cos(phi4)*l4*l5 + pow(l5, 2))*(- pow(l1, 2) + 2*cos(phi1 - phi4)*l1*l4 + 2*cos(phi1)*l1*l5 + pow(l2, 2) + 2*l2*l3 + pow(l3, 2) - pow(l4, 2) \
  - 2*cos(phi4)*l4*l5 - pow(l5, 2))), 0.5) - 2*l1*l2*sin(phi1) + 2*l2*l4*sin(phi4)), (pow(l1, 2) - 2*cos(phi1)*l1*l2 - 2*cos(phi1 - phi4)*l1*l4 \
  - 2*cos(phi1)*l1*l5 + pow(l2, 2) + 2*cos(phi4)*l2*l4 + 2*l2*l5 - pow(l3, 2) + pow(l4, 2) + 2*cos(phi4)*l4*l5 + pow(l5, 2)))) + l1*sin(phi1);
  
  Link_Var->xd = l5 + l4*cos(phi4);
  
  Link_Var->yd = l4*sin(phi4);
  
  Link_Var->xe = l5;
  
  Link_Var->ye = 0;
}

/**
  * @brief  浜旇繛鏉嗗悇鏉嗚川蹇冨潗鏍囪�＄畻
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_link_Centroid_Coordinate_Cal(Link_Var_t* Link_Var)
{
  double xa = 0.f, xb = 0.f, xc = 0.f, xd = 0.f, xe = 0.f;
  double ya = 0.f, yb = 0.f, yc = 0.f, yd = 0.f, ye = 0.f;
  
  xa = Link_Var->xa;  ya = Link_Var->ya;
  xb = Link_Var->xb;  yb = Link_Var->yb;
  xc = Link_Var->xc;  yc = Link_Var->yc;
  xd = Link_Var->xd;  yd = Link_Var->yd;
  xe = Link_Var->xe;  ye = Link_Var->ye;
  
  Link_Var->mx_l1 = l1_cen*(xb - xa) + xa;
  Link_Var->my_l1 = l1_cen*(yb - ya) + ya;
  Link_Var->mx_l2 = l2_cen*(xc - xb) + xb;
  Link_Var->my_l2 = l2_cen*(yc - yb) + yb;
  Link_Var->mx_l3 = l3_cen*(xd - xc) + xc;
  Link_Var->my_l3 = l3_cen*(yd - yc) + yc;
  Link_Var->mx_l4 = l4_cen*(xe - xd) + xd;
  Link_Var->my_l4 = l4_cen*(ye - yd) + yd;
  
  Link_Var->xp = (Link_Var->mx_l1*m_l1 + Link_Var->mx_l2*m_l2 + Link_Var->mx_l3*m_l3 + Link_Var->mx_l4*m_l4) / (m_l1 + m_l2 + m_l3 + m_l4);
  Link_Var->yp = (Link_Var->my_l1*m_l1 + Link_Var->my_l2*m_l2 + Link_Var->my_l3*m_l3 + Link_Var->my_l4*m_l4) / (m_l1 + m_l2 + m_l3 + m_l4);
  
  Link_Var->centriod_coefficient = sqrt(pow((Link_Var->xc - Link_Var->xp), 2) + pow((Link_Var->yc - Link_Var->yp), 2)) \
                                   /pow( (pow((Link_Var->xc - l5/2), 2) + pow(Link_Var->yc, 2) ) , 0.5);
                                   
  Link_Var->centriod_coefficient = 1 - Link_Var->centriod_coefficient;
}

/**
  * @brief  绛夋晥鑵块暱璁＄畻
  * @param  Link_Var_t* Link_Var
  * @retval 瀵瑰簲鑵块暱
  */
double My_Leg_Length_Cal(Link_Var_t* Link_Var)
{
  double l0 = 0.f;
                           
  l0 = pow( (pow((Link_Var->xc - l5/2), 2) + pow(Link_Var->yc, 2) ) , 0.5);
  
  return l0;
}

/**
  * @brief  璁＄畻绛夋晥鑵垮叧鑺傝�掑害phi0
  * @param  Link_Var_t* Link_Var
  * @retval 瀵瑰簲phi0
  */
double My_Phi0_Cal(Link_Var_t* Link_Var)
{
  double phi0 = 0.f, xc = 0.f, yc = 0.f;
  
  xc = Link_Var->xc;
  
  yc = Link_Var->yc;
  
  phi0 = atan2(yc, (xc - l5/2));
  
  // phi0 = -(phi0 - 1.5708);//涓轰簡涓庣洿鑵挎ā鍨嬬粺涓€
  /*涓嶅�癸紒锛侊紒锛侊紒锛侊紒锛侊紒搴旇�ュ厛婊¤冻浜旇繛鏉嗙殑瑙ｇ畻锛屽啀鍦ㄧ姸鎬佸彉閲忚�＄畻涓�婊¤冻鐩磋吙妯″瀷锛侊紒锛侊紒*/
                  
  return phi0;
}

/**
  * @brief  璁＄畻绛夋晥鑵垮叧鑺傝�掑害phi2
  * @param  Link_Var_t* Link_Var
  * @retval 瀵瑰簲phi2
  */
double My_Phi2_Cal(Link_Var_t* Link_Var)
{
  double phi1 = 0.f, phi4 = 0.f, phi2 = 0.f, son = 0.f, mother = 0.f;
  
  phi1 = Link_Var->phi1;
  
  phi4 = Link_Var->phi4;
  
  son = (pow(((pow(l1, 2) - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2, 2) + 2*l2*l3 - pow(l3, 2) + pow(l4, 2) \
  + 2*cos(phi4)*l4*l5 + pow(l5, 2))*(- pow(l1, 2) + 2*cos(phi1 - phi4)*l1*l4 + 2*cos(phi1)*l1*l5 + pow(l2, 2) + 2*l2*l3 + pow(l3, 2) \
  - pow(l4, 2) - 2*cos(phi4)*l4*l5 - pow(l5, 2))), 0.5)- 2*l1*l2*sin(phi1) + 2*l2*l4*sin(phi4));
  
  mother = (pow(l1, 2) - 2*cos(phi1)*l1*l2 - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 + pow(l2, 2) + 2*cos(phi4)*l2*l4 + 2*l2*l5 \
  - pow(l3, 2) + pow(l4, 2) + 2*cos(phi4)*l4*l5 + pow(l5, 2));
  
  phi2 = 2*atan2(son, mother);
  
  return phi2;
}
  
double My_Phi3_Cal(Link_Var_t* Link_Var)
{
  double phi1 = 0.f, phi4 = 0.f, phi3 = 0.f, son = 0.f, mother = 0.f;
  
  phi1 = Link_Var->phi1;
  
  phi4 = Link_Var->phi4;
  
  son = (pow(((pow(l1, 2) - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2, 2) + 2*l2*l3 - pow(l3, 2) \
  + pow(l4, 2) + 2*cos(phi4)*l4*l5 + pow(l5, 2))*(- pow(l1, 2) + 2*cos(phi1 - phi4)*l1*l4 + 2*cos(phi1)*l1*l5 \
  + pow(l2, 2) + 2*l2*l3 + pow(l3, 2) - pow(l4, 2) - 2*cos(phi4)*l4*l5 - pow(l5, 2))), 0.5)- 2*l1*l3*sin(phi1) + 2*l3*l4*sin(phi4));
  
  mother = (pow(l1, 2) + 2*cos(phi1)*l1*l3 - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2, 2) + pow(l3, 2) \
  - 2*cos(phi4)*l3*l4 - 2*l3*l5 + pow(l4, 2) + 2*cos(phi4)*l4*l5 + pow(l5, 2));
  
  phi3 = -2*atan(son/mother);//杩欓噷鐢╝tan2涓嶅お瀵�
  
  return phi3;
}

/**
  * @brief  娌胯吙鏂瑰悜鍔涘墠棣堣ˉ鍋胯�＄畻
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_Link_Feedforward_Cal(Link_Var_t* Link_Var)
{
  Link_Var->F_gravity = (0.5*mb + Link_Var->centriod_coefficient*m_l)*g*cos(Link_Var->phi0 - PI/2);
  
  Link_Var->F_inertial = (0.5*mb + Link_Var->centriod_coefficient*m_l)*(Link_Var->l0 / (2*Rl))*My_State_Var.phid1*My_State_Var.sd1;
}

/**
  * @brief  灏唒hi0鍑嗘崲涓虹被浼肩紪鐮佸櫒鐨勬晥鏋�
  杞�鎹㈠墠锛�
  0           0
  
      90/-90
  鍑嗘崲鍚庯細
  90          -90
  
        0/0
  * @param  Link_Var_t* Link_Var
  * @retval 铏氭嫙鏉嗙紪鐮佸櫒瑙掑害
  */
double My_Virtual_Leg_Rad_Cal(Link_Var_t* Link_Var)
{
  double phi0 = Link_Var->phi0;
  
  double ans = 0.f;
  
  ans = -(phi0 - 1.5708);
  
  return ans;
}

/**
  * @brief  鍗曠幆PID鎺у埗
  * @param  pid_info_t *pid
  * @retval None
  */
void single_pid_ctrl(pid_info_t *pid)
{
  pid->err = pid->target - pid->measure;
  
  if(abs(pid->err)<=(pid->blind_err))
    pid->err = 0;

  pid->integral += pid->err;
  pid->integral = constrain(pid->integral, -pid->integral_max+pid->integral_bias, pid->integral_max+pid->integral_bias);
	
  pid->pout = pid->kp * pid->err;
  pid->iout = pid->ki * pid->integral;
  pid->dout = pid->kd * (pid->err - pid->last_err);

  pid->out = pid->pout + pid->iout + pid->dout;
  pid->out = constrain(pid->out, -pid->out_max, pid->out_max);

  pid->last_err = pid->err;
}

/**
  * @brief  璁＄畻涓嶅悓鑵块暱涓嬫墍闇€娌挎潌鏂瑰悜鍔汧
  * @param  Link_Var_t* Link_Var
  * @retval 鍔汧
  */
double My_Leg_Length_Strength_Cal(Link_Var_t* Link_Var)
{
  double F = 0.f;
  
  Link_Var->length_pid->info->measure = (My_Link_Var[R_Link].l0 + My_Link_Var[L_Link].l0)*0.5f;
  
  Link_Var->length_pid->info->target = Link_Var->l_target;
  
  single_pid_ctrl(Link_Var->length_pid->info);
  
  F = Link_Var->length_pid->info->out;
  
  return F;
}

/**
  * @brief  roll鎺у埗
  * @param  Link_Var_t* Link_Var
  * @retval 鍔汧
  * @note  鍙宠吙鍑忓乏鑵垮姞
  */
double My_Roll_Control(Link_Var_t* Link_Var)
{
  double F = 0.f;
  
  Link_Var->roll_pid->info->measure = My_Posture.roll;
  
  Link_Var->roll_pid->info->target = TAR_ROLL_INITIAL + My_Robot.roll_target;

  single_pid_ctrl(Link_Var->roll_pid->info);
  
  F = Link_Var->roll_pid->info->out;
  
  return F;
}

/**
  * @brief  璧嬩簣鑵块暱鐩�鏍囧€硷紝鍐椾綑鍑芥暟锛屾湁寰呬紭鍖�
  * @param  None
  * @retval None
  */
void My_Leg_Length_Different_Cal(void)
{
    My_Jump_Target_Process();
    
    My_Link_Var[R_Link].l_target = My_Robot.leg_length_target;
    
    My_Link_Var[L_Link].l_target = My_Robot.leg_length_target; 
}

/**
  * @brief  璁＄畻杞﹀悗鍏宠妭鐢垫満杈撳嚭鍔涚煩
  * @param  Link_Var_t* Link_Var, double Fl, double Tp
  * @retval 鍚庡叧鑺傝緭鍑哄姏鐭㏕1
  */
double My_Back_Joint_Torque_Cal(Link_Var_t* Link_Var, double Fl, double Tp)
{
  double phi0 = 0.f, phi2 = 0.f, phi3 = 0.f, phi4 = 0.f, l0 = 0.f;
  
  phi0 = Link_Var->phi0;
  
  phi2 = Link_Var->phi2;
  
  phi3 = Link_Var->phi3;
  
  phi4 = Link_Var->phi4;
  
  l0 = Link_Var->l0;
  
  double T_1 = - (Fl*l4*sin(phi0 - phi2)*sin(phi3 - phi4))/sin(phi2 - phi3) \
  - (Tp*l4*cos(phi0 - phi2)*sin(phi3 - phi4))/(l0*sin(phi2 - phi3));

  return T_1;
}

/**
  * @brief  璁＄畻杞﹀墠鍏宠妭鐢垫満杈撳嚭鍔涚煩
  * @param  Link_Var_t* Link_Var, double Fl, double Tp
  * @retval 鍓嶅叧鑺傝緭鍑哄姏鐭㏕2
  */
double My_Front_Joint_Torque_Cal(Link_Var_t* Link_Var, double Fl, double Tp)
{
  double phi0 = 0.f, phi1 = 0.f, phi2 = 0.f, phi3 = 0.f, l0 = 0.f;
  
  phi0 = Link_Var->phi0;
  
  phi1 = Link_Var->phi1;
  
  phi2 = Link_Var->phi2;
  
  phi3 = Link_Var->phi3;
  
  l0 = Link_Var->l0;

  double T_2 = - (Fl*l1*sin(phi0 - phi3)*sin(phi1 - phi2))/sin(phi2 - phi3) - (Tp*l1*cos(phi0 \
  - phi3)*sin(phi1 - phi2))/(l0*sin(phi2 - phi3));  
  
  return T_2;
}

void Straight_Leg_Model_Cal(void)
{
  Straight_Leg_Model_t *Ace = &Straight_Leg_Model;
  
  /*椹卞姩杞�鎺у埗*/
  /*宸�*/
  // printf("%f\n", My_State_Var.phi);
  Ace->s_part = Ace->wheell_K[0] * (My_State_Var.s - My_Robot.position_target) + Ace->wheell_K[1] * My_State_Var.sd1;//閫熷害鎺у埗
  Ace->phi_part = Ace->wheell_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - My_Robot.yaw_target) + Ace->wheell_K[3] * My_State_Var.phid1;  
  Ace->thetal_l_part = Ace->wheell_K[4] * My_State_Var.thetal_l + Ace->wheell_K[5] * My_State_Var.thetald1_l;
  Ace->thetal_r_part = Ace->wheell_K[6] * My_State_Var.thetal_r + Ace->wheell_K[7] * My_State_Var.thetald1_r;
  Ace->thetab_part = Ace->wheell_K[8] * My_State_Var.thetab + Ace->wheell_K[9] * My_State_Var.thetabd1;
  
  Ace->Tw_l = -(Ace->s_part + Ace->phi_part + Ace->thetal_l_part + Ace->thetal_r_part + Ace->thetab_part);
  
  if(My_Link_Var[L_Link].off_ground == true)
  {
    Ace->Tw_l = 0;
  }
  
  printf("%f\t %f\t %f\t flag %f\n", Ace->Tw_l, (My_State_Var.s - My_Robot.position_target), -Ace->thetal_l_part, Ace->wheell_K[0]);
  
  Ace->Tw_l = constrain(Ace->Tw_l, -4, 4);
  
  /*鍙�*/
  Ace->s_part = Ace->wheelr_K[0] * (My_State_Var.s - My_Robot.position_target) + Ace->wheelr_K[1] * My_State_Var.sd1;//閫熷害鎺у埗
  Ace->phi_part = Ace->wheelr_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - My_Robot.yaw_target) + Ace->wheelr_K[3] * My_State_Var.phid1;
  Ace->thetal_l_part = Ace->wheelr_K[4] * My_State_Var.thetal_l + Ace->wheelr_K[5] * My_State_Var.thetald1_l;
  Ace->thetal_r_part = Ace->wheelr_K[6] * My_State_Var.thetal_r + Ace->wheelr_K[7] * My_State_Var.thetald1_r;
  Ace->thetab_part = Ace->wheelr_K[8] * My_State_Var.thetab + Ace->wheelr_K[9] * My_State_Var.thetabd1;  
  
  phi_to_wheel = -Ace->wheelr_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - My_Robot.yaw_target);
  phid1_to_wheel = -Ace->wheelr_K[3] * My_State_Var.phid1;
  thetal_r_to_wheel = -Ace->wheelr_K[6] * My_State_Var.thetal_r;
  thetald1_r_to_wheel = -Ace->wheelr_K[7] * My_State_Var.thetald1_r;
  thetal_l_to_wheel = -Ace->wheelr_K[4] * My_State_Var.thetal_l;
  thetald1_l_to_wheel = -Ace->wheelr_K[5] * My_State_Var.thetald1_l;
  
  Ace->Tw_r = -(Ace->s_part + Ace->phi_part + Ace->thetal_l_part + Ace->thetal_r_part + Ace->thetab_part);
  
  if(My_Link_Var[R_Link].off_ground == true)
  {
    Ace->Tw_r = 0;
  }
  
  Ace->Tw_r = constrain(Ace->Tw_r, -4, 4);
  
  /*鍏宠妭鍔涜�＄畻*/
  /*宸�*/
  Ace->s_part = Ace->sdl_K[0] * (My_State_Var.s - My_Robot.position_target) + Ace->sdl_K[1] * My_State_Var.sd1;//閫熷害鎺у埗
  Ace->phi_part = Ace->sdl_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - My_Robot.yaw_target) + Ace->sdl_K[3] * My_State_Var.phid1;
  Ace->thetal_l_part = Ace->sdl_K[4] * My_State_Var.thetal_l + Ace->sdl_K[5] * My_State_Var.thetald1_l;
  Ace->thetal_r_part = Ace->sdl_K[6] * My_State_Var.thetal_r + Ace->sdl_K[7] * My_State_Var.thetald1_r;
  Ace->thetab_part = Ace->sdl_K[8] * My_State_Var.thetab + Ace->sdl_K[9] * My_State_Var.thetabd1;
  
  if(My_Link_Var[L_Link].off_ground == true)
  {
    Ace->s_part = 0; Ace->phi_part = 0; Ace->thetab_part = 0; Ace->thetal_r_part = 0;
  }
  
  Ace->Tp_l = -(Ace->s_part + Ace->phi_part + Ace->thetal_l_part + Ace->thetal_r_part + Ace->thetab_part);
  
  Ace->Tp_l = constrain(Ace->Tp_l, -25, 25);
  
  /*鍙�*/
  Ace->s_part = Ace->sdr_K[0] * (My_State_Var.s - My_Robot.position_target) + Ace->sdr_K[1] * My_State_Var.sd1;//閫熷害鎺у埗
  Ace->phi_part = Ace->sdr_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - My_Robot.yaw_target) + Ace->sdr_K[3] * My_State_Var.phid1;
  Ace->thetal_l_part = Ace->sdr_K[4] * My_State_Var.thetal_l + Ace->sdr_K[5] * My_State_Var.thetald1_l;
  Ace->thetal_r_part = Ace->sdr_K[6] * My_State_Var.thetal_r + Ace->sdr_K[7] * My_State_Var.thetald1_r;
  Ace->thetab_part = Ace->sdr_K[8] * My_State_Var.thetab + Ace->sdr_K[9] * My_State_Var.thetabd1;
  
  phi_to_sd = -Ace->sdr_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - My_Robot.yaw_target);
  phid1_to_sd = -Ace->sdr_K[3] * My_State_Var.phid1;
  thetal_r_to_sd = -Ace->sdr_K[6] * My_State_Var.thetal_r;
  thetald1_r_to_sd = -Ace->sdr_K[7] * My_State_Var.thetald1_r;
  thetal_l_to_sd = -Ace->sdr_K[4] * My_State_Var.thetal_l;
  thetald1_l_to_sd = -Ace->sdr_K[5] * My_State_Var.thetald1_l;
  
  if(My_Link_Var[R_Link].off_ground == true)
  {
    Ace->s_part = 0; Ace->phi_part = 0; Ace->thetab_part = 0; Ace->thetal_l_part = 0;
  }
  
  // printf("%f\t %d\t %f\n", Ace->thetal_l_part, My_Link_Var[R_Link].off_ground, My_Link_Var[R_Link].F_support);
  
  Ace->Tp_r = -(Ace->s_part + Ace->phi_part + Ace->thetal_l_part + Ace->thetal_r_part + Ace->thetab_part);
  
  Ace->Tp_r = constrain(Ace->Tp_r, -25, 25);
}

/**
  * @brief  鏍规嵁瀹炴椂鑵块暱鎷熷悎K鐭╅樀锛屽湪鑵块暱鏇存柊鍚庤皟鐢�(鐢ㄦ灇涓惧彲浠ョ畝鍖栦唬鐮�)
  * @param  None
  * @retval None
  * @note   p00 + p10*ll + p01*lr + p20*ll^2 + p11*ll*lr + p02*lr^2
  */
void K_Matrix_Fitting_Update(void)
{
  double lr = My_Link_Var[R_Link].l0, ll = My_Link_Var[L_Link].l0;

  /*宸﹁疆*/
  for(int i = 0; i < 10; i++)
  {
    int m = 6 * i;
    Straight_Leg_Model.wheell_K[i] = Straight_Leg_Model.K_coefficient[0][m + 0] + Straight_Leg_Model.K_coefficient[0][m + 1]*ll + Straight_Leg_Model.K_coefficient[0][m + 2]*lr \
                                     + Straight_Leg_Model.K_coefficient[0][m + 3]*ll*ll + Straight_Leg_Model.K_coefficient[0][m + 4]*ll*lr + Straight_Leg_Model.K_coefficient[0][m + 5]*lr*lr;
  }
  /*鍙宠疆*/
  for(int i = 0; i < 10; i++)
  {
    int m = 6 * i;
    Straight_Leg_Model.wheelr_K[i] = Straight_Leg_Model.K_coefficient[1][m + 0] + Straight_Leg_Model.K_coefficient[1][m + 1]*ll + Straight_Leg_Model.K_coefficient[1][m + 2]*lr \
                                     + Straight_Leg_Model.K_coefficient[1][m + 3]*ll*ll + Straight_Leg_Model.K_coefficient[1][m + 4]*ll*lr + Straight_Leg_Model.K_coefficient[1][m + 5]*lr*lr;
  }
  /*宸﹁偐*/
  for(int i = 0; i < 10; i++)
  {
    int m = 6 * i;
    Straight_Leg_Model.sdl_K[i] = Straight_Leg_Model.K_coefficient[2][m + 0] + Straight_Leg_Model.K_coefficient[2][m + 1]*ll + Straight_Leg_Model.K_coefficient[2][m + 2]*lr \
                                     + Straight_Leg_Model.K_coefficient[2][m + 3]*ll*ll + Straight_Leg_Model.K_coefficient[2][m + 4]*ll*lr + Straight_Leg_Model.K_coefficient[2][m + 5]*lr*lr;
  }
  /*鍙宠偐*/
  for(int i = 0; i < 10; i++)
  {
    int m = 6 * i;
    Straight_Leg_Model.sdr_K[i] = Straight_Leg_Model.K_coefficient[3][m + 0] + Straight_Leg_Model.K_coefficient[3][m + 1]*ll + Straight_Leg_Model.K_coefficient[3][m + 2]*lr \
                                     + Straight_Leg_Model.K_coefficient[3][m + 3]*ll*ll + Straight_Leg_Model.K_coefficient[3][m + 4]*ll*lr + Straight_Leg_Model.K_coefficient[3][m + 5]*lr*lr;
  }
}

/**
  * @brief  鏀�鎸佸姏璁＄畻
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_Supportive_Power_Cal(Link_Var_t* Link_Var)
{
  Link_Var->F_support = Link_Var->Fbl * cos(Link_Var->vir_phi0) + m_l*(g + My_Posture.y_acc \
  - (1 - Link_Var->centriod_coefficient)*Link_Var->l0_dot2*cos(Link_Var->vir_phi0));
  
  // Link_Var->F_support = m_l*(g + My_Posture.y_acc 
  // - (1 - Link_Var->centriod_coefficient)*Link_Var->l0_dot2*cos(Link_Var->vir_phi0));
  
  if(Link_Var->F_support <= OFF_GROUND_SUPPORT)//绂诲湴
  {
    Link_Var->off_ground = true;
  }
  else//瑙﹀湴锛岀�诲湴鍚庤Е搴曠灛闂�
  {
    if(Link_Var->off_ground == true)//绂诲湴鍚庤Е鍦扮灛闂�
    {
      Link_Var->landed = true;
    }
    Link_Var->off_ground = false;//瑙﹀湴
  }
}

/**
  * @brief  yaw杞寸洰鏍囧€煎�勭悊
  * @param  double target
  * @retval 澶勭悊鍚庣殑鐩�鏍囧€�
  */
double My_Yaw_Target_Process(double target)
{
  if(abs(target) > PI)
  {
    target -= one(target)*2*PI;
  }
  
  return target;
}

/**
  * @brief  yaw杞磋��宸�杩囩被杩囬浂鐐瑰�勭悊
  * @param  double err
  * @retval 澶勭悊鍚庣殑璇�宸�鍊�
  */
double My_Yaw_Zero_Point_Process(double err)
{
  if(abs(err) > PI)
  {
    err += -(one(err) * 2*PI);
  }
  
  return err;
}

/**
  * @brief  璺宠穬鑵块暱鐩�鏍囧€煎�勭悊
  * @param  None
  * @retval None
  */
void My_Jump_Target_Process(void)
{
  static int step = 0;
  
  static double org_tar = 0.f;
  
  double mea = (My_Link_Var[R_Link].l0 + My_Link_Var[L_Link].l0)*0.5f;
  
  if(My_Robot.jump_flag == true)
  {
    if(step == 0)
    { 
      org_tar = My_Robot.leg_length_target;
      My_Robot.leg_length_target = 0.13f;
      step++;
      My_Link_Var[R_Link].length_pid->info->kp = 1000;
      My_Link_Var[L_Link].length_pid->info->kp = 1000;
      My_Link_Var[R_Link].length_pid->info->out_max = 300;
      My_Link_Var[L_Link].length_pid->info->out_max = 300;
      
    }
    else if(step == 1 && mea < 0.135f)
    {
      My_Robot.leg_length_target = 0.36f;
      step++;
      My_Link_Var[R_Link].length_pid->info->kp = 2000;
      My_Link_Var[L_Link].length_pid->info->kp = 2000;
      My_Link_Var[R_Link].length_pid->info->out_max = 600;
      My_Link_Var[L_Link].length_pid->info->out_max = 600;
    }
    else if(step == 2 && mea > 0.355f)
    {
      My_Robot.leg_length_target = 0.13f;
      step++;
      My_Link_Var[R_Link].length_pid->info->kp = 4000;
      My_Link_Var[L_Link].length_pid->info->kp = 4000;
      My_Link_Var[R_Link].length_pid->info->out_max = 1000;
      My_Link_Var[L_Link].length_pid->info->out_max = 1000;
    }
    else if(step == 3 && mea < 0.15f)
    {
      My_Robot.leg_length_target = 0.21f;
      step = 0;
      My_Robot.jump_flag = false;
      My_Link_Var[R_Link].length_pid->info->kp = 850;
      My_Link_Var[L_Link].length_pid->info->kp = 850;
      My_Link_Var[R_Link].length_pid->info->out_max = 200;
      My_Link_Var[L_Link].length_pid->info->out_max = 200;
    }
  }
  
  // printf("%d\t %f\t %f\t %f\n", step, My_Robot.leg_length_target, My_Link_Var[R_Link].F, mea);
}

/**
  * @brief  閿�鐩樿緭鍏ユ洿鏂帮紝浼樺厛绾ч珮浜庡叾瀹冩暟鎹�
  * @param  None
  * @retval None
  */
void My_Key_React(void)
{
 int key_value = wb_keyboard_get_key();
  
  switch(key_value)
  {
    case GO_FORWARD:
      My_Key.w_key++;
      My_Key.w_key = constrain(My_Key.w_key, 0, KEY_CNT_MAX);
      
      My_Robot.speed_target = (My_Key.w_key / 50.f) * My_Robot.config->front_speed_max;
      
      My_Key.a_key = 0;
      My_Key.s_key = 0;
      My_Key.d_key = 0;
      My_Robot.spin_target = 0;
      break;
    case GO_BACKWARD:
      My_Key.s_key++;
      My_Key.s_key = constrain(My_Key.s_key, 0, KEY_CNT_MAX);
      
      My_Robot.speed_target = -(My_Key.s_key / 50.f) * My_Robot.config->front_speed_max;
      
      My_Key.w_key = 0;
      My_Key.a_key = 0;
      My_Key.d_key = 0;
      My_Robot.spin_target = 0;
      break;
    case TURN_RIGHT:
      My_Key.d_key++;
      My_Key.d_key = constrain(My_Key.d_key, 0, 50);
      
      My_Robot.spin_target = -My_Robot.config->spin_speed_max;
      
      My_Key.w_key = 0;
      My_Key.a_key = 0;
      My_Key.s_key = 0;
      My_Robot.speed_target = 0;
      break;
    case TURN_LEFT:
      My_Key.a_key++;
      My_Key.a_key = constrain(My_Key.a_key, 0, 50);
      
      My_Robot.spin_target = My_Robot.config->spin_speed_max;
      
      My_Key.w_key = 0;
      My_Key.s_key = 0;
      My_Key.d_key = 0;
      My_Robot.speed_target = 0;
      break;
    case GO_UP:
      My_Robot.leg_length_target += 0.0004;
      break;
    case GO_DOWN:
      My_Robot.leg_length_target -= 0.0004;
      break;
    case TILT_LEFT:
      My_Robot.roll_target += 0.0005;
      break;
    case TILT_RIGHT:
      My_Robot.roll_target -= 0.0005;
      break;
    case TOP_MODE_ON:
      My_Robot.top_mode = true;
      break;
    case TOP_MODE_OFF:
      My_Robot.top_mode = false;
      My_Robot.spin_target = 0.f;
      break;
    case JUMP:
      if(My_Robot.jump_flag != true)
      {
        My_Robot.jump_flag = true;
      }
      
      break;
    case START_RECORD:
      fp = fopen("C:\\Users\\Lenovo\\Desktop\\Balance\\MatlabWorks\\Data_Estimate\\My_Data.csv","w");
      printf("鏁版嵁璁板綍寮€濮�......\n");
      break;
    case END_RECORD:
      fclose(fp);
      printf("鏁版嵁璁板綍缁撴潫......\n");
      break;
    default:
      My_Key.w_key = 0;
      My_Key.a_key = 0;
      My_Key.s_key = 0;
      My_Key.d_key = 0;
      
      My_Robot.speed_target = 0;
      My_Robot.spin_target = 0;
      break;
   }
   
   // printf("%f\t %f\t %f\t %f\n",My_Robot.leg_length_target, LEG_LENGTH_MAX, My_Robot.yaw_target, My_Link_Var[L_Link].l_target);
   
   My_Robot.position_target += (My_Robot.speed_target * (TIME_STEP * 0.001));
   
   if(My_Robot.top_mode == true)
   {
     My_Robot.spin_target = 9.f;
   }
   
   My_Robot.yaw_target += (My_Robot.spin_target * (TIME_STEP * 0.001));
   My_Robot.yaw_target = My_Yaw_Target_Process(My_Robot.yaw_target);
   
    
   
}

void My_Robot_Command_React(void)
{
  if(My_Robot.speed_target != 0)
  {
    My_Robot.status->moving = true;
  }
  else
  {
    My_Robot.status->moving = false;
  }
  
  if(My_Robot.spin_target != 0)
  {
    My_Robot.status->spinning = true;
  }
  else
  {
    My_Robot.status->spinning = false;
  }
}
/*...........................................宸ュ叿鍑芥暟 end...........................................*/






/**
  * @brief  楠岃瘉涓婁氦鐨勪簲杩炴潌锛屼笌鑷�宸辩殑姣旇緝锛岄獙璇佷负杈撳嚭鐩哥�﹀悎
  * @param  None
  * @retval None
  */
void My_Test(Link_Var_t* Link_Var, double Fbl, double Tbl)
{
  // double theta = 0.f, phi1 = 0.f, phi2 = 0.f, l = 0.f, T1 = 0.f, T2 = 0.f;
  
  // theta = Link_Var->vir_phi0;//1
  
  // phi1 = Link_Var->phi1;//1
  
  // phi2 = PI - Link_Var->phi4;//1
  
  // l = Link_Var->l0;//1  
  
  // T1 = (3*((9*Tbl*cos(phi1 + phi2 - theta))/800 - (9*Tbl*cos(phi1 - phi2 + theta))/800 + 
  // (9*Tbl*cos(phi1 + theta))/1600 - (9*Tbl*cos(phi1 - theta))/1600 + (3*Tbl*l*sin(phi1))/40 - 
  // (3*Tbl*l*sin(phi1 - phi2 + 2*theta))/20 + (3*Fbl*pow(l, 2)*cos(phi1 + phi2))/20 
  // - 2*Tbl*pow(l, 2)*cos(phi1 + theta) + (9*Fbl*l*sin(phi1 - theta))/1600 + (3*Tbl*l*sin(phi1 + 2*theta))/40 
  // - (3*Fbl*pow(l, 2)*cos(phi1))/40 + (3*Fbl*pow(l, 2)*cos(phi1 - phi2 + 2*theta))/20 - (3*Fbl*pow(l, 2)*cos(phi1 
  // + 2*theta))/40 - (9*Fbl*l*sin(phi1 + phi2 - theta))/800 - (9*Fbl*l*sin(phi1 - phi2 + theta))/800 
  // + (9*Fbl*l*sin(phi1 + theta))/1600 + (3*Tbl*l*sin(phi1 + phi2))/20))/(40*l*((9*sin(phi1))/800 
  // - (9*sin(phi1 + phi2))/400 + (9*sin(phi2))/800 + (3*l*cos(phi1 + theta))/20 - (3*l*cos(theta))/20 + (3*l*cos(phi2 - theta))/20));
  
  // T2 = -(3*((9*Tbl*cos(phi2 - theta))/1600 - (9*Tbl*cos(phi2 + theta))/1600 - (9*Tbl*cos(phi1 
  // - phi2 + theta))/800 + (9*Tbl*cos(phi1 + phi2 + theta))/800 + (3*Tbl*l*sin(phi2))/40 + (3*Tbl*l*sin(phi1 
  // - phi2 + 2*theta))/20 - (3*Fbl*pow(l, 2)*cos(phi1 + phi2))/20 - (9*Fbl*l*sin(phi2 - theta))/1600 
  // + (3*Tbl*l*sin(phi2 - 2*theta))/40 + (9*Fbl*l*sin(phi1 + phi2 + theta))/800 + (3*Fbl*pow(l, 2)*cos(phi2))/40 
  // - (3*Fbl*pow(l, 2)*cos(phi1 - phi2 + 2*theta))/20 + (3*Fbl*pow(l, 2)*cos(phi2 - 2*theta))/40 
  // - 2*Tbl*pow(l, 2)*cos(phi2 - theta) - (9*Fbl*l*sin(phi1 - phi2 + theta))/800 - (9*Fbl*l*sin(phi2 + theta))/1600 
  // + (3*Tbl*l*sin(phi1 + phi2))/20))/(40*l*((9*sin(phi1))/800 - (9*sin(phi1 + phi2))/400 + (9*sin(phi2))/800 
  // + (3*l*cos(phi1 + theta))/20 - (3*l*cos(theta))/20 + (3*l*cos(phi2 - theta))/20));
  
  // printf("%f\t %f\t %f\t %f\t %f\t %f\n", l, theta, phi1, phi2, Fbl, Tbl);
  // printf("%f\t %f\n", T1, T2);
  
  
}









int main(int argc, char **argv) {

  wb_robot_init();
  
  My_Get_All_Device();
  

  while (wb_robot_step(TIME_STEP) != -1) {
    /*浼樺厛绾ч珮浜庢暟鎹�鏇存柊锛屽洜涓烘湁浜涙暟鎹�鍦ㄦ煇浜涚姸鎬佷笅闇€瑕佺壒娈婂�勭悊 begin*/
    My_Key_React();
    
    My_Robot_Command_React();
    /*end*/
    
    My_Data_Update();
    
    Straight_Leg_Model_Cal();
    
    My_Torque_Cal();    
    
    My_Print_Debug();
    
    wb_motor_set_torque(Motor[R_F_Sd_Motor], constrain(My_Motor[R_F_Sd_Motor].output_torque, -32, 32));
    wb_motor_set_torque(Motor[R_B_Sd_Motor], constrain(My_Motor[R_B_Sd_Motor].output_torque, -32, 32));
    
    wb_motor_set_torque(Motor[L_F_Sd_Motor], constrain(My_Motor[L_F_Sd_Motor].output_torque, -32, 32));
    wb_motor_set_torque(Motor[L_B_Sd_Motor], constrain(My_Motor[L_B_Sd_Motor].output_torque, -32, 32));
    
    // printf("%f\t %f\n",Straight_Leg_Model.Tw_r, Straight_Leg_Model.Tw_l);
    
    wb_motor_set_torque(Motor[R_Wheel_Motor], Straight_Leg_Model.Tw_r);
    wb_motor_set_torque(Motor[L_Wheel_Motor], Straight_Leg_Model.Tw_l);
    
    
    
    //鍏宠妭鐢垫満鐨勫姏鍚戜笂鏋佹€э細
    // wb_motor_set_torque(Motor[R_F_Sd_Motor], -0.7);
    // wb_motor_set_torque(Motor[R_B_Sd_Motor], 0.7);
    
    // wb_motor_set_torque(Motor[L_F_Sd_Motor], -0.7);
    // wb_motor_set_torque(Motor[L_B_Sd_Motor], 0.7);
    
    // wb_motor_set_torque(Motor[R_Wheel_Motor], 0);
    // wb_motor_set_torque(Motor[L_Wheel_Motor], 0);
  };

  wb_robot_cleanup();

  return 0;
}



