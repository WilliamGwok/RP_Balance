#include "straight_leg.h"

double My_Yaw_Zero_Point_Process(double err);

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


void Straight_Leg_Model_Cal(double pos_tar, double yaw_tar, bool l_off_ground_flag, bool r_off_ground_flag)
{
  // K_Matrix_Fitting_Update();

  Straight_Leg_Model_t *Ace = &Straight_Leg_Model;
  
  /*驱动轮控制*/
  /*左*/
  Ace->s_part = Ace->wheell_K[0] * (My_State_Var.s - pos_tar) + Ace->wheell_K[1] * My_State_Var.sd1;//速度控制
  Ace->phi_part = Ace->wheell_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - yaw_tar) + Ace->wheell_K[3] * My_State_Var.phid1;  
  Ace->thetal_l_part = Ace->wheell_K[4] * My_State_Var.thetal_l + Ace->wheell_K[5] * My_State_Var.thetald1_l;
  Ace->thetal_r_part = Ace->wheell_K[6] * My_State_Var.thetal_r + Ace->wheell_K[7] * My_State_Var.thetald1_r;
  Ace->thetab_part = Ace->wheell_K[8] * My_State_Var.thetab + Ace->wheell_K[9] * My_State_Var.thetabd1;
  
  Ace->Tw_l = -(Ace->s_part + Ace->phi_part + Ace->thetal_l_part + Ace->thetal_r_part + Ace->thetab_part);
  
  if(l_off_ground_flag == 1)
  {
    Ace->Tw_l = 0;
  }
  
  Ace->Tw_l = constrain(Ace->Tw_l, -4, 4);
  
  /*右*/
  Ace->s_part = Ace->wheelr_K[0] * (My_State_Var.s - pos_tar) + Ace->wheelr_K[1] * My_State_Var.sd1;//速度控制
  Ace->phi_part = Ace->wheelr_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - yaw_tar) + Ace->wheelr_K[3] * My_State_Var.phid1;
  Ace->thetal_l_part = Ace->wheelr_K[4] * My_State_Var.thetal_l + Ace->wheelr_K[5] * My_State_Var.thetald1_l;
  Ace->thetal_r_part = Ace->wheelr_K[6] * My_State_Var.thetal_r + Ace->wheelr_K[7] * My_State_Var.thetald1_r;
  Ace->thetab_part = Ace->wheelr_K[8] * My_State_Var.thetab + Ace->wheelr_K[9] * My_State_Var.thetabd1;  
  
  Ace->Tw_r = -(Ace->s_part + Ace->phi_part + Ace->thetal_l_part + Ace->thetal_r_part + Ace->thetab_part);
  
  if(r_off_ground_flag == 1)
  {
    Ace->Tw_r = 0;
  }
  
  Ace->Tw_r = constrain(Ace->Tw_r, -4, 4);
  
  /*关节力计算*/
  /*左*/
  Ace->s_part = Ace->sdl_K[0] * (My_State_Var.s - pos_tar) + Ace->sdl_K[1] * My_State_Var.sd1;//速度控制
  Ace->phi_part = Ace->sdl_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - yaw_tar) + Ace->sdl_K[3] * My_State_Var.phid1;
  Ace->thetal_l_part = Ace->sdl_K[4] * My_State_Var.thetal_l + Ace->sdl_K[5] * My_State_Var.thetald1_l;
  Ace->thetal_r_part = Ace->sdl_K[6] * My_State_Var.thetal_r + Ace->sdl_K[7] * My_State_Var.thetald1_r;
  Ace->thetab_part = Ace->sdl_K[8] * My_State_Var.thetab + Ace->sdl_K[9] * My_State_Var.thetabd1;
  
  if(l_off_ground_flag == 1)
  {
    Ace->s_part = 0; Ace->phi_part = 0; Ace->thetab_part = 0; Ace->thetal_r_part = 0;
  }
  
  Ace->Tp_l = -(Ace->s_part + Ace->phi_part + Ace->thetal_l_part + Ace->thetal_r_part + Ace->thetab_part);
  
  Ace->Tp_l = constrain(Ace->Tp_l, -25, 25);
  
  /*右*/
  Ace->s_part = Ace->sdr_K[0] * (My_State_Var.s - pos_tar) + Ace->sdr_K[1] * My_State_Var.sd1;//速度控制
  Ace->phi_part = Ace->sdr_K[2] * My_Yaw_Zero_Point_Process(My_State_Var.phi - yaw_tar) + Ace->sdr_K[3] * My_State_Var.phid1;
  Ace->thetal_l_part = Ace->sdr_K[4] * My_State_Var.thetal_l + Ace->sdr_K[5] * My_State_Var.thetald1_l;
  Ace->thetal_r_part = Ace->sdr_K[6] * My_State_Var.thetal_r + Ace->sdr_K[7] * My_State_Var.thetald1_r;
  Ace->thetab_part = Ace->sdr_K[8] * My_State_Var.thetab + Ace->sdr_K[9] * My_State_Var.thetabd1;
  
  if(r_off_ground_flag == 1)
  {
    Ace->s_part = 0; Ace->phi_part = 0; Ace->thetab_part = 0; Ace->thetal_l_part = 0;
  }
  
  Ace->Tp_r = -(Ace->s_part + Ace->phi_part + Ace->thetal_l_part + Ace->thetal_r_part + Ace->thetab_part);
  
  Ace->Tp_r = constrain(Ace->Tp_r, -25, 25);
}

/**
  * @brief  根据实时腿长拟合K矩阵，在腿长更新后调用(用枚举可以简化代码)
  * @param  None
  * @retval None
  * @note   p00 + p10*ll + p01*lr + p20*ll^2 + p11*ll*lr + p02*lr^2
  */
void K_Matrix_Fitting_Update(void)
{
  double lr = My_Link_Var[R_Link].length->l0, ll = My_Link_Var[L_Link].length->l0;

  /*左轮*/
  for(int i = 0; i < 10; i++)
  {
    int m = 6 * i;
    Straight_Leg_Model.wheell_K[i] = Straight_Leg_Model.K_coefficient[0][m + 0] + Straight_Leg_Model.K_coefficient[0][m + 1]*ll + Straight_Leg_Model.K_coefficient[0][m + 2]*lr \
                                     + Straight_Leg_Model.K_coefficient[0][m + 3]*ll*ll + Straight_Leg_Model.K_coefficient[0][m + 4]*ll*lr + Straight_Leg_Model.K_coefficient[0][m + 5]*lr*lr;
  }
  /*右轮*/
  for(int i = 0; i < 10; i++)
  {
    int m = 6 * i;
    Straight_Leg_Model.wheelr_K[i] = Straight_Leg_Model.K_coefficient[1][m + 0] + Straight_Leg_Model.K_coefficient[1][m + 1]*ll + Straight_Leg_Model.K_coefficient[1][m + 2]*lr \
                                     + Straight_Leg_Model.K_coefficient[1][m + 3]*ll*ll + Straight_Leg_Model.K_coefficient[1][m + 4]*ll*lr + Straight_Leg_Model.K_coefficient[1][m + 5]*lr*lr;
  }
  /*左肩*/
  for(int i = 0; i < 10; i++)
  {
    int m = 6 * i;
    Straight_Leg_Model.sdl_K[i] = Straight_Leg_Model.K_coefficient[2][m + 0] + Straight_Leg_Model.K_coefficient[2][m + 1]*ll + Straight_Leg_Model.K_coefficient[2][m + 2]*lr \
                                     + Straight_Leg_Model.K_coefficient[2][m + 3]*ll*ll + Straight_Leg_Model.K_coefficient[2][m + 4]*ll*lr + Straight_Leg_Model.K_coefficient[2][m + 5]*lr*lr;
  }
  /*右肩*/
  for(int i = 0; i < 10; i++)
  {
    int m = 6 * i;
    Straight_Leg_Model.sdr_K[i] = Straight_Leg_Model.K_coefficient[3][m + 0] + Straight_Leg_Model.K_coefficient[3][m + 1]*ll + Straight_Leg_Model.K_coefficient[3][m + 2]*lr \
                                     + Straight_Leg_Model.K_coefficient[3][m + 3]*ll*ll + Straight_Leg_Model.K_coefficient[3][m + 4]*ll*lr + Straight_Leg_Model.K_coefficient[3][m + 5]*lr*lr;
  }
}


/**
  * @brief  yaw轴误差过类过零点处理
  * @param  double err
  * @retval 处理后的误差值
  */
double My_Yaw_Zero_Point_Process(double err)
{
  if(abs(err) > PI)
  {
    err += -(one(err) * 2*PI);
  }
  
  return err;
}








