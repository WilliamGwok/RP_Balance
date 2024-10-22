#include "five_link.h"
#include "dev_encoder.h"

void My_Link_Var_Angle_Cal(void);
double My_Phi2_Cal(Link_Var_t* Link_Var);
double My_Phi3_Cal(Link_Var_t* Link_Var);
void My_Link_Coordinate_Cal(Link_Var_t* Link_Var);
void My_link_Centroid_Coordinate_Cal(Link_Var_t* Link_Var);
double My_Leg_Length_Cal(Link_Var_t* Link_Var);
void My_Link_Var_Length_Cal(Link_Var_t* Link_Var);
double My_Phi0_Cal(Link_Var_t* Link_Var);
double My_Virtual_Leg_Rad_Cal(Link_Var_t* Link_Var);
void My_Link_Var_Phi0_Cal(void);



Link_Var_Angle_t My_Link_Var_Angle[Link_Num] = 
{
  [L_Link] = {
    .phi1 = 0,
    .phi4 = 0,
  },
  
  [R_Link] = {
    .phi1 = 0,
    .phi4 = 0,
  },
};

Link_Var_Coordinate_t My_Link_Var_Coordinate[Link_Num];

Link_Var_Length_t My_Link_Var_Length[Link_Num];

Link_Var_Centroid_t My_Link_Var_Centroid[Link_Num];


Link_Var_t My_Link_Var[Link_Num] = 
{
  [L_Link] = {
    .angle = &My_Link_Var_Angle[L_Link],
    .coordinate = &My_Link_Var_Coordinate[L_Link],
    .length = &My_Link_Var_Length[L_Link],
    .centroid = &My_Link_Var_Centroid[L_Link],
  },
  
  [R_Link] = {
    .angle = &My_Link_Var_Angle[R_Link],
    .coordinate = &My_Link_Var_Coordinate[R_Link],
    .length = &My_Link_Var_Length[R_Link],
    .centroid = &My_Link_Var_Centroid[R_Link],
  },
};

void My_Link_Data_Update(void)
{
  My_Link_Var_Angle_Cal();

  /*更新连杆各点坐标*/
  My_Link_Coordinate_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Coordinate_Cal(&My_Link_Var[L_Link]);
  
  My_link_Centroid_Coordinate_Cal(&My_Link_Var[R_Link]);
  
  My_link_Centroid_Coordinate_Cal(&My_Link_Var[L_Link]);

  My_Link_Var_Length_Cal(&My_Link_Var[R_Link]);

  My_Link_Var_Length_Cal(&My_Link_Var[L_Link]);

  My_Link_Var_Phi0_Cal();
}


void My_Link_Var_Angle_Cal(void)
{
  /*先更新各�?角度，以便于计算各点坐标*/
  My_Link_Var[R_Link].angle->phi1 = PI - My_Encoder[R_F_Sd_Encoder].rad_now;//1
  
  My_Link_Var[R_Link].angle->phi2 =  My_Phi2_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Var[R_Link].angle->phi3 =  My_Phi3_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Var[R_Link].angle->phi4 = -My_Encoder[R_B_Sd_Encoder].rad_now;//1
  
  My_Link_Var[L_Link].angle->phi1 = PI - My_Encoder[L_F_Sd_Encoder].rad_now;//1
  
  My_Link_Var[L_Link].angle->phi2 =  My_Phi2_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Var[L_Link].angle->phi3 =  My_Phi3_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Var[L_Link].angle->phi4 = -My_Encoder[L_B_Sd_Encoder].rad_now;//1
}

void My_Link_Var_Length_Cal(Link_Var_t* Link_Var)
{
  //计算得到的腿长与期望值非常接近，但因建模不准�?，还有不少�??�?，需要优化建�?   emm,改了还是不太对，�?�?�?1cm左右  
  //右腿                    
  Link_Var->length->l0 = My_Leg_Length_Cal(Link_Var);
  
  Link_Var->length->l0_dot = (Link_Var->length->l0 - Link_Var->length->l0_last) / (TIME_STEP * 0.001);
  
  Link_Var->length->l0_last = Link_Var->length->l0;
  
  Link_Var->length->l0_dot2 = (Link_Var->length->l0_dot - Link_Var->length->l0_dot_last) / (TIME_STEP * 0.001);
  
  Link_Var->length->l0_dot_last = Link_Var->length->l0_dot;
}

void My_Link_Var_Phi0_Cal(void)
{
  My_Link_Var[R_Link].angle->phi0 = My_Phi0_Cal(&My_Link_Var[R_Link]);
  
  My_Link_Var[L_Link].angle->phi0 = My_Phi0_Cal(&My_Link_Var[L_Link]);
  
  My_Link_Var[R_Link].angle->vir_phi0 = My_Virtual_Leg_Rad_Cal(&My_Link_Var[R_Link]);//1
  
  My_Link_Var[L_Link].angle->vir_phi0 = My_Virtual_Leg_Rad_Cal(&My_Link_Var[L_Link]);//1
}





/**
  * @brief  计算等效腿关节�?�度phi2
  * @param  Link_Var_t* Link_Var
  * @retval 对应phi2
  */
double My_Phi2_Cal(Link_Var_t* Link_Var)
{
  double phi1 = 0.f, phi4 = 0.f, phi2 = 0.f, son = 0.f, mother = 0.f;
  
  phi1 = Link_Var->angle->phi1;
  
  phi4 = Link_Var->angle->phi4;
  
  son = (pow(((pow(l1, 2) - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2, 2) + 2*l2*l3 - pow(l3, 2) + pow(l4, 2) \
  + 2*cos(phi4)*l4*l5 + pow(l5, 2))*(- pow(l1, 2) + 2*cos(phi1 - phi4)*l1*l4 + 2*cos(phi1)*l1*l5 + pow(l2, 2) + 2*l2*l3 + pow(l3, 2) \
  - pow(l4, 2) - 2*cos(phi4)*l4*l5 - pow(l5, 2))), 0.5)- 2*l1*l2*sin(phi1) + 2*l2*l4*sin(phi4));
  
  mother = (pow(l1, 2) - 2*cos(phi1)*l1*l2 - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 + pow(l2, 2) + 2*cos(phi4)*l2*l4 + 2*l2*l5 \
  - pow(l3, 2) + pow(l4, 2) + 2*cos(phi4)*l4*l5 + pow(l5, 2));
  
  phi2 = 2*atan2(son, mother);
  
  return phi2;
}

/**
  * @brief  计算等效腿关节�?�度phi3
  * @param  Link_Var_t* Link_Var
  * @retval 对应phi3
  */
double My_Phi3_Cal(Link_Var_t* Link_Var)
{
  double phi1 = 0.f, phi4 = 0.f, phi3 = 0.f, son = 0.f, mother = 0.f;
  
  phi1 = Link_Var->angle->phi1;
  
  phi4 = Link_Var->angle->phi4;
  
  son = (pow(((pow(l1, 2) - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2, 2) + 2*l2*l3 - pow(l3, 2) \
  + pow(l4, 2) + 2*cos(phi4)*l4*l5 + pow(l5, 2))*(- pow(l1, 2) + 2*cos(phi1 - phi4)*l1*l4 + 2*cos(phi1)*l1*l5 \
  + pow(l2, 2) + 2*l2*l3 + pow(l3, 2) - pow(l4, 2) - 2*cos(phi4)*l4*l5 - pow(l5, 2))), 0.5)- 2*l1*l3*sin(phi1) + 2*l3*l4*sin(phi4));
  
  mother = (pow(l1, 2) + 2*cos(phi1)*l1*l3 - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2, 2) + pow(l3, 2) \
  - 2*cos(phi4)*l3*l4 - 2*l3*l5 + pow(l4, 2) + 2*cos(phi4)*l4*l5 + pow(l5, 2));
  
  phi3 = -2*atan(son/mother);//这里用atan2不太�?
  
  return phi3;
}

/**
  * @brief  五连杆各点坐标�?�算
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_Link_Coordinate_Cal(Link_Var_t* Link_Var)
{
  double phi1 = 0.f, phi4 = 0.f;

  phi1 = Link_Var->angle->phi1;
  
  phi4 = Link_Var->angle->phi4;

  Link_Var->coordinate->xa = 0;
  
  Link_Var->coordinate->ya = 0;
  
  Link_Var->coordinate->xb = l1*cos(phi1);
  
  Link_Var->coordinate->yb = l1*sin(phi1);

  Link_Var->coordinate->xc = l1*cos(phi1) + l2*cos(2*atan2((pow(((pow(l1,2) - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2,2) + 2*l2*l3 \
  - pow(l3,2) + pow(l4,2) + 2*cos(phi4)*l4*l5 + pow(l5,2))*(- pow(l1,2) + 2*cos(phi1 - phi4)*l1*l4 + 2*cos(phi1)*l1*l5 + pow(l2,2) + 2*l2*l3 \
  + pow(l3,2) - pow(l4,2) - 2*cos(phi4)*l4*l5 - pow(l5,2))), 0.5) - 2*l1*l2*sin(phi1) + 2*l2*l4*sin(phi4)), (pow(l1,2) - 2*cos(phi1)*l1*l2 \
  - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 + pow(l2,2) + 2*cos(phi4)*l2*l4 + 2*l2*l5 - pow(l3,2) + pow(l4,2) + 2*cos(phi4)*l4*l5 + pow(l5,2))));
                           
  Link_Var->coordinate->yc = l2*sin(2*atan2((pow(((pow(l1, 2) - 2*cos(phi1 - phi4)*l1*l4 - 2*cos(phi1)*l1*l5 - pow(l2, 2) + 2*l2*l3 - pow(l3, 2) + pow(l4, 2) \
  + 2*cos(phi4)*l4*l5 + pow(l5, 2))*(- pow(l1, 2) + 2*cos(phi1 - phi4)*l1*l4 + 2*cos(phi1)*l1*l5 + pow(l2, 2) + 2*l2*l3 + pow(l3, 2) - pow(l4, 2) \
  - 2*cos(phi4)*l4*l5 - pow(l5, 2))), 0.5) - 2*l1*l2*sin(phi1) + 2*l2*l4*sin(phi4)), (pow(l1, 2) - 2*cos(phi1)*l1*l2 - 2*cos(phi1 - phi4)*l1*l4 \
  - 2*cos(phi1)*l1*l5 + pow(l2, 2) + 2*cos(phi4)*l2*l4 + 2*l2*l5 - pow(l3, 2) + pow(l4, 2) + 2*cos(phi4)*l4*l5 + pow(l5, 2)))) + l1*sin(phi1);
  
  Link_Var->coordinate->xd = l5 + l4*cos(phi4);
  
  Link_Var->coordinate->yd = l4*sin(phi4);
  
  Link_Var->coordinate->xe = l5;
  
  Link_Var->coordinate->ye = 0;
}

/**
  * @brief  五连杆各杆质心坐标�?�算
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_link_Centroid_Coordinate_Cal(Link_Var_t* Link_Var)
{
  double xa = 0.f, xb = 0.f, xc = 0.f, xd = 0.f, xe = 0.f;
  double ya = 0.f, yb = 0.f, yc = 0.f, yd = 0.f, ye = 0.f;
  
  xa = Link_Var->coordinate->xa;  ya = Link_Var->coordinate->ya;
  xb = Link_Var->coordinate->xb;  yb = Link_Var->coordinate->yb;
  xc = Link_Var->coordinate->xc;  yc = Link_Var->coordinate->yc;
  xd = Link_Var->coordinate->xd;  yd = Link_Var->coordinate->yd;
  xe = Link_Var->coordinate->xe;  ye = Link_Var->coordinate->ye;
  
  Link_Var->centroid->mx_l1 = l1_cen*(xb - xa) + xa;
  Link_Var->centroid->my_l1 = l1_cen*(yb - ya) + ya;
  Link_Var->centroid->mx_l2 = l2_cen*(xc - xb) + xb;
  Link_Var->centroid->my_l2 = l2_cen*(yc - yb) + yb;
  Link_Var->centroid->mx_l3 = l3_cen*(xd - xc) + xc;
  Link_Var->centroid->my_l3 = l3_cen*(yd - yc) + yc;
  Link_Var->centroid->mx_l4 = l4_cen*(xe - xd) + xd;
  Link_Var->centroid->my_l4 = l4_cen*(ye - yd) + yd;
  
  Link_Var->coordinate->xp = (Link_Var->centroid->mx_l1*m_l1 + Link_Var->centroid->mx_l2*m_l2 + Link_Var->centroid->mx_l3*m_l3 + Link_Var->centroid->mx_l4*m_l4) / (m_l1 + m_l2 + m_l3 + m_l4);
  Link_Var->coordinate->yp = (Link_Var->centroid->my_l1*m_l1 + Link_Var->centroid->my_l2*m_l2 + Link_Var->centroid->my_l3*m_l3 + Link_Var->centroid->my_l4*m_l4) / (m_l1 + m_l2 + m_l3 + m_l4);
  
  Link_Var->centroid->centriod_coefficient = sqrt(pow((Link_Var->coordinate->xc - Link_Var->coordinate->xp), 2) + pow((Link_Var->coordinate->yc - Link_Var->coordinate->yp), 2)) \
                                   /pow( (pow((Link_Var->coordinate->xc - l5/2), 2) + pow(Link_Var->coordinate->yc, 2) ) , 0.5);
                                   
  Link_Var->centroid->centriod_coefficient = 1 - Link_Var->centroid->centriod_coefficient;
}

/**
  * @brief  等效腿长计算
  * @param  Link_Var_t* Link_Var
  * @retval 对应腿长
  */
double My_Leg_Length_Cal(Link_Var_t* Link_Var)
{
  double l0 = 0.f;
                           
  l0 = pow( (pow((Link_Var->coordinate->xc - l5/2), 2) + pow(Link_Var->coordinate->yc, 2) ) , 0.5);
  
  return l0;
}

/**
  * @brief  计算等效腿关节�?�度phi0
  * @param  Link_Var_t* Link_Var
  * @retval 对应phi0
  */
double My_Phi0_Cal(Link_Var_t* Link_Var)
{
  double phi0 = 0.f, xc = 0.f, yc = 0.f;
  
  xc = Link_Var->coordinate->xc;
  
  yc = Link_Var->coordinate->yc;
  
  phi0 = atan2(yc, (xc - l5/2));
  
  // phi0 = -(phi0 - 1.5708);//为了与直腿模型统一
  /*不�?�！！！！！！！！！应�?�先满足五连杆的解算，再在状态变量�?�算�?满足直腿模型！！！！*/
                  
  return phi0;
}

/**
  * @brief  将phi0准换为类似编码器的效�?
  �?换前�?
  0           0
  
      90/-90
  准换后：
  90          -90
  
        0/0
  * @param  Link_Var_t* Link_Var
  * @retval 虚拟杆编码器角度
  */
double My_Virtual_Leg_Rad_Cal(Link_Var_t* Link_Var)
{
  double phi0 = Link_Var->angle->phi0;
  
  double ans = 0.f;
  
  ans = -(phi0 - 1.5708);
  
  return ans;
}

/**
  * @brief  计算车后关节电机输出力矩
  * @param  Link_Var_t* Link_Var, double Fl, double Tp
  * @retval 后关节输出力矩T1
  */
double My_Back_Joint_Torque_Cal(Link_Var_t* Link_Var, double Fl, double Tp)
{
  double phi0 = 0.f, phi2 = 0.f, phi3 = 0.f, phi4 = 0.f, l0 = 0.f;
  
  phi0 = Link_Var->angle->phi0;
  
  phi2 = Link_Var->angle->phi2;
  
  phi3 = Link_Var->angle->phi3;
  
  phi4 = Link_Var->angle->phi4;
  
  l0 = Link_Var->length->l0;
  
  double T_1 = - (Fl*l4*sin(phi0 - phi2)*sin(phi3 - phi4))/sin(phi2 - phi3) \
  - (Tp*l4*cos(phi0 - phi2)*sin(phi3 - phi4))/(l0*sin(phi2 - phi3));

  return T_1;
}

/**
  * @brief  计算车前关节电机输出力矩
  * @param  Link_Var_t* Link_Var, double Fl, double Tp
  * @retval 前关节输出力矩T2
  */
double My_Front_Joint_Torque_Cal(Link_Var_t* Link_Var, double Fl, double Tp)
{
  double phi0 = 0.f, phi1 = 0.f, phi2 = 0.f, phi3 = 0.f, l0 = 0.f;
  
  phi0 = Link_Var->angle->phi0;
  
  phi1 = Link_Var->angle->phi1;
  
  phi2 = Link_Var->angle->phi2;
  
  phi3 = Link_Var->angle->phi3;
  
  l0 = Link_Var->length->l0;

  double T_2 = - (Fl*l1*sin(phi0 - phi3)*sin(phi1 - phi2))/sin(phi2 - phi3) - (Tp*l1*cos(phi0 \
  - phi3)*sin(phi1 - phi2))/(l0*sin(phi2 - phi3));  
  
  return T_2;
}


