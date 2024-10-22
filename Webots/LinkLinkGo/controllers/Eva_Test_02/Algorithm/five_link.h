#ifndef __FIVE_LINK_H
#define __FIVE_LINK_H

#include "config_math.h"
#include "config_robot.h"
#include "config_environment.h"
#include "pid.h"

typedef enum Link_Var_List
{
  L_Link = 0,
  
  R_Link,
  
  Link_Num,
}Link_Var_List_e;

typedef struct Link_Var_Angle_struct_t
{
  double phi1;
  
  double phi2;
  
  double phi3;
  
  double phi4;
  
  double phi0;
  
  double vir_phi0;
}Link_Var_Angle_t;

typedef struct Link_Var_Coordinate_struct_t
{
  double xa;  double ya;
  
  double xb;  double yb;
  
  double xc;  double yc;
  
  double xd;  double yd;
  
  double xe;  double ye;
  
  double xp;  double yp;
}Link_Var_Coordinate_t;

typedef struct Link_Var_Length_struct_t
{
  double l0; double l0_last; double l0_dot; double l0_dot_last; double l0_dot2;

}Link_Var_Length_t;

typedef struct Link_Var_Centroid_struct_t
{
  double mx_l1;  double my_l1;
  
  double mx_l2;  double my_l2;
  
  double mx_l3;  double my_l3;
  
  double mx_l4;  double my_l4;
  
  double centriod_coefficient;
}Link_Var_Centroid_t;


typedef struct Link_Var_struct_t
{
  Link_Var_Angle_t* angle;

  Link_Var_Coordinate_t* coordinate;

  Link_Var_Length_t* length;

  Link_Var_Centroid_t* centroid;
  
  double F_support;
  
  bool off_ground;
  
  bool landed;
  
}Link_Var_t;

extern Link_Var_t My_Link_Var[Link_Num];

void My_Link_Data_Update(void);
double My_Back_Joint_Torque_Cal(Link_Var_t* Link_Var, double Fl, double Tp);
double My_Front_Joint_Torque_Cal(Link_Var_t* Link_Var, double Fl, double Tp);



#endif


