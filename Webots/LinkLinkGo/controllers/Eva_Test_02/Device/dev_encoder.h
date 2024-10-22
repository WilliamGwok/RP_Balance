#ifndef __DEV_ENCODER_H
#define __DEV_ENCODER_H

#include "config_environment.h"

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

extern Encoder_t My_Encoder[Encoder_Total_Num];

void My_Encoder_Init(void);
void My_Encoder_Data_Update(void);

#endif


