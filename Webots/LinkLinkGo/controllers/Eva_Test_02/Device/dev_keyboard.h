#ifndef __DEV_KEYBOARD_H
#define __DEV_KEYBOARD_H

#include "config_environment.h"
#include "config_math.h"

/*¼üÅÌ¼üÖµ¶¨Òå*/
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

typedef struct My_Key_struct_t
{
  int w_key;
  
  int a_key;
  
  int s_key;
  
  int d_key;
  
  int o_key;
  
  int p_key;
  
  int q_key;
  
  int e_key;
  
  int n_key;
  
  int m_key;

  int space_key;
}My_Key_t;

extern My_Key_t My_Key;

void My_Keyboard_Init(void);
void My_Keyboard_Reset(void);
void My_Key_Update(void);

#endif