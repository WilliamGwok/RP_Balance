#include "dev_keyboard.h"

My_Key_t My_Key;

void My_Keyboard_Init(void)
{
  /*键盘初始化*/
  wb_keyboard_enable(TIME_STEP);
}

void My_Keyboard_Reset(void)
{
  memset(&My_Key, 0, sizeof(My_Key));
}

/**
  * @brief  键盘输入更新，优先级高于其它数据
  * @param  None
  * @retval None
  */
void My_Key_Update(void)
{
  int key_value = wb_keyboard_get_key();
  
  switch(key_value)
  {
    case GO_FORWARD:
      My_Key.w_key++;
      My_Key.w_key = constrain(My_Key.w_key, 0, KEY_CNT_MAX);
      
      My_Key.a_key = 0; My_Key.d_key = 0; My_Key.s_key = 0;
      break;
    case GO_BACKWARD:
      My_Key.s_key++;
      My_Key.s_key = constrain(My_Key.s_key, 0, KEY_CNT_MAX);

      My_Key.a_key = 0; My_Key.d_key = 0; My_Key.w_key;
      break;
    case TURN_RIGHT:
      My_Key.d_key++;
      My_Key.d_key = constrain(My_Key.d_key, 0, KEY_CNT_MAX);

      My_Key.w_key = 0; My_Key.s_key = 0; My_Key.a_key;
      break;
    case TURN_LEFT:
      My_Key.a_key++;
      My_Key.a_key = constrain(My_Key.a_key, 0, KEY_CNT_MAX);

      My_Key.w_key = 0; My_Key.s_key = 0; My_Key.d_key;
      break;
    case GO_UP:
      My_Key.o_key = 1;
      break;
    case GO_DOWN:
      My_Key.p_key = 1;
      break;
    case TILT_LEFT:
      My_Key.q_key = 1;
      break;
    case TILT_RIGHT:
      My_Key.e_key = 1;
      break;
    case JUMP:
      My_Key.space_key = 1;
      break;
    case START_RECORD:
      My_Key.n_key = 1;
      break;
    case END_RECORD:
      My_Key.m_key = 1;
      break;
    default:
      My_Keyboard_Reset();
      break;
   }
}


