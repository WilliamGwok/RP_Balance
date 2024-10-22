#include "pid.h"

/**
  * @brief  单环PID控制
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




