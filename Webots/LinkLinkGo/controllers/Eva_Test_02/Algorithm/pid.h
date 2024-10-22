#ifndef __PID_H
#define __PID_H

#include <config_math.h>

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
/* ≈‰÷√ */
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

void single_pid_ctrl(pid_info_t *pid);

#endif


