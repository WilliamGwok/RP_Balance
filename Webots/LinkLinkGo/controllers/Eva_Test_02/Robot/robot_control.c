#include "robot_control.h"

void My_Robot_Command_React(void);
void My_Robot_Data_Update(void);
void My_Robot_Motor_Output(void);
void My_Robot_Torque_Cal(void);
void My_Robot_Off_Ground_State_Update(Robot_Measure_t* mea, Link_Var_t* Link_Var, Virtual_Leg_t* Virtual_Var);
void My_Jump_Target_Process(void);
void My_Leg_Length_Different_Cal(void);
double My_Robot_Yaw_Target_Process(double target);
void My_Robot_Translation_Target_Update(void);
void My_Robot_Spin_Target_Update(void);
void My_Robot_Leg_Length_Target_Update(void);
void My_Robot_Data_Record_Target_Update(void);

FILE *fp = NULL;

Robot_Config_t My_Robot_Config = 
{
  .front_speed_max = 2.0f,//m/s
  
  .spin_speed_max = 4.2f,//rad/s
};

Robot_Measure_t My_Robot_L_Measure;

Robot_Measure_t My_Robot_R_Measure;

Robot_Target_t My_Robot_Target = 
{
  .leg_length_target = TAR_LEG_LENGTH_INITIAL,
};

Robot_Command_t My_Robot_Command = 
{
  .jump_flag = false,

  .data_record_flag = false,
};

Robot_t My_Robot = 
{
  .config = &My_Robot_Config,
  
  .l_measure = &My_Robot_L_Measure,

  .r_measure = &My_Robot_R_Measure,

  .target = &My_Robot_Target,

  .command = &My_Robot_Command,
};

/**
  * @brief  机器人初始化
  * @param  None
  * @retval None
  */
void My_Robot_Init(void)
{
  My_Device_Init();
}

void My_Robot_Work(void)
{
  My_Robot_Command_React();

  My_Robot_Data_Update();

  My_Robot_Torque_Cal();

  My_Robot_Motor_Output();
}




void My_Robot_Command_React(void)
{
  My_Key_Update();

  My_Robot_Translation_Target_Update();

  My_Robot_Spin_Target_Update();

  My_Robot_Leg_Length_Target_Update();

  My_Robot_Data_Record_Target_Update();

}

void My_Robot_Translation_Target_Update(void)
{
  My_Robot.target->speed_target = ((My_Key.w_key - My_Key.s_key) / 50.f) * My_Robot.config->front_speed_max;

  if(My_Robot.target->speed_target != 0)
  {
    My_Robot.target->spin_target = 0;
  }

   My_Robot.target->position_target += (My_Robot.target->speed_target * (TIME_STEP * 0.001));
}

void My_Robot_Spin_Target_Update(void)
{
  int direction = 0;

  if((My_Key.a_key - My_Key.d_key) > 0)
  {
    direction = 1;
  }
  else if((My_Key.a_key - My_Key.d_key) < 0)
  {
    direction = -1;
  }
  else
  {
    direction = 0;
  }

  My_Robot.target->spin_target = direction * My_Robot.config->spin_speed_max;

  if(My_Robot.target->spin_target != 0)
  {
    My_Robot.target->speed_target = 0;
  }

  My_Robot.target->yaw_target += (My_Robot.target->spin_target * (TIME_STEP * 0.001));
  My_Robot.target->yaw_target = My_Robot_Yaw_Target_Process(My_Robot.target->yaw_target);
}

void My_Robot_Leg_Length_Target_Update(void)
{
  if(My_Key.space_key != 0 && My_Robot.command->jump_flag != true)
  {
    My_Robot.command->jump_flag = true;
  }

  if(My_Robot.command->jump_flag != true)
  {
    if(My_Key.o_key != 0)
    {
      My_Robot.target->leg_length_target += 0.0004;
    }

    if(My_Key.p_key != 0)
    {
      My_Robot.target->leg_length_target -= 0.0004;
    }
  }
  else
  {
    My_Jump_Target_Process();
  }
}

void My_Robot_Data_Record_Target_Update(void)
{
  if(My_Key.n_key != 0 && My_Robot.command->data_record_flag != true)
  {
    My_Robot.command->data_record_flag = true;

    fp = fopen(DATA_FILE_PATH,"w");
    printf("数据记录开始......\n");
  }
  if(My_Key.m_key != 0 && My_Robot.command->data_record_flag == true)
  {
    My_Robot.command->data_record_flag = false;

    fclose(fp);
    printf("数据记录结束......\n");
  }
}

/**
  * @brief  机器人数据更新
  * @param  None
  * @retval None
  */
void My_Robot_Data_Update(void)
{
  My_Device_Data_Update();

  My_Link_Data_Update();
  
  My_State_Var_Update();

  My_Robot_Off_Ground_State_Update(My_Robot.l_measure, &My_Link_Var[L_Link], &My_Virtual_Leg[L_Leg]);

  My_Robot_Off_Ground_State_Update(My_Robot.r_measure, &My_Link_Var[R_Link], &My_Virtual_Leg[R_Leg]);
}

/**
  * @brief  计算直腿模型下关节、轮毂的扭矩输出
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_Robot_Torque_Cal(void)
{
  /*赋予左右腿长目标值*/
  My_Leg_Length_Different_Cal();

  My_Virtual_Leg_Force_Cal();

  Straight_Leg_Model_Cal(My_Robot.target->position_target, My_Robot.target->yaw_target, \
                         My_Robot.l_measure->off_ground, My_Robot.r_measure->off_ground);
}

/**
  * @brief  计算五连杆模型下关节、轮毂各个电机的扭矩输出
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_Robot_Motor_Output(void)
{
  /*右边关节*/
  My_Motor[R_F_Sd_Motor].output_torque = My_Back_Joint_Torque_Cal(&My_Link_Var[R_Link], \
                                                                  My_Virtual_Leg[R_Leg].Fbl, Straight_Leg_Model.Tp_r);
  
  My_Motor[R_B_Sd_Motor].output_torque = My_Front_Joint_Torque_Cal(&My_Link_Var[R_Link], \
                                                                   My_Virtual_Leg[R_Leg].Fbl, Straight_Leg_Model.Tp_r);
  
  /*左边关节*/
  My_Motor[L_F_Sd_Motor].output_torque = My_Back_Joint_Torque_Cal(&My_Link_Var[L_Link], \
                                                                  My_Virtual_Leg[L_Leg].Fbl, Straight_Leg_Model.Tp_l);
  
  My_Motor[L_B_Sd_Motor].output_torque = My_Front_Joint_Torque_Cal(&My_Link_Var[L_Link], \
                                                                   My_Virtual_Leg[L_Leg].Fbl, Straight_Leg_Model.Tp_l);

  My_Motor[R_Wheel_Motor].output_torque = Straight_Leg_Model.Tw_r;
  
  My_Motor[L_Wheel_Motor].output_torque = Straight_Leg_Model.Tw_l;

  My_Motor_Work();
}

/**
  * @brief  支持力计算,检测是否离地
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void My_Robot_Off_Ground_State_Update(Robot_Measure_t* mea, Link_Var_t* Link_Var, Virtual_Leg_t* Virtual_Var)
{
  mea->F_support = Virtual_Var->Fbl * cos(Link_Var->angle->vir_phi0) + m_l*(g + My_Posture.y_acc \
  - (1 - Link_Var->centroid->centriod_coefficient)*Link_Var->length->l0_dot2*cos(Link_Var->angle->vir_phi0));
  
  if(mea->F_support <= OFF_GROUND_SUPPORT)//离地
  {
    mea->off_ground = true;
  }
  else//触地，离地后触底瞬间
  {
    mea->off_ground = false;//触地
  }
}

/**
  * @brief  跳跃腿长目标值处理
  * @param  None
  * @retval None
  */
void My_Jump_Target_Process(void)
{
  static int step = 0;
  
  static double org_tar = 0.f;
  
  double mea = (My_Link_Var[R_Link].length->l0 + My_Link_Var[L_Link].length->l0)*0.5f;
  
  if(My_Robot.command->jump_flag == true)
  {
    if(step == 0)
    { 
      org_tar = My_Robot.target->leg_length_target;
      My_Robot.target->leg_length_target = 0.13f;
      step++;
      My_Virtual_Leg[R_Leg].length_pid->info->kp = 1000;
      My_Virtual_Leg[L_Leg].length_pid->info->kp = 1000;
      My_Virtual_Leg[R_Leg].length_pid->info->out_max = 300;
      My_Virtual_Leg[L_Leg].length_pid->info->out_max = 300;
      
    }
    else if(step == 1 && mea < 0.135f)
    {
      My_Robot.target->leg_length_target = 0.36f;
      step++;
      My_Virtual_Leg[R_Leg].length_pid->info->kp = 2000;
      My_Virtual_Leg[L_Leg].length_pid->info->kp = 2000;
      My_Virtual_Leg[R_Leg].length_pid->info->out_max = 600;
      My_Virtual_Leg[L_Leg].length_pid->info->out_max = 600;
    }
    else if(step == 2 && mea > 0.355f)
    {
      My_Robot.target->leg_length_target = 0.13f;
      step++;
      My_Virtual_Leg[R_Leg].length_pid->info->kp = 4000;
      My_Virtual_Leg[L_Leg].length_pid->info->kp = 4000;
      My_Virtual_Leg[R_Leg].length_pid->info->out_max = 1000;
      My_Virtual_Leg[L_Leg].length_pid->info->out_max = 1000;
    }
    else if(step == 3 && mea < 0.15f)
    {
      My_Robot.target->leg_length_target = 0.21f;
      step = 0;
      My_Robot.command->jump_flag = false;
      My_Virtual_Leg[R_Leg].length_pid->info->kp = 850;
      My_Virtual_Leg[L_Leg].length_pid->info->kp = 850;
      My_Virtual_Leg[R_Leg].length_pid->info->out_max = 200;
      My_Virtual_Leg[L_Leg].length_pid->info->out_max = 200;
    }
  }
}

/**
  * @brief  赋予腿长目标值，冗余函数，有待优化
  * @param  None
  * @retval None
  */
void My_Leg_Length_Different_Cal(void)
{
  for(int i = 0; i < 2; i++)
  {
    My_Virtual_Leg[i].l_target = My_Robot.target->leg_length_target;
  }
}

/**
  * @brief  yaw轴目标值处理
  * @param  double target
  * @retval 处理后的目标值
  */
double My_Robot_Yaw_Target_Process(double target)
{
  if(abs(target) > PI)
  {
    target -= one(target)*2*PI;
  }
  
  return target;
}

