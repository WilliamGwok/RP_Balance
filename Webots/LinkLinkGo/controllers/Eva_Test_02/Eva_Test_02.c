#include <webots/robot.h>
#include <stdio.h> 
#include <string.h>
#include <robot_control.h>

/*Debug*/
/*
23/10/9/10:04 五连杆phi2极性错误,phi3极性错误

23/10/19/20:51 怎么判断初始状态的离地状态？
               腿长目标值的限幅要放对

23/10/20/10:16 飞坡时没有收腿

23/10/24/16:38  侧向惯性力没弄懂，用好像也没用对

23/10/24/17:04  启动时有腿离地
*/

int main(int argc, char **argv) {

  wb_robot_init();
  
  My_Robot_Init();

  while (wb_robot_step(TIME_STEP) != -1) {
    /*end*/
    My_Robot_Work();

  };

  wb_robot_cleanup();

  return 0;
}



