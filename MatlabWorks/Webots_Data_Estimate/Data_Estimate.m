%Hello
clc;clear;

my_data = readmatrix('My_Data.csv');

%设置总数据范围
sampling_time = 1281;

%设置取样范围 begin
sample_start = 1;
sample_end   = 1281;

time_data = sample_start : 1 : sample_end;
sampling_data = sample_start : 1 : sample_end;
%设置取样范围 end

%设置曲线颜色
mea_color = [0.8500 0.3250 0.0980];
tar_color = [0 0.4470 0.7410];


figure
%偏航角绘制
yaw_mea_data = my_data(sampling_data, 1);
yaw_tar_data = my_data(sampling_data, 2);

subplot(2, 3, 1);
plot(time_data, yaw_mea_data, 'color', mea_color ,'LineWidth', 1);
hold on;
plot(time_data, yaw_tar_data, 'color', tar_color ,'LineWidth', 1);
title('yaw角度');
grid on;


%%偏航角速度绘制
yaw_v_mea_data = my_data(sampling_data, 3);
yaw_v_tar_data = my_data(sampling_data, 4);

subplot(2, 3, 2);
plot(time_data, yaw_v_mea_data,'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, yaw_v_tar_data, 'color', tar_color, 'LineWidth', 1)
ylim([-pi ,pi])
title('yaw角速度');
grid on;



%%绘制角度给右轮的输出
r_yaw_to_wheel_mea_data = my_data(sampling_data, 5);
r_yaw_to_wheel_data = my_data(sampling_data, 6);

subplot(2, 3, 3);
plot(time_data, r_yaw_to_wheel_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_yaw_to_wheel_data, 'color', tar_color, 'LineWidth', 1)
ylim([-1.5708 1.5708])
title('角度给右驱动轮的输出');
grid on;

%%绘制角速度给右轮的输出
r_yawv_to_wheel_mea_data = my_data(sampling_data, 7);
r_yawv_to_wheel_data = my_data(sampling_data, 8);

subplot(2, 3, 4);
plot(time_data, r_yawv_to_wheel_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_yawv_to_wheel_data, 'color', tar_color, 'LineWidth', 1)
ylim([-1.5708 1.5708])
title('角速度给右驱动轮的输出');
grid on;

%%绘制角度给右关节的输出
r_yaw_to_sd_mea_data = my_data(sampling_data, 9);
r_yaw_to_sd_data = my_data(sampling_data, 10);

subplot(2, 3, 5);
plot(time_data, r_yaw_to_sd_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_yaw_to_sd_data, 'color', tar_color, 'LineWidth', 1)
ylim([-1.5708 1.5708])
title('角度给右关节的输出');
grid on;

%%绘制角速度给右关节的输出
r_yawv_to_sd_mea_data = my_data(sampling_data, 11);
r_yawv_to_sd_data = my_data(sampling_data, 12);

subplot(2, 3, 6);
plot(time_data, r_yawv_to_sd_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_yawv_to_sd_data, 'color', tar_color, 'LineWidth', 1)
ylim([-0.5 0.5])
title('角速度给右关节的输出');
grid on;




figure
%%绘制右腿倾斜角度
r_theta_mea_data = my_data(sampling_data, 13);
r_theta_data = my_data(sampling_data, 14);
subplot(2, 3, 1);
plot(time_data, r_theta_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_theta_data, 'color', tar_color, 'LineWidth', 1)
ylim([-1.5708 1.5708])
title('右腿倾斜角度');
grid on;

%%绘制右腿倾斜角速度
r_thetav_mea_data = my_data(sampling_data, 15);
r_thetav_data = my_data(sampling_data, 16);
subplot(2, 3, 2);
plot(time_data, r_thetav_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_thetav_data, 'color', tar_color, 'LineWidth', 1)
ylim([-1.5708 1.5708])
title('右腿倾斜角速度');
grid on;

%%绘制右腿倾斜角度对右驱动轮的输出
r_theta_to_wheel_mea_data = my_data(sampling_data, 17);
r_theta_to_wheel_data = my_data(sampling_data, 18);
subplot(2, 3, 3);
plot(time_data, r_theta_to_wheel_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_theta_to_wheel_data, 'color', tar_color, 'LineWidth', 1)
ylim auto
title('右腿倾斜角度对右驱动轮的输出');
grid on;


%%绘制右腿倾斜角速度对右驱动轮的输出
r_thetav_to_wheel_mea_data = my_data(sampling_data, 19);
r_thetav_to_wheel_data = my_data(sampling_data, 20);
subplot(2, 3, 4);
plot(time_data, r_thetav_to_wheel_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_thetav_to_wheel_data, 'color', tar_color, 'LineWidth', 1)
ylim auto
title('右腿倾斜角速度对右驱动轮的输出');
grid on;

%%绘制右腿倾斜角度对右关节的输出
r_theta_to_sd_mea_data = my_data(sampling_data, 21);
r_theta_to_sd_data = my_data(sampling_data, 22);
subplot(2, 3, 5);
plot(time_data, r_theta_to_sd_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_theta_to_sd_data, 'color', tar_color, 'LineWidth', 1)
ylim auto
title('右腿倾斜角度对右关节的输出');
grid on;

%%绘制右腿倾斜角速度对右关节的输出
r_thetav_to_sd_mea_data = my_data(sampling_data, 23);
r_thetav_to_sd_data = my_data(sampling_data, 24);
subplot(2, 3, 6);
plot(time_data, r_thetav_to_sd_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, r_thetav_to_sd_data, 'color', tar_color, 'LineWidth', 1)
ylim auto
title('右腿倾斜角速度对右关节的输出');
grid on;



figure
%%绘制左腿倾斜角度
l_theta_mea_data = my_data(sampling_data, 25);
l_theta_data = my_data(sampling_data, 26);
subplot(2, 3, 1);
plot(time_data, l_theta_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, l_theta_data, 'color', tar_color, 'LineWidth', 1)
ylim([-1.5708 1.5708])
title('左腿倾斜角度');
grid on;

%%绘制左腿倾斜角速度
l_thetav_mea_data = my_data(sampling_data, 27);
l_thetav_data = my_data(sampling_data, 28);
subplot(2, 3, 2);
plot(time_data, l_thetav_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, l_thetav_data, 'color', tar_color, 'LineWidth', 1)
ylim([-1.5708 1.5708])
title('左腿倾斜角速度');
grid on;

%%绘制左腿倾斜角度对右驱动轮的输出
l_theta_to_wheel_mea_data = my_data(sampling_data, 29);
l_theta_to_wheel_data = my_data(sampling_data, 30);
subplot(2, 3, 3);
plot(time_data, l_theta_to_wheel_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, l_theta_to_wheel_data, 'color', tar_color, 'LineWidth', 1)
ylim auto
title('左腿倾斜角度对右驱动轮的输出');
grid on;

%%绘制左腿倾斜角速度对右驱动轮的输出
l_thetav_to_wheel_mea_data = my_data(sampling_data, 31);
l_thetav_to_wheel_data = my_data(sampling_data, 32);
subplot(2, 3, 4);
plot(time_data, l_thetav_to_wheel_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, l_thetav_to_wheel_data, 'color', tar_color, 'LineWidth', 1)
ylim auto
title('左腿倾斜角速度对右驱动轮的输出');
grid on;

%%绘制左腿倾斜角度对右关节的输出
l_theta_to_sd_mea_data = my_data(sampling_data, 33);
l_theta_to_sd_data = my_data(sampling_data, 34);
subplot(2, 3, 5);
plot(time_data, l_theta_to_sd_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, l_theta_to_sd_data, 'color', tar_color, 'LineWidth', 1)
ylim auto
title('左腿倾斜角度对右关节的输出');
grid on;

%%绘制左腿倾斜角速度对右关节的输出
l_thetav_to_sd_mea_data = my_data(sampling_data, 35);
l_thetav_to_sd_data = my_data(sampling_data, 36);
subplot(2, 3, 6);
plot(time_data, l_thetav_to_sd_mea_data, 'color', mea_color, 'LineWidth', 1)
hold on
plot(time_data, l_thetav_to_sd_data, 'color', tar_color, 'LineWidth', 1)
ylim auto
title('左腿倾斜角速度对右关节的输出');
grid on;

figure
%%绘制车体位移
s_mea_data = my_data(sampling_data, 37);
s_data = my_data(sampling_data, 38);

subplot(2, 3, 1);
plot(time_data, s_mea_data, 'color', mea_color, 'LineWidth', 1);
hold on;
plot(time_data, s_data, 'color', tar_color, 'LineWidth', 1);
ylim auto;
title('车体位移与车体目标位移'); 
grid on;


%%绘制支持力
R_support_mea_data = my_data(sampling_data, 39);
L_support_mea_data = my_data(sampling_data, 40);

subplot(2, 3, 2);
plot(time_data, R_support_mea_data, 'color', mea_color, 'LineWidth', 1);
ylim ([-50,200]);
title('右轮测得支持力'); 
grid on;

subplot(2, 3, 3);
plot(time_data, L_support_mea_data, 'color', mea_color, 'LineWidth', 1);
ylim ([-50,200]);
title('左轮测得支持力'); 
grid on;

%%绘制车体roll
roll_mea_data = my_data(sampling_data, 41);
roll_data = my_data(sampling_data, 42);

subplot(2, 3, 4);
plot(time_data, roll_mea_data, 'color', mea_color, 'LineWidth', 1);
hold on;
plot(time_data, roll_data, 'color', tar_color, 'LineWidth', 1);
ylim auto;
title('车体roll与车体目标roll'); 
grid on;

%%绘制车体pitch
pitch_mea_data = my_data(sampling_data, 43);
pitch_data = my_data(sampling_data, 44);

subplot(2, 3, 5);
plot(time_data, pitch_mea_data, 'color', mea_color, 'LineWidth', 1);
hold on;
plot(time_data, pitch_data, 'color', tar_color, 'LineWidth', 1);
ylim auto;
title('车体pitch与车体目标pitch'); 
grid on;

%%绘制车体右，左边侧向惯性力补偿
R_inertial_mea_data = my_data(sampling_data, 45);
L_inertial_mea_data = my_data(sampling_data, 46);

subplot(2, 3, 6);
plot(time_data, R_inertial_mea_data, 'color', mea_color, 'LineWidth', 1);
ylim auto;
title('右侧惯性力'); 
grid on;

figure
subplot(2, 3, 1);
plot(time_data, L_inertial_mea_data, 'color', mea_color, 'LineWidth', 1);
ylim auto;
title('左侧惯性力'); 
grid on;


