%Hello
%该函数返回虚拟杆腿长以及对应腿长下K矩阵的值
function [K_unit, vir_Lr, vir_Ll, A_Ballance, B_Ballance, C_Ballance, D_Ballance] = Func_Cal_K(xc_r, yc_r, xp_r, yp_r, Ip_r, xc_l, yc_l, xp_l, yp_l, Ip_l, A_Cal, B_Cal, C_Cal, D_Cal)

syms s sb hb sl_r  hl_r sl_l  hl_l...
     phi thetaw_r thetaw_l thetal_r thetal_l thetab...
     g rw Rl l_r l_l lw_r lw_l lb_r lb_l mw Iw Il_r Il_l mb Ib lc Izz ml...
     Fws_r Fwh_r Fbs_r Fbh_r Tw_r Tb_r f_r...
     Fws_l Fwh_l Fbs_l Fbh_l Tw_l Tb_l f_l
%替代变量
syms sd1 sd2 sbd1 sbd2 hbd1 hbd2 sld1_r sld2_r sld1_l sld2_l...
     hld1_r hld2_r hld1_l hld2_l...
     phid1 phid2 thetawd1_r thetawd2_r thetawd1_l thetawd2_l thetabd1 thetabd2...
     thetald1_r thetald2_r thetald1_l thetald2_l

global l5;

vir_Lr = sqrt((xc_r - l5/2)^2 + yc_r^2);
lw_r_in = sqrt((xc_r - xp_r)^2 + (yc_r - yp_r)^2);
lb_r_in = vir_Lr - lw_r_in;

vir_Ll = sqrt((xc_l - l5/2)^2 + yc_l^2);
lw_l_in = sqrt((xc_l - xp_l)^2 + (yc_l - yp_l)^2);
lb_l_in = vir_Ll - lw_l_in;


All_1 = [g     rw    Rl    l_r     l_l     lw_r      lw_l      lb_r      lb_l      mw    Iw       Il_r    Il_l    mb     Ib      lc     Izz     ml];%有关腿的都是用单腿数据
All_2 = [9.81  0.06  0.18  vir_Lr  vir_Ll  lw_r_in   lw_l_in   lb_r_in   lb_l_in   0.2   0.00036  Ip_r    Ip_l    10.00  0.0708  -0.02  0.1271  1.44];%仿真
%实车      g     rw     Rl     l_r     l_l     lw_r      lw_l      lb_r      lb_l      mw         Iw       Il_r    Il_l    mb     Ib        lc         Izz       ml
% All_3 = [9.81  0.075  0.2545  vir_Lr  vir_Ll  lw_r_in   lw_l_in   lb_r_in   lb_l_in   0.609   0.001627433  Ip_r    Ip_l    5.332  0.071169  -0.089707  0.141715  2.4313];%实车底盘1
% All_3 = [9.81  0.075  0.2574  vir_Lr  vir_Ll  lw_r_in   lw_l_in   lb_r_in   lb_l_in   0.709   0.002093033  Ip_r    Ip_l    7.8  0.131272  -0.002  0.46497758  2.18667788];%实车底盘2
% All_3 = [9.81  0.075  0.2574  vir_Lr  vir_Ll  lw_r_in   lw_l_in   lb_r_in   lb_l_in   0.709   0.002093033  Ip_r    Ip_l    12.5  0.19442  0.0419  0.473315  2.18667788];%整车实车1
All_3 = [9.81  0.075  0.2574  vir_Lr  vir_Ll  lw_r_in   lw_l_in   lb_r_in   lb_l_in   0.497   0.002093033  Ip_r    Ip_l    21  0.19442  0.165  0.473315  2.18667788];%整车实车2
%mw 0.497 ml 2.18667788

% A_Ballance = double(subs(A_Ballance, All_1, All_2));
% B_Ballance = double(subs(B_Ballance, All_1, All_2));
A_Ballance = double(subs(A_Cal, All_1, All_3));
B_Ballance = double(subs(B_Cal, All_1, All_3));
C_Ballance = C_Cal;
D_Ballance = D_Cal;

%               s      yaw   thetal_l thetal_r pitch
%根据Bryson法则，各个变量最大可接受值为 x 0.6m xdot 3.5m/s yaw 0.5rad yaw_v 8rad/s 
%l_l 0.5rad l_l_v 5rad/s  l_r 0.5rad l_r_v 5rad/s  pitch 0.17rad pitch_v
%5rad/s
x1_ = 1 / 0.5^2;
x2_ = 1 / 1^2;
x3_ = 1 / 0.5^2;
x4_ = 1 / 8^2;
x5_ = 1 / 0.5^2;
x6_ = 1 / 5^2;
x7_ = 1 / 0.5^2;
x8_ = 1 / 5^2;
x9_ = 1 / 0.3^2;
x10_ = 1 / 6^2;

% Q = double(diag(1*[1e0*x1_   5e2*x2_   3e2*x3_   10*x4_   6e1*x5_   6*x6_   6e1*x7_   6*x8_   6e2*x9_   1*x10_]));
% R = double(diag(1*[20 20 0.1 0.1]));
Q = double(diag(1*[120*x1_   1*x2_   100*x3_   1*x4_   1*x5_   1*x6_   1*x7_   1*x8_   500*x9_   1*x10_]));
R = double(diag(1*[30 30 0.6 0.6]));
sys = ss(A_Ballance, B_Ballance, C_Ballance, D_Ballance);
K_unit = lqr(sys, Q, R);

% Q = double(diag(1*[180*x1_   50*x2_   300*x3_   50*x4_   30*x5_   10*x6_   30*x7_   10*x8_   1500*x9_   1*x10_]));
% R = double(diag(1*[15 15 0.5 0.5]));%长腿飞坡

% Q = double(diag(1*[180*x1_   30*x2_   200*x3_   30*x4_   80*x5_   15*x6_   80*x7_   15*x8_   370*x9_   1*x10_]));
% R = double(diag(1*[50 50 0.1 0.1]));保守

% Q = double(diag(1*[160*x1_   50*x2_   200*x3_   10*x4_   65*x5_   30*x6_   65*x7_   30*x8_   550*x9_   1*x10_]))
% R = double(diag(1*[30 30 0.3 0.3]));//飞坡

% Q = double(diag(1*[150*x1_   50*x2_   200*x3_   10*x4_   60*x5_   30*x6_   60*x7_   30*x8_   600*x9_   1*x10_]));
% R = double(diag(1*[35 35 0.3 0.3]));快速上小坡

% Q = double(diag(1*[150*x1_   100*x2_   200*x3_   10*x4_   40*x5_   20*x6_   40*x7_   20*x8_   1400*x9_   1*x10_]));
% R = double(diag(1*[45 45 1 1]));

% Q = double(diag(1*[75*x1_   70*x2_   150*x3_   1*x4_   40*x5_   10*x6_   40*x7_   10*x8_   370*x9_   0.1*x10_]))
% R = double(diag(0.5*[45 45 0.06 0.06]));%平地可以稍快一点，但是上坡很慢

% All_3 = [9.81  0.075  0.2574  vir_Lr  vir_Ll  lw_r_in   lw_l_in   lb_r_in   lb_l_in   0.497   0.002093033  Ip_r    Ip_l    20  0.19442  0.0619  0.473315  2.18667788];%整车实车2
% Q = double(diag(1*[50*x1_   20*x2_   150*x3_   1*x4_   20*x5_   1*x6_   20*x7_   1*x8_   400*x9_   0.1*x10_]));
% R = double(diag(0.5*[30 30 0.07 0.07]));飞坡加平地


% Q = double(diag(1*[50*x1_   20*x2_   200*x3_   0.1*x4_   20*x5_   1*x6_   20*x7_   1*x8_   650*x9_   0.1*x10_]));
% R = double(diag(0.5*[30 30 0.07 0.07]));平地

% Q = double(diag([8000 100 10000 10 500 100 500 100 18000 1]));
% R = double(diag([1.8e2 1.8e2 15e-1 15e-1]));








%-- 2023/9/25 12:52 --%


% Q = double(diag([8000 100 10000 10 1000 100 1000 100 50000 1]));
% R = double(diag([1.8e2 1.8e2 15e-1 15e-1]));%1 1 0.5 0.5   ????????????????????10 10 0.1 0.1

% 
% Q = double(diag([240 1 240 1 60 1 60 1 5500 1]));
% R = double(diag([20 20 0.1 0.1]));%1 1 0.5 0.5   ????????????????????10 10 0.1 0.1
           

% Q = double(diag([400 1 500 1 600 1 600 1 8000 1]));
% R = double(diag([20 20 0.2 0.2]));%1 1 0.5 0.5   ????????????????????10 10 0.1 0.1

% Q = double(diag([150 100 150 1 100 1 100 1 5000 1]));
% R = double(diag([10 10 0.1 0.1]));%1 1 0.5 0.5   ????????????????????10 10 0.1 0.1



% Q = double(diag([8000 100 10000 10 500 100 500 100 25000 1]));
% R = double(diag([1.8e2 1.8e2 15e-1 15e-1]));%1 1 0.5 0.5   ????????????????????10 10 0.1 0.1



% Q = double(diag(1*[50*x1_   20*x2_   200*x3_   0.1*x4_   5*x5_   0.1*x6_   5*x7_   0.1*x8_   1500*x9_   0.1*x10_]));
% R = double(diag([50 50 0.05 0.05]));%1 1 0.5 0.5   ????????????????????10 10 0.1 0.1

