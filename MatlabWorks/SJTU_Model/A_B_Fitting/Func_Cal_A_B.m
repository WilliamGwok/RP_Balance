%Hello
%该函数返回虚拟杆腿长以及对应腿长下K矩阵的值
function [A_Ballance, B_Ballance, vir_Lr, vir_Ll] = Func_Cal_A_B(xc_r, yc_r, xp_r, yp_r, Ip_r, xc_l, yc_l, xp_l, yp_l, Ip_l, A_Cal, B_Cal)

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
% All_3 = [9.81  0.075  0.2574  vir_Lr  vir_Ll  lw_r_in   lw_l_in   lb_r_in   lb_l_in   0.497   0.002093033  Ip_r    Ip_l    16  0.19442  0.0419  0.473315  2.18667788];%整车实车2
All_3 = [9.81  0.075  0.2574  vir_Lr  vir_Ll  lw_r_in   lw_l_in   lb_r_in   lb_l_in   0.497   0.002093033  Ip_r    Ip_l    20  0.19442  0.0619  0.473315  2.18667788];
% A_Ballance = double(subs(A_Ballance, All_1, All_2));
% B_Ballance = double(subs(B_Ballance, All_1, All_2));
A_Ballance = double(subs(A_Cal, All_1, All_3));
B_Ballance = double(subs(B_Cal, All_1, All_3));


end