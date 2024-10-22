clc;clear;
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


X = [s; sd1; phi; phid1; thetal_l; thetald1_l; thetal_r; thetald1_r; thetab; thetabd1];%状态变量

F = [(Iw*l_l/rw + mw*rw*l_l + ml*rw*lb_l)*thetawd2_l + (ml*lw_l*lb_l - Il_l)*thetald2_l + (ml*lw_l + (1/2)*mb*l_l)*g*thetal_l + Tb_l - Tw_l*(1 + l_l/rw) == 0;
     (Iw*l_r/rw + mw*rw*l_r + ml*rw*lb_r)*thetawd2_r + (ml*lw_r*lb_r - Il_r)*thetald2_r + (ml*lw_r + (1/2)*mb*l_r)*g*thetal_r + Tb_r - Tw_r*(1 + l_r/rw) == 0;
     -(mw*rw*rw + Iw + ml*rw*rw + (1/2)*mb*rw*rw)*thetawd2_l - (mw*rw*rw + Iw + ml*rw*rw + (1/2)*mb*rw*rw)*thetawd2_r - (ml*rw*lw_l + (1/2)*mb*rw*l_l)*thetald2_l - (ml*rw*lw_r + (1/2)*mb*rw*l_r)*thetald2_r + Tw_l + Tw_r == 0;
     (mw*rw*lc + Iw*lc/rw + ml*rw*lc)*thetawd2_l + (mw*rw*lc + Iw*lc/rw + ml*rw*lc)*thetawd2_r + ml*lw_l*lc*thetald2_l + ml*lw_r*lc*thetald2_r - Ib*thetabd2 + mb*g*lc*thetab - (Tw_l +Tw_r)*lc/rw - (Tb_l + Tb_r) == 0;
     ((1/2)*Izz*rw/Rl + Iw*Rl/rw)*thetawd2_l - ((1/2)*Izz*rw/Rl +Iw*Rl/rw)*thetawd2_r + (1/2)*Izz*(l_l/Rl)*thetald2_l - (1/2)*Izz*(l_r/Rl)*thetald2_r - Tw_l*(Rl/rw) + Tw_r*(Rl/rw) == 0];

Xdot = [thetawd2_l thetawd2_r thetald2_l thetald2_r thetabd2];
[thetawd2_l, thetawd2_r, thetald2_l, thetald2_r, thetabd2] = solve(F, Xdot);

F_Foreign = [phid2 - ((l_l*sin(thetal_l)*thetald1_l^2 - l_r*sin(thetal_r)*thetald1_r^2 - l_l*thetald2_l*cos(thetal_l) + l_r*thetald2_r*cos(thetal_r))/(2*Rl) - (rw*(thetawd2_l - thetawd2_r))/(2*Rl) ) == 0];
F_Foreign = subs(F_Foreign, [sin(thetal_l) sin(thetal_r) cos(thetal_l) cos(thetal_r) thetald1_l^2 thetald1_r^2], [thetal_l thetal_r 1 1 0 0]);
phid2 = solve(F_Foreign, phid2);

F_Foreign = [sd2 - ( (rw*(thetawd2_l + thetawd2_r))/2 - (l_l*sin(thetal_l)*thetald1_l^2)/2 - (l_r*sin(thetal_r)*thetald1_r^2)/2 + (l_l*thetald2_l*cos(thetal_l))/2 + (l_r*thetald2_r*cos(thetal_r))/2 ) == 0];
F_Foreign = subs(F_Foreign, [sin(thetal_l) sin(thetal_r) thetald1_l thetald1_r cos(thetal_l) cos(thetal_r)], [thetal_l thetal_r 0 0 1 1]);
sd2 = solve(F_Foreign, sd2);

Come =  [g          rw      Rl     l_r     l_l    lw_r    lw_l      lb_r   lb_l        mw      Iw           Il_r      Il_l      mb     Ib        lc       Izz       ml];
Leave = [9.81   0.075  0.2545  0.1787  0.1787  0.1067   0.1067   0.0720   0.0720   0.709   0.002093033  0.0437    0.0437    12.5    0.19442   0.0419   0.473315   2.18667788];

Xdot = [sd1; sd2; phid1; phid2; thetald1_l; thetald2_l; thetald1_r; thetald2_r; thetabd1; thetabd2];
U = [Tw_l; Tw_r; Tb_l; Tb_r];%控制变量

A = jacobian(Xdot, X);
B = jacobian(Xdot, U);

A = subs(A, Come, Leave);
B = subs(B, Come, Leave);

X_replace = [sd1; phid1; thetal_l; thetald1_l; thetal_r; thetald1_r; thetab; thetabd1];

A = subs(A, [X_replace;U], zeros(12, 1));
A_Ballance = double(vpa(A));
B = subs(B, [X_replace;U], zeros(12, 1));
B_Ballance = double(vpa(B));

C_Ballance = eye(10);
D_Ballance = zeros(10, 4);

Q = double(diag([50 1 50 1 10 1 10 1 5000 1]));
R = double(diag([1 1 0.25 0.25]));
sys = ss(A_Ballance, B_Ballance, C_Ballance, D_Ballance);
K = lqr(sys, Q, R)

