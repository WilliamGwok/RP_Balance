%Hello
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
     thetald1_r thetald2_r thetald1_l thetald2_l...
     xp yp

F =[
 
(Izz*rw*thetawd2_r - Izz*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l +...
2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l - 2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 -...
Izz*l_r*sin(thetal_r)*thetald1_r^2 - Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) -...
Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 -...
2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) +...
2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) - (Izz*rw*thetawd2_r - Izz*rw*thetawd2_l +...
Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l +...
2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 - Izz*l_r*sin(thetal_r)*thetald1_r^2 - Izz*l_l*thetald2_l*cos(thetal_l) +...
Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 -...
2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) +...
Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) + mw*rw*thetawd2_r == 0;%1


Iw*thetawd2_r - Tw_r + (rw*(Izz*rw*thetawd2_r - Izz*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l +...
2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 - Izz*l_r*sin(thetal_r)*thetald1_r^2 -...
Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 -...
2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) +...
Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r)))/(4*Rl^2) == 0;%2


(Izz*rw*thetawd2_l - Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r -...
2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r - Izz*l_l*sin(thetal_l)*thetald1_l^2 + Izz*l_r*sin(thetal_r)*thetald1_r^2 + Izz*l_l*thetald2_l*cos(thetal_l) -...
Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 -...
2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) +...
2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) - (Izz*rw*thetawd2_l - Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r +...
2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r - Izz*l_l*sin(thetal_l)*thetald1_l^2 +...
Izz*l_r*sin(thetal_r)*thetald1_r^2 + Izz*l_l*thetald2_l*cos(thetal_l) - Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 -...
Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) +...
Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) + mw*rw*thetawd2_l == 0;%3


Iw*thetawd2_l - Tw_l + (rw*(Izz*rw*thetawd2_l - Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r +...
2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r - Izz*l_l*sin(thetal_l)*thetald1_l^2 + Izz*l_r*sin(thetal_r)*thetald1_r^2 + Izz*l_l*thetald2_l*cos(thetal_l) -...
Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 -...
2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) +...
2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r)))/(4*Rl^2) == 0;%4


ml*(- lw_r*sin(thetal_r)*thetald1_r^2 + rw*thetawd2_r + lw_r*thetald2_r*cos(thetal_r)) + (Izz*rw*thetawd2_r - Izz*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_l +...
Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l - 2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l - 2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 -...
Izz*l_r*sin(thetal_r)*thetald1_r^2 - Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 -...
Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 + 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) +...
Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) - 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) - (Izz*rw*thetawd2_r - Izz*rw*thetawd2_l +...
Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l - 2*Rl^2*mw*rw*thetawd2_r +...
Izz*l_l*sin(thetal_l)*thetald1_l^2 - Izz*l_r*sin(thetal_r)*thetald1_r^2 - Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) -...
Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 +...
Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) == 0;%5


(l_l*ml*thetald2_l*sin(thetal_l))/2 - ml*((l_l*cos(thetal_l)*thetald1_l^2)/2 + (l_r*cos(thetal_r)*thetald1_r^2)/2 - lb_r*cos(thetal_r)*thetald1_r^2 +...
(l_l*thetald2_l*sin(thetal_l))/2 + (l_r*thetald2_r*sin(thetal_r))/2 - lb_r*thetald2_r*sin(thetal_r)) + (l_r*ml*thetald2_r*sin(thetal_r))/2 -...
lb_r*ml*thetald2_r*sin(thetal_r) + (l_l*ml*cos(thetal_l)*thetald1_l^2)/2 + (l_r*ml*cos(thetal_r)*thetald1_r^2)/2 - lb_r*ml*cos(thetal_r)*thetald1_r^2 == 0;%6


Tw_r - Tb_r + Il_r*thetald2_r + sin(thetal_r)*(lw_r*((l_l*mb*thetald2_l*sin(thetal_l))/4 - g*ml - (g*mb)/2 + (l_r*mb*thetald2_r*sin(thetal_r))/4 +...
(l_l*ml*thetald2_l*sin(thetal_l))/2 + (l_r*ml*thetald2_r*sin(thetal_r))/2 - (lb_l*ml*thetald2_l*sin(thetal_l))/2 - (lb_r*ml*thetald2_r*sin(thetal_r))/2 +...
(l_l*mb*cos(thetal_l)*thetald1_l^2)/4 + (l_r*mb*cos(thetal_r)*thetald1_r^2)/4 + (l_l*ml*cos(thetal_l)*thetald1_l^2)/2 + (l_r*ml*cos(thetal_r)*thetald1_r^2)/2 -...
(lb_l*ml*cos(thetal_l)*thetald1_l^2)/2 - (lb_r*ml*cos(thetal_r)*thetald1_r^2)/2) + lb_r*((l_l*mb*thetald2_l*sin(thetal_l))/4 -...
(g*mb)/2 + (l_r*mb*thetald2_r*sin(thetal_r))/4 - (lb_l*ml*thetald2_l*sin(thetal_l))/2 + (lb_r*ml*thetald2_r*sin(thetal_r))/2 +...
(l_l*mb*cos(thetal_l)*thetald1_l^2)/4 + (l_r*mb*cos(thetal_r)*thetald1_r^2)/4 - (lb_l*ml*cos(thetal_l)*thetald1_l^2)/2 + (lb_r*ml*cos(thetal_r)*thetald1_r^2)/2)) +...
cos(thetal_r)*((lb_r*(Izz*rw*thetawd2_r - Izz*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l -...
2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l - 2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 - Izz*l_r*sin(thetal_r)*thetald1_r^2 -...
Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 -...
2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 + 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) +...
2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) - 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r)))/(4*Rl^2) + (lw_r*(Izz*rw*thetawd2_r - Izz*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_l +...
Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l - 2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 -...
Izz*l_r*sin(thetal_r)*thetald1_r^2 - Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 -...
Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) +...
Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r)))/(4*Rl^2)) == 0;%7


ml*(- lw_l*sin(thetal_l)*thetald1_l^2 + rw*thetawd2_l + lw_l*thetald2_l*cos(thetal_l)) + (Izz*rw*thetawd2_l - Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l +...
Rl^2*mb*rw*thetawd2_r - 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r - 2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r - Izz*l_l*sin(thetal_l)*thetald1_l^2 +...
Izz*l_r*sin(thetal_r)*thetald1_r^2 + Izz*l_l*thetald2_l*cos(thetal_l) - Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 -...
Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 + 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) +...
Rl^2*l_r*mb*thetald2_r*cos(thetal_r) - 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) - (Izz*rw*thetawd2_l -...
Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r - 2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r -...
Izz*l_l*sin(thetal_l)*thetald1_l^2 + Izz*l_r*sin(thetal_r)*thetald1_r^2 + Izz*l_l*thetald2_l*cos(thetal_l) - Izz*l_r*thetald2_r*cos(thetal_r) -...
Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 +...
Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) == 0;%8


(l_l*ml*thetald2_l*sin(thetal_l))/2 - ml*((l_l*cos(thetal_l)*thetald1_l^2)/2 + (l_r*cos(thetal_r)*thetald1_r^2)/2 - lb_l*cos(thetal_l)*thetald1_l^2 +...
(l_l*thetald2_l*sin(thetal_l))/2 + (l_r*thetald2_r*sin(thetal_r))/2 - lb_l*thetald2_l*sin(thetal_l)) + (l_r*ml*thetald2_r*sin(thetal_r))/2 -...
lb_l*ml*thetald2_l*sin(thetal_l) + (l_l*ml*cos(thetal_l)*thetald1_l^2)/2 + (l_r*ml*cos(thetal_r)*thetald1_r^2)/2 - lb_l*ml*cos(thetal_l)*thetald1_l^2 == 0;%9


Tw_l - Tb_l + Il_l*thetald2_l + sin(thetal_l)*(lw_l*((l_l*mb*thetald2_l*sin(thetal_l))/4 - g*ml - (g*mb)/2 + (l_r*mb*thetald2_r*sin(thetal_r))/4 +...
(l_l*ml*thetald2_l*sin(thetal_l))/2 + (l_r*ml*thetald2_r*sin(thetal_r))/2 - (lb_l*ml*thetald2_l*sin(thetal_l))/2 - (lb_r*ml*thetald2_r*sin(thetal_r))/2 +...
(l_l*mb*cos(thetal_l)*thetald1_l^2)/4 + (l_r*mb*cos(thetal_r)*thetald1_r^2)/4 + (l_l*ml*cos(thetal_l)*thetald1_l^2)/2 + (l_r*ml*cos(thetal_r)*thetald1_r^2)/2 -...
(lb_l*ml*cos(thetal_l)*thetald1_l^2)/2 - (lb_r*ml*cos(thetal_r)*thetald1_r^2)/2) + lb_l*((l_l*mb*thetald2_l*sin(thetal_l))/4 - (g*mb)/2 +...
(l_r*mb*thetald2_r*sin(thetal_r))/4 + (lb_l*ml*thetald2_l*sin(thetal_l))/2 - (lb_r*ml*thetald2_r*sin(thetal_r))/2 + (l_l*mb*cos(thetal_l)*thetald1_l^2)/4 +...
(l_r*mb*cos(thetal_r)*thetald1_r^2)/4 + (lb_l*ml*cos(thetal_l)*thetald1_l^2)/2 - (lb_r*ml*cos(thetal_r)*thetald1_r^2)/2)) + cos(thetal_l)*((lb_l*(Izz*rw*thetawd2_l -...
Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r - 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r - 2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r -...
Izz*l_l*sin(thetal_l)*thetald1_l^2 + Izz*l_r*sin(thetal_r)*thetald1_r^2 + Izz*l_l*thetald2_l*cos(thetal_l) - Izz*l_r*thetald2_r*cos(thetal_r) -...
Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 + 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 +...
Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) - 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r)))/(4*Rl^2) +...
(lw_l*(Izz*rw*thetawd2_l - Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r - 2*Rl^2*mw*rw*thetawd2_l +...
2*Rl^2*mw*rw*thetawd2_r - Izz*l_l*sin(thetal_l)*thetald1_l^2 + Izz*l_r*sin(thetal_r)*thetald1_r^2 + Izz*l_l*thetald2_l*cos(thetal_l) - Izz*l_r*thetald2_r*cos(thetal_r) -...
Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 +...
Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r)))/(4*Rl^2)) == 0;%10


mb*((rw*(thetawd2_l + thetawd2_r))/2 - (l_l*sin(thetal_l)*thetald1_l^2)/2 - (l_r*sin(thetal_r)*thetald1_r^2)/2 + (l_l*thetald2_l*cos(thetal_l))/2 +...
(l_r*thetald2_r*cos(thetal_r))/2) - (Izz*rw*thetawd2_l - Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r - 2*Rl^2*ml*rw*thetawd2_l +...
2*Rl^2*ml*rw*thetawd2_r - 2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r - Izz*l_l*sin(thetal_l)*thetald1_l^2 + Izz*l_r*sin(thetal_r)*thetald1_r^2 +...
Izz*l_l*thetald2_l*cos(thetal_l) - Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 +...
2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) -...
2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) - (Izz*rw*thetawd2_r - Izz*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_l +...
Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l - 2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l - 2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 -...
Izz*l_r*sin(thetal_r)*thetald1_r^2 - Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 -...
Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 + 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) +...
Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) - 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) == 0;%11


(l_l*mb*thetald2_l*sin(thetal_l))/2 - mb*((l_l*cos(thetal_l)*thetald1_l^2)/2 + (l_r*cos(thetal_r)*thetald1_r^2)/2 + (l_l*thetald2_l*sin(thetal_l))/2 +...
(l_r*thetald2_r*sin(thetal_r))/2) + (l_r*mb*thetald2_r*sin(thetal_r))/2 + (l_l*mb*cos(thetal_l)*thetald1_l^2)/2 + (l_r*mb*cos(thetal_r)*thetald1_r^2)/2 == 0;%12


Tb_l + Tb_r + Ib*thetabd2 + lc*cos(thetab)*((Izz*rw*thetawd2_l - Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r - 2*Rl^2*ml*rw*thetawd2_l +...
2*Rl^2*ml*rw*thetawd2_r - 2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r - Izz*l_l*sin(thetal_l)*thetald1_l^2 + Izz*l_r*sin(thetal_r)*thetald1_r^2 +...
Izz*l_l*thetald2_l*cos(thetal_l) - Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 +...
2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) -...
2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) + (Izz*rw*thetawd2_r - Izz*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_l +...
Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l - 2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l - 2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 -...
Izz*l_r*sin(thetal_r)*thetald1_r^2 - Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 -...
Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 + 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) +...
Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) - 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2)) +...
lc*sin(thetab)*((l_l*mb*thetald2_l*sin(thetal_l))/2 - g*mb + (l_r*mb*thetald2_r*sin(thetal_r))/2 + (l_l*mb*cos(thetal_l)*thetald1_l^2)/2 +...
(l_r*mb*cos(thetal_r)*thetald1_r^2)/2) == 0;%13


Izz*((l_l*sin(thetal_l)*thetald1_l^2 - l_r*sin(thetal_r)*thetald1_r^2 - l_l*thetald2_l*cos(thetal_l) + l_r*thetald2_r*cos(thetal_r))/(2*Rl) - (rw*(thetawd2_l -...
thetawd2_r))/(2*Rl)) + Rl*((Izz*rw*thetawd2_l - Izz*rw*thetawd2_r + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l + 2*Rl^2*ml*rw*thetawd2_r +...
2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r - Izz*l_l*sin(thetal_l)*thetald1_l^2 + Izz*l_r*sin(thetal_r)*thetald1_r^2 + Izz*l_l*thetald2_l*cos(thetal_l) -...
Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 - 2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 -...
2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) + 2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) +...
2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2) - (Izz*rw*thetawd2_r - Izz*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_l + Rl^2*mb*rw*thetawd2_r + 2*Rl^2*ml*rw*thetawd2_l +...
2*Rl^2*ml*rw*thetawd2_r + 2*Rl^2*mw*rw*thetawd2_l + 2*Rl^2*mw*rw*thetawd2_r + Izz*l_l*sin(thetal_l)*thetald1_l^2 - Izz*l_r*sin(thetal_r)*thetald1_r^2 -...
Izz*l_l*thetald2_l*cos(thetal_l) + Izz*l_r*thetald2_r*cos(thetal_r) - Rl^2*l_l*mb*sin(thetal_l)*thetald1_l^2 - Rl^2*l_r*mb*sin(thetal_r)*thetald1_r^2 -...
2*Rl^2*lw_l*ml*sin(thetal_l)*thetald1_l^2 - 2*Rl^2*lw_r*ml*sin(thetal_r)*thetald1_r^2 + Rl^2*l_l*mb*thetald2_l*cos(thetal_l) + Rl^2*l_r*mb*thetald2_r*cos(thetal_r) +...
2*Rl^2*lw_l*ml*thetald2_l*cos(thetal_l) + 2*Rl^2*lw_r*ml*thetald2_r*cos(thetal_r))/(4*Rl^2)) == 0];%14



F_Bomb = simplify(F);%测得2 4 7 10 13是有用的，其它等式为常数
F_Smile = [F(2);
           F(4);
           F(7);
           F(10);
           F(13)];
%角度近似

F_Foreign = [phid2 - ( (l_l*sin(thetal_l)*thetald1_l^2 - l_r*sin(thetal_r)*thetald1_r^2 - l_l*thetald2_l*cos(thetal_l) + l_r*thetald2_r*cos(thetal_r))/(2*Rl) - (rw*(thetawd2_l - thetawd2_r))/(2*Rl) ) == 0];
phid2 = solve(F_Foreign, phid2);

F_Foreign = [sd2 - ( (rw*(thetawd2_l + thetawd2_r))/2 - (l_l*sin(thetal_l)*thetald1_l^2)/2 - (l_r*sin(thetal_r)*thetald1_r^2)/2 + (l_l*thetald2_l*cos(thetal_l))/2 + (l_r*thetald2_r*cos(thetal_r))/2 ) == 0];
sd2 = solve(F_Foreign, sd2);

Xdot = [thetawd2_r thetawd2_l thetald2_r thetald2_l thetabd2];    
[sovthetawd2_r, sovthetawd2_l, sovthetald2_r, sovthetald2_l, sovthetabd2] = solve(F_Smile, Xdot);
sovphid2 = subs(phid2, [thetawd2_r thetawd2_l thetald2_r thetald2_l], [sovthetawd2_r sovthetawd2_l sovthetald2_r sovthetald2_l]);
sovsd2 = subs(sd2, [thetawd2_r thetawd2_l thetald2_r thetald2_l], [sovthetawd2_r sovthetawd2_l sovthetald2_r sovthetald2_l]);

X = [s; sd1; phi; phid1; thetal_l; thetald1_l; thetal_r; thetald1_r; thetab; thetabd1];%状态变量
U = [Tw_l; Tw_r; Tb_l; Tb_r];%控制变量
Xdot = [sd1; sovsd2; phid1; sovphid2; thetald1_l; sovthetald2_l; thetald1_r; sovthetald2_r; thetabd1; sovthetabd2];

A = jacobian(Xdot, X);
B = jacobian(Xdot, U);

X_replace = [sd1; phid1; thetal_l; thetald1_l; thetal_r; thetald1_r; thetab; thetabd1];
A_Ballance = subs(A, [X_replace;U], zeros(12, 1));
B_Ballance = subs(B, [X_replace;U], zeros(12, 1));
C_Ballance = eye(10);
D_Ballance = zeros(10, 4);

Model_1 = [g    rw   Rl   l_r   l_l  lw_r  lw_l   lb_r   lb_l   mw    Iw     Il_r   Il_l   mb    Ib     lc    Izz    ml];
Model_2 = [9.81 0.06 0.18 0.26 0.26  0.13  0.13   0.13   0.13   0.2  0.00036 0.3 0.3 10.00 0.1271 0.05  0.1083  1.46];

A_Ballance = double(subs(A_Ballance, Model_1, Model_2));
B_Ballance = double(subs(B_Ballance, Model_1, Model_2));

Q = double(diag([1 1 1 1 1 1 1 1 5 2]));
R = double(diag([1 1 1 1]));
sys = ss(A_Ballance, B_Ballance, C_Ballance, D_Ballance);
K = lqr(sys, Q, R)




%-- 2023/9/25 12:52 --%




                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 