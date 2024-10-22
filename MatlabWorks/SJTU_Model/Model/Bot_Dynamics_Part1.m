%Hello
clc;clear;
syms s(t) sb(t) hb(t) sl_r(t)  hl_r(t) sl_l(t)  hl_l(t)...
     phi(t) thetaw_r(t) thetaw_l(t) thetal_r(t) thetal_l(t) thetab(t)...
     g rw Rl l_r l_l lw_r lw_l lb_r lb_l mw Iw ml Il_r Il_l mb Ib lc Izz...
     Fws_r Fwh_r Fbs_r Fbh_r Tw_r Tb_r f_r...
     Fws_l Fwh_l Fbs_l Fbh_l Tw_l Tb_l f_l
%替代变量
syms sd1(t) sd2 sbd1(t) sbd2 hbd1(t) hbd2 sld1_r(t) sld2_r sld1_l(t) sld2_l...
     hld1_r(t) hld2_r hld1_l(t) hld2_l...
     phid1(t) phid2 thetawd1_r(t) thetawd2_r thetawd1_l(t) thetawd2_l thetabd1(t) thetabd2...
     thetald1_r(t) thetald2_r thetald1_l(t) thetald2_l

%运动学方程
s = (rw/2)*(thetaw_l + thetaw_r);%1
phi = (rw/(2*Rl))*(thetaw_r - thetaw_l) + (1/(2*Rl))*(l_r*sin(thetal_r) - l_l*sin(thetal_l));%1
sb = (rw/2)*(thetaw_r + thetaw_l) + (1/2)*(l_r*sin(thetal_r) + l_l*sin(thetal_l));%1
hb = (1/2)*(l_r*cos(thetal_r) + l_l*cos(thetal_l)); %不代入hb进行计算，方便验证%1
sl_r = rw*thetaw_r + lw_r*sin(thetal_r);%1
sl_l = rw*thetaw_l + lw_l*sin(thetal_l);%1
hl_r = hb - lb_r*cos(thetal_r);%1
hl_l = hb - lb_l*cos(thetal_l);%1

% %求一，二阶导
sd1 = subs(diff(s), [diff(thetaw_l(t), t) diff(thetaw_r(t), t)], [thetawd1_l thetawd1_r]);%1
sd2 = subs(diff(sd1), [diff(thetawd1_l(t), t) diff(thetawd1_r(t), t)], [thetawd2_l thetawd2_r]);%1


hld1_r = subs(diff(hl_r), [diff(thetal_r(t), t) diff(thetal_l(t), t)], [thetald1_r thetald1_l]);%1
hld2_r = subs(diff(hld1_r), [diff(thetald1_r(t), t) diff(thetal_r(t), t) diff(thetald1_l(t), t) diff(thetal_l(t), t)], [thetald2_r thetald1_r thetald2_l thetald1_l]);%1

hld1_l = subs(diff(hl_l), [diff(thetal_l(t), t) diff(thetal_r(t), t)], [thetald1_l thetald1_r]);%1
hld2_l = subs(diff(hld1_l), [diff(thetald1_l(t), t) diff(thetal_l(t), t) diff(thetald1_r(t), t) diff(thetal_r(t), t)], [thetald2_l thetald1_l thetald2_r thetald1_r]);%1


phid1 = subs(diff(phi,t), [diff(thetal_l(t), t) diff(thetal_r(t), t) diff(thetaw_l(t), t) diff(thetaw_r(t), t)], ...%1
             [thetald1_l thetald1_r thetawd1_l thetawd1_r]);
phid2 = subs(diff(phid1), [diff(thetald1_l(t), t) diff(thetald1_r(t), t) diff(thetawd1_l(t), t) diff(thetawd1_r(t), t) diff(thetal_l(t), t) diff(thetal_r(t), t)], ...%1
             [thetald2_l thetald2_r thetawd2_l thetawd2_r thetald1_l thetald1_r]);


sbd1 = subs(diff(sb), [diff(thetal_l(t), t) diff(thetal_r(t), t) diff(thetaw_l(t), t) diff(thetaw_r(t), t)], ...%1
            [thetald1_l thetald1_r thetawd1_l thetawd1_r]);
sbd2 = subs(diff(sbd1), [diff(thetald1_l(t), t) diff(thetald1_r(t), t) diff(thetawd1_l(t), t) diff(thetawd1_r(t), t) diff(thetal_l(t), t) diff(thetal_r(t), t)], ...%1
             [thetald2_l thetald2_r thetawd2_l thetawd2_r thetald1_l thetald1_r]);


hbd1 = subs(diff(hb),[diff(thetal_l(t), t) diff(thetal_r(t), t)], [thetald1_l thetald1_r]);%1
hbd2 = subs(diff(hbd1), [diff(thetald1_l(t), t) diff(thetald1_r(t), t) diff(thetal_l(t), t) diff(thetal_r(t), t)], [thetald2_l thetald2_r thetald1_l thetald1_r]);%1


sld1_r = subs(diff(sl_r), [diff(thetaw_r(t), t) diff(thetal_r(t), t)], [thetawd1_r thetald1_r]);%1
sld2_r = subs(diff(sld1_r), [diff(thetawd1_r(t), t) diff(thetald1_r(t), t) diff(thetal_r(t), t)], [thetawd2_r thetald2_r thetald1_r]);%1


sld1_l = subs(diff(sl_l), [diff(thetaw_l(t), t) diff(thetal_l(t), t)], [thetawd1_l thetald1_l]);%1
sld2_l = subs(diff(sld1_l), [diff(thetawd1_l(t), t) diff(thetald1_l(t), t) diff(thetal_l(t), t)], [thetawd2_l thetald2_l thetald1_l]);%1


%%动力学方程

%对左右驱动轮
%右
f1_r = [mw*rw*thetawd2_r - f_r + Fws_r == 0;
        Iw*thetawd2_r - Tw_r + f_r*rw == 0];%1
%左
f1_l = [mw*rw*thetawd2_l - f_l+ Fws_l == 0;
        Iw*thetawd2_l - Tw_l + f_l*rw == 0];%1

%对左右腿
%右
f2_r = [ml*sld2_r - Fws_r + Fbs_r == 0;
        ml*hld2_r - Fwh_r + Fbh_r + ml*g == 0;
        Il_r*thetald2_r - (  (Fwh_r*lw_r + Fbh_r*lb_r)*sin(thetal_r) - (Fws_r*lw_r + Fbs_r*lb_r)*cos(thetal_r) - Tw_r + Tb_r  ) == 0];%1

%左
f2_l = [ml*sld2_l - Fws_l + Fbs_l == 0;
        ml*hld2_l - Fwh_l + Fbh_l + ml*g == 0;
        Il_l*thetald2_l - (  (Fwh_l*lw_l + Fbh_l*lb_l)*sin(thetal_l) - (Fws_l*lw_l + Fbs_l*lb_l)*cos(thetal_l) - Tw_l + Tb_l  ) == 0];%1

%对机体
f3 = [mb*sbd2 - Fbs_r - Fbs_l == 0;
      mb*hbd2 - Fbh_r - Fbh_l + mb*g == 0;
      Ib*thetabd2 - (  (Fbh_r +Fbh_l)*lc*sin(thetab) - (Fbs_r + Fbs_l)*lc*cos(thetab) - (Tb_l + Tb_r)  ) == 0];%1

% 旋转 
f4 = [Izz*phid2 - (f_r - f_l)*Rl == 0];%1

%支持力
f5 = [Fwh_l - Fwh_r == 0];%1


%总共15个方程,至此方程全部列出但未简化
F = [f1_r; ...
     f1_l; ...
     f2_r; ...
     f2_l; ...
     f3; ...
     f4; ...
     f5];%1

%得出中间变量
mid = [f_r Fws_r f_l Fws_l Fbs_r Fwh_r Fbh_r Fbs_l Fwh_l Fbh_l Tw_r Tw_l Tb_r Tb_l Ib];%后面五个是凑数用的

[sovf_r, sovFws_r, sovf_l, sovFws_l, sovFbs_r, sovFwh_r, sovFbh_r, sovFbs_l, sovFwh_l, sovFbh_l, Tw_r, Tw_l, Tb_r, Tb_l, Ib] = solve(F, mid);

mid = [f_r Fws_r f_l Fws_l Fbs_r Fwh_r Fbh_r Fbs_l Fwh_l Fbh_l];

sovmid = [sovf_r sovFws_r sovf_l sovFws_l sovFbs_r sovFwh_r sovFbh_r sovFbs_l sovFwh_l sovFbh_l];

Fb_l = sqrt(sovFbs_l^2 + sovFbh_l^2);

%消去中间变量
F = subs(F, mid, sovmid)%消去中间变量






