clc;clear;
syms R L Lm l Mw Mp M Iw Ip Im theta thetad1 thetad2 x xdot1 xdot2 phi phidot1 phidot2 T Tp N P Nm Pm Nf g...
     theta_Q thetad1_Q x_Q xdot1_Q phi_Q phidot1_Q T_R Tp_R

%对驱动轮合外力合转矩分析，计算驱动轮加速度
f1 = [Mw*xdot2 - (Nf -N) == 0;
      Iw*xdot2/R - (T - Nf*R) == 0];
x1 = [xdot2 N];
[sovxdot2, ~] = solve(f1,x1);

%对摆杆合外力合转矩分析
f2 = [N - Nm - (Mp*(xdot2 + L*thetad2*cos(theta) - L*(thetad1)^2*sin(theta))) == 0;
      P - Pm - Mp*g - (Mp*(-L*thetad2*sin(theta) - L*(thetad1)^2*cos(theta))) == 0;
      Im*thetad2 - ((P*L + Pm*Lm)*sin(theta) - (N*L + Nm*Lm)*cos(theta) - T + Tp) == 0];

%对机体合外力合转矩分析
f3 = [Nm - ( M* (xdot2 + (L + Lm)*thetad2*cos(theta) - (L + Lm)*(thetad1)^2*sin(theta) - l*phidot2*cos(phi) + l*(phidot1)^2*sin(phi)) ) == 0;
      Pm - M*g - (M* (  -(L + Lm)*thetad2*sin(theta) - (L + Lm)*(thetad1)^2*cos(theta) - l*phidot2*sin(phi) - l*(phidot1)^2*cos(phi)  ) ) == 0;
      Im*phidot2 - ( Tp + Nm*l*cos(phi) + Pm*l*sin(phi) ) == 0];

f = [f1;f2;f3];

%消去中间变量用
Nm_c = solve(f(6),Nm);
N_c = solve(subs(f(3), Nm, Nm_c), N);
Pm_c = solve(f(7), Pm);
P_c = solve(subs(f(4), Pm, Pm_c), P);
Nf_c = solve(subs(f(1), N, N_c), Nf);

%列出非线性化方程（消去中间变量）
xdot2_f = subs((sovxdot2 - xdot2), Nf, Nf_c);
thetad2_f = subs(f(5), [Nm, Pm, P], [Nm_c, Pm_c, P_c]);
phidot2_f = subs(f(8), [Nm, Pm, P], [Nm_c, Pm_c, P_c]);
f_nl = [xdot2_f == 0;thetad2_f;phidot2_f];
[xdot2_c, thetad2_c, phidot2_c] = solve(f_nl, [xdot2, thetad2, phidot2]); 

%状态变量与输入变量
X = [theta; thetad1; x; xdot1; phi; phidot1];
U = [T; Tp];
Xdot = [thetad1; thetad2_c; xdot1; xdot2_c; phidot1; phidot2_c];

%通过jacobian函数得到A，B矩阵
A = jacobian(Xdot, X);
B = jacobian(Xdot, U);

%代入平衡点的值，在平衡点处进行线性化
A_ballance = subs(A, [theta, thetad1, xdot1, phi, phidot1, T, Tp], [0, 0, 0, 0, 0, 0, 0])
B_ballance = subs(B, [theta, thetad1, xdot1, phi, phidot1, T, Tp], [0, 0, 0, 0, 0, 0, 0])
C_ballance = eye(6);
D_ballance = zeros(6, 2);

%LQR控制器 A,B矩阵要被赋值
% Q = diag([theta_Q thetad1_Q x_Q xdot1_Q phi_Q phidot1_Q]);
% R = diag([T_R Tp_R]);
% sys = ss(A_ballance, B_ballance, C_ballance, D_ballance);
% K = lqr(sys, Q, R);










% [xdot2_c, thetad2_c, phidot2_c] = subs(solve(f_nl, [xdot2, thetad2, phidot2]), [Nm, N, Pm, P, Nf], [Nm_c, N_c, Pm_c, P_c, Nf_c]);


% f_c = subs(f, [Nm, N, Pm, P], [Nm_c, N_c, Pm_c, P_c])
% 
% 
% xdot2_c = solve(subs(f_c(2), Nf, Nf_c), xdot2);
% phidot2_c = solve(subs(f(8), [Nm, Pm], [Nm_c, Pm_c]), phidot2);
% thetad2_c = solve(subs(f(5), [Nm, Pm], [Nm_c, Pm_c]), thetad2);
% 
% xdot2_f = collect(xdot2_c,[theta, thetad1, thetad2, x, xdot1, xdot2, phi, phidot1, phidot2]) - xdot2;
% phidot2_f = collect(phidot2_c,[theta, thetad1, thetad2, x, xdot1, xdot2, phi, phidot1, phidot2]) - phidot2;
% thetad2_f = collect(thetad2_c,[theta, thetad1, thetad2, x, xdot1, xdot2, phi, phidot1, phidot2]) - thetad2;
% 
% 
% J = jacobian([xdot2_f;phidot2_f;thetad2_f],[theta, thetad1, x, xdot1, phi, phidot1]);
% J = subs(J, [theta, thetad1, xdot1, phi, phidot1], [0, 0, 0, 0, 0]);

% phidot2_p = subs(subs(phidot2_c, xdot2, xdot2_c),[phi, phidot1, xdot1, theta, thetad1], [0, 0, 0, 0, 0])
% thetad2_p = subs(subs(thetad2_c, xdot2, xdot2_c),[phi, phidot1, xdot1, theta, thetad1], [0, 0, 0, 0, 0])



