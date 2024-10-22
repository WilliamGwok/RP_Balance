function T=Five_Link_Func(paras)
syms phi0 phi1 phi2 phi3 phi4 ...%l1 l2 l3 l4 l5...
    l0  lbd xa xb xc xd xe ya yb yc yd ye...
    xbdot xcdot xddot ybdot ycdot yddot phi1dot phi2dot phi3dot phi4dot...
    T1 T2 Fx Fy Fl Tp;

%设置各杆长度，关节电机转轴中心之间距离
l1 = 2;
l2 = 2;
l3 = 2;
l4 = 2;
l5 = 2;


%通过点c坐标列等式
f = [xb + l2*cos(phi2)-(xd - l3*cos(pi - phi3))==0;
    yb + l2*sin(phi2)-(yd + l3*sin(pi - phi3))==0];
x = [phi2,phi3];
[sovphi2, sovphi3] = solve(f, x);
sovphi2 = sovphi2(1);%取负
sovphi3 = sovphi3(1);%取正

%替换变量：
% xd = l5 + l4*cos(phi4);
% yd = l4*sin(phi4);
% xb = -l1*cos(pi - phi1);
% yb = l1*sin(pi - phi1);
sovphi2 = simplify(collect(subs(sovphi2, [xd, yd, xb, yb], [l5 + l4*cos(phi4), l4*sin(phi4), -l1*cos(pi - phi1), l1*sin(pi - phi1)])));
sovphi3 = simplify(collect(subs(sovphi3, [xd, yd, xb, yb], [l5 + l4*cos(phi4), l4*sin(phi4), -l1*cos(pi - phi1), l1*sin(pi - phi1)])));

%   sovphi2 = subs(sovphi2, [phi1, phi4], [paras(1), paras(2)]);
%   sovphi3 = subs(sovphi3, [phi1, phi4], [paras(1), paras(2)]);

xc = l1*cos(phi1) + l2*cos(phi2);
yc = l1*sin(phi1) + l2*sin(phi2);

xc = subs(xc, phi2, sovphi2);
yc = subs(yc, phi2, sovphi2);
sovphi0 = atan(yc/(xc-l5/2));
sovl0=((xc-l5/2)^2+(yc)^2)^0.5;

%极坐标(l0,phi0)
%l0 = sqrt((xc - 0.5*l5)^2 + yc^2);
%phi0 = atan((yc)/(xc - 0.5*l5));

xbdot = -l1*phi1dot*sin(phi1);
ybdot = l1*phi1dot*cos(phi1);
xddot = -l4*phi4dot*sin(phi4);
yddot = l4*phi4dot*cos(phi4);

%计算phi2dot 1
fdot = [xbdot - l2*phi2dot*sin(phi2) - xddot + l3*phi3dot*sin(phi3) == 0;
        ybdot + l2*phi2dot*cos(phi2) - yddot - l3*phi3dot*cos(phi3) == 0];
xdot = [phi2dot,phi3dot];
[sovphi2dot, sovphi3dot] = solve(fdot, xdot);
sovphi2dot = simplify(collect(sovphi2dot));
phi2dot = simplify(sovphi2dot);%1

xcdot = -l1*phi1dot*sin(phi1) - l2*sovphi2dot*sin(phi2);%1
ycdot = l1*phi1dot*cos(phi1) + l2*sovphi2dot*cos(phi2);%1

T = [T1;T2];%对应phi1与phi4
Fv = [Fx;Fy];
F = [Fl;Tp];

X = [xcdot; ycdot];%1
Q = [phi1dot;phi4dot];%1
X = simplify(collect(X,Q));%1
%速度映射雅可比矩阵，由关节速度映射到支撑点x,y方向的速度
J = simplify(jacobian(X,Q));%1

%根据虚功定理，J的转置为支撑点虚拟力x方向力Fx和y方向力Fy(主动力的两个自由度)到关节力T1，T2的映射矩阵
%T = J'*Fv;
%对l0的末端作用力Fc(沿杆)，Ft(垂直杆)感兴趣，故需要把映射关系由直角坐标转换到极坐标
%由极坐标转换到直角坐标的转置矩阵R:
R = [cos(pi/2 - phi0) sin(pi/2 - phi0);
     -sin(pi/2 - phi0) cos(pi/2 - phi0)];
%还需要将Ft转换成需要用的Tp，故需要转换矩阵M:
M = [0 -1/l0;%转矩方向为垂直杆向左 %Fc
     1 0];                       %Ft
Fv = R*F;
F = subs(F, [Fl, Tp], [paras(3), paras(4)]);

T = simplify(collect(J.'*R*M*F));%給倒立摆的公式
T = subs(T, [phi2, phi3,phi0, l0], [sovphi2, sovphi3, sovphi0, sovl0]);

T = vpa(subs(T, [phi1, phi4], [paras(1), paras(2)]),5);

end
