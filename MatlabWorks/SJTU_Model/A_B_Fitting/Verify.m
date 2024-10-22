%Hello
%该程序用于检查拟合效果
clear

A_filename = 'A_Data.xlsx';
sheet = 'Sheet1';
[A_coefficient_data, txt, raw] = xlsread(A_filename, sheet);

%以元胞数组的形式存储，方便管理与检查
A_coefficient_room = cell(10, 10);
for i = 1 : 1 : 10
    for j = 1 : 1 : 10
        A_coefficient_room{i,j} = zeros(1,6);
        A_coefficient_room{i,j} = A_coefficient_data(i, (j - 1)*6 + 1 : ((j - 1)*6 + 6));
    end
end

B_filename = 'B_Data.xlsx';
sheet = 'Sheet1';
[B_coefficient_data, txt, raw] = xlsread(B_filename, sheet);

B_coefficient_room = cell(10, 4);
for i = 1 : 1 : 10
    for j = 1 : 1 : 4
        B_coefficient_room{i,j} = zeros(1,6);
        B_coefficient_room{i,j} = B_coefficient_data(i, (j - 1)*6 + 1 : ((j - 1)*6 + 6));
    end
end

global l1 l2 l3 l4 l5;%单位为m
l1 = 0.15;
l2 = 0.27;
l3 = 0.27;
l4 = 0.15;
l5 = 0.15;

global m_l1 m_l2 m_l3 m_l4;%单位为kg，假设两条腿的各杆质量均相同
% m_l1 = 0.52;
% m_l2 = 0.2;
% m_l3 = 0.2;
% m_l4 = 0.52;

% m_l1 = 0.55935;
% m_l2 = 0.6738;
% m_l3 = 0.6388;
% m_l4 = 0.55935;

m_l1 = 0.3616469;
m_l2 = 0.692384;
m_l3 = 0.771;
m_l4 = 0.3616469;

global Il1 Il2 Il3 Il4;%假设两条腿的各杆转动惯量均相同
%仿真数据
% Il1 = 0.001;
% Il2 = 0.0012;
% Il3 = 0.0012;
% Il4 = 0.001;
%实车数据
% Il1 = 0.00080701252;
% Il2 = 0.002556459;
% Il3 = 0.00237296755;
% Il4 = 0.00080701252;

Il1 = 0.00061611008;
Il2 = 0.00216959959;
Il3 = 0.002396;
Il4 = 0.00061611008;


%通过角度选定左右腿长
phi_l = 10;
phi_r = 10;

%左腿参数begin
syms phi1_l phi4_l
syms xa_l ya_l xb_l yb_l xd_l yd_l xc_l yc_l xe_l ye_l
syms xp_l yp_l Ip_l
%通过角度变化控制腿长变化
phi1_l = deg2rad(-phi_l) + pi;
phi4_l = deg2rad(phi_l);
%得到坐标值，以a为原点
[xa_l, ya_l, xb_l, yb_l, xc_l, yc_l, xd_l, yd_l, xe_l, ye_l] = Func_Cal_Coordinate(phi1_l, phi4_l);
[xp_l, yp_l, Ip_l] = Func_Cal_Centroid(xa_l, ya_l, xb_l, yb_l, xc_l, yc_l, xd_l, yd_l, xe_l, ye_l);
%左腿参数end

%右腿参数begin 
syms phi1_r phi4_r
syms xa_r ya_r xb_r yb_r xd_r yd_r xc_r yc_r xe_r ye_r
syms xp_r yp_r Ip_r
%通过角度变化控制腿长变化
phi1_r = deg2rad(-phi_r) + pi;
phi4_r = deg2rad(phi_r);
%得到坐标值，以a为原点
[xa_r, ya_r, xb_r, yb_r, xc_r, yc_r, xd_r, yd_r, xe_r, ye_r] = Func_Cal_Coordinate(phi1_r, phi4_r);
[xp_r, yp_r, Ip_r] = Func_Cal_Centroid(xa_r, ya_r, xb_r, yb_r, xc_r, yc_r, xd_r, yd_r, xe_r, ye_r);
%右腿参数end


%%求实K,并得出当前左右腿长
[A_Cal, B_Cal, C_Cal, D_Cal] = Func_Cal_ABCD_Array();
[A_Real, B_Real, vir_Lr, vir_Ll] = Func_Cal_A_B(xc_r, yc_r, xp_r, yp_r, Ip_r, xc_l, yc_l, xp_l, yp_l, Ip_l, A_Cal, B_Cal);

%%求拟合K

% Linear model Poly22:
%      my_fit(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2
%在master中，x取左腿，y取右腿
A_fitting = zeros(10,10);
for i = 1 : 1 : 10
    for j = 1 : 1 : 10
        p00 = A_coefficient_room{i,j}(1);
        p10 = A_coefficient_room{i,j}(2);
        p01 = A_coefficient_room{i,j}(3);
        p20 = A_coefficient_room{i,j}(4);
        p11 = A_coefficient_room{i,j}(5);
        p02 = A_coefficient_room{i,j}(6);
        A_fitting(i,j) = p00 + p10*vir_Ll + p01*vir_Lr + p20*vir_Ll^2 + p11*vir_Ll*vir_Lr + p02*vir_Lr^2;
    end
end

disp('实际A >>')
disp(A_Real)
disp('拟合A >>')
disp(A_fitting)


B_fitting = zeros(10,4);
for i = 1 : 1 : 10
    for j = 1 : 1 : 4
        p00 = B_coefficient_room{i,j}(1);
        p10 = B_coefficient_room{i,j}(2);
        p01 = B_coefficient_room{i,j}(3);
        p20 = B_coefficient_room{i,j}(4);
        p11 = B_coefficient_room{i,j}(5);
        p02 = B_coefficient_room{i,j}(6);
        B_fitting(i,j) = p00 + p10*vir_Ll + p01*vir_Lr + p20*vir_Ll^2 + p11*vir_Ll*vir_Lr + p02*vir_Lr^2;
    end
end

disp('实际B >>')
disp(B_Real)
disp('拟合B >>')
disp(B_fitting)


%拟合效果很棒

