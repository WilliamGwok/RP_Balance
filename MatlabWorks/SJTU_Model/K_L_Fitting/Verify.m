%Hello
%该程序用于检查拟合效果
clear

% filename = 'Will_data.xlsx';
filename = 'old_dude.xlsx';
sheet = 'Sheet1';
[coefficient_data, txt, raw] = xlsread(filename, sheet);

%以元胞数组的形式存储，方便管理与检查
coefficient_room = cell(4, 10);
for i = 1 : 1 : 4
    for j = 1 : 1 : 10
        coefficient_room{i,j} = zeros(1,6);
        coefficient_room{i,j} = coefficient_data(i, (j - 1)*6 + 1 : ((j - 1)*6 + 6));
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

% m_l1 = 0.3616469;
% m_l2 = 0.692384;
% m_l3 = 0.771;
% m_l4 = 0.3616469;
m_l1 = 0.444;
m_l2 = 0.535;
m_l3 = 0.54;
m_l4 = 0.444;

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

stepTime = 3; theta = 17;

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
[K_real, vir_Lr, vir_Ll, A_real, B_real, C_real, D_real] = Func_Cal_K(xc_r, yc_r, xp_r, yp_r, Ip_r, xc_l, yc_l, xp_l, yp_l, Ip_l, A_Cal, B_Cal, C_Cal, D_Cal);

%%求拟合K

% Linear model Poly22:
%      my_fit(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2
%在master中，x取左腿，y取右腿
K_fitting = zeros(4,10);
for i = 1 : 1 : 4
    for j = 1 : 1 : 10
        p00 = coefficient_room{i,j}(1);
        p10 = coefficient_room{i,j}(2);
        p01 = coefficient_room{i,j}(3);
        p20 = coefficient_room{i,j}(4);
        p11 = coefficient_room{i,j}(5);
        p02 = coefficient_room{i,j}(6);
        K_fitting(i,j) = p00 + p10*vir_Ll + p01*vir_Lr + p20*vir_Ll^2 + p11*vir_Ll*vir_Lr + p02*vir_Lr^2;
    end
end

disp('实际K >>')
disp(K_real)
disp('拟合K >>')
disp(K_fitting)
% %拟合效果很棒

disp('实际K >>')
X1 = sprintf('  %f, %f, %f, %f, %f, %f, %f, %f, %f, %f', K_real(1,1), K_real(1,2), K_real(1,3), K_real(1,4), K_real(1,5), K_real(1,6), K_real(1,7), K_real(1,8), K_real(1,9), K_real(1,10));
X2 = sprintf('  %f, %f, %f, %f, %f, %f, %f, %f, %f, %f', K_real(2,1), K_real(2,2), K_real(2,3), K_real(2,4), K_real(2,5), K_real(2,6), K_real(2,7), K_real(2,8), K_real(2,9), K_real(2,10));
X3 = sprintf('  %f, %f, %f, %f, %f, %f, %f, %f, %f, %f', K_real(3,1), K_real(3,2), K_real(3,3), K_real(3,4), K_real(3,5), K_real(3,6), K_real(3,7), K_real(3,8), K_real(3,9), K_real(3,10));
X4 = sprintf('  %f, %f, %f, %f, %f, %f, %f, %f, %f, %f', K_real(4,1), K_real(4,2), K_real(4,3), K_real(4,4), K_real(4,5), K_real(4,6), K_real(4,7), K_real(4,8), K_real(4,9), K_real(4,10));
disp('.wheell_K =')
disp('{')
disp(X1)
disp('},')
disp('.wheelr_K =')
disp('{')
disp(X2)
disp('},')
disp('.sdl_K =')
disp('{')
disp(X3)
disp('},')
disp('.sdr_K  =')
disp('{')
disp(X4)
disp('},')

% disp('拟合K >>')
% disp(K_fitting)




