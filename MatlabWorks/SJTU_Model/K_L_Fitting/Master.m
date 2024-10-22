%Hello
clc;
clear;
% format short
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
% Il1 = 0.001;
% Il2 = 0.0012;
% Il3 = 0.0012;
% Il4 = 0.001;
% Il1 = 0.00080701252;
% Il2 = 0.002556459;
% Il3 = 0.00237296755;
% Il4 = 0.00080701252;
Il1 = 0.00061611008;
Il2 = 0.00216959959;
Il3 = 0.002396;
Il4 = 0.00061611008;

%test begin:
% [K, vir_Lr, vir_Ll] = Func_Cal_K(xc_r, yc_r, xp_r, yp_r, Ip_r, xc_l, yc_l, xp_l, yp_l, Ip_l)
% [K, vir_Lr, vir_Ll] = Func_Cal_K(0.075,0.15,0.075,0.07,0.0026,0.075,0.15,0.075,0.07,0.0026);

% [xp, yp, Ip] = Func_Cal_Centroid(xa, ya, xb, yb, xc, yc, xd, yd, xe, ye)
%要分别计算左右杆的质心坐标

%test end

% linear_times = 120;%拟合次数为linear_times^2
% leg_initial_length = -55;%拟合初始角度，决定初始腿长，值为0时腿长大约为0.15m   23/10/16 0.15~0.32

linear_times = 30;%拟合次数为linear_times^2
leg_initial_length = 8;%拟合初始角度，决定初始腿长，值为0时腿长大约为0.15m   23/10/16 0.15~0.32

%创建载体
K_room = cell(4,10);
for i = 1 : 1 : 4
    for j = 1 : 1 : 10
        K_room{i,j} = cell(1, linear_times);
        for k = 1 : 1 : linear_times
            K_room{i, j}{1, k} = rand(1, linear_times);
        end
    end
end
Lr_room = zeros(1, linear_times^2);
Ll_room = zeros(1, linear_times^2);

%一次性得到代入平衡点后的A,B,C,D矩阵
[A_Cal, B_Cal, C_Cal, D_Cal] = Func_Cal_ABCD_Array();

%遍历腿长
for i = 1 : 1 : linear_times
    %左腿参数begin
    phi_l = (leg_initial_length - 1) + i;
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
   
   for j = 1 : 1 : linear_times
       %右腿参数begin 
       phi_r = (leg_initial_length - 1) + j;
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

       %至此会得到一组数据，接下来记录双腿腿长信息与计算对应K矩阵
       [K_unit, vir_Lr, vir_Ll] = Func_Cal_K(xc_r, yc_r, xp_r, yp_r, Ip_r, xc_l, yc_l, xp_l, yp_l, Ip_l, A_Cal, B_Cal, C_Cal, D_Cal);%该函数十分耗时，可以考虑优化

       Ll_room( (i-1)*linear_times + j ) = vir_Ll;
       Lr_room( (i-1)*linear_times + j ) = vir_Lr;

       if(vir_Lr == vir_Ll)
       disp('拟合腿长的长度：');
       disp(vir_Lr);
       end

       for m = 1 : 1 : 4
           for n = 1 : 1 : 10
               K_room{m,n}{1, i}(j) = K_unit(m, n);
           end
       end
   end
end
%至此获得所有K值，现在对40个元素分别拟合多项式，并保存各项系数

polynomial_order = 2;%拟合多项式的阶次
single_K_room = zeros(1,linear_times^2);
coefficient_room = cell(4, 10);
for i = 1 : 1 : 4
    for j = 1 : 1 : 10
        coefficient_room{i,j} = zeros(1,6);%因为拟合阶次定为2，故会输出6个系数，且最终是以列向量的形式输出
    end
end

%若要输出拟合系数，则linear_times最小为3
for i = 1 : 1 : 4
    for j = 1 : 1 : 10
        %对单个元素进行多项式拟合 begin
        for m = 1 : 1 : linear_times
            for n = 1 : 1 : linear_times
                single_K_room( (m - 1)*linear_times + n ) = K_room{i, j}{1, m}(n);%将元胞数组内的数存进矩阵里
            end
        end
        my_fit = fit([Ll_room', Lr_room'], single_K_room', 'poly22');
        coefficient_room{i,j} = coeffvalues(my_fit);
        %对单个元素进行多项式拟合 end
    end
end

filename = 'Will_data.xlsx';
%%
%将拟合系数保存到excel中
writetable(cell2table(coefficient_room), filename, 'Sheet', 1);
disp('数据已存入excel');

