%因为不可能在单片机中实时计算K矩阵，该文件是用于拟合不同腿长下的K的多项式

% function [xp, yp, Ip] = Centroid(xa, ya, xb, yb, xc, yc, xd, yd, xe, ye, Il1, Il2, Il3, Il4)

syms mx_l1 my_l1 mx_l2 my_l2 mx_l3 my_l3 mx_l4 my_l4...
     xa ya xb yb xc yc xd yd xe ye...
     Il1 Il2 Il3 Il4...
     m_l1 m_l2 m_l3 m_l4

m_l1 = 0.52;
m_l2 = 0.2;
m_l3 = 0.2;
m_l4 = 0.52;


%假设每条杆的质量均匀，即质心在杆中心（实际上并不在！！）
mx_l1 = (1/2)*(xb - xa) + xa;
my_l1 = (1/2)*(yb - ya) + ya;
mx_l2 = (1/2)*(xc - xb) + xb;
my_l2 = (1/2)*(yc - yb) + yb;
mx_l3 = (1/2)*(xd - xc) + xc;
my_l3 = (1/2)*(yd - yc) + yc;
mx_l4 = (1/2)*(xe - xd) + xd;
my_l4 = (1/2)*(ye - yd) + yd;

%等效杆的质心坐标
xp = (mx_l1*m_l1 + mx_l2*m_l2 + mx_l3*m_l3 + mx_l4*m_l4) / (m_l1 + m_l2 + m_l3 + m_l4);
yp = (my_l1*m_l1 + my_l2*m_l2 + my_l3*m_l3 + my_l4*m_l4) / (m_l1 + m_l2 + m_l3 + m_l4);

d1 = sqrt((xp - mx_l1)^2 + (yp - my_l1)^2);
d2 = sqrt((xp - mx_l2)^2 + (yp - my_l2)^2);
d3 = sqrt((xp - mx_l3)^2 + (yp - my_l3)^2);
d4 = sqrt((xp - mx_l4)^2 + (yp - my_l4)^2);

%等效杆的转动惯量
Ip = (Il1 + m_l1*d1^2) + (Il2 + m_l2*d2^2) + (Il3 + m_l3*d3^2) + (Il4 + m_l4*d4^2);

% end