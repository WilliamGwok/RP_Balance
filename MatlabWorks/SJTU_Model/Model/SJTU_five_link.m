syms l1_a l1_u l1_d l2_a l2_u l2_d l theta phi1 phi2 Fbl Tbl T1 T2 real


xe = l * sin(theta);
ye = l * cos(theta);
x1 = l1_a - l1_u*cos(phi1);
y1 = l1_u*sin(phi1);
x2 = l2_a - l2_u*cos(phi2);
y2 = l2_u*sin(phi2);

phi1_l = (1/l1_u)*( ((xe + x1)*sin(theta) + (ye - y1)*cos(theta)) / (-(xe +x1)*sin(phi1) + (ye - y1)*cos(phi1)) );
phi1_theta = (l/l1_u)*( ((xe + x1)*cos(theta) - (ye - y1)*sin(theta)) / (-(xe + x1)*sin(phi1) + (ye - y1)*cos(phi1)) );
  
phi2_l = (1/l2_u)*( ((xe - x2)*sin(theta) + (ye - y2)*cos(theta)) / ((xe - x2)*sin(phi2) + (ye - y2)*cos(phi2)) );
phi2_theta = (l/l2_u)*( ((xe - x2)*cos(theta) - (ye - y2)*sin(theta)) / ((xe - x2)*sin(phi2) + (ye - y2)*cos(phi2)) );

J = [phi1_l phi1_theta; phi2_l phi2_theta];
J = inv(J);
gama = [Fbl Tbl]';
tao = simplify(J'*gama);