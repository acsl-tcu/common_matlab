syms w1 w2 w3 phi theta psi v1 v2 v3 p1 p2 p3 ex ey ez real
w = [w1,w2,w3];
v = [v1,v2,v3];
p = [p1,p2,p3];
q = [phi,theta,psi];
e = [ex,ey,ez];
what= [0,-w3,w2;w3,0,-w1;-w2,w1,0];
Rzyx = [ cos(psi)*cos(theta), ...
          cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), ...
          cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
          
          sin(psi)*cos(theta), ...
          sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), ...
          sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
          
          -sin(theta), ...
          cos(theta)*sin(phi), ...
          cos(theta)*cos(phi) ];
T_inv = [ 1,  sin(phi) * tan(theta),  cos(phi) * tan(theta);
          0,  cos(phi),              -sin(phi);
          0,  sin(phi) / cos(theta),  cos(phi) / cos(theta) ];

qdot = T_inv * w';
Ib = diag([2.24, 2.99, 4.80]) * 1e-3;

f =[v';
    qdot;
    -9.81*e';
    -inv(Ib)*what*Ib*w'];