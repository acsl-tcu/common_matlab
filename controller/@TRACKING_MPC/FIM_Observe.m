function PP = FIM_Observe(obj,x,y,theta,d,alpha,phi)
%FIM_OBSERVE
%    PP = FIM_OBSERVE(X,Y,THETA,D,ALPHA,PHI)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    16-Nov-2021 18:17:45

t2 = cos(alpha);
t3 = sin(alpha);
t6 = -alpha;
t7 = -d;
t4 = t2.*x;
t5 = t3.*y;
t8 = phi+t6+theta;
t9 = cos(t8);
t10 = sin(t8);
t13 = t4+t5+t7;
t11 = 1.0./t9.^2;
t12 = 1.0./t9.^3;
t14 = t2.*t3.*t11;
t15 = t2.*t10.*t12.*t13;
t16 = t3.*t10.*t12.*t13;
PP = reshape([t2.^2.*t11,t14,t15,t14,t3.^2.*t11,t16,t15,t16,t10.^2.*t11.^2.*t13.^2],[3,3]);
