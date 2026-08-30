function [A_att,b_att,h_att,A_cable,b_cable,h_cable] = CBF_Constraints_Attitude_Cable_Taylor(obj,in2,in3,in4,in5,in6,rl,in8)
cableL = in8(:,7);
gamma_cb1 = in6(3,:);
gamma_cb2 = in6(4,:);
gamma_att1 = in6(1,:);
gamma_att2 = in6(2,:);
jx = in8(:,2);
jy = in8(:,3);
jz = in8(:,4);
m = in8(:,1);
o1 = in2(5,:);
o2 = in2(6,:);
o3 = in2(7,:);
ol1 = in2(17,:);
ol2 = in2(18,:);
ol3 = in2(19,:);
pT1 = in2(14,:);
pT2 = in2(15,:);
pT3 = in2(16,:);
q0 = in2(1,:);
q1 = in2(2,:);
q2 = in2(3,:);
q3 = in2(4,:);
theta_att_max = in5(1,:);
theta_cable_max = in5(2,:);
t2 = cos(theta_att_max);
t3 = cos(theta_cable_max);
t4 = ol1.*pT2;
t5 = ol2.*pT1;
t6 = q0.^2;
t7 = q1.^2;
t8 = q2.^2;
t9 = q3.^2;
t10 = q0.*q1.*2.0;
t11 = q0.*q2.*2.0;
t12 = q1.*q3.*2.0;
t13 = q2.*q3.*2.0;
t14 = 1.0./cableL;
t15 = -jz;
t16 = 1.0./jx;
t17 = 1.0./jy;
t18 = 1.0./m;
t19 = -t5;
t20 = -t12;
t21 = -t7;
t22 = -t8;
t23 = t10+t13;
t24 = t11+t20;
A_att = [0.0,t16.*t23,t17.*t24,0.0];
if nargout > 1
b_att = (gamma_att1.*q0.*-2.0+o1.*q1.*2.0+o2.*q2.*2.0).*((o1.*q1)./2.0+(o2.*q2)./2.0+(o3.*q3)./2.0)-(gamma_att1.*q1.*2.0+o1.*q0.*2.0-o2.*q3.*2.0).*((o1.*q0)./2.0-(o2.*q3)./2.0+(o3.*q2)./2.0)-(gamma_att1.*q2.*2.0+o2.*q0.*2.0+o1.*q3.*2.0).*((o2.*q0)./2.0+(o1.*q3)./2.0-(o3.*q1)./2.0)+(gamma_att1.*q3.*2.0-o1.*q2.*2.0+o2.*q1.*2.0).*(o1.*q2.*(-1.0./2.0)+(o2.*q1)./2.0+(o3.*q0)./2.0)-gamma_att2.*(gamma_att1.*t2-gamma_att1.*t6+gamma_att1.*t7+gamma_att1.*t8-gamma_att1.*t9+o1.*t10+o2.*t11+o1.*t13-o2.*q1.*q3.*2.0)+o1.*o3.*t17.*t24.*(jx+t15)-o2.*o3.*t16.*t23.*(jy+t15);
end
if nargout > 2
t25 = t6+t9+t21+t22;
h_att = -t2+t25;
end
if nargout > 3
A_cable = [pT1.*t14.*t18.*(pT3.*(t11+t12)-pT1.*t25)-pT2.*t14.*t18.*(pT2.*t25+pT3.*(t10-t13)),0.0,0.0,0.0];
end
if nargout > 4
b_cable = -gamma_cb1.*(t4+t19)-gamma_cb2.*(t4+t19+gamma_cb1.*(pT3+t3))+ol1.*(ol1.*pT3-ol3.*pT1)+ol2.*(ol2.*pT3-ol3.*pT2);
end
if nargout > 5
h_cable = -pT3-t3;
end
end
