function V1 = Vf(in1,in2,in3,in4)
%Vf
%    V1 = Vf(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/06/12 13:41:19

Xd3 = in2(:,3);
dXd3 = in2(:,7);
dp3 = in1(10,:);
f11 = in4(:,1);
f12 = in4(:,2);
p3 = in1(7,:);
t2 = f11.*f12;
t3 = f12.^2;
t4 = -dp3;
t5 = -p3;
t6 = -t3;
t7 = Xd3+t5;
t8 = dXd3+t4;
t9 = f11+t6;
t10 = f12.*t9;
t11 = t2+t10;
V1 = [f11.*t7+f12.*t8,-t2.*t7+t8.*t9,-t8.*t11-f11.*t7.*t9,-t8.*(f11.*t9-f12.*t11)+f11.*t7.*t11];
end
