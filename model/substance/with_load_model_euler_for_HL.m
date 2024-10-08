function dx = with_load_model_euler_for_HL(in1,in2,in3)
%with_load_model_euler_for_HL
%    DX = with_load_model_euler_for_HL(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2023/10/13 17:31:26

cableL = in3(:,21);
dp1 = in1(7,:);
dp2 = in1(8,:);
dp3 = in1(9,:);
dpl1 = in1(16,:);
dpl2 = in1(17,:);
dpl3 = in1(18,:);
gravity = in3(:,9);
jx = in3(:,6);
jy = in3(:,7);
jz = in3(:,8);
m = in3(:,1);
mL = in3(:,20);
o1 = in1(10,:);
o2 = in1(11,:);
o3 = in1(12,:);
ol1 = in1(22,:);
ol2 = in1(23,:);
ol3 = in1(24,:);
pT1 = in1(19,:);
pT2 = in1(20,:);
pT3 = in1(21,:);
pitch = in1(5,:);
roll = in1(4,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
yaw = in1(6,:);
t2 = cos(pitch);
t3 = cos(roll);
t4 = sin(pitch);
t5 = sin(roll);
t6 = m+mL;
t7 = ol1.*pT2;
t8 = ol2.*pT1;
t9 = ol1.*pT3;
t10 = ol3.*pT1;
t11 = ol2.*pT3;
t12 = ol3.*pT2;
t13 = 1.0./cableL;
t14 = -gravity;
t15 = 1.0./jx;
t16 = 1.0./jy;
t17 = 1.0./jz;
t18 = 1.0./m;
t23 = pitch./2.0;
t24 = roll./2.0;
t25 = yaw./2.0;
t19 = 1.0./t2;
t20 = -t8;
t21 = -t10;
t22 = -t12;
t26 = cos(t23);
t27 = cos(t24);
t28 = cos(t25);
t29 = sin(t23);
t30 = sin(t24);
t31 = sin(t25);
t32 = 1.0./t6;
t33 = t7+t20;
t34 = t9+t21;
t35 = t11+t22;
t39 = t26.*t27.*t28;
t40 = t26.*t27.*t31;
t41 = t26.*t28.*t30;
t42 = t27.*t28.*t29;
t43 = t26.*t30.*t31;
t44 = t27.*t29.*t31;
t45 = t28.*t29.*t30;
t46 = t29.*t30.*t31;
t36 = t33.^2;
t37 = t34.^2;
t38 = t35.^2;
t47 = -t44;
t48 = -t45;
t52 = t39+t46;
t53 = t42+t43;
t49 = t36+t37+t38;
t54 = t40+t48;
t55 = t41+t47;
t56 = t52.^2;
t57 = t53.^2;
t62 = t52.*t53.*2.0;
t50 = cableL.*m.*t49;
t58 = t54.^2;
t59 = t55.^2;
t60 = -t57;
t63 = t52.*t55.*2.0;
t64 = t53.*t54.*2.0;
t66 = t54.*t55.*2.0;
t61 = -t59;
t65 = -t64;
t67 = t62+t66;
t68 = t63+t65;
t69 = pT1.*t67.*u1;
t70 = pT2.*t67.*u1;
t71 = pT3.*t67.*u1;
t77 = t56+t58+t60+t61;
t72 = pT1.*t68.*u1;
t73 = pT2.*t68.*u1;
t74 = pT3.*t68.*u1;
t78 = pT1.*t77.*u1;
t79 = pT2.*t77.*u1;
t80 = pT3.*t77.*u1;
t81 = t70+t72;
t82 = t74+t79;
t85 = -pT1.*t32.*(t50-t69+t73-t80);
t86 = -pT2.*t32.*(t50-t69+t73-t80);
t87 = -pT3.*t32.*(t50-t69+t73-t80);
mt1 = [dp1;dp2;dp3;t19.*(o1.*t2+o3.*t3.*t4+o2.*t4.*t5);o2.*t3-o3.*t5;t19.*(o3.*t3+o2.*t5);t85-cableL.*(ol2.*t33+ol3.*t34-pT2.*t13.*t18.*t81-pT3.*t13.*t18.*(t71-t78));t86-cableL.*(-ol1.*t33+ol3.*t35+pT1.*t13.*t18.*t81+pT3.*t13.*t18.*t82);t14+t87+cableL.*(ol1.*t34+ol2.*t35+pT2.*t13.*t18.*t82-pT1.*t13.*t18.*(t71-t78));t15.*u2+t15.*(jy.*o2.*o3-jz.*o2.*o3);t16.*u3-t16.*(jx.*o1.*o3-jz.*o1.*o3);t17.*u4+t17.*(jx.*o1.*o2-jy.*o1.*o2);dpl1;dpl2;dpl3;t85;t86;t14+t87;t35;-t9+t10;t33;-t13.*t18.*t82];
mt2 = [-t13.*t18.*(t71-t78);t13.*t18.*t81];
dx = [mt1;mt2];
end
