function out1 = Jacobian_load(in1,in2)
%Jacobian_load
%    OUT1 = Jacobian_load(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    2024/05/13 12:36:51

p1 = in2(:,1);
p6 = in2(:,6);
p7 = in2(:,7);
p8 = in2(:,8);
p20 = in2(:,20);
p21 = in2(:,21);
x4 = in1(4,:);
x5 = in1(5,:);
x10 = in1(10,:);
x11 = in1(11,:);
x12 = in1(12,:);
x19 = in1(19,:);
x20 = in1(20,:);
x21 = in1(21,:);
x22 = in1(22,:);
x23 = in1(23,:);
x24 = in1(24,:);
t2 = cos(x4);
t3 = cos(x5);
t4 = sin(x4);
t5 = sin(x5);
t6 = p1+p20;
t7 = p6.*x10;
t8 = p7.*x11;
t9 = p8.*x12;
t10 = x19.*x22;
t11 = x19.*x23;
t12 = x20.*x22;
t13 = x19.*x24;
t14 = x20.*x23;
t15 = x21.*x22;
t16 = x20.*x24;
t17 = x21.*x23;
t18 = x21.*x24;
t19 = x22.^2;
t20 = x23.^2;
t21 = x24.^2;
t23 = p21.*x22.*x23;
t24 = p21.*x22.*x24;
t25 = p21.*x23.*x24;
t27 = 1.0./p6;
t28 = 1.0./p7;
t29 = 1.0./p8;
t22 = t2.*x12;
t26 = t4.*x11;
t30 = 1.0./t3;
t33 = -t12;
t34 = -t15;
t35 = -t17;
t36 = 1.0./t6;
t37 = -t23;
t38 = -t24;
t39 = -t25;
t31 = t30.^2;
t40 = t11+t33;
t41 = t13+t34;
t42 = t16+t35;
t43 = t40.^2;
t44 = t41.^2;
t45 = t42.^2;
t46 = t40.*x19.*2.0;
t47 = t40.*x20.*2.0;
t48 = t41.*x19.*2.0;
t49 = t40.*x22.*2.0;
t50 = t41.*x21.*2.0;
t51 = t40.*x23.*2.0;
t52 = t42.*x20.*2.0;
t53 = t41.*x22.*2.0;
t54 = t42.*x21.*2.0;
t55 = t41.*x24.*2.0;
t56 = t42.*x23.*2.0;
t57 = t42.*x24.*2.0;
t58 = -t54;
t59 = -t57;
t60 = t47+t50;
t61 = t48+t52;
t62 = t51+t55;
t63 = t53+t56;
t66 = t43+t44+t45;
t64 = t46+t58;
t65 = t49+t59;
t67 = p1.*p21.*t36.*t60.*x19;
t68 = p1.*p21.*t36.*t60.*x20;
t69 = p1.*p21.*t36.*t61.*x19;
t70 = p1.*p21.*t36.*t60.*x21;
t71 = p1.*p21.*t36.*t61.*x20;
t72 = p1.*p21.*t36.*t61.*x21;
t73 = p1.*p21.*t36.*t62.*x19;
t74 = p1.*p21.*t36.*t62.*x20;
t75 = p1.*p21.*t36.*t63.*x19;
t76 = p1.*p21.*t36.*t62.*x21;
t77 = p1.*p21.*t36.*t63.*x20;
t78 = p1.*p21.*t36.*t63.*x21;
t94 = p1.*p21.*t36.*t66;
t79 = p1.*p21.*t36.*t64.*x19;
t80 = p1.*p21.*t36.*t64.*x20;
t81 = p1.*p21.*t36.*t64.*x21;
t82 = p1.*p21.*t36.*t65.*x19;
t83 = p1.*p21.*t36.*t65.*x20;
t84 = p1.*p21.*t36.*t65.*x21;
t85 = -t69;
t86 = -t71;
t87 = -t72;
t88 = -t73;
t89 = -t74;
t90 = -t76;
t95 = -t94;
t91 = -t79;
t92 = -t80;
t93 = -t81;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t30.*(t2.*t5.*x11-t4.*t5.*x12),-t22-t26,t30.*(t2.*x11-t4.*x12),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t30.*(t3.*t22+t3.*t26-t5.*x10)+t5.*t31.*(t5.*t22+t5.*t26+t3.*x10),0.0,t5.*t31.*(t22+t26),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt2 = [0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t28.*(t9-p6.*x12),-t29.*(t8-p6.*x11),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t4.*t5.*t30,t2,t4.*t30,0.0,0.0,0.0,-t27.*(t9-p7.*x12),0.0,t29.*(t7-p7.*x10),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t2.*t5.*t30,-t4,t2.*t30,0.0,0.0,0.0,t27.*(t8-p8.*x11)];
mt3 = [-t28.*(t7-p8.*x10),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t88+t95+p21.*(t20+t21),t37+t89,t38+t90,0.0,0.0,0.0,0.0,0.0,0.0,t88+t95,t89,t90,0.0];
mt4 = [x24,-x23,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t37+t82,t83+t95+p21.*(t19+t21),t39+t84,0.0,0.0,0.0,0.0,0.0,0.0,t82,t83+t95,t84,-x24,0.0,x22,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t38+t75,t39+t77,t78+t95+p21.*(t19+t20),0.0,0.0,0.0,0.0,0.0,0.0,t75,t77,t78+t95,x23,-x22,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t67-p21.*(t14+t18),t68-p21.*(t11-t12.*2.0),t70-p21.*(t13-t15.*2.0),0.0,0.0,0.0,0.0,0.0,0.0,t67,t68,t70,0.0,-x21,x20,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t91+p21.*(t11+t40),t92-p21.*(t10+t18),t93-p21.*(t16-t17.*2.0),0.0,0.0,0.0,0.0,0.0,0.0,t91,t92,t93,x21,0.0,-x19,0.0,0.0,0.0,0.0,0.0,0.0];
mt5 = [0.0,0.0,0.0,t85+p21.*(t13+t41),t86+p21.*(t16+t42),t87-p21.*(t10+t14),0.0,0.0,0.0,0.0,0.0,0.0,t85,t86,t87,-x20,x19,0.0,0.0,0.0,0.0];
out1 = reshape([mt1,mt2,mt3,mt4,mt5],24,24);
end
