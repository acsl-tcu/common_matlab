function f = test_CBF_IndependentPlantRHS(x,nu,s)
%TEST_CBF_INDEPENDENTPLANTRHS 既存generateModelの運動方程式を独立記述。
% x:29状態  nu:[T^(4);tau''(3)]
% これは単機・質量なし・張り紐モデル。HLCを使わない。
if numel(x)~=29 || numel(nu)~=4,error('CBF:TestState','入力次元');end
m=s.physical.massDrone;mL=s.physical.massLoad;L=s.physical.cableLength;
g=s.physical.gravity;J=s.physical.inertia(:);
q=x(1:4);w=x(5:7);vL=x(11:13);qc=x(14:16);wc=x(17:19);
T=x(20);tau=x(24:26);
q0=q(1);v=q(2:4);
R=(q0^2-v.'*v)*eye(3)+2*(v*v.')+2*q0*[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
force=T*R(:,3);dqc=cross(wc,qc);
dwc=cross(-qc,force)/(m*L);
ddpl=-g*[0;0;1]+((qc.'*force-m*L*(dqc.'*dqc))/(m+mL))*qc+ ...
   [s.physical.disturbanceXY(:);0];
dq=0.5*[-v.'*w;q0*w+cross(v,w)];
dw=(tau-cross(w,J.*w))./J;
f=[dq;dw;vL;ddpl;dqc;dwc;x(21:23);nu(1);x(27:29);nu(2:4)];
end
