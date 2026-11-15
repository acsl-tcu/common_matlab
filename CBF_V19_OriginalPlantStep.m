function y=CBF_V19_OriginalPlantStep(x,u,s,h,nsub)
% RK4 of the UNMODIFIED R193 generated 24-state Euler plant. R2 augments
% it with 4 thrust states. Convert back to the 19/23 CBF coordinates.
% Intended for independent parity comparison, NOT assumed identical.
if exist('with_load_model_euler_for_HL','file')~=2
 error('CBF:V19OriginalPlantMissing',...
 'Add original project model/substance folder with with_load_model_euler_for_HL.m');
end
x=x(:);u=u(:);nR2=numel(x)==23;
assert(numel(x)==19 || nR2);
L=s.physical.cableLength;quat=x(1:4);w=quat(1);a=quat(2);b=quat(3);c=quat(4);
roll=atan2(2*(w*a+b*c),1-2*(a*a+b*b));
pitch=asin(max(-1,min(1,2*(w*b-c*a))));
yaw=atan2(2*(w*c+a*b),1-2*(b*b+c*c));
qc=x(14:16);qdot=cross(x(17:19),qc);
pD=x(8:10)-L*qc;vD=x(11:13)-L*qdot;
z=[pD;roll;pitch;yaw;vD;x(5:7);x(8:10);x(11:13);qc;x(17:19)];
if nR2,z=[z;x(20:23)];end
P=zeros(1,21);P(1)=s.physical.massDrone;P(6:8)=s.physical.inertia(:).';
P(9)=s.physical.gravity;P(20)=s.physical.massLoad;P(21)=L;
dt=h/nsub;
for k=1:nsub
 a=rhs(z);b1=rhs(z+dt*a/2);c1=rhs(z+dt*b1/2);d=rhs(z+dt*c1);
 z=z+dt*(a+2*b1+2*c1+d)/6;
end
rr=z(4);pp=z(5);yy=z(6);cr=cos(rr/2);sr=sin(rr/2);
cp=cos(pp/2);sp=sin(pp/2);cy=cos(yy/2);sy=sin(yy/2);
q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;...
 cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
y=[q;z(10:12);z(13:24)];
if nR2,y=[y;z(25:28)];end
 function f=rhs(v)
  if nR2
    T=v(25);ff=with_load_model_euler_for_HL(v(1:24),[T;u(2:4)],P);
    f=[ff(:);v(26:28);u(1)];
  else
    f=with_load_model_euler_for_HL(v(1:24),u,P);f=f(:);
  end
 end
end
