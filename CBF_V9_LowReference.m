function out=CBF_V9_LowReference(t,order,opt)
% CBF_V9_LOWREFERENCE 0～6階が段の接続点で連続する低高度参照。
% t=0..1.2:下降、1.2..2.4:低高度巡航、2.4..3.6:再上昇。
% 13次のBeta(7,7) CDFを使用。端点で1～6階の微分がゼロ。
if nargin<3,opt=struct();end
if order<0 || order>6 || floor(order)~=order,error('CBF:V9RefOrder','0～6階の整数');end
zH=CBF_V9_Util('options',opt,'zHigh',.45);
zL=CBF_V9_Util('options',opt,'zLow',.24);
dx=CBF_V9_Util('options',opt,'travelX',.50);
td=CBF_V9_Util('options',opt,'phaseDuration',1.2);
if td<=0,error('CBF:V9Reference','phaseDuration>0');end
vDown=localSmooth(t,order,td);
vTravel=localSmooth(t-td,order,td);
vUp=localSmooth(t-2*td,order,td);
out=[dx*vTravel;0;(zL-zH)*vDown+(zH-zL)*vUp];
if order==0,out(3)=out(3)+zH;end
end
function val=localSmooth(t,order,dur)
if t<=0,val=0;return;end
if t>=dur,val=double(order==0);return;end
s=t/dur;m=6;val=0;
C=factorial(2*m+1)/factorial(m)^2;
for k=0:m
 p=m+k+1;
 if p>=order
  term=(-1)^k*nchoosek(m,k)*factorial(p)/factorial(p-order)*s^(p-order)/p;
  val=val+term;
 end
end
val=C*val/(dur^order);
end
