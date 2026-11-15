function value=CBF_V8_Reference(name,t,order,initialPosition,duration)
% テスト専用解析参照: t=0で位置/速度/加速度/.../6階入力の開始状態を整合。
% S(t) は beta(7,7) CDF の13次多項式で、両端の1～6階微分がゼロ。
% circle/figure8の高階微分は正規化Taylor係数のsin/cos再帰で解析計算。
if order<0 || order>6 || floor(order)~=order
 error('CBF:V8RefDerivative','0～6階の整数を指定してください。');
end
if duration<=0,error('CBF:V8RefDuration','duration>0にしてください。');end
if t<0,error('CBF:V8RefTime','t>=0にしてください。');end
S=zeros(1,7);u=min(t/duration,1);m=6;
if t<duration
 const=factorial(2*m+1)/factorial(m)^2;
 for j=0:6
  v=0;
  for k=0:m
   p=m+k+1;
   if p>=j
     v=v+(-1)^k*nchoosek(m,k)*factorial(p)/factorial(p-j)*u^(p-j)/p;
   end
  end
  S(j+1)=const*v/(duration^j);
 end
else,S(1)=1;
end
if order==0
 value=initialPosition(:);
else
 value=zeros(3,1);
end
switch name
 case 'hover'
 case 'line'
   value=value+[.55;0;0]*S(order+1);
 case 'ascent'
   value=value+[0;0;.55]*S(order+1);
 case {'circle','figure8'}
   th=2*pi*S./factorial(0:6); % normalized Taylor coefficient of theta
   [s1,c1]=sinCosJet(th);
   if strcmp(name,'circle')
      delta=[.4*s1(order+1);.4*(-c1(order+1));0];
      if order==0,delta(2)=delta(2)+.4;end
   else
      [s2,~]=sinCosJet(2*th);
      delta=[.45*s1(order+1);.24*s2(order+1);0];
   end
   value=value+factorial(order)*delta;
 otherwise
   error('CBF:V8ReferenceName','未知の参照軌道: %s',name);
end
end
function [s,c]=sinCosJet(theta)
s=zeros(1,7);c=zeros(1,7);
s(1)=sin(theta(1));c(1)=cos(theta(1));
for n=1:6
   ds=0;dc=0;
   for k=1:n
      ds=ds+k*theta(k+1)*c(n-k+1);
      dc=dc-k*theta(k+1)*s(n-k+1);
   end
   s(n+1)=ds/n;c(n+1)=dc/n;
end
end
