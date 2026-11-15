function xyz=CBF_V10_Reference(route,t,order,options)
%CBF_V10_REFERENCE 実HLC・実プラントの試験用0～6階目標微分。
% 13次S字(両端の1～6階微分ゼロ)で区間を接続する。
if nargin<4,options=struct();end
if order<0 || order>6 || order~=floor(order),error('CBF:V10RefOrder','0～6階');end
zHi=localOpt(options,'zHigh',0.45);
zLo=localOpt(options,'zLow',0.24);
D=localOpt(options,'phaseDuration',1.5);
if D<=0,error('CBF:V10RefTime','phaseDuration は正');end
xyz=zeros(3,1);
if order==0,xyz(3)=zHi;end
switch lower(char(route))
 case 'hover'
 case 'line'
  xyz(1)=0.3*localProfile(t,0,D,order);
 case 'lowcycle'
  xyz(3)=xyz(3)+(zLo-zHi)*localProfile(t,0,D,order) + ...
                       (zHi-zLo)*localProfile(t,2*D,D,order);
  xyz(1)=0.30*localProfile(t,D,D,order);
 case 'recover'
  % 初期下降なし。上昇してから横へ復帰する。
  xyz(3)=xyz(3)+0.20*localProfile(t,0,D,order);
  xyz(1)=0.25*localProfile(t,D,D,order);
 otherwise
  error('CBF:V10RefRoute','不明な試験軌道: %s',route);
end
end
function v=localProfile(t,t0,D,k)
if t<=t0
 v=0;return;
elseif t>=t0+D
 v=double(k==0);return;
end
u=(t-t0)/D;v=0;
% beta(7,7) の逆数=12012
for j=0:6
 n=j+7;
 fac=1;
 for a=0:k-1,fac=fac*(n-a);end
 v=v+(-1)^j*nchoosek(6,j)*fac*u^(n-k)/n;
end
v=12012*v/D^k;
end
function v=localOpt(s,key,def)
if isfield(s,key) && ~isempty(s.(key)),v=s.(key);else,v=def;end
end
