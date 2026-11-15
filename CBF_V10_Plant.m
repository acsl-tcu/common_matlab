function out=CBF_V10_Plant(action,varargin)
%CBF_V10_PLANT 同じ19状態力学へのHLC実推力・トルク接続。入力clipなし。
switch lower(char(action))
 case 'state29'
  x19=varargin{1};u=varargin{2};
  if numel(x19)~=19 || numel(u)~=4,error('CBF:V10PlantSize','19状態/4入力');end
  out=[x19(:);u(1);zeros(3,1);u(2:4);zeros(3,1)];
 case 'rhs'
  x19=varargin{1};u=varargin{2};s=varargin{3};
  xx=CBF_V10_Plant('state29',x19,u);
  allDeriv=test_CBF_IndependentPlantRHS(xx,zeros(4,1),s);
  out=allDeriv(1:19);
 case 'rk4'
  x=varargin{1};u=varargin{2};dt=varargin{3};s=varargin{4};
  a=CBF_V10_Plant('rhs',x,u,s);
  b=CBF_V10_Plant('rhs',x+dt*a/2,u,s);
  c=CBF_V10_Plant('rhs',x+dt*b/2,u,s);
  d=CBF_V10_Plant('rhs',x+dt*c,u,s);
  out=x+dt*(a+2*b+2*c+d)/6;
 otherwise,error('CBF:V10PlantAction','不明な動作');
end
end
