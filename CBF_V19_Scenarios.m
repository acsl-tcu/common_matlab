function sc=CBF_V19_Scenarios(name,paramSet)
% Three C6-smooth load reference trajectories, four obstacle layouts.
% Only ONE cable bounding ellipsoid is protected; 1-2 rotated ellipsoids.
% Uses physical model from V18; R193 uses true flight plant load=0.14, L=2.
if nargin<2,paramSet='R193';end
s=CBF_V18_TestSettings();
switch upper(char(paramSet))
 case 'R193'
   s.physical.massDrone=.762;
   s.physical.massLoad=.14; % R193 run_flight_headless: plant_loadmass=0.14
   s.physical.cableLength=2.0; % R193 run_flight_headless sets cableL=2.0
   sc.estimatedLoadMass=.01; % R193 resets controller loadmass=0.01
 case 'V18'
   sc.estimatedLoadMass=s.physical.massLoad;
 otherwise,error('CBF:V19Parameters','Use R193 or V18.');
end
sc.settings=s;
sc.x0=zeros(19,1);sc.x0(1)=1;
sc.x0(8:10)=[0;0;3.0];sc.x0(14:16)=[0;0;-1];
% Scenario name TRAJECTORY_LAYOUT
parts=strsplit(lower(char(name)),'_');
assert(numel(parts)==2,'CBF:V19Scenario','name must be straight_single, curve_multi, etc');
sc.trajectory=parts{1};sc.layout=parts{2};
switch sc.trajectory
 case 'straight'
   wp=[0 0 3.0;3.1 0 3.0].';times=[0,7];
 case 'curve'
   wp=[0 0 3.0;1.1 .65 3.0;2.1 -.50 3.0;3.1 0 3.0].';times=[0,3,6,9];
 case 'climb'
   wp=[0 0 3.0;1.4 0 3.6;3.1 0 3.0].';times=[0,4.5,9];
 otherwise,error('CBF:V19Trajectory','Unknown trajectory %s',sc.trajectory);
end
sc.waypoints=wp;sc.times=times;sc.duration=times(end);
sc.ref=@(t,k)refLocal(wp,times,t,k);
% Obstacles anchored at cable-center height, not load-center height.
z=3+s.physical.cableLength/2;
switch sc.layout
 case 'single'
  obs={makeObs([1.30;0;z],[.27;.34;.50],pi/8,0)};
 case 'rotated'
  obs={makeObs([1.65;.15;z+.1],[.50;.20;.70],pi/4,pi/5)};
 case 'large'
  obs={makeObs([1.55;0;z],[.53;.48;.78],pi/7,pi/9)};
 case 'multi'
  obs={makeObs([1.12;.15;z],[.30;.29;.65],pi/4,pi/9),...
       makeObs([2.28;-.15;z+.15],[.39;.24;.45],-pi/5,-pi/7)};
 otherwise,error('CBF:V19ObstacleLayout','Unknown layout %s',sc.layout);
end
sc.obstacles=obs;
for i=1:numel(obs)
 assert(strcmp(obs{i}.type,'ellipsoid') && all(obs{i}.axes>0));
end
end
function o=makeObs(c,axes,yaw,pitch)
Rz=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];
Ry=[cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)];
o=struct('type','ellipsoid','center',c(:),'axes',axes(:),'R',Rz*Ry);
end
function xyz=refLocal(wp,times,t,der)
assert(der>=0&&der<=6&&der==floor(der));
if t<=times(1)
 if der==0,xyz=wp(:,1);else,xyz=zeros(3,1);end
 return;
end
if t>=times(end)
 if der==0,xyz=wp(:,end);else,xyz=zeros(3,1);end
 return;
end
seg=find(times<=t,1,'last');seg=min(seg,numel(times)-1);
duration=times(seg+1)-times(seg);u=(t-times(seg))/duration;
% Exact degree-13 smoothstep S_6(u) = 12012*int_0^u v^6(1-v)^6 dv.
% Derivatives 1..6 vanish at the segment boundaries.
factor=0;
for k=0:6
  power=7+k;
  coef=12012*(-1)^k*nchoosek(6,k)/power;
  coeffDer=prod((power-der+1):power);
  if der==0,coeffDer=1;end
  factor=factor+coef*coeffDer*u^(power-der);
end
factor=factor/duration^der;
if der==0,xyz=wp(:,seg)+(wp(:,seg+1)-wp(:,seg))*factor;
else,xyz=(wp(:,seg+1)-wp(:,seg))*factor;end
end
