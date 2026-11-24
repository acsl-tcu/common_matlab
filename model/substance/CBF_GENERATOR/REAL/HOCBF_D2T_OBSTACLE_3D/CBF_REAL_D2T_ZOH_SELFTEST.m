function report=CBF_REAL_D2T_ZOH_SELFTEST()
% Standalone MATLAB checks for ID3's ZOH rotating cable-ellipsoid evaluator.
% This does NOT replace a numerical ODE enclosure, 25ms timing test,
% obstacle navigation experiment, or recursive feasibility verification.
P=struct('m',0.85,'mL',0.14,'J',[0.02;0.02;0.03], ...
    'gravity',9.81,'cableL',2.0,'dstx',0,'dsty',0);
x=[1;0;0;0;0;0;0; 0;0;2; 0;0;0; 0;0;-1; 0;0;0];
h=0.025;u=[(P.m+P.mL)*P.gravity;0;0;0];
geom=struct('a',P.cableL/2+0.02,'b',0.20);
part=struct('kind','sphere','center',[4;0;3], ...
    'params',struct('radius',0.4),'R',eye(3), ...
    'obstacleName','sphere_static');
r=CBF_REAL_D2T_ZOH_EVAL(x,u,P,part,geom,h,3);
assert(all(isfinite([r.h0;r.hEnd;r.intervalLower])));
assert(abs(r.h0-(4-0.4-geom.b))<1e-10);
assert(all(r.intervalLower<=min(r.hNodes,[],2)));
assert(norm(r.xEnd(8:13)-x(8:13))<1e-5);
r2=CBF_REAL_D2T_ZOH_EVAL(x,u,P,part,geom,h,6);
assert(norm(r.xEnd(8:10)-r2.xEnd(8:10))<1e-5);
% For ellipsoid obstacles, the directional support is sqrt(n'Qobs n).
part.kind='ellipsoid';part.params=struct('axes',[.4;.8;1.2]);
e=CBF_REAL_D2T_ZOH_EVAL(x,u,P,part,geom,h,3);
assert(abs(e.h0-(4-.4-geom.b))<1e-10);
% The cable axis affects ellipsoid support; no payload sphere is added.
y=x;y(14:16)=[-1;0;0];
eAxis=CBF_REAL_D2T_ZOH_EVAL(y,u,P,part,geom,h,3);
assert(eAxis.h0>0 && eAxis.h0<e.h0);
% Discrete Tddot == second difference of ACTUALLY applied ZOH commands.
Tkm2=9.0;Tkm1=9.5;Tk=10;
assert(abs((Tk-2*Tkm1+Tkm2)/h^2)<1e-8);
assert(abs((10.5-2*Tkm1+Tkm2)/h^2-800)<1e-8);
assert(all([0,20]>=0)); % physical thrust is always explicitly bounded
partsEmpty=struct('kind',{},'center',{},'params',{},'R',{});
re=CBF_REAL_D2T_ZOH_EVAL(x,u,P,partsEmpty,geom,h,3);
assert(isempty(re.h0));
report=struct('passed',true,'dt',h, ...
    'constraintScope','full rotating cable ellipsoid vs static obstacles', ...
    'mode','DISCRETE_SECOND_THRUST_DIFFERENCE__PHYSICAL_ZOH', ...
    'notes','Unit tests only; full intersample/recursive safety not proven.');
disp('[CBF D2T CABLE-ONLY ZOH SELFTEST] PASS');
end
