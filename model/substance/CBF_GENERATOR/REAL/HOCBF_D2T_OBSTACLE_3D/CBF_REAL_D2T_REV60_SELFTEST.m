function report=CBF_REAL_D2T_REV60_SELFTEST()
% Synthetic algebra / installation checks, not flight or safety certification.
root=fileparts(mfilename('fullpath'));addpath(root,fullfile(root,'GENERATED'),fullfile(fileparts(fileparts(root)),'COMMON'),'-begin');
assert(strcmp(which('CBF_REAL_D2T_ALL_EVAL'),fullfile(root,'GENERATED','CBF_REAL_D2T_ALL_EVAL.m')),'CBF60:Path');
P=struct('m',.762,'J',[.02237568;.02985236;.0480374],'gravity',9.81,'mL',.14,'cableL',2,'dstx',0,'dsty',0);
G=CBF_COMMON_GEOMETRY(P,struct());G.collisionMode='cable_ellipsoid';G.cableAxisShort=.2;G.cableAxisLong=1.02;G.cableCenterFromLoad=.5;
x=[1;0;0;0; .01;-.02;.03; 0;0;1; .1;.2;.02; 0;0;-1; .03;.02;0];y=[x;8.849;0.2];
part=struct('kind','ellipsoid','center',[2;.3;1.7],'params',struct('axes',[.4;.6;.7]),'R',eye(3));
o=struct('mode','CENTER','droneSphereEnabled',false,'loadSphereEnabled',false,'extras',false,'bodyEnabled',true,'cableEnabled',true,'groundEnabled',true,'tensionEnabled',true,'obsLambda',[2;4;8;16],'bodyLambda',[2;2],'cableLambda',[1;2;8;16],'groundLambda',[2;4;8;16],'tensionLambda',[2;2],'bodyDeg',40,'cableDeg',60,'groundZ',0,'tensionMin',0);
w=[11;.07;-.03];tz=.02;z=CBF_REAL_D2T_ALL_EVAL(y,w,tz,P,G,part,o);old=CBF_REAL_D2T_OBSTACLE_EVAL(y,w,tz,P,G,part,o.obsLambda);
err=max(abs(z.stages{1}-old.stages(:,1))./(1+abs(old.stages(:,1))));assert(err<1e-9,'CBF60:CenterParity','CENTER mismatch');
o.droneSphereEnabled=true;o.loadSphereEnabled=true;o.extras=true;
maxerr=0;maxlower=0;count=0;
for mode={'CENTER','SHAPE','PW','SELECTIVE','FIXED_SHAPE'}
 o.mode=mode{1};a=CBF_REAL_D2T_ALL_EVAL(y,zeros(3,1),tz,P,G,part,o);b=CBF_REAL_D2T_ALL_EVAL(y,w,tz,P,G,part,o,a.plan);
 assert(numel(a.h)==8&&isequal(cellfun(@numel,a.stages),[5 5 5 3 5 5 5 3]),'CBF60:Degrees');
 ee=max(abs(b.last-a.b-a.A*w)./(1+abs(b.last)));ll=max(abs(b.lower-a.lower)./(1+abs(b.lower)));maxerr=max(maxerr,ee);maxlower=max(maxlower,ll);count=count+1;
end
assert(maxerr<1e-8&&maxlower<1e-8,'CBF60:Affine');
o.tensionEnabled=false;o.groundEnabled=false;o.bodyEnabled=false;o.cableEnabled=false;o.droneSphereEnabled=false;o.loadSphereEnabled=false;
a=CBF_REAL_D2T_ALL_EVAL(y,zeros(3,1),tz,P,G,part([]),o);assert(isempty(a.h)&&isempty(a.lower)&&size(a.A,1)==0,'CBF60:Flags');
cycles=0;for H=[.02 .025 .05]
 for B={[0;20],[2;11],[0;10]}
  bb=B{1};for sign0=[-1 1]
   T=mean(bb);q=0;
   for k=1:50
    K=CBF_REAL_D2T_INPUT_KERNEL(T,q,H,bb);assert(K.ok,'CBF60:Kernel');
    if sign0<0,nu=K.nu(1);else,nu=K.nu(2);end
    assert(CBF_REAL_D2T_INPUT_KERNEL(T,q,H,bb,nu),'CBF60:Candidate');
    T=T+H*q+.5*H^2*nu;q=q+H*nu;cycles=cycles+1;
   end
  end
 end
end
% Smoke-test real controller integration and fallback without touching GUI.
projectDir=fileparts(fileparts(fileparts(fileparts(fileparts(root)))));
controllerDir=fullfile(projectDir,'controller','HLC_CBF_CONTROLLER','CBF_REAL');
addpath(controllerDir,fullfile(controllerDir,'SCHEMES'),'-begin');
cfg=CBF_REAL_INIT(3,struct('dt',.025));
cfg.paperThrustState=struct('initialized',true,'T',y(20),'q',y(21));
agent=struct('sensor',struct('result',struct('cbfObservedWorld',struct('parts',part([])))));
pp=[P.m;P.J(:);P.gravity;P.mL;P.cableL;0;0];
[u,info]=CBF_REAL_SCHEME_03_PAPER_EXTENSION(agent,cfg,x,[8.849;0;0;0],pp,.025,[], 'f');
assert(info.success&&numel(u)==4&&info.groundActive&&numel(info.paperAllStages)==4,'CBF60:Integration','%s',info.status);
[ub,fb]=CBF_REAL_SCHEME_03_PAPER_EXTENSION('fallback',cfg,[436;0;0;0],.025);
assert(~fb.rateReset&&numel(ub)==4&&CBF_REAL_D2T_INPUT_KERNEL(fb.paperThrust0,fb.paperRate0,.025,cfg.paperThrustBounds,fb.paperNu),'CBF60:Fallback');
report=struct('passed',true,'centerScaledError',err,'maxAffineScaledError',maxerr,'maxLowerScaledError',maxlower,'modes',count,'actuatorCycles',cycles,'sampledDataCBFCertified',false);
fprintf('[REV60 SELFTEST] PASS; modes=%d actuatorCycles=%d affine=%.3g lower=%.3g\n',count,cycles,maxerr,maxlower);
end
