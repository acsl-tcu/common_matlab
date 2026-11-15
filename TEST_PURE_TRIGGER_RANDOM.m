function result=TEST_PURE_TRIGGER_RANDOM(numCases)
%TEST_PURE_TRIGGER_RANDOM Dense ground-truth audit for the mathematical gate.
% R=TEST_PURE_TRIGGER_RANDOM(300). Generated analytic curves are truly
% acceleration-bounded; all tests are independent of original HLC/plant.
% Passing a dense audit is NOT a formal mathematical proof or hardware safety.
if nargin<1,numCases=300;end
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'TRIGGER_LIGHT'));
rng(271828,'twister');
t=linspace(0,4,11);td=linspace(0,4,4001);
state=struct('pDrone',[0;0;3],'pLoad',[0;0;1], ...
             'vDrone',[.5;.1;0],'vLoad',[.5;.1;0]);
cfg=struct('droneRadius',.25,'loadRadius',.16,'cableRadius',.015, ...
    'safetyMargin',0,'checkGround',false);
falseClears=0;witnessHits=0;clears=0;risks=0;
for i=1:numCases
    omega=.8+1.5*rand();aAmp=.01+.17*rand();
    A=aAmp*omega^2;
    b=struct('aDrone',A,'aLoad',A,'posErrorDrone',0, ...
      'posErrorLoad',0,'velErrorDrone',0,'velErrorLoad',0);
    F=wp_trigger_constant_velocity_enclosure(state,t,b);
    c=[2*rand();2*rand()-1;2];r=.08+.24*rand();
    p=struct('kind','sphere','center',c','R',eye(3), ...
      'params',struct('radius',r),'obstacleName','observed_random');
    world=struct('parts',p,'ground',[],'dynamicIndices',[]);
    out=wp_trigger_swept_gate(wp_trigger_prepare_known(world),cfg,F);
    clear=strcmp(out.status,'CLEAR_CONDITIONAL');
    assert(clear || strcmp(out.status,'POSSIBLE_CONTACT'));
    % The REAL trajectory is distinct from the centerline (nonzero accel).
    trueX=state.pDrone(1)+state.vDrone(1)*td;
    trueY=state.pDrone(2)+state.vDrone(2)*td+ ...
        aAmp*(sin(omega*td)-omega*td);
    % z_obstacle=2 lies on the TRUE cable; test exact sphere-capsule distance.
    actualHit=any((trueX-c(1)).^2+(trueY-c(2)).^2<= ...
                 (r+cfg.cableRadius)^2);
    falseClears=falseClears+(clear && actualHit);
    witnessHits=witnessHits+actualHit;
    clears=clears+clear;risks=risks+~clear;
end
assert(falseClears==0,'CONDITIONAL_CLEAR missed an actual bounded collision');
result=struct('status','PASS_DENSE_ANALYTIC_GROUND_TRUTH', ...
    'numCases',numCases,'falseClearCases',falseClears, ...
    'witnessCollisionCases',witnessHits,'clearCases',clears, ...
    'possibleContactCases',risks, ...
    'continuousPhysicalProof',false,'actualHlcTest',false);
fprintf('PASS analytic-swing random audit: %d cases, %d true collisions, %d false-clear, %d clear, %d conservative risk.\n', ...
    numCases,witnessHits,falseClears,clears,risks);
end
