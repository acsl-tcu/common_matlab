function summary=BENCH_PURE_TRIGGER_1_2MS(repeats)
%BENCH_PURE_TRIGGER_1_2MS Real MATLAB timing, all calls perform work.
% B=BENCH_PURE_TRIGGER_1_2MS(1000)
% MUST report median, P95, MAX, state and number of checked intervals;
% not a flight 25-ms / worst-case timing guarantee.
if nargin<1,repeats=1000;end
assert(isscalar(repeats) && repeats>=10 && repeats==floor(repeats));
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'TRIGGER_LIGHT'));
cfg=struct('droneRadius',.25,'loadRadius',.16,'cableRadius',.015, ...
  'safetyMargin',0,'checkGround',false);
t=linspace(0,4,41);
F=struct('timeOffsets',t,'pDrone',[t;zeros(1,numel(t));3*ones(1,numel(t))], ...
 'pLoad',[t;zeros(1,numel(t));ones(1,numel(t))], ...
 'rhoDrone',zeros(1,numel(t)),'rhoLoad',zeros(1,numel(t)), ...
 'accDrone',zeros(1,numel(t)-1),'accLoad',zeros(1,numel(t)-1));
contexts={'empty_observation','distant_36_parts','near_cable_collision','offset_near_36'};
summary=struct();
for caseNum=1:4
    switch caseNum
        case 1,parts=makeParts(0,false);
        case 2,parts=makeParts(36,false);
        case 3,parts=makeParts(1,true);
        case 4,parts=makeParts(36,true);
    end
    world=struct('parts',parts,'ground',[],'dynamicIndices',[]);
    known=wp_trigger_prepare_known(world);
    Ftest=F;
    if caseNum==4,Ftest.pDrone(2,:)=.5;Ftest.pLoad(2,:)=.5;end
    % Warm-up, but DO NOT count as a performance measurement.
    for z=1:10,wp_trigger_swept_gate(known,cfg,Ftest);end
    ms=zeros(1,repeats);status=cell(1,repeats);allIntervals=zeros(1,repeats);
    for i=1:repeats
        start=tic;
        r=wp_trigger_swept_gate(known,cfg,Ftest);
        ms(i)=toc(start)*1e3;
        status{i}=r.status;allIntervals(i)=r.intervalsChecked;
    end
    sm=sort(ms);p95=sm(ceil(.95*repeats));
    result=struct('medianMs',median(ms),'p95Ms',p95,'maxMs',max(ms), ...
      'minMs',min(ms),'numParts',numel(parts), ...
      'resultStatus',status{end},'checkedIntervals',allIntervals(end), ...
      'numMeasuredCalls',repeats,'everyCallEvaluated',all(allIntervals>0), ...
      'targetTwoMsMetAllCalls',all(ms<=2));
    summary.(contexts{caseNum})=result;
    fprintf('%-23s median %.4f ms  P95 %.4f ms  MAX %.4f ms  status %-20s intervals %d\n', ...
       contexts{caseNum},result.medianMs,p95,result.maxMs, ...
       result.resultStatus,result.checkedIntervals);
end
summary.hardwareIndependentGuarantee=false;
summary.fullControlLoopMeasured=false;
summary.physicalSafetyCertified=false;
fprintf('Benchmark includes geometry gate only (no EKF/HLC/CBF/trajectory generation).\n');
end
function parts=makeParts(n,near)
template=struct('kind','ellipsoid','center',[0 0 0], ...
    'R',eye(3),'params',struct('axes',[.5 .4 .3]),'obstacleName','');
parts=repmat(template,1,n);
for i=1:n
    if near
        center=[1+(i-1)*.02,0,2];
    else
        center=[50+i,20,10];
    end
    parts(i).center=center;parts(i).obstacleName=sprintf('observed_%02d',i);
end
end
