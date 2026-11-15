function R=TEST_TRIGGER_V5_KERNEL_BENCH(nReps)
%TEST_TRIGGER_V5_KERNEL_BENCH Timing of governor and modal matrix operations.
% Does NOT include building observed history, generating future reference,
% 3D taut reconstruction, 6-way geometry, EKF, or 25ms Flight pipeline.
% Do not mistake this for the user-requested 1-2ms complete trigger.
if nargin<1,nReps=300;end
assert(exist('wp_governor_step','file')==2);
C=wp_six_trigger_governor_fast_coeff(.025,.7);
cmd=.05+.95*(.5+.5*sin((1:160)*.027));
z0=ones(6,1);tau0=0;
% Warm-up and check exact first, then average contiguous timings.
for warm=1:2
    z=z0;tau=tau0;
    for k=1:160
        [z,tau]=wp_six_trigger_governor_fast_step(z,tau,cmd(k),C);
    end
end
nG=max(15,ceil(nReps/10));
tic
for j=1:nG
    z=z0;tau=tau0;
    for k=1:160
        [z,tau]=wp_governor_step(z,tau,cmd(k),.025,.7);
    end
end
origMs=1000*toc/nG;
tic
for j=1:nG
    z=z0;tau=tau0;
    for k=1:160
        [z,tau]=wp_six_trigger_governor_fast_step(z,tau,cmd(k),C);
    end
end
fastMs=1000*toc/nG;
% Mode-fit matrix work on cached exact transitions, artificial history.
% The initial observed data only affect the RHS and do not change timing.
agent=DRONE();
agent.parameter=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
agent.plant=struct('name','V5_BENCH_ONLY');
ctrl=Controller_HL_Suspended_Load(.025,agent);
A6=diag(ones(5,1),1);
A6(6,:)=-reshape(ctrl.F2,1,[]);
tpast=(-40:0)*.025;tfuture=(0:160)*.025;
Hp=zeros(numel(tpast),6);Hf=zeros(numel(tfuture),6);
for k=1:numel(tpast),B=expm(A6*tpast(k));Hp(k,:)=B(1,:);end
for k=1:numel(tfuture),B=expm(A6*tfuture(k));Hf(k,:)=B(1,:);end
obs=sin(tpast(:))+1;
tic
for j=1:nReps
    x=Hp\obs;
    epred=Hf*x; %#ok<NASGU>
end
modalOneAxisMs=1000*toc/nReps;
R=struct('status','BENCH_ONLY_NOT_FULL_TRIGGER', ...
    'originalGovernor160Ms',origMs,'fastGovernor160Ms',fastMs, ...
    'modalOneAxisCachedMatricesMs',modalOneAxisMs, ...
    'modalAllAxesWouldNeedMore',true, ...
    'originalReferenceGeneratorIncluded',false, ...
    'observedHistoryDataAcquisitionIncluded',false, ...
    'originalHLCIncluded',false,'geometryIncluded',false, ...
    'totalTrigger1to2msDemonstrated',false);
end
