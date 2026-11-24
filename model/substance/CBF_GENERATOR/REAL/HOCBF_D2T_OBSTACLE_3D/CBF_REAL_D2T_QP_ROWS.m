function rows = CBF_REAL_D2T_QP_ROWS(y,tauZ,P,G,parts,lambda,dt,cfg)
% Return linear rows A*v>=b in v=[ddT;tauX;tauY], obstacle-only HOCBF.
% This is an INSTANTANEOUS continuous-time HOCBF, not a sampled-data proof.
% Exact next-hold thrust box constraints are imposed through a helper.
% Lower stages psi0..psi3 must already be >=0 for the HOCBF theorem.
validateattributes(dt,{'double'},{'scalar','positive','finite'});
if nargin<8,cfg=struct();end
lim=[0,20];
if isfield(cfg,'thrustBounds'),lim=cfg.thrustBounds;end
validateattributes(lim,{'double'},{'vector','numel',2,'finite','real'});
lim=lim(:);assert(lim(1)>=0 && lim(2)>lim(1),'CBFD2T:ThrustBounds');
y=y(:);T=y(20);rate=y(21);
base=CBF_REAL_D2T_OBSTACLE_EVAL(y,zeros(3,1),tauZ,P,G,parts,lambda);
n=numel(parts);B=zeros(n,3);tol=1e-8;
affErr=0;lowerErr=0;
for c=1:3
    unit=zeros(3,1);unit(c)=1;
    e=CBF_REAL_D2T_OBSTACLE_EVAL(y,unit,tauZ,P,G,parts,lambda);
    B(:,c)=(e.stages(5,:)-base.stages(5,:)).';
    if n>0
        lowerErr=max(lowerErr,max(abs(e.stages(1:4,:)-base.stages(1:4,:)),[],'all'));
    end
end
probe=[0.37;-0.41;0.23];
mix=CBF_REAL_D2T_OBSTACLE_EVAL(y,probe,tauZ,P,G,parts,lambda);
if n>0
    affErr=max(abs(mix.stages(5,:).'-base.stages(5,:).'-B*probe));
    lowerErr=max(lowerErr,max(abs(mix.stages(1:4,:)-base.stages(1:4,:)),[],'all'));
end
scale=max([1;abs(base.stages(:));abs(mix.stages(:));abs(B(:))]);
%A*v>=b : barrier B*v>= -psi4(0).
A=B;b=-base.stages(5,:).';
names=base.names(:);
% Torque box, ±1 Nm, unchanged HLC yaw torque.
A=[A;0 1 0;0 -1 0;0 0 1;0 0 -1];
b=[b;-1;-1;-1;-1];
names=[names;{'tauX_min';'tauX_max';'tauY_min';'tauY_max'}];
% Tight NECESSARY AND SUFFICIENT ddT interval for the ENTIRE hold.
% The two rows are linear in the chosen ddT despite the state-dependent
% evaluation of their bounds; no guessed actuator ddT limit is required.
tr=CBF_REAL_D2T_THRUST_ROWS(T,rate,dt,lim);
A=[A;tr.A];b=[b;tr.b];names=[names;tr.names];
% Optional extra physical actuator bound, only when provided explicitly.
if isfield(cfg,'ddTBounds')
    vv=cfg.ddTBounds(:);
    validateattributes(vv,{'double'},{'vector','numel',2,'finite','real'});
    assert(vv(2)>vv(1),'CBFD2T:ddTBounds');
    A=[A;1 0 0;-1 0 0];
    b=[b;vv(1);-vv(2)];
    names=[names;{'ddT_actuator_min';'ddT_actuator_max'}];
    effectiveDdT=[max(tr.ddTInterval(1),vv(1)); min(tr.ddTInterval(2),vv(2))];
else
    effectiveDdT=tr.ddTInterval;
end
lowerValid=all(base.stages(1:4,:)>=-tol,'all');
actuatorIntersectionValid=effectiveDdT(1)<=effectiveDdT(2);
baseValid=tr.feasibleInitialHold && actuatorIntersectionValid;
affValid=affErr<=1e-7*scale && lowerErr<=1e-7*scale;
rows=struct('A',A,'b',b,'names',{names},'controlNames', ...
    {{'ddT','tauX','tauY'}},'obstacleNames',{base.names}, ...
    'barrierBias',base.stages(5,:).','barrierInfluence',B, ...
    'lowerStages',base.stages(1:4,:),'hValues',base.stages(1,:), ...
    'hDerivatives',base.hDerivatives,'thrustBounds',lim, ...
    'lowerStagesValid',lowerValid,'thrustHoldValid',baseValid, ...
    'thrustHoldStatus',tr.status,'thrustHoldRows',tr, ...
    'actuatorIntersectionValid',actuatorIntersectionValid, ...
    'thrustExactDdTInterval',tr.ddTInterval, ...
    'effectiveDdTInterval',effectiveDdT, ...
    'affineValid',affValid,'affinityError',affErr, ...
    'lowerStageInputError',lowerErr,'validForSampledDataGuarantee',false, ...
    'ddTBoundsSpecified',isfield(cfg,'ddTBounds'), ...
    'status','ROWS_CONSTRUCTED','numObstacles',n);
if ~affValid,rows.status='AFFINITY_CHECK_FAILED';
elseif ~tr.feasibleInitialHold,rows.status=['THRUST_HOLD_INFEASIBLE_' tr.status];
elseif ~actuatorIntersectionValid,rows.status='ACTUATOR_DDT_INTERSECTION_EMPTY';
elseif ~lowerValid,rows.status='LOWER_HOCBF_STAGE_NEGATIVE';end
end
