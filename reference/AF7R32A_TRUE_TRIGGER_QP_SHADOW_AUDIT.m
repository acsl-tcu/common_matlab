function OUT = AF7R32A_TRUE_TRIGGER_QP_SHADOW_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R32A | TRUE-TRIGGER EXACT SMALL-QP SHADOW AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowOnly=1 | exactActiveSetEnumeration=1 | heuristicNodeCap=0 | productionModification=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r32_exact_qp_mex','file')==3,'[AF7R32A] QP MEX missing.');
assert(exist('af7r31_geometry_batch_mex','file')==3,'[AF7R32A] R13A geometry MEX missing.');

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid)&&logical(E.real_flight_request_edge), ...
    '[AF7R32A] no real first trigger event.');
CTX=af7r32_replay_context(replanner,E);

P=replanner.empty_bezier_d15_c7_proposal();
P.t0=double(E.event_time);
P.tm=double(E.trigger.collision_time);
P.trigger_collision_time=P.tm;
P.obstacle_id=double(E.trigger.obstacle_id);
P.component_code=double(E.trigger.component_code);
P.base_eval_fn=replanner.snapshot_active_reference_eval_fn();
P.base_flat_eval_fn=replanner.snapshot_active_reference_flat_eval_fn();
searchEnd=replanner.resolve_reference_rejoin_search_end(P.tm);
P.t1=searchEnd;

[EPI,okE]=af7r31_episode_batch_v3(replanner,P,CTX,searchEnd);
assert(okE&&EPI.component_count>=1&&EPI.episode_count>=1,'[AF7R32A] episode replay failed.');

C=double(EPI.component_count); NE=numel(EPI.episodes);
nslot=2*C; nvar=3*nslot;
Hs=replanner.piecewise_minimum_snap_hessian(EPI.components,double(P.t0),double(searchEnd));
H=kron(Hs,eye(3)); H=0.5*(H+H.');

OR=struct('Ap',{},'bp',{},'Am',{},'bm',{});
for ie=1:NE
    n0=episode_normal(replanner,P,CTX,EPI,ie);
    [Ap,bp,op]=af7r31_corridor_batch_v3(replanner,P,CTX,EPI,ie,n0,nvar);
    [Am,bm,om]=af7r31_corridor_batch_v3(replanner,P,CTX,EPI,ie,-n0,nvar);
    assert(op&&om,'[AF7R32A] corridor replay failed.');
    OR(ie)=struct('Ap',Ap,'bp',bp,'Am',Am,'bm',bm); %#ok<AGROW>
end

SIGNS=all_signs(NE);
NS=size(SIGNS,2);
legacyOK=false(1,NS); nativeOK=false(1,NS);
zDiff=nan(1,NS); costDiff=nan(1,NS); slackDiff=nan(1,NS);
tested=zeros(1,NS); feasible=zeros(1,NS); spd=false(1,NS);

for is=1:NS
    [A,b]=assemble_rows(OR,SIGNS(:,is));
    [zl,cl,sl,ol]=replanner.solve_piecewise_fixed_sign_qp(H,A,b);
    [zn,cn,sn,on,tt,ff,ss]=af7r32_exact_qp_mex(H,A,b);
    legacyOK(is)=logical(ol); nativeOK(is)=logical(on); tested(is)=tt; feasible(is)=ff; spd(is)=ss;
    if ol&&on
        zDiff(is)=max(abs(double(zl(:))-double(zn(:))));
        costDiff(is)=abs(double(cl)-double(cn));
        slackDiff(is)=abs(double(sl)-double(sn));
    end
end

sameOK=isequal(legacyOK,nativeOK);
dz=max(zDiff,[],'omitnan'); dc=max(costDiff,[],'omitnan'); ds=max(slackDiff,[],'omitnan');
scale=max([1,dz,dc,ds,max(abs(H),[],'all')]);
tol=8192*eps(scale);
closed=sameOK&&all(spd)&& ...
    (isempty(dz)||isnan(dz)||dz<=tol) && ...
    (isempty(dc)||isnan(dc)||dc<=tol) && ...
    (isempty(ds)||isnan(ds)||ds<=tol);

% Timing all exact sign assignments as one synthesis-QP workload.
NREP=21; tLegacy=zeros(1,NREP); tNative=zeros(1,NREP);
for k=1:NREP
    q=tic;
    for is=1:NS
        [A,b]=assemble_rows(OR,SIGNS(:,is));
        replanner.solve_piecewise_fixed_sign_qp(H,A,b);
    end
    tLegacy(k)=1e3*toc(q);

    q=tic;
    for is=1:NS
        [A,b]=assemble_rows(OR,SIGNS(:,is));
        af7r32_exact_qp_mex(H,A,b);
    end
    tNative(k)=1e3*toc(q);
end

fprintf(' trigger time [s]                         = %.9f\n',double(E.event_time));
fprintf(' components / episodes / variables       = %d / %d / %d\n',C,NE,nvar);
fprintf(' exact sign assignments                  = %d\n',NS);
fprintf(' legacy/native feasible signs            = %d / %d\n',nnz(legacyOK),nnz(nativeOK));
fprintf(' max |z legacy-native|                   = %.17g\n',dz);
fprintf(' max |cost legacy-native|                = %.17g\n',dc);
fprintf(' max |slack legacy-native|               = %.17g\n',ds);
fprintf(' equivalence tolerance                   = %.17g\n',tol);
fprintf(' all H SPD for native theorem            = %d\n',all(spd));
fprintf(' native active sets tested total         = %.0f\n',sum(tested));
fprintf(' legacy all-sign min/med/max [ms]        = %.6g / %.6g / %.6g\n',min(tLegacy),median(tLegacy),max(tLegacy));
fprintf(' native all-sign min/med/max [ms]        = %.6g / %.6g / %.6g\n',min(tNative),median(tNative),max(tNative));
fprintf(' QP shadow equivalence closed            = %d\n',closed);
fprintf(' production source modified              = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R32A_TRUE_TRIGGER_QP_SHADOW_AUDIT_V1', ...
    'closed',closed,'legacy_ok',legacyOK,'native_ok',nativeOK, ...
    'max_z_diff',dz,'max_cost_diff',dc,'max_slack_diff',ds, ...
    'tolerance',tol,'legacy_ms',tLegacy,'native_ms',tNative, ...
    'tested_active_sets',tested,'feasible_active_sets',feasible);
assignin('base','AF7R32A_TRUE_TRIGGER_QP_SHADOW_AUDIT_OUT',OUT);
end

function S=all_signs(E)
if E==0
    S=zeros(0,1);
    return;
end
P=all_signs(E-1);
S=[ [ones(1,size(P,2));P], [-ones(1,size(P,2));P] ];
end

function [A,b]=assemble_rows(OR,s)
A=zeros(0,size(OR(1).Ap,2)); b=zeros(0,1);
for i=1:numel(OR)
    if s(i)>0, A=[A;OR(i).Ap]; b=[b;OR(i).bp]; %#ok<AGROW>
    else, A=[A;OR(i).Am]; b=[b;OR(i).bm]; %#ok<AGROW>
    end
end
end

function n0=episode_normal(replanner,P,CTX,EPI,ie)
ep=EPI.episodes(ie); tca=double(ep.start_s); tcb=double(ep.end_s);
xa=double(P.base_flat_eval_fn(tca)); xa=xa(:);
xb=double(P.base_flat_eval_fn(tcb)); xb=xb(:);
progress=xb(1:3)-xa(1:3);
if norm(progress)<=128*eps(max(1,norm(xa(1:3))))
    xm=double(P.base_flat_eval_fn(0.5*(tca+tcb))); xm=xm(:);
    progress=xm(5:7);
end
ii=double(ep.obstacle_index);
R=reshape(double(CTX.obstacles.rotations_flat(:,ii)),3,3);
a=double(CTX.obstacles.base_axes(:,ii));
n0=replanner.bezier_lateral_separator_normal(R,a,progress);
n0=n0/norm(n0);
end
