function OUT = AF7R52B_V2_TERMINAL_BRANCH_KKT_SHADOW(replanner,B49,R50A)
% AF7R52B_TERMINAL_BRANCH_KKT_SHADOW
%
% Track B:
%   Expose the exact QP sign/active-set branch around the terminal feasible
%   component that contains the known-safe source-domain upper bound t_U.
%
% IMPORTANT:
%   This file does NOT promote a sampled/grid/bisection result to production.
%   It only determines which KKT branch must be intervalized next.
%   Final production t1 remains UNSELECTED until an interval/outward-rounding
%   proof closes.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R52B V2 | TERMINAL COMPONENT KKT / SPEED-BRANCH SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | shadowOnly=1 | sampledSafetyAcceptance=0 | productionT1Selected=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r36_single_core_mex','file')==3,'[AF7R52B] AF7R36 MEX missing.');
assert(exist('af7r52_branch_mex','file')==3,'[AF7R52B] build AF7R52 MEX first.');

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);
% Newly built MEX persistent storage is empty until explicitly primed.
% Bind the shadow MEX to the SAME current/saved nominal source before op=1.
PRIME52=AF7R52B_V2_PRIME_BRANCH_REFERENCE(replanner,M); %#ok<NASGU>
if nargin<2 || isempty(B49)
    assert(isfield(AUX,'B49'),'[AF7R52B] B49 not supplied and not present in theory snapshot.');
    B49=AUX.B49;
end
if nargin<3 || isempty(R50A)
    if isfield(AUX,'R50A'), R50A=AUX.R50A; else, R50A=struct(); end
end
assert(isstruct(B49)&&isfield(B49,'success')&&isfield(B49,'t1'));

[~,pkt]=make_true_trigger_packet(replanner,E,M);

t=double(B49.t1(:).');
pass=logical(B49.success(:).');
reason=double(B49.reason(:).');
assert(numel(t)==numel(pass)&&numel(pass)==numel(reason));

d=diff([false pass false]);
beg=find(d==1); fin=find(d==-1)-1;
term=find(fin==numel(pass),1,'last');
assert(~isempty(term),'[AF7R52B] terminal component containing t_U not found.');
kL=beg(term); kU=fin(term);
assert(kL>1,'[AF7R52B] no failing predecessor for terminal component.');

tFail=t(kL-1);
tPass=t(kL);
tU=t(kU);

D=E.dependency_signature;
rho=max([double(D.r_drone),double(D.cable_length)+double(D.r_load), ...
         double(D.cable_length)+double(D.r_cable)]);
Vlim=(double(D.sensor_range)-rho)/(double(D.horizon)+double(D.dt));

fprintf(' authority source                         = %s\n',authoritySource);
fprintf(' terminal diagnostic lattice component    = [%.15g, %.15g] s\n',tPass,tU);
fprintf(' immediate predecessor                    = %.15g s | reason=%g\n',tFail,reason(kL-1));
fprintf(' finite-sensing VQ limit [m/s]            = %.15g\n',Vlim);

% -------------------------------------------------------------------------
% 1) Prove the diagnostic branch MEX has not changed AF7R36 output.
% -------------------------------------------------------------------------
checkIdx=unique(round(linspace(max(1,kL-2),kU,17)));
maxCp=0;maxSpeed=0;maxGeom=0;maxCost=0;
sameDiscrete=true;
for jj=checkIdx
    P=pkt;P.t1=t(jj);
    A=af7r36_single_core_mex(1,P);
    B=af7r52_branch_mex(1,P);

    sameDiscrete=sameDiscrete && isequal(logical(A.success),logical(B.success)) && ...
        isequal(double(A.reason_code),double(B.reason_code)) && ...
        isequal(double(A.episode_count),double(B.episode_count)) && ...
        isequal(double(A.component_count),double(B.component_count)) && ...
        isequal(double(A.segment_t0),double(B.segment_t0)) && ...
        isequal(double(A.segment_t1),double(B.segment_t1));
    if logical(A.success)
        maxCp=max(maxCp,cpdiff(A.control_points,B.control_points));
        maxSpeed=max(maxSpeed,maxabs([double(A.speed_payload)-double(B.speed_payload); ...
            double(A.speed_uav)-double(B.speed_uav);double(A.speed_sigma)-double(B.speed_sigma); ...
            double(A.speed_qdot)-double(B.speed_qdot)]));
        maxGeom=max(maxGeom,maxabs([double(A.geometry_min_margin)-double(B.geometry_min_margin); ...
            double(A.geometry_min_payload_margin)-double(B.geometry_min_payload_margin); ...
            double(A.geometry_min_uav_margin)-double(B.geometry_min_uav_margin); ...
            double(A.geometry_min_cable_margin)-double(B.geometry_min_cable_margin)]));
        maxCost=max(maxCost,abs(double(A.qp_cost)-double(B.qp_cost)));
        sameDiscrete=sameDiscrete && isequal(double(A.corridor_signs),double(B.corridor_signs));
    end
end
tol=4096*eps(max(1,max([abs(t),Vlim])));
coreEquivalent=sameDiscrete&&maxCp<=tol&&maxSpeed<=tol&&maxGeom<=tol&&maxCost<=tol;

fprintf('\n[A. DIAGNOSTIC-CORE EQUIVALENCE]\n');
fprintf(' discrete fields identical               = %d\n',sameDiscrete);
fprintf(' max control-point diff                  = %.17g\n',maxCp);
fprintf(' max speed-field diff                    = %.17g\n',maxSpeed);
fprintf(' max geometry-margin diff                = %.17g\n',maxGeom);
fprintf(' max QP-cost diff                        = %.17g\n',maxCost);
fprintf(' diagnostic core equivalence closed      = %d\n',coreEquivalent);
assert(coreEquivalent,'[AF7R52B] diagnostic branch core changed AF7R36 mathematics.');

% -------------------------------------------------------------------------
% 2) Traverse the terminal component and record exact KKT fingerprints.
% -------------------------------------------------------------------------
idx=(kL-1):kU;
N=numel(idx);
fpFirst=cell(1,N);fpSelected=cell(1,N);
succ=false(1,N);rsn=nan(1,N);vqm=nan(1,N);witness=nan(1,N);
finiteWitness=false(1,N);reprFail=false(1,N);
activeCountFirst=nan(1,N);activeCountSelected=nan(1,N);
minLambdaFirst=nan(1,N);minLambdaSelected=nan(1,N);

for ii=1:N
    k=idx(ii);P=pkt;P.t1=t(k);
    R=af7r52_branch_mex(1,P);
    succ(ii)=logical(R.success);rsn(ii)=double(R.reason_code);
    fpFirst{ii}=fingerprint(R.diagnostic_corridor_signs,R.diagnostic_qp_active_set);
    activeCountFirst(ii)=numel(R.diagnostic_qp_active_set);
    if ~isempty(R.diagnostic_qp_lambda)
        minLambdaFirst(ii)=min(double(R.diagnostic_qp_lambda(:)));
    end
    finiteWitness(ii)=logical(R.diagnostic_finite_witness);
    reprFail(ii)=logical(R.diagnostic_repr);
    witness(ii)=double(R.diagnostic_witness_max);

    if succ(ii)
        fpSelected{ii}=fingerprint(R.corridor_signs,R.qp_active_set);
        activeCountSelected(ii)=numel(R.qp_active_set);
        if ~isempty(R.qp_lambda),minLambdaSelected(ii)=min(double(R.qp_lambda(:)));end
        vqm(ii)=Vlim-double(R.speed_uav);
    else
        fpSelected{ii}='';
    end
end

termSuccess=find(succ);
assert(~isempty(termSuccess));
uniqueFirst=unique(fpFirst);
uniqueSelected=unique(fpSelected(succ));

pre=1;
firstPass=find(succ,1,'first');
sameBoundaryFirstBranch=strcmp(fpFirst{pre},fpFirst{firstPass});
selectedEqualsFirstAtPass=strcmp(fpSelected{firstPass},fpFirst{firstPass});

fprintf('\n[B. TERMINAL BRANCH TOPOLOGY]\n');
fprintf(' terminal points replayed                 = %d\n',N);
fprintf(' unique first-QP branch fingerprints      = %d\n',numel(uniqueFirst));
fprintf(' unique selected-safe branch fingerprints = %d\n',numel(uniqueSelected));
fprintf(' fail predecessor / first-pass same first branch = %d\n',sameBoundaryFirstBranch);
fprintf(' first-pass selected branch == first-QP branch   = %d\n',selectedEqualsFirstAtPass);
fprintf(' predecessor finite-witness reject        = %d\n',finiteWitness(pre));
fprintf(' predecessor representability failure     = %d\n',reprFail(pre));
fprintf(' predecessor witness VQ [m/s]             = %.15g\n',witness(pre));
fprintf(' witness - VQ limit [m/s]                 = %.15g\n',witness(pre)-Vlim);
fprintf(' first-pass certified VQ margin [m/s]     = %.15g\n',vqm(firstPass));
fprintf(' first-QP active count fail/pass          = %.0f / %.0f\n', ...
    activeCountFirst(pre),activeCountFirst(firstPass));
fprintf(' first-QP min lambda fail/pass             = %.15g / %.15g\n', ...
    minLambdaFirst(pre),minLambdaFirst(firstPass));

% -------------------------------------------------------------------------
% 3) Numerical bracket only, restricted to the same observed KKT fingerprint.
% This is NOT a production authority and is deliberately labelled so.
% -------------------------------------------------------------------------
lo=tFail; hi=tPass;
targetFP=fpFirst{firstPass};
bracketStable=sameBoundaryFirstBranch;
lastLoReason=reason(kL-1);
for it=1:56
    if ~bracketStable, break; end
    mid=0.5*(lo+hi);
    if ~(mid>lo&&mid<hi), break; end
    P=pkt;P.t1=mid;
    R=af7r52_branch_mex(1,P);
    fpm=fingerprint(R.diagnostic_corridor_signs,R.diagnostic_qp_active_set);
    if ~strcmp(fpm,targetFP)
        bracketStable=false;break;
    end
    if logical(R.success)
        if ~strcmp(fingerprint(R.corridor_signs,R.qp_active_set),targetFP)
            bracketStable=false;break;
        end
        hi=mid;
    else
        if double(R.reason_code)~=24
            bracketStable=false;break;
        end
        lo=mid;lastLoReason=double(R.reason_code);
    end
end

P=pkt;P.t1=hi;Rh=af7r52_branch_mex(1,P);
hiMargin=NaN;
if logical(Rh.success),hiMargin=Vlim-double(Rh.speed_uav);end

fprintf('\n[C. NUMERICAL SAME-BRANCH BOUNDARY BRACKET — DIAGNOSTIC ONLY]\n');
fprintf(' branch fingerprint stable during refine  = %d\n',bracketStable);
fprintf(' lower / upper [s]                        = %.17g / %.17g\n',lo,hi);
fprintf(' bracket width [s]                        = %.17g\n',hi-lo);
fprintf(' lower reason                             = %g\n',lastLoReason);
fprintf(' upper directly certified                 = %d\n',logical(Rh.success));
fprintf(' upper finite-sensing margin [m/s]        = %.17g\n',hiMargin);
fprintf(' production selection from this bracket   = 0\n');

% -------------------------------------------------------------------------
% 4) Formal proof gate.
% -------------------------------------------------------------------------
% Even a machine-tight pointwise bracket is NOT an interval proof.  The
% missing obligation is an outward-rounded enclosure of the fixed KKT branch
% and Stage-B leaf inequalities over the whole t1 interval.
if bracketStable && sameBoundaryFirstBranch && selectedEqualsFirstAtPass
    next='INTERVALIZE_FIXED_KKT_BRANCH_AND_STAGE_B_LEAF_INEQUALITIES';
else
    next='SPLIT_KKT_BRANCHES_THEN_INTERVALIZE_EACH_BRANCH';
end

fprintf('\n[D. FORMAL CONTINUATION GATE]\n');
fprintf(' interval/outward KKT enclosure closed    = 0\n');
fprintf(' interval Stage-B enclosure closed        = 0\n');
fprintf(' validated terminal-boundary proof closed = 0\n');
fprintf(' production t1 selected                   = 0\n');
fprintf(' next proof implementation                = %s\n',next);
fprintf(' existing final AF7R36 certificate        = UNCHANGED\n');
fprintf(' unresolved interval policy               = FAIL CLOSED\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R52B_V2_TERMINAL_BRANCH_KKT_SHADOW_V1';
OUT.core_equivalence_closed=coreEquivalent;
OUT.terminal_component=[tPass tU];
OUT.fail_predecessor=tFail;
OUT.unique_first_branch_count=numel(uniqueFirst);
OUT.unique_selected_branch_count=numel(uniqueSelected);
OUT.same_boundary_first_branch=sameBoundaryFirstBranch;
OUT.selected_equals_first_at_pass=selectedEqualsFirstAtPass;
OUT.predecessor_finite_witness=finiteWitness(pre);
OUT.predecessor_witness_m_s=witness(pre);
OUT.VQ_limit_m_s=Vlim;
OUT.first_pass_speed_margin_m_s=vqm(firstPass);
OUT.diagnostic_boundary_bracket=[lo hi];
OUT.bracket_branch_stable=bracketStable;
OUT.interval_proof_closed=false;
OUT.production_t1_selected=false;
OUT.next_proof_implementation=next;
assignin('base','AF7R52B_TERMINAL_BRANCH_KKT_SHADOW_OUT',OUT);
end


function [CTX,pkt]=make_true_trigger_packet(replanner,E,M)
CTX=af7r49_replay_context(replanner,E);
pkt=af7r34_make_core_packet(replanner,CTX,[],M);
SC0=E.speed_certificate;
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);
end


function s=fingerprint(signs,active)
a=double(signs(:).');b=double(active(:).');
s=['S' mat2str(a) '|A' mat2str(b)];
end


function d=cpdiff(A,B)
assert(numel(A)==numel(B));
d=0;
for k=1:numel(A)
    d=max(d,maxabs(double(A{k})-double(B{k})));
end
end


function y=maxabs(x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),y=0;else,y=max(abs(x));end
end
