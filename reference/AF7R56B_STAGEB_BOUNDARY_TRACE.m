function OUT = AF7R56B_STAGEB_BOUNDARY_TRACE(replanner,B49,R50A)
% AF7R56B_STAGEB_BOUNDARY_TRACE
%
% This closes the KKT prerequisite on the ACTUAL fixed-corridor boundary
% found by AF7R55B, then exposes the exact adaptive Stage-B leaf topology at
% both adjacent boundary doubles for every sign candidate.
%
% This is still NOT the outward Stage-B proof.  It determines the common
% dyadic leaf refinement that the next interval theorem must enclose.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R56B | FIXED-BOUNDARY KKT + STAGE-B LEAF TRACE\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | actualFixedBoundary=1 | sampledAcceptance=0 | productionT1Selected=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r56_trace_mex','file')==3);
assert(exist('af7r56_interval_kkt_mex','file')==3);

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);
if nargin<2||isempty(B49)
    assert(isfield(AUX,'B49'),'[AF7R56B] B49 unavailable.');B49=AUX.B49;
end
if nargin<3||isempty(R50A)
    if isfield(AUX,'R50A'),R50A=AUX.R50A;else,R50A=struct();end %#ok<NASGU>
end

AF7R56B_PRIME_TRACE_REFERENCE(replanner,M);

CTX=af7r49_replay_context(replanner,E);
pkt=af7r34_make_core_packet(replanner,CTX,[],M);
SC0=E.speed_certificate;
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);

% Known-safe upper endpoint.
t=double(B49.t1(:).');oldPass=logical(B49.success(:).');
d=diff([false oldPass false]);beg=find(d==1);fin=find(d==-1)-1;
term=find(fin==numel(oldPass),1,'last');assert(~isempty(term));
tU=t(fin(term));

F=af7r56_trace_mex(3,pkt,tU);
assert(logical(F.episode_ok)&&logical(F.tail_safe)&&double(F.tail_repr_fail)==0);
U=af7r56_trace_mex(4,pkt,tU,tU);
n=double(U.all_candidate_count);
assert(n>=1);

% Locate terminal component of the theorem-backed fixed synthesis.
use=find(t>double(F.tail_start)&t<=tU);
succ=false(size(use));reason=nan(size(use));
for ii=1:numel(use)
    R=af7r56_trace_mex(4,pkt,tU,t(use(ii)));
    succ(ii)=logical(R.success);reason(ii)=double(R.reason_code);
end
dd=diff([false succ false]);bb=find(dd==1);ff=find(dd==-1)-1;
terminal=find(ff==numel(use),1,'last');
assert(~isempty(terminal)&&bb(terminal)>1);
iL=bb(terminal);

lo=t(use(iL-1));hi=t(use(iL));
for it=1:64
    mid=0.5*(lo+hi);
    if ~(mid>lo&&mid<hi),break;end
    R=af7r56_trace_mex(4,pkt,tU,mid);
    if logical(R.success),hi=mid;else,lo=mid;end
end
RL=af7r56_trace_mex(4,pkt,tU,lo);
RU=af7r56_trace_mex(4,pkt,tU,hi);

fprintf(' authority source                         = %s\n',authoritySource);
fprintf(' fixed tU [s]                             = %.15g\n',tU);
fprintf(' actual fixed boundary bracket [s]        = [%.17g, %.17g]\n',lo,hi);
fprintf(' bracket width [s]                        = %.17g\n',hi-lo);
fprintf(' lower/upper success                      = %d / %d\n',logical(RL.success),logical(RU.success));
fprintf(' lower/upper reason                       = %.0f / %.0f\n',double(RL.reason_code),double(RU.reason_code));

% Fixed rows / all-candidate KKT on THIS actual boundary.
allKKT=true;K=cell(1,n);
fprintf('\n[A. ACTUAL FIXED-BOUNDARY OUTWARD KKT]\n');
for c=1:n
    sg=double(U.all_candidate_signs{c}(:).');
    ac0=double(U.all_candidate_active_set{c}(:).');
    A=double(U.all_candidate_rows_A{c});
    b=double(U.all_candidate_rows_b{c});
    fprintf(' candidate %.0f sign / native active      = %s / %s\n',c,mat2str(sg),mat2str(ac0));
    if numel(ac0)==1&&double(U.component_count)==1
        signTag=0;if ~isempty(sg),signTag=sg(1);end
        K{c}=af7r56_interval_kkt_mex( ...
            lo,hi,double(U.t0),double(U.component_start(1)), ...
            double(U.component_end(1)),A,b,ac0(1)+1,signTag,c);
        fprintf(' candidate %.0f interval KKT closed        = %d\n',c,logical(K{c}.closed));
        fprintf(' candidate %.0f lambda interval            = [%.17g %.17g]\n', ...
            c,K{c}.lambda(1),K{c}.lambda(2));
        fprintf(' candidate %.0f inactive slack LB          = %.17g\n', ...
            c,double(K{c}.min_inactive_slack_lb));
        allKKT=allKKT&&logical(K{c}.closed);
    else
        allKKT=false;
        fprintf(' candidate %.0f interval KKT closed        = 0\n',c);
    end
end
fprintf(' all-candidate KKT on actual boundary     = %d\n',allKKT);

% Exact adaptive Stage-B traces at both endpoints.
TL=cell(1,n);TU=cell(1,n);
topologyKnown=true;
fprintf('\n[B. STAGE-B ENDPOINT LEAF TOPOLOGY]\n');
for c=1:n
    TL{c}=af7r56_trace_mex(5,pkt,tU,lo,c);
    TU{c}=af7r56_trace_mex(5,pkt,tU,hi,c);

    keyL=leaf_keys(TL{c});
    keyU=leaf_keys(TU{c});
    common=numel(intersect(keyL,keyU));
    uni=numel(union(keyL,keyU));

    [wL,iwL]=nanmaxv(double(TL{c}.leaf_witness));
    [qU,iqU]=nanmaxv(double(TU{c}.leaf_vq));

    fprintf(' candidate %.0f lower cert/witness/repr    = %d / %d / %d\n', ...
        c,logical(TL{c}.certified),logical(TL{c}.finite_witness),logical(TL{c}.repr));
    fprintf(' candidate %.0f upper cert/witness/repr    = %d / %d / %d\n', ...
        c,logical(TU{c}.certified),logical(TU{c}.finite_witness),logical(TU{c}.repr));
    fprintf(' candidate %.0f lower leaves/splits        = %d / %.0f\n', ...
        c,numel(TL{c}.leaf_seg),double(TL{c}.split_count));
    fprintf(' candidate %.0f upper leaves/splits        = %d / %.0f\n', ...
        c,numel(TU{c}.leaf_seg),double(TU{c}.split_count));
    fprintf(' candidate %.0f dyadic leaf common/union   = %d / %d\n',c,common,uni);

    if isfinite(iwL)
        fprintf(' candidate %.0f max lower witness          = %.15g | seg=%g u=[%.17g %.17g]\n', ...
            c,wL,double(TL{c}.leaf_seg(iwL)),double(TL{c}.leaf_u0(iwL)),double(TL{c}.leaf_u1(iwL)));
    end
    if isfinite(iqU)
        fprintf(' candidate %.0f max upper leaf VQ          = %.15g | seg=%g u=[%.17g %.17g]\n', ...
            c,qU,double(TU{c}.leaf_seg(iqU)),double(TU{c}.leaf_u0(iqU)),double(TU{c}.leaf_u1(iqU)));
    end

    topologyKnown=topologyKnown && ~isempty(keyL) && ~isempty(keyU);
end

% Same finite-sensing authority as production.
rho=max([double(E.dependency_signature.r_drone), ...
    double(E.dependency_signature.cable_length)+double(E.dependency_signature.r_load), ...
    double(E.dependency_signature.cable_length)+double(E.dependency_signature.r_cable)]);
Vlim=(double(E.dependency_signature.sensor_range)-rho)/ ...
    (double(E.dependency_signature.horizon)+double(E.dependency_signature.dt));

fprintf('\n[C. FINITE-SENSING AUTHORITY]\n');
fprintf(' VQ limit [m/s]                           = %.15g\n',Vlim);
fprintf(' selected lower finite witness [m/s]      = %.15g\n',double(TL{1}.witness_max));
fprintf(' selected lower witness-limit [m/s]       = %.15g\n',double(TL{1}.witness_max)-Vlim);
fprintf(' selected upper certified VQ [m/s]        = %.15g\n',double(TU{1}.speed_uav));
fprintf(' selected upper limit-VQ [m/s]            = %.17g\n',Vlim-double(TU{1}.speed_uav));

fprintf('\n[D. FORMAL GATE]\n');
fprintf(' fixed-domain episode theorem             = 1\n');
fprintf(' actual-boundary all-candidate KKT closed = %d\n',allKKT);
fprintf(' endpoint Stage-B topology characterized  = %d\n',topologyKnown);
fprintf(' outward Stage-B leaf interval proof      = 0\n');
fprintf(' continuous terminal-boundary proof       = 0\n');
fprintf(' production t1 selected                   = 0\n');
fprintf(' next theorem                             = COMMON_DYADIC_REFINEMENT_INTERVAL_STAGEB\n');
fprintf(' unresolved interval policy               = FAIL CLOSED\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R56B_STAGEB_BOUNDARY_TRACE_V1';
OUT.boundary_bracket=[lo hi];
OUT.all_candidate_kkt_closed=allKKT;
OUT.kkt=K;
OUT.lower_trace=TL;
OUT.upper_trace=TU;
OUT.topology_characterized=topologyKnown;
OUT.stageb_interval_closed=false;
OUT.production_t1_selected=false;
assignin('base','AF7R56B_STAGEB_BOUNDARY_TRACE_OUT',OUT);
end

function K=leaf_keys(T)
seg=double(T.leaf_seg(:));u0=double(T.leaf_u0(:));u1=double(T.leaf_u1(:));
K=cell(numel(seg),1);
for k=1:numel(seg)
    % u intervals are dyadic because production Stage-B only splits halves.
    K{k}=sprintf('s%.0f|%.17g|%.17g',seg(k),u0(k),u1(k));
end
end

function [m,ix]=nanmaxv(x)
x=double(x(:));good=isfinite(x);
if ~any(good),m=NaN;ix=NaN;return;end
ii=find(good);[m,j]=max(x(good));ix=ii(j);
end
