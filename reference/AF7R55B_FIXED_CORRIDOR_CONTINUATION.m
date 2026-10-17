function OUT = AF7R55B_FIXED_CORRIDOR_CONTINUATION(replanner,B49,R50A)
% AF7R55B_FIXED_CORRIDOR_CONTINUATION
%
% R54 proved:
%   (1) one fixed unsafe episode built on [t0,tU],
%   (2) the nominal whole body is continuously safe after the last episode
%       through tU,
%   (3) both sign-candidate KKT branches are interval-valid on the old
%       machine bracket.
%
% This script now evaluates the NEW fixed-corridor synthesis itself as t1
% varies.  The old AF7R36 bracket is NOT assumed to remain its boundary.
%
% The lattice is diagnostic only.  No sampled point is production authority.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R55B | FIXED-CORRIDOR CONTINUATION SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | fixedEpisodeTheoremRequired=1 | sampledAcceptance=0 | productionT1Selected=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r55_fixed_continuation_mex','file')==3);
assert(exist('af7r36_single_core_mex','file')==3);

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);
if nargin<2||isempty(B49)
    assert(isfield(AUX,'B49'),'[AF7R55B] B49 unavailable.');B49=AUX.B49;
end
if nargin<3||isempty(R50A)
    if isfield(AUX,'R50A'),R50A=AUX.R50A;else,R50A=struct();end %#ok<NASGU>
end

AF7R55B_PRIME_FIXED_CONTINUATION_REFERENCE(replanner,M);

CTX=af7r49_replay_context(replanner,E);
pkt=af7r34_make_core_packet(replanner,CTX,[],M);
SC0=E.speed_certificate;
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);

t=double(B49.t1(:).');
oldPass=logical(B49.success(:).');
d=diff([false oldPass false]);beg=find(d==1);fin=find(d==-1)-1;
term=find(fin==numel(oldPass),1,'last');assert(~isempty(term));
tU=t(fin(term));

% Fixed-domain theorem packet.
F=af7r55_fixed_continuation_mex(3,pkt,tU);
assert(logical(F.episode_ok)&&logical(F.tail_safe)&&double(F.tail_repr_fail)==0);
tailStart=double(F.tail_start);

% At tU the fixed planner must be mathematically identical to AF7R36,
% because both use exactly the episode built on [t0,tU].
P=pkt;P.t1=tU;
A=af7r36_single_core_mex(1,P);
B=af7r55_fixed_continuation_mex(4,pkt,tU,tU);
eqTU=compare_plan(A,B);

fprintf(' authority source                         = %s\n',authoritySource);
fprintf(' fixed domain tU [s]                      = %.15g\n',tU);
fprintf(' fixed tail start [s]                     = %.15g\n',tailStart);
fprintf(' tU fixed/original exact plan equality    = %d\n',eqTU);
assert(eqTU,'[AF7R55B] fixed synthesis differs from AF7R36 at tU.');

% Diagnostic lattice inherited from B49 only as a location aid.
use=find(t>tailStart & t<=tU);
N=numel(use);succ=false(1,N);reason=nan(1,N);speed=nan(1,N);
candCount=nan(1,N);fp1=cell(1,N);fp2=cell(1,N);
for ii=1:N
    P=af7r55_fixed_continuation_mex(4,pkt,tU,t(use(ii)));
    succ(ii)=logical(P.success);reason(ii)=double(P.reason_code);
    if succ(ii),speed(ii)=double(P.speed_uav);end
    candCount(ii)=double(P.all_candidate_count);
    if candCount(ii)>=1
        fp1{ii}=fingerprint(P.all_candidate_signs{1},P.all_candidate_active_set{1});
    end
    if candCount(ii)>=2
        fp2{ii}=fingerprint(P.all_candidate_signs{2},P.all_candidate_active_set{2});
    end
end

dd=diff([false succ false]);bb=find(dd==1);ff=find(dd==-1)-1;
terminal=find(ff==N,1,'last');
assert(~isempty(terminal),'[AF7R55B] fixed synthesis has no terminal feasible component reaching tU.');
iL=bb(terminal);iU=ff(terminal);

fprintf('\n[A. FIXED-SYNTHESIS DIAGNOSTIC TOPOLOGY]\n');
fprintf(' tested diagnostic lattice points         = %d\n',N);
fprintf(' feasible components on lattice           = %d\n',numel(bb));
for q=1:numel(bb)
    fprintf(' component %d [s]                         = [%.15g, %.15g] | points=%d\n', ...
        q,t(use(bb(q))),t(use(ff(q))),ff(q)-bb(q)+1);
end
fprintf(' terminal component lattice start [s]     = %.15g\n',t(use(iL)));
fprintf(' terminal component upper [s]             = %.15g\n',t(use(iU)));
fprintf(' predecessor [s]/reason                   = %.15g / %.0f\n', ...
    t(use(iL-1)),reason(iL-1));

% Candidate branch topology on the entire terminal component + predecessor.
jj=(iL-1):iU;
ncountUnique=unique(candCount(jj));
u1=unique(fp1(jj));
u2=unique(fp2(jj));
fprintf(' candidate-count values                   = %s\n',mat2str(ncountUnique));
fprintf(' candidate-1 fingerprints                 = %d\n',numel(u1));
fprintf(' candidate-2 fingerprints                 = %d\n',numel(u2));

% Refine only the local predecessor/pass interval. This remains diagnostic.
lo=t(use(iL-1));hi=t(use(iL));
fpTarget1=fp1{iL};fpTarget2=fp2{iL};
branchStable=true;
for it=1:64
    mid=0.5*(lo+hi);
    if ~(mid>lo&&mid<hi),break;end
    R=af7r55_fixed_continuation_mex(4,pkt,tU,mid);
    if double(R.all_candidate_count)~=candCount(iL)
        branchStable=false;break;
    end
    q1='';q2='';
    if double(R.all_candidate_count)>=1,q1=fingerprint(R.all_candidate_signs{1},R.all_candidate_active_set{1});end
    if double(R.all_candidate_count)>=2,q2=fingerprint(R.all_candidate_signs{2},R.all_candidate_active_set{2});end
    if ~strcmp(q1,fpTarget1)||~strcmp(q2,fpTarget2)
        branchStable=false;break;
    end
    if logical(R.success),hi=mid;else,lo=mid;end
end

RL=af7r55_fixed_continuation_mex(4,pkt,tU,lo);
RU=af7r55_fixed_continuation_mex(4,pkt,tU,hi);

rho=max([double(E.dependency_signature.r_drone), ...
    double(E.dependency_signature.cable_length)+double(E.dependency_signature.r_load), ...
    double(E.dependency_signature.cable_length)+double(E.dependency_signature.r_cable)]);
Vlim=(double(E.dependency_signature.sensor_range)-rho)/ ...
    (double(E.dependency_signature.horizon)+double(E.dependency_signature.dt));

fprintf('\n[B. FIXED-SYNTHESIS LOCAL BOUNDARY — DIAGNOSTIC ONLY]\n');
fprintf(' branch fingerprints stable in refine     = %d\n',branchStable);
fprintf(' lower / upper [s]                        = %.17g / %.17g\n',lo,hi);
fprintf(' bracket width [s]                        = %.17g\n',hi-lo);
fprintf(' lower success/reason                     = %d / %.0f\n',logical(RL.success),double(RL.reason_code));
fprintf(' upper success/reason                     = %d / %.0f\n',logical(RU.success),double(RU.reason_code));
fprintf(' lower selected finite witness [m/s]      = %.15g\n',double(RL.diagnostic_witness_max));
fprintf(' upper certified VQ [m/s]                 = %.15g\n',double(RU.speed_uav));
fprintf(' upper strict speed margin [m/s]          = %.17g\n',Vlim-double(RU.speed_uav));

fprintf('\n[C. FORMAL STATUS]\n');
fprintf(' fixed-domain episode theorem             = 1\n');
fprintf(' fixed synthesis equals AF7R36 at tU      = %d\n',eqTU);
fprintf(' fixed-corridor terminal component found  = 1\n');
fprintf(' local fixed-branch diagnostic bracket    = %d\n',branchStable);
fprintf(' Stage-B outward interval proof           = 0\n');
fprintf(' production t1 selected                   = 0\n');
fprintf(' next theorem                             = OUTWARD_STAGE_B_FOR_FIXED_SYNTHESIS_BOUNDARY\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R55B_FIXED_CORRIDOR_CONTINUATION_V1';
OUT.tU=tU;OUT.tail_start=tailStart;
OUT.tU_equivalence=eqTU;
OUT.diagnostic_t=t(use);
OUT.diagnostic_success=succ;
OUT.diagnostic_reason=reason;
OUT.terminal_component_lattice=[t(use(iL)) t(use(iU))];
OUT.boundary_bracket=[lo hi];
OUT.branch_stable=branchStable;
OUT.lower=RL;OUT.upper=RU;
OUT.production_t1_selected=false;
assignin('base','AF7R55B_FIXED_CORRIDOR_CONTINUATION_OUT',OUT);
end

function ok=compare_plan(A,B)
ok=isequal(logical(A.success),logical(B.success)) && ...
   isequal(double(A.reason_code),double(B.reason_code)) && ...
   isequal(double(A.component_start),double(B.component_start)) && ...
   isequal(double(A.component_end),double(B.component_end)) && ...
   isequal(double(A.segment_t0),double(B.segment_t0)) && ...
   isequal(double(A.segment_t1),double(B.segment_t1));
if ok&&logical(A.success)
    ok=ok&&isequal(double(A.corridor_signs),double(B.corridor_signs)) && ...
        maxabs(double(A.speed_uav)-double(B.speed_uav))==0 && ...
        maxabs(double(A.geometry_min_margin)-double(B.geometry_min_margin))==0 && ...
        cpdiff(A.control_points,B.control_points)==0;
end
end

function d=cpdiff(A,B)
if numel(A)~=numel(B),d=Inf;return;end
d=0;for k=1:numel(A),d=max(d,maxabs(double(A{k})-double(B{k})));end
end

function s=fingerprint(signs,active)
s=['S' mat2str(double(signs(:).')) '|A' mat2str(double(active(:).'))];
end

function y=maxabs(x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),y=0;else,y=max(abs(x));end
end
