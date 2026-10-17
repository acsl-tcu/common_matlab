function OUT = AF7R56A_SINGLE_TOKEN_CYCLE_SHADOW(replanner,B49)
% Prove the cycle-level authority rule required before any timing integration:
%
%   nu = snapshot(active_version)       exactly once
%   HLC current packet  = Eval(nu,t)
%   future/sensing jet  = Eval(nu,Tfuture)
%
% Even if a complete new version is activated after nu is captured, both
% consumers remain on the same immutable version nu for that cycle.
%
% This is SHADOW ONLY.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R56A | SINGLE ACTIVE-TOKEN HLC / FUTURE-PACKET SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | oneTokenPerCycle=1 | productionModified=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r56_chain_mex','file')==3);

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);
if nargin<2||isempty(B49)
    assert(isfield(AUX,'B49'),'[AF7R56A] B49 unavailable.');B49=AUX.B49;
end

CTX=af7r49_replay_context(replanner,E);
pkt=af7r34_make_core_packet(replanner,CTX,[],M);
SC0=E.speed_certificate;
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);

tgrid=double(B49.t1(:).');pass=logical(B49.success(:).');
d=diff([false pass false]);beg=find(d==1);fin=find(d==-1)-1;
term=find(fin==numel(pass),1,'last');assert(~isempty(term));
safeIdx=beg(term):fin(term);
pick=unique([safeIdx(1),safeIdx(round((numel(safeIdx)+1)/2)),safeIdx(end)]);
V=numel(pick);

Corr=cell(1,V);Legacy=cell(1,V);
for j=1:V
    P=pkt;P.t1=tgrid(pick(j));
    R=af7r36_single_core_mex(1,P);
    assert(logical(R.success));
    Corr{j}=pack_corr(R);
    Legacy{j}=legacy_proposal(R);
end

af7r56_chain_mex(0);
for j=1:V
    q=af7r56_chain_mex(1,Corr{j});
    assert(double(q)==j);
end
S=af7r56_chain_mex(5);
assert(double(S.active_version)==0&&double(S.stored_versions)==V);

% Base reference authority.
fh=replanner.base_ref.func;
flatfh=replanner.nominal_flat_derivative_fn(1:7);
tf0=double(M.flight_reference_time_origin);
baseEval=@(t)replanner.evaluate_nominal_snapshot_global(fh,tf0,t);
baseFlat=@(t)replanner.evaluate_nominal_flat_snapshot_global(fh,flatfh,tf0,t);

% One current HLC time and a future packet vector.  Values are arbitrary
% evaluation locations inside the certified source domain; no safety
% acceptance is based on them.
tnow=double(E.event_time)+0.5;
tfuture=tnow+(0:double(E.dependency_signature.dt):double(E.dependency_signature.horizon));
tfuture=tfuture(1:min(numel(tfuture),41));

maxHLC=0;maxFuture=0;
snapshotStable=true;activeCanAdvance=true;
trace=zeros(max(V-1,1),4);

af7r56_chain_mex(2,1);
for nu=1:V
    st=af7r56_chain_mex(5);
    assert(double(st.active_version)==nu);
    snap=double(st.active_version);

    % Simulate a commit after the cycle token has been captured.
    if nu<V
        af7r56_chain_mex(2,nu+1);
        st2=af7r56_chain_mex(5);
        activeCanAdvance=activeCanAdvance&&(double(st2.active_version)==nu+1);
    else
        st2=st;
    end

    % Both consumers use the explicit SNAPSHOT token, never a second active read.
    Ynow=double(af7r56_chain_mex(4,snap,tnow));
    Yfuture=double(af7r56_chain_mex(4,snap,tfuture));

    [XL,FL]=legacy_packets(replanner,Legacy,baseEval,baseFlat,tnow,tfuture,snap);
    XN=compose_packet(baseEval(tnow),Ynow(:,1));
    FN=zeros(28,numel(tfuture));
    for k=1:numel(tfuture)
        FN(:,k)=compose_packet(baseFlat(tfuture(k)),Yfuture(:,k));
    end

    maxHLC=max(maxHLC,maxabs(XN-XL));
    maxFuture=max(maxFuture,maxabs(FN-FL));

    % After activation of a newer version, explicit evaluation of snap must
    % remain bitwise/numerically identical to the old complete version.
    Yagain=double(af7r56_chain_mex(4,snap,[tnow tfuture(end)]));
    Ylegacy=legacy_jet(replanner,Legacy,[tnow tfuture(end)],snap);
    snapshotStable=snapshotStable&&(maxabs(Yagain-Ylegacy)==0);

    if nu<V
        trace(nu,:)=[snap,double(st2.active_version),maxabs(XN-XL),maxabs(FN-FL)];
    end

    % Prepare next loop: active version is already nu+1 after the simulated
    % commit. For final version nothing changes.
end

tol=8192*eps(max([1,maxHLC,maxFuture]));
closed=snapshotStable&&activeCanAdvance&&maxHLC<=tol&&maxFuture<=tol;

fprintf(' true-trigger authority source            = %s\n',authoritySource);
fprintf(' immutable versions stored                = %d\n',V);
fprintf(' selected rejoin times [s]                = %s\n',mat2str(tgrid(pick),15));

fprintf('\n[A. ONE-TOKEN CYCLE SEMANTICS]\n');
fprintf(' active version may advance after snapshot= %d\n',activeCanAdvance);
fprintf(' explicit snapshot remains immutable      = %d\n',snapshotStable);
fprintf(' HLC and future packet use same token     = 1\n');
fprintf(' second active-token read required        = 0\n');

fprintf('\n[B. PACKET EQUIVALENCE]\n');
fprintf(' max |HLC current packet - legacy|        = %.17g\n',maxHLC);
fprintf(' max |future flat packet - legacy|        = %.17g\n',maxFuture);
fprintf(' tolerance                               = %.17g\n',tol);
fprintf(' single-token theorem shadow closed       = %d\n',closed);

fprintf('\n[C. PRODUCTION GATE]\n');
fprintf(' production integration installed         = 0\n');
fprintf(' next timing implementation               = PREALLOCATED_BUNDLE_PLUS_SINGLE_TOKEN_SHADOW\n');
fprintf(' hard 4 ms acceptance modified            = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R56A_SINGLE_TOKEN_CYCLE_SHADOW_V1', ...
    'closed',closed,'snapshot_stable',snapshotStable, ...
    'active_can_advance',activeCanAdvance,'max_hlc_diff',maxHLC, ...
    'max_future_diff',maxFuture,'trace',trace,'production_installed',false);
assignin('base','AF7R56A_SINGLE_TOKEN_CYCLE_SHADOW_OUT',OUT);
end

function Q=pack_corr(R)
Q=struct();Q.segment_t0=double(R.segment_t0(:).');Q.segment_t1=double(R.segment_t1(:).');
S=numel(R.control_points);A=zeros(48,S);
for k=1:S,C=double(R.control_points{k});A(:,k)=C(:);end
Q.control_points_matrix=A;
end
function P=legacy_proposal(R)
P=struct('t0',double(R.t0),'t1',double(R.t1), ...
    'piecewise_segment_t0',double(R.segment_t0(:).'), ...
    'piecewise_segment_t1',double(R.segment_t1(:).'), ...
    'piecewise_control_points',{R.control_points});
end
function [X,F]=legacy_packets(replanner,L,baseEval,baseFlat,tnow,tf,version)
Y=legacy_jet(replanner,L,[tnow tf],version);
X=compose_packet(baseEval(tnow),Y(:,1));
F=zeros(28,numel(tf));
for k=1:numel(tf),F(:,k)=compose_packet(baseFlat(tf(k)),Y(:,k+1));end
end
function Y=legacy_jet(replanner,L,tg,v)
Y=zeros(21,numel(tg));
for k=1:numel(tg)
    y=zeros(21,1);
    for j=1:v
        D=replanner.evaluate_piecewise_bezier_correction_derivatives(tg(k),L{j},6);
        for r=0:6,y(3*r+(1:3))=y(3*r+(1:3))+D(:,r+1);end
    end
    Y(:,k)=y;
end
end
function x=compose_packet(base,y)
x=double(base(:));if numel(x)<28,x=[x;zeros(28-numel(x),1)];end
for r=0:6,ii=4*r+(1:3);x(ii)=x(ii)+y(3*r+(1:3));end
x=x(1:28);
end
function y=maxabs(x)
x=double(x(:));x=x(isfinite(x));if isempty(x),y=0;else,y=max(abs(x));end
end
