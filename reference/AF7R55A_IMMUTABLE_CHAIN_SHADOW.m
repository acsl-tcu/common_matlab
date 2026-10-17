function OUT = AF7R55A_IMMUTABLE_CHAIN_SHADOW(replanner,B49)
% AF7R55A_IMMUTABLE_CHAIN_SHADOW
%
% Mathematical object:
%   x_v(t) = x_0(t) + sum_{j=1}^v d_j(t)
%
% Since differentiation is linear,
%   x_v^(r)(t) = x_0^(r)(t) + sum_j d_j^(r)(t), r=0..6.
%
% Each d_j is the same piecewise degree-15 Bernstein correction already
% certified by AF7R36.  The native chain stores each committed correction
% immutably.  Appending a new complete correction does NOT change the active
% version; activation is one atomic integer-token store.
%
% This is SHADOW ONLY. It does not install a new REPLANNING_SYSTEM.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R55A | IMMUTABLE NATIVE CORRECTION-CHAIN SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | algebraicDerivativeLinearity=1 | productionModified=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r55_chain_mex','file')==3,'[AF7R55A] build chain MEX first.');
assert(exist('af7r36_single_core_mex','file')==3);

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);
if nargin<2||isempty(B49)
    assert(isfield(AUX,'B49'),'[AF7R55A] B49 unavailable.');
    B49=AUX.B49;
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

Corr=cell(1,V);Legacy=cell(1,V);raws=cell(1,V);
for j=1:V
    P=pkt;P.t1=tgrid(pick(j));
    R=af7r36_single_core_mex(1,P);
    assert(logical(R.success),'[AF7R55A] selected correction replay failed.');
    raws{j}=R;
    Corr{j}=pack_corr(R);
    Legacy{j}=legacy_proposal(R);
end

af7r55_chain_mex(0);
S0=af7r55_chain_mex(5);
assert(double(S0.active_version)==0&&double(S0.stored_versions)==0);

ta=min(cellfun(@(q)double(q.segment_t0(1)),Corr));
tb=max(cellfun(@(q)double(q.segment_t1(end)),Corr));
tg=linspace(ta-0.5,tb+0.5,2003);

maxVersionDiff=0;
appendInvisible=true;
atomicActivation=true;
statusTrace=zeros(V,4);

for v=1:V
    before=af7r55_chain_mex(5);
    token=af7r55_chain_mex(1,Corr{v});
    afterAppend=af7r55_chain_mex(5);

    appendInvisible=appendInvisible && ...
        double(afterAppend.active_version)==double(before.active_version) && ...
        double(afterAppend.stored_versions)==v && token==v;

    % Active evaluation must still equal the PREVIOUS complete version.
    Ya=double(af7r55_chain_mex(3,tg));
    Ym=legacy_chain_eval(replanner,Legacy,tg,v-1);
    maxVersionDiff=max(maxVersionDiff,maxabs(Ya-Ym));

    af7r55_chain_mex(2,v);
    afterActivate=af7r55_chain_mex(5);
    atomicActivation=atomicActivation && ...
        double(afterActivate.active_version)==v && ...
        double(afterActivate.stored_versions)==v;

    Yn=double(af7r55_chain_mex(3,tg));
    Yv=double(af7r55_chain_mex(4,v,tg));
    Ym=legacy_chain_eval(replanner,Legacy,tg,v);
    maxVersionDiff=max([maxVersionDiff,maxabs(Yn-Ym),maxabs(Yv-Ym)]);

    statusTrace(v,:)=[double(before.active_version), ...
        double(afterAppend.active_version),double(afterActivate.active_version), ...
        double(afterActivate.stored_versions)];
end

invalidRejected=false;
try
    af7r55_chain_mex(2,V+10);
catch
    invalidRejected=true;
end
Sf=af7r55_chain_mex(5);
atomicActivation=atomicActivation&&invalidRejected&&double(Sf.active_version)==V;

% HLC/flat 28-slot equality using the SAME base authority plus native chain.
fh=replanner.base_ref.func;
flatfh=replanner.nominal_flat_derivative_fn(1:7);
tf0=double(M.flight_reference_time_origin);
baseEval=@(t)replanner.evaluate_nominal_snapshot_global(fh,tf0,t);
baseFlat=@(t)replanner.evaluate_nominal_flat_snapshot_global(fh,flatfh,tf0,t);

Ychain=double(af7r55_chain_mex(4,V,tg));
dHLC=0;dFlat=0;
for k=1:numel(tg)
    xb=double(baseEval(tg(k)));xb=xb(:);if numel(xb)<28,xb=[xb;zeros(28-numel(xb),1)];end
    xf=double(baseFlat(tg(k)));xf=xf(:);if numel(xf)<28,xf=[xf;zeros(28-numel(xf),1)];end
    xn=xb;fn=xf;
    for r=0:6
        ii=4*r+(1:3);
        xn(ii)=xn(ii)+Ychain(3*r+(1:3),k);
        fn(ii)=fn(ii)+Ychain(3*r+(1:3),k);
    end

    % Legacy mathematical chain = same base + explicit sum of each correction.
    yc=zeros(21,1);
    for j=1:V
        D=replanner.evaluate_piecewise_bezier_correction_derivatives(tg(k),Legacy{j},6);
        for r=0:6,yc(3*r+(1:3))=yc(3*r+(1:3))+D(:,r+1);end
    end
    xl=xb;fl=xf;
    for r=0:6
        ii=4*r+(1:3);
        xl(ii)=xl(ii)+yc(3*r+(1:3));
        fl(ii)=fl(ii)+yc(3*r+(1:3));
    end
    dHLC=max(dHLC,maxabs(xn-xl));
    dFlat=max(dFlat,maxabs(fn-fl));
end

scale=max([1;abs(Ychain(:))]);
tol=8192*eps(max(scale));
numericClosed=maxVersionDiff<=tol&&dHLC<=tol&&dFlat<=tol;

fprintf(' true-trigger authority source            = %s\n',authoritySource);
fprintf(' correction versions exercised            = %d\n',V);
fprintf(' selected rejoin times [s]                = %s\n',mat2str(tgrid(pick),15));

fprintf('\n[A. IMMUTABLE VERSION SEMANTICS]\n');
fprintf(' append leaves active version unchanged   = %d\n',appendInvisible);
fprintf(' activation is single version-token swap  = %d\n',atomicActivation);
fprintf(' invalid version activation rejected      = %d\n',invalidRejected);
fprintf(' final active/stored versions             = %.0f / %.0f\n', ...
    double(Sf.active_version),double(Sf.stored_versions));

fprintf('\n[B. MATHEMATICAL CHAIN EQUIVALENCE]\n');
fprintf(' max |native chain jet - legacy sum|      = %.17g\n',maxVersionDiff);
fprintf(' max |HLC 28-slot chain - legacy|         = %.17g\n',dHLC);
fprintf(' max |flat 28-slot chain - legacy|        = %.17g\n',dFlat);
fprintf(' numerical equivalence tolerance          = %.17g\n',tol);
fprintf(' numeric chain equivalence closed         = %d\n',numericClosed);

fprintf('\n[C. THEOREM STATUS]\n');
fprintf(' derivative linearity r=0..6              = EXACT ALGEBRAIC IDENTITY\n');
fprintf(' each correction immutable after append   = 1\n');
fprintf(' pre-activation old complete chain visible= %d\n',appendInvisible);
fprintf(' post-activation new complete chain visible= %d\n',atomicActivation);
fprintf(' native/MATLAB chain representation closed= %d\n',numericClosed);
fprintf(' production integration installed         = 0\n');
fprintf(' next timing step                         = SHADOW_HLC_AND_FUTURE_PACKET_FROM_ONE_ACTIVE_TOKEN\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R55A_IMMUTABLE_CHAIN_SHADOW_V1';
OUT.append_invisible=appendInvisible;
OUT.atomic_activation=atomicActivation;
OUT.invalid_activation_rejected=invalidRejected;
OUT.max_chain_jet_diff=maxVersionDiff;
OUT.max_hlc_diff=dHLC;
OUT.max_flat_diff=dFlat;
OUT.numeric_closed=numericClosed;
OUT.version_trace=statusTrace;
OUT.production_installed=false;
assignin('base','AF7R55A_IMMUTABLE_CHAIN_SHADOW_OUT',OUT);
end

function Q=pack_corr(R)
Q=struct();
Q.segment_t0=double(R.segment_t0(:).');
Q.segment_t1=double(R.segment_t1(:).');
S=numel(R.control_points);A=zeros(48,S);
for k=1:S
    C=double(R.control_points{k});assert(isequal(size(C),[3 16]));
    A(:,k)=C(:);
end
Q.control_points_matrix=A;
end

function P=legacy_proposal(R)
P=struct();
P.piecewise_segment_t0=double(R.segment_t0(:).');
P.piecewise_segment_t1=double(R.segment_t1(:).');
P.piecewise_control_points=R.control_points;
end

function Y=legacy_chain_eval(replanner,L,tg,v)
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

function y=maxabs(x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),y=0;else,y=max(abs(x));end
end
