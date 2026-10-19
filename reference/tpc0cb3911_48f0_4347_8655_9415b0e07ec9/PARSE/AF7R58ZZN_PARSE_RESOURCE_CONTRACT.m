function C = AF7R58ZZN_PARSE_RESOURCE_CONTRACT(replanner)
% AF7R58I source-derived finite active-resource theorem.
% No obstacle geometry/count, physical parameter, actuator limit, or mL value is hard-coded.
assert(isa(replanner,'REPLANNING_SYSTEM'));
C=struct('closed',false,'reason','', ...
    'flight_t0',NaN,'reference_tend',NaN,'dt',NaN, ...
    'Tcorr_max',NaN,'Dcommit_min',NaN,'Kmax',NaN, ...
    'Cmax',NaN,'Sgroup_max',NaN,'Gmax',NaN,'Smax',NaN, ...
    'theorem','AF7R58E_OUTWARD_TREE_PLUS_FINITE_LIFETIME_AND_EXPLICIT_COMMIT_SPACING');

tf0=double(replanner.flight_reference_time_origin);
tend=double(replanner.nominal_table_tend);
dt=double(replanner.prediction_dt);

% AF7R58ZZN_R2_RESOURCE_CONTRACT_MEMO
% Exact theorem memoization only. The current theorem consumes no
% replanner value other than {tf0,tend,dt}. A key mismatch executes
% the original theorem below without modification.
persistent AF7R58ZZN_R2_KEY AF7R58ZZN_R2_CLOSED_CONTRACT
af7r58zzn_r2_key=[tf0 tend dt];
if ~isempty(AF7R58ZZN_R2_KEY) && ...
        isequal(AF7R58ZZN_R2_KEY,af7r58zzn_r2_key) && ...
        isstruct(AF7R58ZZN_R2_CLOSED_CONTRACT) && ...
        isfield(AF7R58ZZN_R2_CLOSED_CONTRACT,'closed') && ...
        logical(AF7R58ZZN_R2_CLOSED_CONTRACT.closed)
    C=AF7R58ZZN_R2_CLOSED_CONTRACT;
    return;
end
if ~(all(isfinite([tf0 tend dt]))&&tend>tf0&&dt>0)
    C.reason='LIVE_TIME_AUTHORITY_OPEN';
    return;
end

M=max([1 abs(tf0) abs(tend) abs(dt)]);
tol=64*eps(M);
loss=tol+4*eps(M);
d0=dt-loss;
if ~(isfinite(d0)&&d0>0)
    C.reason='DCOMMIT_LOWER_BOUND_OPEN';
    return;
end
Dmin=af7r58i_nextdown_positive(d0);
if ~(isfinite(Dmin)&&Dmin>0)
    C.reason='DCOMMIT_LOWER_BOUND_OPEN';
    return;
end

span=tend-tf0;
if ~(isfinite(span)&&span>0)
    C.reason='TCORR_SPAN_OPEN';
    return;
end
Tcorr=af7r58i_nextup_nonnegative(span);

eta=2*eps(M);
W=af7r58i_nextup_nonnegative(span);
K=0;
while af7r58i_nextup_nonnegative(W)>dt
    V=af7r58i_nextup_nonnegative(af7r58i_nextup_nonnegative(W/2)+eta);
    if ~(isfinite(V)&&V<W)
        C.reason='TREE_WIDTH_RECURRENCE_NOT_CONTRACTING';
        return;
    end
    W=V;
    K=K+1;
    if K>52
        C.reason='TREE_COUNT_SYMBOLIC_ONLY';
        return;
    end
end

Cmax=2^max(K-1,0);
Sgroup=2*Cmax+1;
if ~(Cmax<=flintmax && Sgroup<=flintmax)
    C.reason='GROUP_SEGMENT_COUNT_NOT_EXACT';
    return;
end

ratio=af7r58i_nextup_nonnegative(Tcorr/Dmin);
Gmax=ceil(ratio)+1;
if ~(isfinite(Gmax)&&Gmax>=1&&Gmax<=flintmax)
    C.reason='GMAX_NOT_EXACT';
    return;
end
if Gmax>floor(flintmax/Sgroup)
    C.reason='SMAX_NOT_EXACT';
    return;
end
Smax=Gmax*Sgroup;

C.closed=true;
C.reason='RESOURCE_CONTRACT_CLOSED';
C.flight_t0=tf0;
C.reference_tend=tend;
C.dt=dt;
C.Tcorr_max=Tcorr;
C.Dcommit_min=Dmin;
C.Kmax=K;
C.Cmax=Cmax;
C.Sgroup_max=Sgroup;
C.Gmax=Gmax;
C.Smax=Smax;

% Store only a CLOSED theorem packet. Open/failed theorem evaluations
% are never memoized.
AF7R58ZZN_R2_KEY=af7r58zzn_r2_key;
AF7R58ZZN_R2_CLOSED_CONTRACT=C;
end