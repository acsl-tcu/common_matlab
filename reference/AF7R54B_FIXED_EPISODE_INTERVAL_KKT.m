function OUT = AF7R54B_FIXED_EPISODE_INTERVAL_KKT(replanner,B49,R50A)
% Theorem-first continuation foundation.
%
% Instead of rebuilding the adaptive unsafe episode for every t1, this uses
% ONE fixed domain [t0,tU].  The unsafe episode E_U is computed once at the
% known-safe upper endpoint tU.  A separate adaptive continuous-time tube
% certificate proves that the nominal whole body is safe on
% (lastComponentEnd,tU].  Therefore for every t1 in the target continuation
% interval, no additional unsafe component can appear after E_U.
%
% The corridor rows generated from E_U are consequently a fixed synthesis
% authority over the continuation interval.  This removes the non-rigorous
% "endpoint rows are exactly equal" assumption from AF7R53B.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R54B | FIXED-DOMAIN EPISODE + OUTWARD INTERVAL KKT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | continuousTailCertificate=1 | fixedCorridorAuthority=1 | productionT1Selected=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r54_fixed_episode_mex','file')==3);
assert(exist('af7r54_interval_kkt_mex','file')==3);

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);
if nargin<2||isempty(B49)
    assert(isfield(AUX,'B49'),'[AF7R54B] B49 unavailable.'); B49=AUX.B49;
end
if nargin<3||isempty(R50A)
    if isfield(AUX,'R50A'), R50A=AUX.R50A; else, R50A=struct(); end %#ok<NASGU>
end

AF7R54B_PRIME_FIXED_EPISODE_REFERENCE(replanner,M);

CTX=af7r49_replay_context(replanner,E);
pkt=af7r34_make_core_packet(replanner,CTX,[],M);
SC0=E.speed_certificate;
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);

% Terminal lattice component only supplies an initial numerical enclosure.
t=double(B49.t1(:).'); pass=logical(B49.success(:).');
d=diff([false pass false]); beg=find(d==1); fin=find(d==-1)-1;
term=find(fin==numel(pass),1,'last');
assert(~isempty(term)&&beg(term)>1);
kL=beg(term);
tFail=t(kL-1); tPass=t(kL); tU=t(fin(term));

% Machine bracket from the unchanged AF7R36 authority.
lo=tFail; hi=tPass;
for it=1:64
    mid=0.5*(lo+hi);
    if ~(mid>lo&&mid<hi), break; end
    P=pkt; P.t1=mid;
    R=af7r36_single_core_mex(1,P);
    if logical(R.success), hi=mid; else, lo=mid; end
end

% Fixed-domain episode and rigorous continuous tail theorem.
F=af7r54_fixed_episode_mex(3,pkt,tU);
U=F.plan;

fprintf(' authority source                         = %s\n',authoritySource);
fprintf(' fixed episode domain                     = [%.15g, %.15g] s\n',double(U.t0),tU);
fprintf(' last unsafe component end [s]            = %.15g\n',double(F.last_component_end));
fprintf(' certified tail start [s]                 = %.17g\n',double(F.tail_start));
fprintf(' tail safe continuous certificate         = %d\n',logical(F.tail_safe));
fprintf(' tail adaptive leaves/accepted/splits     = %.0f / %.0f / %.0f\n', ...
    double(F.tail_leaves),double(F.tail_accepted),double(F.tail_splits));
fprintf(' tail binary64 representability failures  = %.0f\n',double(F.tail_repr_fail));
fprintf(' tail minimum static-speed margin [m]     = %.15g\n',double(F.tail_min_margin));
fprintf(' numerical boundary bracket [s]           = [%.17g, %.17g]\n',lo,hi);
fprintf(' bracket lower > certified tail start     = %d\n',lo>double(F.tail_start));

freezeClosed=logical(F.episode_ok)&&logical(F.tail_safe)&& ...
    double(F.tail_repr_fail)==0 && isfinite(double(F.tail_min_margin)) && ...
    double(F.tail_min_margin)>0 && lo>double(F.tail_start);

% The fixed plan at tU provides the corridor sign candidates, rows and active
% sets. These rows are now intentionally kept FIXED for continuation.
n=double(U.all_candidate_count);
fprintf('\n[A. FIXED CORRIDOR AUTHORITY]\n');
fprintf(' fixed-domain episode theorem closed      = %d\n',freezeClosed);
fprintf(' sign candidate count                     = %.0f\n',n);
fprintf(' fixed component count                    = %.0f\n',double(U.component_count));

allKKT=freezeClosed && n>=1;
K=cell(1,n);
for c=1:n
    sg=double(U.all_candidate_signs{c}(:).');
    ac0=double(U.all_candidate_active_set{c}(:).'); % native is 0-based
    A=double(U.all_candidate_rows_A{c});
    b=double(U.all_candidate_rows_b{c});

    fprintf(' candidate %.0f sign                      = %s\n',c,mat2str(sg));
    fprintf(' candidate %.0f native active set (0base) = %s\n',c,mat2str(ac0));
    fprintf(' candidate %.0f fixed row count           = %.0f\n',c,size(A,1));

    if numel(ac0)==1 && double(U.component_count)==1
        activeMatlab=ac0(1)+1; % convert native 0-based -> MATLAB/C API 1-based input
        signTag=0;if ~isempty(sg),signTag=sg(1);end
        K{c}=af7r54_interval_kkt_mex( ...
            lo,hi,double(U.t0),double(U.component_start(1)), ...
            double(U.component_end(1)),A,b,activeMatlab,signTag,c);

        fprintf(' candidate %.0f interval SPD              = %d\n',c,logical(K{c}.spd));
        fprintf(' candidate %.0f lambda interval            = [%.17g %.17g]\n', ...
            c,K{c}.lambda(1),K{c}.lambda(2));
        fprintf(' candidate %.0f inactive slack LB          = %.17g\n', ...
            c,double(K{c}.min_inactive_slack_lb));
        fprintf(' candidate %.0f interval KKT closed        = %d\n',c,logical(K{c}.closed));
        allKKT=allKKT&&logical(K{c}.closed);
    else
        allKKT=false;
        fprintf(' candidate %.0f interval KKT closed        = 0 (unsupported active/component cardinality)\n',c);
    end
end

fprintf('\n[B. KKT GATE]\n');
fprintf(' fixed-domain episode theorem closed      = %d\n',freezeClosed);
fprintf(' all sign-candidate interval KKT closed   = %d\n',allKKT);

% Safety authority is deliberately NOT promoted here.
rho=max([double(E.dependency_signature.r_drone), ...
    double(E.dependency_signature.cable_length)+double(E.dependency_signature.r_load), ...
    double(E.dependency_signature.cable_length)+double(E.dependency_signature.r_cable)]);
Vlim=(double(E.dependency_signature.sensor_range)-rho)/ ...
    (double(E.dependency_signature.horizon)+double(E.dependency_signature.dt));

PL=pkt;PL.t1=lo;RL=af7r36_single_core_mex(1,PL);
PU=pkt;PU.t1=hi;RU=af7r36_single_core_mex(1,PU);

fprintf('\n[C. EXISTING AF7R36 CONTINUOUS AUTHORITY AT BRACKET]\n');
fprintf(' lower success/reason                     = %d / %.0f\n',logical(RL.success),double(RL.reason_code));
fprintf(' upper success/reason                     = %d / %.0f\n',logical(RU.success),double(RU.reason_code));
fprintf(' finite-sensing VQ limit [m/s]            = %.15g\n',Vlim);
fprintf(' upper certified VQ [m/s]                 = %.15g\n',double(RU.speed_uav));
fprintf(' upper strict speed margin [m/s]          = %.17g\n',Vlim-double(RU.speed_uav));

fprintf('\n[D. FORMAL STATUS]\n');
fprintf(' fixed episode/corridor invariance closed = %d\n',freezeClosed);
fprintf(' all-candidate KKT interval closed        = %d\n',allKKT);
fprintf(' Stage-B interval propagation closed      = 0\n');
fprintf(' continuous terminal-boundary proof       = 0\n');
fprintf(' production t1 selected                   = 0\n');
fprintf(' unresolved interval policy               = FAIL CLOSED\n');
fprintf(' next theorem                             = INTERVAL_STAGE_B_ON_FIXED_EPISODE_KKT\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R54B_FIXED_EPISODE_INTERVAL_KKT_V1';
OUT.boundary_bracket=[lo hi];
OUT.fixed_episode_closed=freezeClosed;
OUT.tail_start=double(F.tail_start);
OUT.tail_min_margin=double(F.tail_min_margin);
OUT.candidate_count=n;
OUT.all_candidate_kkt_closed=allKKT;
OUT.kkt=K;
OUT.stageB_interval_closed=false;
OUT.production_t1_selected=false;
assignin('base','AF7R54B_FIXED_EPISODE_INTERVAL_KKT_OUT',OUT);
end
