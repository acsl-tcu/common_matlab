% AF7R20_R8_LOCAL_VECTOR_SPEED_FORENSIC.m
% =========================================================================
% R8 piecewise-C7 local vector-coupled suspended-load speed forensic.
%
% Run this SCRIPT while MATLAB is paused inside
% REPLANNING_SYSTEM/builtin_bezier_d15_c7_proposal after CANDS, OUT,
% baseSC, VQlimit and CTX exist (e.g. around the old speed-certificate loop).
%
% This is READ-ONLY / DIAGNOSTIC ONLY.
% It does NOT modify REPLANNING_SYSTEM and does NOT accept a trajectory.
%
% Continuous-time enclosure on each adaptive leaf I=[ta,tb]:
%
%   pL_r = pL_base + d
%   s    = pL_r_ddot + g e3
%   q    = -s/||s||
%   pQ_r = pL_r - L q
%
% Base velocity is anchored at the leaf midpoint and enclosed with the
% certified base acceleration bound:
%
%   ||v_base(t)-v_base(tc)|| <= A_base * h.
%
% The correction velocity is bounded directly as a VECTOR Bernstein curve,
% so the old scalar loss
%
%   ||v_base|| + ||d_dot||
%
% is avoided.
%
% Specific-force lower bound uses a midpoint anchor plus the certified total
% jerk bound over the leaf:
%
%   ||s(t)|| >= ||s(tc)|| - (J_base + ||d_dddot||_UB) h.
%
% Then
%
%   ||qdot|| <= J_r_UB / sigma_LB,
%   ||vQ||   <= VL_r_UB + L ||qdot||.
%
% Acceptance in this forensic is by continuous-time upper bounds only.
% Midpoint values are LOWER witnesses for rejection/diagnosis only.
% There is no fixed subdivision-depth acceptance cap. If binary64 can no
% longer represent a strict midpoint, the result is INCONCLUSIVE.
% =========================================================================

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R20 | R8 LOCAL VECTOR-COUPLED SUSPENDED-LOAD SPEED FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | continuousUpperEnclosure=1 | sampledAcceptance=0 | fixedDepthCap=0\n');

need = {'OUT','CANDS','baseSC','VQlimit','CTX'};
for k20 = 1:numel(need)
    assert(exist(need{k20},'var')==1, ...
        '[AF7R20] Missing paused-workspace variable: %s',need{k20});
end

assert(isstruct(OUT) && isfield(OUT,'base_flat_eval_fn') && ...
    isa(OUT.base_flat_eval_fn,'function_handle'), ...
    '[AF7R20] OUT.base_flat_eval_fn unavailable.');
assert(isstruct(baseSC) && isfield(baseSC,'certified') && ...
    logical(baseSC.certified), ...
    '[AF7R20] baseSC is not certified.');
assert(isstruct(CTX) && isfield(CTX,'live'), ...
    '[AF7R20] CTX.live unavailable.');

baseEval20 = OUT.base_flat_eval_fn;
A0_20 = double(baseSC.payload_accel_UB_m_s2);
J0_20 = double(baseSC.payload_jerk_UB_m_s3);
g20 = double(CTX.live.gravity);
L20 = double(CTX.live.cable_length);
lim20 = double(VQlimit);

assert(all(isfinite([A0_20,J0_20,g20,L20,lim20])) && ...
    A0_20>=0 && J0_20>=0 && g20>0 && L20>0 && lim20>0, ...
    '[AF7R20] Invalid live/base constants.');

N20 = numel(CANDS);
fprintf(' candidate count                         = %d\n',N20);
fprintf(' VQ finite-sensing limit [m/s]           = %.17g\n',lim20);
fprintf(' base accel / jerk UB                    = %.17g / %.17g\n',A0_20,J0_20);
fprintf(' cable length / gravity                  = %.17g / %.17g\n',L20,g20);

RES20 = repmat(struct( ...
    'candidate_index',NaN, ...
    'certified',false, ...
    'finite_sensing_speed_violation_witness',false, ...
    'representability_open',false, ...
    'split_count',0, ...
    'leaf_count',0, ...
    'VQ_UB_m_s',NaN, ...
    'VQ_midpoint_LB_witness_m_s',NaN, ...
    'payload_speed_UB_m_s',NaN, ...
    'payload_accel_UB_m_s2',NaN, ...
    'payload_jerk_UB_m_s3',NaN, ...
    'specific_force_norm_LB_m_s2',NaN, ...
    'qdot_UB_rad_s',NaN, ...
    'old_raw_d1_ub',NaN, ...
    'old_raw_d2_ub',NaN, ...
    'old_raw_d3_ub',NaN, ...
    'classification',''),1,N20);

for ic20 = 1:N20
    Q20 = CANDS(ic20);

    assert(isfield(Q20,'segment_t0') && isfield(Q20,'segment_t1') && ...
        isfield(Q20,'control_points'), ...
        '[AF7R20] candidate %d lacks piecewise representation.',ic20);

    s0_20 = double(Q20.segment_t0(:));
    s1_20 = double(Q20.segment_t1(:));
    CP20 = Q20.control_points;
    assert(numel(s0_20)==numel(s1_20) && numel(CP20)==numel(s0_20), ...
        '[AF7R20] candidate %d segment arrays mismatch.',ic20);

    LEAF20 = repmat(local_empty_leaf20(),1,0);
    for is20 = 1:numel(CP20)
        c20 = double(CP20{is20});
        assert(isequal(size(c20),[3 16]) && all(isfinite(c20(:))), ...
            '[AF7R20] candidate %d segment %d has invalid control points.',ic20,is20);
        lf20 = local_make_leaf20(c20,s0_20(is20),s1_20(is20), ...
            is20,baseEval20,A0_20,J0_20,g20,L20,lim20);
        LEAF20(end+1)=lf20; %#ok<SAGROW>
    end

    split20 = 0;
    cert20 = false;
    witnessFail20 = false;
    reprOpen20 = false;

    while true
        ubv20 = [LEAF20.vq_ub];
        lbv20 = [LEAF20.vq_mid_witness];

        if all(isfinite(ubv20)) && all(ubv20 < lim20)
            cert20 = true;
            break;
        end

        if any(isfinite(lbv20) & lbv20 >= lim20)
            witnessFail20 = true;
            break;
        end

        unresolved20 = find(~isfinite(ubv20) | ubv20 >= lim20);
        if isempty(unresolved20)
            cert20 = true;
            break;
        end

        score20 = ubv20(unresolved20) - lim20;
        score20(~isfinite(score20)) = inf;
        [~,kk20] = max(score20);
        il20 = unresolved20(kk20);
        old20 = LEAF20(il20);

        tm20 = 0.5*(old20.ta+old20.tb);
        if ~(isfinite(tm20) && tm20>old20.ta && tm20<old20.tb)
            reprOpen20 = true;
            break;
        end

        [cL20,cR20] = local_split_half20(old20.C0);
        left20 = local_make_leaf20(cL20,old20.ta,tm20,old20.segment_index, ...
            baseEval20,A0_20,J0_20,g20,L20,lim20);
        right20 = local_make_leaf20(cR20,tm20,old20.tb,old20.segment_index, ...
            baseEval20,A0_20,J0_20,g20,L20,lim20);

        LEAF20(il20) = left20;
        LEAF20(end+1) = right20; %#ok<SAGROW>
        split20 = split20 + 1;
    end

    ubv20 = [LEAF20.vq_ub];
    lbv20 = [LEAF20.vq_mid_witness];
    vl20 = [LEAF20.vl_ub];
    al20 = [LEAF20.al_ub];
    jl20 = [LEAF20.jl_ub];
    sg20 = [LEAF20.sigma_lb];
    qd20 = [LEAF20.qdot_ub];

    RES20(ic20).candidate_index = ic20;
    RES20(ic20).certified = cert20;
    RES20(ic20).finite_sensing_speed_violation_witness = witnessFail20;
    RES20(ic20).representability_open = reprOpen20;
    RES20(ic20).split_count = split20;
    RES20(ic20).leaf_count = numel(LEAF20);
    RES20(ic20).VQ_UB_m_s = local_finite_max20(ubv20);
    RES20(ic20).VQ_midpoint_LB_witness_m_s = local_finite_max20(lbv20);
    RES20(ic20).payload_speed_UB_m_s = local_finite_max20(vl20);
    RES20(ic20).payload_accel_UB_m_s2 = local_finite_max20(al20);
    RES20(ic20).payload_jerk_UB_m_s3 = local_finite_max20(jl20);
    RES20(ic20).specific_force_norm_LB_m_s2 = local_finite_min20(sg20);
    RES20(ic20).qdot_UB_rad_s = local_finite_max20(qd20);

    if isfield(Q20,'derivative_ub')
        du20 = double(Q20.derivative_ub(:));
        if numel(du20)>=3
            RES20(ic20).old_raw_d1_ub = du20(1);
            RES20(ic20).old_raw_d2_ub = du20(2);
            RES20(ic20).old_raw_d3_ub = du20(3);
        end
    end

    if cert20
        RES20(ic20).classification = 'LOCAL_VECTOR_CONTINUOUS_SPEED_CERTIFIED';
    elseif witnessFail20
        RES20(ic20).classification = 'FINITE_SENSING_SPEED_LIMIT_POINT_WITNESS';
    elseif reprOpen20
        RES20(ic20).classification = 'INCONCLUSIVE_BINARY64_REPRESENTABILITY';
    else
        RES20(ic20).classification = 'INCONCLUSIVE';
    end

    fprintf('\n[CANDIDATE %d]\n',ic20);
    fprintf(' local vector speed certified            = %d\n',cert20);
    fprintf(' finite-sensing violation point witness  = %d\n',witnessFail20);
    fprintf(' binary64 representability open          = %d\n',reprOpen20);
    fprintf(' split / leaf count                      = %d / %d\n',split20,numel(LEAF20));
    fprintf(' local VQ UB [m/s]                       = %.17g\n',RES20(ic20).VQ_UB_m_s);
    fprintf(' midpoint VQ lower witness max [m/s]     = %.17g\n',RES20(ic20).VQ_midpoint_LB_witness_m_s);
    fprintf(' payload speed UB [m/s]                  = %.17g\n',RES20(ic20).payload_speed_UB_m_s);
    fprintf(' payload accel UB [m/s^2]                = %.17g\n',RES20(ic20).payload_accel_UB_m_s2);
    fprintf(' payload jerk UB [m/s^3]                 = %.17g\n',RES20(ic20).payload_jerk_UB_m_s3);
    fprintf(' specific-force norm LB [m/s^2]          = %.17g\n',RES20(ic20).specific_force_norm_LB_m_s2);
    fprintf(' qdot UB [rad/s]                         = %.17g\n',RES20(ic20).qdot_UB_rad_s);
    fprintf(' old raw d1/d2/d3 UB                     = %.17g / %.17g / %.17g\n', ...
        RES20(ic20).old_raw_d1_ub,RES20(ic20).old_raw_d2_ub,RES20(ic20).old_raw_d3_ub);
    fprintf(' classification                          = %s\n',RES20(ic20).classification);
end

closedAny20 = any([RES20.certified]);
fprintf('\n[SUMMARY]\n');
fprintf(' at least one candidate locally certified = %d\n',closedAny20);
fprintf(' sampled acceptance                       = 0\n');
fprintf(' fixed depth acceptance cap               = 0\n');
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

AF7R20_OUT = struct();
AF7R20_OUT.schema = 'AF7R20_R8_LOCAL_VECTOR_SPEED_FORENSIC_V1';
AF7R20_OUT.read_only = true;
AF7R20_OUT.continuous_time = true;
AF7R20_OUT.sampled_acceptance = false;
AF7R20_OUT.fixed_depth_cap = false;
AF7R20_OUT.VQ_limit_m_s = lim20;
AF7R20_OUT.results = RES20;
AF7R20_OUT.any_candidate_certified = closedAny20;


function lf = local_empty_leaf20()
lf = struct( ...
    'segment_index',NaN,'ta',NaN,'tb',NaN,'C0',nan(3,16), ...
    'vl_ub',NaN,'al_ub',NaN,'jl_ub',NaN,'sigma_lb',NaN, ...
    'qdot_ub',NaN,'vq_ub',NaN,'vq_mid_witness',NaN);
end


function lf = local_make_leaf20(C0,ta,tb,iseg,baseEval,A0,J0,g,L,VQlimit) %#ok<INUSD>
lf = local_empty_leaf20();
lf.segment_index = iseg;
lf.ta = double(ta);
lf.tb = double(tb);
lf.C0 = double(C0);

T = lf.tb-lf.ta;
if ~(isfinite(T) && T>0 && isequal(size(C0),[3 16]) && all(isfinite(C0(:))))
    lf.vq_ub = inf;
    return;
end

C1 = (15/T)*diff(C0,1,2);
C2 = (14/T)*diff(C1,1,2);
C3 = (13/T)*diff(C2,1,2);

tc = 0.5*(lf.ta+lf.tb);
h = 0.5*T;
xd = double(baseEval(tc));
xd = xd(:);
if numel(xd)<15 || any(~isfinite(xd(1:15)))
    lf.vq_ub = inf;
    return;
end

v0 = xd(5:7);
a0 = xd(9:11);
j0 = xd(13:15);

% Vector-coupled payload velocity enclosure:
% correction velocity is a Bernstein curve in conv(C1).
Vctrl = C1 + v0;
lf.vl_ub = max(sqrt(sum(Vctrl.^2,1))) + A0*h;

% Base acceleration is locally enclosed by certified base jerk.
Actrl = C2 + a0;
lf.al_ub = max(sqrt(sum(Actrl.^2,1))) + J0*h;

d1m = local_decasteljau20(C1,0.5);
d2m = local_decasteljau20(C2,0.5);
d3m = local_decasteljau20(C3,0.5);

d3ub = max(sqrt(sum(C3.^2,1)));
lf.jl_ub = J0 + d3ub;

sMid = a0 + d2m + [0;0;g];
lf.sigma_lb = norm(sMid) - lf.jl_ub*h;

if ~(isfinite(lf.sigma_lb) && lf.sigma_lb>0)
    lf.qdot_ub = inf;
    lf.vq_ub = inf;
else
    lf.qdot_ub = lf.jl_ub/lf.sigma_lb;
    lf.vq_ub = lf.vl_ub + L*lf.qdot_ub;
end

% Exact midpoint lower witness only. Never used for acceptance.
vMid = v0 + d1m;
aMid = a0 + d2m;
jMid = j0 + d3m;
s = aMid + [0;0;g];
ns = norm(s);
if isfinite(ns) && ns>0
    q = -s/ns;
    qdot = -(eye(3)-q*q.')*jMid/ns;
    vQ = vMid - L*qdot;
    lf.vq_mid_witness = norm(vQ);
end
end


function [L,R] = local_split_half20(C)
C = double(C);
m = size(C,2);
L = zeros(size(C));
R = zeros(size(C));
T = C;
L(:,1)=T(:,1);
R(:,m)=T(:,end);
for k=1:m-1
    T = 0.5*(T(:,1:end-1)+T(:,2:end));
    L(:,k+1)=T(:,1);
    R(:,m-k)=T(:,end);
end
end


function v = local_decasteljau20(C,u)
T = double(C);
m = size(T,2);
for k=1:m-1
    T = (1-u)*T(:,1:end-1) + u*T(:,2:end);
end
v = T(:,1);
end


function x = local_finite_max20(v)
v = double(v(:));
v = v(isfinite(v));
if isempty(v)
    x = NaN;
else
    x = max(v);
end
end


function x = local_finite_min20(v)
v = double(v(:));
v = v(isfinite(v));
if isempty(v)
    x = NaN;
else
    x = min(v);
end
end
