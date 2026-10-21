function S=AF7R38_SPEED_ROOTCAUSE_AUDIT()
% AF7R38_SPEED_ROOTCAUSE_AUDIT
% Diagnostic-only. Does not modify planner/controller/runner/MEX state.
% Run immediately after the same simulation that produced S38.

R = evalin('base','replanner_instance');
P = R.get_staged_pipeline_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R38 DEEP SPEED / ROOT-CAUSE AUDIT (DIAGNOSTIC ONLY)\n');
fprintf('====================================================================================================\n');

fprintf('\n[A. R37/R38 CONSTRUCTION]\n');
fprintf('side assignments                              = %.0f\n', val(P,'r37_side_assignments'));
fprintf('spatial QP solutions                          = %.0f\n', val(P,'r37_spatial_qp_solutions'));
fprintf('constructive exact-C7 candidates              = %.0f\n', val(P,'r38_constructive_c7_count'));
fprintf('legacy floating false rejects                 = %.0f\n', val(P,'r38_legacy_float_false_reject_count'));
fprintf('path-tangent yaw-C6 pass                      = %.0f\n', val(P,'r37_heading_pass_count'));
fprintf('spatial knot count                            = %.0f\n', val(P,'r37_knot_count'));
fprintf('support start / rejoin [s]                    = %.12g / %.12g\n', ...
    val(P,'r37_support_start_s'), val(P,'r37_actual_rejoin_s'));
fprintf('best total velocity UB                        = %.12g m/s\n', val(P,'r37_best_total_velocity_ub'));
fprintf('best correction acceleration UB               = %.12g m/s^2\n', val(P,'r37_best_accel_ub'));
fprintf('best correction jerk UB                       = %.12g m/s^3\n', val(P,'r37_best_jerk_ub'));

D = struct();
try
    D = R.generator_last_status;
catch
end

fprintf('\n[B. FINITE-SENSING SPEED GATE]\n');
if isempty(fieldnames(D))
    fprintf('generator_last_status                         = EMPTY\n');
    fprintf('Run this audit immediately after the failed replan/simulation.\n');
else
    lim = val(D,'finite_sensing_limit_m_s');
    vq  = val(D,'primary_min_stageA_uav_speed_UB_m_s');
    sig = val(D,'primary_min_stageA_specific_force_LB_m_s2');
    u0  = val(D,'primary_min_stageA_correction_speed_UB_m_s');
    u1  = val(D,'primary_min_stageA_correction_accel_UB_m_s2');
    u2  = val(D,'primary_min_stageA_correction_jerk_UB_m_s3');

    fprintf('finite-sensing UAV-speed limit                = %.12g m/s\n',lim);
    fprintf('best primary Stage-A UAV-speed UB             = %.12g m/s\n',vq);
    fprintf('Stage-A UB / limit                            = %.12g\n',safe_ratio(vq,lim));
    fprintf('best Stage-A specific-force LB sigma          = %.12g m/s^2\n',sig);
    fprintf('correction speed UB U0                        = %.12g m/s\n',u0);
    fprintf('correction acceleration UB U1                 = %.12g m/s^2\n',u1);
    fprintf('correction jerk UB U2                         = %.12g m/s^3\n',u2);
    fprintf('minimum correction envelope                   = %.12g m\n',val(D,'primary_min_envelope_m'));

    fprintf('\n[C. STAGE-A / STAGE-B RESOLUTION]\n');
    a = val(D,'primary_stageA_certified_count');
    b = val(D,'primary_stageB_certified_count');
    w = val(D,'primary_stageB_finite_witness_count');
    r = val(D,'primary_stageB_repr_fail_count');
    fprintf('Stage-A certified candidates                  = %.0f\n',a);
    fprintf('Stage-B certified candidates                  = %.0f\n',b);
    fprintf('Stage-B finite violation witnesses            = %.0f\n',w);
    fprintf('Stage-B representability failures             = %.0f\n',r);

    fprintf('\n[D. ROOT-CAUSE CLASSIFICATION]\n');
    if isfinite(w) && w > 0
        fprintf('classification                                = PROVEN SPEED VIOLATION WITNESS EXISTS\n');
        fprintf('meaning                                       = not only a loose global UB; Stage-B found a finite-time witness at/above the hard limit.\n');
    elseif isfinite(b) && b > 0
        fprintf('classification                                = SPEED CERTIFICATE CAN CLOSE FOR AT LEAST ONE PRIMARY\n');
        fprintf('meaning                                       = inspect rejoin/geometry next; aggregate audit may be hiding later rejection.\n');
    elseif isfinite(r) && r > 0
        fprintf('classification                                = STAGE-B REPRESENTABILITY BLOCKED\n');
        fprintf('meaning                                       = adaptive subdivision could not continue exactly; do not redesign geometry yet.\n');
    elseif isfinite(vq) && isfinite(lim) && vq >= lim
        fprintf('classification                                = STAGE-A TOO LARGE, STAGE-B INCONCLUSIVE\n');
        fprintf('meaning                                       = need per-segment Stage-B diagnostics before changing path theory.\n');
    else
        fprintf('classification                                = UNRESOLVED FROM EXPOSED SCALARS\n');
    end

    if isfinite(u0) && isfinite(lim)
        fprintf('correction-speed UB / UAV limit               = %.12g\n',safe_ratio(u0,lim));
    end
    if isfinite(sig) && sig <= 0
        fprintf('specific-force singularity                    = YES (sigma <= 0)\n');
    else
        fprintf('specific-force singularity                    = NO from Stage-A best candidate\n');
    end
end

fprintf('\n[E. HARD PIPELINE]\n');
fprintf('candidate success                             = %d\n',logical(val(P,'predeadline_candidate_success')));
fprintf('candidate family                              = %s\n',strval(P,'predeadline_candidate_family'));
fprintf('exact C7                                      = %d\n',logical(val(P,'predeadline_c7_exact')));
fprintf('speed / rejoin / geometry                     = %.0f / %.0f / %.0f\n', ...
    val(P,'predeadline_speed_pass_count'), ...
    val(P,'predeadline_rejoin_pass_count'), ...
    val(P,'predeadline_geometry_test_count'));
fprintf('state / reason                                = %s / %s\n',strval(P,'state'),strval(P,'last_reason'));
fprintf('stage1 [ms]                                   = %.9g\n',val(P,'last_stage1_ms'));

fprintf('\n[F. WHAT THE NEXT DIAGNOSTIC MUST EXPOSE IF SPEED STILL FAILS]\n');
fprintf('1) per-candidate side assignment\n');
fprintf('2) all knot times and segment durations\n');
fprintf('3) each knot ||P_j|| and ||V_j||\n');
fprintf('4) per-segment total-velocity / accel / jerk Bernstein UBs\n');
fprintf('5) worst Stage-B leaf: segment, [ta,tb], VL, sigma, qdot, VQ\n');
fprintf('6) finite witness time/value when one exists\n');
fprintf('These are diagnostic only; none may grant acceptance.\n');

fprintf('====================================================================================================\n');

S = struct('schema','AF7R38_SPEED_ROOTCAUSE_AUDIT','pipeline',P,'generator',D);
end

function x=val(S,f)
x=NaN;
if isstruct(S)&&isfield(S,f)
    try
        q=double(S.(f));
        if isscalar(q),x=q;end
    catch
    end
end
end

function y=safe_ratio(a,b)
if isfinite(a)&&isfinite(b)&&b~=0
    y=a/b;
else
    y=NaN;
end
end

function s=strval(S,f)
s='?';
if isstruct(S)&&isfield(S,f)
    try
        s=char(S.(f));
    catch
    end
end
end
