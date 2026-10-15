function OUT = AF7R9_HLC_HARD_OPERATOR_AUDIT_V2(replanner)
% AF7R9_HLC_HARD_OPERATOR_AUDIT
% =========================================================================
% Read-only audit of the CURRENT execution-aware hard-authority boundary.
%
% Usage after run_flight_headless:
%   AF7R9_HLC_HARD_OPERATOR_AUDIT
%
% This audit does NOT promote logged RMSE, sampled maxima, covariance, Q/R,
% or center-command evaluations into safety authority.
% =========================================================================

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R9 | CURRENT HLC HARD-OPERATOR / FIXED-POINT BOUNDARY AUDIT V2\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | guessedBounds=0 | loggedErrorAuthority=0 | sampledAcceptance=0\n');

if nargin<1 || isempty(replanner)
    assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
        '[AF7R9] replanner_instance not found in base workspace.');
    replanner=evalin('base','replanner_instance');
end

M=replanner.get_avoidance_mode_status();
C=replanner.get_runtime_certificate_status();
O=replanner.get_observer_hard_support_status();
L=replanner.get_last_replan_status();

OUT=struct();
OUT.schema='AF7R9_HLC_HARD_OPERATOR_AUDIT_V2';

fprintf('\n[A. CURRENT CANDIDATE / DEADLINE STATE]\n');
fprintf(' reference evaluation count             = %.0f\n',M.reference_only_evaluation_count);
fprintf(' execution evaluation count             = %.0f\n',M.execution_aware_evaluation_count);
fprintf(' reference-only commit count            = %.0f\n',M.reference_only_commit_count);
fprintf(' execution-aware commit count           = %.0f\n',M.execution_aware_commit_count);
fprintf(' commit policy                          = %s\n',M.commit_policy);
fprintf(' active repaired safety level           = %s\n',M.active_repaired_safety_level);

lastReason='NONE';
lastCycle=NaN;
lastDeadline=NaN;
if isstruct(L)
    if isfield(L,'reason'), lastReason=char(string(L.reason)); end
    if isfield(L,'cycle_ms'), lastCycle=double(L.cycle_ms); end
    if isfield(L,'deadline_ms'), lastDeadline=double(L.deadline_ms); end
end
fprintf(' last replan reason                     = %s\n',lastReason);
fprintf(' last generator cycle / deadline [ms]   = %.9g / %.9g\n',lastCycle,lastDeadline);

X=M.last_execution_aware_evaluation;
F=struct();
if isstruct(X) && isfield(X,'foundation') && isstruct(X.foundation)
    F=X.foundation;
end

fprintf('\n[B. OBSERVER -> HLC HARD INPUT AUTHORITY]\n');
fprintf(' observer enabled                       = %d\n',logical(O.enabled));
fprintf(' observer initialized                   = %d\n',logical(O.initialized));
fprintf(' selector verified                      = %d\n',logical(O.selector_verified));
fprintf(' observer fail count                    = %.0f\n',double(O.fail_count));

bridgeClosed=false;
if isstruct(O.hlc_state_mass_bridge) && isfield(O.hlc_state_mass_bridge,'closed')
    bridgeClosed=logical(O.hlc_state_mass_bridge.closed);
end
fprintf(' observer -> HLC bridge closed          = %d\n',bridgeClosed);

if bridgeClosed
    B=O.hlc_state_mass_bridge;

    if isfield(B,'model_state')
        S=B.model_state;
        fprintf(' compact quaternion chord UB            = %.9g\n', ...
            scalar_field(S,'compact_quaternion_chord_norm_ub'));
        fprintf(' body-rate error norm UB                = %.9g\n', ...
            scalar_field(S,'w_norm_ub'));
        fprintf(' payload velocity error norm UB         = %.9g\n', ...
            scalar_field(S,'vL_norm_ub'));
        fprintf(' cable angular-rate error norm UB       = %.9g\n', ...
            scalar_field(S,'wL_norm_ub'));

        q=vector_field(S,'pL_abs_ub',3);
        fprintf(' payload position error abs UB xyz [m]  = [% .6g % .6g % .6g]\n',q);

        q=vector_field(S,'pT_abs_ub',3);
        fprintf(' cable-direction error abs UB           = [% .6g % .6g % .6g]\n',q);

        q=vector_field(S,'euler_abs_ub',3);
        fprintf(' Euler measurement/error abs UB [rad]   = [% .6g % .6g % .6g]\n',q);
    end

    if isfield(B,'live_mL')
        ML=B.live_mL;
        I=vector_field(ML,'true_interval',2);
        fprintf(' live true mL hard interval [kg]        = [% .9g % .9g]\n',I);
        fprintf(' estimator mL error abs UB [kg]         = %.9g\n', ...
            scalar_field(ML,'error_abs_ub'));
    end
end

fprintf('\n[C. EXECUTION-AWARE FOUNDATION SNAPSHOT]\n');
if isempty(fieldnames(F))
    fprintf(' foundation snapshot                    = NOT EVALUATED IN THIS RUN\n');
    fprintf(' next action                            = rerun with current REPLANNING_SYSTEM;\n');
    fprintf('                                          deadline miss will block COMMIT but no longer discard read-only evaluation.\n');
else
    print_bool(F,'reference_proposal_valid','valid reference proposal');
    print_bool(F,'observer_bridge_closed','observer bridge closed');
    print_bool(F,'model_state_hard_set_closed','model-state hard set closed');
    print_bool(F,'live_mL_interval_closed','live mL hard interval available');
    print_bool(F,'reference_derivative_bounds_closed','reference full C6 derivative hard bounds');
    print_bool(F,'reference_hold_input_box_closed','reference 28-D ZOH hold input box');
    print_bool(F,'command_limit_source_closed','command-limit source binding');
    print_bool(F,'live_hlc_binding_closed','live HLC/gain binding');
    print_bool(F,'generated_map_sources_resolved','generated HLC map sources resolved');

    fprintf(' first missing authority                = %s\n', ...
        text_field(F,'first_missing_authority','UNKNOWN'));

    if isfield(F,'reference_speed_certificate') && ...
            isstruct(F.reference_speed_certificate)
        SC=F.reference_speed_certificate;
        if isfield(SC,'position_derivative_norm_ub_1_to_6')
            q=double(SC.position_derivative_norm_ub_1_to_6(:));
            if numel(q)==6
                fprintf(' position derivative norm UB d1..d6    = [%s ]\n', ...
                    sprintf(' %.6g',q));
            end
        end
        if isfield(SC,'yaw_derivative_abs_ub_1_to_6')
            q=double(SC.yaw_derivative_abs_ub_1_to_6(:));
            if numel(q)==6
                fprintf(' yaw derivative abs UB d1..d6          = [%s ]\n', ...
                    sprintf(' %.6g',q));
            end
        end
        if isfield(SC,'hold_packet_abs_delta_ub')
            q=double(SC.hold_packet_abs_delta_ub(:));
            if numel(q)==28
                fprintf(' max 28-D hold-box halfwidth           = %.9g\n',max(q));
                fprintf(' highest C6 position hold bound xyz    = [% .6g % .6g % .6g]\n',q(25:27));
                fprintf(' highest C6 yaw hold bound              = %.9g\n',q(28));
            end
        end
        if isfield(SC,'hold_bound_theorem')
            fprintf(' hold theorem                           = %s\n', ...
                char(string(SC.hold_bound_theorem)));
        end
    end

    fprintf('\n[D. NUMERICAL HARD-OPERATOR CLOSURE]\n');
    print_bool(F,'model_state_interval_image_closed', ...
        'model-state -> HLC virtual mismatch image');
    print_bool(F,'hold_reference_mismatch_closed', ...
        'reference-hold mismatch bound');
    print_bool(F,'saturation_gap_closed', ...
        'same-cycle saturation-gap bound');
    print_bool(F,'hard_disturbance_set_closed', ...
        'combined hard disturbance set D(Delta)');
    print_bool(F,'phi_fixed_point_closed', ...
        'attained fixed point Delta >= Phi(Delta)');
    print_bool(F,'position_error_ub_closed', ...
        'hard componentwise position_error_ub');
    print_bool(F,'continuous_actuator_reserve_closed', ...
        'continuous actuator reserve on same Delta');
    print_bool(F,'ieee754_outward_closed', ...
        'directed/outward numerical closure');
end

fprintf('\n[E. MATHEMATICAL NEXT OPERATOR]\n');
fprintf(' Existing closed structure is reused; the current numerical operator must implement:\n\n');
fprintf('   For a valid C6 repaired reference, the hold-input box is now derived by:\n');
fprintf('     r=0..5: |z^(r)(t+tau)-z^(r)(t)| <= UB(z^(r+1))*dt\n');
fprintf('     r=6   : |z6(t2)-z6(t1)| <= 2*sup|z6|   [no d7 assumed]\n\n');
fprintf('   E_obs  = current observer hard set in HLC coordinates\n');
fprintf('   E_D    = tracking-error tube parameterized by Delta\n');
fprintf('   D(Delta) = D_state(E_obs,E_D)\n');
fprintf('            (+) D_mL(E_obs.mL,E_D)\n');
fprintf('            (+) D_hold(reference C6/C7 derivative bounds)\n');
fprintf('            (+) D_sat(Delta)\n\n');
fprintf('   Phi(Delta) = continuous-intersample RPI response to D(Delta)\n\n');
fprintf(' Required fixed point:\n');
fprintf('   Delta* >= Phi(Delta*)\n\n');
fprintf(' Only after that may:\n');
fprintf('   position_error_ub = position projection of the SAME Delta*\n');
fprintf(' be exported to obstacle-clearance inflation.\n\n');
fprintf(' The unclamped HLC command must then be certified on that SAME tube;\n');
fprintf(' positive reserve makes D_sat=0. Otherwise the candidate is rejected.\n');

fprintf('\n[F. CURRENT PROMOTION STATUS]\n');
execClosed=false;
if isstruct(X) && isfield(X,'certified')
    execClosed=logical(X.certified);
end
fprintf(' execution-aware composite certified    = %d\n',execClosed);
fprintf(' current saturation closed              = %d\n',logical(O.current_saturation_closed));
fprintf(' HLC robust runtime certificate         = %d\n',certified(C.hlc_robust));
fprintf(' actuator dynamics runtime certificate  = %d\n',certified(C.actuator_dynamics));

OUT.mode=M;
OUT.runtime=C;
OUT.observer=O;
OUT.last_replan_status=L;
OUT.execution_evaluation=X;
OUT.foundation=F;
OUT.execution_aware_closed=execClosed;

if isempty(fieldnames(F))
    OUT.classification='RERUN_REQUIRED__FOUNDATION_NOT_EVALUATED';
elseif execClosed
    OUT.classification='EXECUTION_AWARE_HARD_OPERATOR_CLOSED';
else
    OUT.classification=['FOUNDATION_INPUTS_BOUND__FIRST_MISSING__' ...
        text_field(F,'first_missing_authority','UNKNOWN')];
end

fprintf('\n[G. CLASSIFICATION]\n');
fprintf(' %s\n',OUT.classification);
fprintf('====================================================================================================\n');

assignin('base','AF7R9_HLC_HARD_OPERATOR_AUDIT_OUT',OUT);
end

function x=scalar_field(S,name)
x=NaN;
try
    if isstruct(S)&&isfield(S,name)
        z=double(S.(name));
        if isscalar(z)&&isfinite(z), x=z; end
    end
catch
end
end

function v=vector_field(S,name,n)
v=nan(n,1);
try
    if isstruct(S)&&isfield(S,name)
        z=double(S.(name));
        z=z(:);
        if numel(z)==n&&all(isfinite(z)), v=z; end
    end
catch
end
end

function s=text_field(S,name,default)
s=default;
try
    if isstruct(S)&&isfield(S,name)
        s=char(string(S.(name)));
    end
catch
end
end

function print_bool(S,name,label)
tf=false;
try
    if isstruct(S)&&isfield(S,name)
        v=S.(name);
        if (islogical(v)||isnumeric(v))&&isscalar(v)&&isfinite(double(v))
            tf=logical(v);
        end
    end
catch
end
fprintf(' %-43s = %d\n',label,tf);
end

function tf=certified(S)
tf=false;
try
    tf=isstruct(S)&&isfield(S,'certified')&&logical(S.certified);
catch
end
end
