function OUT = AF7R8_DUAL_MODE_AVOIDANCE_AUDIT_V3(agent,logger,replanner)
% AF7R8_DUAL_MODE_AVOIDANCE_AUDIT
% =========================================================================
% Read-only audit of the dual avoidance-certificate architecture.
%
% Usage after run_flight_headless:
%   AF7R8_DUAL_MODE_AVOIDANCE_AUDIT
%
% It answers:
%   1) Was the SAME candidate evaluated by reference-only and execution-aware?
%   2) Which layer is allowed to commit?
%   3) Did a REFERENCE_ONLY repair actually become active?
%   4) Why is EXECUTION_AWARE still open?
% =========================================================================

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R8 | DUAL REFERENCE / EXECUTION-AWARE AVOIDANCE AUDIT V3\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sameCandidateComparison=1 | sampledAcceptance=0\n');

if nargin<1 || isempty(agent)
    assert(evalin('base','exist(''agent'',''var'')==1'), ...
        '[AF7R8] agent not found.');
    agent=evalin('base','agent');
end
if nargin<2 || isempty(logger)
    assert(evalin('base','exist(''logger'',''var'')==1'), ...
        '[AF7R8] logger not found.');
    logger=evalin('base','logger');
end
if nargin<3 || isempty(replanner)
    if evalin('base','exist(''replanner_instance'',''var'')==1')
        replanner=evalin('base','replanner_instance');
    else
        replanner=agent.reference.timevarying;
    end
end

OUT=struct();
OUT.schema='AF7R8_DUAL_MODE_AVOIDANCE_AUDIT_V3';

M=replanner.get_avoidance_mode_status();
C=replanner.get_runtime_certificate_status();
try
    LR=replanner.get_last_replan_status();
catch
    LR=struct();
end

fprintf('\n[A. SWITCHES / COMMIT POLICY]\n');
fprintf(' reference-only evaluation enabled      = %d\n', ...
    M.reference_only_evaluation_enabled);
fprintf(' execution-aware evaluation enabled     = %d\n', ...
    M.execution_aware_evaluation_enabled);
fprintf(' commit policy                          = %s\n',M.commit_policy);
fprintf(' auto commit                            = %d\n',M.auto_commit);
fprintf(' active repaired safety level           = %s\n', ...
    M.active_repaired_safety_level);
fprintf(' reference eval count                   = %.0f\n', ...
    M.reference_only_evaluation_count);
fprintf(' execution eval count                   = %.0f\n', ...
    M.execution_aware_evaluation_count);
fprintf(' reference-only commit count            = %.0f\n', ...
    M.reference_only_commit_count);
fprintf(' execution-aware commit count           = %.0f\n', ...
    M.execution_aware_commit_count);
if isstruct(LR) && isfield(LR,'reason')
    fprintf(' last replan reason                     = %s\n',char(string(LR.reason)));
end
if isstruct(LR) && isfield(LR,'cycle_ms')
    fprintf(' last generator cycle [ms]              = %.9g\n',double(LR.cycle_ms));
end
if isstruct(LR) && isfield(LR,'deadline_ms')
    fprintf(' hard deadline [ms]                     = %.9g\n',double(LR.deadline_ms));
end
if isstruct(LR) && isfield(LR,'proposal_reason_code')
    fprintf(' proposal reason code                   = %.0f\n',double(LR.proposal_reason_code));
end
if isstruct(LR) && isfield(LR,'proposal_reason')
    fprintf(' proposal reason                        = %s\n',char(string(LR.proposal_reason)));
end
if isstruct(LR) && isfield(LR,'proposal_t0')
    fprintf(' proposal t0/tm/t1 [s]                 = %.9g / %.9g / %.9g\n', ...
        double(LR.proposal_t0),double(LR.proposal_tm),double(LR.proposal_t1));
end
if isstruct(LR) && isfield(LR,'rejoin_search_full_certificate_tests')
    fprintf(' rejoin full-cert tests                 = %.0f\n', ...
        double(LR.rejoin_search_full_certificate_tests));
end
if isstruct(LR) && isfield(LR,'rejoin_search_endpoint_safe_count')
    fprintf(' rejoin endpoint-safe candidates        = %.0f\n', ...
        double(LR.rejoin_search_endpoint_safe_count));
end

R=M.last_reference_only_evaluation;
X=M.last_execution_aware_evaluation;

fprintf('\n[A2. GENERATOR ATTEMPT HISTORY FROM LOGGER]\n');
ATT=extract_generator_attempts(logger);
if isempty(ATT)
    fprintf(' no generator request transitions found in logger.\n');
else
    fprintf([' #   t[s]      TTC[s]    coll[s]   status  reason  ' ...
        'generated   t0/tm/t1 [s]\n']);
    for ka=1:numel(ATT)
        a=ATT(ka);
        fprintf('%2d  %8.3f  %8.3f  %8.3f  %6.0f  %6.0f  %8.0f   %.3f / %.3f / %.3f\n', ...
            ka,a.t,a.ttc,a.collision_time,a.status_code,a.reason_code, ...
            a.generated,a.t0,a.tm,a.t1);
    end
end
OUT.generator_attempts=ATT;

fprintf('\n[B. SAME-CANDIDATE REFERENCE CERTIFICATE]\n');
rClosed=field_bool(R,'certified');
fprintf(' certified                              = %d\n',rClosed);
fprintf(' full system authority                  = %d\n', ...
    field_bool(R,'full_system_authority'));
fprintf(' reason                                 = %s\n', ...
    field_text(R,'reason','NO_REFERENCE_EVALUATION'));
fprintf(' proposal generated                     = %d\n', ...
    field_bool(R,'proposal_generated'));
fprintf(' proposal success                       = %d\n', ...
    field_bool(R,'proposal_success'));
fprintf(' proposal reason code                   = %.0f\n', ...
    field_num(R,'proposal_reason_code'));
fprintf(' proposal reason                        = %s\n', ...
    field_text(R,'proposal_reason','UNKNOWN'));
fprintf(' proposal t0/tm/t1 [s]                 = %.9g / %.9g / %.9g\n', ...
    field_num(R,'proposal_t0'),field_num(R,'proposal_tm'),field_num(R,'proposal_t1'));

if isstruct(R) && isfield(R,'geometry_certificate')
    G=R.geometry_certificate;
    fprintf(' geometry continuous-time               = %d\n', ...
        field_bool(G,'continuous_time'));
    fprintf(' geometry sampled acceptance            = %d\n', ...
        field_bool(G,'sampled_acceptance'));
    fprintf(' geometry IEEE754 outward               = %d\n', ...
        field_bool(G,'ieee754_outward_closed'));
    fprintf(' geometry min margin [m]                = %.9g\n', ...
        field_num(G,'min_margin_m'));
end

fprintf('\n[C. SAME-CANDIDATE EXECUTION-AWARE CERTIFICATE]\n');
xClosed=field_bool(X,'certified');
fprintf(' certified                              = %d\n',xClosed);
fprintf(' reason                                 = %s\n', ...
    field_text(X,'reason','NO_EXECUTION_EVALUATION'));
fprintf(' execution provider installed           = %d\n', ...
    M.execution_provider_installed);

if isstruct(X)
    fprintf(' tracking/actuator layer certified      = %d\n', ...
        field_bool(X,'tracking_actuator_layer_certified'));
    fprintf(' tracking-inflated geometry certified   = %d\n', ...
        field_bool(X,'reference_geometry_certified'));
    fprintf(' tracking-inflated geometry outward     = %d\n', ...
        field_bool(X,'reference_geometry_numerically_closed'));
    fprintf(' execution numerical outward            = %d\n', ...
        field_bool(X,'ieee754_outward_closed'));
    fprintf(' tracking inflation [m]                 = %.9g\n', ...
        field_num(X,'tracking_inflation_m'));

    if isfield(X,'position_error_ub')
        q=double(X.position_error_ub(:));
        if numel(q)==3
            fprintf(' position error UB xyz [m]               = [% .6g % .6g % .6g]\n',q);
        end
    end
    if isfield(X,'foundation') && isstruct(X.foundation)
        F=X.foundation;
        fprintf(' reference proposal valid                = %d\n', ...
            field_bool(F,'reference_proposal_valid'));
        fprintf(' reference derivative C6 bounds         = %d\n', ...
            field_bool(F,'reference_derivative_bounds_closed'));
        fprintf(' reference hold input box               = %d\n', ...
            field_bool(F,'reference_hold_input_box_closed'));
        fprintf(' first missing hard authority            = %s\n', ...
            field_text(F,'first_missing_authority','UNKNOWN'));
    end
end

fprintf('\n[D. OBSERVER / HLC BRIDGE]\n');
O=replanner.get_observer_hard_support_status();
fprintf(' observer enabled                       = %d\n',O.enabled);
fprintf(' observer initialized                   = %d\n',O.initialized);
fprintf(' selector verified                      = %d\n',O.selector_verified);
fprintf(' observer fail count                    = %.0f\n',O.fail_count);

bridgeClosed=false;
if isstruct(O.hlc_state_mass_bridge) && ...
        isfield(O.hlc_state_mass_bridge,'closed')
    bridgeClosed=logical(O.hlc_state_mass_bridge.closed);
end
fprintf(' observer -> HLC state/mass bridge      = %d\n',bridgeClosed);
fprintf(' current saturation closed              = %d\n', ...
    O.current_saturation_closed);

fprintf('\n[E. CURRENT REPAIRED REFERENCE]\n');
fprintf(' active reference mode                  = %s\n', ...
    char(C.signature.reference_mode));
fprintf(' active reference version               = %.0f\n', ...
    double(C.signature.reference_version));
fprintf(' active repaired safety level           = %s\n', ...
    C.active_repaired_safety_level);

fullAuthority=strcmpi(C.active_repaired_safety_level,'EXECUTION_AWARE') && ...
    M.execution_aware_commit_count>0;
referenceActive=strcmpi(C.active_repaired_safety_level,'REFERENCE_ONLY') && ...
    M.reference_only_commit_count>0;

fprintf(' reference-only repair active           = %d\n',referenceActive);
fprintf(' full-system execution-aware active     = %d\n',fullAuthority);

fprintf('\n[F. THEOREM STATUS]\n');
fprintf(' reference geometry theorem             = %d\n',rClosed);
fprintf(' source-specific D(Delta)->Phi(Delta)   = %d\n',xClosed);
fprintf(' same-tube actuator reserve             = %d\n', ...
    xClosed && isstruct(X) && isfield(X,'dynamics_certificate') && ...
    field_bool(X.dynamics_certificate,'certified'));
fprintf(' tracking-inflated geometry             = %d\n', ...
    field_bool(X,'reference_geometry_certified'));
fprintf(' recursive execution-aware backup       = 0\n');
fprintf(' full-system atomic commit              = %d\n',fullAuthority);

if ~xClosed
    firstOpen='SOURCE_SPECIFIC_HARD_DISTURBANCE_OPERATOR';
    if isstruct(X) && isfield(X,'foundation') && isstruct(X.foundation)
        firstOpen=field_text(X.foundation,'first_missing_authority',firstOpen);
    end
    fprintf(['\n[OPEN FIRST HARD AUTHORITY]\n' ...
        ' %s\n' ...
        '   Delta -> D(Delta) -> Phi(Delta)\n' ...
        ' No logged RMSE/max error may substitute for this hard operator.\n'], ...
        firstOpen);
end

OUT.mode=M;
OUT.runtime=C;
OUT.reference_certificate=R;
OUT.execution_certificate=X;
OUT.observer=O;
OUT.reference_only_active=referenceActive;
OUT.full_system_authority_active=fullAuthority;
OUT.closed_reference_layer=rClosed;
OUT.closed_execution_layer=xClosed;

fprintf('\n[G. CLASSIFICATION]\n');
if fullAuthority
    OUT.classification='EXECUTION_AWARE_REPAIR_COMMITTED__FULL_SYSTEM_AUTHORITY_ACTIVE';
elseif referenceActive
    OUT.classification= ...
        'REFERENCE_ONLY_REPAIR_COMMITTED__EXECUTION_AWARE_SHADOW_OPEN';
elseif rClosed
    OUT.classification= ...
        'REFERENCE_CERTIFIED__NO_REPAIR_COMMITTED__EXECUTION_AWARE_OPEN';
else
    OUT.classification='REFERENCE_AND_EXECUTION_AVOIDANCE_OPEN';
end
fprintf(' %s\n',OUT.classification);
fprintf('====================================================================================================\n');

assignin('base','AF7R8_DUAL_MODE_AVOIDANCE_AUDIT_OUT',OUT);
end

function ATT=extract_generator_attempts(logger)
ATT=repmat(struct( ...
    't',NaN,'request_count',NaN,'ttc',NaN,'collision_time',NaN, ...
    'status_code',NaN,'reason_code',NaN,'generated',NaN, ...
    't0',NaN,'tm',NaN,'t1',NaN),1,0);

try
    D=logger.Data;
    A=D.agent;
    if numel(A)>1, A=A(1); end
    R=result_cells_local(member_local(member_local(A,'reference'),'result'));
    N=numel(R);
    if N<1, return; end

    if isfield(D,'t') && numel(D.t)>=N
        tt=double(D.t(1:N));
        tt=tt(:);
    else
        tt=(0:N-1).';
    end

    req=series_local(R,'state.generator_request_count');
    status=series_local(R,'state.generator_last_status_code');
    ttc=series_local(R,'state.trigger_ttc');
    coll=series_local(R,'state.trigger_collision_time');
    reason=series_local(R,'state.bezier_shadow_last_reason_code');
    genCount=series_local(R,'state.bezier_shadow_generated_count');
    t0=series_local(R,'state.bezier_shadow_last_t0');
    tm=series_local(R,'state.bezier_shadow_last_tm');
    t1=series_local(R,'state.bezier_shadow_last_t1');

    prev=0;
    prevGen=0;
    for k=1:N
        if ~isfinite(req(k)), continue; end
        if req(k)>prev
            a=struct();
            a.t=tt(k);
            a.request_count=req(k);
            a.ttc=ttc(k);
            a.collision_time=coll(k);
            a.status_code=status(k);
            a.reason_code=reason(k);
            if isfinite(genCount(k))
                a.generated=double(genCount(k)>prevGen);
                prevGen=max(prevGen,genCount(k));
            else
                a.generated=NaN;
            end
            a.t0=t0(k);
            a.tm=tm(k);
            a.t1=t1(k);
            ATT(end+1)=a; %#ok<AGROW>
            prev=req(k);
        elseif req(k)>prev
            prev=req(k);
        end
    end
catch
    ATT=repmat(struct( ...
        't',NaN,'request_count',NaN,'ttc',NaN,'collision_time',NaN, ...
        'status_code',NaN,'reason_code',NaN,'generated',NaN, ...
        't0',NaN,'tm',NaN,'t1',NaN),1,0);
end
end

function x=member_local(S,name)
x=[];
try
    if isempty(S), return; end
    if isstruct(S)
        if isfield(S,name), x=S.(name); end
    elseif isobject(S)
        if isprop(S,name), x=S.(name); end
    end
catch
end
end

function C=result_cells_local(X)
if isempty(X)
    C={};
elseif iscell(X)
    C=X(:);
elseif isstruct(X) || isobject(X)
    C=num2cell(X(:));
else
    C={};
end
end

function x=nested_local(S,path)
x=S;
P=strsplit(path,'.');
for k=1:numel(P)
    x=member_local(x,P{k});
    if isempty(x), return; end
end
end

function v=series_local(C,path)
v=nan(numel(C),1);
for k=1:numel(C)
    x=nested_local(C{k},path);
    try
        if (isnumeric(x)||islogical(x))&&isscalar(x)
            v(k)=double(x);
        end
    catch
    end
end
end

function tf=field_bool(S,name)
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
end

function x=field_num(S,name)
x=NaN;
try
    if isstruct(S)&&isfield(S,name)
        v=double(S.(name));
        if isscalar(v)&&isfinite(v), x=v; end
    end
catch
end
end

function x=field_text(S,name,default)
x=default;
try
    if isstruct(S)&&isfield(S,name)
        x=char(string(S.(name)));
    end
catch
end
end
