function R = CHECK_TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(agent)
% CHECK_TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL
% Corrected end-to-end regression for nominal C9 torque certificate.
% All physical fixture parameters are obtained from the current parameter
% class and all input clamps are read from the current HLC SOURCE.  These
% values are NOT carried into Flight, not cached for future Flights, and
% no result grants permission to COMMIT a trajectory.
% If supplied, agent is the already initialized actual Flight agent.
% No-argument mode is a clearly labelled TEST FIXTURE (not real EKF).
%
% Unconditional MAT/TXT/ZIP export into common_matlab/ even on failure.
% Distinguishes missing/wrong active source, malformed parameters, proof
% UNKNOWN, and numerical/implementation exceptions.  No fail->pass bypass.
root = fileparts(mfilename('fullpath'));
R = struct('status','FAIL_C9_NOMINAL_TORQUE_NOT_EXECUTED', ...
    'stage','INITIALIZE','error_identifier','','error_message','', ...
    'error_stack','', 'real_hlc_limits_read',false, ...
    'parameter_class_source_file','', ...
    'parameter_set_line_104','', ...
    'hlc_source_file','','hlc_input_lower',NaN(4,1), ...
    'hlc_input_upper',NaN(4,1), ...
    'physical_parameter_names',{{'mass','loadmass','cableL','jx','jy','jz','gravity'}}, ...
    'physical_parameter_values',NaN(1,7), ...
    'mass_source','NOT_EVALUATED','payload_mass_used_kg',NaN, ...
    'current_estimator_mass_used',false,'zero_mass_rejected',false, ...
    'constant_heading_x_pass',false,'rotated_heading_y_pass',false, ...
    'zero_horizontal_speed_rejected',false, ...
    'incorrect_torque_window_rejected',false, ...
    'force_vertical_singularity_rejected',false, ...
    'mass_interval_pass',false, ...
    'invalid_mass_interval_rejected',false, ...
    'derivative_control_tampering_rejected',false, ...
    'no_hlc_commit',false, ...
    'constant_heading_x_status','NOT_EVALUATED', ...
    'rotated_heading_y_status','NOT_EVALUATED', ...
    'zero_horizontal_speed_status','NOT_EVALUATED', ...
    'incorrect_torque_window_status','NOT_EVALUATED', ...
    'force_vertical_singularity_status','NOT_EVALUATED', ...
    'mass_interval_status','NOT_EVALUATED', ...
    'invalid_mass_interval_status','NOT_EVALUATED', ...
    'derivative_control_tampering_status','NOT_EVALUATED', ...
    'constant_heading_x_leaf_reason','NOT_EVALUATED', ...
    'mass_interval_leaf_reason','NOT_EVALUATED', ...
    'constant_heading_x_interval_exception','', ...
    'constant_heading_x_exception_message','', ...
    'mass_interval_interval_exception','', ...
    'mass_interval_exception_message','', ...
    'constant_heading_x_unresolved_interval_s',[NaN NaN], ...
    'mass_interval_unresolved_interval_s',[NaN NaN], ...
    'nominal_elapsed_ms',NaN, ...
    'fast_norm_leaf_verified',false, ...
    'detailed_fallback_preserved',false, ...
    'fast_norm_enabled_for_nominal',false, ...
    'fast_norm_checked_nodes',NaN, ...
    'fast_norm_accepted_leaves',NaN, ...
    'detailed_interval_checked_nodes',NaN, ...
    'reference_detailed_status','NOT_EVALUATED', ...
    'fast_and_detailed_both_prove_nominal',false, ...
    'fast_median_ms',NaN,'detailed_median_ms',NaN, ...
    'fast_p95_ms',NaN,'detailed_p95_ms',NaN, ...
    'fast_max_ms',NaN,'detailed_max_ms',NaN, ...
    'torque_headroom_Nm',NaN, ...
    'actual_HLC_all_time_tracking_proven',false);
try
    R.stage='CHECK_PROJECT_ROOT';
    assert(isfile(fullfile(root,'run_flight_headless.m')), ...
        'C9_TORQUE_CHECK:Root','CHECK must be placed in common_matlab.');
    addpath(fullfile(root,'controller'),'-begin');
    addpath(fullfile(root,'model'),'-begin');
    addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
        'TRAJECTORY_GENERATION'),'-begin');
    R.stage='BIND_REAL_HLC_FOUR_INPUT_LIMITS';
    [lower,upper,source_file] = ...
        TRAJECTORY_C9_HLC_FORCE_AUTHORITY.source_input_limits();
    lower=double(lower(:));upper=double(upper(:));
    assert(numel(lower)==4&&numel(upper)==4&& ...
        all(isfinite([lower;upper]))&&all(lower<upper), ...
        'C9_TORQUE_CHECK:HLCLimits', ...
        'HLC source did not supply four valid actuator limits.');
    R.hlc_source_file=char(source_file);
    R.hlc_input_lower=lower;R.hlc_input_upper=upper;
    R.real_hlc_limits_read=true;

    R.stage='CONSTRUCT_CURRENT_PARAMETER_FIXTURE';
    % Record the RESOLVED normal-program parameter implementation.  Its
    % source is not bundled with this patch, so never infer line 104.
    R.parameter_class_source_file=which('DRONE_PARAM_SUSPENDED_LOAD');
    if isfile(R.parameter_class_source_file)
        try
            param_lines=regexp(fileread(R.parameter_class_source_file), ...
                '\r\n|\n|\r','split');
            if numel(param_lines)>=104
                R.parameter_set_line_104=strtrim(param_lines{104});
            end
        catch
            % Source excerpt is diagnostic only; never weaken proof.
        end
    end
    if nargin>=1 && ~isempty(agent)
        % Read from the actual Flight agent, never fabricate an estimate.
        % The state manager has already updated estimator.result.state.mL
        % before the HLC within each normal Flight cycle.
        R.stage='BIND_CURRENT_FLIGHT_ESTIMATOR';
        assert(strcmp(char(agent.cha),'f'), ...
            'C9_TORQUE_CHECK:NotFlight','Live estimator is only valid in Flight.');
        assert(isa(agent.controller.hlc_suspended,'HLC_SUSPENDED_LOAD'), ...
            'C9_TORQUE_CHECK:ActiveHLC','Actual active HLC is required.');
        P=agent.parameter;
        est=agent.estimator.result.state.mL;
        assert(isnumeric(est)&&isreal(est)&&isscalar(est)&& ...
            isfinite(est)&&est>0, ...
            'C9_TORQUE_CHECK:EstimatorMass','Current EKF mass must be positive.');
        R.mass_source='LIVE_EKF_AT_CHECK_CALL';
    else
        % Standalone audit only.  Same constructor and STRING-key set()
        % API used in run_flight_headless.m.  Test mass is NOT production.
        R.stage='CONSTRUCT_CURRENT_PARAMETER_FIXTURE';
        P=DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
        P.set("loadmass",0.14); % synthetic test value ONLY
        est=[];
        R.mass_source='SYNTHETIC_PARAMETER_FIXTURE__NOT_EKF';
    end
    R.stage='READ_HLC_PARAMETER_VECTOR';
    % This is the EXACT seven-key request (and ordering) used by the
    % active HLC_SUSPENDED_LOAD.do in the normal program.
    raw=double(P.get(["mass","jx","jy","jz", ...
                      "gravity","loadmass","cableL"]));
    assert(isreal(raw)&&numel(raw)==7&&all(isfinite(raw(:)))&& ...
        all(raw(:)>0), ...
        'C9_TORQUE_CHECK:PhysicalParameter', ...
        'The active HLC parameter getter returned invalid values.');
    raw=reshape(raw,1,7);
    values=raw([1 6 7 2 3 4 5]);
    R.physical_parameter_values=values;
    R.payload_mass_used_kg=values(2);
    if ~isempty(est)
        assert(isequal(double(est),values(2)), ...
            'C9_TORQUE_CHECK:EKFHLCMassMismatch', ...
            'Estimator mass differs from mass read by the actual HLC.');
        R.current_estimator_mass_used=true;
    end
    A=struct('uav_mass_kg',values(1), ...
        'payload_mass_kg',values(2), ...
        'cable_length_m',values(3), ...
        'uav_inertia_kgm2',values(4:6).', ...
        'gravity_world_mps2',[0;0;-values(7)], ...
        'torque_limits_Nm',[lower(2:4),upper(2:4)]);
    O=struct('max_subdivision_depth',6,'max_node_count',127, ...
        'minimum_force_z_N',0,'minimum_planar_speed_mps',0, ...
        'torque_guard_Nm',[0;0;0]);

    T=2;t0=8;
    R.stage='CERTIFY_X_HEADING';
    Q=make_affine(t0,T,[0;0;3],[0.5;0;0]);
    H=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,O);
    R.constant_heading_x_status=H.status;
    R.constant_heading_x_leaf_reason=H.last_leaf_reason;
    R.constant_heading_x_interval_exception=H.last_exception_identifier;
    R.constant_heading_x_exception_message=H.last_exception_message;
    R.constant_heading_x_unresolved_interval_s=H.unresolved_interval_s;
    R.constant_heading_x_pass=H.conditional_nominal_all_time_torque_proven;
    R.nominal_elapsed_ms=H.elapsed_ms;
    R.torque_headroom_Nm=H.minimum_torque_headroom_Nm;
    R.fast_norm_enabled_for_nominal=H.fast_norm_enabled;
    R.fast_norm_checked_nodes=H.fast_norm_checked_nodes;
    R.fast_norm_accepted_leaves=H.fast_norm_accepted_leaves;
    R.detailed_interval_checked_nodes=H.detailed_interval_checked_nodes;
    R.fast_norm_leaf_verified=H.conditional_nominal_all_time_torque_proven && ...
        H.fast_norm_accepted_leaves>=1 && H.fast_norm_exception_count==0;

    R.stage='COMPARE_EXACT_SAME_PIECE_WITH_DETAILED_INTERVAL_BASELINE';
    Odetail=O;Odetail.enable_fast_norm_sufficient_bound=false;
    Baseline=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,Odetail);
    R.reference_detailed_status=Baseline.status;
    R.fast_and_detailed_both_prove_nominal= ...
        H.conditional_nominal_all_time_torque_proven && ...
        Baseline.conditional_nominal_all_time_torque_proven && ...
        Baseline.fast_norm_checked_nodes==0 && ...
        Baseline.detailed_interval_checked_nodes>0 && ...
        ~Baseline.safe_to_commit && ~H.safe_to_commit;
    % Repeated whole-certificate timings: never hide a cost in preflight.
    % Runtime acceptance is independent of speed so a slow machine does not
    % yield a spurious mathematical failure.
    K=7;
    timings_fast=zeros(1,K);timings_detail=zeros(1,K);
    for bi=1:K
        if mod(bi,2)==1
            a=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,O);
            b=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,Odetail);
        else
            b=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,Odetail);
            a=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,O);
        end
        assert(a.conditional_nominal_all_time_torque_proven && ...
            b.conditional_nominal_all_time_torque_proven, ...
            'C9_TORQUE_CHECK:BenchmarkCertification', ...
            'Fast and detailed reference must both certify the same polynomial');
        timings_fast(bi)=a.elapsed_ms;
        timings_detail(bi)=b.elapsed_ms;
    end
    R.fast_median_ms=median(timings_fast);
    R.detailed_median_ms=median(timings_detail);
    R.fast_p95_ms=percentile95_local(timings_fast);
    R.detailed_p95_ms=percentile95_local(timings_detail);
    R.fast_max_ms=max(timings_fast);
    R.detailed_max_ms=max(timings_detail);

    R.stage='CERTIFY_ROTATED_Y_HEADING';
    Qy=make_affine(t0,T,[0;0;3],[0;0.5;0]);
    Y=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Qy,A,O);
    R.rotated_heading_y_status=Y.status;
    R.rotated_heading_y_pass=Y.conditional_nominal_all_time_torque_proven;

    R.stage='REJECT_ZERO_HEADING_SPEED';
    Q0=make_affine(t0,T,[0;0;3],[0;0;0]);
    Z=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q0,A,O);
    R.zero_horizontal_speed_status=Z.status;
    R.zero_horizontal_speed_rejected= ...
        ~Z.conditional_nominal_all_time_torque_proven;

    R.stage='REJECT_INFEASIBLE_LOWER_TORQUE';
    Awrong=A;
    Awrong.torque_limits_Nm(:,1)=0.1;
    W=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,Awrong,O);
    R.incorrect_torque_window_status=W.status;
    R.incorrect_torque_window_rejected= ...
        ~W.conditional_nominal_all_time_torque_proven;
    R.detailed_fallback_preserved= W.detailed_interval_checked_nodes>0 && ...
        ~W.conditional_nominal_all_time_torque_proven;

    R.stage='REJECT_INFEASIBLE_VERTICAL_THRUST';
    Owrong=O;Owrong.minimum_force_z_N=1e9;
    V=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,Owrong);
    R.force_vertical_singularity_status=V.status;
    R.force_vertical_singularity_rejected= ...
        ~V.conditional_nominal_all_time_torque_proven;

    R.stage='CERTIFY_EXACT_CURRENT_POINT_MASS';
    Omass=O;
    % No invented future mass range. A degenerate interval is exactly
    % the current estimator/fixture point mass, not a future guarantee.
    Omass.certified_constant_physical_mass_interval_kg= ...
        [values(2),values(2)];
    Mm=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,Omass);
    R.mass_interval_status=Mm.status;
    R.mass_interval_leaf_reason=Mm.last_leaf_reason;
    R.mass_interval_interval_exception=Mm.last_exception_identifier;
    R.mass_interval_exception_message=Mm.last_exception_message;
    R.mass_interval_unresolved_interval_s=Mm.unresolved_interval_s;
    R.mass_interval_pass=Mm.conditional_nominal_all_time_torque_proven;

    R.stage='REJECT_MALFORMED_MASS_INTERVAL';
    Obad=O;Obad.certified_constant_physical_mass_interval_kg= ...
        [values(2),values(2),values(2)];
    Bad=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,A,Obad);
    R.invalid_mass_interval_status=Bad.status;
    R.invalid_mass_interval_rejected= ...
        ~Bad.conditional_nominal_all_time_torque_proven;

    R.stage='REJECT_ZERO_PAYLOAD_IN_TAUT_MODEL';
    Azero=A;
    Azero.payload_mass_kg=0;
    Zero=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Q,Azero,O);
    R.zero_mass_rejected=~Zero.conditional_nominal_all_time_torque_proven;

    R.stage='REJECT_TAMPERED_POLYNOMIAL_DERIVATIVES';
    Qtampered=Q;
    Qtampered.derivative_controls{2}(1,5)= ...
        Qtampered.derivative_controls{2}(1,5)+1e-3;
    Tampered=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(Qtampered,A,O);
    R.derivative_control_tampering_status=Tampered.status;
    R.derivative_control_tampering_rejected= ...
        ~Tampered.conditional_nominal_all_time_torque_proven && ...
        contains(Tampered.status,'JetsBinding');

    R.stage='VERIFY_NEVER_COMMITS';
    R.no_hlc_commit=~H.safe_to_commit&&~Y.safe_to_commit&& ...
        ~Mm.safe_to_commit&&~H.actual_HLC_tracking_proven&& ...
        ~H.numerically_directed_rounding_proven;
    fields={'real_hlc_limits_read','constant_heading_x_pass', ...
        'rotated_heading_y_pass','zero_horizontal_speed_rejected', ...
        'incorrect_torque_window_rejected', ...
        'force_vertical_singularity_rejected','mass_interval_pass', ...
        'invalid_mass_interval_rejected','zero_mass_rejected', ...
        'derivative_control_tampering_rejected','no_hlc_commit', ...
        'fast_norm_leaf_verified','fast_and_detailed_both_prove_nominal', ...
        'detailed_fallback_preserved'};
    if all(cellfun(@(name) R.(name),fields))
        R.status='PASS_NOMINAL_RIGID_BODY_TORQUE_INTERVAL__HLC_TRACKING_PENDING';
    else
        R.status='FAIL_C9_NOMINAL_TORQUE_INTERVAL';
        R.error_identifier='C9_TORQUE_CHECK:CertificateNotProven';
        R.error_message='One or more independent fixture assertions failed; inspect per-case statuses.';
    end
catch ME
    R.status='FAIL_C9_NOMINAL_TORQUE_INTERVAL';
    R.error_identifier=ME.identifier;
    R.error_message=ME.message;
    if ~isempty(ME.stack)
        lines=cell(numel(ME.stack),1);
        for j=1:numel(ME.stack)
            lines{j}=sprintf('%s:%d',ME.stack(j).name,ME.stack(j).line);
        end
        R.error_stack=strjoin(lines,' | ');
    end
end
base='TRAJECTORY_C9_NOMINAL_TORQUE_RESULT';
save(fullfile(root,[base '.mat']),'R');
fid=fopen(fullfile(root,[base '.txt']),'wt');
assert(fid>0,'C9_TORQUE_CHECK:Write','Cannot save audit TXT.');
c=onCleanup(@()fclose(fid));
f=fieldnames(R);
for i=1:numel(f)
    v=R.(f{i});
    if islogical(v)&&isscalar(v)
        fprintf(fid,'%s=%d\n',f{i},v);
    elseif ischar(v)
        fprintf(fid,'%s=%s\n',f{i},v);
    elseif isnumeric(v)&&isscalar(v)
        fprintf(fid,'%s=%.16g\n',f{i},v);
    elseif isnumeric(v)&&isvector(v)
        fprintf(fid,'%s=%s\n',f{i},mat2str(v(:).',16));
    end
end
clear c
out=fullfile(root,[base '_FOR_CHATGPT.zip']);
if isfile(out),delete(out);end
zip(out,{[base '.mat'],[base '.txt']},root);
assert(isfile(out)&&dir(out).bytes>100,'C9_TORQUE_CHECK:Export', ...
    'Failed to verify result ZIP.');
fprintf('[C9 NOMINAL TORQUE] %s | stage=%s | %s\n', ...
    R.status,R.stage,out);
end

function piece=make_affine(t0,T,p,v)
left=zeros(3,10);right=zeros(3,10);
left(:,1)=p;left(:,2)=v;
right(:,1)=p+T*v;right(:,2)=v;
free=repmat(p,1,4)+v*((T/23)*(10:13));
piece=TRAJECTORY_C9_FLAT_BERNSTEIN.build(t0,T,left,right,free);
end

function name=upper_identifier(text)
name=upper(regexprep(char(text),'[^A-Za-z0-9]','_'));
end

function p=percentile95_local(x)
% Definition fixed for a reproducible toolbox-free 7-run diagnostic.
x=sort(double(x(:)));
q=1+0.95*(numel(x)-1);
a=floor(q);b=ceil(q);
p=x(a)+(q-a)*(x(b)-x(a));
end
