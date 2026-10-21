function S = AF7R36_OBSERVER_CHART_FAILURE_AUDIT_R18_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R18-POST] replanner_instance unavailable.');

R=evalin('base','replanner_instance');
O=R.get_observer_hard_support_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 OBSERVER CHART FAILURE AUDIT R18 RESULT\n');
fprintf(' exact one-step ZYX chart theorem witness\n');
fprintf('====================================================================================================\n');

if ~isfield(O,'last_forward_failure') || isempty(O.last_forward_failure) || ...
        ~isstruct(O.last_forward_failure) || ~isfield(O.last_forward_failure,'Theta')
    fprintf('No persisted forward-support chart failure is available.\n');
    S=struct('schema','AF7_OBSERVER_CHART_FAILURE_AUDIT_R18', ...
        'available',false,'status',O);
    return;
end

F=O.last_forward_failure;

fprintf('closure reason                             = %s\n',char(F.closure_reason));
fprintf('step h                                     = %.15g s\n',double(F.h));
fprintf('q0 abs [roll pitch yaw]                    = [%.15g %.15g %.15g] rad\n', ...
    double(F.q_abs_start(1)),double(F.q_abs_start(2)),double(F.q_abs_start(3)));
fprintf('W0 true norm UB                            = %.15g rad/s\n',double(F.w_norm_ub_start));
fprintf('energy-equivalent Wabs0 UB                 = %.15g rad/s\n',double(F.w_norm_ub_energy_start));
fprintf('held torque norm                           = %.15g N*m\n',double(F.torque_norm));
fprintf('Iw = integral ||omega|| UB                 = %.15g rad\n',double(F.Iw));
fprintf('Theta = q0_pitch + Iw                      = %.15g rad\n',double(F.Theta));
fprintf('pi/2                                       = %.15g rad\n',pi/2);
fprintf('pitch chart margin                         = %+.15e rad\n',double(F.pitch_margin_rad));
fprintf('roll chart margin                          = %+.15e rad\n',double(F.roll_margin_rad));
fprintf('yaw chart margin                           = %+.15e rad\n',double(F.yaw_margin_rad));
fprintf('q interval UB [roll pitch yaw]             = [%.15g %.15g %.15g] rad\n', ...
    double(F.q_abs_ub_interval(1)),double(F.q_abs_ub_interval(2)),double(F.q_abs_ub_interval(3)));
fprintf('Jmin / Jmax                                = %.15g / %.15g kg*m^2\n', ...
    double(F.Jmin),double(F.Jmax));

if ~logical(F.pitch_closed)
    cls='PITCH_CHART_PROOF_FAILED';
elseif double(F.roll_margin_rad)<=0 && double(F.yaw_margin_rad)<=0
    cls='ROLL_AND_YAW_CHART_PROOF_FAILED';
elseif double(F.roll_margin_rad)<=0
    cls='ROLL_CHART_PROOF_FAILED';
elseif double(F.yaw_margin_rad)<=0
    cls='YAW_CHART_PROOF_FAILED';
else
    cls='UNEXPECTED_CLOSED_CHART_STATE';
end

fprintf('\n[MATHEMATICAL DECISION]\n');
fprintf('classification                             = %s\n',cls);

if strcmp(cls,'PITCH_CHART_PROOF_FAILED')
    fprintf(['The current theorem failed because q0_pitch + integral||omega|| reached pi/2. ' ...
             'This does NOT by itself prove the true pitch reached pi/2; it proves the present norm-based enclosure is insufficient.\n']);
else
    fprintf(['Pitch support stayed inside (-pi/2,pi/2), but the propagated roll/yaw enclosure reached its chart boundary. ' ...
             'The next theorem must sharpen that coordinate-specific propagation.\n']);
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_OBSERVER_CHART_FAILURE_AUDIT_R18';
S.available=true;
S.status=O;
S.failure=F;
S.classification=cls;
end
