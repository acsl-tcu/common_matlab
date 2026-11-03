function R=TRIGGER_TIMEONLY_SOURCE_PREPARE(fn,audit)
% Preflight software FIRST-USE of the ACTUAL time-only reference function.
% No unknown obstacle, future sensor frame, EKF, time of Flight onset, or
% future phase clock is read. Argument 0 is a fixed SOURCE test argument.
% No result/value is stored, injected into the HLC, or cached as a Flight jet.
% Only invoke when the existing source-purity audit declares eligibility.
% There is NO claim that any MATLAB JIT tier or future Flight latency is
% guaranteed by repeated calls; failures must not fabricate references.
R=struct('status','NOT_ATTEMPTED','calls',0, ...
    'stored_reference_values',false,'used_flight_epoch',false, ...
    'safe_to_commit',false,'reason','');
try
    assert(isa(fn,'function_handle') && isstruct(audit) && ...
        isfield(audit,'eligible') && islogical(audit.eligible) && ...
        isscalar(audit.eligible), ...
        'TRIGGER_TIMEONLY_SOURCE_PREPARE:Audit','Expected audited function.');
    if ~audit.eligible
        R.status='SKIPPED_SOURCE_NOT_AUDITED_PURE';return
    end
    for k=1:8
        % Each result is discarded immediately, no preflight trajectory.
        v=fn(0);
        assert(isnumeric(v)&&isreal(v)&&iscolumn(v)&& ...
            numel(v)>=4&&numel(v)<=28&&all(isfinite(v)), ...
            'TRIGGER_TIMEONLY_SOURCE_PREPARE:Jet', ...
            'Source-only scalar probe produced invalid reference jet.');
        R.calls=k;
    end
    R.status='PASS_REFERENCE_SOURCE_EXECUTED_8__NO_JIT_GUARANTEE';
catch ME
    R.status='SOURCE_FIRSTUSE_PROBE_NOT_COMPLETED';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
