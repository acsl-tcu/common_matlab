function Q=AF7R58R103_ATOMIC_HANDOVER(replanner,stage,tCommit)
% Explicit bound between NEW candidate and ORIGINAL atomic commit.
% No new certificate flags are invented here: the existing production
% validator decides, and its rollback is retained.
Q=struct('status','BLOCKED_NOT_CERTIFIED','committed',false, ...
    'reason','NO_NEW_FULL_EXECUTION_CERTIFICATE');
assert(isa(replanner,'REPLANNING_SYSTEM')&&isscalar(replanner), ...
    'R103:LiveReplannerHandleRequired', ...
    'Only the unchanged native replanner owns the atomic commit.');
assert(isstruct(stage)&&isscalar(stage)&& ...
    isscalar(tCommit)&&isfinite(tCommit), ...
    'R103:StageAndCommitTimeRequired','A current event and time are required.');
if ~isfield(stage,'ready_for_atomic_commit')|| ...
        ~logical(stage.ready_for_atomic_commit)
    return;
end
if ~isfield(stage,'physical_trigger')|| ...
        ~isstruct(stage.physical_trigger)|| ...
        ~isfield(stage.physical_trigger,'complete_preview')|| ...
        ~stage.physical_trigger.complete_preview
    Q.reason='PHYSICAL_TRIGGER_SOURCE_UNBOUND';return;
end
if ~isfield(stage,'candidate')||~isstruct(stage.candidate)|| ...
        ~isscalar(stage.candidate)||~isfield(stage.candidate,'certified')|| ...
        ~logical(stage.candidate.certified)
    Q.reason='NO_FULLY_CERTIFIED_CANDIDATE';return;
end
if ~isfield(stage,'native_execution_certificate_closed')|| ...
        ~logical(stage.native_execution_certificate_closed)
    Q.reason='HLC_CLOSED_LOOP_CERTIFICATE_OPEN';return;
end
% No bypass of deadline, smoothness, geometry, actuator, robust tube,
% backup, known obstacle, phase, numerical or dependency checks.
try
    replanner.commit_repaired_reference(stage.candidate,double(tCommit));
    Q.status='COMMITTED_BY_UNMODIFIED_NATIVE_ATOMIC_VALIDATOR';
    Q.committed=true;Q.reason='';
catch ME
    Q.status='REJECTED_BY_NATIVE_VALIDATOR_ROLLBACK';
    Q.reason=[char(ME.identifier) ':' char(ME.message)];
end
end
