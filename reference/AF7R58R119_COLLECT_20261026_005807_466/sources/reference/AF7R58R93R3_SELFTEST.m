function S=AF7R58R93R3_SELFTEST()
% No installation or production changes. Source-bound reference-only tests.
base=fileparts(mfilename('fullpath'));
runner=which('AF7R58R93R3_RUN_SAVED_ROUTE');
wrapper=which('AF7R58R93R3_ADAPTIVE_SAVED_SEARCH');
if ~strcmpi(fileparts(runner),base)||~strcmpi(fileparts(wrapper),base)
    error('R93R3:PathShadow','R93R3 functions are not resolved from the same reference directory.');
end
wanted='4ad8aba0c1ded835d5f9a2cb560aaad5a3a07024cee3e259314aec41ef897c6b';
actual=AF7R58R93_SHA256_FILE(runner);
if ~strcmpi(actual,wanted)
    error('R93R3:SourceIntegrity','Independent runner changed: expected %s, observed %s',wanted,actual);
end
code=fileread(runner);
required={'R.seed_attempt_log','R.best_partial_route', ...
    'R.partial_reference_found','R.production_commit_authorized', ...
    'all_segment_reference_conditions_pass'};
for i=1:numel(required)
    if ~contains(code,required{i})
        error('R93R3:MissingTraceContract','Required source token missing: %s',required{i});
    end
end
T=AF7R58R92_SELFTEST();
if ~strcmp(T.status,'PASS_R92_MULTISEGMENT_C7_MIN8_AND_FAIL_CLOSED_REFERENCE_GATE')
    error('R93R3:R92SyntheticRegression','Existing R92 synthetic test did not pass.');
end
S=struct('status','PASS_R93R3_PARALLEL_RUNNER_SOURCE_AND_R92_TEST', ...
    'independent_runner_sha256',actual, ...
    'r92_existing_runner_untouched',true, ...
    'real_snapshot_replay_performed',false, ...
    'flight_class_changed',false,'trigger_switched',false, ...
    'closed_loop_safety_proved',false);
end
