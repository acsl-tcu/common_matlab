function T=AF7R58R88_SELFTEST()
% PURE synthetic tests; no production write, MEX, EKF, flight or HLC calls.
root=fileparts(mfilename('fullpath'));
patch=fileread(fullfile(root,'REPLANNING_SYSTEM_R88_PATCHED_SOURCE.txt'));
assert(contains(patch,'obj.r88_capture_one_version(double(time.t),active);'));
assert(contains(patch,'r88_replay_saved_version_reference_only'));
assert(contains(patch,'R88_NO_EXACT_TIME_BOUND_SNAPSHOT__NO_R81'));
assert(contains(patch,'estimated_mass_kg'',mL'));
assert(~contains(patch,'estimated_mass_kg'',double(obj.loadmass)'));
S=struct('time_s',{13.325,19.675,19.75}, ...
    'reference_version',{1,2,3});
assert(isequal(AF7R58R88_SELECT_TIME(S,19.75),3));
assert(isempty(AF7R58R88_SELECT_TIME(S,19.725)));
assert(isempty(AF7R58R88_SELECT_TIME(S,19.75001)));
A=S([1 1]);assert(isempty(AF7R58R88_SELECT_TIME(A,13.325)));
fns=cell(1,8);
for k=1:8
    q=k;
    fns{k}=@(tau)[tau.^q;2*tau.^q;3*tau.^q];
end
J=AF7R58R88_NOMINAL_JETS(fns,5,6);
assert(isequal(size(J),[3,8])&&J(1,1)==1 && J(3,8)==3);
T=struct('status','PASS_R88_EXACT_TIME_MASS_AND_SOURCE_SNAPSHOT_POLICY', ...
    'same_event_timestamp_required',true, ...
    'missing_timestamp_fails_closed',true, ...
    'duplicate_timestamp_fails_closed',true, ...
    'native_computation_performed',false, ...
    'matlab_live_replay_performed',false, ...
    'production_modified',false, ...
    'execution_safety_proved',false);
end
