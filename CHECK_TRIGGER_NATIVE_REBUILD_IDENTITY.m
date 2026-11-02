function D=CHECK_TRIGGER_NATIVE_REBUILD_IDENTITY(varargin)
% Read-only standalone MEX rebuild identity audit. Flight not required.
% Optional single argument: alternate .mexw64 path for direct comparison.
% NO agent, EKF, sensor, reference or plant state is evaluated.
root=fileparts(which('TRIGGER_NATIVE_MODEL_SOURCE_CHECK'));
assert(~isempty(root),'CHECK_TRIGGER_NATIVE_REBUILD_IDENTITY:Path',...
    'Native source checker must be on MATLAB path.');
local=fullfile(root,['TRIGGER_SYMBOLIC_NATIVE_CORE.' mexext]);
assert(isfile(local),'CHECK_TRIGGER_NATIVE_REBUILD_IDENTITY:Local',...
    'Native MEX missing in local trigger folder. Preflight must compile it.');
R=TRIGGER_NATIVE_PE_CODE_IDENTITY(local);
S=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(true);
D=struct('status','FAIL_CLOSED','active_mex',local, ...
    'active_binary_sha256',R.whole_binary_sha256,...
    'active_code_fingerprint',R.fingerprint,...
    'active_section_valid',R.valid,...
    'source_check',S,'same_computation_fingerprint',false,...
    'comparison_available',false,'continuous_safety_certified',false);
expected='f15ac1f3f4483fac6b004f13e515ca89353febb7feae13c9d8419b2b5cd07d6a';
D.same_computation_fingerprint=R.valid && strcmpi(R.fingerprint,expected);
D.cpp_source_matches_verified=S.cpp_source_matches_verified;
D.native_code_matches_verified=S.native_code_matches_verified;
D.verification_config_valid=S.verification_config_valid;
D.accepted_for_nominal_prediction=logical(S.passed);
if nargin>=1 && ~isempty(varargin{1})
    T=TRIGGER_NATIVE_PE_CODE_IDENTITY(varargin{1});
    D.comparison_available=T.valid;
    D.alternate_path=char(varargin{1});
    D.alternate_binary_sha256=T.whole_binary_sha256;
    D.alternate_code_fingerprint=T.fingerprint;
    D.specific_files_same_code=R.valid&&T.valid&&...
        strcmpi(R.fingerprint,T.fingerprint);
end
if S.passed
    D.status='VERIFIED_NATIVE_EXECUTABLE_IDENTITY__NOMINAL_ONLY';
else
    D.status=char(S.status);
end
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_NATIVE_REBUILD_IDENTITY.mat'),'D');
fid=fopen(fullfile(folder,'TRIGGER_NATIVE_REBUILD_IDENTITY.txt'),'wt');
if fid~=-1
    cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nsource_passed=%d\n',D.status,S.passed);
    fprintf(fid,'mex_full_sha256=%s\n',R.whole_binary_sha256);
    fprintf(fid,'mex_stable_executable_fingerprint=%s\n',R.fingerprint);
    fprintf(fid,'cpp_sha256=%s\n',S.cpp_sha256);
    fprintf(fid,'cpp_source_matches_verified=%d\n',D.cpp_source_matches_verified);
    fprintf(fid,'native_code_matches_verified=%d\n',D.native_code_matches_verified);
    fprintf(fid,'verification_config_valid=%d\n',D.verification_config_valid);
    fprintf(fid,'rebuild_same_verified_code=%d\n',S.reproducible_pe_identity);
    fprintf(fid,'accepted_for_nominal_prediction=%d\n',D.accepted_for_nominal_prediction);
    fprintf(fid,'geometry_and_closed_loop_certified=0\n');
end
fprintf('[MEX REBUILD] %s | HLC/plant=%d CPP=%d code=%d accepted=%d\n',...
    D.status,S.source_match,D.cpp_source_matches_verified,...
    D.native_code_matches_verified,D.accepted_for_nominal_prediction);
end
