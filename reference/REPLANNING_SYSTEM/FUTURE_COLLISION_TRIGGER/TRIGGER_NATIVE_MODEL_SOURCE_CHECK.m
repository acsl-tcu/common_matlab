function R = TRIGGER_NATIVE_MODEL_SOURCE_CHECK(require_audit)
% Read-only exact source/compiled-binary pinning for a proposed native
% closed-loop nominal predictor. NO sensor/preflight/Flight state changes.
% Mismatch => fail closed; this is NOT a continuous-time safety proof.
if nargin<1,require_audit=true;end
home=fileparts(mfilename('fullpath'));
main=which('HLC_SUSPENDED_LOAD'); rhs=which('with_load_model_euler_for_HL');
base={...
 'HLC_SUSPENDED_LOAD.m', ...
 'Vfd_SuspendedLoadxyDst.m', ...
 'Vs_SuspendedLoadxyDst.m', ...
 'Uf_SuspendedLoadxyDst.m', ...
 'Beta2_SuspendedLoadxyDst.m', ...
 'V2_alpha2_SuspendedLoadxyDst.m', ...
 'with_load_model_euler_for_HL.m'};
expected={...
 '09b38e167a43979a41660fe32088f44255bb6d3e9591093701b4a41712bced5a',...
 '16b7ca30550dc19dc609cd9e8cebb7202c3e39c29d0efdb6b76d5f308d88dd6f',...
 '0a5519b27f3580286351c09c98b46cac98dae749ead5729006fd06baeee945f7',...
 'ed69708edff75ef997a891ee0e00464e53fffe925e574e82bf59657742013deb',...
 '053428eefd259a3ceea63c12806df4dc5d8462b9f9ce39d12f10ed37f8755094',...
 'e4a4f320c83d350c142b468b9fd793a51fc06ddb69abac733cacdd9f54eb6825',...
 '183837c484f6ec8953998aacb20b93120ce0d6fde647c0e54bcc27f5ae02164a'};
actual=cell(size(base));paths=cell(size(base));matched=false(size(base));
if ~isempty(main)
 for i=1:6,paths{i}=fullfile(fileparts(main),base{i});end
end
paths{7}=rhs;
for i=1:7
 if ~isempty(paths{i}) && isfile(paths{i})
  actual{i}=local_sha256(paths{i});
  matched(i)=strcmpi(actual{i},expected{i});
 else
  actual{i}='MISSING';
 end
end
mxfile=which('TRIGGER_SYMBOLIC_NATIVE_CORE');
compiled=~isempty(mxfile) && ...
    strcmpi(mxfile(end-min(length(mxfile)-1,length(mexext)):end),['.' mexext]);
compiled_sha='';
if compiled,compiled_sha=local_sha256(mxfile);end
source_pass=all(matched);
% Full-file MEX SHA is NOT reproducible across equivalent rebuilds:
% the MATLAB resource manifest and COFF timestamp change. The code/data
% fingerprint below is restricted to the PREVIOUS audited binary.
% The optimized C++ requires a FRESH full offline numerical parity report
% with the rebuilt binary SHA before it can be approved for Flight.
% Any changed C++ source or executable/data section fails closed until
% a fresh independent HLC+plant parity audit is completed.
cpp_path=fullfile(home,'TRIGGER_SYMBOLIC_NATIVE_CORE.cpp');
trusted_cpp_sha='5433a4826ea2a05ef17bb7f222c7d27b88431d8662580c27f356305ffa2f7648';
trusted_code_identity='f15ac1f3f4483fac6b004f13e515ca89353febb7feae13c9d8419b2b5cd07d6a';
cpp_sha='';
if isfile(cpp_path),cpp_sha=local_sha256(cpp_path);end
identity=struct('valid',false,'status','NOT_CHECKED','fingerprint','');
if compiled
    identity=TRIGGER_NATIVE_PE_CODE_IDENTITY(mxfile);
end
% Never silently accept a truncated/mistyped expected digest again.
verification_config_valid=length(trusted_cpp_sha)==64 && ...
    length(trusted_code_identity)==64 && ...
    all(isstrprop(trusted_cpp_sha,'xdigit')) && ...
    all(isstrprop(trusted_code_identity,'xdigit'));
cpp_matches=verification_config_valid && strcmpi(cpp_sha,trusted_cpp_sha);
code_matches=verification_config_valid && compiled && identity.valid && ...
    strcmpi(identity.fingerprint,trusted_code_identity);
rebuild_equivalent=source_pass && compiled && cpp_matches && code_matches;
audit_found=false;audit_pass=false;audit_status='NOT_RUN';
auditfile=fullfile(home,'TRIGGER_NATIVE_PARITY_RESULT.mat');
if isfile(auditfile)
 try
  saved=load(auditfile,'A');a=saved.A;
  audit_found=true;
  audit_status=char(a.status);
  audit_pass=isfield(a,'parity_pass') && isequal(a.parity_pass,true) && ...
      isfield(a,'native_mex_sha256') && strcmpi(a.native_mex_sha256,compiled_sha) && ...
      isfield(a,'source_sha256') && isequal(a.source_sha256,actual);
 catch ME
  audit_status=['AUDIT_UNREADABLE: ' ME.identifier];
 end
end
R=struct('status','FAIL_CLOSED','source_match',source_pass,...
    'source_files',{base},'source_paths',{paths},...
    'source_sha256',{actual},'expected_sha256',{expected},...
    'file_matches',matched,'native_exists',compiled,...
    'native_path',mxfile,'native_mex_sha256',compiled_sha,...
    'cpp_sha256',cpp_sha,'expected_cpp_sha256',trusted_cpp_sha,...
    'cpp_source_matches_verified',logical(cpp_matches),...
    'native_code_fingerprint',identity.fingerprint,...
    'native_code_matches_verified',logical(code_matches),...
    'verification_config_valid',logical(verification_config_valid),...
    'reproducible_pe_identity',logical(rebuild_equivalent),...
    'native_identity_status',identity.status,...
    'audit_found',audit_found,'audit_status',audit_status,'audit_pass',audit_pass,...
    'require_audit',logical(require_audit),'passed',false);
% A byte-identical previously audited binary and a code-identical rebuild
% are both accepted. Neither is allowed with a mismatched local C++ source.
R.passed=source_pass && compiled && cpp_matches && ...
    (~require_audit || audit_pass || rebuild_equivalent);
if R.passed && audit_pass
    R.status='EXACT_SOURCE_PIN_AND_NATIVE_AUDIT_PASS__NOMINAL_ONLY';
elseif R.passed && rebuild_equivalent
    R.status='REBUILT_MEX_SAME_VERIFIED_EXECUTABLE_CODE__NOMINAL_ONLY';
elseif ~verification_config_valid
    R.status='UNKNOWN_INVALID_VERIFICATION_CONFIGURATION';
elseif ~source_pass
    R.status='UNKNOWN_HLC_OR_PLANT_SOURCE_MISMATCH';
elseif ~cpp_matches
    R.status='UNKNOWN_CPP_SOURCE_NOT_VERIFIED';
elseif ~compiled
    R.status='UNKNOWN_NATIVE_NOT_COMPILED';
elseif require_audit
    R.status='UNKNOWN_MEX_EXECUTABLE_NOT_VERIFIED__RUN_PARITY';
end
end

function hx=local_sha256(filename)
f=fopen(filename,'rb');
if f<0,error('TRIGGER_NATIVE_MODEL_SOURCE_CHECK:Open','Cannot read %s',filename);end
c=onCleanup(@()fclose(f));
data=fread(f,inf,'*uint8');
h=java.security.MessageDigest.getInstance('SHA-256');
h.update(typecast(data,'int8'));
raw=typecast(int8(h.digest()),'uint8');
hx=lower(reshape(dec2hex(raw,2).',1,[]));
end
