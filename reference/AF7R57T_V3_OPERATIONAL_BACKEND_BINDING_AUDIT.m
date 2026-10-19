function OUT = AF7R57T_V3_OPERATIONAL_BACKEND_BINDING_AUDIT()
% AF7R57T_V3_OPERATIONAL_BACKEND_BINDING_AUDIT
% Read-only correction of V2 classification semantics.
% Distinguishes:
%   - default-wired production helper
%   - user-registered callback-only backend
%   - no-callback fail-closed branch
% No production modification. No timing claim upgrade.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57T V3 | OPERATIONAL BACKEND DEFAULT-BINDING / CALLBACK FAIL-CLOSED AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | currentSourceOnly=1 | helperBindingRecognized=1 | noBackendRemoval=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
selfFile=[mfilename('fullpath') '.m'];
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

rps=fullfile(root,'REPLANNING_SYSTEM.m');
assert(isfile(rps),'[AF7R57T V3] REPLANNING_SYSTEM.m missing.');
sha=sha256_file(rps);
expected='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
src=normnl(fileread(rps));

fprintf('\n[A. EXACT CURRENT SOURCE BINDING]\n');
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',sha);
fprintf(' current source matches bound authority  = %d\n',strcmp(sha,expected));
assert(strcmp(sha,expected),'[AF7R57T V3] current source differs from bound authority.');

% Parser self-test and exact method extraction.
synthetic=sprintf(['classdef X\nmethods\nfunction y = a(obj,x)\ny=x;\nend\n' ...
                   'function b(obj)\nq=1;\nend\nend\nend\n']);
assert(contains(method_body(synthetic,'a'),'y=x;') && contains(method_body(synthetic,'b'),'q=1;'), ...
    '[AF7R57T V3] method parser self-test failed.');
freeze=method_body(src,'freeze_generator_backend_configuration');
invoke=method_body(src,'invoke_replan_generator_shadow');

B={'BEZIER_D15_C7','POLY_D15_C7','MINCO_STYLE_D15_C7','BSPLINE_D8_C7'};
fprintf('\n[B. ROUTER CATALOG]\n');
cat=false(1,4);
for k=1:4
    cat(k)=contains(src,B{k});
    fprintf(' catalog backend %d %-24s = %d\n',k,B{k},cat(k));
end
fprintf(' four-name catalog present               = %d\n',all(cat));
fprintf(' method parser self-test                 = 1\n');

% Default wiring semantics.  V2 incorrectly required an in-class builtin.
branch1=contains(freeze,'generator_backend_locked_code)==1') || ...
        contains(freeze,'generator_backend_locked_code) == 1');
prime1=contains(freeze,'af7r36_runtime_prime(obj)');
helper1=contains(freeze,'af7r36_single_core_proposal(obj,CTX)');
builtin1=contains(freeze,'builtin_bezier_d15_c7_proposal(CTX)');
wire1=branch1 && (helper1 || builtin1);

% Search for explicit default branches for codes 2..4.  Catalog/name tokens alone do not count.
wire=false(1,4); wire(1)=wire1;
for k=2:4
    p1=sprintf('generator_backend_locked_code)==%d',k);
    p2=sprintf('generator_backend_locked_code) == %d',k);
    wire(k)=contains(freeze,p1)||contains(freeze,p2);
end

% Resolve helper files as actual current source dependencies.
primePath=which('af7r36_runtime_prime');
proposalPath=which('af7r36_single_core_proposal');
primePresent=~isempty(primePath)&&isfile(primePath);
proposalPresent=~isempty(proposalPath)&&isfile(proposalPath);

fprintf('\n[C. FREEZE-TIME DEFAULT WIRING]\n');
fprintf(' backend 1 code==1 default branch        = %d\n',branch1);
fprintf(' backend 1 af7r36_runtime_prime binding  = %d\n',prime1);
fprintf(' backend 1 af7r36 proposal binding       = %d\n',helper1);
fprintf(' backend 1 in-class builtin alternative  = %d\n',builtin1);
fprintf(' backend 1 default-wired source          = %d\n',wire1);
fprintf(' af7r36_runtime_prime file present       = %d\n',primePresent);
fprintf(' af7r36_single_core_proposal file present= %d\n',proposalPresent);
if primePresent
    fprintf(' af7r36_runtime_prime path               = %s\n',primePath);
    fprintf(' af7r36_runtime_prime SHA-256            = %s\n',sha256_file(primePath));
end
if proposalPresent
    fprintf(' af7r36_single_core_proposal path        = %s\n',proposalPath);
    fprintf(' af7r36_single_core_proposal SHA-256     = %s\n',sha256_file(proposalPath));
end
for k=2:4
    fprintf(' backend %d explicit default wiring       = %d\n',k,wire(k));
end

callbackTable=contains(freeze,'generator_backend_callbacks');
legacyFallback=contains(freeze,'repaired_candidate_generator_fn');
callbackType=contains(freeze,'isa(fn,''function_handle'')');
fprintf('\n[D. CALLBACK ROUTER CONTRACT]\n');
fprintf(' generic per-backend callback table      = %d\n',callbackTable);
fprintf(' legacy/global callback fallback         = %d\n',legacyFallback);
fprintf(' callback type checked                   = %d\n',callbackType);
fprintf(' callback representation contract here   = %d\n',contains(freeze,'canonical_native_representation'));

% Exact no-callback / callback-failure behavior in invocation method.
emptyLocked=contains(invoke,'if isempty(obj.generator_backend_locked_callback)');
emptyToCandidate=contains(invoke,'candidate=[];');
callbackCatch=contains(invoke,'callback_ok=false');
callbackReject=contains(invoke,'if ~callback_ok') && contains(invoke,'generator_last_status_code=-1');
noCandidateReject=contains(invoke,'if isempty(candidate)') && ...
                  contains(invoke,'generator_last_status_code=1');
returns=contains(invoke,'return;');
failClosedInvoke=emptyLocked&&emptyToCandidate&&callbackCatch&&callbackReject&&noCandidateReject&&returns;

fprintf('\n[E. INVOCATION FAIL-CLOSED CONTRACT]\n');
fprintf(' empty locked callback detected          = %d\n',emptyLocked);
fprintf(' empty callback produces candidate=[]    = %d\n',emptyToCandidate);
fprintf(' callback exception caught               = %d\n',callbackCatch);
fprintf(' callback failure returns rejected       = %d\n',callbackReject);
fprintf(' no-candidate returns without commit     = %d\n',noCandidateReject);
fprintf(' invocation fail-closed contract         = %d\n',failClosedInvoke);

% Candidate backend/version binding. Use full source because validation may live elsewhere.
backendRef=contains(src,'candidate.backend');
lockedName=contains(src,'generator_backend_locked_name');
backendCompare=(contains(src,'strcmpi(char(candidate.backend)') || contains(src,'strcmp(char(candidate.backend)')) && lockedName;
versionMach=contains(src,'active_reference_version') && contains(src,'targetVersion');
canonicalReq=contains(src,'canonical_native_representation');

fprintf('\n[F. CANDIDATE BACKEND / VERSION BINDING]\n');
fprintf(' candidate.backend referenced            = %d\n',backendRef);
fprintf(' locked backend name referenced          = %d\n',lockedName);
fprintf(' explicit candidate-vs-locked compare    = %d\n',backendCompare);
fprintf(' reference-version machinery present     = %d\n',versionMach);
fprintf(' canonical native representation required= %d\n',canonicalReq);

% Operational classification.
cls=cell(1,4);
if wire1 && proposalPresent
    cls{1}='DEFAULT_WIRED_NATIVE_HELPER__CURRENT_SOURCE';
elseif wire1
    cls{1}='DEFAULT_WIRED_HELPER_REFERENCE__HELPER_FILE_MISSING';
else
    cls{1}='NO_DEFAULT_WIRING';
end
for k=2:4
    if wire(k)
        cls{k}='DEFAULT_WIRING_PRESENT__AUDIT_EXACT_SOURCE';
    else
        cls{k}='CALLBACK_ONLY__FAIL_CLOSED_IF_UNREGISTERED';
    end
end

fprintf('\n[G. CURRENT OPERATIONAL CLASSIFICATION]\n');
for k=1:4
    fprintf(' %-24s = %s\n',B{k},cls{k});
end
fprintf(' catalog backends silently removed       = 0\n');
fprintf(' callback-only treated as proved backend = 0\n');

routerClosed=all(cat) && wire1 && proposalPresent && callbackTable && callbackType && failClosedInvoke;
currentDefaultCount=sum(wire & [proposalPresent true true true]);
backend1Current=(wire1&&proposalPresent);
backends234CallbackOnly=~wire(2)&&~wire(3)&&~wire(4);

fprintf('\n[H. FORMAL SCOPE GATE]\n');
fprintf(' router catalog/implementation distinction closed = %d\n',routerClosed);
fprintf(' current default-wired backend count      = %d\n',double(backend1Current));
fprintf(' current default theorem backend          = %s\n',ternary(backend1Current,'BEZIER_D15_C7','OPEN'));
fprintf(' backends 2-4 callback-only              = %d\n',backends234CallbackOnly);
fprintf(' backends 2-4 retained in router catalog = %d\n',all(cat(2:4)));
fprintf(' backends 2-4 exact production theorem   = 0\n');
fprintf(' canonical native contract still open    = %d\n',~canonicalReq);
fprintf(' production source modified              = 0\n');
fprintf(' timing claim upgraded                   = 0\n');
fprintf(' next action                              = CLOSE_CURRENT_BEZIER_AF7R36_TO_NATIVE_CANONICAL_SERIALIZATION__THEN_G_S_PREPARSE_GUARD\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57T_V3_OPERATIONAL_BACKEND_BINDING';
OUT.closed=routerClosed;
OUT.catalog=cat;
OUT.default_wiring=wire;
OUT.classification=cls;
OUT.backend1_helper_present=proposalPresent;
OUT.fail_closed_invoke=failClosedInvoke;
OUT.backend_compare=backendCompare;
OUT.canonical_native_required=canonicalReq;
OUT.production_modified=false;
OUT.entrypoint=selfFile;
OUT.entrypoint_sha256=selfSHA;
assignin('base','AF7R57T_V3_OPERATIONAL_BACKEND_BINDING_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57T_V3_OPERATIONAL_BACKEND_BINDING_AUDIT_OUT.mat'),'OUT');
end

function body=method_body(src,name)
L=splitlines(string(src));
start=0;
for i=1:numel(L)
    z=strtrim(L(i));
    if startsWith(z,'function ')
        s=char(z);
        if contains(s,[name '('])
            start=i; break;
        end
    end
end
assert(start>0,'[AF7R57T V3] method not found: %s',name);
stop=numel(L);
for i=start+1:numel(L)
    z=strtrim(L(i));
    if startsWith(z,'function ')
        stop=i-1; break;
    end
end
body=char(strjoin(L(start:stop),newline));
end

function y=ternary(c,a,b)
if c, y=a; else, y=b; end
end

function s=normnl(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function h=sha256_file(f)
fid=fopen(f,'r'); assert(fid>=0,'cannot open %s',f);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
