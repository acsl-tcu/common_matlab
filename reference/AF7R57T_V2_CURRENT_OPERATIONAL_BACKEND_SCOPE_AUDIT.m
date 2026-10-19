function OUT = AF7R57T_V2_CURRENT_OPERATIONAL_BACKEND_SCOPE_AUDIT()
% AF7R57T_V2_CURRENT_OPERATIONAL_BACKEND_SCOPE_AUDIT
% Read-only/source-bound audit of CURRENT production backend operability.
% No production modification, no backend removal, no timing claim upgrade.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57T V2 | CURRENT OPERATIONAL BACKEND SCOPE / FAIL-CLOSED ROUTER AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | currentSourceOnly=1 | noBackendRemoval=1 | noTimingClaimUpgrade=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
selfFile=[mfilename('fullpath') '.m'];
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

rps=fullfile(root,'REPLANNING_SYSTEM.m');
assert(isfile(rps),'[AF7R57T] REPLANNING_SYSTEM.m missing.');
sha=sha256_file(rps);
expected='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
assert(strcmp(sha,expected),'[AF7R57T] REPLANNING_SYSTEM source drifted from R57O/R57S authority.');
src=normnl(fileread(rps));

fprintf('\n[A. EXACT CURRENT SOURCE BINDING]\n');
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',sha);
fprintf(' current source matches bound authority  = 1\n');

names={'BEZIER_D15_C7','POLY_D15_C7','MINCO_STYLE_D15_C7','BSPLINE_D8_C7'};
fprintf('\n[B. ROUTER CATALOG != IMPLEMENTATION SET]\n');
for k=1:4
    fprintf(' catalog backend %d %-22s = %d\n',k,names{k},contains(src,names{k}));
end
hasCatalog=all(cellfun(@(x)contains(src,x),names));
fprintf(' four-name catalog present               = %d\n',hasCatalog);

parserSelfTest=method_parser_self_test();
fprintf(' method parser self-test                 = %d\n',parserSelfTest);
assert(parserSelfTest,'[AF7R57T V2] method parser self-test failed.');

freeze=method_body(src,'freeze_generator_backend_configuration');
invoke=method_body(src,'invoke_replan_generator_shadow');
normalize=method_body(src,'normalize_generator_backend_callbacks');

fprintf('\n[C. FREEZE-TIME IMPLEMENTATION SOURCE]\n');
default1 = contains(freeze,'double(obj.generator_backend_locked_code)==1') && ...
           (contains(freeze,'af7r36_single_core_proposal') || contains(freeze,'builtin_bezier_d15_c7_proposal'));
default2 = contains(freeze,'double(obj.generator_backend_locked_code)==2');
default3 = contains(freeze,'double(obj.generator_backend_locked_code)==3');
default4 = contains(freeze,'double(obj.generator_backend_locked_code)==4');
fprintf(' backend 1 self-contained/default source = %d\n',default1);
fprintf(' backend 2 self-contained/default source = %d\n',default2);
fprintf(' backend 3 self-contained/default source = %d\n',default3);
fprintf(' backend 4 self-contained/default source = %d\n',default4);
fprintf(' generic per-backend callback table      = %d\n',contains(freeze,'generator_backend_callbacks'));
fprintf(' legacy/global callback fallback         = %d\n',contains(freeze,'repaired_candidate_generator_fn'));

fprintf('\n[D. CALLBACK CONTRACT]\n');
callbackHandleOnly=contains(normalize,'isa(C.(n),''function_handle'')');
fprintf(' callback type check = function_handle   = %d\n',callbackHandleOnly);
fprintf(' callback representation contract here   = %d\n',contains(normalize,'representation'));
fprintf(' callback native-packet contract here    = %d\n',contains(normalize,'canonical_native_representation'));

% Exact invocation-side evidence around callback usage.
lockedHits=strfind(invoke,'generator_backend_locked_callback');
isemptyHits=strfind(invoke,'isempty');
returnHits=strfind(invoke,'return');
statusHits=strfind(invoke,'generator_last_status_code');
fprintf('\n[E. INVOCATION-SIDE FAIL-CLOSED FORENSICS]\n');
fprintf(' locked callback referenced              = %d\n',~isempty(lockedHits));
fprintf(' isempty checks present in invoke         = %d\n',~isempty(isemptyHits));
fprintf(' explicit returns present in invoke       = %d\n',~isempty(returnHits));
fprintf(' status-code writes present in invoke     = %d\n',~isempty(statusHits));

% Print only source windows around callback references and isempty checks.
fprintf('\n[E2. EXACT INVOCATION SOURCE WINDOWS]\n');
print_windows(invoke,{'generator_backend_locked_callback','isempty(fn)','isempty(OUT)','generator_last_status_code'},10);

% Find backend consistency/validator evidence anywhere in current source.
fprintf('\n[F. CANDIDATE BACKEND / VERSION BINDING]\n');
backendField=contains(src,'candidate.backend');
backendName=contains(src,'generator_backend_locked_name');
backendCompare = ~isempty(regexp(src,'(?s)candidate\.backend.{0,300}(strcmpi|strcmp)|(?:strcmpi|strcmp).{0,300}candidate\.backend','once'));
versionBinding = contains(src,'reference_version') && contains(src,'active_reference_version');
canonicalNativeRequired = contains(src,'canonical_native_representation');
fprintf(' candidate.backend referenced            = %d\n',backendField);
fprintf(' locked backend name referenced          = %d\n',backendName);
fprintf(' explicit candidate-vs-locked compare    = %d\n',backendCompare);
fprintf(' reference-version machinery present     = %d\n',versionBinding);
fprintf(' canonical native representation required= %d\n',canonicalNativeRequired);

% Current operational classification.
selfContained=[default1 default2 default3 default4];
callbackOnly=~selfContained;

fprintf('\n[G. CURRENT OPERATIONAL CLASSIFICATION]\n');
for k=1:4
    if selfContained(k)
        cls='SELF_CONTAINED_CURRENT_SOURCE';
    else
        cls='CALLBACK_ONLY__NO_BUILTIN_CURRENT_SOURCE';
    end
    fprintf(' %-24s = %s\n',names{k},cls);
end
fprintf(' catalog backends silently removed       = 0\n');
fprintf(' callback-only treated as proved backend = 0\n');

% Conservative theorem scope.
currentProdSet = find(selfContained);
routerScopeClosed = hasCatalog && default1 && ~default2 && ~default3 && ~default4;
canonicalStillOpen = ~canonicalNativeRequired;

fprintf('\n[H. FORMAL SCOPE GATE]\n');
fprintf(' router catalog/implementation distinction closed = %d\n',routerScopeClosed);
fprintf(' current self-contained backend count    = %d\n',numel(currentProdSet));
if isequal(currentProdSet,1)
    fprintf(' current self-contained theorem backend  = BEZIER_D15_C7\n');
else
    fprintf(' current self-contained theorem backend  = SOURCE_DEPENDENT_SET\n');
end
fprintf(' backends 2-4 retained in router catalog = %d\n',hasCatalog);
fprintf(' backends 2-4 exact production theorem   = 0\n');
fprintf(' canonical native contract still open    = %d\n',canonicalStillOpen);
fprintf(' production source modified              = 0\n');
fprintf(' timing claim upgraded                   = 0\n');
fprintf(' next action                              = CLOSE_BEZIER_CURRENT_SOURCE_CANONICAL_SERIALIZATION_FIRST__KEEP_2_TO_4_FAIL_CLOSED_UNTIL_CALLBACK_EXACT_CONTRACT_EXISTS\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57T_V2_CURRENT_OPERATIONAL_BACKEND_SCOPE_AUDIT_V1';
OUT.backend_names=names;
OUT.self_contained=selfContained;
OUT.callback_only=callbackOnly;
OUT.router_scope_closed=routerScopeClosed;
OUT.current_self_contained_backend_indices=currentProdSet;
OUT.canonical_native_contract_open=canonicalStillOpen;
OUT.production_modified=false;
OUT.entrypoint=selfFile;
OUT.entrypoint_sha256=selfSHA;
assignin('base','AF7R57T_V2_CURRENT_OPERATIONAL_BACKEND_SCOPE_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57T_V2_CURRENT_OPERATIONAL_BACKEND_SCOPE_AUDIT_OUT.mat'),'OUT');
end

function body=method_body(src,name)
% Line-oriented MATLAB method extractor.
% Deliberately avoids regexp backslash semantics for function discovery.
src=normnl(src);
L=splitlines(string(src));
T=strtrim(L);
isFcn=false(size(T));
for i=1:numel(T)
    ti=char(T(i));
    isFcn(i)=strcmp(ti,'function') || startsWith(ti,'function ');
end

hit=[];
target=[char(name) '('];
for i=1:numel(T)
    if ~isFcn(i), continue; end
    compact=regexprep(char(T(i)),'\s+','');
    if contains(compact,target)
        hit(end+1)=i; %#ok<AGROW>
    end
end
assert(numel(hit)==1,'[AF7R57T V2] expected exactly one method declaration for %s, got %d.',name,numel(hit));
i0=hit(1);
next=find(isFcn((i0+1):end),1,'first');
if isempty(next)
    i1=numel(L)+1;
else
    i1=i0+next;
end
body=char(strjoin(L(i0:i1-1),newline));
end

function ok=method_parser_self_test()
% Covers output-assignment method, no-output method, nested control-flow END,
% and spacing variants. This tests the forensic parser only.
x=sprintf(['classdef X\n' ...
    '  methods\n' ...
    '    function y = alpha(obj,a)\n' ...
    '      if a>0\n' ...
    '        y=a;\n' ...
    '      else\n' ...
    '        y=0;\n' ...
    '      end\n' ...
    '    end\n' ...
    '    function beta ( obj )\n' ...
    '      z=1; %% token alpha(obj) in comment must not alter declaration search\n' ...
    '    end\n' ...
    '  end\n' ...
    'end\n']);
a=method_body(x,'alpha');
b=method_body(x,'beta');
ok=contains(a,'function y = alpha(obj,a)') && contains(a,'if a>0') && ...
   ~contains(a,'function beta') && contains(b,'function beta ( obj )') && ...
   ~contains(b,'function y = alpha');
end

function print_windows(src,tokens,radius)
L=splitlines(string(src));
for t=1:numel(tokens)
    tok=string(tokens{t});
    hit=find(contains(L,tok));
    fprintf(' token "%s" hits=%d\n',tok,numel(hit));
    for h=1:min(numel(hit),3)
        a=max(1,hit(h)-radius); b=min(numel(L),hit(h)+radius);
        fprintf('  --- hit %d ---\n',h);
        for i=a:b
            fprintf('   %5d | %s\n',i,L(i));
        end
    end
end
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
md.update(b); d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
