function OUT = AF7R39_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R39A | EXACT-SOURCE PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] sourceBound=1 | classParse=1 | helperPresence=1 | noRuntimeClaim=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R39A] REPLANNING_SYSTEM unresolved.');
expected='880cd6a3f9ec0c6d7a54445091becf776e0d766e8327aff7f2e899ac5ff41682';
h=sha256_file(src);
assert(strcmp(h,expected), ...
    '[AF7R39A] source hash differs from the audited AF7R39 replacement.');

raw=fileread(src);
compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];

checks=struct();
checks.r13c=contains(raw,'AF7R33-R13C');
checks.r14d=contains(raw,'AF7R37-R14D-PRODUCTION');
checks.r38=contains(raw,'AF7R38-PREFLIGHT-POLICY-HOTPATH');
checks.r39=contains(raw,'AF7R39-EXACT-R38-PROPERTY-REPAIR');
checks.native_default=contains(raw,'fn=@(CTX)af7r36_single_core_proposal(obj,CTX);');
checks.canonical_separator=contains(raw,'af7r36_canonical_separator_normal');
checks.reference_only_defer=contains(raw,'EXECUTION_AWARE_DEFERRED_IN_REFERENCE_ONLY_HOTPATH');
checks.observer_prime_logic=contains(raw,'obj.observer_preflight_prime_done=true;');
checks.qpc_prime_logic=contains(raw,'obj.qpc_stamp_prime_done=true;');
checks.postcallback_prime_logic=contains(raw,'obj.postcallback_prime_done=true;');
checks.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
checks.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
checks.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
checks.sampled_acceptance_true=contains(compact,needle);

% Force MATLAB to parse/load the class definition without constructing it.
mc=meta.class.fromName('REPLANNING_SYSTEM');
assert(~isempty(mc),'[AF7R39A] MATLAB could not load the REPLANNING_SYSTEM class definition.');

PL=mc.PropertyList;
pnames=cell(1,numel(PL));
if iscell(PL)
    for k=1:numel(PL), pnames{k}=PL{k}.Name; end
else
    for k=1:numel(PL), pnames{k}=PL(k).Name; end
end

requiredProps={ ...
    'observer_preflight_prime_active', ...
    'observer_preflight_prime_done', ...
    'observer_preflight_prime_ms', ...
    'qpc_stamp_prime_done', ...
    'postcallback_prime_candidate_found', ...
    'postcallback_prime_done', ...
    'postcallback_prime_ms'};

propClosed=true;
for k=1:numel(requiredProps)
    tf=any(strcmp(pnames,requiredProps{k}));
    fprintf(' property %-38s = %d\n',requiredProps{k},tf);
    propClosed=propClosed&&tf;
end

reqM={ ...
    'af7r36_runtime_prime', ...
    'af7r36_single_core_proposal', ...
    'af7r36_canonical_separator_normal', ...
    'af7r34_make_core_packet', ...
    'af7r33_make_batch_model_from_proposal', ...
    'af7r33_nominal_batch_model'};
helperClosed=true;
for k=1:numel(reqM)
    tf=exist(reqM{k},'file')==2;
    fprintf(' helper %-40s = %d\n',reqM{k},tf);
    helperClosed=helperClosed&&tf;
end

reqMex={'af7r36_single_core_mex','af7r25_stamp_mex'};
for k=1:numel(reqMex)
    tf=exist(reqMex{k},'file')==3;
    fprintf(' MEX    %-40s = %d\n',reqMex{k},tf);
    helperClosed=helperClosed&&tf;
end

logicClosed=checks.r13c&&checks.r14d&&checks.r38&&checks.r39&& ...
    checks.native_default&&checks.canonical_separator&&checks.reference_only_defer&& ...
    checks.observer_prime_logic&&checks.qpc_prime_logic&&checks.postcallback_prime_logic&& ...
    checks.hard4ms&&checks.r9&&checks.wholebody&&~checks.sampled_acceptance_true;

closed=propClosed&&helperClosed&&logicClosed;

fprintf('\n source SHA-256                          = %s\n',h);
fprintf(' class definition parsed by MATLAB       = 1\n');
fprintf(' safety/deadline source contract closed  = %d\n',logicClosed);
fprintf(' AF7R39 pre-run closed                   = %d\n',closed);
fprintf(' runtime/4-ms performance claimed        = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R39_PRE_RUN_AUDIT_V1','closed',closed, ...
    'source',src,'sha256',h,'checks',checks, ...
    'properties_closed',propClosed,'helpers_closed',helperClosed);
assignin('base','AF7R39_PRE_RUN_AUDIT_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb');
assert(fid>=0,'[AF7R39A] cannot open file for SHA-256: %s',fp);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
clear c;
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
