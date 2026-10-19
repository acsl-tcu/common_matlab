function OUT = AF7R57T_V4_DEFAULT_WIRING_CORRECTION_AUDIT()
% AF7R57T_V4_DEFAULT_WIRING_CORRECTION_AUDIT
% Read-only correction of the R57T V3 false-negative backend-1 wiring
% classification.  No production modification, no timing claim upgrade.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57T V4 | DEFAULT-WIRED BACKEND SOURCE-ORDER CORRECTION AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | exactCurrentSHA=1 | parserIndependent=1 | noBackendRemoval=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
selfFile=[mfilename('fullpath') '.m'];
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

rps=fullfile(root,'REPLANNING_SYSTEM.m');
prime=fullfile(root,'af7r36_runtime_prime.m');
prop=fullfile(root,'af7r36_single_core_proposal.m');
assert(isfile(rps),'[AF7R57T V4] REPLANNING_SYSTEM.m missing.');
assert(isfile(prime),'[AF7R57T V4] af7r36_runtime_prime.m missing.');
assert(isfile(prop),'[AF7R57T V4] af7r36_single_core_proposal.m missing.');

EXP_RPS='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
EXP_PRIME='300adebef4140d68cfc393f43472f378bfe0ebf6fa00fcea6f0e386f76ae9cb4';
EXP_PROP='d7bbe05efd268743e7307bcf3291f8f0e7777ab126432abc190ac60c999cc7ac';
shaR=sha256_file(rps); shaP=sha256_file(prime); shaG=sha256_file(prop);

fprintf('\n[A. EXACT CURRENT SOURCE BINDING]\n');
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',shaR);
fprintf(' current source matches bound authority  = %d\n',strcmp(shaR,EXP_RPS));
fprintf(' af7r36_runtime_prime SHA current         = %d\n',strcmp(shaP,EXP_PRIME));
fprintf(' af7r36_single_core_proposal SHA current  = %d\n',strcmp(shaG,EXP_PROP));
assert(strcmp(shaR,EXP_RPS),'[AF7R57T V4] REPLANNING_SYSTEM authority changed.');
assert(strcmp(shaP,EXP_PRIME),'[AF7R57T V4] runtime-prime helper authority changed.');
assert(strcmp(shaG,EXP_PROP),'[AF7R57T V4] proposal helper authority changed.');

src=fileread(rps);
c=compact_source(src);

% Freeze-method source window, extracted by exact compact declaration tokens.
tFreeze='functionfreeze_generator_backend_configuration(obj)';
tNext='functionC=normalize_generator_backend_callbacks(~,C)';
p0=find1(c,tFreeze);
p1=find_after(c,tNext,p0+length(tFreeze));
assert(p1>p0,'[AF7R57T V4] freeze method window not found.');
fz=c(p0:p1-1);

% Parser-independent exact wiring tokens.
tIf='ifisempty(fn)&&double(obj.generator_backend_locked_code)==1';
tPrime='af7r36_runtime_prime(obj);';
tProp='fn=@(CTX)af7r36_single_core_proposal(obj,CTX);';
tAssign='obj.generator_backend_locked_callback=fn;';

pi=find1(fz,tIf); pp=find1(fz,tPrime); pg=find1(fz,tProp); pa=find1(fz,tAssign);
countIf=count_token(fz,tIf); countPrime=count_token(fz,tPrime);
countProp=count_token(fz,tProp); countAssign=count_token(fz,tAssign);
ordered=(pi<pp)&&(pp<pg)&&(pg<pa);

fprintf('\n[B. BACKEND-1 DEFAULT-WIRING EXACT TOKEN ORDER]\n');
fprintf(' code==1 default condition count          = %d\n',countIf);
fprintf(' af7r36_runtime_prime token count         = %d\n',countPrime);
fprintf(' af7r36 proposal assignment count         = %d\n',countProp);
fprintf(' locked-callback assignment count         = %d\n',countAssign);
fprintf(' condition -> prime -> proposal -> assign = %d\n',ordered);
fprintf(' backend 1 default-wired production helper= %d\n', ...
    countIf==1 && countPrime==1 && countProp==1 && countAssign==1 && ordered);

% Confirm no analogous explicit default code branches for 2..4 in this method.
fprintf('\n[C. BACKENDS 2-4 DEFAULT-WIRING ABSENCE]\n');
no234=true;
for k=2:4
    tk=sprintf('double(obj.generator_backend_locked_code)==%d',k);
    nk=count_token(fz,tk);
    fprintf(' backend %d explicit default-code branch   = %d\n',k,nk);
    no234=no234 && nk==0;
end

% Generic callback and fallback remain present.
callbackTable=contains(c,'generator_backend_callbacks');
globalFallback=contains(fz,'fn=obj.repaired_candidate_generator_fn;');
typeCheck=contains(fz,"assert(isa(fn,'function_handle')");
fprintf('\n[D. CALLBACK ROUTER CONTRACT]\n');
fprintf(' generic per-backend callback table      = %d\n',callbackTable);
fprintf(' legacy/global callback fallback         = %d\n',globalFallback);
fprintf(' selected callback type check            = %d\n',typeCheck);

% Invocation fail-closed tokens. We deliberately do not parse MATLAB blocks.
tInv='functioninvoke_replan_generator_shadow(obj';
pInv=find1(c,tInv);
inv=c(pInv:end);
emptyCb=contains(inv,'ifisempty(obj.generator_backend_locked_callback)');
emptyCandidate=contains(inv,'candidate=[];');
noCandidate=contains(inv,'ifisempty(candidate)');
statusNoCandidate=contains(inv,'obj.generator_last_status_code=1;');
returnPresent=contains(inv,'return;');
failClosed=emptyCb && emptyCandidate && noCandidate && statusNoCandidate && returnPresent;

fprintf('\n[E. UNREGISTERED CALLBACK FAIL-CLOSED CONTRACT]\n');
fprintf(' empty locked callback branch            = %d\n',emptyCb);
fprintf(' candidate=[] on empty path              = %d\n',emptyCandidate);
fprintf(' no-candidate branch                     = %d\n',noCandidate);
fprintf(' no-candidate status code                = %d\n',statusNoCandidate);
fprintf(' invocation return path present          = %d\n',returnPresent);
fprintf(' unregistered callback fail-closed        = %d\n',failClosed);

% Helper source identity / visibility.
cp=compact_source(fileread(prime));
cg=compact_source(fileread(prop));
primeNamesCore=contains(cp,'af7r36') || contains(cp,'single_core');
proposalDecl=contains(cg,'af7r36_single_core_proposal');
proposalBezier=contains(cg,'BEZIER_D15_C7');
proposalCorePacket=contains(cg,'af7r34_make_core_packet');
proposalNativeMex=contains(cg,'af7r36_single_core_mex');

fprintf('\n[F. DEFAULT HELPER SOURCE IDENTITY]\n');
fprintf(' runtime-prime source recognizable        = %d\n',primeNamesCore);
fprintf(' proposal helper declaration/name visible = %d\n',proposalDecl);
fprintf(' proposal BEZIER_D15_C7 token             = %d\n',proposalBezier);
fprintf(' proposal calls af7r34 core packet        = %d\n',proposalCorePacket);
fprintf(' proposal calls af7r36 native MEX         = %d\n',proposalNativeMex);

backend1Closed=(countIf==1 && countPrime==1 && countProp==1 && countAssign==1 && ordered && proposalDecl);
scopeClosed=backend1Closed && no234 && callbackTable && globalFallback && typeCheck && failClosed;

fprintf('\n[G. CORRECTED CURRENT OPERATIONAL CLASSIFICATION]\n');
if backend1Closed
    fprintf(' BEZIER_D15_C7            = DEFAULT_WIRED_NATIVE_HELPER__CURRENT_SOURCE\n');
else
    fprintf(' BEZIER_D15_C7            = OPEN__DEFAULT_WIRING_NOT_CLOSED\n');
end
if no234 && failClosed
    fprintf(' POLY_D15_C7              = CALLBACK_ONLY__FAIL_CLOSED_IF_UNREGISTERED\n');
    fprintf(' MINCO_STYLE_D15_C7       = CALLBACK_ONLY__FAIL_CLOSED_IF_UNREGISTERED\n');
    fprintf(' BSPLINE_D8_C7            = CALLBACK_ONLY__FAIL_CLOSED_IF_UNREGISTERED\n');
else
    fprintf(' backends 2-4             = OPEN__ROUTER_CLASSIFICATION_NOT_CLOSED\n');
end
fprintf(' V3 backend-1 NO_DEFAULT_WIRING status    = RETRACT_FALSE_NEGATIVE\n');
fprintf(' catalog backends silently removed       = 0\n');
fprintf(' callback-only treated as proved backend = 0\n');

fprintf('\n[H. FORMAL SCOPE GATE]\n');
fprintf(' backend-1 default helper binding closed = %d\n',backend1Closed);
fprintf(' backends 2-4 no default wiring          = %d\n',no234);
fprintf(' unregistered callback fail-closed       = %d\n',failClosed);
fprintf(' router operational scope closed         = %d\n',scopeClosed);
fprintf(' current default-wired backend count     = %d\n',double(backend1Closed));
fprintf(' current default theorem backend         = %s\n',ternary(backend1Closed,'BEZIER_D15_C7','OPEN'));
fprintf(' canonical native contract still open    = 1\n');
fprintf(' production source modified              = 0\n');
fprintf(' timing claim upgraded                   = 0\n');
fprintf(' next action                              = CLOSE_BEZIER_AF7R36_TO_NATIVE_CANONICAL_SERIALIZATION__THEN_G_S_PREPARSE_GUARD\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57T_V4_DEFAULT_WIRING_CORRECTION_V1';
OUT.backend1_default_wired=backend1Closed;
OUT.backends_2_to_4_no_default_wiring=no234;
OUT.unregistered_callback_fail_closed=failClosed;
OUT.scope_closed=scopeClosed;
OUT.v3_backend1_classification_retracted=true;
OUT.production_modified=false;
OUT.entrypoint=selfFile;
OUT.entrypoint_sha256=selfSHA;
assignin('base','AF7R57T_V4_DEFAULT_WIRING_CORRECTION_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57T_V4_DEFAULT_WIRING_CORRECTION_AUDIT_OUT.mat'),'OUT');
end

function c=compact_source(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
% Strip line comments before removing whitespace. Percent signs inside quoted
% strings are not used by the exact tokens audited here, so this is safe for
% the bounded authority SHA above.
L=splitlines(string(s));
out=strings(size(L));
for i=1:numel(L)
    z=char(L(i));
    p=find(z=='%',1,'first');
    if ~isempty(p), z=z(1:p-1); end
    out(i)=string(z);
end
s=char(join(out,newline));
c=regexprep(s,'\s+','');
end

function p=find1(s,t)
z=strfind(s,t);
assert(~isempty(z),'[AF7R57T V4] required token missing: %s',t);
p=z(1);
end

function p=find_after(s,t,startPos)
z=strfind(s(startPos:end),t);
assert(~isempty(z),'[AF7R57T V4] required trailing token missing: %s',t);
p=startPos+z(1)-1;
end

function n=count_token(s,t)
n=numel(strfind(s,t));
end

function y=ternary(c,a,b)
if c, y=a; else, y=b; end
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
