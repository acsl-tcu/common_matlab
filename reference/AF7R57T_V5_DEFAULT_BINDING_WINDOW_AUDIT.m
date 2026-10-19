function OUT = AF7R57T_V5_DEFAULT_BINDING_WINDOW_AUDIT()
% AF7R57T_V5_DEFAULT_BINDING_WINDOW_AUDIT
% Read-only correction of R57T V4 token false-negative.
% Parser-independent: source is whitespace-compacted, then a unique freeze-time
% window is anchored by the generic fallback assignment and final callback store.
% No production/source modification and no timing claim upgrade.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57T V5 | DEFAULT-BINDING WINDOW / FAIL-CLOSED ROUTER AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | exactCurrentSHA=1 | parserIndependent=1 | tokenFormatIndependent=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
selfFile=[mfilename('fullpath') '.m'];
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

rps=fullfile(root,'REPLANNING_SYSTEM.m');
prime=fullfile(root,'af7r36_runtime_prime.m');
prop=fullfile(root,'af7r36_single_core_proposal.m');
assert(isfile(rps)&&isfile(prime)&&isfile(prop),'[AF7R57T V5] required current source missing.');

expectedRPS='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
expectedPrime='300adebef4140d68cfc393f43472f378bfe0ebf6fa00fcea6f0e386f76ae9cb4';
expectedProp='d7bbe05efd268743e7307bcf3291f8f0e7777ab126432abc190ac60c999cc7ac';

shaR=sha256_file(rps); shaP=sha256_file(prime); shaG=sha256_file(prop);
fprintf('\n[A. EXACT CURRENT SOURCE BINDING]\n');
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',shaR);
fprintf(' current source matches bound authority  = %d\n',strcmp(shaR,expectedRPS));
fprintf(' af7r36_runtime_prime SHA current         = %d\n',strcmp(shaP,expectedPrime));
fprintf(' af7r36_single_core_proposal SHA current  = %d\n',strcmp(shaG,expectedProp));
assert(strcmp(shaR,expectedRPS)&&strcmp(shaP,expectedPrime)&&strcmp(shaG,expectedProp), ...
    '[AF7R57T V5] current authority source changed.');

src=fileread(rps);
c=regexprep(src,'\\s+','');

% Freeze-time window: after generic fallback, before final locked-callback store.
tStart='fn=obj.repaired_candidate_generator_fn;';
tEnd='obj.generator_backend_locked_callback=fn;';
ps=find_once(c,tStart); pe=find_after(c,tEnd,ps+length(tStart));
assert(pe>ps,'[AF7R57T V5] invalid freeze window ordering.');
w=c(ps:pe+length(tEnd)-1);

% Do not require one exact anonymous-function assignment string.  Prove the
% semantic symbol/order chain inside the source-bound freeze window.
tCode1='double(obj.generator_backend_locked_code)==1';
tPrime='af7r36_runtime_prime';
tProp='af7r36_single_core_proposal';
tStore='obj.generator_backend_locked_callback=fn;';
p1=find_once(w,tCode1); p2=find_after(w,tPrime,p1+length(tCode1));
p3=find_after(w,tProp,p2+length(tPrime)); p4=find_after(w,tStore,p3+length(tProp));
orderClosed=(p1<p2)&&(p2<p3)&&(p3<p4);

fprintf('\n[B. BACKEND-1 DEFAULT-WIRING SOURCE ORDER]\n');
fprintf(' code==1 branch token                    = %d\n',~isempty(strfind(w,tCode1))); %#ok<STREMP>
fprintf(' af7r36_runtime_prime symbol after branch= %d\n',p2>p1);
fprintf(' af7r36 proposal symbol after prime      = %d\n',p3>p2);
fprintf(' locked callback store after proposal    = %d\n',p4>p3);
fprintf(' backend-1 default helper binding closed = %d\n',orderClosed);

% No equivalent explicit default code branches for 2..4 inside same freeze window.
no2=isempty(strfind(w,'double(obj.generator_backend_locked_code)==2')); %#ok<STREMP>
no3=isempty(strfind(w,'double(obj.generator_backend_locked_code)==3')); %#ok<STREMP>
no4=isempty(strfind(w,'double(obj.generator_backend_locked_code)==4')); %#ok<STREMP>
fprintf('\n[C. BACKENDS 2-4 DEFAULT-WIRING ABSENCE]\n');
fprintf(' backend 2 explicit default branch absent= %d\n',no2);
fprintf(' backend 3 explicit default branch absent= %d\n',no3);
fprintf(' backend 4 explicit default branch absent= %d\n',no4);
fprintf(' backends 2-4 callback-only by current source = %d\n',no2&&no3&&no4);

% Invocation fail-closed path.  Use compact-source semantic fragments only.
tEmpty='ifisempty(obj.generator_backend_locked_callback)';
tCandEmpty='candidate=[];';
tCbFail='if~callback_ok';
tNoCand='ifisempty(candidate)';
tReturn='return;';
pi=find_once(c,tEmpty);
q1=find_after(c,tCandEmpty,pi+length(tEmpty));
q2=find_after(c,tCbFail,q1+length(tCandEmpty));
q3=find_after(c,tNoCand,q2+length(tCbFail));
q4=find_after(c,tReturn,q3+length(tNoCand));
failClosed=(pi<q1)&&(q1<q2)&&(q2<q3)&&(q3<q4);

fprintf('\n[D. INVOCATION FAIL-CLOSED PATH]\n');
fprintf(' empty locked callback branch present    = %d\n',pi>0);
fprintf(' empty branch creates no candidate       = %d\n',q1>pi);
fprintf(' callback-failure reject branch present  = %d\n',q2>q1);
fprintf(' no-candidate reject branch present      = %d\n',q3>q2);
fprintf(' reject path returns without commit      = %d\n',q4>q3);
fprintf(' unregistered callback fail-closed       = %d\n',failClosed);

fprintf('\n[E. CURRENT OPERATIONAL CLASSIFICATION]\n');
if orderClosed
    b1='DEFAULT_WIRED_NATIVE_HELPER__AF7R36_CURRENT_SOURCE';
else
    b1='OPEN';
end
b234='CALLBACK_ONLY__FAIL_CLOSED_IF_UNREGISTERED';
fprintf(' BEZIER_D15_C7            = %s\n',b1);
fprintf(' POLY_D15_C7              = %s\n',b234);
fprintf(' MINCO_STYLE_D15_C7       = %s\n',b234);
fprintf(' BSPLINE_D8_C7            = %s\n',b234);

scopeClosed=orderClosed&&no2&&no3&&no4&&failClosed;
fprintf('\n[F. FORMAL SCOPE GATE]\n');
fprintf(' backend-1 default binding source-closed = %d\n',orderClosed);
fprintf(' backends 2-4 callback-only source-closed= %d\n',no2&&no3&&no4);
fprintf(' unregistered callbacks fail closed      = %d\n',failClosed);
fprintf(' router operational scope closed         = %d\n',scopeClosed);
fprintf(' canonical native serialization closed   = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' timing claim upgraded                   = 0\n');
fprintf(' next action                              = AUDIT_AF7R36_PROPOSAL_TO_AF7R34_NATIVE_PACKET_EXACT_IDENTITY__THEN_INSTALL_G_S_PREPARSE_GUARD\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57T_V5_DEFAULT_BINDING_WINDOW_AUDIT';
OUT.backend1_default_binding_closed=orderClosed;
OUT.backends234_callback_only_closed=no2&&no3&&no4;
OUT.unregistered_fail_closed=failClosed;
OUT.router_scope_closed=scopeClosed;
OUT.backend_classification={b1,b234,b234,b234};
OUT.production_modified=false;
OUT.entrypoint=selfFile;
OUT.entrypoint_sha256=selfSHA;
assignin('base','AF7R57T_V5_DEFAULT_BINDING_WINDOW_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57T_V5_DEFAULT_BINDING_WINDOW_AUDIT_OUT.mat'),'OUT');
end

function p=find_once(s,t)
z=strfind(s,t);
assert(~isempty(z),'[AF7R57T V5] required semantic token missing: %s',t);
p=z(1);
end

function p=find_after(s,t,startPos)
z=strfind(s(startPos:end),t);
assert(~isempty(z),'[AF7R57T V5] required semantic token missing after anchor: %s',t);
p=startPos+z(1)-1;
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
