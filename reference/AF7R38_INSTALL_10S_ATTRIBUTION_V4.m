function OUT = AF7R38_INSTALL_10S_ATTRIBUTION_V4()
% Windows/CRLF-safe, semantic-token attribution installer.
%
% IMPORTANT:
% - Run this on the CURRENT corrected R38 V2 source.
% - Do NOT restore R14D/R38 again.
% - This installer performs a complete resolve pass BEFORE changing/writing
%   anything. If a semantic location is not uniquely resolved, source is not
%   written.
% - Added fields/timers are diagnostic only and are never used by safety or
%   deadline acceptance.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R38 V4 | WINDOWS-CRLF-SAFE 10 s ATTRIBUTION INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] resolveBeforeWrite=1 | diagnosticOnly=1 | safetyLogicUnchanged=1 | hard4msUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
if isempty(src)
    cand=fullfile(pwd,'REPLANNING_SYSTEM.m');
    assert(isfile(cand), ...
        '[AF7R38V4] REPLANNING_SYSTEM.m unresolved. cd to the reference folder first.');
    src=cand;
end
assert(isfile(src),'[AF7R38V4] source file missing.');

raw0=fileread(src);

assert(contains(raw0,'AF7R37-R14D-PRODUCTION'), ...
    '[AF7R38V4] R14D production marker missing.');
assert(contains(raw0,'AF7R38-PREFLIGHT-POLICY-HOTPATH'), ...
    '[AF7R38V4] corrected R38 V2 is not present.');
assert(~contains(raw0,'AF7R38V4-10S-ATTRIBUTION'), ...
    '[AF7R38V4] V4 attribution already installed.');

% Detect source newline only for generated text. No line-end regex authority.
if contains(raw0,sprintf('\r\n'))
    NL=sprintf('\r\n');
else
    NL=sprintf('\n');
end

% =========================================================================
% RESOLVE PASS ON THE UNMODIFIED SOURCE
% =========================================================================
R=struct();

% Property semantic token: no ^/$, no CRLF sensitivity.
R.prop=resolve_regex(raw0, ...
    'postcallback_prime_candidate_found[ \t]*=[ \t]*false;', ...
    'postcallback_prime_candidate_found property');

% REFERENCE_ONLY policy block.
R.refCase=resolve_literal(raw0, ...
    'case ''REFERENCE_ONLY''', ...
    'REFERENCE_ONLY policy case');
R.execCase=resolve_literal_after(raw0, ...
    'case ''EXECUTION_AWARE''',R.refCase.e, ...
    'EXECUTION_AWARE policy case after REFERENCE_ONLY');
assert(R.execCase.s>R.refCase.s, ...
    '[AF7R38V4] policy case order is invalid.');
policyBlock=raw0(R.refCase.s:R.execCase.s-1);
q=strfind(policyBlock, ...
    'Rref=obj.evaluate_reference_only_proposal_certificate(candidate);');
assert(numel(q)==1, ...
    '[AF7R38V4] REFERENCE_ONLY certificate call count inside policy block = %d, expected 1.',numel(q));
R.refCall.s=R.refCase.s+q(1)-1;
R.refCall.e=R.refCall.s+numel('Rref=obj.evaluate_reference_only_proposal_certificate(candidate);')-1;

% Commit method boundaries.
R.commitStart=resolve_literal(raw0, ...
    'function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)', ...
    'reference-only commit method');
nextFun=regexp(raw0(R.commitStart.e+1:end), ...
    '(?m)^[ \t]*function[ \t]+','start','once');
assert(~isempty(nextFun),'[AF7R38V4] could not resolve end of commit method.');
R.commitEnd=R.commitStart.e+nextFun-1;
commit0=raw0(R.commitStart.s:R.commitEnd);

% Method-local semantic anchors.
R.c_okfalse = local_literal(commit0,'ok=false;','commit ok=false');
R.c_reason0 = local_literal_after(commit0,'reason='''';',R.c_okfalse.e,'commit initial reason');
R.c_evalfn  = local_literal(commit0, ...
    'evalFn=@(tg)obj.evaluate_bezier_proposal_reference(tg,P);','commit evalFn');
R.c_old     = local_literal(commit0,'old=struct( ...','commit rollback snapshot');
R.c_valspd  = local_literal(commit0,'obj.validate_active_speed_certificate(SC);','commit speed validation');
R.c_sweep   = local_literal_after(commit0,'obj.trigger_sweep_inflation=',R.c_valspd.e,'commit sweep rebuild start');
R.c_depsig  = local_literal(commit0,'obj.runtime_dependency_signature=','commit dependency signature');
R.c_depfast = local_literal_after(commit0,'obj.capture_runtime_dependency_signature_fast();',R.c_depsig.e,'commit fast dependency capture');
R.c_oktrue  = local_literal(commit0,'ok=true;','commit success ok');
R.c_closed  = local_literal_after(commit0,'reason=''REFERENCE_ONLY_COMMIT_CLOSED'';',R.c_oktrue.e,'commit success reason');
R.c_oldtime = local_literal(commit0,'obj.last_replan_commit_time=old.time;','commit rollback last time');
R.c_memsg   = local_literal_after(commit0,'reason=ME.message;',R.c_oldtime.e,'commit rollback reason');

fprintf('\n[RESOLVE PASS -- CURRENT SOURCE, BEFORE WRITE]\n');
fprintf(' property semantic token                  = 1\n');
fprintf(' REFERENCE_ONLY policy block              = 1\n');
fprintf(' reference certificate call in block      = 1\n');
fprintf(' commit method                             = 1\n');
fprintf(' commit prepare anchors                    = 1\n');
fprintf(' commit rebuild anchors                    = 1\n');
fprintf(' commit success/rollback anchors           = 1\n');

% =========================================================================
% APPLY PATCH IN MEMORY ONLY
% =========================================================================
raw=raw0;

% Marker immediately after classdef token.
C=resolve_regex(raw,'classdef[ \t]+REPLANNING_SYSTEM[ \t]*<[ \t]*handle','classdef');
marker=[NL NL '%% AF7R38V4-10S-ATTRIBUTION: diagnostic-only post-callback timing.'];
raw=insert_after(raw,C.e,marker);

% Properties. Re-resolve after marker insertion.
P=resolve_regex(raw, ...
    'postcallback_prime_candidate_found[ \t]*=[ \t]*false;', ...
    'postcallback property after marker');
propText=[ ...
    NL '        %% AF7R38V4 diagnostic-only timing fields' ...
    NL '        r38_ref_eval_ms                  = NaN;' ...
    NL '        r38_commit_prepare_ms            = NaN;' ...
    NL '        r38_commit_rebuild_ms            = NaN;' ...
    NL '        r38_commit_total_ms              = NaN;' ...
    NL '        r38_commit_attempt_count         = 0;' ...
    NL '        r38_commit_last_ok               = false;' ...
    NL '        r38_commit_last_reason           = '''';'];
raw=insert_after(raw,P.e,propText);

% REFERENCE_ONLY call: add timers around exactly that call inside that case.
caseR=resolve_literal(raw,'case ''REFERENCE_ONLY''','REFERENCE_ONLY case after property insert');
caseX=resolve_literal_after(raw,'case ''EXECUTION_AWARE''',caseR.e,'EXECUTION_AWARE case after property insert');
block=raw(caseR.s:caseX.s-1);
q=strfind(block,'Rref=obj.evaluate_reference_only_proposal_certificate(candidate);');
assert(numel(q)==1);
absCall=caseR.s+q(1)-1;
lineStart=find_line_start(raw,absCall);
indent=raw(lineStart:absCall-1);
call='Rref=obj.evaluate_reference_only_proposal_certificate(candidate);';
refReplacement=[ ...
    indent 'tr38_ref=tic;' NL ...
    indent call NL ...
    indent 'obj.r38_ref_eval_ms=1e3*toc(tr38_ref);'];
raw=[raw(1:lineStart-1) refReplacement raw(absCall+numel(call):end)];

% Commit method after earlier insertions.
M=resolve_literal(raw, ...
    'function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)', ...
    'commit method after prior insertions');
nextFun=regexp(raw(M.e+1:end),'(?m)^[ \t]*function[ \t]+','start','once');
assert(~isempty(nextFun));
mEnd=M.e+nextFun-1;
method=raw(M.s:mEnd);

% A) Entrance: insert after initial reason='';.
A=local_literal(method,'reason='''';','commit initial reason after insertions');
absA=M.s+A.e-1;
indentA=line_indent_at(raw,absA);
entrance=[ ...
    NL indentA 'obj.r38_commit_attempt_count=obj.r38_commit_attempt_count+1;' ...
    NL indentA 'obj.r38_commit_last_ok=false;' ...
    NL indentA 'obj.r38_commit_last_reason='''';' ...
    NL indentA 'obj.r38_commit_prepare_ms=NaN;' ...
    NL indentA 'obj.r38_commit_rebuild_ms=NaN;' ...
    NL indentA 'obj.r38_commit_total_ms=NaN;' ...
    NL indentA 'tr38_total=tic;'];
raw=insert_after(raw,absA,entrance);

% Refresh method location after each structural insertion.
[M,mEnd,method]=get_commit_method(raw);

% B) Prepare timer starts before evalFn.
A=local_literal(method,'evalFn=@(tg)obj.evaluate_bezier_proposal_reference(tg,P);','evalFn');
absS=M.s+A.s-1;
ls=find_line_start(raw,absS);
ind=line_indent_at(raw,absS);
raw=[raw(1:ls-1) ind 'tr38_prepare=tic;' NL raw(ls:end)];

[M,mEnd,method]=get_commit_method(raw);

% C) Prepare timer closes immediately before rollback snapshot.
A=local_literal(method,'old=struct( ...','rollback snapshot');
absS=M.s+A.s-1;
ls=find_line_start(raw,absS);
ind=line_indent_at(raw,absS);
raw=[raw(1:ls-1) ind 'obj.r38_commit_prepare_ms=1e3*toc(tr38_prepare);' NL raw(ls:end)];

[M,mEnd,method]=get_commit_method(raw);

% D) Rebuild starts after speed certificate validation, before sweep rebuild.
A=local_literal(method,'obj.validate_active_speed_certificate(SC);','speed validation');
absE=M.s+A.e-1;
ind=line_indent_at(raw,absE);
raw=insert_after(raw,absE,[NL ind 'tr38_rebuild=tic;']);

[M,mEnd,method]=get_commit_method(raw);

% E) Rebuild ends after fast dependency signature capture.
A=local_literal(method,'obj.capture_runtime_dependency_signature_fast();','dependency capture');
absE=M.s+A.e-1;
ind=line_indent_at(raw,absE);
raw=insert_after(raw,absE,[NL ind 'obj.r38_commit_rebuild_ms=1e3*toc(tr38_rebuild);']);

[M,mEnd,method]=get_commit_method(raw);

% F) Success status after success reason.
A=local_literal(method,'reason=''REFERENCE_ONLY_COMMIT_CLOSED'';','success reason');
absE=M.s+A.e-1;
ind=line_indent_at(raw,absE);
successText=[ ...
    NL ind 'obj.r38_commit_total_ms=1e3*toc(tr38_total);' ...
    NL ind 'obj.r38_commit_last_ok=true;' ...
    NL ind 'obj.r38_commit_last_reason=reason;'];
raw=insert_after(raw,absE,successText);

[M,mEnd,method]=get_commit_method(raw);

% G) Rollback status after ME.message reason.
A=local_literal(method,'obj.last_replan_commit_time=old.time;','rollback last time');
B=local_literal_after(method,'reason=ME.message;',A.e,'rollback reason');
absE=M.s+B.e-1;
ind=line_indent_at(raw,absE);
rollbackText=[ ...
    NL ind 'obj.r38_commit_total_ms=1e3*toc(tr38_total);' ...
    NL ind 'obj.r38_commit_last_ok=false;' ...
    NL ind 'obj.r38_commit_last_reason=reason;'];
raw=insert_after(raw,absE,rollbackText);

% =========================================================================
% POST-PATCH IN-MEMORY AUDIT BEFORE WRITE
% =========================================================================
assert(contains(raw,'AF7R38V4-10S-ATTRIBUTION'));
assert(contains(raw,'obj.r38_ref_eval_ms=1e3*toc(tr38_ref);'));
assert(contains(raw,'obj.r38_commit_prepare_ms=1e3*toc(tr38_prepare);'));
assert(contains(raw,'obj.r38_commit_rebuild_ms=1e3*toc(tr38_rebuild);'));
assert(count(raw,'obj.r38_commit_total_ms=1e3*toc(tr38_total);')>=2);

compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];
assert(~contains(compact,needle), ...
    '[AF7R38V4] sampled_acceptance=true detected.');
assert(contains(raw,'hard_modify_deadline_s<=4.0e-3'), ...
    '[AF7R38V4] hard 4 ms source contract disappeared.');
assert(contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH'), ...
    '[AF7R38V4] whole-body theorem marker disappeared.');
assert(contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED'), ...
    '[AF7R38V4] R9 theorem marker disappeared.');
assert(contains(raw,'EXECUTION_AWARE_DEFERRED_IN_REFERENCE_ONLY_HOTPATH'), ...
    '[AF7R38V4] R38 REFERENCE_ONLY policy deferral disappeared.');
assert(contains(raw,'af7r36_single_core_proposal(obj,CTX)'), ...
    '[AF7R38V4] R14D native production callback disappeared.');

% Only now create backup and write.
root=fileparts(src);
stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(root,['REPLANNING_SYSTEM_pre_R38V4attr_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f');
assert(ok,'[AF7R38V4] backup failed: %s',msg);

fid=fopen(src,'w');
assert(fid>=0,'[AF7R38V4] cannot open source for writing.');
cc=onCleanup(@()fclose(fid));
fwrite(fid,raw,'char');
clear cc;
rehash;

OUT=struct('schema','AF7R38_INSTALL_10S_ATTRIBUTION_V4', ...
    'installed',true,'source',src,'backup',bak, ...
    'resolve_before_write',true,'diagnostic_only',true, ...
    'hard_deadline_modified',false,'safety_logic_modified',false);
assignin('base','AF7R38_INSTALL_10S_ATTRIBUTION_V4_OUT',OUT);

fprintf('\n[WRITE]\n');
fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' resolve-before-write                    = 1\n');
fprintf(' CRLF-sensitive line anchor used         = 0\n');
fprintf(' reference certificate timing            = 1\n');
fprintf(' commit prepare/rebuild/total timing      = 1\n');
fprintf(' safety logic modified                   = 0\n');
fprintf(' hard deadline modified                  = 0\n');
fprintf('====================================================================================================\n');
end


function R=resolve_regex(raw,pat,label)
[s,e]=regexp(raw,pat,'start','end');
assert(numel(s)==1, ...
    '[AF7R38V4] %s count = %d, expected 1.',label,numel(s));
R=struct('s',s(1),'e',e(1));
end

function R=resolve_literal(raw,token,label)
p=strfind(raw,token);
assert(numel(p)==1, ...
    '[AF7R38V4] %s count = %d, expected 1.',label,numel(p));
R=struct('s',p(1),'e',p(1)+numel(token)-1);
end

function R=resolve_literal_after(raw,token,afterPos,label)
p=strfind(raw(afterPos+1:end),token);
assert(numel(p)==1, ...
    '[AF7R38V4] %s count after anchor = %d, expected 1.',label,numel(p));
s=afterPos+p(1);
R=struct('s',s,'e',s+numel(token)-1);
end

function R=local_literal(txt,token,label)
p=strfind(txt,token);
assert(numel(p)==1, ...
    '[AF7R38V4] %s count in commit method = %d, expected 1.',label,numel(p));
R=struct('s',p(1),'e',p(1)+numel(token)-1);
end

function R=local_literal_after(txt,token,afterPos,label)
p=strfind(txt(afterPos+1:end),token);
assert(numel(p)==1, ...
    '[AF7R38V4] %s count after method anchor = %d, expected 1.',label,numel(p));
s=afterPos+p(1);
R=struct('s',s,'e',s+numel(token)-1);
end

function out=insert_after(in,pos,addition)
out=[in(1:pos) addition in(pos+1:end)];
end

function s=find_line_start(raw,pos)
q=find(raw(1:pos)==newline,1,'last');
if isempty(q)
    s=1;
else
    s=q+1;
end
if s<=numel(raw) && raw(s)==char(13)
    s=s+1;
end
end

function ind=line_indent_at(raw,pos)
s=find_line_start(raw,pos);
k=s;
while k<=numel(raw) && (raw(k)==' ' || raw(k)==char(9))
    k=k+1;
end
ind=raw(s:k-1);
end

function [M,mEnd,method]=get_commit_method(raw)
M=resolve_literal(raw, ...
    'function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)', ...
    'commit method');
nextFun=regexp(raw(M.e+1:end),'(?m)^[ \t]*function[ \t]+','start','once');
assert(~isempty(nextFun),'[AF7R38V4] commit method end unresolved.');
mEnd=M.e+nextFun-1;
method=raw(M.s:mEnd);
end
