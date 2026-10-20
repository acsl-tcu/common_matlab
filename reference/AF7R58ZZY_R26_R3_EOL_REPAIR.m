function OUT = AF7R58ZZY_R26_R3_EOL_REPAIR()
% AF7R58ZZY-R26-R3 runner EOL repair.
% =========================================================================
% PURPOSE
%   Repair ONLY run_flight_headless.m after the R26-R3 installer.
%
% ROOT CAUSE ADDRESSED
%   The R25 runner is CRLF after the prior transactional write.
%   R26-R3 used fopen(...,'wt') + fwrite(...,'char') on Windows.
%   Text-mode newline translation can transform existing CRLF bytes
%   [13 10] into [13 13 10], yielding a syntactically broken MATLAB file.
%
% STRATEGY
%   Do NOT edit the damaged file in place.
%   1) Find the exact R25 runner backup created by R26-R3 installer.
%   2) Require its SHA-256 to equal the known R25 authority SHA.
%   3) Apply ONLY the four intended R26-R3 ASCII substitutions in raw bytes.
%   4) Require the exact expected repaired SHA.
%   5) Write using binary mode ('wb'), so line endings are not translated.
%
% DOES NOT MODIFY
%   REPLANNING_SYSTEM.m, R25/R21/R19, controller, estimator, certificates,
%   trajectory math, obstacle geometry, or any safety decision.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R3 | RUNNER EOL REPAIR\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] runnerOnly=1 | binaryWrite=1 | productionModified=0 | controllerModified=0 | safetyMathChanged=0\n');

runner = which('run_flight_headless');
assert(~isempty(runner) && isfile(runner), ...
    '[R26-R3 EOL] run_flight_headless.m unresolved.');
root = fileparts(runner);

expectedR25 = '0581116b243151e9b6855aacb676c704ea4b15991620213833f8ba914b9a3451';
expectedR26Fixed = '02abb24ba821931d5a64e60004e971f0c62a7db36bfeeaf5f12b1459dab268f9';
expectedClass = '2aa8f4e0c3ba2d272b8b5f2d7cfd921654c5f312d5f6ded5300988235ea64cd6';

OUT = struct();
OUT.schema = 'AF7R58ZZY_R26_R3_EOL_REPAIR_V1';
OUT.closed = false;
OUT.runner = runner;
OUT.root = root;
OUT.production_modified = false;
OUT.controller_modified = false;

fprintf('\n[A. CURRENT DAMAGED/ACTIVE RUNNER]\n');
OUT.current_sha = sha_file(runner);
[curCRLF,curCRCRLF,curLF] = eol_counts(read_bytes(runner));
fprintf(' runner                                  = %s\n',runner);
fprintf(' SHA before                              = %s\n',OUT.current_sha);
fprintf(' CRLF / CRCRLF / LF bytes               = %d / %d / %d\n',curCRLF,curCRCRLF,curLF);

% R26-R3 class must be the exact class delivered with the package.
classPath = which('REPLANNING_SYSTEM_R26_BALANCED');
assert(~isempty(classPath) && isfile(classPath), ...
    '[R26-R3 EOL] R26 class source missing.');
classSHA = sha_file(classPath);
fprintf(' R26 class                               = %s\n',classPath);
fprintf(' R26 class SHA                           = %s | expected=%d\n', ...
    classSHA,strcmp(classSHA,expectedClass));
assert(strcmp(classSHA,expectedClass), ...
    '[R26-R3 EOL] R26 class is not the exact R3 class. Stop.');

% If already repaired, close without rewriting.
if strcmp(OUT.current_sha,expectedR26Fixed)
    fprintf('\n[B. STATUS]\n');
    fprintf(' runner already has exact repaired SHA. No write needed.\n');
    OUT.backup_source='';
    OUT.repaired_sha=OUT.current_sha;
    OUT.closed=true;
    OUT.status='ALREADY_REPAIRED';
    assignin('base','AF7R58ZZY_R26_R3_EOL_REPAIR_OUT',OUT);
    fprintf(' SELFTEST CLOSED                         = 1\n');
    return;
end

fprintf('\n[B. FIND EXACT R25 TRANSACTION BACKUP]\n');
D = dir(fullfile(root,'AF7R58ZZY_R26_R3_BACKUP_*'));
assert(~isempty(D), ...
    '[R26-R3 EOL] No R26-R3 backup directory found.');

% newest first
[~,ord] = sort([D.datenum],'descend');
D = D(ord);
backup = '';
for k=1:numel(D)
    p = fullfile(D(k).folder,D(k).name,'run_flight_headless.m');
    if isfile(p)
        h = sha_file(p);
        fprintf(' candidate %-50s SHA=%s\n',D(k).name,h);
        if strcmp(h,expectedR25)
            backup=p;
            break;
        end
    end
end
assert(~isempty(backup), ...
    '[R26-R3 EOL] Exact R25 runner backup SHA was not found.');
OUT.backup_source=backup;
fprintf(' selected backup                         = %s\n',backup);
fprintf(' selected SHA                            = %s\n',sha_file(backup));

fprintf('\n[C. RAW-BYTE R26-R3 PATCH]\n');
b = read_bytes(backup);

[b,n1] = replace_ascii_once(b, ...
    'REPLANNING_SYSTEM_R25_LEAN(agent,nominal_ref,replan_opts)', ...
    'REPLANNING_SYSTEM_R26_BALANCED(agent,nominal_ref,replan_opts)');
[b,n2] = replace_ascii_once(b, ...
    'replan_opts.ENABLE_PROFILING = true;', ...
    'replan_opts.ENABLE_PROFILING = false;');
[b,n3] = replace_ascii_once(b, ...
    'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = true;', ...
    'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = false;');
[b,n4] = replace_ascii_once(b, ...
    'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 0.0;', ...
    'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 2.0;');

fprintf(' constructor replacements                = %d\n',n1);
fprintf(' profiling replacements                  = %d\n',n2);
fprintf(' light-audit replacements                = %d\n',n3);
fprintf(' spike-threshold replacements            = %d\n',n4);
assert(all([n1 n2 n3 n4]==1), ...
    '[R26-R3 EOL] Raw patch occurrence count is not exactly one.');

% R25 fast merge must already be present in the exact R25 backup.
mergeNeedle=uint8('af7r77_merge_result_reference_fast_R25(agent.reference.result,r22_sload_result)');
assert(numel(strfind(rowvec(b),rowvec(mergeNeedle)))==1, ...
    '[R26-R3 EOL] Exact R25 fast merge binding missing.');

candidateSHA = sha_bytes(b);
[cCRLF,cCRCRLF,cLF] = eol_counts(b);
fprintf(' candidate SHA                           = %s\n',candidateSHA);
fprintf(' expected repaired SHA                   = %s\n',expectedR26Fixed);
fprintf(' candidate CRLF / CRCRLF / LF            = %d / %d / %d\n',cCRLF,cCRCRLF,cLF);
assert(strcmp(candidateSHA,expectedR26Fixed), ...
    '[R26-R3 EOL] Candidate bytes differ from exact expected R26-R3 runner.');
assert(cCRCRLF==0, ...
    '[R26-R3 EOL] Candidate still contains CRCRLF.');

fprintf('\n[D. BACKUP CURRENT FILE + BINARY WRITE]\n');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
damagedBackup=fullfile(root,['run_flight_headless.R26R3_DAMAGED_' stamp '.m']);
copyfile(runner,damagedBackup);
OUT.damaged_backup=damagedBackup;
fprintf(' damaged runner backup                   = %s\n',damagedBackup);

write_bytes(runner,b);
rehash;
OUT.repaired_sha=sha_file(runner);
fprintf(' SHA after                               = %s\n',OUT.repaired_sha);
assert(strcmp(OUT.repaired_sha,expectedR26Fixed), ...
    '[R26-R3 EOL] Binary write verification failed.');

% Verify exact textual bindings without rewriting.
txt=fileread(runner);
assert(numel(strfind(txt,'REPLANNING_SYSTEM_R26_BALANCED(agent,nominal_ref,replan_opts)'))==1, ...
    '[R26-R3 EOL] R26 constructor binding missing after repair.');
assert(numel(strfind(txt,'replan_opts.ENABLE_PROFILING = false;'))==1, ...
    '[R26-R3 EOL] profiling OFF binding missing.');
assert(numel(strfind(txt,'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = false;'))==1, ...
    '[R26-R3 EOL] light-audit OFF binding missing.');
assert(numel(strfind(txt,'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 2.0;'))==1, ...
    '[R26-R3 EOL] spike threshold binding missing.');
assert(numel(strfind(txt,'af7r77_merge_result_reference_fast_R25(agent.reference.result,r22_sload_result)'))==1, ...
    '[R26-R3 EOL] fast merge binding missing.');

% Lightweight parser/linter. Warnings are printed, but any thrown parser error
% aborts the repair.
fprintf('\n[E. MATLAB SOURCE CHECK]\n');
try
    C = checkcode(runner,'-id');
    OUT.checkcode = C;
    fprintf(' checkcode entries                       = %d\n',numel(C));
    for k=1:min(numel(C),20)
        if isfield(C,'id')
            fprintf('  L%d C%d | %s | %s\n',C(k).line,C(k).column,C(k).id,C(k).message);
        else
            fprintf('  L%d C%d | %s\n',C(k).line,C(k).column,C(k).message);
        end
    end
catch ME
    OUT.checkcode_error=struct('identifier',ME.identifier,'message',ME.message);
    error('[R26-R3 EOL] checkcode/parser failed after repair: %s',ME.message);
end

% Confirm class itself is parseable without constructing/running Flight.
mc=meta.class.fromName('REPLANNING_SYSTEM_R26_BALANCED');
assert(~isempty(mc),'[R26-R3 EOL] R26 class parse failed after runner repair.');
fprintf(' R26 class parse                         = PASS\n');

OUT.closed=true;
OUT.status='R26_R3_RUNNER_EOL_REPAIRED__RUN_FLIGHT_NEXT';
assignin('base','AF7R58ZZY_R26_R3_EOL_REPAIR_OUT',OUT);

fprintf('\n====================================================================================================\n');
fprintf(' R26-R3 RUNNER EOL REPAIR COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf('production replanner modified           = 0\n');
fprintf('controller modified                      = 0\n');
fprintf('runner repaired SHA                      = %s\n',OUT.repaired_sha);
fprintf('SELFTEST CLOSED                          = 1\n');
fprintf('Next:\n');
fprintf('  run_flight_headless\n');
fprintf('  AF7R58ZZY_R26_R3_REPORT\n');
fprintf('====================================================================================================\n');
end

function [b,n]=replace_ascii_once(b,old,new)
b=rowvec(b);
o=rowvec(uint8(old));
r=rowvec(uint8(new));
ix=strfind(b,o);
n=numel(ix);
if n~=1, return; end
i=ix(1);
b=[b(1:i-1) r b(i+numel(o):end)];
b=b(:);
end

function b=read_bytes(p)
f=fopen(p,'rb'); assert(f>=0,'cannot open %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
end

function write_bytes(p,b)
f=fopen(p,'wb'); assert(f>=0,'cannot write %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
n=fwrite(f,b,'uint8');
assert(n==numel(b),'short write: %s',p);
end

function [nCRLF,nCRCRLF,nLF]=eol_counts(b)
b=rowvec(b);
nCRLF=numel(strfind(b,uint8([13 10])));
nCRCRLF=numel(strfind(b,uint8([13 13 10])));
nLF=sum(b==10);
end

function x=rowvec(x)
x=x(:).';
end

function h=sha_file(p)
h=sha_bytes(read_bytes(p));
end

function h=sha_bytes(b)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(uint8(b(:)),'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
