function T = AF7R58R17_AUTO_RESULT()
% AF7R58R17: Inspect independent on-disk R16 bridge / Stage1/2/3 triage.
% No caller-workspace variables, root appdata, replanner handle, or MEX call.
% This report never changes trajectory acceptance or certificates.
folder=fileparts(mfilename('fullpath'));
path=fullfile(folder,'AF7R58R17_DIAGNOSTIC_CAPTURE.mat');
assert(isfile(path), ...
    ['[R17] No diagnostic capture file was produced. Confirm that ', ...
     'REPLANNING_SYSTEM.m is the R17 source, the FOUR research flags ', ...
     'are set before running the simulation, and a new simulator ', ...
     'instance has executed.']);
F=load(path,'D');
assert(isfield(F,'D')&&isstruct(F.D)&& ...
    isfield(F.D,'schema')&& ...
    strcmp(F.D.schema,'AF7R58R17_DIAG_ONLY_NO_COMMIT_V1')&& ...
    isfield(F.D,'events'), ...
    '[R17] Invalid diagnostic capture schema.');
D=F.D;
E=D.events;
if isempty(E)
    names={};
else
    names={E.name};
end
has=@(name)any(strcmp(names,name));
startWith=@(prefix)any(strncmp(names,prefix,length(prefix)));
T=D;
T.deadline_stage1_would_reject=has('STAGE1_25MS_WOULD_REJECT');
T.deadline_stage2_would_reject=has('STAGE2_25MS_WOULD_REJECT');
T.stage1_generated=has('STAGE1_GENERATED');
T.stage2_entered=has('STAGE2_ENTERED');
T.stage2_prepared=has('STAGE2_CERTIFIED_AND_PREPARED');
T.stage3_readonly_entered=startWith('STAGE3_PRECOMMIT_');
T.actual_commit_attempted=false;
T.actual_reference_activation=false;
if T.stage3_readonly_entered
    if has('STAGE3_PRECOMMIT_READY_NO_COMMIT')
        T.classification='STAGE2_PASS__STAGE3_PRECOMMIT_PASS__NOT_COMMITTED';
    else
        T.classification='STAGE2_PASS__STAGE3_PRECOMMIT_REJECT__NOT_COMMITTED';
    end
elseif T.stage2_prepared
    T.classification='STAGE2_PREPARED__STAGE3_NOT_REACHED';
elseif T.stage2_entered
    T.classification='STAGE2_ENTERED__REJECTED_OR_EXCEPTION';
elseif T.stage1_generated
    T.classification='STAGE1_GENERATED__STAGE2_NOT_REACHED';
elseif startWith('STAGE1_')
    T.classification='STAGE1_ENTERED__NOT_GENERATED';
else
    T.classification='STAGE1_NOT_OBSERVED';
end

report=fullfile(folder,'AF7R58R17_RESULT.txt');
fid=fopen(report,'w');
assert(fid>=0,'[R17] Could not open diagnostic text report.');
c=onCleanup(@()fclose(fid));
line=@(varargin)fprintf(fid,varargin{:});
line('AF7R58R17 | 25-MS BYPASS RESEARCH DIAGNOSTIC, NO COMMIT\n');
line('classification=%s\n',T.classification);
line('stage1_deadline_would_reject=%d\n',T.deadline_stage1_would_reject);
line('stage2_deadline_would_reject=%d\n',T.deadline_stage2_would_reject);
line('stage1_generated=%d stage2_entered=%d stage2_prepared=%d stage3_readonly_entered=%d\n', ...
    T.stage1_generated,T.stage2_entered,T.stage2_prepared,T.stage3_readonly_entered);
line('native_connected=%.9g native_success=%.9g\n',safe_num(D,'native_connected'),safe_num(D,'native_success'));
line('speed/heading/geometry=%.9g / %.9g / %.9g\n', ...
    safe_num(D,'native_speed_pass'),safe_num(D,'native_heading_pass'),safe_num(D,'native_geometry_pass'));
line('family=%s\n',safe_str(D,'native_family'));
line('stage1_wall_ms=%.9g stage1_cycle_ms=%.9g\n', ...
    safe_num(D,'stage1_ms'),safe_num(D,'stage1_cycle_ms'));
line('stage2_wall_ms=%.9g stage2_cycle_ms=%.9g\n', ...
    safe_num(D,'stage2_ms'),safe_num(D,'stage2_cycle_ms'));
line('stage3_precommit_reason=%s\n',safe_str(D,'stage3_precommit_reason'));
line('commit_attempted=0 activation_confirmed=0\n');
line('NOTE: R17 may exceed 25 ms; never use it as a production timing or flight-safe result.\n');
line('NOTE: Stage3 is a READ-ONLY precommit check; atomic activation and runtime hot guard are NOT exercised.\n');
line('\nTIMELINE (event, sim_t_s, stage_ms, cycle_ms)\n');
for k=1:numel(E)
    line('%s | %.12g | %.9g | %.9g\n',E(k).name,E(k).time_s,E(k).stage_ms,E(k).cycle_ms);
end
save(fullfile(folder,'AF7R58R17_RESULT.mat'),'T','D');
fprintf('========== R17 DIAGNOSTIC ==========\n');
fprintf('classification = %s\n',T.classification);
fprintf('stage1 25ms would reject = %d\n',T.deadline_stage1_would_reject);
fprintf('stage2 25ms would reject = %d\n',T.deadline_stage2_would_reject);
fprintf('stage1 generated / stage2 prepared / stage3 precommit = %d / %d / %d\n', ...
    T.stage1_generated,T.stage2_prepared,T.stage3_readonly_entered);
fprintf('native connected / success = %.9g / %.9g\n', ...
    safe_num(D,'native_connected'),safe_num(D,'native_success'));
fprintf('stage1 / stage2 = %.6f / %.6f ms\n', ...
    safe_num(D,'stage1_ms'),safe_num(D,'stage2_ms'));
fprintf('report = %s\n',report);
end

function x=safe_num(D,n)
x=NaN;
if isfield(D,n) && isnumeric(D.(n)) && isscalar(D.(n))
    x=double(D.(n));
end
end

function s=safe_str(D,n)
s='NOT_AVAILABLE';
if isfield(D,n) && (ischar(D.(n)) || isstring(D.(n)))
    s=char(D.(n));
end
end
