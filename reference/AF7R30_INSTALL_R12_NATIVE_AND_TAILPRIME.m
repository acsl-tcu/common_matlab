function OUT = AF7R30_INSTALL_R12_NATIVE_AND_TAILPRIME()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R30 | INSTALL R12 NATIVE SUPPORT + TAIL PRIME\n');
fprintf('====================================================================================================\n');

assert(exist('af7r30_support_hull_mex','file')==3, ...
    '[AF7R30] Build MEX first with AF7R30_BUILD_R12_NATIVE_MEX.');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R30] source unresolved.');
raw=fileread(src);
assert(contains(raw,'AF7R27-R10'),'[AF7R30] R10 missing.');
assert(contains(raw,'AF7R29-R11'),'[AF7R30] R11 missing.');
assert(~contains(raw,'AF7R30-R12'),'[AF7R30] R12 already installed.');

here=fileparts(mfilename('fullpath'));
pt=fileread(fullfile(here,'AF7R30_POINT_SNIPPET.txt'));
sg=fileread(fullfile(here,'AF7R30_SEGMENT_SNIPPET.txt'));
hu=fileread(fullfile(here,'AF7R30_HULL_SNIPPET.txt'));
tm=fileread(fullfile(here,'AF7R30_TAIL_METHOD_SNIPPET.txt'));

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss'));
backup=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_R12_' stamp '.m']);
copyfile(src,backup);

anchor='        ENABLE_NONCOMMIT_DIAGNOSTIC_EVAL = 0; % failed/deadline-missed proposal theorem diagnostics are deferred';
idx=strfind(raw,anchor);
assert(numel(idx)==1,'[AF7R30] R11 switch anchor not unique.');
ins=[anchor newline ...
'        r12_tail_prime_done = false;' newline ...
'        r12_tail_prime_ms = NaN;' newline];
raw=[raw(1:idx-1) ins raw(idx+length(anchor):end)];

raw=replace_between(raw, ...
    '        function [margin,ok] = best_point_support_margin', ...
    '        function [margin,ok] = best_segment_support_margin',pt,'point support');

raw=replace_between(raw, ...
    '        function [margin,ok] = best_segment_support_margin', ...
    '        function [margin_m,ok] = support_margin_point',sg,'segment support');

raw=replace_between(raw, ...
    '        function keep = r10_upper_envelope_row_indices', ...
    '        function CANDS = enumerate_piecewise_sign_qp_recursive',hu,'upper hull');

anchorMethod='        function warmup_online_path(obj,time,varargin_in,xd_now)';
idx=strfind(raw,anchorMethod);
assert(numel(idx)==1,'[AF7R30] warmup method anchor not unique.');
raw=[raw(1:idx-1) tm raw(idx:end)];

anchorCall=[ ...
'                obj.write_nominal_output(xd_nom);' newline ...
'                result_out=obj.result;' newline ...
'                return;' newline ...
'            end' newline];
idx=strfind(raw,anchorCall);
assert(~isempty(idx),'[AF7R30] nonprotected return anchor not found.');
idx=idx(1);
call=[ ...
'                if isequal(cha,''t'') && ~obj.r12_tail_prime_done' newline ...
'                    [tf12,ok12]=obj.try_resolve_takeoff_end_time_nonfatal();' newline ...
'                    t12=double(time.t);' newline ...
'                    if ok12 && isfinite(tf12) && isfinite(t12) && t12<tf12 && ...' newline ...
'                            (tf12-t12)<=2.5*double(obj.prediction_dt)' newline ...
'                        obj.r12_tail_prime_hot_path(time,varargin,xd_nom,tf12);' newline ...
'                    end' newline ...
'                end' newline ...
anchorCall];
raw=[raw(1:idx-1) call raw(idx+length(anchorCall):end)];

hdr='% AF7R29-R11: noncommittable proposal diagnostics deferred from production hot path.';
assert(contains(raw,hdr),'[AF7R30] R11 header missing.');
raw=strrep(raw,hdr,[hdr newline ...
'% AF7R30-R12: native exact-support/hull helpers + last-takeoff hot-path prime.']);

compact=regexprep(raw,'\s+','');
assert(~contains(compact,'''sampled_acceptance'',true), ...
    '[AF7R30] sampled acceptance true detected.');
assert(contains(raw,'AF7R30-R12'));
assert(contains(raw,'af7r30_support_hull_mex'));
assert(contains(raw,'r12_tail_prime_hot_path'));

fid=fopen(src,'w');
assert(fid>=0,'[AF7R30] cannot write source.');
c=onCleanup(@()fclose(fid));
fwrite(fid,raw,'char');
clear c;

OUT=struct('schema','AF7R30_INSTALL_R12_NATIVE_AND_TAILPRIME_V1', ...
    'source',src,'backup',backup,'installed',true, ...
    'safety_acceptance_changed',false,'commit_policy_changed',false, ...
    'hard_deadline_changed',false);
assignin('base','AF7R30_INSTALL_R12_NATIVE_AND_TAILPRIME_OUT',OUT);

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',backup);
fprintf(' native point/segment support installed  = 1\n');
fprintf(' native upper-hull installed             = 1\n');
fprintf(' last-takeoff tail prime installed       = 1\n');
fprintf(' safety/commit/deadline modified         = 0\n');
fprintf('====================================================================================================\n');
end

function out=replace_between(in,startPat,endPat,repl,label)
s=strfind(in,startPat); e=strfind(in,endPat);
assert(numel(s)==1,'[AF7R30] %s start count=%d',label,numel(s));
e=e(e>s);
assert(numel(e)==1,'[AF7R30] %s end count=%d',label,numel(e));
out=[in(1:s-1) repl in(e:end)];
end
