function A=AF7R58R91_REPLAY(agent,work)
% Read-only postflight replay of the latest EXACT R88 episode snapshot.
% No replay if this session has no snapshot; never use final EKF mass for
% another Flight time. No Flight or production state modification.
assert(nargin==2&&isstruct(work),'R91:ExplicitWorkOptionsRequired');
C=AF7R58R88_CAPTURE_STATUS(agent);
A=struct('status','NOT_RUN','selected_time_s',NaN, ...
    'outer_status','','r81_status','','attempts',0, ...
    'status_counts',struct(),'first_problem_interval_s',[NaN NaN], ...
    'first_violation_speed_m_s',NaN,'production_modified',false);
if ~isfield(C,'times_s')||isempty(C.times_s)
 A.status='NO_SAVED_R88_EPISODE';return;
end
A.selected_time_s=double(C.times_s(end));
R=AF7R58R88_RUN_AFTER_FLIGHT(agent,A.selected_time_s,work);
A.outer_status=char(R.status);
if ~isfield(R,'r81')||~isstruct(R.r81)
 A.status='R88_NO_R81_STRUCT';return;
end
X=R.r81;
if isfield(X,'status'),A.r81_status=char(X.status);end
if isfield(X,'attempts'),A.attempts=double(X.attempts);end
if isfield(X,'kinematic_attempt_log')&&~isempty(X.kinematic_attempt_log)
 J=X.kinematic_attempt_log;labels=unique(string({J.status}));
 for k=1:numel(labels)
  name=matlab.lang.makeValidName(char(labels(k)));
  A.status_counts.(name)=sum(strcmp({J.status},char(labels(k))));
 end
 A.first_problem_interval_s=J(1).uncertified_interval_s;
end
A.status='R91_SAVED_EPISODE_REPLAYED__NOT_COMMITTABLE';
disp(A);
% Do not serialize potentially large native packets; small report only.
root=fileparts(mfilename('fullpath'));
fn=fullfile(root,'AF7R58R91_REPLAY_REPORT.txt');
fid=fopen(fn,'wt');
if fid>=0
 cleaner=onCleanup(@()fclose(fid)); %#ok<NASGU>
 fprintf(fid,'R91 exact R88 snapshot replay - REFERENCE ONLY\n');
 fprintf(fid,'t=%.17g outer=%s r81=%s attempts=%d\n', ...
  A.selected_time_s,A.outer_status,A.r81_status,A.attempts);
 keys=fieldnames(A.status_counts);
 for k=1:numel(keys)
  fprintf(fid,'%s=%d\n',keys{k},A.status_counts.(keys{k}));
 end
 fprintf(fid,'No production trigger/generator/commit modified.\n');
end
end
