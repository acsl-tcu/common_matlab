function A=AF7R58R90_ATTEMPT_REPORT(R89,R89_first)
% Zero-flight-rerun, zero-production-change report of actual R89 results.
% Status NOT_RUN is NOT a kinematic failure: inspect enclosing R88 status.
assert(isstruct(R89)&&isfield(R89,'r81'), ...
    'R90:NeedOriginalR89ReplayResult');
if nargin<2, R89_first=struct();end
A=struct('status','R90_R89_FAILURE_CLASSIFIED_READ_ONLY', ...
    'main_replay_status',char(R89.status), ...
    'first_replay_status','NOT_SUPPLIED', ...
    'attempts',0,'rejected_kinematic',0,'cause_counts',struct(), ...
    'rows',table(),'first_error','', ...
    'production_modified',false,'production_commit_authorized',false);
if isfield(R89_first,'status')
    A.first_replay_status=char(R89_first.status);
    if isfield(R89_first,'error_message')
        A.first_error=char(R89_first.error_message);
    end
end
R=R89.r81;
if isfield(R,'attempts'),A.attempts=R.attempts;end
if isfield(R,'rejected_kinematic'),A.rejected_kinematic=R.rejected_kinematic;end
if isfield(R,'kinematic_attempt_log')&&~isempty(R.kinematic_attempt_log)
    X=R.kinematic_attempt_log;
    A.rows=struct2table(X);
    names=unique(string({X.status}));
    for i=1:numel(names)
        key=matlab.lang.makeValidName(char(names(i)));
        A.cause_counts.(key)=sum(strcmp({X.status},char(names(i))));
    end
end
fprintf('\nR90 19.75 result = %s | attempts=%d | kinematic rejects=%d\n', ...
    A.main_replay_status,A.attempts,A.rejected_kinematic);
fprintf('13.325 replay status = %s\n',A.first_replay_status);
if ~isempty(A.first_error),fprintf('13.325 error = %s\n',A.first_error);end
if ~isempty(A.rows)
    disp(A.rows(:,{'duration_s','phase_delay_s','status','nodes', ...
      'start_vQ_m_s','end_vQ_m_s','uncertified_interval_s', ...
      'specific_force_lower_m_s2','uav_speed_upper_m_s'}));
end
keys=fieldnames(A.cause_counts);
for i=1:numel(keys)
    fprintf('%s = %d\n',keys{i},A.cause_counts.(keys{i}));
end
% Store alongside the audit function, not in repository root or main.
root=fileparts(mfilename('fullpath'));
path=fullfile(root,'AF7R58R90_ATTEMPT_REPORT.txt');
fid=fopen(path,'wt');
if fid>=0
    cleaner=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'R90 source-bound attempt report; NOT a safety certificate\n');
    fprintf(fid,'main=%s first=%s attempts=%d\n', ...
        A.main_replay_status,A.first_replay_status,A.attempts);
    fprintf(fid,'first_error=%s\n',A.first_error);
    for i=1:numel(keys)
        fprintf(fid,'%s=%d\n',keys{i},A.cause_counts.(keys{i}));
    end
    if ~isempty(A.rows)
        for j=1:height(A.rows)
            rr=X(j);
            fprintf(fid,'attempt=%d T=%.17g delay=%.17g status=%s nodes=%d startVQ=%.17g endVQ=%.17g interval=[%.17g %.17g] sigmaLB=%.17g VQUB=%.17g\n', ...
                j,rr.duration_s,rr.phase_delay_s,char(rr.status),rr.nodes, ...
                rr.start_vQ_m_s,rr.end_vQ_m_s,rr.uncertified_interval_s(1), ...
                rr.uncertified_interval_s(2),rr.specific_force_lower_m_s2, ...
                rr.uav_speed_upper_m_s);
        end
    end
    A.report_file=path;
else
    A.report_file='WRITE_FAILED';
end
end
