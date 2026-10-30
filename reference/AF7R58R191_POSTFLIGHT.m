function R=AF7R58R191_POSTFLIGHT(obj)
% Offline collection: actual SIM Flight tick wall times plus source reasons.
assert(isa(obj,'REPLANNING_SYSTEM'),'R191:BadInstance',...
 'Pass the replanner_instance from the completed Flight.');
root=fileparts(which('REPLANNING_SYSTEM'));
cnt=min(double(obj.r191_tick_count),size(obj.r191_tick_ring,1));
if double(obj.r191_tick_count)>size(obj.r191_tick_ring,1)
 cap=size(obj.r191_tick_ring,1);
 head=mod(double(obj.r191_tick_count),cap);
 idx=[(head+1):cap,1:head];
 rows=double(obj.r191_tick_ring(idx,:));
else
 rows=double(obj.r191_tick_ring(1:cnt,:));
end
% Row columns: time, preview wall ms, sensor-known count, risk-request,
% reference-version, R188 new-event, C7 generated count, status-code.
R=struct('status','R191_SAME_FLIGHT_SOURCE_TIMING__NO_EXECUTION_PROOF',...
 'source',which('REPLANNING_SYSTEM'),'tick_count',double(obj.r191_tick_count),...
 'tick_ring',rows,'preview_attempts',double(obj.r181_attempt_count),...
 'unique_risk_events',double(obj.r188_new_event_count),...
 'duplicate_suppressed',double(obj.r188_duplicate_suppressed_count),...
 'c7_attempts',double(obj.r189_trial_count),...
 'c7_generated',double(obj.r189_generated_count),...
 'c7_max_trial_ms',double(obj.r189_max_trial_ms),...
 'reference_commits',double(obj.replan_commit_count),...
 'last_risk',obj.r188_last_event,'first_event_snapshot',obj.r191_first_event_snapshot,...
 'event_ring',obj.r188_event_ring,'first_risk_time_s',double(obj.r188_first_event_time_s),...
 'last_c7',obj.r189_last_trial,...
 'last_preview',obj.r181_last_trigger,...
 'full_closed_loop_proof',false,'production_approved',false);
if cnt>0
 R.preview_p50_ms=median(rows(:,2));
 R.preview_p95_ms=r191_pctl(rows(:,2),95);
 R.preview_max_ms=max(rows(:,2));
else
 R.preview_p50_ms=NaN;R.preview_p95_ms=NaN;R.preview_max_ms=NaN;
end
R.interval_summary=zeros(0,6);
if cnt>0
 t=rows(:,1);
 for a=floor(min(t)/5)*5:5:ceil(max(t)/5)*5
   ii=t>=a & t<a+5;
   if any(ii)
     x=rows(ii,2);
     R.interval_summary(end+1,:)=[a,a+5,nnz(ii),median(x),r191_pctl(x,95),max(x)]; %#ok<AGROW>
   end
 end
end
R.mat_file=fullfile(root,'AF7R58R191_LAST_FLIGHT.mat');
R.txt_file=fullfile(root,'AF7R58R191_LAST_FLIGHT.txt');
save(R.mat_file,'R','-v7');
f=fopen(R.txt_file,'wt');
assert(f>=0,'R191:ReportOpen','Cannot write report to reference.');
c=onCleanup(@()fclose(f));
fprintf(f,'status=%s\nsource=%s\n',R.status,R.source);
fprintf(f,'ticks=%d preview_attempts=%d risk_events=%d duplicate_suppressed=%d\n',...
 R.tick_count,R.preview_attempts,R.unique_risk_events,R.duplicate_suppressed);
fprintf(f,'C7 attempts=%d generated=%d commits=%d max_trial_ms=%.5f\n',...
 R.c7_attempts,R.c7_generated,R.reference_commits,R.c7_max_trial_ms);
fprintf(f,'preview p50/p95/max ms = %.5f %.5f %.5f\n',...
 R.preview_p50_ms,R.preview_p95_ms,R.preview_max_ms);
fprintf(f,'5s bins: [start end count p50ms p95ms maxms]\n');
for j=1:size(R.interval_summary,1)
 fprintf(f,'%.2f %.2f %d %.5f %.5f %.5f\n',R.interval_summary(j,:));
end
if isstruct(R.last_preview)&&isfield(R.last_preview,'status')
 fprintf(f,'last_preview_status=%s\n',char(R.last_preview.status));
 if isfield(R.last_preview,'error_message')
 fprintf(f,'last_preview_error=%s\n',char(R.last_preview.error_message));end
end
if isstruct(R.last_c7)&&isfield(R.last_c7,'status')
 fprintf(f,'last_C7_status=%s\n',char(R.last_c7.status));
 if isfield(R.last_c7,'reason')
 fprintf(f,'last_C7_reason=%s\n',char(R.last_c7.reason));end
end
fprintf(f,'CONTINUOUS_HLC_PROOF=0; PHYSICAL_COMMIT=0\n');
end

function y=r191_pctl(x,p)
% Local deterministic percentile (no Statistics Toolbox dependency).
x=sort(double(x(isfinite(x))));
if isempty(x),y=NaN;return;end
q=(numel(x)-1)*p/100+1;k=floor(q);a=q-k;
if k>=numel(x),y=x(end);else,y=(1-a)*x(k)+a*x(k+1);end
end
