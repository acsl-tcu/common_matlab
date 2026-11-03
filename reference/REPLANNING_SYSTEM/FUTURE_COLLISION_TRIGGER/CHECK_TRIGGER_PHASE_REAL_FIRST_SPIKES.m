function R=CHECK_TRIGGER_PHASE_REAL_FIRST_SPIKES(logger,replanner_instance)
% Offline audit. Do not assume Flight begins at 5s, or at a known index.
% Separate nonpredicting Arming/Takeoff/phase changes, first predicted
% cycles, acquired-obstacle events and later no-acquisition cycles.
R=struct('status','NOT_COMPLETED_PHASE_SPIKE_AUDIT','reason','', ...
    'first_predicted_index',NaN,'first_predicted_time_s',NaN, ...
    'preprediction_max_ms',NaN,'first_eight_max_ms',NaN, ...
    'later_no_new_id_max_ms',NaN,'top_pre_idx',[], ...
    'top_later_idx',[],'safe_to_commit',false);
try
    Q=CHECK_TRIGGER_FLIGHT_SPIKES(logger,replanner_instance);
    t=double(Q.time_s(:));rep=double(Q.replanner_ms(:));
    trig=double(Q.trigger_ms(:));sens=double(Q.sensor_ms(:));
    pred=logical(Q.prediction_available(:));
    n=numel(t);
    assert(n==numel(rep)&&n==numel(trig)&&n==numel(sens)&&any(pred), ...
        'CHECK_TRIGGER_PHASE_REAL_FIRST_SPIKES:BadLogs');
    p=find(pred); first=p(1);
    R.first_predicted_index=first;R.first_predicted_time_s=t(first);
    before=find(~pred & (1:n)'<first & isfinite(rep));
    if ~isempty(before)
        R.preprediction_max_ms=max(rep(before));
        [~,ii]=sort(rep(before),'descend');
        R.top_pre_idx=before(ii(1:min(12,numel(ii))));
    end
    R.first_eight_max_ms=max(rep(p(1:min(8,numel(p)))));
    news=false(n,1);
    if isfield(Q,'new_ids')
        x=Q.new_ids;
        if iscell(x)&&numel(x)==1&&iscell(x{1}),x=x{1};end
        if iscell(x)&&numel(x)==n,news=~cellfun(@isempty,x(:));end
    end
    later=find(pred & (1:n)'>p(min(8,numel(p))) & ~news & isfinite(rep));
    if ~isempty(later)
        R.later_no_new_id_max_ms=max(rep(later));
        [~,ii]=sort(rep(later),'descend');
        R.top_later_idx=later(ii(1:min(12,numel(ii))));
    end
    R.status='PASS_PHASE_RELATIVE_FIRST_AND_NONFLIGHT_DECOMPOSITION';
    fid=fopen('TRIGGER_PHASE_REAL_FIRST_SPIKES_RESULT.txt','w');
    assert(fid>=0,'CHECK_TRIGGER_PHASE_REAL_FIRST_SPIKES:Output');
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\nfirst_predicted_index=%d\nfirst_predicted_time_s=%.9f\n', ...
        R.status,first,t(first));
    fprintf(fid,'preprediction_max_ms=%.6f\nfirst_eight_max_ms=%.6f\n', ...
        R.preprediction_max_ms,R.first_eight_max_ms);
    fprintf(fid,'later_no_new_id_max_ms=%.6f\n',R.later_no_new_id_max_ms);
    fprintf(fid,'Index and time are from the ACTUAL run, not a fixed phase schedule.\n');
    fprintf(fid,'--- Before first prediction (can include Arming/Takeoff) ---\n');
    fprintf(fid,'k t_s replanner_ms trigger_ms sensor_ms\n');
    for k=reshape(R.top_pre_idx,1,[])
        fprintf(fid,'%d %.9f %.6f %.6f %.6f\n',k,t(k),rep(k),trig(k),sens(k));
    end
    fprintf(fid,'--- Later prediction with no new obstacle ID ---\n');
    fprintf(fid,'k dt_since_first_pred_s replanner_ms trigger_ms sensor_ms\n');
    for k=reshape(R.top_later_idx,1,[])
        fprintf(fid,'%d %.9f %.6f %.6f %.6f\n',k,t(k)-t(first),rep(k),trig(k),sens(k));
    end
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
    fid=fopen('TRIGGER_PHASE_REAL_FIRST_SPIKES_RESULT.txt','w');
    if fid>=0
        fprintf(fid,'status=%s\nreason=%s\n',R.status,R.reason);fclose(fid);
    end
end
save('TRIGGER_PHASE_REAL_FIRST_SPIKES_RESULT.mat','R');
end
