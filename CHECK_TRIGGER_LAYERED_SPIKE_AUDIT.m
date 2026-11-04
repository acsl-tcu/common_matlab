function R=CHECK_TRIGGER_LAYERED_SPIKE_AUDIT(logger,replanner_instance)
% Offline, PHASE-RELATIVE decomposition of the ORIGINAL Sim logger.
% Independent of Flight start clock, obstacle IDs, or trajectory family.
% The remainder rep-trigger-sensor is an attribution TARGET, NOT an
% assertion that timing regions are disjoint or that it is JIT compilation.
% No model/flight/controller code is touched; this program is disposable.
R=struct('status','NOT_COMPLETED_LAYERED_SPIKE_AUDIT','reason','', ...
    'first_prediction_index',NaN,'first_prediction_time_s',NaN, ...
    'preprediction_max_ms',NaN,'preprediction_unattributed_max_ms',NaN, ...
    'new_acquisition_max_ms',NaN,'later_no_acquisition_max_ms',NaN, ...
    'shadow_parked_calls',NaN,'safe_to_commit',false);
try
    Q=CHECK_TRIGGER_FLIGHT_SPIKES(logger,replanner_instance);
    n=numel(Q.replanner_ms);
    v=double(Q.replanner_ms(:));tr=double(Q.trigger_ms(:));
    sn=double(Q.sensor_ms(:));t=double(Q.time_s(:));
    pr=logical(Q.prediction_available(:));
    assert(numel(t)==n&&numel(tr)==n&&numel(sn)==n&& ...
        numel(pr)==n&&any(pr), ...
        'CHECK_TRIGGER_LAYERED_SPIKE_AUDIT:Shape');
    first=find(pr,1,'first');
    R.first_prediction_index=first;R.first_prediction_time_s=t(first);
    rem=v-tr-sn;
    before=find((1:n)'<first & isfinite(v)&isfinite(tr)&isfinite(sn));
    assert(~isempty(before),'CHECK_TRIGGER_LAYERED_SPIKE_AUDIT:Before');
    [~,i]=sort(v(before),'descend');pre=before(i(1:min(10,numel(i))));
    R.preprediction_max_ms=max(v(before));
    R.preprediction_unattributed_max_ms=max(rem(before));
    acquired=false(n,1);
    if isfield(Q,'new_ids')
        ids=Q.new_ids;
        if iscell(ids)&&numel(ids)==1&&iscell(ids{1}),ids=ids{1};end
        if iscell(ids)&&numel(ids)==n
            acquired=~cellfun(@isempty,ids(:));
        end
    end
    ididx=find(acquired&pr&isfinite(v));
    if ~isempty(ididx),R.new_acquisition_max_ms=max(v(ididx));end
    later=find(pr & (1:n)'>first & ~acquired & isfinite(v));
    if ~isempty(later),R.later_no_acquisition_max_ms=max(v(later));end
    [~,i]=sort(v(later),'descend');post=later(i(1:min(10,numel(i))));
    S=replanner_instance.get_status();
    if isfield(S,'phase_shadow_parked_calls')
        R.shadow_parked_calls=double(S.phase_shadow_parked_calls);
    end
    R.status='PASS_LOGGED_NONFLIGHT_AND_TRIGGER_LAYER_DIAGNOSTIC__NO_JIT_PROOF';
    fid=fopen('TRIGGER_LAYERED_SPIKE_AUDIT_RESULT.txt','w');
    assert(fid>=0,'CHECK_TRIGGER_LAYERED_SPIKE_AUDIT:File');
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\nfirst_prediction_index=%d\nfirst_prediction_time_s=%.9f\n', ...
        R.status,first,t(first));
    fprintf(fid,'preprediction_max_ms=%.6f\npreprediction_unattributed_max_ms=%.6f\n', ...
        R.preprediction_max_ms,R.preprediction_unattributed_max_ms);
    fprintf(fid,'new_acquisition_max_ms=%.6f\nlater_no_acquisition_max_ms=%.6f\n', ...
        R.new_acquisition_max_ms,R.later_no_acquisition_max_ms);
    fprintf(fid,'shadow_parked_calls=%.0f\n',R.shadow_parked_calls);
    fprintf(fid,'NOTE: remainder=update-trigger-sensor includes memory, shadow, output storing and timer mismatch. It is NOT a JIT measurement.\n');
    fprintf(fid,'--- pre-first-prediction top ---\nk t_s update_ms trigger_ms sensor_ms remainder_ms\n');
    for k=reshape(pre,1,[])
        fprintf(fid,'%d %.9f %.6f %.6f %.6f %.6f\n', ...
            k,t(k),v(k),tr(k),sn(k),rem(k));
    end
    fprintf(fid,'--- later prediction, no new acquisition ---\nk delta_since_first_s update_ms trigger_ms sensor_ms remainder_ms\n');
    for k=reshape(post,1,[])
        fprintf(fid,'%d %.9f %.6f %.6f %.6f %.6f\n', ...
            k,t(k)-t(first),v(k),tr(k),sn(k),rem(k));
    end
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
    fid=fopen('TRIGGER_LAYERED_SPIKE_AUDIT_RESULT.txt','w');
    if fid>=0
        fprintf(fid,'status=%s\nreason=%s\n',R.status,R.reason);
        fclose(fid);
    end
end
save('TRIGGER_LAYERED_SPIKE_AUDIT_RESULT.mat','R');
end
