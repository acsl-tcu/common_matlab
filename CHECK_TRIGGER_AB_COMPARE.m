function A=CHECK_TRIGGER_AB_COMPARE(baseline_mat,candidate_mat)
% Disposable comparison. Reads the existing CHECK_TRIGGER_FLIGHT_SPIKES Q
% records only. NO Flight, sensor, obstacle truth or reference is evaluated.
% Does NOT automatically approve adoption or claim continuous-time safety.
A=struct('status','NOT_COMPARED','equivalent',false,'reason','', ...
    'baseline_file',char(baseline_mat),'candidate_file',char(candidate_mat), ...
    'baseline_median_ms',NaN,'candidate_median_ms',NaN, ...
    'baseline_p95_ms',NaN,'candidate_p95_ms',NaN, ...
    'baseline_max_ms',NaN,'candidate_max_ms',NaN, ...
    'baseline_first_flight_ms',NaN,'candidate_first_flight_ms',NaN, ...
    'baseline_new_id_5_125_ms',NaN,'candidate_new_id_5_125_ms',NaN, ...
    'baseline_acquisition_max_ms',NaN,'candidate_acquisition_max_ms',NaN, ...
    'baseline_steady_ms',NaN,'candidate_steady_ms',NaN, ...
    'baseline_prediction_count',NaN,'candidate_prediction_count',NaN, ...
    'baseline_complete_4s_count',NaN,'candidate_complete_4s_count',NaN, ...
    'exact_timeline',false,'same_event_ids',false, ...
    'same_detection_and_trigger_statuses',false, ...
    'same_native_and_horizon_statuses',false, ...
    'adopted',false,'decision','REVIEW_ABA_REPEATS_REQUIRED');
try
    bs=load(char(baseline_mat),'Q');cs=load(char(candidate_mat),'Q');
    assert(isfield(bs,'Q')&&isfield(cs,'Q'), ...
        'CHECK_TRIGGER_AB_COMPARE:File','Both MAT files must contain Q.');
    b=bs.Q;c=cs.Q;
    assert(numel(b.time_s)==numel(c.time_s)&&numel(b.time_s)>0, ...
        'CHECK_TRIGGER_AB_COMPARE:Frames','Frame count differs.');
    A.exact_timeline=isequaln(b.time_s,c.time_s);
    A.same_event_ids=isequaln(b.known_ids,c.known_ids)&& ...
        isequaln(b.new_ids,c.new_ids) && ...
        isequaln(b.acquisition_indices,c.acquisition_indices);
    A.same_detection_and_trigger_statuses= ...
        isequaln(b.prediction_available,c.prediction_available) && ...
        isequaln(b.trigger_status,c.trigger_status);
    A.same_native_and_horizon_statuses= ...
        isequaln(b.direct_native,c.direct_native) && ...
        isequaln(b.prediction_complete,c.prediction_complete) && ...
        isequaln(b.completed_horizon_s,c.completed_horizon_s) && ...
        isequaln(b.completed_samples,c.completed_samples);
    A.equivalent=A.exact_timeline && A.same_event_ids && ...
        A.same_detection_and_trigger_statuses && ...
        A.same_native_and_horizon_statuses;
    A.baseline_prediction_count=b.n_predicted;
    A.candidate_prediction_count=c.n_predicted;
    A.baseline_complete_4s_count=b.n_full_four_sec;
    A.candidate_complete_4s_count=c.n_full_four_sec;
    A.baseline_median_ms=median(b.replanner_ms);
    A.candidate_median_ms=median(c.replanner_ms);
    A.baseline_p95_ms=local_p95(b.replanner_ms);
    A.candidate_p95_ms=local_p95(c.replanner_ms);
    A.baseline_max_ms=max(b.replanner_ms);
    A.candidate_max_ms=max(c.replanner_ms);
    idx=find(abs(double(b.time_s)-5)<1e-8,1);
    if ~isempty(idx)
        A.baseline_first_flight_ms=b.replanner_ms(idx);
        A.candidate_first_flight_ms=c.replanner_ms(idx);
    end
    i5125=find(abs(double(b.time_s)-5.125)<1e-8,1);
    if ~isempty(i5125)
        A.baseline_new_id_5_125_ms=b.replanner_ms(i5125);
        A.candidate_new_id_5_125_ms=c.replanner_ms(i5125);
    end
    inds=b.acquisition_indices;
    inds=double(inds(:));inds=inds(isfinite(inds)&inds>=1&inds<=numel(b.time_s));
    inds=unique(round(inds));
    if ~isempty(inds)
        A.baseline_acquisition_max_ms=max(b.replanner_ms(inds));
        A.candidate_acquisition_max_ms=max(c.replanner_ms(inds));
    end
    j=double(b.time_s)>=6 & double(b.time_s)<=42 & logical(b.prediction_available);
    if any(j)
        A.baseline_steady_ms=median(b.replanner_ms(j));
        A.candidate_steady_ms=median(c.replanner_ms(j));
    end
    if A.equivalent && A.baseline_prediction_count==A.candidate_prediction_count && ...
            A.baseline_complete_4s_count==A.candidate_complete_4s_count
        A.status='PASS_LOGGED_EVENT_AND_PREDICTION_PARITY__NOT_SAFETY';
    else
        A.status='REJECT_EXPERIMENT__FLIGHT_BEHAVIOR_DIFFERS';
        A.decision='DO_NOT_ADOPT';
    end
    A.reason='Q captures logged events/horizons, not bitwise predicted state trajectories or proof of Flight safety.';
catch ME
    A.status='COMPARE_FAILED__DO_NOT_ADOPT';
    A.decision='DO_NOT_ADOPT';
    A.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_AB_COMPARISON_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ndecision=%s\nreason=%s\n',A.status,A.decision,A.reason);
    fprintf(fid,'equivalent=%d\nexact_timeline=%d\nsame_event_ids=%d\nsame_status=%d\nsame_native_horizon=%d\n', ...
        A.equivalent,A.exact_timeline,A.same_event_ids, ...
        A.same_detection_and_trigger_statuses,A.same_native_and_horizon_statuses);
    fprintf(fid,'median_baseline/candidate=%.6f / %.6f ms\n',A.baseline_median_ms,A.candidate_median_ms);
    fprintf(fid,'p95_baseline/candidate=%.6f / %.6f ms\n',A.baseline_p95_ms,A.candidate_p95_ms);
    fprintf(fid,'max_baseline/candidate=%.6f / %.6f ms\n',A.baseline_max_ms,A.candidate_max_ms);
    fprintf(fid,'first_Flight_baseline/candidate=%.6f / %.6f ms\n',A.baseline_first_flight_ms,A.candidate_first_flight_ms);
    fprintf(fid,'new_id_5_125_baseline/candidate=%.6f / %.6f ms\n',A.baseline_new_id_5_125_ms,A.candidate_new_id_5_125_ms);
    fprintf(fid,'acquisition_max_baseline/candidate=%.6f / %.6f ms\n',A.baseline_acquisition_max_ms,A.candidate_acquisition_max_ms);
    fprintf(fid,'steady_6_42_baseline/candidate=%.6f / %.6f ms\n',A.baseline_steady_ms,A.candidate_steady_ms);
    fprintf(fid,'prediction_baseline/candidate=%d / %d\n',A.baseline_prediction_count,A.candidate_prediction_count);
    fprintf(fid,'full4s_baseline/candidate=%d / %d\n',A.baseline_complete_4s_count,A.candidate_complete_4s_count);
    fclose(fid);
end
save('TRIGGER_AB_COMPARISON_RESULT.mat','A');
fprintf('%s\n',A.status);
end
function v=local_p95(x)
 x=sort(double(x(:)));x=x(isfinite(x));
 if isempty(x),v=NaN;return;end
 v=x(max(1,ceil(.95*numel(x))));
end
