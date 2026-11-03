function R=CHECK_TRIGGER_SOURCE_CALLSITE_ABA(A1file,Bfile,A2file)
% Disposable root check. A1/B/A2, independent reference-callsite candidate.
% Q comes from CHECK_TRIGGER_FLIGHT_SPIKES; do NOT conflate event-log parity
% with exact native state/control equality or continuous-time Flight safety.
R=struct('status','UNKNOWN_SOURCE_CALLSITE_ABA', ...
 'decision','DO_NOT_ADOPT_AUTOMATICALLY','reason','', ...
 'equivalent_events',false,'same_4s_native',false, ...
 'all_median_ms',nan(1,3),'flight_median_ms',nan(1,3), ...
 'flight_p95_ms',nan(1,3),'flight_p99_ms',nan(1,3), ...
 'max_ms',nan(1,3),'first_predicted_ms',nan(1,3), ...
 'first_6_prediction_max_ms',nan(1,3), ...
 'new_id_max_ms',nan(1,3),'steady_median_ms',nan(1,3));
try
    paths={A1file,Bfile,A2file};q=cell(1,3);
    keyFields={'time_s','known_ids','new_ids','prediction_available', ...
        'prediction_complete','completed_horizon_s', ...
        'completed_samples','direct_native','trigger_status'};
    for i=1:3
        tmp=load(char(paths{i}),'Q');
        assert(isfield(tmp,'Q')&&isstruct(tmp.Q)&&isscalar(tmp.Q), ...
            'CHECK_TRIGGER_LOG_ABA:Q','Missing Q in %s.',char(paths{i}));
        q{i}=tmp.Q;
        assert(all(isfield(q{i},[keyFields,{'replanner_ms', ...
            'n_predicted','n_full_four_sec','acquisition_indices'}])), ...
            'CHECK_TRIGGER_LOG_ABA:Q','Incomplete flight log.');
        assert(numel(q{i}.replanner_ms)==numel(q{i}.time_s) && ...
            all(isfinite(q{i}.replanner_ms(:))) && ...
            all(q{i}.replanner_ms(:)>=0), ...
            'CHECK_TRIGGER_LOG_ABA:Timing','Invalid times.');
    end
    same=true;
    for k=1:numel(keyFields)
        f=keyFields{k};
        same=same&&isequaln(q{1}.(f),q{2}.(f)) && ...
                   isequaln(q{1}.(f),q{3}.(f));
    end
    same=same && isequaln(q{1}.acquisition_indices,q{2}.acquisition_indices) && ...
        isequaln(q{1}.acquisition_indices,q{3}.acquisition_indices);
    R.equivalent_events=same;
    predicted=cellfun(@(z)double(z.n_predicted),q);
    full=cellfun(@(z)double(z.n_full_four_sec),q);
    R.same_4s_native=all(predicted==predicted(1)) && ...
        all(full==full(1)) && all(full==predicted);
    for i=1:3
        t=double(q{i}.time_s(:));
        y=double(q{i}.replanner_ms(:));
        pred=logical(q{i}.prediction_available(:));
        assert(any(pred),'CHECK_TRIGGER_LOG_ABA:Prediction', ...
            'No actual predictions available.');
        j=find(pred,1,'first');
        firstBlock=find(pred);firstBlock=firstBlock(1:min(6,numel(firstBlock)));
        ids=double(q{i}.acquisition_indices(:));
        ids=ids(isfinite(ids)&ids==fix(ids)&ids>=1&ids<=numel(y));
        steady=pred & (t>=6) & (t<=42);
        R.all_median_ms(i)=median(y);
        R.flight_median_ms(i)=median(y(pred));
        R.flight_p95_ms(i)=prc(y(pred),95);
        R.flight_p99_ms(i)=prc(y(pred),99);
        R.max_ms(i)=max(y);
        R.first_predicted_ms(i)=y(j);
        R.first_6_prediction_max_ms(i)=max(y(firstBlock));
        if ~isempty(ids),R.new_id_max_ms(i)=max(y(ids));end
        if any(steady),R.steady_median_ms(i)=median(y(steady));end
    end
    if ~same || ~R.same_4s_native
        R.status='REJECT_SOURCE_CALLSITE_EVENT_OR_4S_PARITY';
        R.decision='RESTORE_A';
    else
        R.status='PASS_SOURCE_CALLSITE_ABA_EVENT_PARITY__NOT_STATE_SAFETY';
        % Candidate must improve both surrounding baselines on the primary
        % metrics to be considered, and also pass full native state/input
        % and Sim SOURCE schema comparisons before any production adoption.
        metrics={'flight_median_ms','flight_p95_ms','max_ms', ...
            'first_predicted_ms','first_6_prediction_max_ms', ...
            'steady_median_ms'};
        better=true;
        for k=1:numel(metrics)
            a=R.(metrics{k});
            better=better && isfinite(a(2)) && a(2)<min(a([1 3]));
        end
        if better
            R.decision='PROMISING__REPEAT_AND_CHECK_FULL_LOG_SCHEMA';
        else
            R.decision='NO_CONSISTENT_IMPROVEMENT__RESTORE_A';
        end
    end
    R.reason=['A/B/A checks event and timing records ONLY. Run-to-run ' ...
        'JIT/OS variation and source changes need independent control. ' ...
        'No automatic adoption or flight-safety certificate.'];
catch ME
    R.status='FAILED_SOURCE_CALLSITE_ABA';
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_SOURCE_CALLSITE_ABA_RESULT.mat','R');
f=fopen('TRIGGER_SOURCE_CALLSITE_ABA_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\ndecision=%s\nevents=%d\nfull4s=%d\n', ...
        R.status,R.decision,R.equivalent_events,R.same_4s_native);
    names={'all_median_ms','flight_median_ms','flight_p95_ms', ...
        'flight_p99_ms','max_ms','first_predicted_ms', ...
        'first_6_prediction_max_ms','new_id_max_ms','steady_median_ms'};
    for i=1:numel(names)
        v=R.(names{i});
        fprintf(f,'%s_A1_B_A2=%.6f %.6f %.6f\n',names{i},v);
    end
    fprintf(f,'reason=%s\n',R.reason);
    fclose(f);
end
end
function x=prc(y,p)
y=sort(double(y(:)));
x=y(max(1,ceil(numel(y)*p/100)));
end
