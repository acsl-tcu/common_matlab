function R=CHECK_TRIGGER_ABA_REPEAT_DECISION(A1file,Bfile,A2file)
% Disposable common_matlab-root comparison; no new Flight/code executed.
% Compares A1-B-A2 with exact timeline/event parity and key timing statistics.
% A single A-B improvement must never be accepted without an A recovery.
R=struct('status','UNKNOWN_ABA','reason','', ...
    'equivalent',false,'decision','NO_AUTOMATIC_ADOPTION', ...
    'median_A1',NaN,'median_B',NaN,'median_A2',NaN, ...
    'max_A1',NaN,'max_B',NaN,'max_A2',NaN, ...
    'initial_A1',NaN,'initial_B',NaN,'initial_A2',NaN, ...
    'id3_A1',NaN,'id3_B',NaN,'id3_A2',NaN);
try
    a=load(A1file,'Q');b=load(Bfile,'Q');c=load(A2file,'Q');
    assert(isfield(a,'Q')&&isfield(b,'Q')&&isfield(c,'Q'), ...
        'CHECK_TRIGGER_ABA:Missing','All MAT files require Q.');
    a=a.Q;b=b.Q;c=c.Q;
    fields={'time_s','known_ids','new_ids','prediction_available', ...
        'prediction_complete','completed_horizon_s', ...
        'completed_samples','direct_native','trigger_status'};
    equivalent=true;
    for k=1:numel(fields)
        f=fields{k};
        equivalent=equivalent && isequaln(a.(f),b.(f)) && ...
            isequaln(a.(f),c.(f));
    end
    equivalent=equivalent && ...
        a.n_full_four_sec==b.n_full_four_sec && ...
        a.n_full_four_sec==c.n_full_four_sec;
    R.equivalent=equivalent;
    Qs={a,b,c};
    for k=1:3
        q=Qs{k};times=double(q.time_s(:));
        assert(~isempty(times)&&any(abs(times-5)<1e-8), ...
            'CHECK_TRIGGER_ABA:Flight','Missing first Flight time.');
        vals=double(q.replanner_ms(:));
        m(k)=median(vals);x(k)=max(vals); %#ok<AGROW>
        initial(k)=vals(find(abs(times-5)<1e-8,1)); %#ok<AGROW>
        idx=find(abs(times-5.125)<1e-8,1);
        id3(k)=NaN;
        if ~isempty(idx),id3(k)=vals(idx);end %#ok<AGROW>
    end
    [R.median_A1,R.median_B,R.median_A2]=deal(m(1),m(2),m(3));
    [R.max_A1,R.max_B,R.max_A2]=deal(x(1),x(2),x(3));
    [R.initial_A1,R.initial_B,R.initial_A2]=deal(initial(1),initial(2),initial(3));
    [R.id3_A1,R.id3_B,R.id3_A2]=deal(id3(1),id3(2),id3(3));
    if ~equivalent
        R.status='REJECT_ABA__EVENT_OR_PREDICTION_DIFFERENCE';
        R.decision='REJECT';
    else
        R.status='PASS_ABA_LOG_PARITY__TIMING_REVIEW_REQUIRED';
        % Means only that B beats both surrounding A measurements on the
        % requested metrics. No event parity implies numerical state parity.
        if m(2)<min(m([1,3])) && x(2)<min(x([1,3])) && ...
             initial(2)<min(initial([1,3])) && ...
             id3(2)<min(id3([1,3]))
            R.decision='POTENTIAL_BENEFIT__VERIFY_FULL_OUTPUTS_AND_REPEAT';
        else
            R.decision='NO_DEMONSTRATED_BENEFIT__RESTORE_BASELINE';
        end
    end
    R.reason=['Only logged event/horizon equivalence and measured times; ', ...
        'not a bitwise state/input parity test or safety certificate.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_ABA_REPEAT_RESULT.mat','R');
fd=fopen('TRIGGER_ABA_REPEAT_RESULT.txt','w');
if fd>=0
 fprintf(fd,'status=%s\ndecision=%s\nequivalent=%d\n', ...
     R.status,R.decision,R.equivalent);
 fprintf(fd,'median_A1_B_A2=%.6f %.6f %.6f ms\n', ...
     R.median_A1,R.median_B,R.median_A2);
 fprintf(fd,'max_A1_B_A2=%.6f %.6f %.6f ms\n', ...
     R.max_A1,R.max_B,R.max_A2);
 fprintf(fd,'first_A1_B_A2=%.6f %.6f %.6f ms\n', ...
     R.initial_A1,R.initial_B,R.initial_A2);
 fprintf(fd,'id3_A1_B_A2=%.6f %.6f %.6f ms\nreason=%s\n', ...
     R.id3_A1,R.id3_B,R.id3_A2,R.reason);
 fclose(fd);
end
end
