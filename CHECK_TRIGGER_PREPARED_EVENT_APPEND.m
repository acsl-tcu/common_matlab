function A=CHECK_TRIGGER_PREPARED_EVENT_APPEND()
% Offline, deletion-ready audit. Run from any pwd; outputs beside this file.
% Real production sensor/plant/HLC/runner are NOT exercised or modified.
root=fileparts(mfilename('fullpath'));
A=struct('status','FAIL_PREPARED_EVENT_INCREMENTAL', ...
    'cases',0,'failures',0,'negative_cases',0, ...
    'legacy_event_median_ms',NaN, ...
    'incremental_event_median_ms',NaN, ...
    'speedup_unproven',true,'reason','');
try
    for trial=1:501
        nold=mod(trial*37,113);
        nnew=1+mod(trial*7,5);
        % Nonconsecutive arbitrary positive IDs; never fixed known slots.
        previous_ids=5000*trial+(1:nold)*3;
        fresh_ids=5000*trial+1000+(1:nnew)*7;
        previous_shapes=make_shapes(previous_ids);
        fresh_shapes=make_shapes(fresh_ids);
        [shapes,ids]=TRIGGER_OBSERVED_PREPARED_APPEND( ...
            previous_shapes,previous_ids,fresh_shapes,fresh_ids);
        reference_shapes=OBSERVED_OBSTACLE_MEMORY.append_row_structs( ...
            previous_shapes,fresh_shapes);
        reference_ids=[previous_ids fresh_ids];
        assert(isequaln(shapes,reference_shapes) && ...
            isequal(ids,reference_ids) && ...
            isequal(reshape(double([shapes.id]),1,[]),ids), ...
            'CHECK_TRIGGER_PREPARED_EVENT_APPEND:Parity', ...
            'Incremental append differs from original full-history oracle.');
        A.cases=A.cases+1;
    end
    old=make_shapes([11 17 31]);added=make_shapes([39 55]);
    % These cases must refuse, not silently bind unknown geometry.
    reject(@()TRIGGER_OBSERVED_PREPARED_APPEND( ...
        old,[11 17 31],added,[39 39]));
    reject(@()TRIGGER_OBSERVED_PREPARED_APPEND( ...
        old,[11 17 31],added,[55 39]));
    reject(@()TRIGGER_OBSERVED_PREPARED_APPEND( ...
        old,[11 17 31],make_shapes([17]),17));
    reject(@()TRIGGER_OBSERVED_PREPARED_APPEND( ...
        old,[11 17 31],added,[39 0]));
    reject(@()TRIGGER_OBSERVED_PREPARED_APPEND( ...
        old,[11 17],added,[39 55]));
    % The benchmark is EVENT-ONLY. It cannot establish a full-flight WCET.
    prior_ids=1000+(1:80)*3;
    prior=make_shapes(prior_ids);
    add_ids=[2001 2011];add=make_shapes(add_ids);
    old_ms=zeros(1,151);new_ms=zeros(1,151);
    for rep=1:151
        c=tic;
        legacy=OBSERVED_OBSTACLE_MEMORY.append_row_structs(prior,add);
        full_id_list=[prior_ids add_ids];
        assert(numel(legacy)==numel(full_id_list) && ...
            isequal(reshape(double([legacy.id]),1,[]),full_id_list));
        old_ms(rep)=toc(c)*1000;
        c=tic;
        [faster,fast_ids]=TRIGGER_OBSERVED_PREPARED_APPEND( ...
            prior,prior_ids,add,add_ids);
        assert(isequal(fast_ids,full_id_list) && ...
            isequal(faster,legacy));
        new_ms(rep)=toc(c)*1000;
    end
    A.legacy_event_median_ms=median(old_ms);
    A.incremental_event_median_ms=median(new_ms);
    A.status='PASS_OBSERVED_PREPARED_INDUCTIVE_PARITY__NOT_WCET';
catch ME
    A.failures=A.failures+1;
    A.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_PREPARED_EVENT_APPEND_RESULT.mat'),'A');
fid=fopen(fullfile(root,'TRIGGER_PREPARED_EVENT_APPEND_RESULT.txt'),'wt');
if fid>=0
    fprintf(fid,'status=%s\ncases=%d\nfailures=%d\nnegative_cases=%d\n', ...
        A.status,A.cases,A.failures,A.negative_cases);
    fprintf(fid,'legacy_event_median_ms=%.9f\nincremental_event_median_ms=%.9f\nreason=%s\n', ...
        A.legacy_event_median_ms,A.incremental_event_median_ms,A.reason);
    fclose(fid);
end
    function reject(f)
        failed=false;
        try
            f();
        catch
            failed=true;
        end
        assert(failed,'CHECK_TRIGGER_PREPARED_EVENT_APPEND:FailClosed', ...
            'Invalid sensor observation was accepted.');
        A.negative_cases=A.negative_cases+1;
        A.cases=A.cases+1;
    end
end
function shapes=make_shapes(ids)
ids=reshape(ids,1,[]);
blank=struct('id',0,'center0',zeros(3,1), ...
    'bounding_radius_m',1,'is_stationary',true, ...
    'type','sphere','dimensions',1,'rotation',eye(3));
shapes=repmat(blank,1,numel(ids));
for k=1:numel(ids)
    shapes(k).id=ids(k);
    shapes(k).center0=[ids(k);k;-k]/1000;
end
end
