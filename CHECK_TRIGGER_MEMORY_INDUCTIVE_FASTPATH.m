function R=CHECK_TRIGGER_MEMORY_INDUCTIVE_FASTPATH()
% Temporary common_matlab/ auditor. No source-world access, no Flight or EKF.
% Full changed-ID validation is retained. EXACT unchanged IDs inherit the
% verified uniqueness invariant; this is not detection-mask precomputation.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_MEMORY_INDUCTIVE_FASTPATH','reason','', ...
    'cases',0,'failures',0,'new_observations',0,'retained_checks',0, ...
    'invalid_inputs_rejected',0,'repeat_frame_checked',false, ...
    'empty_new_event_cases',0,'known_singleton_cases',0, ...
    'safety_certified',false,'wcet_certified',false);
saved_rng=rng;cleanup=onCleanup(@()rng(saved_rng)); %#ok<NASGU>
try
    rng(1199,'twister');
    template=struct('id',{},'motion',{},'position',{});
    memory=OBSERVED_OBSTACLE_MEMORY(template);
    % Arbitrary IDs, gaps and reentries; no 2^N combination table.
    possible=[3 17 58 102 400 987 1502 10101 50031];
    known=zeros(1,0);
    for k=1:1500
        n=randi([0,numel(possible)]);
        perm=randperm(numel(possible));
        ids=possible(perm(1:n));
        obstacles=reshape(template,1,[]);
        for j=1:numel(ids)
            o=struct('id',ids(j),'motion','static', ...
                     'position',[double(ids(j));0;0]);
            obstacles(end+1)=o; %#ok<AGROW>
        end
        p=[0.01*k;sin(k);cos(k)];
        f=struct('time_s',0.025*k, ...
            'uav_position_ekf_m',p,'detected_ids',ids, ...
            'detected_count',numel(ids),'detected_obstacles',obstacles);
        old=known;
        % MATLAB logical indexing of a scalar can produce a 0x0 empty,
        % whereas the memory API deliberately returns 1x0 for no event.
        % Normalize the ORACLE event vector; never relax the ID or ID-order
        % comparison. In particular, an existing singleton must give no event.
        fresh=reshape(ids(~ismember(ids,known)),1,[]);
        known=[known fresh];
        r=memory.update(f);
        actual_newly=reshape(double(r.newly_acquired_ids),1,[]);
        if isempty(fresh)
            R.empty_new_event_cases=R.empty_new_event_cases+1;
        end
        if isscalar(ids) && ismember(ids,old)
            R.known_singleton_cases=R.known_singleton_cases+1;
        end
        parity=[isequal(actual_newly,fresh), ...
               isequal(r.known_obstacle_ids,known), ...
               isequal(r.currently_detected_ids,ids), ...
               isequal(r.held_outside_current_detection_ids, ...
                   known(~ismember(known,ids))), ...
               isequal(r.memory_version,uint64(numel(known))), ...
               isequal(r.geometry_version,uint64(0)), ...
               isequal(memory.known_ids,known)];
        if ~all(parity)
            labels={'newly_acquired_ids','known_obstacle_ids', ...
                'currently_detected_ids','held_outside_current_detection_ids', ...
                'memory_version','geometry_version','memory.known_ids'};
            ix=find(~parity);
            error('CHECK_TRIGGER_MEMORY:HistoryParity', ...
                ['tick=%d mismatch=%s | ids=%s | known_expected=%s | ' ...
                'known_actual=%s | newly_expected=%s | newly_actual=%s | ' ...
                'newly_actual_size=%s'], ...
                k,strjoin(labels(ix),','),mat2str(ids),mat2str(known), ...
                mat2str(r.known_obstacle_ids),mat2str(fresh), ...
                mat2str(actual_newly),mat2str(size(r.newly_acquired_ids)));
        end
        assert(numel(fresh)==numel(setdiff(known,old)), ...
            'CHECK_TRIGGER_MEMORY:NewCount');
        R.new_observations=R.new_observations+numel(fresh);
        R.retained_checks=R.retained_checks+1;
        if mod(k,7)==0
            % Same physical observation twice cannot create a new event.
            rep=memory.update(f);
            assert(isempty(rep.newly_acquired_ids) && ...
                   isequal(rep.known_obstacle_ids,known), ...
                   'CHECK_TRIGGER_MEMORY:DuplicateUpdate');
            R.repeat_frame_checked=true;
        end
        R.cases=R.cases+1;
    end
    % Same IDs are not a loophole: altered per-frame obstacle geometry ID
    % must still be rejected, even though unique() may be skipped.
    ids=possible(1:2);
    obs=repmat(struct('id',ids(1),'motion','static', ...
        'position',zeros(3,1)),1,2);
    obs(2).id=ids(2);
    p=[0;0;0];
    valid=struct('time_s',40,'uav_position_ekf_m',p, ...
        'detected_ids',ids,'detected_count',2, ...
        'detected_obstacles',obs);
    memory.update(valid);R.cases=R.cases+1;
    invalid=valid;invalid.time_s=40.025;
    invalid.detected_obstacles(2).id=ids(1);
    rejected=false;
    try
        memory.update(invalid);
    catch ME
        rejected=strcmp(ME.identifier,'OBSERVED_OBSTACLE_MEMORY:SensorIDs');
    end
    assert(rejected,'CHECK_TRIGGER_MEMORY:MismatchNotRejected');
    R.invalid_inputs_rejected=R.invalid_inputs_rejected+1;
    R.cases=R.cases+1;
    invalid=valid;invalid.time_s=40.025;
    invalid.detected_ids=[ids(1),ids(1)];
    invalid.detected_obstacles(2).id=ids(1);
    rejected=false;
    try
        memory.update(invalid);
    catch ME
        rejected=strcmp(ME.identifier,'OBSERVED_OBSTACLE_MEMORY:SensorIDs');
    end
    assert(rejected,'CHECK_TRIGGER_MEMORY:DuplicateNotRejected');
    R.invalid_inputs_rejected=R.invalid_inputs_rejected+1;
    R.cases=R.cases+1;
    assert(R.empty_new_event_cases>0 && R.known_singleton_cases>0, ...
        'CHECK_TRIGGER_MEMORY:OracleCoverage', ...
        'The empty-event and already-known singleton cases were not exercised.');
    R.status='PASS_MEMORY_EVENT_ORACLE_PARITY__NOT_WCET';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_MEMORY_INDUCTIVE_FASTPATH_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_MEMORY_INDUCTIVE_FASTPATH_RESULT.txt'),'w');
if f>=0
    fprintf(f,['status=%s\ncases=%d\nfailures=%d\n' ...
        'new_observations=%d\nretained_checks=%d\n' ...
        'invalid_inputs_rejected=%d\n' ...
        'empty_new_event_cases=%d\nknown_singleton_cases=%d\n' ...
        'reason=%s\n'], ...
        R.status,R.cases,R.failures,R.new_observations, ...
        R.retained_checks,R.invalid_inputs_rejected, ...
        R.empty_new_event_cases,R.known_singleton_cases,R.reason);
    fclose(f);
end
end
