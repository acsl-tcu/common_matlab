function R=CHECK_TRIGGER_PREPARED_SENSOR_HISTORY()
% Disposable synthetic-only regression. NO live Flight or hidden obstacles.
% The sensor must never reveal a historically known but CURRENTLY unseen ID.
R=struct('status','NOT_COMPLETED_PREPARED_SENSOR_HISTORY', ...
    'reason','','steps',0,'fresh_ids',{{}},'cumulative_ids',{{}}, ...
    'unseen_api_rejected',false,'safe_to_commit',false);
try
    a=OBSTACLE_INFORMATION.define(82001,'sphere',[-0.5;0;0],0.15);
    b=OBSTACLE_INFORMATION.define(82002,'cylinder',[0.5;0;0],[0.15 0.4]);
    c=OBSTACLE_INFORMATION.define(82003,'box',[1.5;0;0],[0.3 0.3 0.3]);
    fake=[a b c];
    sensor=SIMULATED_OBSTACLE_SENSOR(fake,0.8,false);
    memory=OBSERVED_OBSTACLE_MEMORY(fake([]));
    ledger=sensor.observed_prepared_only(zeros(1,0));
    ledger_ids=zeros(1,0);
    positions=[0 1 1 0 1 0 1 1];
    for k=1:numel(positions)
        p=[positions(k);0;0];
        f=sensor.observe(0.025*(k-1),p);
        mr=memory.update(f);
        frame=memory.known_frame_for_trigger(f);
        % Re-detection is NOT a new acquisition. The already observed
        % geometry must be retained, not appended twice.
        fresh=reshape(double(mr.newly_acquired_ids),1,[]);
        if ~isempty(fresh)
            new_shapes=sensor.observed_prepared_only(fresh);
            assert(isequal(reshape(double([new_shapes.id]),1,[]),fresh), ...
                'CHECK_TRIGGER_PREPARED_SENSOR_HISTORY:NewShapeIds');
            ledger=[reshape(ledger,1,[]) reshape(new_shapes,1,[])]; %#ok<AGROW>
            ledger_ids=[ledger_ids fresh]; %#ok<AGROW>
        end
        ids=reshape(double(frame.detected_ids),1,[]);
        assert(isequal(ids,reshape(double(mr.known_obstacle_ids),1,[])), ...
            'CHECK_TRIGGER_PREPARED_SENSOR_HISTORY:MemoryFrameMismatch');
        assert(isequal(ledger_ids,ids) && ...
            isequal(reshape(double([ledger.id]),1,[]),ids), ...
            'CHECK_TRIGGER_PREPARED_SENSOR_HISTORY:LedgerMismatch', ...
            'Cumulative geometry must contain all and ONLY acquired IDs.');
        R.fresh_ids{end+1}=fresh;
        R.cumulative_ids{end+1}=ids;
        if k==2
            assert(any(ids==82001) && ...
                ~any(double(f.detected_ids(:))==82001), ...
                'CHECK_TRIGGER_PREPARED_SENSOR_HISTORY:Fixture', ...
                'At tick two, the first obstacle must be known but out of range.');
            try
                sensor.observed_prepared_only(ids); %#ok<VUNUS>
            catch ME
                R.unseen_api_rejected=strcmp(ME.identifier, ...
                    'SIMULATED_OBSTACLE_SENSOR:UnseenGeometry');
            end
            assert(R.unseen_api_rejected, ...
                'CHECK_TRIGGER_PREPARED_SENSOR_HISTORY:Guard', ...
                'Current-scan sensor disclosure guard must remain intact.');
        end
        R.steps=k;
    end
    % Verify that the earlier-sensed shape returning to range at tick four
    % was NOT published as a new memory acquisition.
    assert(isempty(R.fresh_ids{4}) && isempty(R.fresh_ids{6}), ...
        'CHECK_TRIGGER_PREPARED_SENSOR_HISTORY:RedetectionDuplicate');
    assert(R.steps==8 && R.unseen_api_rejected && numel(ledger_ids)==3);
    R.status='PASS_SENSOR_ACQUIRED_HISTORY_ONLY__NO_JIT_OR_SAFETY_PROOF';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_PREPARED_SENSOR_HISTORY_RESULT.mat','R');
fid=fopen('TRIGGER_PREPARED_SENSOR_HISTORY_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nsteps=%d\nunseen_api_rejected=%d\nreason=%s\n', ...
        R.status,R.steps,R.unseen_api_rejected,R.reason);
    fclose(fid);
end
end
