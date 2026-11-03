function T=CHECK_TRIGGER_LEAN_MEMORY_FRAME()
% Synthetic-only semantic parity: old/full vs new/thin known-frame adapters.
% Does NOT use obstacle definitions, actual sensor, EKF or Flight states.
T=struct('status','FAIL_LEAN_MEMORY_ADAPTER_PARITY', ...
    'pass',false,'cases',0,'reason','');
try
    s1=OBSTACLE_INFORMATION.define(11,'sphere',[50;0;0],1.0);
    s2=OBSTACLE_INFORMATION.define(22,'ellipsoid',[70;0;0],[2;1;3]);
    s3=OBSTACLE_INFORMATION.define(45,'box',[80;0;0],[2;3;4]);
    shapes=[s1 s2 s3];
    mem=OBSERVED_OBSTACLE_MEMORY(shapes([]));
    p=zeros(3,1);
    seq={[],[11],[11 22],[22],[22 45],[],[11 45],[11 22 45]};
    for k=1:numel(seq)
        ids=seq{k};
        mask=ismember([shapes.id],ids);
        records=shapes(mask);
        frame=struct('time_s',double(k-1)/40, ...
            'uav_position_ekf_m',p,'detected_ids',reshape(ids,1,[]), ...
            'detected_count',numel(ids),'detected_obstacles',records, ...
            'detected_distances_m',100+zeros(1,numel(ids)));
        mem.update(frame);
        old=mem.known_frame(frame);
        thin=mem.known_frame_for_trigger(frame);
        keys={'time_s','uav_position_ekf_m','detected_obstacles', ...
            'detected_ids','detected_count','observation_provenance', ...
            'memory_version','geometry_version'};
        for i=1:numel(keys)
            key=keys{i};
            assert(isequaln(old.(key),thin.(key)), ...
                'CHECK_TRIGGER_LEAN_MEMORY_FRAME:Mismatch', ...
                'Original and thin trigger input differ on %s at tick %d.',key,k);
        end
        assert(~isfield(thin,'detected_distances_m') && ...
            ~isfield(thin,'retained_only_ids'), ...
            'CHECK_TRIGGER_LEAN_MEMORY_FRAME:Extra', ...
            'Trigger thin frame contains extraneous scan metadata.');
        assert(isequal(reshape([thin.detected_obstacles.id],1,[]), ...
            reshape(thin.detected_ids,1,[])));
        T.cases=T.cases+1;
    end
    T.pass=true;
    T.status='PASS_LEAN_MEMORY_FRAME_PARITY__NOT_FLIGHT_SAFETY';
catch ME
    T.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_LEAN_MEMORY_FRAME_RESULT.mat','T');
fid=fopen('TRIGGER_LEAN_MEMORY_FRAME_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ncases=%d\nreason=%s\n', ...
        T.status,T.cases,T.reason);fclose(fid);
end
end
