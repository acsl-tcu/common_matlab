function R=CHECK_TRIGGER_SHADOW_STAGING_FIXTURE()
% All data are isolated synthetic test shapes. No live Flight, world or EKF.
R=struct('status','NOT_RUN_SHADOW_STAGING_FIXTURE', ...
 'first_generation_prepared',false,'new_generation_prepared',false, ...
 'no_flight_evaluation',false,'steps',0,'reason','');
try
    o1=OBSTACLE_INFORMATION.define(81001,'sphere',[0;0;0],1);
    o2=OBSTACLE_INFORMATION.define(81002,'cylinder',[2;0;0],[1 2]);
    o3=OBSTACLE_INFORMATION.define(81003,'box',[15;0;0],[2 2 2]);
    sensor=SIMULATED_OBSTACLE_SENSOR([o1 o2 o3],6,false);
    mem=OBSERVED_OBSTACLE_MEMORY(sensor.empty_observed_obstacle_schema());
    prepared=sensor.observed_prepared_only(zeros(1,0));
    ids=zeros(1,0);
    core=FUTURE_COLLISION_TRIGGER(struct('horizon_s',4));
    for k=1:8
        if k<=3,p=[0;0;0];else,p=[10;0;0];end
        t=(k-1)*0.025;
        frame=sensor.observe(t,p);
        M=mem.update(frame);
        newids=reshape(double(M.newly_acquired_ids),1,[]);
        if ~isempty(newids)
            fresh=sensor.observed_prepared_only(newids);
            prepared=OBSERVED_OBSTACLE_MEMORY.append_row_structs(prepared,fresh);
            ids=[ids newids]; %#ok<AGROW> synthetic fixture only
        end
        slim=mem.known_frame_for_trigger(frame);
        assert(isequal(reshape(double(slim.detected_ids),1,[]),ids), ...
          'CHECK_TRIGGER_SHADOW_STAGING_FIXTURE:Memory', ...
          'Synthetic memory sequence incorrect.');
        slim.prepared_sensor_shapes=prepared;
        core.prepare_shadow_observed_static(slim,t,p);
        R.steps=k;
        G=core.shadow_observation_preparation_status();
        if k==3
            R.first_generation_prepared= ...
                isequal(G.verified_ids,[81001 81002]) && ...
                isequal(G.outer_ids,[81001 81002]);
        end
    end
    G=core.shadow_observation_preparation_status();
    R.new_generation_prepared= ...
        isequal(G.verified_ids,[81001 81002 81003]) && ...
        isequal(G.outer_ids,[81001 81002 81003]);
    R.no_flight_evaluation=(core.evaluation_count==0);
    assert(R.first_generation_prepared && ...
        R.new_generation_prepared && R.no_flight_evaluation && ...
        G.invalid_observations==0, ...
        'CHECK_TRIGGER_SHADOW_STAGING_FIXTURE:Staging', ...
        ['Verified-generation cache was not prepared, or the synthetic ' ...
         'test accidentally evaluated a Flight trigger.']);
    R.status='PASS_SYNTHETIC_STAGED_GENERATIONS__NO_FLIGHT_SAFETY';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
f=fopen('TRIGGER_SHADOW_STAGING_FIXTURE_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\nsteps=%d\nfirst_generation=%d\nnew_generation=%d\nno_flight_evaluation=%d\nreason=%s\n', ...
        R.status,R.steps,R.first_generation_prepared, ...
        R.new_generation_prepared,R.no_flight_evaluation,R.reason);
    fclose(f);
end
end
