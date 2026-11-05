function R=CHECK_SIM_SENSOR_TRUE_CENTER_AND_EPOCH()
% Sim-only sensor regression: truth center and trigger EKF must remain distinct.
R=struct('status','FAIL_TRUE_CENTER_OR_EPOCH','cases',0,'failures',0, ...
    'different_center_cases',0,'epoch_cases',0,'reason','');
try
    dt=0.025;
    S=struct();
    [p,S]=SIM_SENSOR_TICK_ALIGNED_ORIGIN(S,0,'a',[0;0;0],dt);
    assert(isequal(p,[0;0;0])); R.cases=R.cases+1;
    [p,S]=SIM_SENSOR_TICK_ALIGNED_ORIGIN(S,0,'t',[1;2;3],dt);
    assert(isequal(p,[0;0;0])); R.cases=R.cases+1;
    [p,S]=SIM_SENSOR_TICK_ALIGNED_ORIGIN(S,dt,'t',[4;5;6],dt);
    assert(isequal(p,[1;2;3])); R.cases=R.cases+1;
    [p,S]=SIM_SENSOR_TICK_ALIGNED_ORIGIN(S,2*dt,'t',[7;8;9],dt);
    assert(isequal(p,[4;5;6])); R.cases=R.cases+1;
    R.epoch_cases=R.cases;
    mustReject(@() SIM_SENSOR_TICK_ALIGNED_ORIGIN(struct(),0,'f',[0;0;0],dt));
    R.cases=R.cases+1;
    mustReject(@() SIM_SENSOR_TICK_ALIGNED_ORIGIN(S,0.25,'f',[0;0;0],dt));
    R.cases=R.cases+1;
    mustReject(@() SIM_SENSOR_TICK_ALIGNED_ORIGIN(S,2*dt,'f',[0;0;0],dt));
    R.cases=R.cases+1;
    mustReject(@() SIM_SENSOR_TICK_ALIGNED_ORIGIN(S,3*dt,'f',[NaN;0;0],dt));
    R.cases=R.cases+1;

    obstacles=OBSTACLE_INFORMATION.definitions();
    assert(~isempty(obstacles),'No defined obstacles.');
    Sen=SIMULATED_OBSTACLE_SENSOR(obstacles,20,false);
    c=double(obstacles(1).initial_position(:));
    far=c+[1000;0;0];
    a=Sen.observe(0,c,far);
    assert(any(double(a.detected_ids)==double(obstacles(1).id)) && ...
        isequal(a.uav_position_true_m,c)&& ...
        isequal(a.uav_position_ekf_m,far)&& ...
        strcmp(a.sensor_center_source,'SIM_TRUE_TIME_ALIGNED'), ...
        'Detected obstacle must be selected by TRUE position, not EKF.');
    R.cases=R.cases+1;R.different_center_cases=R.different_center_cases+1;
    b=Sen.observe(dt,far,c);
    assert(~any(double(b.detected_ids)==double(obstacles(1).id)) && ...
        isequal(b.uav_position_true_m,far) && ...
        isequal(b.uav_position_ekf_m,c), ...
        'Distant TRUE position must not leak EKF proximity into detection.');
    R.cases=R.cases+1;R.different_center_cases=R.different_center_cases+1;
    main=fileread(which('REPLANNING_SYSTEM'));
    assert(contains(main,'SIM_SENSOR_READ_TRUE_POSITION(agent)') && ...
        contains(main,'SIM_SENSOR_TICK_ALIGNED_ORIGIN(') && ...
        contains(main,'obj.observe_sensor(time_s,p_true,p_ekf)') && ...
        contains(main,'REPLANNING_SYSTEM.require_ekf_uav_position(agent)'), ...
        'REPLANNING_SYSTEM true sensor / EKF trigger source wiring missing.');
    R.cases=R.cases+1;
    R.status='PASS_SENSOR_TRUE_CENTER_EKF_TRIGGER_SEPARATED__SIM_ONLY';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    save('TRIGGER_SIM_SENSOR_TRUE_CENTER_RESULT.mat','R');
    fid=fopen('TRIGGER_SIM_SENSOR_TRUE_CENTER_RESULT.txt','w');
    if fid>0
        fprintf(fid,'%s\ncases=%d failures=%d\n%s\n',R.status,R.cases,R.failures,R.reason);
        fclose(fid);
    end
catch
end
end
function mustReject(f)
failed=false;
try
    f();
catch
    failed=true;
end
assert(failed,'CHECK_SIM_SENSOR_TRUE_CENTER_AND_EPOCH:ExpectedError', ...
    'Missing physical Sim truth, inconsistent epoch, or illegal initial phase must fail closed.');
end
