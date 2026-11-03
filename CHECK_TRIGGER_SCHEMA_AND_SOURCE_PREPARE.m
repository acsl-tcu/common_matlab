function R=CHECK_TRIGGER_SCHEMA_AND_SOURCE_PREPARE()
% Disposable test at common_matlab root. No sensor truth / live EKF.
R=struct('status','NOT_RUN','cases',0,'reason','');
try
    T=FUTURE_COLLISION_TRIGGER(struct('horizon_s',4));
    e=struct('p',zeros(3,1),'q',zeros(3,1),'v',zeros(3,1), ...
        'w',zeros(3,1),'pL',[0;0;-2],'vL',zeros(3,1), ...
        'pT',[0;0;-1],'wL',zeros(3,1));
    obs=OBSTACLE_INFORMATION.define(90001,'sphere',[100;0;0],1);
    f=struct('time_s',0,'uav_position_ekf_m',e.p, ...
        'detected_obstacles',obs,'detected_ids',90001,'detected_count',1);
    run=struct('sample_dt_s',0.025,'r_uav_m',0.2, ...
        'r_payload_m',0.1,'r_cable_m',0.01, ...
        'cable_length_m',2,'horizon_s',4);
    a=struct('runtime',run,'payload_mass_kg',1, ...
        'reference_at',@(t)zeros(28,1),'predictor',@local_fixture, ...
        'numeric_tolerances',[1e-10;1e-8]);
    one=T.evaluate(0,e,f,a);
    assert(one.prediction_complete && ~one.safe_to_commit && ...
        one.current_payload_mass_kg==1 && one.time_s==0 && ...
        one.sample_count>1);
    R.cases=R.cases+1;
    a.payload_mass_kg=2;f.time_s=0.025;
    two=T.evaluate(0.025,e,f,a);
    assert(two.prediction_complete && ~two.safe_to_commit && ...
        two.current_payload_mass_kg==2 && two.time_s==0.025 && ...
        isequal(fieldnames(one),fieldnames(two)));
    R.cases=R.cases+1;
    % This is only a software fixture. No actual reference source is used.
    q=TRIGGER_TIMEONLY_SOURCE_PREPARE(@(t)[t;0;0;0], ...
        struct('eligible',true));
    assert(q.calls==8 && ~q.stored_reference_values && ...
        ~q.used_flight_epoch && ~q.safe_to_commit);
    R.cases=R.cases+1;
    q=TRIGGER_TIMEONLY_SOURCE_PREPARE(@(t)[t;0;0;0], ...
        struct('eligible',false));
    assert(q.calls==0 && strcmp(q.status,'SKIPPED_SOURCE_NOT_AUDITED_PURE'));
    R.cases=R.cases+1;
    R.status='PASS_TEMPLATE_RESET_AND_SOURCE_ONLY_PREPARE__NOT_SAFETY';
catch ME
    R.status='FAIL_TEMPLATE_OR_SOURCE_PREPARE';
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_SCHEMA_SOURCE_RESULT.mat','R');
fid=fopen('TRIGGER_SCHEMA_SOURCE_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ncases=%d\nreason=%s\n',R.status,R.cases,R.reason);
    fclose(fid);
end
end
function P=local_fixture(req)
n=numel(req.sample_times_s);
P=struct('sample_times_s',req.sample_times_s, ...
    'uav_m',repmat(req.ekf_now.p,1,n), ...
    'payload_m',repmat(req.ekf_now.pL,1,n), ...
    'model_description','DISPOSABLE_NUMERIC_FIXTURE');
end
