function P=TRIGGER_F_PHASE_KERNEL_PREPARE(static_order,source,horizon_s,tolerances)
% Discarded pre-arm exercise of the EXACT f-phase runtime inner kernel.
% Uses actual TIME/DRONE/STATE_CLASS types and ACTUAL model source only.
% All states, observed shapes, clocks, reference and events are synthetic;
% no real world, sensor, EKF or Flight epoch is read. Entire synthetic
% cache/trigger is discarded. This is JIT-path preparation, NOT proof.
P=struct('status','NOT_COMPLETED_F_PHASE_KERNEL_PREPARE', ...
    'calls',0,'native_complete_calls',0,'reason','', ...
    'used_live_epoch',false,'used_live_obstacles',false, ...
    'published_events',false,'safe_to_commit',false);
try
    [ready,F]=TRIGGER_ISOLATED_MCOSTYPE_PREPARE(static_order,source);
    assert(ready.calls==8 && ready.same_mcos_classes && ...
        isstruct(F) && isfield(F,'agent'), ...
        'TRIGGER_F_PHASE_KERNEL_PREPARE:MCOS', ...
        'Disposable source-identified MCOS path must first be executable.');
    assert(numel(tolerances)==2 && all(isfinite(tolerances)) && ...
        all(tolerances>0), ...
        'TRIGGER_F_PHASE_KERNEL_PREPARE:Tolerances');
    dt=double(F.sample_dt_s);L=double(F.cable_length_m);
    agent=F.agent;time=F.time;binding=F.authority;
    nominal=TIME_VARYING_REFERENCE( ...
        struct('estimator',struct('result',struct('state',struct()))), ...
        {'TRIGGER_SOFTWARE_FIXTURE_REFERENCE_FACTORY',{}});
    nominal.t=0; % SYNTHETIC, never the real Flight reference epoch.
    agent.reference.set_function_class('timevarying',nominal);
    agent.cha='f'; % local fixture ONLY; no mutation to real agent.
    fake1=OBSTACLE_INFORMATION.define(82001,'sphere',[-0.5;0;0],0.15);
    fake2=OBSTACLE_INFORMATION.define(82002,'cylinder',[0.5;0;0],[0.15 0.4]);
    fake3=OBSTACLE_INFORMATION.define(82003,'box',[1.5;0;0],[0.3 0.3 0.3]);
    fake=[fake1 fake2 fake3];
    sensor=SIMULATED_OBSTACLE_SENSOR(fake,0.8,false);
    memory=OBSERVED_OBSTACLE_MEMORY(fake([]));
    core=FUTURE_COLLISION_TRIGGER(struct('horizon_s',double(horizon_s)));
    core.prepare_static_timing(dt,double(horizon_s));
    cache=TRIGGER_REFERENCE_WINDOW_CACHE();
    audit=cache.audit_immutable_source(nominal.func);
    assert(audit.eligible,'TRIGGER_F_PHASE_KERNEL_PREPARE:ReferenceAudit');
    % Prepared geometry is exposed only at the moment a REAL synthetic
    % scan first detects each ID. The memory's known IDs are cumulative:
    % requesting every known ID from observed_prepared_only on each tick
    % would unlawfully ask the sensor to reveal an out-of-range shape.
    % Preserve the genuine sensor history and retain already-revealed
    % prepared shapes, exactly like the real replanning system.
    historical_shapes=sensor.observed_prepared_only(zeros(1,0));
    historical_ids=zeros(1,0);
    for k=1:8
        time.t=dt*(k-1);
        if k==1,position=[0;0;0];else,position=[1;0;0];end
        st=agent.estimator.result.state;
        st.p=position;
        st.pL=position+[0;0;-L];
        st.pT=[0;0;-1];
        st.mL=1;
        agent.reference.result=nominal.result;
        agent.reference.result.state.xd= ...
            REPLANNING_SYSTEM.flight_reference_from_nominal( ...
                nominal,time.t,L);
        f=sensor.observe(time.t,position);
        memory.update(f);
        frame=memory.known_frame_for_trigger(f);
        fresh_ids=reshape(double(f.newly_detected_ids),1,[]);
        if ~isempty(fresh_ids)
            % Only same-scan newly detected IDs can pass this sensor API.
            newly_prepared=sensor.observed_prepared_only(fresh_ids);
            assert(isequal(reshape(double([newly_prepared.id]),1,[]),fresh_ids), ...
                'TRIGGER_F_PHASE_KERNEL_PREPARE:FreshGeometry', ...
                'The real sensor API must reveal exactly the newly detected IDs.');
            historical_shapes=[reshape(historical_shapes,1,[]) ...
                reshape(newly_prepared,1,[])]; %#ok<AGROW>
            historical_ids=[historical_ids fresh_ids]; %#ok<AGROW>
        end
        assert(isequal(historical_ids,reshape(double(frame.detected_ids),1,[])) && ...
            numel(historical_shapes)==numel(historical_ids), ...
            'TRIGGER_F_PHASE_KERNEL_PREPARE:HistoricalIds', ...
            'The prepared synthetic geometry must match the actual memory ledger.');
        frame.prepared_sensor_shapes=historical_shapes;
        [r,~]=TRIGGER_F_PHASE_RUNTIME_KERNEL( ...
            time,agent,frame,binding,cache,core, ...
            double(tolerances(1)),double(tolerances(2)),false);
        assert(isstruct(r)&&isfield(r,'safe_to_commit') && ...
            ~r.safe_to_commit && r.prediction_available && ...
            r.prediction_complete && ...
            strcmp(r.predictor_dispatch,'DIRECT_NATIVE_VERIFIED_BINDING'), ...
            'TRIGGER_F_PHASE_KERNEL_PREPARE:InnerPath', ...
            'The actual Flight inner native predictor path did not complete.');
        P.calls=k;
        P.native_complete_calls=P.native_complete_calls+1;
    end
    P.status='PASS_DISCARDED_SAME_FLIGHT_INNER_KERNEL_8__NOT_JIT_OR_SAFETY';
catch ME
    P.reason=[ME.identifier ' | ' ME.message];
end
end
