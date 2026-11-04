function P=TRIGGER_F_PHASE_KERNEL_PREPARE(static_order,source,horizon_s,tolerances)
% Discarded pre-arm exercise of the EXACT f-phase runtime inner kernel.
% Uses actual TIME/DRONE/STATE_CLASS types and ACTUAL model source only.
% All states, observed shapes, clocks, reference and events are synthetic;
% no real world, sensor, EKF or Flight epoch is read. Entire synthetic
% cache/trigger is discarded. This is JIT-path preparation, NOT proof.
P=struct('status','NOT_COMPLETED_F_PHASE_KERNEL_PREPARE', ...
    'calls',0,'native_complete_calls',0,'far_clear_calls',0, ...
    'shadow_stage_calls',0,'typed_nonempty_appends',0,'reason','', ...
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
    fixture_positions=[0 1 1 0 1 0 1 1];
    for k=1:8
        time.t=dt*(k-1);
        position=[fixture_positions(k);0;0];
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
        mr=memory.update(f);
        frame=memory.known_frame_for_trigger(f);
        % Sensor's newly_detected_ids means a TRANSITION into range.
        % A previously acquired ID may re-enter the sensor and MUST NOT
        % be appended again to the cumulative prepared-geometry ledger.
        % Only the MEMORY's first-acquisition IDs are new to that ledger.
        fresh_ids=reshape(double(mr.newly_acquired_ids),1,[]);
        if ~isempty(fresh_ids)
            % Only same-scan newly detected IDs can pass this sensor API.
            newly_prepared=sensor.observed_prepared_only(fresh_ids);
            assert(isequal(reshape(double([newly_prepared.id]),1,[]),fresh_ids), ...
                'TRIGGER_F_PHASE_KERNEL_PREPARE:FreshGeometry', ...
                'The real sensor API must reveal exactly the newly detected IDs.');
            % Production uses THIS append method, not plain [] concatenation.
            historical_shapes=OBSERVED_OBSTACLE_MEMORY.append_row_structs( ...
                historical_shapes,newly_prepared);
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
    % IMPORTANT: The previous 8-call fixture always began in contact.
    % That did NOT execute the broadphase-all-excluded / no-contact path
    % encountered on a genuine first Flight prediction. Exercise that
    % SAME kernel with DIFFERENT wholly fictitious, already observed solids.
    % This is separate disposable history: it never changes the real sensor,
    % planner, class parameter, obstacle count, Flight epoch or detection set.
    far1=OBSTACLE_INFORMATION.define(83001,'sphere',[1000;0;0],0.2);
    far2=OBSTACLE_INFORMATION.define(83002,'cylinder',[1005;0;0],[0.2 0.5]);
    far3=OBSTACLE_INFORMATION.define(83003,'box',[1010;0;0],[0.5 0.5 0.5]);
    far_sensor=SIMULATED_OBSTACLE_SENSOR([far1 far2 far3],1500,false);
    far_memory=OBSERVED_OBSTACLE_MEMORY( ...
        far_sensor.empty_observed_obstacle_schema());
    far_core=FUTURE_COLLISION_TRIGGER(struct('horizon_s',double(horizon_s)));
    far_core.prepare_static_timing(dt,double(horizon_s));
    far_cache=TRIGGER_REFERENCE_WINDOW_CACHE();
    far_audit=far_cache.audit_immutable_source(nominal.func);
    assert(far_audit.eligible,'TRIGGER_F_PHASE_KERNEL_PREPARE:FarReferenceAudit');
    far_shapes=far_sensor.observed_prepared_only(zeros(1,0));
    far_ids=zeros(1,0);
    for replay=1:8
        time.t=dt*(replay-1);
        position=zeros(3,1);
        state=agent.estimator.result.state;
        state.p=position;
        state.pL=position+[0;0;-L];
        state.pT=[0;0;-1];
        state.mL=1;
        agent.reference.result=nominal.result;
        agent.reference.result.state.xd= ...
            REPLANNING_SYSTEM.flight_reference_from_nominal(nominal,time.t,L);
        sensed=far_sensor.observe(time.t,position);
        learned=far_memory.update(sensed);
        known=far_memory.known_frame_for_trigger(sensed);
        first_ids=reshape(double(learned.newly_acquired_ids),1,[]);
        if ~isempty(first_ids)
            new_shapes=far_sensor.observed_prepared_only(first_ids);
            far_shapes=OBSERVED_OBSTACLE_MEMORY.append_row_structs( ...
                far_shapes,new_shapes);
            far_ids=[far_ids first_ids]; %#ok<AGROW>
        end
        assert(isequal(far_ids,reshape(double(known.detected_ids),1,[])), ...
            'TRIGGER_F_PHASE_KERNEL_PREPARE:FarObservation');
        known.prepared_sensor_shapes=far_shapes;
        if replay==1
            % Exercise actual staged-cache geometry verification. It
            % processes ONE observed solid per call; no unseen world truth.
            for stage=1:numel(far_ids)
                far_core.prepare_shadow_observed_static( ...
                    known,time.t,position);
                P.shadow_stage_calls=P.shadow_stage_calls+1;
            end
        end
        [clear_result,~]=TRIGGER_F_PHASE_RUNTIME_KERNEL( ...
            time,agent,known,binding,far_cache,far_core, ...
            double(tolerances(1)),double(tolerances(2)),false);
        assert(isstruct(clear_result) && ...
            clear_result.prediction_complete && ...
            clear_result.prediction_available && ...
            ~clear_result.sampled_collision && ...
            ~clear_result.safe_to_commit, ...
            'TRIGGER_F_PHASE_KERNEL_PREPARE:FarClear', ...
            'The same native Flight kernel did not complete a far-field non-contact case.');
        P.far_clear_calls=P.far_clear_calls+1;
        P.native_complete_calls=P.native_complete_calls+1;
    end
    % Exercise actual production nonempty-struct append repeatedly, using
    % distinct disposable local values. Not a detection-mask table.
    for replay=1:8
        first_shapes=far_sensor.observed_prepared_only(far_ids(1));
        more_shapes=far_sensor.observed_prepared_only(far_ids(2:end));
        combined=OBSERVED_OBSTACLE_MEMORY.append_row_structs( ...
            first_shapes,more_shapes);
        assert(isequal(reshape(double([combined.id]),1,[]),far_ids), ...
            'TRIGGER_F_PHASE_KERNEL_PREPARE:TypedAppend');
        P.typed_nonempty_appends=P.typed_nonempty_appends+1;
    end
    assert(P.far_clear_calls==8 && P.typed_nonempty_appends==8 && ...
        P.native_complete_calls==16 && P.shadow_stage_calls==3, ...
        'TRIGGER_F_PHASE_KERNEL_PREPARE:Coverage');
    P.status='PASS_DISCARDED_FLIGHT_KERNEL_CONTACT_8_CLEAR_8__NOT_JIT_OR_SAFETY';
catch ME
    P.reason=[ME.identifier ' | ' ME.message];
end
end
