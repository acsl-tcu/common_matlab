function OUT = af7r58zzt_process_worker(op,payload)
% AF7R58ZZT process-local planning worker.
% One worker process is used intentionally.  The worker holds a persistent
% copy of REPLANNING_SYSTEM so AF7R36 MEX/JIT state remains process-local.
%
% INIT serializes the already-primed main object once before motion.
% RUN receives only an immutable request/authority snapshot.

persistent WOBJ

op=upper(char(op));
switch op
    case 'INIT'
        assert(isa(payload,'REPLANNING_SYSTEM'), ...
            '[AF7R58ZZT-WORKER] INIT payload must be REPLANNING_SYSTEM.');
        WOBJ=payload;

        % Client-side parallel handles are Transient and arrive empty.
        % Rebind the selected callback to THIS worker-local object.
        WOBJ.generator_backend_locked_callback=[];
        WOBJ.generator_backend_configuration_frozen=false;
        WOBJ.freeze_generator_backend_configuration();
        af7r36_runtime_prime(WOBJ);

        OUT=struct( ...
            'ok',true, ...
            'schema','AF7R58ZZT_WORKER_INIT_V1', ...
            'active_reference_mode',char(WOBJ.active_reference_mode), ...
            'active_reference_version',double(WOBJ.active_reference_version), ...
            'horizon_s',double(WOBJ.horizon_s), ...
            'dt',double(WOBJ.prediction_dt));

    case 'CALIBRATE'
        assert(~isempty(WOBJ) && isa(WOBJ,'REPLANNING_SYSTEM'), ...
            '[AF7R58ZZT-R2-WORKER] CALIBRATE before INIT.');
        assert(isstruct(payload) && isfield(payload,'schema') && ...
            strcmp(char(payload.schema),'AF7R58ZZT_R2_IPC_CALIBRATION_V1'), ...
            '[AF7R58ZZT-R2-WORKER] invalid CALIBRATE payload.');
        % Echo the full payload so the measured round trip includes a
        % conservative same-family serialization cost in both directions.
        OUT=payload;
        OUT.ok=true;
        OUT.schema='AF7R58ZZT_R2_IPC_CALIBRATION_ACK_V1';

    case 'RUN'
        assert(~isempty(WOBJ) && isa(WOBJ,'REPLANNING_SYSTEM'), ...
            '[AF7R58ZZT-WORKER] RUN before INIT.');
        assert(isstruct(payload) && ...
            isfield(payload,'schema') && ...
            strcmp(char(payload.schema),'AF7R58ZZT_REQUEST_V1'), ...
            '[AF7R58ZZT-WORKER] invalid RUN payload.');

        tw=tic;
        OUT=struct( ...
            'ok',false, ...
            'schema','AF7R58ZZT_COMMIT_READY_V1', ...
            'reason','OPEN');

        try
            apply_source_authority(WOBJ,payload.source_authority);

            assert(strcmp(char(WOBJ.active_reference_mode),char(payload.source_mode)) && ...
                double(WOBJ.active_reference_version)==double(payload.source_version), ...
                '[AF7R58ZZT-WORKER] source authority sync mismatch.');
            assert(isequaln(WOBJ.reference_only_worker_dependency_key(), ...
                payload.source_dependency_key), ...
                '[AF7R58ZZT-WORKER] source dependency sync mismatch.');
            assert(isequaln(WOBJ.static_obstacle_map, ...
                payload.static_obstacle_map_snapshot), ...
                '[AF7R58ZZT-WORKER] static obstacle map mismatch.');

            ta=double(payload.activation_time);
            dt=double(WOBJ.prediction_dt);
            assert(isfinite(ta)&&dt>0&&isequal(double(WOBJ.horizon_s),4.0), ...
                '[AF7R58ZZT-WORKER] fixed-H=4 s activation contract open.');

            % Exact active-reference packet at FUTURE activation t_a.
            N=round(double(WOBJ.horizon_s)/dt)+1;
            xd_nom=WOBJ.get_nominal_xd_from_table(ta);
            xd_active=WOBJ.evaluate_active_reference_xd(ta,xd_nom);

            if strcmpi(char(WOBJ.active_reference_mode),'NOMINAL')
                assert(WOBJ.nominal_table_valid && ...
                    abs(double(WOBJ.nominal_table_dt)-dt)<=1e-12, ...
                    '[AF7R58ZZT-WORKER] nominal table unavailable.');
                i0=round((ta-double(WOBJ.nominal_table_t0))/dt)+1;
                i1=i0+N-1;
                assert(i0>=1 && i1<=size(WOBJ.nominal_table_pL,2), ...
                    '[AF7R58ZZT-WORKER] activation packet outside nominal table.');
                pL_ref=WOBJ.nominal_table_pL(:,i0:i1);
                q_ref=WOBJ.nominal_table_q(:,i0:i1);
                ref_source='PROCESS_WORKER_NOMINAL_TABLE';
            else
                [pL_ref,q_ref,ref_source]=WOBJ.repaired_reference_packet(ta);
            end
            WOBJ.validate_reference_packet(pL_ref,q_ref,N);

            % W1 is a reference-level worker.  The native core/certificate
            % does not use a future measured plant state.  Supply the exact
            % active-reference geometry at the activation boundary.
            pL_cur=double(pL_ref(:,1));
            q_cur=double(q_ref(:,1));
            pQ_cur=pL_cur-double(WOBJ.L_cable)*q_cur;
            cable_residual=0;

            active=logical(payload.known_mask(:).');
            all_dists=double(payload.all_dists(:).');
            T=payload.trigger;
            ttc=max(0,double(T.collision_time)-ta);
            jhit=max(1,double(T.hit_index)-1);

            % Use the unchanged production generator + reference certificate
            % + canonical preparation + atomic commit on the worker-local copy.
            tlocal=tic;
            WOBJ.invoke_replan_generator_shadow( ...
                tlocal,ta,xd_nom,xd_active, ...
                pQ_cur,pL_cur,q_cur,cable_residual, ...
                active,all_dists,pL_ref,q_ref,ref_source, ...
                ttc,double(T.collision_time),double(T.obstacle_id), ...
                char(T.component),double(T.component_code), ...
                double(T.pL_collision(:)),double(T.q_collision(:)), ...
                double(T.pQ_collision(:)),jhit);

            assert(strcmpi(char(WOBJ.active_reference_mode),'REPAIRED') && ...
                double(WOBJ.active_reference_version)==double(payload.source_version)+1 && ...
                strcmpi(char(WOBJ.active_repaired_safety_level),'REFERENCE_ONLY'), ...
                '[AF7R58ZZT-WORKER] worker attempt did not close REFERENCE_ONLY commit.');

            R=WOBJ.active_repaired_reference;
            if isfield(R,'eval_fn'), R=rmfield(R,'eval_fn'); end
            if isfield(R,'flat_eval_fn'), R=rmfield(R,'flat_eval_fn'); end

            OUT.ok=true;
            OUT.reason='WORKER_REFERENCE_ONLY_COMMIT_READY';
            OUT.source_mode=char(payload.source_mode);
            OUT.source_version=double(payload.source_version);
            OUT.target_version=double(WOBJ.active_reference_version);
            OUT.activation_time=ta;
            OUT.source_dependency_key=payload.source_dependency_key;
            OUT.known_mask=active;
            OUT.static_obstacle_map_snapshot=payload.static_obstacle_map_snapshot;
            OUT.repaired_reference=R;
            OUT.repaired_certificates=WOBJ.active_repaired_certificates;
            OUT.speed=WOBJ.active_speed_certificate;
            OUT.sweep=WOBJ.trigger_sweep_inflation;
            OUT.packet=WOBJ.obstacle_packet;
            OUT.sensor=WOBJ.sensor_horizon_certificate;
            OUT.dep=WOBJ.runtime_dependency_signature;
            OUT.stale=WOBJ.certificate_stale;
            OUT.safety_level=char(WOBJ.active_repaired_safety_level);
            OUT.worker_elapsed_ms=1e3*toc(tw);
            OUT.callback_ms=double(WOBJ.generator_last_callback_ms);
            OUT.worker_status_code=double(WOBJ.generator_last_status_code);

        catch ME
            OUT.ok=false;
            OUT.reason=ME.message;
            OUT.error_identifier=ME.identifier;
            OUT.worker_elapsed_ms=1e3*toc(tw);
            try
                OUT.callback_ms=double(WOBJ.generator_last_callback_ms);
                OUT.worker_status_code=double(WOBJ.generator_last_status_code);
            catch
                OUT.callback_ms=NaN;
                OUT.worker_status_code=NaN;
            end
        end

    case 'RESET'
        WOBJ=[];
        OUT=struct('ok',true,'schema','AF7R58ZZT_WORKER_RESET_V1');

    otherwise
        error('[AF7R58ZZT-WORKER] unknown operation.');
end
end


function apply_source_authority(W,A)
assert(isstruct(A) && isfield(A,'schema') && ...
    strcmp(char(A.schema),'AF7R58ZZT_SOURCE_AUTHORITY_V1'), ...
    '[AF7R58ZZT-WORKER] invalid source authority.');

W.active_reference_mode=char(A.mode);
W.active_reference_version=double(A.version);
W.active_speed_certificate=A.speed;
W.trigger_sweep_inflation=A.sweep;
W.obstacle_packet=A.packet;
W.sensor_horizon_certificate=A.sensor;
W.runtime_dependency_signature=A.dep;
W.certificate_stale=A.stale;
W.active_repaired_safety_level=char(A.level);
W.last_replan_commit_time=double(A.last_commit_time);
W.static_obstacle_map=A.static_obstacle_map;
W.active_repaired_certificates=A.repaired_certificates;
W.repaired_reference_cache=struct();
W.repaired_reference_cache_valid=false;
W.replan_request_active=false;
W.replan_request_reference_version=-1;
W.replan_request_obstacle_id=NaN;
W.replan_request_component_code=NaN;

if strcmpi(char(A.mode),'REPAIRED')
    R=A.repaired_reference;
    assert(isstruct(R)&&isfield(R,'batch_flat_model'), ...
        '[AF7R58ZZT-WORKER] repaired source batch model missing.');
    [evalFn,flatEvalFn]=af7r58i_make_canonical_eval_handles(W,R.batch_flat_model);
    R.eval_fn=evalFn;
    R.flat_eval_fn=flatEvalFn;
    W.active_repaired_reference=R;
    W.assert_repaired_reference_current();
else
    W.active_repaired_reference=struct();
    W.active_repaired_safety_level='NONE';
    W.validate_active_speed_certificate(W.active_speed_certificate);
end
end
