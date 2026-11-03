function R=TRIGGER_ISOLATED_SENSOR_EVENT_PREPARE(cfg)
% PRE-ARM CODE-PATH EXERCISE ONLY -- NO real obstacle/world/EKF/flight data.
% Synthesizes three clearly fictional solids and an independently allocated
% sensor/memory. Nothing from these fixtures survives this function.
% It exercises 0 -> 1 -> 2 -> 1 -> 0 real-API sensor/memory/shape handoff,
% including the absent/retained IDs and nonempty append branches.
% Repeated execution does NOT prove MATLAB JIT compilation or a time bound.
R=struct('status','NOT_COMPLETED_ISOLATED_SENSOR_EVENTS', ...
    'pass',false,'cycles',0,'steps',0,'reason','', ...
    'ever_read_live_obstacles',false,'safe_to_commit',false, ...
    'phase','NOT_STARTED');
try
    assert(isstruct(cfg)&&isscalar(cfg) && ...
        isfield(cfg,'sample_dt_s') && isfinite(cfg.sample_dt_s) && ...
        cfg.sample_dt_s>0, ...
        'TRIGGER_ISOLATED_SENSOR_EVENT_PREPARE:Cfg', ...
        'Positive static sample dt required.');
    % These numbers are SYNTHETIC SOFTWARE FIXTURES, never physical defaults
    % or precomputed actual-world observations.
    o1=OBSTACLE_INFORMATION.define(70001,'sphere',[0;0;0],2);
    o2=OBSTACLE_INFORMATION.define(70002,'cylinder',[7;0;0],[1 3]);
    o3=OBSTACLE_INFORMATION.define(70003,'box',[15;0;0],[2 2 2]);
    fiction=[o1 o2 o3];
    visit=[-20 0 4 7 30];
    expected={zeros(1,0),70001,[70001 70002],70002,zeros(1,0)};
    for repetition=1:8
        R.phase='CREATING_THROWAWAY_SENSOR';
        sensor=SIMULATED_OBSTACLE_SENSOR(fiction,4,false);
        mem=OBSERVED_OBSTACLE_MEMORY(sensor.empty_observed_obstacle_schema());
        prepared=sensor.observed_prepared_only(zeros(1,0));
        p_ids=zeros(1,0);
        for j=1:numel(visit)
            R.phase=sprintf('REPEAT_%d_STEP_%d',repetition,j);
            t=(double(repetition-1)*numel(visit)+double(j-1))* ...
                double(cfg.sample_dt_s);
            observed=sensor.observe(t,[visit(j);0;0]);
            ids=reshape(double(observed.detected_ids),1,[]);
            assert(isequal(ids,expected{j}), ...
                'TRIGGER_ISOLATED_SENSOR_EVENT_PREPARE:DetectionFixture', ...
                'Synthetic detection fixture produced unexpected IDs.');
            result=mem.update(observed);
            fresh=reshape(double(result.newly_acquired_ids),1,[]);
            if ~isempty(fresh)
                acquired=sensor.observed_prepared_only(fresh);
                prepared=OBSERVED_OBSTACLE_MEMORY.append_row_structs( ...
                    prepared,acquired);
                p_ids=[p_ids fresh]; %#ok<AGROW> software fixture only
            end
            slim=mem.known_frame_for_trigger(observed);
            assert(isequal(slim.detected_ids,p_ids) && ...
                isequal(reshape(double([prepared.id]),1,[]),p_ids) && ...
                isequal(result.known_obstacle_ids,p_ids) && ...
                ~result.automatic_release_enabled, ...
                'TRIGGER_ISOLATED_SENSOR_EVENT_PREPARE:Memory', ...
                'Synthetic memory/prepared geometry order mismatch.');
            R.steps=R.steps+1;
        end
        R.cycles=repetition;
    end
    R.phase='FINISHED';
    R.pass=true;
    R.status='SYNTHETIC_SENSOR_MEMORY_TRANSITIONS_PASS__NO_JIT_OR_SAFETY_PROOF';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
