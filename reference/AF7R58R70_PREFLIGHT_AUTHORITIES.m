function S = AF7R58R70_PREFLIGHT_AUTHORITIES(agent,replanner,time,opts,nominal_ref)
% R70: FAIL-CLOSED SOURCE-BOUND PREFLIGHT. Run ONCE AFTER arming, BEFORE Flight.
% Does not modify any controller, estimator, plant, reference, sensor, or parameter.
% NO cached parameter table and NO guessed fallback for external authorities.
    arguments
        agent
        replanner
        time
        opts (1,1) struct
        nominal_ref
    end
    required = {'trigger_dist','r_drone','r_load','cable'};
    for k=1:numel(required)
        assert(isfield(opts,required{k}), ...
            '[R70] Missing externally specified option: %s',required{k});
    end
    assert(isequal(replanner.self,agent) && isequal(replanner.base_ref,nominal_ref), ...
        '[R70] Active replanner is bound to a different agent or nominal reference.');
    assert(isprop(agent,'plant') && ~isempty(agent.plant), ...
        '[R70] Live plant is unavailable.');
    assert(isprop(agent,'parameter') && ~isempty(agent.parameter), ...
        '[R70] Live external parameter provider is unavailable.');
    p=agent.parameter;
    g=R70_scalar(p.get("gravity"),'gravity');
    L=R70_scalar(p.get("cableL"),'cableL');
    mQ=R70_scalar(p.get("mass"),'mass');
    jx=R70_scalar(p.get("jx"),'jx');
    jy=R70_scalar(p.get("jy"),'jy');
    jz=R70_scalar(p.get("jz"),'jz');
    assert(all([g,L,mQ,jx,jy,jz]>0), ...
        '[R70] External physical constants must be strictly positive.');
    dt=R70_scalar(time.dt,'time.dt');
    assert(dt>0 && R70_same(dt,double(agent.plant.dt)) && ...
        R70_same(dt,double(replanner.prediction_dt)), ...
        '[R70] time, plant, and replanner control periods disagree.');
    assert(R70_same(L,double(replanner.L_cable)) && ...
        R70_same(g,double(replanner.gravity)), ...
        '[R70] Replanner has stale physical constants. Reconstruct it before Flight.');
    sensorRange=R70_scalar(opts.trigger_dist,'opts.trigger_dist');
    assert(R70_same(sensorRange,double(replanner.trigger_dist)), ...
        '[R70] Sensor range changed after replanner construction.');
    assert(R70_same(R70_scalar(opts.r_drone,'opts.r_drone'),double(replanner.r_drone)) && ...
        R70_same(R70_scalar(opts.r_load,'opts.r_load'),double(replanner.r_load)) && ...
        R70_same(R70_scalar(opts.cable,'opts.cable'),double(replanner.r_cable)), ...
        '[R70] Protection geometry differs from external Flight options.');
    H=R70_scalar(replanner.horizon_s,'replanner.horizon_s');
    assert(H>0 && R70_same(H,4.0), ...
        '[R70] Four-second collision trigger horizon contract is violated.');
    assert(sensorRange>0 && all([replanner.r_drone,replanner.r_load,replanner.r_cable]>0), ...
        '[R70] Missing valid externally supplied sensing/body geometry.');
    % Flight mass authority is the latest estimator output, never the
    % plant's simulation truth or parameter provider's initial guess.
    mL=R70_scalar(agent.estimator.result.state.mL,'estimator.state.mL');
    assert(mL>0, '[R70] Live estimated payload mass must be positive.');
    % Ensure the static-obstacle source is not silently changed after
    % construction. This read does not teach the sensor an unseen obstacle.
    liveObs=ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(0.0);
    assert(isequaln(liveObs,replanner.static_obstacle_map), ...
        '[R70] Obstacle source changed after construction; rebuild and recertify before Flight.');
    meth=agent.plant.method;
    assert(isa(meth,'function_handle'), '[R70] Live plant ODE function handle is missing.');
    f=functions(meth);
    src='';
    if isfield(f,'file') && ~isempty(f.file),src=char(f.file);end
    if isempty(src),src=which(func2str(meth));end
    assert(~isempty(src), '[R70] Cannot resolve the live suspended-load ODE authority.');
    S=struct();
    S.status='PREFLIGHT_LIVE_EXTERNAL_AUTHORITIES_MATCH';
    S.dynamic_read_per_flight=true;
    S.live_mass_read_every_replanner_tick=true;
    S.nominal_class=class(nominal_ref);
    S.plant_method=func2str(meth);
    S.plant_method_source=src;
    S.gravity=g;S.cableL=L;S.massQ=mQ;
    S.inertia=[jx,jy,jz];S.massL_estimate_at_preflight=mL;
    S.control_dt=dt;S.sensor_range=sensorRange;S.trigger_horizon=H;
    S.hlc_loadmass_parameter_at_preflight=R70_scalar(p.get("loadmass"),'parameter.loadmass');
    S.hlc_and_ekf_mass_match_at_preflight=R70_same( ...
        S.hlc_loadmass_parameter_at_preflight,mL);
    S.obstacle_count_at_preflight=numel(liveObs);
    S.body_radii=[replanner.r_drone,replanner.r_cable,replanner.r_load];
    S.hlc_modified=false;
    S.true_closed_loop_four_second_certified=false;
    S.natural_rejoin_committed=false;
end

function v = R70_scalar(v,name)
    assert(isnumeric(v)&&isscalar(v)&&isreal(v)&&isfinite(v), ...
        '[R70] Invalid external authority: %s',name);
    v=double(v);
end

function yes=R70_same(a,b)
    yes=isnumeric(a)&&isnumeric(b)&&isscalar(a)&&isscalar(b)&& ...
        isfinite(a)&&isfinite(b)&&abs(double(a)-double(b))<= ...
        64*eps(max(1,max(abs(double(a)),abs(double(b)))));
end
