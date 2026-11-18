function R=wp_six_trigger_selected_nominal_v8(agent,H,K,routeCache,known,cfg,model)
%WP_SIX_TRIGGER_SELECTED_NOMINAL_V8 PV-modal 4s + ONE selected geometry.
% Requires PREPARED route, modal matrices, and observed-obstacle cache.
% Per cycle reads ONLY latest EKF estimated mL as prediction mass.
% No HLC call, no RHS integration, no new obstacle fitting in this tick.
%
% IMPORTANT: This is NOMINAL obstacle screening, NOT certified clearance.
% The geometry subkernel is evaluated on the piecewise-linear predicted
% 0.1-second knots with ZERO additional interpolation/physical error
% envelope, *solely* to compare NOMINAL paths. The zero envelope is NOT
% claimed as an actual acceleration or uncertainty bound. The final API
% NEVER returns CLEAR_CONDITIONAL / safe-to-commit.
watch=tic;
R=struct('status','UNKNOWN','reason','', ...
    'physicalClearCertified',false,'safeToCommit',false, ...
    'futureInputHeldConstant',false,'oneSelectedGeometry',true, ...
    'modelLoadMassUsed',NaN,'forecastMs',NaN,'geometryMs',NaN, ...
    'totalMs',NaN,'nominalContactCandidate',false, ...
    'nominalPolylineGeometryOnly',true,'boundsCertified',false, ...
    'rawNominalGeometryStatus','','nominalFirstTime_s',NaN);
try
    if known.numParts==0 && ~known.groundEnabled
        R.status='NO_OBSERVED_OBSTACLES_SKIP_PREDICTION';
        R.totalMs=1000*toc(watch);return
    end
    if known.dynamicUnsupported || ~known.groundSupported || ...
        ~strcmp(known.approximation,cfg.obstacleApproximation)
        R.reason='unsupported_observed_world';
        R.totalMs=1000*toc(watch);return
    end
    if nargin<7
        model=wp_six_trigger_model_snapshot(agent);
    end
    R.modelLoadMassUsed=model.loadMass;
    if cfg.cableLength~=model.cableLength || ...
            H.cableLength~=model.cableLength || ...
            H.gravity~=model.gravity
        R.reason='stale_physical_model_or_route_cache';
        R.totalMs=1000*toc(watch);return
    end
    st=agent.estimator.result.state;
    % A caller may retain a rolling 1s buffer, but it must be brought
    % up-to-date with the CURRENT EKF sample before this function runs.
    if any(~isfinite([st.pL(:);st.vL(:)])) || ...
        norm(H.pastLoad(:,end)-st.pL(:))>1e-8 || ...
        norm(H.pastVelocity(:,end)-st.vL(:))>1e-8
        R.reason='stale_EKF_history_latest_sample';
        R.totalMs=1000*toc(watch);return
    end
    H.estimatedML=model.loadMass; % each tick; NO plant mass, NO fallback
    cfg.currentLoadMass=model.loadMass;
    w=tic;
    pred=wp_six_trigger_pv_forecast_v8(H,K,routeCache);
    R.forecastMs=1000*toc(w);
    R.initialAnchor=pred.initialAnchor;
    % Do not label an inconsistent cable model as a nominal clearance.
    % In particular, NEVER snap just one t=0 output to the EKF position.
    if pred.initialAnchor.cableAnchorRequested && ...
            ~pred.initialAnchor.cableAnchorApplied
        R.status='UNKNOWN_INITIAL_CABLE_DYNAMICS';
        R.reason=pred.initialAnchor.cableAnchorStatus;
        R.totalMs=1000*toc(watch);return
    end
    % 41 fixed 0.1 s knots, exact sampled output; NOT an intersample proof.
    ids=1:4:161;
    n=numel(ids);
    F=struct('timeOffsets',pred.timeOffsets(ids), ...
        'pDrone',pred.pDrone(:,ids),'pLoad',pred.pLoad(:,ids), ...
        'rhoDrone',zeros(1,n),'rhoLoad',zeros(1,n), ...
        'accDrone',zeros(1,n-1),'accLoad',zeros(1,n-1), ...
        'modelLoadMass',model.loadMass);
    w=tic;
    % Its conditional status is never exposed as a physical SAFE claim.
    G=wp_six_trigger_evaluate_fast_v8(known,cfg,F);
    R.geometryMs=1000*toc(w);
    R.rawNominalGeometryStatus=G.status;
    if isfield(G,'firstTime'), R.nominalFirstTime_s=G.firstTime; end
    switch G.status
        case 'CLEAR_CONDITIONAL'
            R.status='NOMINAL_POLYLINE_SEPARATED__NOT_CERTIFIED';
        case 'POSSIBLE_CONTACT'
            R.status='NOMINAL_POLYLINE_POSSIBLE_CONTACT__NOT_CERTIFIED';
            R.nominalContactCandidate=true;
        otherwise
            R.status='UNKNOWN';
            R.reason=G.reason;
    end
    R.nominalGeometry=G;
    R.predictor=struct('status',pred.status, ...
         'massUsed',pred.modelEstimatedMassUsed, ...
         'governorAt4s',pred.virtualTime(end), ...
         'fitCondition',pred.fitCondition);
catch ME
    R.status='UNKNOWN';R.reason=[ME.identifier ': ' ME.message];
end
R.totalMs=1000*toc(watch);
end
