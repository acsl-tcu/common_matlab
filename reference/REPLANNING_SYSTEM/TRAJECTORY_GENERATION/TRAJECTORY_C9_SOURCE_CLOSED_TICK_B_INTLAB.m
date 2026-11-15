function R=TRAJECTORY_C9_SOURCE_CLOSED_TICK_B_INTLAB(agent,prior_ekf_25x2,prior_cov_25x25x2,current_meas_12x2,previous_held_4x2,current_plant_24x2,reference_HLC_28x2,observed,radii,dt)
% Source-linked idealized ONE-TICK EKF -> HLC -> plant -> entire-body tube.
% This is NOT a claimed 4-second invariant B tube or a 5ms online path.
% State/measurement/covariance/reference enclosures must be rigorous, not
% EKF covariance mistaken for deterministic state-error bounds.
% Flight samples u_pre at t_k; source MODEL_CLASS holds the 4 inputs on
% [t_k,t_k+dt]. For the EXACT loaded MATLAB controller logic, the yaw
% wrapping must have a unique continuous integer branch on the state box.
% In particular, the CURRENT estimated mass interval is used as truth of
% this ideal-model tick. No frozen payload mass or model-mismatch bound.
R=struct('status','UNKNOWN_C9_SOURCE_CLOSED_TICK_NOT_RUN', ...
 'input_limit_proven',false,'plant_full_tick_proven',false, ...
 'ekf_map_validated',false,'body_full_tick_proven',false, ...
 'closed_one_tick_conditionally_proven',false, ...
 'future_4s_invariance_proven',false,'active_Flight_switch_authorized',false, ...
 'actual_double_execution_roundoff_proven',false,'safe_to_commit',false, ...
 'estimated_next_25x2',NaN(25,2),'cov_next_25x25x2',NaN(25,25,2), ...
 'plant_next_24x2',NaN(24,2),'plant_all_24x2',NaN(24,2), ...
 'unsaturated_four_inputs_4x2',NaN(4,2), ...
 'stage','INIT','error_message','','elapsed_ms',NaN);
clock=tic;
try
    if exist('infsup','file')~=2 || exist('intval','file')~=2
        R.status='UNKNOWN_C9_CLOSED_TICK_INTLAB_MISSING';
        R.elapsed_ms=1000*toc(clock);return
    end
    if ~(valid_box(reference_HLC_28x2,28) && ...
        valid_box(current_plant_24x2,24) && ...
        isobject(agent)&&isprop(agent,'cha')&&strcmp(char(agent.cha),'f'))
        R.status='UNKNOWN_C9_CLOSED_TICK_INVALID_FLIGHT_BOX';
        R.elapsed_ms=1000*toc(clock);return
    end
    % 1. Original EKF source, INCLUDING sampled covariance/gain update.
    R.stage='SOURCE_EKF';
    E=TRAJECTORY_C9_EKF_INTERVAL_UPDATE_INTLAB( ...
        agent,prior_ekf_25x2,prior_cov_25x25x2, ...
        current_meas_12x2,previous_held_4x2,dt, ...
        prior_ekf_25x2(25,:));
    if ~E.valid
        R.status=['UNKNOWN_C9_CLOSED_TICK_EKF_' E.status];
        R.elapsed_ms=1000*toc(clock);return
    end
    R.ekf_map_validated=true;
    R.estimated_next_25x2=E.xhat_next_25x2;
    R.cov_next_25x25x2=E.cov_next_25x25x2;
    est=infsup(E.xhat_next_25x2(:,1),E.xhat_next_25x2(:,2));
    % 2. Original HLC input state order: [compact quaternion; w; pL;
    % vL; pT; wL]. Convert source EKF Euler to source Eul2Quat.
    R.stage='SOURCE_HLC_STATE_AND_YAW';
    ctrl=agent.controller.hlc_suspended;
    if ~(isa(ctrl,'HLC_SUSPENDED_LOAD') && ...
            isequal(double(ctrl.param.dt),double(dt)))
        R.status='UNKNOWN_C9_CLOSED_TICK_HLC_SOURCE_NOT_MATCHED';
        R.elapsed_ms=1000*toc(clock);return
    end
    quat=Eul2Quat(est(4:6));
    if ~isa(quat,'intval')||~isequal(size(quat),[4 1])
        R.status='UNKNOWN_C9_CLOSED_TICK_QUATERNION_RANGE';
        R.elapsed_ms=1000*toc(clock);return
    end
    x=[quat;est(10:12);est(13:15);est(16:18); ...
       est(19:21);est(22:24)];
    if ~isequal(size(x),[19 1])
        R.status='UNKNOWN_C9_CLOSED_TICK_HLC_STATE_LAYOUT';
        R.elapsed_ms=1000*toc(clock);return
    end
    xd=infsup(reference_HLC_28x2(:,1),reference_HLC_28x2(:,2));
    % Equivalent source yaw branch in a domain with no +/-pi cut.
    yaw=est(6);refYaw=xd(4);
    yl=double(inf(yaw)); yh=double(sup(yaw));
    dl=double(inf(yaw-refYaw));dh=double(sup(yaw-refYaw));
    if ~(yl>-pi&&yh<pi&&isfinite(dl)&&isfinite(dh))
        R.status='UNKNOWN_C9_CLOSED_TICK_YAW_WRAP_BOUNDARY';
        R.elapsed_ms=1000*toc(clock);return
    end
    j=round((dl+dh)/(4*pi));
    if ~(dl>(2*j-1)*pi && dh<(2*j+1)*pi)
        R.status='UNKNOWN_C9_CLOSED_TICK_YAW_BRANCH_AMBIGUOUS';
        R.elapsed_ms=1000*toc(clock);return
    end
    % Sign(cross)*acos(dot) is the principal angle difference. Its
    % source-equivalent adjusted reference is yd+2*pi*j on this branch.
    xd(4)=refYaw+intval(2*pi*j);
    physics=double(agent.parameter.get(["mass","jx","jy","jz", ...
                        "gravity","loadmass","cableL"]));
    physics=physics(:).';
    if ~(numel(physics)==7 && all(isfinite(physics)) && ...
            physics(1)>0 && all(physics(2:4)>0)&&physics(7)>0)
        R.status='UNKNOWN_C9_CLOSED_TICK_PHYSICS';
        R.elapsed_ms=1000*toc(clock);return
    end
    % The latest EKF mL is the model truth here, including zero as valid.
    PI=[infsup(physics(1:5),physics(1:5)),est(25), ...
        intval(physics(7)),intval(0),intval(0)];
    g=ctrl.param;
    R.stage='SOURCE_FOUR_PRECLAMP_INPUTS';
    vf=ctrl.Vfd_SuspendedLoadxyDst(g.dt,x,xd.',g.F1);
    vs=ctrl.Vs_SuspendedLoadxyDst(x,xd.',vf,PI,g.F2,g.F3,g.F4);
    beta=ctrl.Beta2_SuspendedLoadxyDst(x,xd.',vf,PI);
    rhs=ctrl.V2_alpha2_SuspendedLoadxyDst(x,xd.',vf,vs',PI);
    uf=ctrl.Uf_SuspendedLoadxyDst(x,xd.',vf,PI);
    [ulo,uhi]=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.source_input_limits();
    I=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_INTLAB(ulo,uhi,beta,rhs,uf(1));
    if ~I.conditional_4input_bound_proven
        R.status=['UNKNOWN_C9_CLOSED_TICK_4INPUT_' I.status];
        R.elapsed_ms=1000*toc(clock);return
    end
    R.unsaturated_four_inputs_4x2=I.input_enclosure_4x2;
    R.input_limit_proven=true;
    % 3. Original nonlinear plant RHS, with the latest EKF estimate held
    % as the ideal payload mass throughout THIS plant integration tick.
    R.stage='SOURCE_PLANT_ZOH';
    plant=agent.plant;
    PP=double(plant.param(:).');
    if ~(isa(plant,'MODEL_CLASS')&&numel(PP)==21&& ...
            all(isfinite(PP))&&strcmp(func2str(plant.method), ...
                                      'with_load_model_euler_for_HL'))
        R.status='UNKNOWN_C9_CLOSED_TICK_PLANT_SOURCE';
        R.elapsed_ms=1000*toc(clock);return
    end
    Pb=[PP(:),PP(:)];
    Pb(20,:)=E.xhat_next_25x2(25,:); % ideal mass is live, NEVER fixed.
    Q=TRAJECTORY_C9_PLANT_MASS_INTERVAL_ZOH_REACH( ...
        plant.method,Pb,current_plant_24x2,I.input_enclosure_4x2,dt);
    if ~Q.valid
        R.status=['UNKNOWN_C9_CLOSED_TICK_PLANT_' Q.status];
        R.elapsed_ms=1000*toc(clock);return
    end
    R.plant_full_tick_proven=true;
    R.plant_next_24x2=Q.end_24x2;
    R.plant_all_24x2=Q.all_24x2;
    % 4. Entire straight cable, UAV and payload, every obstacle and ground.
    R.stage='DIRECTED_FULL_BODY_SEPARATION';
    S=TRAJECTORY_C9_HLC_TICK_BODY_SEPARATION_INTLAB( ...
        Q.all_24x2,observed,radii);
    if ~S.conditional_alltime_body_separation
        R.status=['UNKNOWN_C9_CLOSED_TICK_GEOMETRY_' S.status];
        R.elapsed_ms=1000*toc(clock);return
    end
    R.body_full_tick_proven=true;
    R.closed_one_tick_conditionally_proven=true;
    R.status='CONDITIONAL_SOURCE_EKF_HLC_ZOH_PLANT_ALLBODY_ONE_TICK__MEASUREMENT_PROVENANCE_AND_DOUBLE_ROUNDOFF_PENDING';
catch ME
    R.status=['UNKNOWN_C9_CLOSED_TICK_EXCEPTION_' ME.identifier];
    R.error_message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
function ok=valid_box(x,n)
ok=isnumeric(x)&&isreal(x)&&isequal(size(x),[n 2]) && ...
    all(isfinite(x(:)))&&all(x(:,1)<=x(:,2));
end
