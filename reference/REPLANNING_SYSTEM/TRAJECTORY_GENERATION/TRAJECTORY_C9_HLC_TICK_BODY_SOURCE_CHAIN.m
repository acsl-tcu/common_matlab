function R=TRAJECTORY_C9_HLC_TICK_BODY_SOURCE_CHAIN(agent,time,initial_plant_box_24x2,observed,radii)
%TRAJECTORY_C9_HLC_TICK_BODY_SOURCE_CHAIN One source-bound ideal-model tick.
% AFTER the actual HLC.do, BEFORE plant.do: validate HLC PRE-clamp inputs,
% contain the actual source nonlinear ZOH plant on [t_k,t_k+dt], check the
% ENTIRE body/cable swept occupancy against all sensor-known static shapes
% and Flight ground. HLC/plant/EKF/runner are not modified.
%
% A mathematical ONE-TICK model result is NOT a future full-horizon B tube
% or a Flight deployment approval. No fabricated EKF future states,
% reference yaw boxes, mass mismatch bounds or numerical samples.
R=struct('status','UNKNOWN_SOURCE_TICK_BODY_CHAIN_NOT_RUN', ...
    'current_tick_preclamp_four_inputs_proven',false, ...
    'source_plant_intersample_reach_proven',false, ...
    'conditional_body_separation_proven',false, ...
    'future_EKF_HLC_inductive_reach_proven',false, ...
    'safe_to_commit',false,'hlc_plant_status','NOT_RUN', ...
    'body_status','NOT_RUN','hlc_plant_result',struct(), ...
    'body_result',struct(),'elapsed_ms',NaN, ...
    'scope','IDEAL_MODEL_SOURCE_BOUND_ONE_TICK_ONLY');
tk=tic;
try
    H=TRAJECTORY_C9_HLC_SOURCE_TICK_PLANT_CHAIN( ...
        agent,time,initial_plant_box_24x2);
    R.hlc_plant_result=H;
    R.hlc_plant_status=H.status;
    R.current_tick_preclamp_four_inputs_proven= ...
        H.four_preclamp_inputs_in_bounds;
    R.source_plant_intersample_reach_proven= ...
        H.ideal_plant_source_tick_reach_proven;
    if ~H.ideal_plant_source_tick_reach_proven || ...
            ~H.four_preclamp_inputs_in_bounds
        R.status=['UNKNOWN_SOURCE_TICK_BODY_PREREQUISITE_' H.status];
        R.elapsed_ms=1000*toc(tk);return
    end
    G=TRAJECTORY_C9_HLC_TICK_BODY_SEPARATION_INTLAB( ...
        H.plant_box_all_24x2,observed,radii);
    R.body_result=G;
    R.body_status=G.status;
    R.conditional_body_separation_proven= ...
        G.conditional_alltime_body_separation;
    if G.conditional_alltime_body_separation
        R.status='CONDITIONAL_HLC_SOURCE_ZOH_TICK_FOUR_INPUT_AND_BODY_ALLTIME__FUTURE_EKF_PENDING';
    else
        R.status=['UNKNOWN_HLC_SOURCE_TICK_BODY_' G.status];
    end
catch ME
    R.status=['UNKNOWN_HLC_SOURCE_TICK_BODY_EXCEPTION_' ME.identifier];
end
R.elapsed_ms=1000*toc(tk);
end
