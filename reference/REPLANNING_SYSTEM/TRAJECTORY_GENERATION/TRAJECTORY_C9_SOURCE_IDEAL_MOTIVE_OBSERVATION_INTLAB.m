function R=TRAJECTORY_C9_SOURCE_IDEAL_MOTIVE_OBSERVATION_INTLAB(agent,plant_24x2)
% Source-matched IDEAL noiseless MOTIVE observation of ONE plant state box.
% Does not replace sensor observations or add a made-up noise tolerance.
% run_flight_headless calls NATNET_CONNECTOR_SIM.getData and MOTIVE.do with
% the function motive_output: y=[p;Quat2Eul(q);pL;(pL-p)/||pL-p||].
% The plant stores RPY; its state.getq('compact') is Eul2Quat(RPY).
% Only valid for Flight connector Noise=0, no occlusion, unmodified
% output callback and nonzero cable displacement. Uses INTLAB throughout.
% Mathematical real-function enclosure; actual MATLAB double-roundoff of
% Eul2Quat/Quat2Eul/normalization is NOT yet a validated error model.
R=struct('status','UNKNOWN_C9_MOTIVE_OBSERVATION_INPUT', ...
 'valid',false,'ideal_measurement_12x2',NaN(12,2), ...
 'noise_free_source_checked',false,'no_occlusion_checked',false, ...
 'matlab_double_roundoff_proven',false,'safe_to_commit',false);
if ~(isnumeric(plant_24x2)&&isreal(plant_24x2)&& ...
    isequal(size(plant_24x2),[24 2])&&all(isfinite(plant_24x2(:)))&& ...
    all(plant_24x2(:,1)<=plant_24x2(:,2)))
    return
end
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_C9_MOTIVE_INTLAB_MISSING';return
end
try
    sensor=agent.sensor.motive;
    connector=sensor.motive;
    if ~(isa(sensor,'MOTIVE') && isa(connector,'NATNET_CONNECTOR_SIM') && ...
            isstruct(connector.Flag)&&isfield(connector.Flag,'Noise')&& ...
            isequal(connector.Flag.Noise,0) && isempty(connector.occlusion_param))
        R.status='UNKNOWN_C9_MOTIVE_SENSOR_NOISE_OR_OCCLUSION';return
    end
    if ~(isa(sensor.output_func,'function_handle') && ...
            endsWith(char(func2str(sensor.output_func)),'motive_output') && ...
            numel(sensor.rigid_id)==2 && ...
            isequal(double(sensor.rigid_id(:).'),[1 2]) && ...
            strcmp(char(sensor.q_type),'3'))
        R.status='UNKNOWN_C9_MOTIVE_SOURCE_CALLBACK_NOT_VERIFIED';return
    end
    initq=compact(sensor.initq);
    if ~(numel(initq)==4 && norm(double(initq(:).')-[1 0 0 0])<1e-14)
        R.status='UNKNOWN_C9_MOTIVE_NONIDENTITY_YAW_CALIBRATION';return
    end
    R.noise_free_source_checked=true;
    R.no_occlusion_checked=true;
    X=infsup(plant_24x2(:,1),plant_24x2(:,2));
    p=X(1:3);pL=X(13:15);
    d=pL-p;
    norm2=sum(d.^2);
    if ~(double(inf(norm2))>0 && isfinite(double(sup(norm2))))
        R.status='UNKNOWN_C9_MOTIVE_CABLE_DISPLACEMENT_NOT_NONZERO';return
    end
    q=Eul2Quat(X(4:6));
    angles=Quat2Eul(q);
    if ~isa(angles,'intval')||~isequal(size(angles),[3 1])
        R.status='UNKNOWN_C9_MOTIVE_SOURCE_ATTITUDE_INTERVAL';return
    end
    % No assertion of branch continuity from samples. A non-singleton
    % interval crossing an atan2 branch must be rejected explicitly.
    if any(double(inf(X(4:6)))<=-pi | double(sup(X(4:6)))>=pi) || ...
            double(inf(X(5)))<=-pi/2 || double(sup(X(5)))>=pi/2
        R.status='UNKNOWN_C9_MOTIVE_EULER_BRANCH_AMBIGUOUS';return
    end
    pT=d./sqrt(norm2);
    y=[p;angles;pL;pT];
    lo=double(inf(y));hi=double(sup(y));
    if any(~isfinite([lo;hi]))||any(lo>hi)
        R.status='UNKNOWN_C9_MOTIVE_MEASUREMENT_NOT_FINITE';return
    end
    R.ideal_measurement_12x2=[lo,hi];
    R.valid=true;
    R.status='CONDITIONAL_NOISELESS_SIM_MOTIVE_IDEAL_MEASUREMENT_BOX__DOUBLE_ROUNDOFF_PENDING';
catch ME
    R.status=['UNKNOWN_C9_MOTIVE_EXCEPTION_' ME.identifier];
end
end
