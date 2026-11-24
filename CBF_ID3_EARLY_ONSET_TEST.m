function result = CBF_ID3_EARLY_ONSET_TEST()
% Standalone diagnosis: FIRST CBF intervention and growth BEFORE failure.
% Does NOT edit HLC, Sim, controller, generator, any project sources.
% Run: result = CBF_ID3_EARLY_ONSET_TEST;
% Writes ONE ZIP next to this .m, no persistent CSV/MAT/TXT outside ZIP.
% Logged quantities below are TRANSCRIBED from the original displayed
% 2026-11-24 flight log (rounded by diagnostic printing).
% Console R is the reference xd, P is estimator position p; R is NOT load.
% The physical x(19) is NOT in that log. Reconstructed geometry/dynamics
% sensitivity is a HYPOTHESIS, NOT an exact replay of the flight.
% FIRST intervention t=26.075; first QP failure t=26.450.
% Analysis is intentionally focused on pre-failure t<=26.300.
%
% Key identity for the actual discrete-four-stage generator:
% phi4(k+1) = product_j(1+dt*lambda_j)*h(k+1) - C(phi0..phi3(k)).
% Therefore the gradient w.r.t. the current physical input is exactly
% kappa*dh(k+1)/du. Past stage history is constant w.r.t. this input.
% We anchor counterfactual deltas to the *observed* phi4 of the already
% applied command, rather than pretending synthetic history is measured.

projectPath=path;
cleanupPath=onCleanup(@() path(projectPath)); %#ok<NASGU>
here=fileparts(mfilename('fullpath'));
required={'DRONE_PARAM_SUSPENDED_LOAD','DRONE_OBSTACLES', ...
    'CBF_REAL_D2T_SAMPLED_EVAL'};
for q=1:numel(required)
    if exist(required{q},'file')~=2
        addpath(genpath(here),'-begin');
        break
    end
end
for q=1:numel(required)
    if exist(required{q},'file')~=2
        error('ID3Early:MissingFunction', ...
            '%s not on MATLAB path. Put test in common_matlab root.',required{q});
    end
end

% EXACT PRINTED LOG (precision is limited to printed diagnostic digits).
% cols: t, pDroneXYZ, applied[T,tx,ty], rawHLC[T,tx,ty],
%       logged[phi0,phi1,phi2,phi3,phi4_next], ms, logged[dT,dtauX,dtauY],
%       statusCode:0 nominal,1 joint accepted,2 QP failed.
D=[
    25.9 0 7.987 3 8.849 0 0 8.85 1.31e-11 9.97e-12 0.53 0.0146 0.000869 0.000115 2.98e-05 0.8158 0 0 0 0;
    25.925 0 8 3 8.849 0 0 8.85 2e-11 8.57e-12 0.518 0.014 0.000811 0.000104 2.51e-05 1.179 0 0 0 0;
    25.95 0 8.012 3 8.849 0 0 8.85 3.83e-11 3.63e-12 0.506 0.0134 0.000752 9.19e-05 2.05e-05 0.8137 0 0 0 0;
    25.975 0 8.025 3 8.849 0 -0 8.85 4.95e-11 -2.29e-12 0.494 0.0128 0.000694 8.03e-05 1.58e-05 0.7154 0 0 0 0;
    26 0 8.037 3 8.849 0 -0 8.85 3.6e-11 -4.25e-12 0.482 0.0122 0.000635 6.86e-05 1.11e-05 0.7858 0 0 0 0;
    26.025 0 8.05 3 8.849 0 -0 8.85 3.07e-11 -7.47e-12 0.47 0.0116 0.000577 5.7e-05 6.5e-06 0.6763 0 0 0 0;
    26.05 0 8.062 3 8.849 0 -0 8.85 8.69e-12 -6.23e-12 0.459 0.0111 0.000519 4.53e-05 1.86e-06 0.7961 0 0 0 0;
    26.075 0 8.075 3 8.862 0 -0 8.85 -1.34e-11 -2e-12 0.447 0.0105 0.00046 3.37e-05 -1.45e-10 362.4 0.0137 5.26e-06 -1.36e-06 1;
    26.1 0 8.087 3 8.893 0 -0 8.84 -2.33e-06 6e-07 0.435 0.00989 0.000404 2.42e-05 -1.2e-11 29 0.0534 2.06e-05 -5.34e-06 1;
    26.125 0 8.1 3 8.944 0 -0 8.81 -1.08e-05 2.78e-06 0.423 0.00931 0.000351 1.77e-05 8.07e-10 47.33 0.134 5.22e-05 -1.36e-05 1;
    26.15 0 8.112 3 9.014 0 -0 8.75 -3.06e-05 7.96e-06 0.411 0.00874 0.000304 1.35e-05 3.13e-10 7.643 0.269 0.000106 -2.76e-05 1;
    26.175 -0 8.125 3 9.102 0 -0 8.63 -6.81e-05 1.78e-05 0.399 0.00818 0.000262 1.11e-05 1.77e-11 13.23 0.472 0.000188 -4.95e-05 1;
    26.2 -0 8.137 3.001 9.182 0 -0 8.45 -0.00013 3.42e-05 0.388 0.00764 0.000226 8.77e-06 2.34e-10 8.748 0.732 0.000297 -7.83e-05 1;
    26.225 -0 8.15 3.001 9.303 0 -0 8.2 -0.000221 5.81e-05 0.376 0.00713 0.000194 7.32e-06 1.49e-09 5.845 1.1 0.000457 -0.000121 1;
    26.25 -0 8.162 3.002 9.414 0 -0 7.86 -0.000352 9.29e-05 0.364 0.00663 0.000168 6.66e-06 1.69e-09 6.507 1.55 0.000662 -0.000176 1;
    26.275 -0 8.175 3.003 9.574 0 -0 7.43 -0.00053 0.00014 0.353 0.00616 0.000145 6.51e-06 1.72e-10 6.089 2.15 0.000954 -0.000253 1;
    26.3 -0 8.187 3.005 9.747 0.001 -0 6.86 -0.000773 0.000205 0.342 0.00571 0.000127 6.83e-06 -1.96e-10 4.948 2.89 0.00135 -0.000358 1;
    26.325 -0 8.2 3.007 10.007 0.001 -0 6.14 -0.0011 0.000291 0.33 0.0053 0.000112 7.49e-06 -7.71e-10 5.16 3.87 0.00193 -0.000512 1;
    26.35 -0 8.212 3.01 10.364 0.001 -0 5.21 -0.00153 0.000406 0.319 0.00491 0.0001 8.54e-06 2.5e-10 6.264 5.15 0.00286 -0.000755 1;
    26.375 -0 8.225 3.013 10.999 0.002 -0.001 3.99 -0.00207 0.000549 0.308 0.00454 9.18e-05 9.97e-06 2.64e-10 6.54 7.01 0.00454 -0.00119 1;
    26.4 -0 8.237 3.019 12.278 0.006 -0.002 2.28 -0.00203 0.000534 0.298 0.00421 8.66e-05 1.21e-05 -1.84e-10 6.351 10 0.00847 -0.00219 1;
    26.425 -0 8.25 3.026 16.074 0.059 -0.016 -0.375 0.0348 -0.00933 0.287 0.0039 8.33e-05 1.34e-05 1.36e-09 12.66 16.4 0.0246 -0.0062 1;
    26.45 -0 8.262 3.036 0 -0.047 0.012 -5.65 -0.0466 0.0122 0.277 0.00362 7.83e-05 1.06e-05 -0.00169 26.4 5.65 0 0 2;
];
alpha=[2;4;8;16]*0.025;
kappa=prod(1+alpha);
assert(abs(kappa-1.9404)<1e-12,'ID3Early:FourStageFactor');

% Object/geometry read from ORIGINAL project, never synthetic sphere.
par=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
P=struct('m',double(par.mass),'mL',0.14, ...
    'J',double([par.jx;par.jy;par.jz]), ...
    'gravity',double(par.gravity),'cableL',2.0, ...
    'dstx',0,'dsty',0);
world=DRONE_OBSTACLES('scene_cylinders_high_side', ...
    'single_ellipsoid','none');
idx=find(strcmp({world.parts.obstacleName},'a'));
if isempty(idx),error('ID3Early:NoObstacleA','Obstacle a was not found.');end
parts=world.parts(idx);
geom=struct('a',P.cableL/2+0.02,'b',0.20,'centerFromLoad',0.5);
dt=0.025;lambda=[2;4;8;16];reserve=0.005;

% Actual orientation is unknown. Use vertical cable and level attitude ONLY
% to study input *sensitivity* under a stated hypothesis.
anchorTimes=[26.025,26.050,26.075,26.100,26.125,26.200,26.300];
Tanchors=[0,2,5,8,10,12,16,20];
sTcases=[1,10,5,10,10,0.0625];
sTauCases=[1,1,1,2,5,2.5];
weightNames={'unit_physical','box_half_range','thrust_more_costly', ...
    'torque_4x_cheaper','torque_25x_cheaper','paper_scale_FOR_COMPARISON_ONLY'};

% Build an exact-log timeline independent of synthetic model assumptions.
% the observed phi4 is for the selected/applied command, NOT the nominal.
logRows=zeros(size(D,1),21);
for i=1:size(D,1)
    logRows(i,:)=[D(i,:), D(i,16)>dt*1000];
end

% COLS for each diagnostic are defined by their CSV headers below.
A=zeros(0,17);W=zeros(0,12);G=zeros(0,13);Q=zeros(0,13);N=zeros(0,12);S=zeros(0,28);H=zeros(0,7);
for kk=1:numel(anchorTimes)
    t=anchorTimes(kk);
    ix=find(abs(D(:,1)-t)<1e-8,1);
    if isempty(ix),error('ID3Early:InternalTime','Time not in embedded data');end
    z=D(ix,:);
    % Historical applied thrusts from TRUE logged (but rounded) U values.
    if ix<=2,error('ID3Early:History','Not enough past thrust data');end
    old=[D(ix-1,5);D(ix-2,5)]; Tbase=2*old(1)-old(2);
    % ASSUMED 19-state: q identity; cable vertical; no body/cable angular
    % rate; load velocity from printed drone positions, not true state.
    pv=(D(ix,2:4)-D(ix-1,2:4)).'/dt;
    pDrone=z(2:4).';pLoad=pDrone-[0;0;P.cableL];
    x=[1;0;0;0; 0;0;0; pLoad; pv; 0;0;-1; 0;0;0];
    evalPhysical=@(T,tx,ty) CBF_REAL_D2T_SAMPLED_EVAL( ...
        x,old(1),old(2),[T-Tbase;tx;ty],0,P,parts,geom, ...
        dt,3,lambda,reserve,[]);
    % Observed, applied command is the ONLY legitimate calibration anchor.
    ua=z(5:7).';ref=evalPhysical(ua(1),ua(2),ua(3));
    hApplied=ref.hEnd(1);
    loggedPhi4=z(15);
    loggedStages=z(11:14);
    % First-onset NOMINAL counterfactual: only the MODEL-PREDICTED CHANGE
    % is used. Absolute phi4 is ANCHORED to logged selected phi4.
    % Earliest 7 time anchors have VALID raw HLC inputs, no clipping.
    % Later negative HLC requests are deliberately NOT included in this
    % physical candidate evaluation because they violate the actuator box.
    rawU=z(8:10).';
    if rawU(1)<0 || rawU(1)>20 || any(abs(rawU(2:3))>1)
        error('ID3Early:RawOutOfBox','Unexpected raw input outside box at early anchor');
    end
    ehRaw=evalPhysical(rawU(1),rawU(2),rawU(3));
    ehTonly=evalPhysical(rawU(1),ua(2),ua(3));
    ehTauOnly=evalPhysical(ua(1),rawU(2),rawU(3));
    phiRaw=loggedPhi4+kappa*(ehRaw.hEnd(1)-hApplied);
    phiTonly=loggedPhi4+kappa*(ehTonly.hEnd(1)-hApplied);
    phiTauOnly=loggedPhi4+kappa*(ehTauOnly.hEnd(1)-hApplied);
    N(end+1,:)=[t,loggedPhi4,z(8),ua(1),z(9:10),ua(2:3).', ...
        phiRaw,phiTonly,phiTauOnly,ua(1)-z(8)]; %#ok<AGROW>
    % Actual current stages are positive in first-onset period.
    [gradGeom, axisRatio,majorAxis]=geometryGrad(x,P,parts,geom);
    G(end+1,:)=[t,axisRatio,gradGeom(:).',majorAxis(:).', ...
        loggedStages(1),hApplied,ua(1),ua(2:3).']; %#ok<AGROW>
    % Spatial cross-check against the actual obstacle approximation,
    % using the log's estimated body position and an ASSUMED vertical cable.
    dObs=x(8:10)-.5*P.cableL*x(14:16)-parts(1).center(:);
    nObs=dObs/norm(dObs);
    ax=double(parts(1).params.axes(:));
    RR=double(parts(1).R);
    sObstacle=norm(diag(ax)*RR.'*nObs);
    sCable=sqrt(geom.b^2+(geom.a^2-geom.b^2)*(nObs.'*x(14:16))^2);
    hNow=norm(dObs)-sObstacle-sCable;
    hOrigin=geomH(x,P,parts,geom);
    for j=1:3
        for sg=[-1 1]
            xp=x;xp(7+j)=xp(7+j)+.1*sg;
            hv=geomH(xp,P,parts,geom);
            H(end+1,:)=[t,j,sg,.1,hOrigin,hv,hv-hOrigin]; %#ok<AGROW>
        end
    end
    S(end+1,:)=[t, z(2:4), x(8:10).', ...
        (x(8:10)-.5*P.cableL*x(14:16)).', ...
        double(parts(1).center(:)).', norm(dObs), ...
        ax.', sObstacle,sCable,hNow,loggedStages(1), ...
        hNow-loggedStages(1),majorAxis(:).', ...
        gradGeom(:).']; %#ok<AGROW>
    for ai=1:numel(Tanchors)
        T=Tanchors(ai); tx=0;ty=0;
        ej=evalPhysical(T,tx,ty);
        % Endpoint and stage sensitivities: adaptive one-sided box FD.
        steps=[0.20;0.10;0.10];
        v=[T;tx;ty]; lo=[0;-1;-1];hi=[20;1;1];
        jH=zeros(1,3); jPhi=zeros(1,3);
        for dim=1:3
            vp=v;vm=v;
            vp(dim)=min(hi(dim),v(dim)+steps(dim));
            vm(dim)=max(lo(dim),v(dim)-steps(dim));
            if vp(dim)-vm(dim)<1e-12
                error('ID3Early:FiniteDifference','Degenerate FD step');
            end
            ep=evalPhysical(vp(1),vp(2),vp(3));
            em=evalPhysical(vm(1),vm(2),vm(3));
            jH(dim)=(ep.hEnd(1)-em.hEnd(1))/(vp(dim)-vm(dim));
            jPhi(dim)=kappa*jH(dim);
        end
        % The numerical phi4 counterfactual is ANCHORED to the actual
        % printed phi4 at applied input. Synthetic-state h differences
        % are approximate and must NOT be called an exact safe condition.
        phi4Surrogate=loggedPhi4+kappa*(ej.hEnd(1)-hApplied);
        A(end+1,:)=[t,T,ua(1),jH,jPhi,phi4Surrogate, ...
            ej.hEnd(1),hApplied, norm(jH), ...
            abs(jH(2:3))*[1;1], max(abs(jH(2:3))), ...
            kappa,loggedStages(1)]; %#ok<AGROW>
        if abs(T-8)<1e-6 || abs(T-10)<1e-6
            for w=1:numel(sTcases)
                sens=[jPhi(1)*sTcases(w), ...
                      jPhi(2)*sTauCases(w),jPhi(3)*sTauCases(w)];
                den=sum(sens.^2);
                if den>1e-30
                    shares=sens.^2/den;
                else
                    shares=[NaN,NaN,NaN];
                end
                W(end+1,:)=[t,T,w,sTcases(w),sTauCases(w), ...
                    jPhi,shares,den]; %#ok<AGROW>
            end
        end
    end
    % At the very first intervention and before it, compare the
    % one-step authority of T-only vs tau-only changes (physical box).
    % This is a surrogate *difference*, not a full flight replay.
    if t>=26.025 && t<=26.125
        h0=evalPhysical(ua(1),ua(2),ua(3));
        baseH=h0.hEnd(1);
        maxThrust=-Inf;bestT=NaN;
        for T=Tanchors
            e=evalPhysical(T,ua(2),ua(3));
            d=e.hEnd(1)-baseH;
            if d>maxThrust,maxThrust=d;bestT=T;end
        end
        maxTorque=-Inf;bestX=NaN;bestY=NaN;
        for tx=[-1,-0.5,0,0.5,1]
            for ty=[-1,-0.5,0,0.5,1]
                e=evalPhysical(ua(1),tx,ty);
                d=e.hEnd(1)-baseH;
                if d>maxTorque,maxTorque=d;bestX=tx;bestY=ty;end
            end
        end
        % An input box can change hEnd by only limited amount in one dt.
        Q(end+1,:)=[t,baseH,maxThrust,bestT,maxTorque,bestX,bestY, ...
            maxThrust/(maxTorque+eps), kappa*maxThrust,kappa*maxTorque, ...
            loggedPhi4,ua(1),min(loggedStages)]; %#ok<AGROW>
    end
end

% Create ONE self-contained ZIP beside this .m. No exported loose files.
tmp=tempname;[ok,msg]=mkdir(tmp);
if ~ok,error('ID3Early:Temp','%s',msg);end
clean=onCleanup(@() localRmdir(tmp)); %#ok<NASGU>
writeCsv(fullfile(tmp,'early_actual_log.csv'), ...
    {'time_s','P_x','P_y','P_z','applied_T','applied_tauX','applied_tauY', ...
    'HLC_raw_T','HLC_raw_tauX','HLC_raw_tauY', ...
    'logged_phi0','logged_phi1','logged_phi2','logged_phi3', ...
    'logged_phi4_next_selected','CBF_ms','logged_dT','logged_dtauX', ...
    'logged_dtauY','status_code','exceeds_25ms'}, ...
    logRows);
writeCsv(fullfile(tmp,'early_anchor_sensitivity.csv'), ...
    {'time_s','linearization_T_N','logged_applied_T_N', ...
    'dhEnd_dT','dhEnd_dtauX','dhEnd_dtauY', ...
    'dphi4_dT','dphi4_dtauX','dphi4_dtauY', ...
    'counterfactual_phi4_APPROX','hEnd_model','hEnd_applied_model', ...
    'sensitivity_norm','abs_tau_sum','abs_tau_max', ...
    'kappa','logged_h_current'},A);
writeCsv(fullfile(tmp,'early_weight_effect.csv'), ...
    {'time_s','linearization_T_N','weight_id','sT_N','sTau_Nm', ...
    'dphi4_dT','dphi4_dtauX','dphi4_dtauY', ...
    'predicted_share_T','predicted_share_tauX','predicted_share_tauY', ...
    'sum_squared_scaled_gradient'},W);
writeCsv(fullfile(tmp,'early_nominal_counterfactual.csv'), ...
    {'time_s','logged_phi4_selected','HLC_raw_T','applied_T', ...
    'HLC_raw_tauX','HLC_raw_tauY','applied_tauX','applied_tauY', ...
    'nominal_phi4_SURROGATE','phi4_rawT_plus_appliedTorque', ...
    'phi4_appliedT_plus_rawTorque','CBF_logged_dT'},N);
writeCsv(fullfile(tmp,'early_geometry.csv'), ...
    {'time_s','obstacle_axis_ratio','dh_dp_x','dh_dp_y','dh_dp_z', ...
    'major_axis_x','major_axis_y','major_axis_z', ...
    'logged_h','model_hEnd_at_applied','applied_T','applied_tauX', ...
    'applied_tauY'},G);
writeCsv(fullfile(tmp,'early_spatial_consistency.csv'), ...
    {'time_s','estimator_drone_x','estimator_drone_y','estimator_drone_z', ...
    'ASSUMED_load_x','ASSUMED_load_y','ASSUMED_load_z', ...
    'ASSUMED_cable_mid_x','ASSUMED_cable_mid_y','ASSUMED_cable_mid_z', ...
    'obstacle_center_x','obstacle_center_y','obstacle_center_z', ...
    'center_distance_m','ellipsoid_axis_1','ellipsoid_axis_2','ellipsoid_axis_3', ...
    'obstacle_support_m','cable_support_m','ASSUMED_h_m','logged_h_m', ...
    'h_assumed_minus_logged_m', ...
    'obstacle_major_axis_x','obstacle_major_axis_y','obstacle_major_axis_z', ...
    'dh_dp_x','dh_dp_y','dh_dp_z'},S);
writeCsv(fullfile(tmp,'early_geometric_moves.csv'), ...
    {'time_s','axis_1X_2Y_3Z','direction_sign','shift_m', ...
    'assumed_h_before','assumed_h_after','delta_h_m'},H);
writeCsv(fullfile(tmp,'early_one_step_authority.csv'), ...
    {'time_s','model_hEnd_applied','max_delta_h_T_only_m', ...
    'best_T_N','max_delta_h_tau_only_m','best_tauX','best_tauY', ...
    'T_over_tau_effect_ratio','phi4_improvement_T_only', ...
    'phi4_improvement_tau_only','logged_phi4_selected', ...
    'logged_applied_T_N','min_current_nested_stage_logged'},Q);

fid=fopen(fullfile(tmp,'SUMMARY.txt'),'wt');
if fid<0,error('ID3Early:Write','Cannot write summary');end
fprintf(fid,'ID3 EARLY-ONSET STUDY — ORIGINAL PROGRAM UNMODIFIED\n');
fprintf(fid,'Only 25.900..26.450 recorded period, physics sensitivity focused <=26.300.\n');
fprintf(fid,'SOURCE DISPLAY SEMANTICS: R=reference xd, P=ESTIMATED DRONE p (not real payload position).\n');
fprintf(fid,'True payload location/cable orientation/angular velocities are NOT displayed.\n');
fprintf(fid,'early_spatial_consistency.csv audits estimated P, hypothesized vertical cable, and scene a.\n');
fprintf(fid,'early_geometric_moves.csv isolates +/-0.1m XYZ cable translation from actuator dynamics.\n');
fprintf(fid,'No clipping of RAW HLC inputs at early analysis anchors; outside-box negative raw inputs are excluded.\n');
fprintf(fid,'Original logged earliest correction: 26.075 s; first QP failure 26.450 s.\n');
fprintf(fid,'At FIRST intervention T=8.862 N, HLC raw T=8.85 N: NO ZERO THRUST CLIP.\n');
fprintf(fid,'Logged selected phi4 first intervention = -1.45e-10 (accepted tolerance), NOT raw nominal phi4.\n');
fprintf(fid,'Log 26.050: phi4 selected=1.86e-6, no CBF modification.\n');
fprintf(fid,'Log 26.425: current phi0..phi3 ALL POSITIVE; first infeasibility 26.450.\n');
fprintf(fid,'First intervention computational time 362.4 ms > 25 ms deadline.\n\n');
fprintf(fid,'DISCRETE FOUR-STAGE IDENTITY\n');
fprintf(fid,'alpha=lambda*dt=[%.6g %.6g %.6g %.6g], kappa=%.9f.\n',alpha,kappa);
fprintf(fid,'phi4_next(u)=kappa*hEnd(u)-C(phi0..phi3 at k).\n');
fprintf(fid,'dphi4_next/du=kappa*dhEnd/du (with current state and past stages fixed).\n');
fprintf(fid,'This is a mathematically exact dependency of GENERATED finite-difference stages,\n');
fprintf(fid,'not evidence of a real 4th-order ACTUATOR integral chain.\n\n');
fprintf(fid,'TEST CAUTIONS\n');
fprintf(fid,'early_actual_log.csv is copied from displayed log; printed precision is limited.\n');
fprintf(fid,'ALL other CSV tables use ASSUMED q identity, cable vertical, load velocity from P,\n');
fprintf(fid,'zero body/cable angular rates, and rounded applied inputs.\n');
fprintf(fid,'Input sensitivities are independent of assumed barrier memory, but NOT of unknown physical state.\n');
fprintf(fid,'Counterfactual phi4 values use logged selected phi4 as anchor plus model-predicted\n');
fprintf(fid,'hEnd DIFFERENCES. Therefore these are approximate, not exact QP replay or safety proof.\n');
fprintf(fid,'Weight scale choices change objective preference, not input feasibility.\n');
fprintf(fid,'Full physical state x(19), all prior stages, exact controls and real solver status\n');
fprintf(fid,'are required to establish the exact FIRST failure cause.\n\n');
fprintf(fid,'FIRST-ONSET GEOMETRIC CROSS-CHECK (ASSUMED VERTICAL CABLE)\n');
ix=find(abs(S(:,1)-26.075)<1e-8,1);
if ~isempty(ix)
    fprintf(fid,'26.075 estimated body P=[%.4f %.4f %.4f], presumed midpoint=[%.4f %.4f %.4f].\n', ...
        S(ix,2:4),S(ix,8:10));
    fprintf(fid,'original fitted obstacle centre=[%.4f %.4f %.4f], axes=[%.4f %.4f %.4f].\n', ...
        S(ix,11:13),S(ix,15:17));
    fprintf(fid,'model h0=%.6g, logged h=%.6g, difference=%.6g (printed precision).\n\n', ...
        S(ix,20:22));
end
fprintf(fid,'FIRST NOMINAL COUNTERFACTUAL (NOT EXACT FULL-STATE REPLAY)\n');
for q=1:size(N,1)
    fprintf(fid,'t=%.3f: selected_phi4=%.4g, estimated_raw_nominal_phi4=%.4g, dT=%.4g N.\n', ...
        N(q,1),N(q,2),N(q,9),N(q,12));
end
fprintf(fid,'The nominal estimate uses approximate unknown state and printed rounded inputs.\n\n');
fprintf(fid,'WEIGHT ID LABELS\n');
for w=1:numel(weightNames)
    fprintf(fid,'%d %s: sT=%.6g N, sTau=%.6g Nm\n', ...
        w,weightNames{w},sTcases(w),sTauCases(w));
end
fprintf(fid,'Paper continuous nuT scales are not physical discrete thrust limits.\n');
for q=1:size(Q,1)
    fprintf(fid,'t=%.3f: best one-step hEnd improvement T only=%.9g m, torque only=%.9g m.\n', ...
        Q(q,1),Q(q,3),Q(q,5));
end
fclose(fid);
result=struct('observedLog',D,'nominalCounterfactual',N,'earlySensitivity',A, ...
    'weightSensitivity',W,'geometry',G,'spatial',S,'geometricMoves',H,'oneStepAuthority',Q, ...
    'kappa',kappa,'resultZip',fullfile(here,'CBF_ID3_EARLY_ONSET_RESULTS.zip'), ...
    'accuracy','LOG EXACT AS PRINTED; MODEL HYPOTHETICAL');
% ZIP in temporary folder then atomic-ish final replacement.
files={'early_actual_log.csv','early_nominal_counterfactual.csv', ...
    'early_anchor_sensitivity.csv','early_spatial_consistency.csv','early_geometric_moves.csv', ...
    'early_weight_effect.csv','early_geometry.csv', ...
    'early_one_step_authority.csv','SUMMARY.txt'};
zipTmp=fullfile(tmp,'result.zip');
zip(zipTmp,files,tmp);
[ok,msg]=movefile(zipTmp,result.resultZip,'f');
if ~ok,error('ID3Early:ZIPMove','%s',msg);end
fprintf('[ID3 EARLY] first intervention: 26.075 s; no 0 N linearization at onset.\n');
fprintf('[ID3 EARLY] four-stage kappa=%.6f, log 23 rows, model %d anchor rows.\n', ...
    kappa,size(A,1));
fprintf('[ID3 EARLY] ZIP only: %s\n',result.resultZip);
end

function [dh,ratio,major]=geometryGrad(x,P,parts,geom)
q=parts(1); R=eye(3); if isfield(q,'R'),R=q.R;end
if isfield(q,'params') && isfield(q.params,'axes')
    ax=q.params.axes(:);
else
    ax=q.axes(:);
end
Q=R*diag(ax.^2)*R.';
[V,D]=eig(Q);
[~,ind]=max(diag(D));major=V(:,ind);
ratio=sqrt(max(diag(D))/min(diag(D)));
dh=zeros(3,1);epsPos=1e-3;
for j=1:3
    xp=x;xm=x;
    xp(7+j)=xp(7+j)+epsPos;
    xm(7+j)=xm(7+j)-epsPos;
    dh(j)=(geomH(xp,P,parts,geom)-geomH(xm,P,parts,geom))/(2*epsPos);
end
end

function h=geomH(x,P,parts,geom)
part=parts(1);C=x(8:10)-0.5*P.cableL*x(14:16);
d=C-part.center(:);n=d/norm(d);
rCable=sqrt(geom.b^2+(geom.a^2-geom.b^2)*(n.'*x(14:16))^2);
if strcmp(char(part.kind),'sphere')
    if isfield(part,'params') && isfield(part.params,'radius')
        rObs=double(part.params.radius);
    else
        rObs=double(part.radius);
    end
else
    if isfield(part,'params') && isfield(part.params,'axes')
        ax=double(part.params.axes(:));
    else
        ax=double(part.axes(:));
    end
    R=eye(3);if isfield(part,'R'),R=double(part.R);end
    rObs=norm(diag(ax)*R.'*n);
end
h=norm(d)-rCable-rObs;
end

function writeCsv(file,headers,data)
fid=fopen(file,'wt');
if fid<0,error('ID3Early:CsvOpen','Cannot open %s',file);end
fprintf(fid,'%s\n',strjoin(headers,','));
for i=1:size(data,1)
    fprintf(fid,'%.16g',data(i,1));
    for j=2:size(data,2)
        fprintf(fid,',%.16g',data(i,j));
    end
    fprintf(fid,'\n');
end
fclose(fid);
end

function localRmdir(where)
if exist(where,'dir')==7
    try,rmdir(where,'s');catch,end
end
end
