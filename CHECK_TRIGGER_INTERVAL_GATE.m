function T=CHECK_TRIGGER_INTERVAL_GATE()
% Synthetic theorem regression only. NEVER touches Flight/EKF/sensor state.
% This does not establish real flight safety.
folder=fileparts(mfilename('fullpath'));
try
    o1=OBSTACLE_INFORMATION.define(41,'sphere',[10 0 0],1);
    o2=OBSTACLE_INFORMATION.define(73,'sphere',[0 0 0],0.4);
    sh=OBSTACLE_GEOMETRY_QUERY.prepare([o1 o2]);
    grid=[0 0.025];
    A=TRIGGER_CONTINUOUS_INTERVAL_GATE.evaluate( ...
        [-2 2;0 0;0 0],[-2 2;0 0;-2 -2],grid,sh,0.5,0.5,0,[4],[4]);
    % Position endpoints appear clear at t=0, but a full temporal sweep
    % can pass through obstacle 73. The certified excursion must flag it.
    assert(~A.conditionally_clear && ismember(73,A.possibly_at_risk_ids));
    assert(~ismember(41,A.possibly_at_risk_ids));
    B=TRIGGER_CONTINUOUS_INTERVAL_GATE.evaluate( ...
        [-2 -2;0 0;0 0],[-2 -2;0 0;-2 -2],grid,sh(1),0.5,0.5,0,[0.1],[0.1]);
    assert(B.conditionally_clear);
    % A cable crossing at the LEFT endpoint cannot become safe merely
    % because both spherical body endpoints are clear.
    C=TRIGGER_CONTINUOUS_INTERVAL_GATE.evaluate( ...
        [0 0;0 0;2 2],[0 0;0 0;-2 -2],grid,sh(2),0.5,0.5,0,[0.01],[0.01]);
    assert(~C.conditionally_clear);
    T=struct('status','PASS_CONDITIONAL_SPATIOTEMPORAL_THEOREM_TEST',...
        'crossing_risk_detected',true,'distant_separation_detected',true,...
        'full_cable_checked',true,'real_flight_tested',false,...
        'continuous_closed_loop_safe_certified',false);
catch ME
    T=struct('status','FAIL_INTERVAL_THEOREM_TEST',...
        'error',[ME.identifier ' | ' ME.message],...
        'real_flight_tested',false,'continuous_closed_loop_safe_certified',false);
end
save(fullfile(folder,'TRIGGER_INTERVAL_GATE_RESULT.mat'),'T','-v7');
fid=fopen(fullfile(folder,'TRIGGER_INTERVAL_GATE_RESULT.txt'),'w');
if fid>=0
    f=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\n',T.status);
    if isfield(T,'error'),fprintf(fid,'error=%s\n',T.error);end
    fprintf(fid,'real_flight_tested=0\ncontinuous_safe=0\n');
end
end
