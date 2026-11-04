function A=CHECK_TRIGGER_ENDPOINT_LIPSCHITZ_GATE()
% Tests sufficiency and the fail-closed behavior; does not test live EKF.
A=struct('status','FAIL','reason','','cases',0);
try
    f={'true_endpoint_speed_bounds','true_sample_clearance_lower_bounds', ...
       'fixed_body_radii','straight_taut_cable','static_observed_obstacles', ...
       'estimation_and_tracking_errors_included'};
    a=cell2struct(num2cell(true(1,numel(f))),f,2);
    % Equivalent true speed of the entire cable is the maximum endpoint speed.
    R=TRIGGER_ENDPOINT_LIPSCHITZ_GATE([0 1],[0.7 0.6], ...
                                     1,0.8,7,a);
    assert(R.conditional_interval_clear && ...
           strcmp(R.status,'CONDITIONAL_FULL_INTERVAL_CLEAR__NOT_A_FLIGHT_CERTIFICATE') && ...
           ~R.safe_to_commit,'CHECK_ENDPOINT_GATE:Clear');
    A.cases=A.cases+1;
    % Equal threshold: no margin => cannot prove clearance.
    R=TRIGGER_ENDPOINT_LIPSCHITZ_GATE([0 1],[0.4 0.6],1,0.8,7,a);
    assert(~R.conditional_interval_clear && ...
           isequal(R.unresolved_obstacle_ids,7), ...
           'CHECK_ENDPOINT_GATE:Tangent');
    A.cases=A.cases+1;
    % Arbitrary observed IDs and multiple non-uniform interval durations.
    D=[2.0 2.0 2.0;0.10 0.10 1.0];
    R=TRIGGER_ENDPOINT_LIPSCHITZ_GATE([0 0.4 1.5],D, ...
                                     [0.5 0.5],[0.7 0.4],[5 22],a);
    assert(~R.conditional_interval_clear && ...
           isequal(R.unresolved_obstacle_ids,22) && ...
           R.first_unresolved_left_s==0,'CHECK_ENDPOINT_GATE:Multi');
    A.cases=A.cases+1;
    % Missing guaranteed tracking-error provenance cannot be declared clear.
    a.estimation_and_tracking_errors_included=false;
    R=TRIGGER_ENDPOINT_LIPSCHITZ_GATE([0 1],[100 100],0,0,8,a);
    assert(~R.conditional_interval_clear && ~R.safe_to_commit && ...
            strcmp(R.status,'UNKNOWN_INPUT_OR_PROVENANCE'), ...
            'CHECK_ENDPOINT_GATE:Provenance');
    A.cases=A.cases+1;
    A.status='PASS_CONDITIONAL_LIPSCHITZ_THEOREM__NOT_FLIGHT_SAFETY';
catch ME
    A.reason=sprintf('%s: %s',ME.identifier,ME.message);
end
fid=fopen('TRIGGER_ENDPOINT_LIPSCHITZ_GATE_RESULT.txt','w');
if fid>=0
    clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\nreason=%s\ncases=%d\n', ...
       A.status,A.reason,A.cases);
end
end
