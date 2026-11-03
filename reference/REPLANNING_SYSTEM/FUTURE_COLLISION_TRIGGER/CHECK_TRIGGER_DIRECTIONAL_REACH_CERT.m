function T=CHECK_TRIGGER_DIRECTIONAL_REACH_CERT()
% Isolated conditional-theorem fixtures. NOT real closed-loop safety.
T=struct('status','FAIL_DIRECTIONAL_CONDITIONAL_TESTS', ...
    'pass',false,'checks',0,'reason','');
try
    c=cos(0.7);s=sin(0.7);
    R=[c -s 0;s c 0;0 0 1];
    obs=[OBSTACLE_INFORMATION.define(11,'ellipsoid',[0;0;0], ...
        [1.2;0.6;1.0],'rotation',R), ...
        OBSTACLE_INFORMATION.define(44,'sphere',[9;0;0],0.4)];
    shapes=OBSTACLE_GEOMETRY_QUERY.prepare(obs);
    body=struct('r_uav_m',0.12,'r_payload_m',0.15,'r_cable_m',0.025);
    base=struct('time_grid_s',[0 0.25 0.50], ...
        'uav_lower_m',repmat([4;0;0],1,2), ...
        'uav_upper_m',repmat([4.1;0.1;0.1],1,2), ...
        'payload_lower_m',repmat([4;0;-2],1,2), ...
        'payload_upper_m',repmat([4.1;0.1;-1.9],1,2));
    X=TRIGGER_DIRECTIONAL_REACH_CERT(base,shapes,body);
    assert(X.conditional_geometry_pass && ...
        all(X.conditional_separated_intervals(:)) && ...
        ~X.safe_to_commit && ~X.continuous_closed_loop_safe_certified);
    T.checks=T.checks+1;
    % Cable runs THROUGH obstacle; endpoints are outside.
    base.uav_lower_m=repmat([0;0;2],1,2);
    base.uav_upper_m=base.uav_lower_m;
    base.payload_lower_m=repmat([0;0;-2],1,2);
    base.payload_upper_m=base.payload_lower_m;
    X=TRIGGER_DIRECTIONAL_REACH_CERT(base,shapes(1),body);
    assert(~X.conditional_geometry_pass && ...
        isequal(X.possibly_at_risk_ids,11) && ~X.safe_to_commit);
    T.checks=T.checks+1;
    % A wide continuous reach tube must NOT be deemed safe simply because
    % the nominal endpoint samples are outside the obstacle.
    base.uav_lower_m=repmat([-2;0;-2],1,2);
    base.uav_upper_m=repmat([2;0;2],1,2);
    base.payload_lower_m=base.uav_lower_m;
    base.payload_upper_m=base.uav_upper_m;
    X=TRIGGER_DIRECTIONAL_REACH_CERT(base,shapes(1),body);
    assert(~X.conditional_geometry_pass && ~X.safe_to_commit);
    T.checks=T.checks+1;
    % Missing/invalid all-time enclosures cannot silently create SAFE.
    base.uav_upper_m(1,1)=NaN;
    X=TRIGGER_DIRECTIONAL_REACH_CERT(base,shapes(1),body);
    assert(startsWith(X.status,'UNKNOWN') && ~X.safe_to_commit);
    T.checks=T.checks+1;
    % Exact support of a rotated ellipsoid (NOT a bounding sphere).
    n=[0.31;-0.54;0.73];n=n/norm(n);
    expected=norm([1.2;0.6;1.0].*(R'*n));
    observed=TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many( ...
        shapes(1),n);
    assert(abs(observed-expected)<1e-12);
    T.checks=T.checks+1;
    T.pass=true;
    T.status='PASS_DIRECTIONAL_CONDITIONAL_THEOREM_FIXTURES__NOT_FLIGHT_SAFETY';
catch ME
    T.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_DIRECTIONAL_REACH_AUDIT_RESULT.mat','T');
fid=fopen('TRIGGER_DIRECTIONAL_REACH_AUDIT_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nchecks=%d\nreason=%s\n', ...
        T.status,T.checks,T.reason);fclose(fid);
end
end
