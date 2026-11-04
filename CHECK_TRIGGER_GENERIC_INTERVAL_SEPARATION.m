function R=CHECK_TRIGGER_GENERIC_INTERVAL_SEPARATION()
% Synthetic mathematical regression; no flight or sensor world is accessed.
% Tests ALL shape families via a single support-based interface, not just
% cylinders or an obstacle configuration from the user's actual Flight.
R=struct('status','NOT_COMPLETED','reason','', ...
    'tested_types',{{}},'rotated_types',0,'far_clear',false, ...
    'contact_unknown',false,'provenance_rejected',false, ...
    'no_observation_unknown',false,'noncommit',false);
try
    theta=0.71;
    Q=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
    types={'sphere','ellipsoid','box','cylinder','capsule', ...
        'cone','frustum','torus'};
    dims={0.3,[0.5 0.3 0.4],[0.6 0.4 0.8], ...
        [0.3 0.8],[0.2 0.6],[0.4 0.6],[0.3 0.4 0.6],[0.5 0.1]};
    shapes=repmat(struct('id',0,'center0',zeros(3,1), ...
        'rotation',eye(3),'type','sphere','dimensions',0.3, ...
        'is_stationary',true,'bounding_radius_m',2),1,numel(types));
    for j=1:numel(types)
        shapes(j).id=100+j;
        shapes(j).type=types{j};
        shapes(j).dimensions=dims{j};
        ang=0.17*j;
        shapes(j).rotation=Q*[cos(ang) 0 sin(ang);0 1 0;-sin(ang) 0 cos(ang)];
        shapes(j).center0=[10+3*j;sin(j);0.25*j];
    end
    T=struct('q_lo',zeros(3,2),'q_hi',zeros(3,2), ...
        'axis',repmat([0;0;-1],1,2), ...
        'half_angle_rad',[0 0.1],'L_lo',[2 1.9], ...
        'L_hi',[2 2.1],'r_uav',0.35,'r_cable',0.02, ...
        'r_payload',0.25,'interval_bounds_s',[0 2;2 4], ...
        'horizon_s',4);
    O=struct('shapes',shapes,'observed_ids',[shapes.id], ...
        'verified_observation',true);
    A=TRIGGER_GENERIC_INTERVAL_SEPARATION.evaluate(T,O);
    assert(A.conditional_observed_geometry_clear && ...
        isequal(size(A.component_clear),[3 numel(types) 2]) && ...
        A.obstacles_checked==numel(types) && ~A.safe_to_commit, ...
        'CHECK_TRIGGER_GENERIC_INTERVAL_SEPARATION:Far');
    % Vary the observed obstacle COUNT and order without specializing code.
    for indices={[1],[2 5 7],[8 3 6 4 1 2 5 7]}
        jj=indices{1};
        P=struct('shapes',shapes(jj),'observed_ids',[shapes(jj).id], ...
            'verified_observation',true);
        Z=TRIGGER_GENERIC_INTERVAL_SEPARATION.evaluate(T,P);
        assert(Z.conditional_observed_geometry_clear && ...
            Z.obstacles_checked==numel(jj), ...
            'CHECK_TRIGGER_GENERIC_INTERVAL_SEPARATION:ArbitraryCount');
    end
    R.tested_types=types;R.rotated_types=numel(types);R.far_clear=true;
    % An obstacle centered ON the UAV must not be called separated.
    O.shapes=shapes(1);O.shapes(1).center0=zeros(3,1);
    O.observed_ids=O.shapes.id;
    B=TRIGGER_GENERIC_INTERVAL_SEPARATION.evaluate(T,O);
    assert(~B.conditional_observed_geometry_clear && ~B.safe_to_commit, ...
        'CHECK_TRIGGER_GENERIC_INTERVAL_SEPARATION:Contact');
    R.contact_unknown=true;
    O.verified_observation=false;
    C=TRIGGER_GENERIC_INTERVAL_SEPARATION.evaluate(T,O);
    assert(~C.conditional_observed_geometry_clear && ~C.safe_to_commit, ...
        'CHECK_TRIGGER_GENERIC_INTERVAL_SEPARATION:Provenance');
    R.provenance_rejected=true;
    O.verified_observation=true;
    O.shapes=shapes([]);O.observed_ids=zeros(1,0);
    D=TRIGGER_GENERIC_INTERVAL_SEPARATION.evaluate(T,O);
    assert(~D.conditional_observed_geometry_clear && ~D.safe_to_commit, ...
        'CHECK_TRIGGER_GENERIC_INTERVAL_SEPARATION:EmptySet');
    R.no_observation_unknown=true;
    R.noncommit=true;
    R.status='PASS_MULTI_SHAPE_WHOLE_INTERVAL_CONDITIONAL__NOT_FLIGHT_SAFETY';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    fid=fopen('TRIGGER_GENERIC_INTERVAL_SEPARATION_RESULT.txt','w');
    if fid>0
        fprintf(fid,'status=%s\n',R.status);
        fprintf(fid,'types=%s\n',strjoin(R.tested_types,','));
        fprintf(fid,'rotated_types=%d\n',R.rotated_types);
        fprintf(fid,'far_clear=%d contact_unknown=%d provenance_rejected=%d no_observation_unknown=%d\n', ...
            R.far_clear,R.contact_unknown,R.provenance_rejected,R.no_observation_unknown);
        fprintf(fid,'reason=%s\n',R.reason);fclose(fid);
    end
catch
end
end
