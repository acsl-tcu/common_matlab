function R=CHECK_TRIGGER_TRUE_SHAPE_FLOWPIPE_BRIDGE()
% Disposable conditional-physics shape-neutral flowpipe regression.
% Write reports into common_matlab root. No synthetic Flight is executed.
% These tests do NOT prove real EKF/HLC/mass/hybrid/rounding enclosures.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_TRUE_SHAPE_FLOWPIPE_BRIDGE', ...
    'cases',0,'failures',0,'shape_families',0,'multiple_risk_ids',0, ...
    'nonconvex_hole_clear',false,'unknown_shape_fails_closed',false, ...
    'safe_to_commit',false,'reason','');
try
    stage='far_shape';
    types={'sphere','box','cylinder','ellipsoid','capsule', ...
        'torus','cone','frustum','regular_prism','polygon_prism'};
    for k=1:numel(types)
        typ=types{k};
        aa=0.31*k;bb=0.19*k;
        Rz=[cos(aa) -sin(aa) 0;sin(aa) cos(aa) 0;0 0 1];
        Ry=[cos(bb) 0 sin(bb);0 1 0;-sin(bb) 0 cos(bb)];
        s=local_shape(typ,Rz*Ry,[20;0;0],100+k);
        F=local_pipe([0;0;1],[0;0;-1],2);
        B=struct('flowpipe',F,'time_grid_s',[0 2 4], ...
            'observed_prepared_shapes',s,'observed_ids',s.id, ...
            'body',local_body());
        Z=TRIGGER_TAUT_TRUE_SHAPE_FLOWPIPE_GATE(B);
        assert(Z.conditional_geometry_pass && ...
            all(Z.separated_intervals(:)) && ...
            ~Z.safe_to_commit && ...
            ~Z.continuous_closed_loop_safe_certified, ...
            'CHECK_TRIGGER_FLOWPIPE:FarShape', ...
            'Whole-time far-shape witness failed for %s.',typ);
        R.cases=R.cases+1;R.shape_families=R.shape_families+1;
    end
    % A torus hole has no suitable plane through the convex hull; the
    % finite straight cable passes through the actual vacant hole.
    stage='torus_hole';
    s=local_shape('torus',eye(3),[0;0;0],41);
    F=local_pipe([0;0;1],[0;0;-1],2);
    B=struct('flowpipe',F,'time_grid_s',[0 2 4], ...
        'observed_prepared_shapes',s,'observed_ids',s.id, ...
        'body',local_body());
    Z=TRIGGER_TAUT_TRUE_SHAPE_FLOWPIPE_GATE(B);
    assert(Z.conditional_geometry_pass && ~Z.safe_to_commit, ...
        'CHECK_TRIGGER_FLOWPIPE:TorusHole', ...
        'True shape torus hole should admit a conditional tube.');
    R.nonconvex_hole_clear=true;R.cases=R.cases+1;
    % Multiple simultaneous ID risks. A far first obstacle must NOT
    % suppress the two real observed near solids.
    stage='multiple_simultaneous_ids';
    far=local_shape('sphere',eye(3),[20;0;0],17);
    near1=local_shape('sphere',eye(3),[0;0;0],18);
    near2=local_shape('sphere',eye(3),[0;0;-0.3],19);
    S=[far near1 near2];
    B.observed_prepared_shapes=S;
    B.observed_ids=[S.id];
    Z=TRIGGER_TAUT_TRUE_SHAPE_FLOWPIPE_GATE(B);
    % A 3x2 logical witness MUST be reduced to a scalar before &&.
    % Prior regression accidentally used all(~mask(2:3,:)), which yields
    % a 1x2 logical row and fails the MATLAB short-circuit contract.
    expected_mask=logical([1 1;0 0;0 0]);
    ids_ok=isequal(Z.possible_risk_ids,[18 19]);
    mask_ok=isequal(Z.separated_intervals,expected_mask);
    event_ok=isstruct(Z.earliest_possible_event) && ...
        isscalar(Z.earliest_possible_event) && ...
        all(isfield(Z.earliest_possible_event, ...
            {'earliest_possible_ids','first_interval_index'}));
    if event_ok
        event_ok=isequal(Z.earliest_possible_event.earliest_possible_ids, ...
            [18 19]) && ...
            isequal(Z.earliest_possible_event.first_interval_index,1);
    end
    R.multiple_risk_ids=numel(Z.possible_risk_ids);
    assert(~Z.conditional_geometry_pass && ids_ok && ...
        mask_ok && event_ok && ~Z.safe_to_commit, ...
        'CHECK_TRIGGER_FLOWPIPE:MultipleIDs', ...
        ['Earliest simultaneous IDs are inconsistent. ' ...
         'status=%s; ids=%s; separated=%s; reason=%s'], ...
        Z.status,mat2str(Z.possible_risk_ids), ...
        mat2str(double(Z.separated_intervals)),Z.reason);
    R.cases=R.cases+1;
    % An unknown new shape must never be cleared by the known-shape oracle.
    stage='unsupported_shape';
    bad=local_shape('sphere',eye(3),[20;0;0],22);
    bad.type='not_a_supported_distance_geometry';
    B.observed_prepared_shapes=bad;
    B.observed_ids=bad.id;
    Z=TRIGGER_TAUT_TRUE_SHAPE_FLOWPIPE_GATE(B);
    assert(~Z.conditional_geometry_pass && ~Z.safe_to_commit, ...
        'CHECK_TRIGGER_FLOWPIPE:UnknownShape', ...
        'Unsupported shape cannot produce safety clearance.');
    R.unknown_shape_fails_closed=true;R.cases=R.cases+1;
    % Arbitrary observed obstacle count; no static 3-ID or detection-mask
    % precomputation may be assumed by the conditional geometry evaluator.
    stage='dynamic_obstacle_count';
    S=repmat(far,1,17);
    for j=1:numel(S)
        S(j).id=300+j;
        S(j).center0=[20+j;0;0];
        S(j).center_norm_m=norm(S(j).center0);
    end
    B.observed_prepared_shapes=S;B.observed_ids=[S.id];
    Z=TRIGGER_TAUT_TRUE_SHAPE_FLOWPIPE_GATE(B);
    assert(Z.conditional_geometry_pass && Z.observed_count==17 && ...
        isequal(size(Z.separated_intervals),[17 2]), ...
        'CHECK_TRIGGER_FLOWPIPE:DynamicCount', ...
        'Shape/interval dimensions cannot assume a fixed count.');
    R.cases=R.cases+1;
    % Invalid/NaN tubes, lost positive tension, missing observations and
    % deliberately inconsistent IDs must never yield a positive pass.
    stage='fail_closed_inputs';
    B.observed_prepared_shapes=far;B.observed_ids=far.id;
    for j=1:5
        X=B;
        switch j
            case 1, X.flowpipe.tube_upper(1,1)=NaN;
            case 2, X.flowpipe.conditional_tension_positive_all_intervals=false;
            case 3, X.flowpipe.manifold_compatible_intervals(1)=false;
            case 4, X.observed_ids=1234;
            case 5, X.observed_prepared_shapes=far([]);X.observed_ids=[];
        end
        Y=TRIGGER_TAUT_TRUE_SHAPE_FLOWPIPE_GATE(X);
        assert(~Y.conditional_geometry_pass && ~Y.safe_to_commit, ...
            'CHECK_TRIGGER_FLOWPIPE:FailClosed', ...
            'Invalid case %d granted a conditional clearance.',j);
        R.cases=R.cases+1;
    end
    R.status='PASS_TRUE_SHAPE_FULL_TIME_BRIDGE__CONDITIONAL_NOT_FLIGHT_PROOF';
catch ME
    R.failures=R.failures+1;
    R.reason=['stage=' stage ' | ' ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_TRUE_SHAPE_FLOWPIPE_BRIDGE_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_TRUE_SHAPE_FLOWPIPE_BRIDGE_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nfailures=%d\n', ...
        R.status,R.cases,R.failures);
    fprintf(f,'shape_families=%d\nmultiple_risk_ids=%d\n', ...
        R.shape_families,R.multiple_risk_ids);
    fprintf(f,'nonconvex_hole_clear=%d\nunknown_shape_fails_closed=%d\n', ...
        R.nonconvex_hole_clear,R.unknown_shape_fails_closed);
    fprintf(f,'safe_to_commit=%d\nreason=%s\n',R.safe_to_commit,R.reason);
    fclose(f);
end
end
function F=local_pipe(q,l,K)
F=struct('conditional_chain_pass',true,'completed_intervals',K, ...
    'conditional_tension_positive_all_intervals',true, ...
    'manifold_compatible_intervals',true(1,K), ...
    'tube_lower',zeros(24,K),'tube_upper',zeros(24,K));
F.tube_lower(1:3,:)=repmat(q,1,K);
F.tube_upper(1:3,:)=repmat(q,1,K);
F.tube_lower(13:15,:)=repmat(l,1,K);
F.tube_upper(13:15,:)=repmat(l,1,K);
end
function b=local_body()
b=struct('r_uav_m',0.15,'r_payload_m',0.15,'r_cable_m',0.02);
end
function s=local_shape(type,R,c,id)
s=struct('id',double(id),'type',type,'dimensions',[], ...
    'center0',double(c(:)),'rotation',double(R), ...
    'velocity',zeros(3,1),'reference_time_s',0, ...
    'bounding_radius_m',NaN,'is_stationary',true, ...
    'center_norm_m',norm(c),'outline',zeros(0,2), ...
    'vertices',zeros(0,3),'faces',zeros(0,3));
switch type
    case 'sphere'
        s.dimensions=1;s.bounding_radius_m=1;
    case 'ellipsoid'
        s.dimensions=[0.5;8;0.6];s.bounding_radius_m=8;
    case 'box'
        s.dimensions=[1;7;2];s.bounding_radius_m=norm(s.dimensions)/2;
    case 'cylinder'
        s.dimensions=[1;6];s.bounding_radius_m=hypot(1,3);
    case 'capsule'
        s.dimensions=[0.8;5];s.bounding_radius_m=3.3;
    case 'cone'
        s.dimensions=[1.2;6];s.bounding_radius_m=hypot(1.2,3);
    case 'frustum'
        s.dimensions=[1.2;0.3;6];s.bounding_radius_m=hypot(1.2,3);
    case 'torus'
        s.dimensions=[4;0.7];s.bounding_radius_m=4.7;
    case 'regular_prism'
        s.dimensions=[6;1.2;6];n=6;th=(0:n-1)'*2*pi/n;
        s.outline=1.2*[cos(th),sin(th)];s.bounding_radius_m=hypot(1.2,3);
    case 'polygon_prism'
        s.outline=[-1 -1;2 -1;2 0;0 0;0 1;-1 1];
        s.dimensions=struct('outline',s.outline,'height',4);
        s.bounding_radius_m=hypot(max(sqrt(sum(s.outline.^2,2))),2);
end
end
