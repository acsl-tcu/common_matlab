function R=CHECK_TRIGGER_BATCH_SHAPE_NEUTRAL_REACH()
% COMMON ROOT disposable conditional geometry test; no Flight is run.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_BATCH_SHAPE_NEUTRAL_REACH', ...
  'cases',0,'failures',0,'multiple_ids',0, ...
  'missing_outer_refused',0,'real_flight_proved',false,'reason','');
try
    t=[0 .025 .05 .075];K=numel(t)-1;
    F=struct('conditional_chain_pass',true, ...
        'conditional_tension_positive_all_intervals',true, ...
        'manifold_compatible_intervals',true(1,K), ...
        'completed_intervals',K, ...
        'tube_lower',zeros(24,K),'tube_upper',zeros(24,K));
    % Three true full-time endpoint boxes: interval 1 entirely far;
    % interval 2 near two obstacle centers simultaneously; interval 3 far.
    F.tube_lower(1:3,:)=repmat([10;0;0],1,K);
    F.tube_upper(1:3,:)=F.tube_lower(1:3,:);
    F.tube_lower(13:15,:)=repmat([10;0;-2],1,K);
    F.tube_upper(13:15,:)=F.tube_lower(13:15,:);
    F.tube_lower(1,2)=0;F.tube_upper(1,2)=0;
    F.tube_lower(13,2)=0;F.tube_upper(13,2)=0;
    % Arbitrary shape labels; the proof uses only independently certified
    % physical occupied outer-ball bounds, not type-specific approximations.
    ids=[18 19 88];
    S=repmat(struct('id',0,'type','unknown_nonconvex', ...
      'center0',zeros(3,1),'bounding_radius_m',0.2, ...
      'is_stationary',true,'rigorous_outer_ball_certified',true),1,3);
    for j=1:3, S(j).id=ids(j); end
    S(1).center0=[0;0;0];S(2).center0=[0;0;-2];
    S(3).center0=[-50;0;0];
    body=struct('r_uav_m',0.1,'r_payload_m',0.1,'r_cable_m',0.02);
    C=struct('flowpipe',F,'time_grid_s',t,'observed_ids',ids, ...
        'observed_prepared_shapes',S,'body',body);
    W=TRIGGER_BATCH_SHAPE_NEUTRAL_REACH(C);
    assert(isequal(size(W.separated_intervals),[3 K]) && ...
        all(W.separated_intervals(:,1)) && ...
        ~W.separated_intervals(1,2) && ...
        ~W.separated_intervals(2,2) && ...
        all(W.separated_intervals(:,3)) && ...
        isequal(W.earliest_possible_event.earliest_possible_ids,[18 19]) && ...
        W.earliest_possible_event.first_interval_index==2 && ...
        ~W.safe_to_commit, ...
        'CHECK_TRIGGER_BATCH:MultipleIDs',W.reason);
    R.cases=R.cases+1;R.multiple_ids=2;
    % Test singleton dimensions and arbitrary K; no squeeze ambiguity.
    for j=1:120
        K=1+mod(j,13);M=1+mod(j,17);
        F.tube_lower=zeros(24,K);F.tube_upper=zeros(24,K);
        F.tube_lower(1:3,:)=20;F.tube_upper(1:3,:)=20;
        F.tube_lower(13:15,:)=21;F.tube_upper(13:15,:)=21;
        F.manifold_compatible_intervals=true(1,K);
        F.completed_intervals=K;
        ids=1:M;
        S=repmat(S(1),1,M);
        for v=1:M,S(v).id=v;S(v).center0=zeros(3,1);end
        C.flowpipe=F;C.time_grid_s=0:K;C.observed_ids=ids;
        C.observed_prepared_shapes=S;
        W=TRIGGER_BATCH_SHAPE_NEUTRAL_REACH(C);
        assert(isequal(size(W.separated_intervals),[M K]) && ...
            all(W.separated_intervals(:)) && ...
            W.conditional_all_intervals_separated && ~W.safe_to_commit, ...
            'CHECK_TRIGGER_BATCH:FarBox',W.reason);
        R.cases=R.cases+1;
    end
    % Missing rigorous outer enclosure must never establish free space.
    S(1).rigorous_outer_ball_certified=false;
    C.observed_prepared_shapes=S;
    W=TRIGGER_BATCH_SHAPE_NEUTRAL_REACH(C);
    assert(~W.conditional_all_intervals_separated && ...
        all(~W.separated_intervals(1,:)) && ...
        ismember(ids(1),W.possible_risk_ids), ...
        'CHECK_TRIGGER_BATCH:MissingProof','Missing certified outer ball falsely separated.');
    R.cases=R.cases+1;R.missing_outer_refused=1;
    R.status='PASS_BATCH_ALL_TIME_SHAPE_NEUTRAL__EXTERNAL_PHYSICS_MISSING';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_BATCH_SHAPE_NEUTRAL_REACH_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_BATCH_SHAPE_NEUTRAL_REACH_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nfailures=%d\nmultiple_ids=%d\nmissing_outer_refused=%d\nreason=%s\n', ...
        R.status,R.cases,R.failures,R.multiple_ids,R.missing_outer_refused,R.reason);
    fclose(f);
end
end
