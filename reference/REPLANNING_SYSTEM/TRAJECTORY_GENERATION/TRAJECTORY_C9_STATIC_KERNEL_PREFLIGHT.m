function R=TRAJECTORY_C9_STATIC_KERNEL_PREFLIGHT()
% Pure software preparation BEFORE Flight/Arming. No HLC, EKF, obstacle,
% reference, trajectory generation, simulation clock or sensing is touched.
% Only representation-constant degree-21 Gram and degrees 17..23 midpoint
% split maps (polynomial control counts 18..24) are constructed/cached.
% Call AFTER clear classes / rehash, BEFORE starting the Flight runner.
clock=tic;
R=struct('status','UNKNOWN_STATIC_PREFLIGHT_NOT_RUN', ...
    'ready',false,'safe_to_commit',false, ...
    'prepared_split_control_counts',18:24, ...
    'split_max_absolute_parity',NaN, ...
    'gram_21_parity_max_abs',NaN, ...
    'gram_cache_hit_on_second_call',false, ...
    'duration_ms',NaN,'error_message','');
try
    % The static Gram is rebuilt once during preflight and subsequently
    % reused by EXACTLY the production 13D QP module's local wrapper.
    [G,~]=TRAJECTORY_C9_13D_STATIC_GRAM(21,true);
    [G2,cache]=TRAJECTORY_C9_13D_STATIC_GRAM(21,true);
    assert(isequal(G,G2)&&cache.cache_hit, ...
        'C9_PREFLIGHT:GramCache','Degree-21 Gram cache not reusable.');
    R.gram_cache_hit_on_second_call=cache.cache_hit;
    % Warm the midpoint-split matrices of ALL physical C9 derivative
    % polygons r=0:6. No current/synthetic Flight trajectory is cached.
    largest=0;
    for n=18:24
        % A deterministic CODE TEST vector, never a planned trajectory.
        t=double(1:n);
        P=[t;t.^2;t.^3]/double(n^3);
        [L,Rr]=TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);
        [L2,R2]=TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);
        assert(isequal(L,L2)&&isequal(Rr,R2), ...
            'C9_PREFLIGHT:SplitCache','Midpoint map not reproducible.');
        % Compare against the de Casteljau algebra at u=1/2.
        left=zeros(size(P));right=zeros(size(P));V=P;
        left(:,1)=P(:,1);right(:,end)=P(:,end);
        for k=2:n
            V=0.5*(V(:,1:end-1)+V(:,2:end));
            left(:,k)=V(:,1);right(:,n-k+1)=V(:,end);
        end
        err=max([abs(L(:)-left(:));abs(Rr(:)-right(:))]);
        largest=max(largest,err);
        scale=max(1,max(abs(P(:))));
        assert(err<=128*n*eps(scale), ...
            'C9_PREFLIGHT:SplitParity','Static midpoint split parity failed.');
    end
    R.split_max_absolute_parity=largest;
    R.gram_21_parity_max_abs=max(abs(G(:)-G2(:)));
    R.ready=true;
    R.status='PASS_STATIC_KERNEL_PREPARED__NOT_A_FLIGHT_CERTIFICATE';
catch ME
    R.status=['UNKNOWN_STATIC_PREFLIGHT_' ME.identifier];
    R.error_message=ME.message;
end
R.duration_ms=1000*toc(clock);
end
