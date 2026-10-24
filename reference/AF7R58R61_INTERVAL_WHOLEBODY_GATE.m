function OUT=AF7R58R61_INTERVAL_WHOLEBODY_GATE(IN)
% Optional proof-structure gate for a supplied HLC/plant 4-s model rollout.
% This function does NOT compute the rollout. It requires external verified
% trajectory and continuous-time speed BOUNDS for each time cell. The bounds
% must be derived from live HLC/plant and current EKF state, not guessed from
% sampled finite differences. No robust tracking/observer-error tube is used.
% Supports only sensor-known static oriented ellipsoids.
% Even passing means CONDITIONAL mathematically in exact real arithmetic;
% binary64 directed rounding, actual plant rollout and execution proof remain open.
required={'time_s','pQ','pL','q','cable_length','radii','obstacles', ...
          'speed_ub_Q','speed_ub_L','speed_bounds_certified','prediction_horizon_s'};
for i=1:numel(required)
    assert(isstruct(IN)&&isfield(IN,required{i}),'R61:Input','Missing %s',required{i});
end
t=double(IN.time_s(:)');n=numel(t);
Q=double(IN.pQ);L=double(IN.pL);q=double(IN.q);cl=double(IN.cable_length);
r=double(IN.radii(:));vQ=double(IN.speed_ub_Q(:)');vL=double(IN.speed_ub_L(:)');
assert(n>=2&&all(isfinite(t))&&all(diff(t)>0) && ...
    abs(t(end)-t(1)-double(IN.prediction_horizon_s))<= ...
        64*eps(max([1,abs(t(end)),abs(double(IN.prediction_horizon_s))])) && ...
    isequal(size(Q),[3 n])&&isequal(size(L),[3 n])&&isequal(size(q),[3 n]) && ...
    all(isfinite([Q(:);L(:);q(:)]))&&numel(r)==3&&all(r>=0)&& ...
    numel(vQ)==n-1&&numel(vL)==n-1&&all(isfinite([vQ,vL])) && ...
    all(vQ>=0)&&all(vL>=0)&&cl>0,'R61:Geometry','Invalid time/geometry or speed UB');
% Fail closed on any incorrect straight-cable state. Tolerance is an explicit
% input: no fallback physical scale or error margin.
assert(isfield(IN,'state_consistency_tolerance_m') && ...
    isfinite(IN.state_consistency_tolerance_m) && IN.state_consistency_tolerance_m>=0, ...
    'R61:Consistency','Explicit state consistency tolerance required');
tol=double(IN.state_consistency_tolerance_m);
assert(all(vecnorm(L-Q-cl*q)<=tol) && ...
    all(abs(vecnorm(q)-1)<=tol/max(1,cl)), ...
    'R61:Consistency','Inconsistent HLC/plant Q/L/q state');
obs=IN.obstacles;assert(isstruct(obs),'R61:Obstacles','Only sensor-known obstacle structs');
OUT=struct('status','UNKNOWN_NO_CERTIFIED_MODEL_ROLLOUT_BOUNDS','commit_allowed',false, ...
    'horizon_s',t(end)-t(1),'obstacle_count',numel(obs), ...
    'wholebody_geometric_cells_tested',0,'smallest_numerical_margin_m',NaN, ...
    'conditional_continuous_model_clearance',false,'unknown_cells',0, ...
    'robust_error_tube_installed',false,'outward_rounding_closed',false);
if ~logical(IN.speed_bounds_certified)
    OUT.unknown_cells=(n-1)*numel(obs);return;
end
minMargin=Inf;unknown=0;
for i=1:numel(obs)
    O=obs(i);
    assert(all(isfield(O,{'center','rotation','axes'})), 'R61:Obstacle','Missing ellipsoid fields');
    c=double(O.center(:));R=double(O.rotation);a=double(O.axes(:));
    assert(isequal(size(R),[3 3])&&numel(c)==3&&numel(a)==3&& ...
        all(isfinite([c;a;R(:)]))&&all(a>0)&& ...
        norm(R'*R-eye(3),'fro')<=1e-8,'R61:Obstacle','Invalid oriented ellipsoid');
    for k=1:n-1
        h=t(k+1)-t(k);
        % Potential separator normals only. A failure to find one is UNKNOWN.
        X=[Q(:,k),Q(:,k+1),L(:,k),L(:,k+1)];
        cc=mean(X,2);
        N=[cc-c,X-c,R(:,1:3),-R(:,1:3)];
        best=-Inf;
        for j=1:size(N,2)
            w=N(:,j);nw=norm(w);if nw<=eps(max(1,norm(c))),continue;end
            w=w/nw;
            support=w'*c+norm(a.*(R'*w));
            bQ=min(w'*Q(:,k:k+1))-r(1)-vQ(k)*h/2;
            bL=min(w'*L(:,k:k+1))-r(2)-vL(k)*h/2;
            bC=min(w'*X)-r(3)-max(vQ(k),vL(k))*h/2;
            margin=min([bQ,bL,bC])-support;
            best=max(best,margin);
        end
        minMargin=min(minMargin,best);
        OUT.wholebody_geometric_cells_tested=OUT.wholebody_geometric_cells_tested+1;
        if ~(best>0),unknown=unknown+1;end
    end
end
OUT.unknown_cells=unknown;
if isfinite(minMargin),OUT.smallest_numerical_margin_m=minMargin;end
if unknown==0
    OUT.status='CONDITIONAL_WHOLEBODY_MODEL_CLEARANCE__NOT_ROBUST_OR_ROUNDED';
    OUT.conditional_continuous_model_clearance=true;
else
    OUT.status='UNKNOWN_POSSIBLE_COLLISION_OR_INSUFFICIENT_SEPARATORS';
end
end
