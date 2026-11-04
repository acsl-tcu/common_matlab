function R=TRIGGER_BATCH_SHAPE_NEUTRAL_REACH(C)
% Shape-neutral vectorized conditional witness for ALL TIME intervals.
% A single original 24-state full-time tube is screened against an arbitrary
% number of already observed/retained STATIC bodies via PROVED outer balls.
% The tube of the entire straight taut cable uses each endpoint's coordinate
% hull, not a spherical swing envelope. O(M*K) vectorized, no shape switch.
%
% This is only a sufficient separator. Unknown shape enclosure, missing
% provenance, invalid interval, failure of taut/tension verification, or
% arithmetic overflow yields UNKNOWN. NEVER a Flight safety/commit permit.
R=struct('status','UNKNOWN_BATCH_SHAPE_NEUTRAL_REACH', ...
    'reason','','safe_to_commit',false,'true_flight_certified',false, ...
    'conditional_all_intervals_separated',false, ...
    'separated_intervals',false(0,0),'earliest_possible_event',struct(), ...
    'possible_risk_ids',zeros(1,0),'shape_count',0,'interval_count',0, ...
    'verified_outer_count',0);
try
    assert(isstruct(C)&&isscalar(C)&& ...
        all(isfield(C,{'flowpipe','time_grid_s','observed_ids', ...
                       'observed_prepared_shapes','body'})), ...
        'TRIGGER_BATCH_REACH:Inputs','All-time state tube, real observed IDs, body radii required.');
    F=C.flowpipe;
    t=reshape(double(C.time_grid_s),1,[]);K=numel(t)-1;
    ids=reshape(double(C.observed_ids),1,[]);M=numel(ids);
    S=C.observed_prepared_shapes;
    b=C.body;
    assert(K>=1&&all(isfinite(t))&&all(diff(t)>0) && ...
        M>=1 && all(isfinite(ids)) && all(ids>0) && ...
        all(ids==fix(ids)) && numel(unique(ids))==M && ...
        isstruct(S) && numel(S)==M && ...
        all(isfield(S,{'id','center0','bounding_radius_m','is_stationary'})) && ...
        isequal(reshape(double([S.id]),1,[]),ids) && ...
        all([S.is_stationary]), ...
        'TRIGGER_BATCH_REACH:Observation','Only a matching static observed snapshot accepted.');
    assert(isstruct(F)&&isscalar(F) && ...
        isfield(F,'conditional_chain_pass') && isequal(F.conditional_chain_pass,true) && ...
        isfield(F,'conditional_tension_positive_all_intervals') && ...
        isequal(F.conditional_tension_positive_all_intervals,true) && ...
        isfield(F,'manifold_compatible_intervals') && ...
        isequal(size(F.manifold_compatible_intervals),[1 K]) && ...
        all(F.manifold_compatible_intervals) && ...
        isfield(F,'completed_intervals') && isequal(F.completed_intervals,K) && ...
        isfield(F,'tube_lower') && isfield(F,'tube_upper'), ...
        'TRIGGER_BATCH_REACH:Flowpipe','Complete taut/tension-compatible original 24-state tube required.');
    lo=double(F.tube_lower);hi=double(F.tube_upper);
    assert(isequal(size(lo),[24 K]) && isequal(size(hi),[24 K]) && ...
        all(isfinite(lo(:))) && all(isfinite(hi(:))) && all(lo(:)<=hi(:)), ...
        'TRIGGER_BATCH_REACH:Tube','A finite ordered full-time 24xK tube is mandatory.');
    assert(isstruct(b)&&isscalar(b) && ...
        all(isfield(b,{'r_uav_m','r_payload_m','r_cable_m'})), ...
        'TRIGGER_BATCH_REACH:Body','Actual radii missing.');
    r=[double(b.r_uav_m) double(b.r_payload_m) double(b.r_cable_m)];
    assert(all(isfinite(r))&&all(r>=0), ...
        'TRIGGER_BATCH_REACH:Radii','Physical radii must be nonnegative and finite.');
    R.shape_count=M;R.interval_count=K;
    % Do not treat a conventional sensor bounding_radius_m as a rigorous
    % enclosure. Only independently certified source geometry can enter.
    has_outer=isfield(S,'rigorous_outer_ball_certified');
    proved=false(M,1);
    if has_outer
        for j=1:M
            proved(j)=isequal(S(j).rigorous_outer_ball_certified,true);
        end
    end
    R.verified_outer_count=nnz(proved);
    centers=zeros(3,M);radius=zeros(1,M);
    for j=1:M
        cj=double(S(j).center0(:));
        rr=double(S(j).bounding_radius_m);
        if numel(cj)==3 && all(isfinite(cj)) && ...
                isscalar(rr) && isfinite(rr) && rr>=0
            centers(:,j)=cj;radius(j)=rr;
        else
            proved(j)=false;
        end
    end
    % 3 x M x K broadcasting: all original shape outer balls + true state
    % coordinate boxes. Each subtraction is expanded outward ONE binary64
    % step, so strict inequalities never rely on favorable cancellation.
    centers=reshape(centers,3,M,1);
    qlo=reshape(lo(1:3,:),3,1,K);
    qhi=reshape(hi(1:3,:),3,1,K);
    llo=reshape(lo(13:15,:),3,1,K);
    lhi=reshape(hi(13:15,:),3,1,K);
    % Each cable coordinate lies in the convex hull of its TRUE endpoints.
    clo=min(qlo,llo);chi=max(qhi,lhi);
    qgap=local_gap(qlo,qhi,centers);
    lgap=local_gap(llo,lhi,centers);
    cgap=local_gap(clo,chi,centers);
    rq=TRIGGER_FLOAT_OUTWARD_ONE_ULP(radius+r(1),+1);
    rl=TRIGGER_FLOAT_OUTWARD_ONE_ULP(radius+r(2),+1);
    rc=TRIGGER_FLOAT_OUTWARD_ONE_ULP(radius+r(3),+1);
    assert(all(isfinite([rq rl rc])), ...
        'TRIGGER_BATCH_REACH:Overflow','Radius upper enclosure overflow.');
    q_sep=reshape(any(qgap>reshape(rq,1,M,1),1),M,K);
    l_sep=reshape(any(lgap>reshape(rl,1,M,1),1),M,K);
    c_sep=reshape(any(cgap>reshape(rc,1,M,1),1),M,K);
    separated=q_sep & l_sep & c_sep & repmat(proved,1,K);
    R.separated_intervals=separated;
    R.possible_risk_ids=ids(any(~separated,2).');
    R.earliest_possible_event=TRIGGER_FIRST_UNPROVED_INTERVAL(t,ids,separated);
    R.conditional_all_intervals_separated=all(separated(:));
    if R.conditional_all_intervals_separated
        R.status='CONDITIONAL_BATCH_ALL_TIME_SEPARATED__NOT_FLIGHT_PROOF';
    else
        R.status='UNKNOWN_BATCH_ALL_TIME_SOME_INTERVALS_UNPROVED';
    end
    R.reason=['Original flowpipe and outer balls must be proved to include ' ...
       'true physical states/solids for all times. True initial EKF error, ' ...
       'applied future HLC, mass evolution, taut mode and interval arithmetic ' ...
       'source are separate unsatisfied obligations.'];
catch ME
    R.status='UNKNOWN_BATCH_REACH_INPUT_OR_ARITHMETIC';
    R.reason=[ME.identifier ' | ' ME.message];
end
end

function gap=local_gap(xlo,xhi,centers)
% Correctly-rounded elementwise real subtraction, expanded toward -Inf.
a=TRIGGER_FLOAT_OUTWARD_ONE_ULP(xlo-centers,-1);
b=TRIGGER_FLOAT_OUTWARD_ONE_ULP(centers-xhi,-1);
gap=max(max(a,b),0);
end
