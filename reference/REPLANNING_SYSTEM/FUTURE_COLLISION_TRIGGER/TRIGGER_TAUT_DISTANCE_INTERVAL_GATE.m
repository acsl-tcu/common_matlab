function R=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE(q,l,shape, ...
    r_uav,r_payload,r_cable,e_u,e_l,time_s)
% TRIGGER_TAUT_DISTANCE_INTERVAL_GATE
% WHOLE-TIME (conditional) clearance for original occupied shape O.
% Each true endpoint must remain within an EXTERNALLY PROVED ball of radius
% e_u/e_l around q/l for ALL times in the interval. If cable stays taut
% and straight, its true spatial segment is within max(e_u,e_l) of the
% nominal segment at any t, using the SAME interpolation parameter.
%
% For every closed set O, its unsigned Euclidean distance d(x,O) is
% 1-Lipschitz: d(x,O)>=d(m,O)-||x-m||. The entire nominal cable is thus
% separated if a finite midpoint subdivision establishes
% d(mid,O)-length(subsegment)/2 > r_cable+max(e_u,e_l).
% This works for nonconvex holes (torus), arbitrary SO(3) shapes and any
% future sensor shape with the common true-shape distance oracle. NO
% obstacle-type switches or enclosing ellipsoid are introduced.
%
% The original distance oracle uses floating arithmetic and is NOT yet an
% independently proven outward-ROUNDED LOWER BOUND. This is CONDITIONAL
% mathematics only; NEVER grants Flight safety or safe_to_commit.
R=struct('status','UNKNOWN_NO_ORACLE_CLEARANCE', ...
    'conditionally_separated',false, ...
    'components_clear',false(1,3), ...
    'distance_queries',0,'exhausted_budget',false, ...
    'nonconvex_distance_witness',false, ...
    'endpoint_lipschitz_used',false, ...
    'safe_to_commit',false,'reason','');
try
    if ~(isnumeric(q)&&isreal(q)&&isequal(size(q),[3 1])&& ...
            isnumeric(l)&&isreal(l)&&isequal(size(l),[3 1]) && ...
            all(isfinite([q;l])) && ...
            isstruct(shape)&&isscalar(shape)&& ...
            all(isfield(shape,{'center0','is_stationary','id'}))&& ...
            isequal(shape.is_stationary,true) && ...
            all(isfinite([r_uav r_payload r_cable e_u e_l time_s])) && ...
            all([r_uav r_payload r_cable e_u e_l]>=0))
        R.reason='No stationary shape or bounded true endpoint excursion.';
        return
    end
    c=double(shape.center0(:));
    if ~isequal(size(c),[3 1])||any(~isfinite(c)),return;end
    rU=double(r_uav)+double(e_u);
    rL=double(r_payload)+double(e_l);
    rC=double(r_cable)+max(double(e_u),double(e_l));
    geom_scale=max([1;abs(double(q(:)));abs(double(l(:))); ...
        abs(c);rU;rL;rC]);
    guard=8192*eps(geom_scale);
    % This finite work budget never produces a clearance conclusion on
    % exhaustion; it is a scheduling guard, NOT an invented physical bound.
    max_queries=48;
    d0=OBSTACLE_GEOMETRY_QUERY.distance(q,shape,time_s,c);
    R.distance_queries=R.distance_queries+1;
    if ~(isfinite(d0)&&d0>rU+guard),return;end
    R.components_clear(1)=true;
    d1=OBSTACLE_GEOMETRY_QUERY.distance(l,shape,time_s,c);
    R.distance_queries=R.distance_queries+1;
    if ~(isfinite(d1)&&d1>rL+guard),return;end
    R.components_clear(2)=true;
    % Evaluate the spatial cable segment independently of body spheres.
    % At a degenerate segment, the common point still needs the cable test.
    if d0<=rC+guard || d1<=rC+guard,return;end
    delta=l-q;len=norm(delta);
    % NEW shape-independent TWO-ENDPOINT LIPSCHITZ WITNESS.
    % For an arbitrary closed occupied set O, d(x,O) is 1-Lipschitz.
    % For x(s)=q+s*(l-q), s in [0,1], let a=s*len. Then
    %   d(x(s),O) >= max(d0-a, d1-(len-a))
    %             >= (d0+d1-len)/2.
    % Consequently, if d0+d1-len > 2*(r_cable+max(e_u,e_l))
    % plus an engineering rounding guard, every material cable point
    % is conditionally separated without ANY midpoint distance calls.
    % This is valid for convex/nonconvex, rotated, and future shape types
    % as long as the sensor's true-shape distance is a LOWER bound.
    % Existing subtraction/midpoint oracle remains a conservative fallback.
    if isfinite(len) && isfinite(d0+d1) && ...
            d0+d1-len>2*(rC+guard)
        R.components_clear(3)=true;
        R.endpoint_lipschitz_used=true;
    elseif len==0
        R.components_clear(3)=true;
    else
        % Stack-based deterministic DFS. Each node carries a subsegment
        % in [0,1]. Queue budget is capped by allowed distance calls.
        stack0=zeros(1,max_queries+2);
        stack1=zeros(1,max_queries+2);
        top=1;stack0(1)=0;stack1(1)=1;
        while top>0
            a=stack0(top);b=stack1(top);top=top-1;
            mid=0.5*(a+b);
            if mid==a || mid==b
                R.reason='Floating-resolution limit is unresolved.';
                return
            end
            if R.distance_queries>=max_queries
                R.exhausted_budget=true;
                R.reason='Safe finite distance-query budget exhausted; UNKNOWN.';
                return
            end
            dm=OBSTACLE_GEOMETRY_QUERY.distance(q+mid*delta,shape,time_s,c);
            R.distance_queries=R.distance_queries+1;
            if ~isfinite(dm)||dm<0
                R.reason='Shape distance was not a finite lower bound.';
                return
            end
            if dm-(b-a)*len/2>rC+guard
                continue
            end
            % Even if inflated nominal segment intersects O, this is NOT
            % a statement about actual collision. It is simply UNKNOWN.
            if dm<=rC+guard
                R.reason='Proximity prevents a continuous clearance witness.';
                return
            end
            top=top+1;stack0(top)=mid;stack1(top)=b;
            top=top+1;stack0(top)=a;stack1(top)=mid;
        end
        R.components_clear(3)=true;
    end
    R.conditionally_separated=all(R.components_clear);
    if R.conditionally_separated
        R.status='CONDITIONAL_ALL_TIME_TRUE_SHAPE_DISTANCE_CLEAR';
        R.nonconvex_distance_witness=true;
        R.reason=['Only valid if the input excursion tubes, true taut ' ...
            'geometry, static sensor provenance and distance LOWER BOUNDS ' ...
            'are mathematically verified. No real Flight proof.'];
    end
catch ME
    % Any malformed original-shape oracle must never suppress a contact.
    R.status='UNKNOWN_SHAPE_DISTANCE_EXCEPTION';
    R.conditionally_separated=false;
    R.components_clear=false(1,3);
    R.reason=[ME.identifier ' | ' ME.message];
end
end
