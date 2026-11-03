function R=TRIGGER_DIRECTIONAL_REACH_CERT(reach,shapes,body)
% TRIGGER_DIRECTIONAL_REACH_CERT
% Conditional continuous-time separation theorem for an actual taut cable.
%
% No sampling inference: supplied 3xK lower/upper matrices MUST contain
% ALL actual positions at ALL times in [grid(k),grid(k+1)]. They are NOT
% sampled numerical trajectories and MUST be supplied by a separate
% independently audited, outward-rounded closed-loop reachability solver.
% This function cannot verify that obligation and NEVER grants safe_to_commit.
%
% For a unit direction n, each interval has the guaranteed lower supports
%   l_Q(n)=n'*q_mid-|n|'*q_half;  l_L(n)=n'*l_mid-|n|'*l_half.
% The taut cable material is the segment [Q,L] thickened by physical rC.
% Hence min_{cable} n'*x >= min(l_Q,l_L)-rC. Independently,
% UAV >= l_Q-rU and payload >= l_L-rL. A static obstacle O satisfies
%   max_{x in O} n'*x = n'*center+support_O(n).
% If all three lower bounds STRICTLY exceed the latter for the SAME n,
% all three physical bodies are separated for the ENTIRE time interval.
% The exact radial support for arbitrarily rotated ellipsoids is reused.
%
% Floating-point support values below are exploratory double arithmetic,
% NOT directed/outward-rounded bounds. Positive numerical margins alone
% are NOT a flight safety proof. Unknown stays unknown.
R=struct('status','UNKNOWN_REACH_TUBE_NOT_AUDITED', ...
    'conditional_geometry_pass',false,'safe_to_commit',false, ...
    'continuous_closed_loop_safe_certified',false, ...
    'requires_audited_continuous_reach_tube',true, ...
    'requires_verified_straight_taut_cable',true, ...
    'requires_outward_rounding',true, ...
    'possibly_at_risk_ids',zeros(1,0),'witness_direction',[], ...
    'minimum_nominal_separation_m',[], ...
    'conditional_separated_intervals',[], 'reason','');
try
    assert(isstruct(reach)&&isscalar(reach) && ...
        all(isfield(reach,{'time_grid_s','uav_lower_m','uav_upper_m', ...
            'payload_lower_m','payload_upper_m'})), ...
        'TRIGGER_DIRECTIONAL_REACH_CERT:Reach','Complete interval enclosures required.');
    grid=double(reach.time_grid_s(:).');
    assert(numel(grid)>=2&&all(isfinite(grid))&&all(diff(grid)>0), ...
        'TRIGGER_DIRECTIONAL_REACH_CERT:Time','Invalid interval time grid.');
    k=numel(grid)-1;
    data={reach.uav_lower_m,reach.uav_upper_m,reach.payload_lower_m,reach.payload_upper_m};
    for j=1:4
        a=data{j};
        assert(isnumeric(a)&&isreal(a)&&isequal(size(a),[3 k])&& ...
            all(isfinite(a(:))), ...
            'TRIGGER_DIRECTIONAL_REACH_CERT:Boxes','Invalid 3xK continuous tube.');
        data{j}=double(a);
    end
    qlo=data{1};qhi=data{2};llo=data{3};lhi=data{4};
    assert(all(qhi(:)>=qlo(:))&&all(lhi(:)>=llo(:)), ...
        'TRIGGER_DIRECTIONAL_REACH_CERT:Boxes','Lower exceeds upper.');
    assert(isstruct(body)&&isscalar(body) && ...
        all(isfield(body,{'r_uav_m','r_payload_m','r_cable_m'})), ...
        'TRIGGER_DIRECTIONAL_REACH_CERT:Body','Physical radii required.');
    rad=[body.r_uav_m;body.r_payload_m;body.r_cable_m];
    assert(isnumeric(rad)&&isreal(rad)&&isequal(size(rad),[3 1])&& ...
        all(isfinite(rad))&&all(rad>=0), ...
        'TRIGGER_DIRECTIONAL_REACH_CERT:Body','Nonnegative physical radii required.');
    assert(isstruct(shapes)&&all(isfield(shapes,{'id','center0', ...
        'rotation','dimensions','type','is_stationary'})), ...
        'TRIGGER_DIRECTIONAL_REACH_CERT:Shapes','Prepared observed geometry required.');
    assert(isempty(shapes)|| ...
        (numel(unique([shapes.id]))==numel(shapes) && ...
         all([shapes.is_stationary])), ...
        'TRIGGER_DIRECTIONAL_REACH_CERT:Shapes', ...
        'Only individually observed static obstacle IDs are supported.');
    nobs=numel(shapes);
    separated=false(nobs,k);best=-inf(nobs,k);
    direction=nan(3,k,nobs);
    qc=(qlo+qhi)/2; qh=(qhi-qlo)/2;
    lc=(llo+lhi)/2; lh=(lhi-llo)/2;
    for j=1:nobs
        sh=shapes(j);
        c=double(sh.center0(:));
        assert(isequal(size(c),[3 1])&&all(isfinite(c)), ...
            'TRIGGER_DIRECTIONAL_REACH_CERT:Center','Invalid known obstacle center.');
        for i=1:k
            % Actual-obstacle-derived normals are search hints ONLY. A
            % witness is checked via exact support of the original solid.
            mid=(qc(:,i)+lc(:,i))/2;
            opts=[qc(:,i)-c,lc(:,i)-c,mid-c, ...
                sh.rotation, -sh.rotation, eye(3),-eye(3)];
            a=lc(:,i)-qc(:,i);
            if dot(a,a)>0
                alpha=max(0,min(1,dot(c-qc(:,i),a)/dot(a,a)));
                near=qc(:,i)+alpha*a;
                opts=[opts near-c]; %#ok<AGROW>
            end
            [X,Y,Z]=ndgrid(-1:1,-1:1,-1:1);
            opts=[opts, [X(:).';Y(:).';Z(:).']]; %#ok<AGROW>
            norms=sqrt(sum(opts.^2,1));ok=isfinite(norms)&norms>0;
            ns=opts(:,ok)./norms(ok);
            if isempty(ns),continue;end
            support=double(c.'*ns)+ ...
                TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many(sh,ns);
            lbQ=(qc(:,i).'*ns)-qh(:,i).'*abs(ns);
            lbL=(lc(:,i).'*ns)-lh(:,i).'*abs(ns);
            clearance=min([lbQ-rad(1);lbL-rad(2); ...
                min(lbQ,lbL)-rad(3)],[],1)-support;
            [best(j,i),ix]=max(clearance);
            if best(j,i)>0
                separated(j,i)=true;
                direction(:,i,j)=ns(:,ix);
            end
        end
    end
    ids=reshape(double([shapes.id]),1,[]);
    R.conditional_separated_intervals=separated;
    R.witness_direction=direction;
    R.minimum_nominal_separation_m=best;
    R.possibly_at_risk_ids=ids(any(~separated,2).');
    R.conditional_geometry_pass=all(separated(:));
    if R.conditional_geometry_pass
        R.status='CONDITIONAL_DIRECTIONAL_HULL_GEOMETRY_PASS__NO_FLIGHT_CERTIFICATE';
        R.reason='Numeric support separation only; closed-loop tube, taut cable and outward rounding are NOT verified.';
    else
        R.status='UNKNOWN_UNSEPARATED_REACH_INTERVAL';
        R.reason='Candidate directions did not establish separation; this does not prove actual contact.';
    end
catch ME
    R.status='UNKNOWN_DIRECTIONAL_CERT_INPUT_OR_NUMERIC_ERROR';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
