function R=TRIGGER_TAUT_REDUCED_DIRECTIONAL_ENCLOSURE(X,L,radii,normals,shape)
% Conditional full-time interval support of the TAUT UAV+cable+load union.
% The caller MUST supply a continuous-time enclosure of q,v,d,w for an
% entire integration interval. Reconstructs pL=q+L*d and
% vL=v+L*cross(w,d), rather than using independently inflated endpoints.
% Normals are columns; a separating plane exists if the returned upper
% support of the flying assembly is below an obstacle's lower support.
% Not a complete flight certificate: input enclosure, tautness, EKF tube,
% source/rounding correctness and all other intervals remain unverified.
% No nominal Flight/sensor world read, and NEVER changes control/reference.
R=struct('status','UNKNOWN_TAUT_DIRECTIONAL_ENCLOSURE', ...
    'conditional_enclosure',false,'safe_to_commit',false, ...
    'reason','','payload_position_lower',[], ...
    'payload_position_upper',[], 'payload_velocity_lower',[], ...
    'payload_velocity_upper',[], ...
    'uav_support_upper',[], 'cable_support_upper',[], ...
    'payload_support_upper',[],'union_support_upper',[], ...
    'obstacle_support_lower',[],'conditional_separated',[]);
try
    assert(isstruct(X)&&isscalar(X), ...
        'TRIGGER_TAUT_REDUCED:State','X must be a scalar structure.');
    required={'q','v','d','w'};
    for j=1:numel(required)
        field=required{j};
        assert(isfield(X,field)&&isstruct(X.(field))&& ...
            isfield(X.(field),'lower')&&isfield(X.(field),'upper'), ...
            'TRIGGER_TAUT_REDUCED:State','Missing interval field: %s',field);
        a=X.(field).lower;b=X.(field).upper;
        assert(isnumeric(a)&&isnumeric(b)&& ...
            isequal(size(a),[3 1])&&isequal(size(b),[3 1])&& ...
            isreal(a)&&isreal(b)&&all(isfinite(a))&& ...
            all(isfinite(b))&&all(a<=b), ...
            'TRIGGER_TAUT_REDUCED:State','Invalid 3x1 state interval.');
    end
    if isnumeric(L)
        assert(isreal(L)&&isscalar(L)&&isfinite(L)&&L>0, ...
            'TRIGGER_TAUT_REDUCED:Length','Positive length needed.');
        L=struct('lower',double(L),'upper',double(L));
    end
    assert(isstruct(L)&&isscalar(L)&& ...
        isfield(L,'lower')&&isfield(L,'upper')&& ...
        isnumeric(L.lower)&&isnumeric(L.upper)&& ...
        isscalar(L.lower)&&isscalar(L.upper)&& ...
        isfinite(L.lower)&&isfinite(L.upper)&& ...
        L.lower>0&&L.upper>=L.lower, ...
        'TRIGGER_TAUT_REDUCED:Length','Positive interval length required.');
    assert(isstruct(radii)&&isscalar(radii)&& ...
        all(isfield(radii,{'uav','cable','payload'})), ...
        'TRIGGER_TAUT_REDUCED:Radii', ...
        'Actual conservative sphere/cable material radii required.');
    rv=[radii.uav,radii.cable,radii.payload];
    assert(isnumeric(rv)&&isreal(rv)&&numel(rv)==3&& ...
        all(isfinite(rv))&&all(rv>=0), ...
        'TRIGGER_TAUT_REDUCED:Radii','Nonnegative finite radii required.');
    assert(isnumeric(normals)&&isreal(normals)&&size(normals,1)==3&& ...
        ~isempty(normals)&&all(isfinite(normals(:)))&& ...
        all(sqrt(sum(normals.^2,1))>0), ...
        'TRIGGER_TAUT_REDUCED:Normals', 'Nonzero finite 3xN normals.');
    % Unit sphere must at least intersect d box. This test is NECESSARY,
    % not a proof that every vector in the box is unit length.
    a=X.d.lower;b=X.d.upper;
    mind=max(0,max(a,-b)); % componentwise min |d_i|
    maxd=max(abs(a),abs(b));
    assert(sum(mind.^2)<=1 && sum(maxd.^2)>=1, ...
        'TRIGGER_TAUT_REDUCED:Direction', ...
        'Direction enclosure contains no possible unit vector.');
    mk=@(a,b) TRIGGER_OUTWARD_INTERVAL(double(a),double(b));
    qi=cell(3,1);di=cell(3,1);vi=cell(3,1);wi=cell(3,1);
    for j=1:3
        qi{j}=mk(X.q.lower(j),X.q.upper(j));
        di{j}=mk(X.d.lower(j),X.d.upper(j));
        vi{j}=mk(X.v.lower(j),X.v.upper(j));
        wi{j}=mk(X.w.lower(j),X.w.upper(j));
    end
    Li=mk(L.lower,L.upper);
    pL=cell(3,1);vL=cell(3,1);
    for j=1:3
        k=mod(j,3)+1;h=mod(j+1,3)+1;
        pL{j}=qi{j}+Li.*di{j};
        vL{j}=vi{j}+Li.*(wi{k}.*di{h}-wi{h}.*di{k});
        R.payload_position_lower(j,1)=pL{j}.lo;
        R.payload_position_upper(j,1)=pL{j}.hi;
        R.payload_velocity_lower(j,1)=vL{j}.lo;
        R.payload_velocity_upper(j,1)=vL{j}.hi;
    end
    count=size(normals,2);
    R.uav_support_upper=zeros(1,count);
    R.cable_support_upper=zeros(1,count);
    R.payload_support_upper=zeros(1,count);
    R.union_support_upper=zeros(1,count);
    R.conditional_separated=false(1,count);
    if nargin>=5 && ~isempty(shape)
        assert(isstruct(shape)&&isscalar(shape)&& ...
            isfield(shape,'center0')&& ...
            isfield(shape,'rotation')&&isfield(shape,'type'), ...
            'TRIGGER_TAUT_REDUCED:Obstacle', ...
            'Need observed original shape (center0/rotation/type).');
        assert(isnumeric(shape.center0)&& ...
            isequal(size(shape.center0),[3 1])&& ...
            all(isfinite(shape.center0)), ...
            'TRIGGER_TAUT_REDUCED:Obstacle','Invalid obstacle center.');
        R.obstacle_support_lower=zeros(1,count);
    end
    for k=1:count
        n=double(normals(:,k));n=n/norm(n);
        nq=mk(0,0);nd=mk(0,0);
        for j=1:3
            nq=nq+qi{j}.*n(j);
            nd=nd+di{j}.*n(j);
        end
        ld=Li.*nd;
        pp=nq+ld;
        iu=nq+mk(rv(1),rv(1));u=iu.hi;
        % The full finite straight cable: max of both endpoints, never
        % a spherical envelope centered on the UAV or rope midpoint.
        ic=nq+mk(max(0,ld.hi),max(0,ld.hi))+ ...
            mk(rv(2),rv(2));c=ic.hi;
        ip=pp+mk(rv(3),rv(3));p=ip.hi;
        R.uav_support_upper(k)=u;
        R.cable_support_upper(k)=c;
        R.payload_support_upper(k)=p;
        R.union_support_upper(k)=max([u,c,p]);
        if ~isempty(R.obstacle_support_lower)
            % Original obstacle support is center + exact convex-hull radial
            % support. Numerical rotations/support are NOT yet formally
            % outward-rounded. Output is a conditional geometry test ONLY.
            radial=TRIGGER_GEOMETRY_CERTIFICATE.support_radial(shape,-n);
            lower=n.'*double(shape.center0(:))-radial;
            R.obstacle_support_lower(k)=lower;
            R.conditional_separated(k)=R.union_support_upper(k)<lower;
        end
    end
    R.conditional_enclosure=true;
    R.status='CONDITIONAL_REDUCED_TAUT_SWEPT_SUPPORT__NOT_FLIGHT_SAFETY';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
