function OUT = AF7R58R60_EKF_WHOLEBODY_NOW(IN)
% Independent CURRENT-INSTANT geometry check. Non-committable and not a
% forward trigger. Sensor-known objects only; this code never searches map.
% Required: pQ, pL, q, cable_length, radii [rQ;rL;rCable],
%            obstacles (struct array: center 3x1, rotation 3x3,
%                       axes 3x1, id), geometry_tolerance.
% No inferred mass, sensor range, flight duration, or tracking tube.
% Sufficient support-plane separation for UAV sphere, payload sphere, and
% the COMPLETE straight cable (endpoints). Failure => UNKNOWN, not collision.
% Inward/outward IEEE floating point enclosure is NOT implemented here;
% results are numerical witnesses only. Existing hard certificates govern commit.
req={'pQ','pL','q','cable_length','radii','obstacles','geometry_tolerance'};
for k=1:numel(req)
    assert(isstruct(IN)&&isfield(IN,req{k}), ...
        'R60:MissingInput','Missing EKF field: %s',req{k});
end
pQ=double(IN.pQ(:));pL=double(IN.pL(:));q=double(IN.q(:));
r=double(IN.radii(:));L=double(IN.cable_length);tol=double(IN.geometry_tolerance);
assert(numel(pQ)==3 && numel(pL)==3 && numel(q)==3 && numel(r)==3 ...
    && all(isfinite([pQ;pL;q;r])) && all(r>=0) && ...
    isscalar(L)&&isfinite(L)&&L>0 && ...
    isscalar(tol)&&isfinite(tol)&&tol>=0, ...
    'R60:InvalidGeometry','Invalid required physical parameter');
res=norm(pL-pQ-L*q);
assert(abs(norm(q)-1)<=tol && res<=tol, ...
    'R60:InconsistentEKF','pL != pQ + L*q at supplied tolerance');
obs=IN.obstacles;
assert(isstruct(obs),'R60:Obstacles','Supply sensor-known struct array');
OUT=struct('status','CURRENT_INSTANT_ONLY_NOT_A_4S_TRIGGER', ...
    'commit_allowed',false,'hlc_forward_simulated',false, ...
    'tube_applied',false,'cable_geometry_residual_m',res, ...
    'obstacles_known_count',numel(obs), ...
    'results',repmat(struct('id',NaN,'status','UNKNOWN', ...
    'best_plane_margin_m',NaN,'normal',[NaN;NaN;NaN]),1,numel(obs)));
for j=1:numel(obs)
    o=obs(j);
    assert(all(isfield(o,{'center','rotation','axes','id'})), ...
        'R60:Obstacles','Obstacles must supply center, rotation, axes, id');
    c=double(o.center(:));R=double(o.rotation);ax=double(o.axes(:));
    assert(numel(c)==3&&numel(ax)==3&&isequal(size(R),[3 3])&& ...
        all(isfinite([c;ax;R(:)]))&&all(ax>0) && ...
        norm(R.'*R-eye(3),'fro')<=tol, ...
        'R60:ObstacleGeometry','Invalid oriented ellipsoid');
    mid=(pQ+pL)/2;
    % Normals are search candidates only; any strict supported separator
    % is a sufficient proof in exact arithmetic (not a necessary one).
    V=[pQ-c,pL-c,mid-c,pQ-pL,pL-pQ];
    best=-Inf; nb=[NaN;NaN;NaN];
    for k=1:size(V,2)
        n=V(:,k); ln=norm(n);
        if ln<=eps(max(1,norm(c)))
            continue;
        end
        n=n/ln;
        E=n.'*c+norm(ax.*(R.'*n));
        % The cable is the entire straight segment. By convexity, its
        % minimum n-projection is attained at one of its endpoints.
        bodyMin=min([n.'*pQ-r(1), n.'*pL-r(2), ...
            min(n.'*pQ,n.'*pL)-r(3)]);
        margin=bodyMin-E;
        if margin>best
            best=margin;nb=n;
        end
    end
    OUT.results(j).id=o.id;
    OUT.results(j).best_plane_margin_m=best;
    OUT.results(j).normal=nb;
    if best>tol
        OUT.results(j).status='NUMERIC_SEPARATING_PLANE_FOUND';
    else
        OUT.results(j).status='UNKNOWN_OR_POSSIBLE_INTERSECTION';
    end
end
end
