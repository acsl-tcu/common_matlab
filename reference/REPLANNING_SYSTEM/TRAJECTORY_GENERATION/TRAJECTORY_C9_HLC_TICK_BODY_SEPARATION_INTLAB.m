function R=TRAJECTORY_C9_HLC_TICK_BODY_SEPARATION_INTLAB(state_box_24x2,observed,radii)
%TRAJECTORY_C9_HLC_TICK_BODY_SEPARATION_INTLAB All-time ONE-tick body lemma.
% Given a rigorously outward-rounded 24x2 plant reach box, prove that
% UAV(x[1:3]), payload(x[13:15]) and every convex combination of their
% endpoints (the ENTIRE STRAIGHT CABLE) miss EVERY OBSERVED static shape.
% All 12 shapes and arbitrary SO(3) are supported via outward INTLAB
% evaluations of the SOURCE convex-hull support. Nonconvex shapes use
% the support of their exact convex HULL (sufficient for exclusion).
%
% A shape-specific arbitrary 3D separating normal is used only if the
% rigorous inequality inf(n' BODY_REACH) > sup(h_OBSTACLE(n)+r*||n||)
% holds. No sampled point, fake clearance, fixed avoidance direction or
% substitute obstacle sphere can make this function return TRUE.
%
% This is a CONDITIONAL mathematical lemma: the caller must establish
% that state_box_24x2 really encloses the active model's entire tick;
% moreover the physical cable is assumed straight between these bodies.
% It does NOT certify future EKF/HLC ticks or allow a Flight command.
R=struct('status','UNKNOWN_TICK_BODY_INTERVAL_NOT_RUN', ...
    'conditional_alltime_body_separation',false, ...
    'observed_ids',zeros(1,0),'unresolved_ids',zeros(1,0), ...
    'separated_body_matrix',false(3,0), ...
    'ground_certified',false,'ground_unresolved',false, ...
    'tested_halfspaces',0,'shape_support_evaluations',0, ...
    'safe_to_commit',false,'elapsed_ms',NaN, ...
    'scope','CONDITIONAL_ONE_TICK_SWEPT_UAV_CABLE_PAYLOAD');
tk=tic;
if ~(isnumeric(state_box_24x2)&&isreal(state_box_24x2)&& ...
        isequal(size(state_box_24x2),[24 2])&& ...
        all(isfinite(state_box_24x2(:)))&& ...
        all(state_box_24x2(:,1)<=state_box_24x2(:,2))&& ...
        isstruct(observed)&& ...
        isstruct(radii)&&isscalar(radii)&& ...
        all(isfield(radii,{'uav_m','cable_m','payload_m'})))
    R.status='UNKNOWN_TICK_BODY_INVALID_AUTHORITY';
    R.elapsed_ms=1000*toc(tk);return
end
radius=[radii.uav_m,radii.cable_m,radii.payload_m];
if ~(isnumeric(radius)&&isreal(radius)&&numel(radius)==3&& ...
        all(isfinite(radius))&&all(radius>=0))
    R.status='UNKNOWN_TICK_BODY_INVALID_RADII';
    R.elapsed_ms=1000*toc(tk);return
end
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_TICK_BODY_DIRECTED_INTERVAL_ENGINE_MISSING';
    R.elapsed_ms=1000*toc(tk);return
end
try
    % Flight ground authority is separate from any finite obstacle.
    G=GROUND_INFORMATION.definition();
    if ~(strcmp(char(G.type),'ground_halfspace')&& ...
         isequal(G.normal_world,[0;0;1])&& ...
         isequal(G.height_m,0)&& ...
         strcmp(char(G.occupied_side),'negative'))
        R.status='UNKNOWN_TICK_BODY_GROUND_DEFINITION';
        R.elapsed_ms=1000*toc(tk);return
    end
    S=TRAJECTORY_OBSTACLE_SUPPORT.prepare(observed);
    R.observed_ids=reshape([S.id],1,[]);
    R.separated_body_matrix=false(3,numel(S));
    X=infsup(state_box_24x2(:,1),state_box_24x2(:,2));
    U=X(1:3); L=X(13:15);
    % The entire cable is a convex combination of U and L at each t.
    % Bounds may ignore their dependence; this makes the proof conservative.
    ui=double(inf(U(3)));li=double(inf(L(3)));
    R.ground_certified=(ui>G.height_m+radius(1))&& ...
        (li>G.height_m+radius(3))&& ...
        (min(ui,li)>G.height_m+radius(2));
    R.ground_unresolved=~R.ground_certified;
    % Candidate normal directions are geometry-derived, not navigation
    % directions. Every positive certificate is re-evaluated in INTLAB.
    UM=(state_box_24x2(1:3,1)+state_box_24x2(1:3,2))/2;
    LM=(state_box_24x2(13:15,1)+state_box_24x2(13:15,2))/2;
    for j=1:numel(S)
        O=S(j);
        normals=[UM-O.center,LM-O.center, ...
            (UM+LM)/2-O.center,O.R,-O.R];
        proved=false(3,1);
        for k=1:size(normals,2)
            n=normals(:,k);
            if ~(all(isfinite(n))&&any(n~=0)),continue;end
            N=infsup(n,n);
            nv=sqrt(sum(N.^2));
            hi_support=source_support_upper(S(j),N);
            R.shape_support_evaluations=R.shape_support_evaluations+1;
            if ~isfinite(hi_support),continue;end
            lowerU=double(inf(sum(U.*N)));
            lowerL=double(inf(sum(L.*N)));
            lower=[lowerU;min(lowerU,lowerL);lowerL];
            for part=1:3
                if proved(part),continue;end
                highObstacle=double(sup(infsup(hi_support,hi_support) ...
                    +infsup(radius(part),radius(part))*nv));
                R.tested_halfspaces=R.tested_halfspaces+1;
                if isfinite(highObstacle)&&isfinite(lower(part))&& ...
                        lower(part)>highObstacle
                    proved(part)=true;
                end
            end
            if all(proved),break;end
        end
        R.separated_body_matrix(:,j)=proved;
        if ~all(proved)
            R.unresolved_ids(end+1)=double(O.id); %#ok<AGROW>
        end
    end
    R.conditional_alltime_body_separation= ...
        R.ground_certified&&all(R.separated_body_matrix(:));
    if R.conditional_alltime_body_separation
        R.status='CONDITIONAL_ONE_TICK_SOURCE_BOX_ALLBODY_ALLTIME_SEPARATED';
    elseif R.ground_unresolved
        R.status='UNKNOWN_ONE_TICK_GROUND_SEPARATION';
    else
        R.status='UNKNOWN_ONE_TICK_BODY_SEPARATION';
    end
catch ME
    R.status=['UNKNOWN_TICK_BODY_INTERVAL_EXCEPTION_' ME.identifier];
end
R.elapsed_ms=1000*toc(tk);
end

function up=source_support_upper(S,n)
% The literal canonical 12-shape formulas of
% TRAJECTORY_OBSTACLE_SUPPORT.value, re-evaluated with directed INTLAB.
% The returned scalar is a one-sided upper bound for a FIXED chosen n.
w=infsup(S.R.',S.R.')*n;
rho=sqrt(w(1).^2+w(2).^2);
mag=sqrt(sum(w.^2));
d=S.dim;
base=sum(infsup(S.center,S.center).*n);
if ~isempty(S.local_vertices)
    V=infsup(S.local_vertices,S.local_vertices);
    support_candidates=V*w;
    up=max(double(sup(base+support_candidates)));
    return
end
switch char(S.kind)
    case 'sphere'
        candidates={infsup(d(1),d(1))*mag};
    case 'ellipsoid'
        D=infsup(double(d(:)),double(d(:)));
        candidates={sqrt(sum((D.*w).^2))};
    case 'box'
        D=infsup(double(d(:)),double(d(:)));
        candidates={sum(D.*abs(w))/2};
    case 'cylinder'
        candidates={infsup(d(1),d(1))*rho+ ...
            infsup(d(2),d(2))*abs(w(3))/2};
    case 'capsule'
        candidates={infsup(d(1),d(1))*mag+ ...
            infsup(d(2),d(2))*abs(w(3))/2};
    case 'cone'
        candidates={infsup(d(1),d(1))*rho-infsup(d(2),d(2))*w(3)/2, ...
            infsup(d(2),d(2))*w(3)/2};
    case 'frustum'
        candidates={infsup(d(1),d(1))*rho-infsup(d(3),d(3))*w(3)/2, ...
            infsup(d(2),d(2))*rho+infsup(d(3),d(3))*w(3)/2};
    case 'torus'
        candidates={infsup(d(1),d(1))*rho+infsup(d(2),d(2))*mag};
    otherwise
        error('C9_TICK_BODY:UnpreparedShape', ...
            'Nonvertex canonical shape unsupported: %s',char(S.kind));
end
up=-Inf;
for i=1:numel(candidates)
    up=max(up,double(sup(base+candidates{i})));
end
end
