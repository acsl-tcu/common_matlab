function D=AF7R58R95_CORRIDOR_PREFLIGHT(W,obs,clearance)
% Reference-generation ONLY. Construct one convex PAYLOAD corridor per
% straight waypoint edge from exact oriented-ellipsoid support planes.
% CP containment is evaluated later by the independent R94 certifier.
% Absence of a separating plane means THIS CORRIDOR FAMILY is unresolved,
% NOT that no collision-free C7 trajectory exists.
% No obstacle-count/radius/time limit. No sensed-unknown assumptions.
assert(isnumeric(W)&&size(W,1)==3&&size(W,2)>=2&& ...
    all(isfinite(W(:))),'R95:FiniteWaypointsRequired');
assert(isstruct(obs)&&isnumeric(clearance)&&isscalar(clearance)&& ...
    isfinite(clearance)&&clearance>=0,'R95:ExternalGeometryRequired');
S=size(W,2)-1;
cor=cell(1,S);
D=struct('status','CANDIDATE_CORRIDORS_FOUND__NOT_SAFETY_PROOF', ...
    'corridors',{cor},'feasible_corridor_family',true,'first_bad_segment',NaN, ...
    'first_bad_obstacle',NaN,'min_endpoint_margin_m',Inf, ...
    'known_obstacles_checked',numel(obs),'whole_body_certified',false, ...
    'production_commit_authorized',false);
for s=1:S
    a=W(:,s);b=W(:,s+1);v=b-a; vv=v'*v;
    assert(vv>0,'R95:CoincidentWaypointsNeedLocalChart');
    N=zeros(0,3);rhs=zeros(0,1);
    for j=1:numel(obs)
        o=obs(j);
        assert(isequal(size(o.center),[3 1])&& ...
            isequal(size(o.axes),[3 1])&&all(o.axes>0)&& ...
            isequal(size(o.R),[3 3])&& ...
            all(isfinite([o.center;o.axes;o.R(:)]))&& ...
            norm(o.R'*o.R-eye(3),'fro')<1e-9&& ...
            abs(det(o.R)-1)<1e-9,'R95:BadOrientedEllipsoid');
        c=double(o.center);Rt=double(o.R);ax=double(o.axes);
        t=max(0,min(1,(c-a)'*v/vv));p=a+t*v;
        % Bounding sphere is used only to omit planes from this initial
        % candidate corridor, never to omit known obstacles from R80.
        if norm(p-c)>max(ax)+clearance+ ...
            256*eps(max([1,norm(p),norm(c),max(ax)]))
            continue;
        end
        % Analytic ellipsoid support, sampled UNIT NORMAL candidates,
        % including outward radial directions at every chord parameter.
        z=[a-c,b-c,p-c];
        for ttest=linspace(0,1,33)
            z(:,end+1)=a+ttest*v-c; %#ok<AGROW>
        end
        % Rotated axis directions are relevant for extremely eccentric
        % obstacles. Sampling normals is an initialization algorithm,
        % not an exact separation completeness theorem.
        z=[z,Rt,-Rt];
        best=-Inf;nBest=[];planeBest=NaN;
        for h=1:size(z,2)
            nn=z(:,h);len=norm(nn);
            if ~(isfinite(len)&&len>0),continue;end
            nn=nn/len;
            support=nn'*c+norm(ax.*(Rt'*nn))+clearance;
            margin=min(nn'*a,nn'*b)-support;
            if margin>best
                best=margin;nBest=nn;planeBest=support;
            end
        end
        D.min_endpoint_margin_m=min(D.min_endpoint_margin_m,best);
        guard=512*eps(max([1,norm(a),norm(b),norm(c),max(ax),clearance]));
        if ~(isfinite(best)&&best>guard)
            D.feasible_corridor_family=false;
            D.first_bad_segment=s;D.first_bad_obstacle=j;
            D.status='CORRIDOR_FAMILY_UNRESOLVED__TRY_DIFFERENT_WAYPOINTS';
            D.corridors=cor;return;
        end
        N(end+1,:)=-nBest.'; %#ok<AGROW>
        rhs(end+1,1)=-planeBest; %#ok<AGROW>
    end
    cor{s}=struct('A',N,'b',rhs,'source','EXACT_ELLIPSOID_SUPPORT_PAYLOAD_ONLY');
end
D.corridors=cor;
end
