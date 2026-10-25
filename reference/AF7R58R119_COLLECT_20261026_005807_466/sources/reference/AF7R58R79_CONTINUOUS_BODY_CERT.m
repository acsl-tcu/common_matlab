function Q=AF7R58R79_CONTINUOUS_BODY_CERT(C,obstacles,cable_length_m,r_uav_m,r_cable_m,r_payload_m,max_nodes)
% Conservative continuous-time geometric certificate for ONE connector.
% Proves all UAV/cable/payload points lie inside a ball centered on the
% payload (taut cable L) and separates this ball from every ellipsoid via
% a support plane over each Bezier subsegment's entire convex hull.
% No sampled acceptance. INCONCLUSIVE is NOT safe; no actuation/tension/tube proof.
assert(isstruct(C)&&isfield(C,'payload_cp')&& ...
    isequal(size(C.payload_cp),[3,16]),'[R79] invalid candidate.');
vals=[cable_length_m,r_uav_m,r_cable_m,r_payload_m];
assert(isnumeric(vals)&&numel(vals)==4&&all(isfinite(vals))&& ...
    cable_length_m>0&&all(vals(2:4)>=0), ...
    '[R79] exact live geometry radii and cable length required.');
assert(isnumeric(max_nodes)&&isscalar(max_nodes)&&isfinite(max_nodes)&& ...
    max_nodes==fix(max_nodes)&&max_nodes>=1, ...
    '[R79] finite caller-authorized certification work budget required.');
% Cable direction may swing in 3D; bounding ball needs no q assumption except |q|=1.
rho=max([r_payload_m,cable_length_m+r_cable_m,cable_length_m+r_uav_m]);
obs=obstacles;
if isempty(obs)
    Q=struct('certified',false,'status','UNKNOWN_NO_KNOWN_OBSTACLE_COVERAGE', ...
        'continuous_time',false,'whole_body_ball_radius_m',rho, ...
        'leaves',1,'min_support_margin_m',Inf, ...
        'limitations','KNOWN obstacles; one connector; requires taut cable and exact geometry');
    return;
end
assert(isstruct(obs),'[R79] obstacle array of structs required.');
for k=1:numel(obs)
    o=obs(k);
    assert(isfield(o,'center')&&isfield(o,'R')&&isfield(o,'axes'), ...
        '[R79] obstacle requires center,R,axes in world frame.');
    assert(isequal(size(o.center),[3 1])&&isequal(size(o.R),[3 3])&& ...
        isequal(size(o.axes),[3 1])&& ...
        all(isfinite(o.center))&&all(isfinite(o.R(:)))&& ...
        all(isfinite(o.axes))&&all(o.axes>0) && ...
        norm(o.R'*o.R-eye(3),'fro')<1e-9 && det(o.R)>0, ...
        '[R79] invalid oriented ellipsoid surface.');
end
% Depth-first adaptive Bezier subdivisions. No arbitrary maneuver duration cap.
% max_nodes is a COMPUTATION resource budget, not an allowed safety gap.
work=struct('cp',C.payload_cp,'t0',C.t0,'t1',C.t1);
stack=work; certifiedLeaves=0; nodes=0; worst=Inf; reason='';
while ~isempty(stack)
    if nodes>=max_nodes
        reason='INCONCLUSIVE_RESOURCE_BUDGET';break;
    end
    W=stack(end);stack(end)=[];
    nodes=nodes+1;
    [ok,margin]=r79_leaf_support(W.cp,obs,rho);
    if ok
        certifiedLeaves=certifiedLeaves+1;
        worst=min(worst,margin);
        continue;
    end
    mid=W.t0+(W.t1-W.t0)/2;
    if ~(mid>W.t0&&mid<W.t1)
        reason='INCONCLUSIVE_FLOAT_INTERVAL_RESOLUTION';break;
    end
    [a,b]=r79_split_cp(W.cp);
    stack(end+1)=struct('cp',b,'t0',mid,'t1',W.t1); %#ok<AGROW>
    stack(end+1)=struct('cp',a,'t0',W.t0,'t1',mid); %#ok<AGROW>
end
pass=isempty(stack)&&isempty(reason);
if pass
    status='CERTIFIED_CONTINUOUS_KNOWN_ELLIPSOIDS';
else
    status=reason;
    if isempty(status),status='INCONCLUSIVE_SUPPORT_PLANE';end
end
Q=struct('certified',pass,'status',status, ...
    'continuous_time',pass,'whole_body_ball_radius_m',rho, ...
    'leaves',certifiedLeaves,'nodes',nodes, ...
    'min_support_margin_m',worst, ...
    'limitations','NOT actuator/tension/observer/tracking-tube or post-rejoin nominal certificate');
end
function [pass,worst]=r79_leaf_support(P,obs,rho)
pass=true;worst=Inf;
for k=1:numel(obs)
    c=double(obs(k).center);R=double(obs(k).R);a=double(obs(k).axes);
    V=P-c;
    % Candidate normals obtained from hull vertices and centroid.  The
    % plane inequality itself (not normal selection) is the proof.
    N=[mean(V,2),V];
    good=false;best=-Inf;
    for ii=1:size(N,2)
        n=N(:,ii);nn=norm(n);
        if nn==0,continue;end
        n=n/nn;
        ell=sqrt(sum((a.*(R'*n)).^2));
        margin=min(n'*P)-n'*c-ell-rho;
        best=max(best,margin);
        if margin>0
            good=true;worst=min(worst,margin);break;
        end
    end
    if ~good
        pass=false;worst=min(worst,best);return;
    end
end
end
function [L,R]=r79_split_cp(P)
N=size(P,2);T=P;L=zeros(size(P));R=zeros(size(P));
L(:,1)=T(:,1);R(:,end)=T(:,end);
for j=2:N
    T=0.5*(T(:,1:end-1)+T(:,2:end));
    L(:,j)=T(:,1);
    R(:,N-j+1)=T(:,end);
end
end
