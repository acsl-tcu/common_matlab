function H=AF7R58R79_HEADING_CERT(C,heading_anchor_rad,max_nodes)
% Exact continuous-time constructive C6 yaw chart via adaptive Bernstein hulls.
% A positive (strict) lower bound n·p'(t) on every certified leaf guarantees
% nonzero horizontal velocity and a globally consistent atan2 branch.
assert(isstruct(C)&&isfield(C,'payload_cp')&& ...
    isequal(size(C.payload_cp),[3 16]),'[R79] invalid Bezier controls.');
assert(isnumeric(heading_anchor_rad)&&isscalar(heading_anchor_rad)&& ...
    isfinite(heading_anchor_rad),'[R79] external yaw chart anchor required.');
assert(isnumeric(max_nodes)&&isscalar(max_nodes)&&isfinite(max_nodes)&& ...
    max_nodes==fix(max_nodes)&&max_nodes>=1, ...
    '[R79] caller-provided finite proof work budget required.');
n=[cos(heading_anchor_rad);sin(heading_anchor_rad)];
stack=struct('cp',C.payload_cp,'t0',C.t0,'t1',C.t1);
nodes=0;leaves=0;gamma=Inf;reason='';
while ~isempty(stack)
    if nodes>=max_nodes
        reason='INCONCLUSIVE_HEADING_RESOURCE_BUDGET';break;
    end
    Z=stack(end);stack(end)=[];nodes=nodes+1;
    T=Z.t1-Z.t0;
    if ~(T>0 && isfinite(T))
        reason='INCONCLUSIVE_HEADING_INTERVAL';break;
    end
    D=(15/T)*diff(Z.cp(1:2,:),1,2);
    g=min(n'*D);
    if isfinite(g)&&g>0
        gamma=min(gamma,g);leaves=leaves+1;continue;
    end
    mid=Z.t0+(Z.t1-Z.t0)/2;
    if ~(mid>Z.t0&&mid<Z.t1)
        reason='INCONCLUSIVE_HEADING_FLOAT_RESOLUTION';break;
    end
    [a,b]=r79_yaw_split(Z.cp);
    stack(end+1)=struct('cp',b,'t0',mid,'t1',Z.t1); %#ok<AGROW>
    stack(end+1)=struct('cp',a,'t0',Z.t0,'t1',mid); %#ok<AGROW>
end
pass=isempty(stack)&&isempty(reason);
if pass
    status='CERTIFIED_TANGENT_YAW_C6_OVER_FULL_CONNECTOR';
else
    status=reason;
    if isempty(status),status='INCONCLUSIVE_TANGENT_YAW_CHART';end
end
H=struct('certified',pass,'status',status, ...
    'horizontal_forward_speed_lower_m_s',gamma, ...
    'anchor_rad',heading_anchor_rad,'smoothness_order',6, ...
    'continuous_time',pass,'nodes',nodes,'leaves',leaves, ...
    'limitations','C7 endpoint jets and single angular chart required across both joins');
end
function [L,R]=r79_yaw_split(P)
N=size(P,2);T=P;L=zeros(size(P));R=zeros(size(P));
L(:,1)=T(:,1);R(:,end)=T(:,end);
for j=2:N
    T=0.5*(T(:,1:end-1)+T(:,2:end));
    L(:,j)=T(:,1);R(:,N-j+1)=T(:,end);
end
end
