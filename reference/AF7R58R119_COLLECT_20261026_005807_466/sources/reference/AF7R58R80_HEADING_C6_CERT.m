function H=AF7R58R80_HEADING_C6_CERT(C,max_nodes)
% Continuous-time C6 tangent yaw via LOCAL adaptive charts, not one global
% half-plane. Handles large smooth direction changes and loop-like paths.
% Every accepted leaf has a Bernstein certificate n'v_xy >= gamma > 0.
% Nonvanishing C6 vector field on a time interval admits a C6 angle lift.
assert(isstruct(C)&&isfield(C,'payload_cp')&& ...
    isequal(size(C.payload_cp),[3,16])&& ...
    all(isfinite(C.payload_cp(:)))&&isfinite(C.t0)&& ...
    isfinite(C.t1)&&C.t1>C.t0,'[R80] invalid C7 connector.');
assert(isnumeric(max_nodes)&&isscalar(max_nodes)&&isfinite(max_nodes)&& ...
    max_nodes==fix(max_nodes)&&max_nodes>=1, ...
    '[R80] caller-authorized finite work budget required.');
H=struct('certified',false,'continuous_time',false,'smoothness_order',6, ...
    'status','INCONCLUSIVE','nodes',0,'leaves',0, ...
    'horizontal_speed_lower_m_s',NaN, ...
    'global_single_half_plane_required',false);
stack=struct('cp',double(C.payload_cp),'a',double(C.t0),'b',double(C.t1));
gamma=Inf;nodes=0;leaves=0;reason='';
while ~isempty(stack)
    if nodes>=max_nodes,reason='INCONCLUSIVE_RESOURCE_BUDGET';break;end
    S=stack(end);stack(end)=[];nodes=nodes+1;
    T=S.b-S.a;V=(15/T)*diff(S.cp(1:2,:),1,2);
    dirs=[mean(V,2),V];good=false;g=-Inf;
    for j=1:size(dirs,2)
        v=dirs(:,j);nv=norm(v);
        if ~(isfinite(nv)&&nv>0),continue;end
        n=v/nv;g=min(n'*V);
        if isfinite(g)&&g>0
            good=true;gamma=min(gamma,g);break;
        end
    end
    if good,leaves=leaves+1;continue;end
    mid=S.a+0.5*(S.b-S.a);
    if ~(mid>S.a&&mid<S.b),reason='INCONCLUSIVE_FLOAT_INTERVAL_RESOLUTION';break;end
    [P1,P2]=r80_yaw_split(S.cp);
    stack(end+1)=struct('cp',P2,'a',mid,'b',S.b); %#ok<AGROW>
    stack(end+1)=struct('cp',P1,'a',S.a,'b',mid); %#ok<AGROW>
end
H.nodes=nodes;H.leaves=leaves;
if isempty(stack)&&isempty(reason)
    H.certified=true;H.continuous_time=true;
    H.horizontal_speed_lower_m_s=gamma;
    H.status='CERTIFIED_LOCAL_CHART_TANGENT_YAW_C6';
else
    if isempty(reason),reason='INCONCLUSIVE_HORIZONTAL_REGULARITY';end
    H.status=reason;
end
end
function [A,B]=r80_yaw_split(P)
N=size(P,2);A=zeros(size(P));B=zeros(size(P));
A(:,1)=P(:,1);B(:,end)=P(:,end);
for k=2:N
    P=0.5*(P(:,1:end-1)+P(:,2:end));
    A(:,k)=P(:,1);B(:,N-k+1)=P(:,end);
end
end
