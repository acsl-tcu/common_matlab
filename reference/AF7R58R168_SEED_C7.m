function S=AF7R58R168_SEED_C7(route,J0,Jend,totalT)
% Build an all-XYZ C7 initial curve BEFORE any QP. Zero internal d1..d7;
% source J0/Jend remain unchanged. This is only a geometric starting point,
% not a physically feasible path or a live-flight authority.
S=struct('status','UNKNOWN_SEED','feasible',false,'control_points',{{}},...
 'times_s',[],'segment_durations_s',[]);
if ~isstruct(route)||~isfield(route,'payload_knots')||~isfield(route,'cells')||...
 ~isequal(size(J0),[3,8])||~isequal(size(Jend),[3,8])||...
 ~isscalar(totalT)||~isfinite(totalT)||totalT<=0,return;end
K=double(route.payload_knots);if size(K,1)~=3&&size(K,2)==3,K=K';end
n=size(K,2)-1;
if size(K,1)~=3||n<1||numel(route.cells)~=n||...
 any(~isfinite([K(:);J0(:);Jend(:)])),return;end
if max(abs(K(:,1)-J0(:,1)))>1e-9||...
 max(abs(K(:,end)-Jend(:,1)))>1e-9
 S.status='REJECT_SEED_ENDPOINT_POSITION_MISMATCH';return;
end
len=vecnorm(diff(K,1,2),2,1);
if any(~isfinite(len))||any(len<=0),return;end
h=totalT*len/sum(len);J=cell(1,n+1);
J{1}=J0;J{end}=Jend;
for s=2:n,J{s}=zeros(3,8);J{s}(:,1)=K(:,s);end
P=cell(1,n);
for s=1:n
 X=zeros(3,16);
 for k=0:7
  a=zeros(3,1);b=zeros(3,1);
  for j=0:k
   fall=1;if j>0,fall=prod(16-j:15);end
   gain=nchoosek(k,j)*h(s)^j/fall;
   a=a+gain*J{s}(:,j+1);
   b=b+(-1)^j*gain*J{s+1}(:,j+1);
  end
  X(:,k+1)=a;X(:,16-k)=b;
 end
 P{s}=X;
end
S.control_points=P;S.times_s=[0,cumsum(h)];S.segment_durations_s=h;
S.feasible=true;S.status='INITIAL_C7_SHARED_JET_SEED__NOT_CORRIDOR_OR_DYNAMICS_PROVEN';
end
