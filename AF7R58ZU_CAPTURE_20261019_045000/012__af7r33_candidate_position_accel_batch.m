function [pL,aL,valid] = af7r33_candidate_position_accel_batch(replanner,P,tg)
tg=double(tg(:).');
[Y,vb]=af7r33_active_flat_jet_batch(replanner,tg);
pL=Y(1:3,:); aL=Y(7:9,:); valid=logical(vb);

assert(isfield(P,'representation_code')&&double(P.representation_code)==2, ...
    '[AF7R33] candidate batch requires piecewise representation_code=2.');

C=struct('t0',double(P.t0),'t1',double(P.t1), ...
    'segment_t0',double(P.piecewise_segment_t0(:).'), ...
    'segment_t1',double(P.piecewise_segment_t1(:).'), ...
    'control_points',{P.piecewise_control_points});
mask=tg>C.t0 & tg<C.t1;
if any(mask)
    D=piecewise_d0_d2(tg(mask),C);
    pL(:,mask)=pL(:,mask)+D(:,:,1);
    aL(:,mask)=aL(:,mask)+D(:,:,2);
end
valid=valid & all(isfinite(pL),1) & all(isfinite(aL),1);
end

function DOUT=piecewise_d0_d2(t,C)
t=double(t(:).'); N=numel(t); DOUT=zeros(3,N,2);
s0=double(C.segment_t0(:)); s1=double(C.segment_t1(:)); CP=C.control_points;
for is=1:numel(CP)
    idx=find(t>=s0(is)&t<=s1(is)); if isempty(idx), continue; end
    T=s1(is)-s0(is); assert(isfinite(T)&&T>0);
    u=max(0,min(1,(t(idx)-s0(is))/T));
    B=double(CP{is});
    DOUT(:,idx,1)=decasteljau_batch(B,u);
    D1=(15/T)*diff(B,1,2);
    D2=(14/T)*diff(D1,1,2);
    DOUT(:,idx,2)=decasteljau_batch(D2,u);
end
end

function Y=decasteljau_batch(C,u)
u=reshape(double(u),1,1,[]);
T=repmat(double(C),1,1,numel(u));
while size(T,2)>1
    T=(1-u).*T(:,1:end-1,:)+u.*T(:,2:end,:);
end
Y=reshape(T(:,1,:),3,[]);
end
