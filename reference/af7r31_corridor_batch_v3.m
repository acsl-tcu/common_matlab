function [A,b,ok] = af7r31_corridor_batch_v3(replanner,P,CTX,EPI,episodeIndex,n,nvar)
ep=EPI.episodes(episodeIndex);
jj=double(ep.component_index);
tComp=double(EPI.components(jj).start_s);
LIV=CTX.live; O=CTX.obstacles;
SC=replanner.active_speed_certificate;
replanner.validate_active_speed_certificate(SC);

A=zeros(0,nvar); b=zeros(0,1); ok=false;
n=double(n(:)); nn=norm(n);
if ~(numel(n)==3&&all(isfinite(n))&&isfinite(nn)&&nn>0), return; end
n=n/nn;

ii=double(ep.obstacle_index);
c=double(O.centers(:,ii));
R=reshape(double(O.rotations_flat(:,ii)),3,3);
ax=double(O.base_axes(:,ii));
VL=double(SC.payload_speed_UB_m_s); VQ=double(SC.uav_speed_UB_m_s);

track=0;
if isfield(P,'execution_tracking_inflation_m')&&isfinite(double(P.execution_tracking_inflation_m))
    track=max(0,double(P.execution_tracking_inflation_m));
end
rhoL0=double(LIV.r_load)+double(LIV.d_margin)+track;
rhoQ0=double(LIV.r_drone)+double(LIV.d_margin)+track;
rhoC0=double(LIV.r_cable)+double(LIV.d_margin)+track;

pidx=(2*jj-2)*3+(1:3);
vidx=(2*jj-1)*3+(1:3);
dt=double(LIV.dt);
ta=double(ep.start_s); te=double(ep.end_s);

TA=zeros(0,1); TB=zeros(0,1);
while ta<te-8*eps(max(1,abs(te)))
    tb=min(te,ta+dt);
    if ~(tb>ta), break; end
    TA(end+1,1)=ta; TB(end+1,1)=tb; %#ok<AGROW>
    ta=tb;
end
if isempty(TA), return; end

tc=0.5*(TA+TB); half=0.5*(TB-TA);
[Y,vflat]=af7r31_flat_jet_batch_v3(replanner,tc.');
if ~all(vflat), return; end

pL=Y(1:3,:); aL=Y(7:9,:);
sf=aL+[zeros(2,numel(tc));double(LIV.gravity)*ones(1,numel(tc))];
ns=sqrt(sum(sf.^2,1));
if any(~isfinite(ns)|ns<=0), return; end
q=-sf./ns;
pQ=pL-double(LIV.cable_length)*q;

req=double(af7r31_geometry_batch_mex(1,pL,pQ,half.',c,R,ax,n, ...
    rhoL0,rhoQ0,rhoC0,VL,VQ));
req=req(:);
if any(~isfinite(req)), return; end

S=zeros(2*numel(TA),1); B=zeros(2*numel(TA),1);
S(1:2:end)=TA-tComp; S(2:2:end)=TB-tComp;
B(1:2:end)=req; B(2:2:end)=req;

keep=replanner.r10_upper_envelope_row_indices(S,B);
if isempty(keep), return; end

S=S(keep); B=B(keep);
nk=numel(S);
A=zeros(nk,nvar); b=B(:);
A(:,pidx)=repmat(n.',nk,1);
A(:,vidx)=S(:)*n.';
ok=all(isfinite(A),'all')&&all(isfinite(b));
end
