function [H,meta]=CBF_V22_CableJet21(x,u,s,obs,K)
% Same conservative cable-outer-ellipsoid vs rotated obstacle ellipsoid
% as CBF_V18_CableOnly, but uses the new full-3D R2 21-state plant.
assert(numel(x)==21 && numel(u)==4 && K>=1 && K<=6 && K==floor(K));
assert(strcmpi(obs.type,'ellipsoid'));
assert(all(obs.axes(:)>0) && isequal(size(obs.R),[3,3]));
assert(norm(obs.R.'*obs.R-eye(3),'fro')<1e-8 && det(obs.R)>0);
Z=CBF_V22_R2TwoTaylor(x,u,s,K);
pL=Z(8:10,:);qc=Z(14:16,:);
L=s.physical.cableLength;rc=s.shape.cableRadius;
dm=s.cbf.safetyMargin;epss=s.cbf.smoothEpsilon;
a=L/2+rc;b=sqrt(a*rc);
C=@(v)cbf_jet('const',v,K);
Dot=@(v,w)cbf_jet('dot',v,w);
Mul=@(v,w)cbf_jet('mul',v,w);
Sqrt=@(v)cbf_jet('sqrt',v);
c=pL-(L/2)*qc;
d=C(obs.center(:))-c;
nd=Dot(d,d);dq=Dot(d,qc);
supC=Sqrt(b*b*nd+(a*a-b*b)*Mul(dq,dq)+C(epss^2));
rd=obs.R.'*d;ar=obs.axes(:).*rd;
supO=Sqrt(Dot(ar,ar)+C(epss^2));
H=nd-supC-supO-dm*Sqrt(nd+C(epss^2));
assert(all(isfinite(H(:))) && size(H,2)==K+1);
meta=struct('h',H(1),'cableCenter',c(:,1),'cableAxis',qc(:,1), ...
    'semiMajor',a,'semiMinor',b, 'scope','3D_21_STATE_CABLE_ONLY');
end
