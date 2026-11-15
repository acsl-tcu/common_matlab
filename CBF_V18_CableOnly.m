function [H,meta]=CBF_V18_CableOnly(mode,x,u,s,obs,K)
%CBF_V18_CABLEONLY One cable outer ellipsoid vs ONE static rotated ellipsoid.
% ONLY obstacle_cable_ellipsoid. No ground, drone sphere, load sphere.
% Same FULL 3D original equations in CBF_V11_RealTaylor for R1 and R2.
% VIRTUAL uses corrected HLC flat-output cable jet (max K=4).
% H(:,k+1)=h^(k)/k!. h>=0 is a conservative separating-plane test.
% Requires the installed cbf_jet primitives and V11/V12 test functions.
mode=upper(char(mode));x=x(:);u=u(:);
assert(numel(u)==4 && any(strcmp(mode,{'R1','R2','VIRTUAL'})));
assert(strcmpi(obs.type,'ellipsoid'),'V18: Only ellipsoid obstacles are allowed.');
assert(isfield(obs,'center')&&numel(obs.center)==3&&numel(obs.axes)==3&&...
   isequal(size(obs.R),[3,3])&&all(obs.axes(:)>0));
assert(all(isfinite(obs.R(:)))&&norm(obs.R.'*obs.R-eye(3),'fro')<1e-8&&det(obs.R)>0);
assert(K>=1&&K<=6&&K==floor(K));
if strcmp(mode,'VIRTUAL')
    assert(numel(x)==20 && K<=4,'Virtual cable geometry supports K<=4 only.');
    [pL,qc]=CBF_V12_VirtualJetFixed(x,u,cbf_parameter_vector(s),K);
else
    assert((strcmp(mode,'R1')&&numel(x)==19)||(strcmp(mode,'R2')&&numel(x)==23));
    Z=CBF_V11_RealTaylor(mode,x,u,s,K);
    pL=Z(8:10,:);qc=Z(14:16,:);
end
assert(abs(norm(qc(:,1))-1)<1e-8,'Cable q must be a unit vector.');
L=s.physical.cableLength;rc=s.shape.cableRadius; dm=s.cbf.safetyMargin;
epss=s.cbf.smoothEpsilon;
assert(L>0&&rc>0&&dm>=0&&epss>=0);
a=L/2+rc;b=sqrt(a*rc); % documented D outer ellipsoid, contains capsule
C=@(v)cbf_jet('const',v,K);
Dot=@(v,w)cbf_jet('dot',v,w);
Mul=@(v,w)cbf_jet('mul',v,w);
Sqrt=@(v)cbf_jet('sqrt',v);
c=pL-(L/2)*qc;
d=C(obs.center(:))-c;
nd=Dot(d,d);
dq=Dot(d,qc);
supportC=Sqrt(b*b*nd+(a*a-b*b)*Mul(dq,dq)+C(epss^2));
rd=obs.R.'*d;
ar=obs.axes(:).*rd;
supportO=Sqrt(Dot(ar,ar)+C(epss^2));
smoothDist=Sqrt(nd+C(epss^2));
H=nd-supportC-supportO-dm*smoothDist;
assert(isequal(size(H),[1,K+1])&&all(isfinite(H(:))),'V18: bad cable ellipsoid jet');
meta=struct('label','obstacle_cable_ellipsoid_only','cableCenter',c(:,1),...
    'cableAxis',qc(:,1),'semiMajor',a,'semiMinor',b,'obstacleCenter',obs.center(:),...
    'h',H(1),'smoothing','CONSERVATIVE_SUPPORT_EPSILON','scope',...
    'FULL3D_ONE_CABLE_ELLIPSOID_ONE_STATIC_ELLIPSOID_NO_OTHER_GEOMETRY');
end
