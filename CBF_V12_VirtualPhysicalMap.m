function M=CBF_V12_VirtualPhysicalMap(x19,xv,nuBase,s,opts)
%CBF_V12_VIRTUALPHYSICALMAP TEST ONLY: generated HLC inverse as physical box.
% nu=[pLx6,pLy6,pLz6,yawddot] at FIXED x19,xv, reference derivatives.
% v1^(0..3) determined by xv-ref, v1^(4)=nu_z-ref_z^(6).
% Vs=[nu_x-ref_x^(6); nu_y-ref_y^(6);nu_yaw-ref_yawddot].
% The generated Uf/Beta2/V2_alpha2 are evaluated at each candidate nu.
% Numerically verifies local/global affine structure BEFORE making QP rows.
if nargin<5,opts=struct();end
x19=x19(:);xv=xv(:);nuBase=nuBase(:);
if numel(x19)~=19||numel(xv)~=20||numel(nuBase)~=4
    error('CBF:V12InverseSize','x19=19,xv=20,nu=4 expected.');
end
common=fileparts(mfilename('fullpath'));
root=fullfile(common,'controller');
if isfield(opts,'hlcFolder') && ~isempty(opts.hlcFolder)
    root=char(opts.hlcFolder);
end
[~,last]=fileparts(root);
if strcmp(last,'@HLC_SUSPENDED_LOAD'),root=fileparts(root);end
if ~isfolder(fullfile(root,'@HLC_SUSPENDED_LOAD'))
    error('CBF:V12HLCMissing','Cannot find common_matlab/controller/@HLC_SUSPENDED_LOAD: %s',root);
end
addpath(root,'-begin');
obj=HLC_SUSPENDED_LOAD([],struct());
param=[s.physical.massDrone;s.physical.inertia(:);s.physical.gravity;...
   s.physical.massLoad;s.physical.cableLength;s.physical.disturbanceXY(:)].';
% Reference positions match the present physical load position. Its higher
% derivatives and yaw reference are zero in THIS audit (no assumed ref filter).
XD=zeros(1,28);XD(1:3)=x19(8:10).';
v1lower=[xv(9);xv(12);xv(15);xv(18)].'; % z^(2),z^(3),z^(4),z^(5)
% The generated HLC accepts 6 slots; the sixth is not used by V2_alpha2.
VF=[v1lower,nuBase(3),0];
baseThrust=obj.Uf_SuspendedLoadxyDst(x19,XD,VF,param);
baseBeta=obj.Beta2_SuspendedLoadxyDst(x19,XD,VF,param);
if ~isequal(size(baseBeta),[3,3])
    error('CBF:V12BetaSize','Beta2 must be 3x3.');
end
betaRcond=rcond(baseBeta);
if ~isfinite(betaRcond) || betaRcond<1e-12
    error('CBF:V12BetaSingular','HLC Beta2 is singular / badly conditioned.');
end
[T0,tau0]=localPhysical(nuBase);
step=[0.1;0.1;0.1;0.1];J=zeros(3,4);
for j=1:4
    diff=zeros(4,1);diff(j)=step(j);
    [~,tp]=localPhysical(nuBase+diff);
    [~,tm]=localPhysical(nuBase-diff);
    J(:,j)=(tp-tm)/(2*step(j));
end
probe=[0.13;-0.09;0.17;0.07];
[T1,tau1]=localPhysical(nuBase+probe);
nonAffine=norm(tau1-(tau0+J*probe),Inf);
nonAffineRelative=nonAffine/max(1,norm(tau1,Inf));
thrustChanging=abs(T1-T0);
% Physical hard bounds in nu coordinates. No clipping, no slack.
A=[J;-J];
lo=s.input.torqueMin(:);hi=s.input.torqueMax(:);
b=[lo-tau0+J*nuBase;-hi+tau0-J*nuBase];
Tinside=double(T0>=s.input.thrustMin-1e-8 && ...
   T0<=s.input.thrustMax+1e-8);
M=struct('A',A,'b',b,'thrust',T0,'torque',tau0, ...
 'thrustWithinLimits',Tinside,'torqueWithinLimits',...
 double(all(tau0>=lo-1e-8)&all(tau0<=hi+1e-8)), ...
 'affineResidual',nonAffine,'affineRelative',nonAffineRelative,...
 'thrustVariationInNu',thrustChanging,'betaRcond',betaRcond,...
 'J',J,'nuBase',nuBase,'hlcRoot',root,...
 'source','GENERATED_HLC_INVERSE_TEST_ONLY');

    function [T,tau]=localPhysical(nu)
        vf=[v1lower,nu(3),0];
        vs=[nu(1);nu(2);nu(4)]; % reference(6/yaw2)=0 in this test
        Uf=obj.Uf_SuspendedLoadxyDst(x19,XD,vf,param);
        Beta=obj.Beta2_SuspendedLoadxyDst(x19,XD,vf,param);
        rhs=obj.V2_alpha2_SuspendedLoadxyDst(x19,XD,vf,vs,param);
        if numel(Uf)<1||~isequal(size(Beta),[3 3])||numel(rhs)~=3
            error('CBF:V12HLCOutput','Generated HLC inverse output size mismatch.');
        end
        T=Uf(1);tau=Beta\rhs(:);
        if any(~isfinite([T;tau]))
            error('CBF:V12HLCNonFinite','Generated HLC inverse returned nonfinite.');
        end
    end
end
