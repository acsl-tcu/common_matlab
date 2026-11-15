function out=CBF_V16_R2_HighOrderRectified(x23,s,opts)
%CBF_V16_R2_HIGHORDERRECTIFIED Ong et al. higher-order ReCBF, r=6,
% specialized to the EXACT VERTICAL SIX-STATE LINEAR SURROGATE.
% q=[0;0;-1], level hover, no lateral/cable motion, thrust-only 4th integrator.
% This is NOT a global ReCBF for the physical 3D load/cable/drone model.
% Recursion: psi_i=Lf h_(i-1)+k*h_(i-1),
% h_i=h_(i-1)-ReLU(-c_i*psi_i^7); i=1..5.
% Uses exact 6th order Taylor jets so that h5dot depends AFFINELY on j=T4.
% ReCBF Theorem3 requires a regular set and globally valid weak-degree
% assumptions. Here L_g L_f^5 h=1/mTot everywhere in the surrogate.
if nargin<3,opts=struct();end
if ~isfield(opts,'k'),opts.k=1.2;end
if ~isfield(opts,'coeff'),opts.coeff=0.01*ones(5,1);end
if ~isfield(opts,'dt'),opts.dt=0.025;end
assert(numel(x23)==23 && opts.k>0 && opts.dt>0);
c=opts.coeff(:);assert(numel(c)==5&&all(c>0));
m=s.physical.massDrone+s.physical.massLoad;g=s.physical.gravity;
h=x23(10)-s.ground.height-s.shape.loadRadius-s.cbf.safetyMargin;
base=[h,x23(13),(x23(20)/m-g)/2,x23(21)/(6*m),...
      x23(22)/(24*m),x23(23)/(120*m)]; % j-dependent 6th coefficient appended below
[H,d0,stages,near]=localCompute(0);
[~,dp]=localCompute(1);[~,dm]=localCompute(-1);
A=(dp-dm)/2;nonAffine=abs(dp+dm-2*d0);
B=-(d0+opts.k*H);
% For nominal j, use a smooth fixed-gain controller of the vertical chain.
w=2;
y=[h-0.20;x23(13);x23(20)-m*g;x23(21:23)];
K=[m*w^6,6*m*w^5,15*w^4,20*w^3,15*w^2,6*w];
jNom=-K*y;
% Next-sample physical thrust endpoints: hard admissible interval for held j.
Tbase=x23(20)+opts.dt*x23(21)+opts.dt^2*x23(22)/2+...
 opts.dt^3*x23(23)/6;
jlow=24*(s.input.thrustMin-Tbase)/opts.dt^4;
jhigh=24*(s.input.thrustMax-Tbase)/opts.dt^4;
low=jlow;high=jhigh;tol=1e-10;
if nonAffine>1e-7
    state='NONAFFINE_OR_BRANCH_DISCONTINUITY';j=NaN;
elseif abs(A)<=1e-10
    if B>tol,state='ZERO_AUTHORITY_POSITIVE_RHS';j=NaN;
    else,state='INACTIVE_RECTIFIED_CONSTRAINT';
        % This is scalar constrained QP projection, NOT safety clipping.
        j=min(max(jNom,low),high);
    end
else
    if A>0,low=max(low,B/A);else,high=min(high,B/A);end
    if low>high+tol,state='INFEASIBLE_HARD_SNAP_INTERVAL';j=NaN;
    else,j=min(max(jNom,low),high);state='SURROGATE_1D_ENDPOINT_THRUST_QP_FEASIBLE';end
end
modelJetMismatch=NaN;
try
  checkObs=struct('type','sphere','center',[100;0;x23(10)],'radius',0.25);
  [physJets,physLabels]=CBF_V11_JetRows('R2','B',x23,zeros(4,1),s,checkObs,6);
  target=find(strcmp(physLabels,'ground_load'),1);
  if ~isempty(target)
      % Reference vertical Taylor jet, j=0, compared with exact full3D jets.
      verticalJet=[base,0];
      modelJetMismatch=max(abs((physJets(target,:)-verticalJet).*factorial(0:6)));
  end
catch
  modelJetMismatch=NaN;
end
if isfinite(j)
    actualResidual=A*j-B;
else,actualResidual=NaN;end
out=struct('hGround',h,'hRectified',H,'initialRectifiedValid',H>=0,...
 'activeStages',sum(stages<0),'stageValues',stages,'nearActivation',near,...
 'full3DLoadGroundJetMismatch',modelJetMismatch,...
 'A',A,'b',B,'nonaffine',nonAffine,'jNom',jNom,'jChosen',j,...
 'QPResidual',actualResidual,'status',state,'jLower',jlow,'jUpper',jhigh,...
 'scope','THEOREM3_FORM_ON_VERTICAL_R6_SURROGATE_ONLY_NO_FULL3D_CERTIFICATE');
    function [Hout,hdot,psiInit,ambiguous]=localCompute(jj)
        hjet=[base,jj/(720*m)]; % Taylor coefficients [h,h',...,h^(6)/6!]
        psiInit=zeros(5,1);ambiguous=0;
        for ii=1:5
            len=numel(hjet);
            wjet=(1:len-1).*hjet(2:end)+opts.k*hjet(1:end-1);
            psiInit(ii)=wjet(1);
            if abs(wjet(1))<1e-9,ambiguous=ambiguous+1;end
            if wjet(1)<0
                qjet=1;
                for power=1:7
                    qjet=conv(qjet,-wjet);
                    qjet=qjet(1:min(numel(qjet),len-1));
                end
                penalty=c(ii)*qjet(1:len-1);
            else
                penalty=zeros(1,len-1);
            end
            hjet=hjet(1:end-1)-penalty;
        end
        Hout=hjet(1);hdot=hjet(2);
    end
end
