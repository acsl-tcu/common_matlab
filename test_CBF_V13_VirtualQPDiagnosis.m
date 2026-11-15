function [report,rowReport]=test_CBF_V13_VirtualQPDiagnosis(opts)
%TEST_CBF_V13_VIRTUALQPDIAGNOSIS Why 'QP feasible' can demand 1e14 inputs.
% Pointwise geometry+torque constraints only. HLC thrust constraints at the
% PRESENT instant cannot constrain higher derivatives of v1 by themselves.
% Frozen-state, constant-input short-time thrust preview is DIAGNOSTIC ONLY.
% Does not substitute for a nonlinear closed-loop proof or certified reachability.
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(opts,'full')&&opts.full,ids=0:4;else,ids=[0 2 3 4];end
if ~isfield(opts,'previewDt'),opts.previewDt=0.01;end
if ~isscalar(opts.previewDt)||~isfinite(opts.previewDt)||opts.previewDt<=0
    error('CBF:V13Dt','previewDt must be positive and finite.');
end
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};distances=[1,100];
rows=cell(0,30);detail=cell(0,15);n=0;nr=0;
for id=ids
 if id==0,[x19,u]=CBF_V12_PureHover(s);
 else,[x19,u]=CBF_V11_Examples(id,s,'sphere',1);end
 Z=CBF_V11_RealTaylor('R1',x19,u,s,6);
 xv=zeros(20,1);
 for j=0:5,xv(3*j+1:3*j+3)=factorial(j)*Z(8:10,j+1);end
 nuNom=[factorial(6)*Z(8:10,7);0];
 for si=1:3
  for kindid=1:2
   for di=1:numel(distances)
    shape=shapes{si};kind=kinds{kindid};distance=distances(di);
    [~,~,ob]=CBF_V11_Examples(1,s,kind,distance);
    status='TEST_ERROR';errorText='';D=[];M=[];nrows=NaN;
    minH=NaN;minPsi=NaN;zeroFeasible=NaN;nomResidual=NaN;
    rankG=NaN;rankAll=NaN;rankTorque=NaN;torqueZnorm=NaN;minSv=NaN;maxSv=NaN;
    minNormCol=NaN;maxNormCol=NaN;zeroViolation=NaN;
    qpnorm=NaN;qpres=NaN;physicalAtQP=NaN;
    frozenEndT=NaN;frozenMaxT=NaN;frozenMinT=NaN;
    sol=nan(4,1);Aall=[];ball=[];
    try
      D=CBF_V13_VirtualCoefficientAudit(shape,xv,s,ob);
      nrows=numel(D.labels);minH=min(D.H0);minPsi=min(D.psiMin);
      M=CBF_V12_VirtualPhysicalMap(x19,xv,nuNom,s,opts);
      Aall=[D.A;M.A];ball=[D.b;M.b];
      zsg=svd(D.A,'econ');zsa=svd(Aall,'econ');
      rankG=sum(zsg>max(size(D.A))*eps(max(1,max(zsg))));
      rankAll=sum(zsa>max(size(Aall))*eps(max(1,max(zsa))));
      zst=svd(M.J,'econ');
      rankTorque=sum(zst>max(size(M.J))*eps(max(1,max(zst))));
      torqueZnorm=norm(M.J(:,3));
      minSv=min(zsa);maxSv=max(zsa);
      cn=sqrt(sum(D.A.^2,1));minNormCol=min(cn);maxNormCol=max(cn);
      zeroViolation=sum(sum(abs(D.A),2)<=1e-11 & D.b>1e-7);
      zeroFeasible=double(all(-ball>=-1e-8));
      nomResidual=min(Aall*nuNom-ball);
      if D.maxEarly>1e-7 || D.maxCurvature>1e-5
         status='CBF_ROW_INPUT_STRUCTURE_INVALID';
      elseif M.affineRelative>1e-6 || M.thrustVariationInNu>1e-7
         status='PHYSICAL_INVERSE_NOT_AFFINE';
      elseif ~M.thrustWithinLimits
         status='CURRENT_THRUST_OUT_OF_BOUNDS';
      else
         W=eye(4);qpopt=optimoptions('quadprog','Display','off');
         [sqp,~,ef]=quadprog(2*W,-2*W*nuNom,-Aall,-ball,[],[],[],[],[],qpopt);
         if ef>0 && ~isempty(sqp) && all(isfinite(sqp))
            sol=sqp;qpnorm=norm(sol,Inf);qpres=min(Aall*sol-ball);
            Mc=CBF_V12_VirtualPhysicalMap(x19,xv,sol,s,opts);
            physicalAtQP=min([Mc.torque-s.input.torqueMin(:);...
                s.input.torqueMax(:)-Mc.torque;Mc.thrust-s.input.thrustMin;...
                s.input.thrustMax-Mc.thrust]);
            try
               [frozenEndT,frozenMinT,frozenMaxT]=localFrozenThrust(x19,xv,sol,s,opts.previewDt);
            catch PRE
               errorText=['FROZEN_PREVIEW_ERROR: ' PRE.identifier ': ' PRE.message];
            end
            if qpres < -1e-6 || physicalAtQP < -1e-6
               status='QP_RESIDUAL_OR_PHYSICAL_BOX_FAILURE';
            elseif minH < -1e-9 || minPsi < -1e-9
               status='INITIAL_HOCBF_SET_INVALID';
            elseif qpnorm>=1e6
               status='QP_EXCESSIVE_DERIVATIVE_UNVERIFIED';
            elseif ~isfinite(frozenEndT)
               status='POINTWISE_FEASIBLE_FROZEN_PREVIEW_FAILED';
            else
               status='POINTWISE_FEASIBLE_PREVIEW_NOT_PROOF';
            end
         else
            status='QP_NOT_SOLVED';
         end
      end
    catch ME
      errorText=[ME.identifier ': ' ME.message];
      status='TEST_ERROR';
    end
    n=n+1;
    rows(n,:)={id,shape,kind,distance,status,nrows,minH,minPsi,...
       rankG,rankAll,rankTorque,torqueZnorm,minSv,maxSv,minNormCol,maxNormCol,zeroViolation,...
       zeroFeasible,nomResidual,qpnorm,sol(1),sol(2),sol(3),sol(4),qpres,physicalAtQP,...
       frozenEndT,frozenMinT,frozenMaxT,errorText};
    if ~isempty(D)
      for j=1:numel(D.labels)
        nr=nr+1;
        if all(isfinite(sol)),res=D.A(j,:)*sol-D.b(j);else,res=NaN;end
        detail(nr,:)={id,shape,kind,distance,D.labels{j},D.orders(j),...
          D.H0(j),D.psiMin(j),D.A(j,1),D.A(j,2),D.A(j,3),D.A(j,4),...
          D.b(j),res,max(D.early(j,:))};
      end
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',...
 {'stateId','shape','obstacleType','distance','status','nGeometryRows',...
  'minGeometryH','minInitialPsi','geometryRank','combinedRank',...
  'torqueMapRank','verticalSixthTorqueSensitivity',...
  'minCombinedSingularValue','maxCombinedSingularValue',...
  'minGeometryColumnNorm','maxGeometryColumnNorm','zeroControlPositiveRHSRows',...
  'zeroInputFeasible','nominalResidual','qpNuInf',...
  'qpLoadX6','qpLoadY6','qpLoadZ6','qpYaw2',...
  'qpResidual','physicalInputResidual',...
  'frozenPreviewThrustEnd_N','frozenPreviewMinThrust_N',...
  'frozenPreviewMaxThrust_N','errorText'});
rowReport=cell2table(detail,'VariableNames',...
 {'stateId','shape','obstacleType','distance','barrier','order',...
  'geometricH','minInitialPsi','A1','A2','A3','A4','rhsB',...
  'rowResidualAtQP','earlyInputEffect'});
out=CBF_V11_OutputFolder();
writetable(report,fullfile(out,'v13_virtual_qp_diagnostics.csv'));
writetable(rowReport,fullfile(out,'v13_virtual_constraint_rows.csv'));
fprintf('[V13] 仮想4/6次＋HLC入力上下限 %d条件、行別 %d行。\n',height(report),height(rowReport));
end

function [endT,minT,maxT]=localFrozenThrust(x19,xv,nu,s,dt)
% Necessary trend check only: x19 held frozen; v1 0..3 follows its Taylor
% polynomial under constant v1''''=nu_z. A real plant does NOT freeze x19.
common=fileparts(mfilename('fullpath'));root=fullfile(common,'controller');
if ~isfolder(fullfile(root,'@HLC_SUSPENDED_LOAD'))
   error('CBF:V13HLCMissing','Expected common_matlab/controller HLC folder.');
end
addpath(root,'-begin');obj=HLC_SUSPENDED_LOAD([],struct());
p=[s.physical.massDrone;s.physical.inertia(:);s.physical.gravity;...
   s.physical.massLoad;s.physical.cableLength;s.physical.disturbanceXY(:)].';
XD=zeros(1,28);XD(1:3)=x19(8:10).';
v=[xv(9);xv(12);xv(15);xv(18);nu(3)];
vals=zeros(3,1);
for j=1:3
    t=dt*(j-1)/2;
    v0=v(1)+v(2)*t+v(3)*t^2/2+v(4)*t^3/6+v(5)*t^4/24;
    v1=v(2)+v(3)*t+v(4)*t^2/2+v(5)*t^3/6;
    v2=v(3)+v(4)*t+v(5)*t^2/2;
    v3=v(4)+v(5)*t;
    VF=[v0,v1,v2,v3,v(5),0];
    tmp=obj.Uf_SuspendedLoadxyDst(x19,XD,VF,p);
    vals(j)=tmp(1);
end
endT=vals(end);minT=min(vals);maxT=max(vals);
end
