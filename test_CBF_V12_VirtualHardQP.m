function report=test_CBF_V12_VirtualHardQP(opts)
%TEST_CBF_V12_VIRTUALHARDQP Corrected 4/6 jet CBF + HLC physical torque box.
% TEST-ONLY candidate: no guarantee that HLC flat inverse or auxiliary-set
% assumptions hold globally. Distinguishes QP residual, early psi, thrust,
% input affine map, and actual nonlinear inversion feasibility.
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(opts,'full') && opts.full,ids=0:4;ds=[1 100];
else,ids=[0 2 3 4];ds=[1 100];end
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};
rows=cell(0,19);n=0;
for id=ids
 if id==0,[x,u]=CBF_V12_PureHover(s);
 else,[x,u]=CBF_V11_Examples(id,s,'sphere',1);end
 Z=CBF_V11_RealTaylor('R1',x,u,s,6);
 xv=zeros(20,1);
 for k=0:5,xv(3*k+1:3*k+3)=factorial(k)*Z(8:10,k+1);end
 nuNom=[factorial(6)*Z(8:10,7);0];
 for si=1:3
  for ti=1:2
   for distance=ds
    shape=shapes{si};kind=kinds{ti};
    [~,~,ob]=CBF_V11_Examples(1,s,kind,distance);
    status='TEST_ERROR';err='';nrows=NaN;maxEarly=NaN;maxNL=NaN;
    minPsi=NaN;minH=NaN;betaRcond=NaN;torqueAffine=NaN;
    thrust=NaN;baseTauInf=NaN;QPnorm=NaN;QPminResidual=NaN;
    nominalViol=NaN;realInverseResidual=NaN;
    try
      H0=[];[H0,labels,orders]=CBF_V12_VirtualRows(shape,xv,zeros(4,1),s,ob);
      HP=zeros(size(H0,1),7,4);HM=HP;
      nrows=numel(labels);A=zeros(nrows,4);b=zeros(nrows,1);
      minP=zeros(nrows,1);early=zeros(nrows,4);nl=zeros(nrows,4);
      scale=[1;1;1;1];
      for j=1:4
         e=zeros(4,1);e(j)=scale(j);
         HP(:,:,j)=CBF_V12_VirtualRows(shape,xv,e,s,ob); %#ok<AGROW>
         HM(:,:,j)=CBF_V12_VirtualRows(shape,xv,-e,s,ob); %#ok<AGROW>
      end
      for i=1:nrows
       r=orders(i);gain=s.cbf.hocbfGain;
       if strncmp(labels{i},'ground_',7) && isfield(s.cbf,'groundGain') ...
           && ~isempty(s.cbf.groundGain),gain=s.cbf.groundGain;end
       f=factorial(0:r);der=H0(i,1:r+1).*f;
       bb=der(r+1);
       for k=0:r-1
          bb=bb+nchoosek(r,k)*gain^(r-k)*der(k+1);
       end
       b(i)=-bb;
       psi=zeros(r,1);
       for k=0:r-1
        for m=0:k
          psi(k+1)=psi(k+1)+nchoosek(k,m)*gain^(k-m)*der(m+1);
        end
       end
       minP(i)=min(psi);
       for j=1:4
        A(i,j)=factorial(r)*(HP(i,r+1,j)-HM(i,r+1,j))/2;
        early(i,j)=max(abs((HP(i,1:r,j)-HM(i,1:r,j))/2.*factorial(0:r-1)));
        nl(i,j)=factorial(r)*abs(HP(i,r+1,j)+HM(i,r+1,j)-2*H0(i,r+1));
       end
      end
      maxEarly=max(early(:));maxNL=max(nl(:));
      minPsi=min(minP);minH=min(H0(:,1));
      M=CBF_V12_VirtualPhysicalMap(x,xv,nuNom,s,opts);
      betaRcond=M.betaRcond;torqueAffine=M.affineRelative;
      thrust=M.thrust;baseTauInf=norm(M.torque,Inf);
      nominalViol=min([A*nuNom-b;M.A*nuNom-M.b]);
      if maxEarly>1e-7 || maxNL>1e-5
         status='GEOMETRY_INPUT_NOT_AFFINE_AT_DESIGN_ORDER';
      elseif M.affineRelative>1e-6 || M.thrustVariationInNu>1e-7
         status='HLC_PHYSICAL_MAP_NOT_AFFINE';
      elseif ~M.thrustWithinLimits
         status='THRUST_OUTSIDE_PHYSICAL_LIMIT';
      else
         Aall=[A;M.A];ball=[b;M.b];W=eye(4);
         qpopt=optimoptions('quadprog','Display','off');
         [sol,~,ef]=quadprog(2*W,-2*W*nuNom,-Aall,-ball,[],[],[],[],[],qpopt);
         if ef>0 && ~isempty(sol)
           QPnorm=norm(sol,Inf);QPminResidual=min(Aall*sol-ball);
           Mcandidate=CBF_V12_VirtualPhysicalMap(x,xv,sol,s,opts);
           realInverseResidual=min([Mcandidate.torque-s.input.torqueMin(:);...
               s.input.torqueMax(:)-Mcandidate.torque;...
               Mcandidate.thrust-s.input.thrustMin;...
               s.input.thrustMax-Mcandidate.thrust]);
           if QPminResidual>=-1e-6 && realInverseResidual>=-1e-6
             if minH>=0 && minPsi>=0
               status='POINTWISE_QP_FEASIBLE_INITIAL_SET_VALID';
             else
               status='POINTWISE_QP_FEASIBLE_INITIAL_SET_INVALID';
             end
           else
             status='QP_SUCCESS_BUT_RESIDUAL_FAILURE';
           end
         else
           status='QP_INFEASIBLE_OR_SOLVER_FAILURE';
         end
      end
    catch ME
        err=sprintf('%s: %s',ME.identifier,ME.message);
    end
    n=n+1;
    rows(n,:)={id,shape,kind,distance,status,nrows,minH,minPsi,...
      maxEarly,maxNL,betaRcond,torqueAffine,thrust,baseTauInf,...
      norm(nuNom,Inf),nominalViol,QPnorm,QPminResidual,...
      sprintf('nonlinearInputResidual=%g; %s',realInverseResidual,err)};
   end
  end
 end
end
report=cell2table(rows,'VariableNames',...
 {'stateId','shape','obstacleType','distance','status','nGeometryRows',...
  'minGeometricH','minInitialPsi','maxEarlyInput',...
  'maxGeometryNonaffine','betaRcond','physicalMapNonaffineRelative',...
  'physicalThrust_N','physicalNominalTorqueInf','nominalNuInf',...
  'nominalHardResidual','qpNuInf','minQPResidual','errorText'});
writetable(report,fullfile(CBF_V11_OutputFolder(),'v12_virtual_corrected_hlc_hard_qp.csv'));
fprintf('[V12] 修正仮想4/6次ジオメトリ＋HLC物理制約QP：%d条件\n',height(report));
end
