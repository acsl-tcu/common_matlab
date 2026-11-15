function report=test_CBF_V12_R1Ground(opts)
%TEST_CBF_V12_R1GROUND R1 physical-input ground-only, second-order hard CBF.
% x19 model and decision u=[T;taux;tauy;tauz]. The CBP acts at hddot.
% This deliberately DOES NOT claim obstacle/rope safety or closed-loop invariance.
% Geometric C/B/D ground rows are jointly enforced; actual torque limits hold.
% The early auxiliary condition hdot+k1*h>=0 is checked independently.
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(opts,'full') && opts.full
    ids=0:4;altitudes=[0.15 0.20 0.35 0.50 1.2];
    velocities=[-1.2 -0.65 -0.25 0 0.25];
else
    ids=[0 2 3 4];altitudes=[0.15 0.35 0.50];
    velocities=[-0.65 -0.25 0];
end
shapes={'C','B','D'};gains=[1.2 3.0];
rows=cell(0,18);n=0;
for id=ids
 for alt=altitudes
  for vz=velocities
   for shapeIndex=1:3
    sh=shapes{shapeIndex};
    if id==0,[x19,uNom]=CBF_V12_PureHover(s);
    else,[x19,uNom]=CBF_V11_Examples(id,s,'sphere',100);end
    x19(10)=alt;x19(13)=vz;
    ob=struct('type','sphere','center',[100;0;alt],'radius',0.25);
    status='TEST_ERROR';errmsg='';A=zeros(0,4);b=[];
    hmin=NaN;psiMin=NaN;zeroFeas=NaN;QPvalid=0;
    maxNonaffine=NaN;maxEarly=NaN;optT=NaN;residual=NaN;
    try
      H0=CBF_V11_JetRows('R1',sh,x19,zeros(4,1),s,ob,2);
      [~,labels]=CBF_V11_JetRows('R1',sh,x19,zeros(4,1),s,ob,2);
      keep=strncmp(labels,'ground_',7);
      H0=H0(keep,:);ng=size(H0,1);nuis=zeros(ng,4);
      nonaffine=zeros(ng,4);early=zeros(ng,4);
      for ch=1:4
         du=zeros(4,1);du(ch)=1;
         HP=CBF_V11_JetRows('R1',sh,x19,du,s,ob,2);
         HM=CBF_V11_JetRows('R1',sh,x19,-du,s,ob,2);
         HP=HP(keep,:);HM=HM(keep,:);
         nuis(:,ch)=(HP(:,3)-HM(:,3)); % 2!*central diff
         nonaffine(:,ch)=2*abs(HP(:,3)+HM(:,3)-2*H0(:,3));
         early(:,ch)=max(abs(HP(:,1:2)-HM(:,1:2)),[],2);
      end
      maxNonaffine=max(nonaffine(:));maxEarly=max(early(:));
      hmin=min(H0(:,1));
      for gain=gains
       k1=gain;k2=gain;
       psi=H0(:,2)+k1*H0(:,1);psiMin=min(psi);
       b=-(2*H0(:,3)+(k1+k2)*H0(:,2)+k1*k2*H0(:,1));
       A=nuis;
       QPvalid=double(maxNonaffine<=1e-7 && maxEarly<=1e-8);
       % Keep the physical box EXACT, no clipping/no slack.
       lb=[s.input.thrustMin;s.input.torqueMin(:)];
       ub=[s.input.thrustMax;s.input.torqueMax(:)];
       zeroFeas=double(all(A*uNom>=b-1e-8) && ...
            all(uNom>=lb) && all(uNom<=ub));
       if ~QPvalid
         status='INPUT_AFFINITY_NOT_VALID';
       else
         W=diag([1,10,10,10]);qpopt=optimoptions('quadprog','Display','off');
         [sol,~,ef]=quadprog(2*W,-2*W*uNom,-A,-b,[],[],lb,ub,[],qpopt);
         if ef>0 && ~isempty(sol)
           residual=min([A*sol-b;sol-lb;ub-sol]);optT=sol(1);
           if residual>=-1e-6
             if hmin>=-1e-9 && psiMin>=-1e-9
               status='QP_FEASIBLE_INITIAL_SET_VALID';
             else
               status='QP_FEASIBLE_BUT_INITIAL_SET_INVALID';
             end
           else
             status='QP_EXIT_SUCCESS_RESIDUAL_BAD';
           end
         else
           status='QP_INFEASIBLE_OR_SOLVER_FAILURE';
         end
       end
       n=n+1;
       rows(n,:)={id,alt,vz,sh,gain,status,ng,hmin,psiMin, ...
         QPvalid,maxEarly,maxNonaffine,zeroFeas,optT, ...
         uNom(1),residual,'GROUND_ONLY_NO_OBSTACLE',errmsg};
      end
    catch ME
      for gain=gains
       n=n+1;
       rows(n,:)={id,alt,vz,sh,gain,'TEST_ERROR',0,NaN,NaN,0,...
         NaN,NaN,NaN,NaN,uNom(1),NaN,'GROUND_ONLY_NO_OBSTACLE',...
         [ME.identifier ': ' ME.message]};
      end
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',...
 {'stateId','loadAltitude_m','loadVZ_m_s','shape','gain','status',...
 'nGroundRows','minGeometricH','minInitialPsi1','affineCheckPass',...
 'maxEarlyInputEffect','maxNonaffine','nominalBoxAndCBFFeasible',...
 'qpThrust_N','nominalThrust_N','minHardResidual',...
 'scopeWarning','errorText'});
writetable(report,fullfile(CBF_V11_OutputFolder(),'v12_r1_ground_hard_qp.csv'));
fprintf('[V12] R1 physical-input ground-only hard QP: %d cases.\n',height(report));
end
