function tbl=test_CBF_V16_VirtualGuard(opts)
%TEST_CBF_V16_VIRTUALGUARD HLC inverse + corrected 4/6 geometry + QP audit.
% A~=0 numeric noise with b>0 is an ERROR; do not omit the safety row.
% Guard is a diagnostic, NOT a proof that true exact coefficient is zero.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if opts.full,ids=[0 2 3 4];ds=[0.65 1 100];
else,ids=[0 2];ds=[0.65 100];end
shapes={'C','B','D'};types={'sphere','ellipsoid'};
R=cell(0,19);n=0;
for id=ids
 if id==0,[x,u]=CBF_V12_PureHover(s);
 else,[x,u]=CBF_V11_Examples(id,s,'sphere',1);end
 try
   jets=CBF_V11_RealTaylor('R1',x,u,s,6);
   xv=zeros(20,1);
   for k=0:5,xv(3*k+1:3*k+3)=factorial(k)*jets(8:10,k+1);end
   nuNom=[factorial(6)*jets(8:10,7);0];
 catch ME
   fprintf('[V16 Virtual] State %d Taylor failed: %s\n',id,ME.message);continue;
 end
 for shape=shapes
  for typ=types
   for dist=ds
     [~,~,obs]=CBF_V11_Examples(1,s,typ{1},dist);
     state='TEST_ERROR';err='';nrow=NaN;minPsi=NaN;minH=NaN;
     nInvalid=NaN;coefMin=NaN;minB=NaN;minRes=NaN;
     beta=NaN;thrust=NaN;maxnu=NaN;hardPhysical=NaN;
     earlyErr=NaN;nonAffErr=NaN;
     try
       [H,labels,orders]=CBF_V12_VirtualRows(shape{1},xv,zeros(4,1),s,obs);
       nrow=numel(labels);A=zeros(nrow,4);b=zeros(nrow,1);
       psiMin=zeros(nrow,1);early=0;nonAff=0;
       for j=1:4
           e=zeros(4,1);e(j)=1;
           HP=CBF_V12_VirtualRows(shape{1},xv,e,s,obs);
           HM=CBF_V12_VirtualRows(shape{1},xv,-e,s,obs);
           for ii=1:nrow
               ord=orders(ii);
               A(ii,j)=factorial(ord)*(HP(ii,ord+1)-HM(ii,ord+1))/2;
               nonAff=max(nonAff,abs(factorial(ord)*(HP(ii,ord+1)+HM(ii,ord+1)-2*H(ii,ord+1))));
               low=factorial(0:ord-1).*(HP(ii,1:ord)-HM(ii,1:ord))/2;
               early=max(early,max(abs(low)));
           end
       end
       for ii=1:nrow
           ord=orders(ii);gain=s.cbf.hocbfGain;
           if strncmp(labels{ii},'ground_',7) && isfield(s.cbf,'groundGain') && ~isempty(s.cbf.groundGain)
               gain=s.cbf.groundGain;
           end
           der=H(ii,1:ord+1).*factorial(0:ord);
           rhs=der(ord+1);
           for j=0:ord-1
               rhs=rhs+nchoosek(ord,j)*gain^(ord-j)*der(j+1);
           end
           b(ii)=-rhs;
           mn=inf;
           for j=0:ord-1
             v=0;
             for kk=0:j,v=v+nchoosek(j,kk)*gain^(j-kk)*der(kk+1);end
             mn=min(mn,v);
           end
           psiMin(ii)=mn;
       end
       earlyErr=early;nonAffErr=nonAff;
       minPsi=min(psiMin);minH=min(H(:,1));minB=min(b);
       norms=sqrt(sum(A.^2,2));coefMin=min(norms);
       % Numerical guard only: do NOT delete rows, and distinguish from
       % analytic proof. A tiny numerical row can fake a huge QP solution.
       nInvalid=sum(norms<=1e-10 & b>1e-8);
       map=CBF_V12_VirtualPhysicalMap(x,xv,nuNom,s,opts);
       beta=map.betaRcond;thrust=map.thrust;
       if early>1e-7 || nonAff>1e-5
           state='NONAFFINE_OR_EARLY_INPUT';
       elseif nInvalid>0
           state='NUMERIC_ZERO_COEFFICIENT_POSITIVE_RHS_NO_QP';
       elseif minPsi<0 || minH<0
           state='INITIAL_AUXILIARY_OR_GEOMETRY_INVALID';
       elseif ~map.thrustWithinLimits || map.affineRelative>1e-6
           state='HLC_MAP_INVALID_OR_THRUST_OUTSIDE';
       else
           AA=[A;map.A];bb=[b;map.b];
           W=eye(4);qo=optimoptions('quadprog','Display','off');
           [v,~,ef]=quadprog(2*W,-2*W*nuNom,-AA,-bb,[],[],[],[],[],qo);
           if ef>0&&~isempty(v)&&all(isfinite(v))
               minRes=min(AA*v-bb);maxnu=norm(v,Inf);
               mm=CBF_V12_VirtualPhysicalMap(x,xv,v,s,opts);
               hardPhysical=min([mm.torque-s.input.torqueMin(:);...
                   s.input.torqueMax(:)-mm.torque;...
                   mm.thrust-s.input.thrustMin;s.input.thrustMax-mm.thrust]);
               if minRes>=-1e-7 && hardPhysical>=-1e-7
                   state='POINTWISE_PASS_STILL_NOT_DYNAMICALLY_CERTIFIED';
               else,state='QP_OR_INVERSE_RESIDUAL_FAILURE';end
           else,state='QP_FAILED';end
       end
     catch ME
         err=[ME.identifier ': ' ME.message];
     end
     n=n+1;R(n,:)={id,shape{1},typ{1},dist,state,nrow,minH,minPsi,...
         nInvalid,coefMin,minB,earlyErr,nonAffErr,beta,thrust,...
         maxnu,minRes,hardPhysical,err};
   end
  end
 end
end
tbl=cell2table(R,'VariableNames',{'stateId','shape','obstacleType','obstacleDistance_m',...
 'status','nRows','minGeometricH','minInitialPsi','nUncontrollableNumericalRows',...
 'smallestRowNorm','minimumRHS','maxEarlyInputEffect','maxNonaffinity',...
 'hlcBetaRcond','hlcPhysicalThrust','qpNuInf','minQPResidual','physicalHardResidual','errorText'});
writetable(tbl,fullfile(CBF_V11_OutputFolder(),'v16_virtual_corrected_hlc_degeneracy_guard.csv'));
fprintf('[V16 Virtual] %d corrected-Taylor HLC inverse scenarios, NO clipping/no ignored rows.\n',height(tbl));
end
