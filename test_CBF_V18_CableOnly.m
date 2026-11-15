function tab=test_CBF_V18_CableOnly(opts)
% V18 focused test. R1 / R2-1 / R2-2 / R2-3 / virtual ALL use ONLY
% cable outer ellipsoid vs the ONE rotated ellipsoid obstacle.
% R2-1 / R2-2 are structural applicability audits, NOT completed controls.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if ~isfield(opts,'runZOCBF'),opts.runZOCBF=true;end
if ~isfield(opts,'runVirtual'),opts.runVirtual=true;end
test_CBF_SetupPaths();s=CBF_V18_TestSettings();
if opts.full,ids=[1 2 3 4]; ds=[.45,.55,.8,1.0];
else,ids=[1 3];ds=[.46,.75];end
rows=cell(0,15);
for id=ids
 [x19,uHover]=CBF_V11_Examples(id,s,'ellipsoid',1);
 x23=[x19;uHover(1);0;0;0];
 Z=CBF_V11_RealTaylor('R1',x19,uHover,s,6);
 xv=zeros(20,1);for jj=0:5,xv(3*jj+1:3*jj+3)=factorial(jj)*Z(8:10,jj+1);end
 nuNom=[factorial(6)*Z(8:10,7);0];
 for d=ds
   [~,~,obs]=CBF_V11_Examples(1,s,'ellipsoid',d);
   % One verified common h0 for R1 and R2, same cable geometry and state.
   h1=CBF_V18_CableOnly('R1',x19,uHover,s,obs,2);
   h2=CBF_V18_CableOnly('R2',x23,zeros(4,1),s,obs,6);
   if abs(h1(1)-h2(1))>1e-9,error('V18: R1/R2 geometry differs.');end
   % R2-1: 4th order torque vs 6th order snap input audit.
   u0=zeros(4,1);H0=CBF_V18_CableOnly('R2',x23,u0,s,obs,6);
   A4=zeros(1,4);A6=zeros(1,4);NL6=0;
   for j=1:4
      e=zeros(4,1);step=.001;if j==1,step=1;end;e(j)=step;
      hp=CBF_V18_CableOnly('R2',x23,u0+e,s,obs,6);
      hm=CBF_V18_CableOnly('R2',x23,u0-e,s,obs,6);
      A4(j)=factorial(4)*(hp(5)-hm(5))/(2*step);
      A6(j)=factorial(6)*(hp(7)-hm(7))/(2*step);
      NL6=max(NL6,abs(factorial(6)*(hp(7)+hm(7)-2*H0(7))));
   end
   % R2-2: no invented smooth upper safe control from this rank test.
   % Report prerequisites unproven; don't claim cascade is complete.
   try
      r1=CBF_V18_R1_RectifiedCable(x19,s,obs);
      rows(end+1,:)={'R1',id,d,h1(1),r1.status,r1.qpResidual,r1.thrustCoefficientH2,...
         NaN,NaN,NaN,NaN,NaN,NaN,'',r1.scope}; %#ok<AGROW>
   catch ME
      rows(end+1,:)={'R1',id,d,h1(1),'ERROR',NaN,NaN,...
         NaN,NaN,NaN,NaN,NaN,NaN,ME.message,''}; %#ok<AGROW>
   end
   try
      r21=CBF_V18_R2_1_RectifiedCable4(x23,s,obs);
      rows(end+1,:)={'R2-1_RECBF',id,d,h2(1),r21.status,r21.residual,NaN,...
          A4(2),A4(3),A4(4),A6(1),NL6,NaN,'',r21.scope}; %#ok<AGROW>
   catch ME
      rows(end+1,:)={'R2-1_RECBF',id,d,h2(1),'ERROR',NaN,NaN,...
         A4(2),A4(3),A4(4),A6(1),NL6,NaN,ME.message,''}; %#ok<AGROW>
   end
   % Backstepping prerequisite: a bounded pointwise safe upper T must
   % exist before asking for its 0..4 derivatives and cascade errors.
   try
      r22=CBF_V18_R1_RectifiedCable(x19,s,obs);
      if isfinite(r22.input(1))
          bstatus='UPPER_T_POINTWISE_EXISTS_BUT_SMOOTH_SAFE_LAW_UNPROVEN';
      else
          bstatus='UPPER_T_POINTWISE_INFEASIBLE_OR_INITIAL_INVALID';
      end
      rows(end+1,:)={'R2-2_BACKSTEPPING',id,d,h2(1),bstatus,...
          r22.qpResidual,r22.thrustCoefficientH2,...
          A4(2),A4(3),A4(4),A6(1),NaN,NaN,'',...
          'UPPER_T_PRECONDITION_ONLY_NO_BACKSTEPPING_CERTIFICATE'}; %#ok<AGROW>
   catch ME
      rows(end+1,:)={'R2-2_BACKSTEPPING',id,d,h2(1),'ERROR',NaN,NaN,...
          A4(2),A4(3),A4(4),A6(1),NaN,NaN,ME.message,''}; %#ok<AGROW>
   end
   if opts.runZOCBF
      try
       r3=CBF_V18_R2_ZOCable(x23,s,obs);
       rows(end+1,:)={'R2-3_ZOCBF',id,d,h2(1),r3.status,r3.hardResidual,NaN,...
          NaN,NaN,NaN,NaN,NaN,r3.milliseconds,r3.errorText,r3.scope}; %#ok<AGROW>
      catch ME
       rows(end+1,:)={'R2-3_ZOCBF',id,d,h2(1),'ERROR',NaN,...
          NaN,NaN,NaN,NaN,NaN,NaN,NaN,ME.message,''}; %#ok<AGROW>
      end
   end
   if opts.runVirtual
      try
        rv=CBF_V18_VirtualCable(x19,xv,nuNom,s,obs);
        rows(end+1,:)={'VIRTUAL',id,d,rv.h,rv.status,...
          NaN,NaN,NaN,NaN,NaN,NaN,rv.nonAffine,NaN,rv.errorText,rv.scope}; %#ok<AGROW>
      catch ME
        rows(end+1,:)={'VIRTUAL',id,d,h2(1),'ERROR',NaN,...
          NaN,NaN,NaN,NaN,NaN,NaN,NaN,ME.message,''}; %#ok<AGROW>
      end
   end
 end
end
tab=cell2table(rows,'VariableNames',{'mode','stateId','obstacleX','h0','status',...
  'minPointwiseResidual','R1ThrustH2','R2TauXH4','R2TauYH4','R2TauZH4',...
  'R2SnapH6','R2NonaffinityH6','solverMilliseconds','errorText','scope'});
writetable(tab,fullfile(CBF_V11_OutputFolder(),'v18_one_cable_vs_one_ellipsoid.csv'));
fprintf('[V18] %d focused scenario/mode records: one cable vs one ellipsoid ONLY.\n',height(tab));
end
