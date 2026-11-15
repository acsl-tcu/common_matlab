function tbl=test_CBF_V16_R1ZOCBF(opts)
%TEST_CBF_V16_R1ZOCBF Zero-order sampled filter with DIRECT physical T,tau.
% Compare with R2 in v16_r2_rectified_backstep_zocbf.csv (same geometry).
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if opts.full,ids=[0 2 3 4];dset=[0.45 0.65 100];kinds={'sphere','ellipsoid'};
else,ids=[0 2];dset=[0.45 100];kinds={'sphere'};end
shapes={'C','B','D'};R=cell(0,12);n=0;
for id=ids
 for sh=shapes
  for typ=kinds
   for d=dset
    if id==0,[x,~]=CBF_V12_PureHover(s);else,[x,~]=CBF_V11_Examples(id,s,typ{1},d);end
    if id==0,x(10)=0.35;else,x(10)=0.50;end
    if strcmp(typ{1},'sphere')
        ob=struct('type','sphere','center',[d;0;x(10)],'radius',0.25);
    else
        th=pi/5;rot=[cos(th),-sin(th),0;sin(th),cos(th),0;0,0,1];
        ob=struct('type','ellipsoid','center',[d;0;x(10)],...
          'axes',[0.25;0.38;0.50],'R',rot);
    end
    status='TEST_ERROR';res=NaN;ef=NaN;minh=NaN;mint=NaN;
    T=NaN;torque=NaN;ms=NaN;err='';
    try
       z=CBF_V16_ZOCBFStep('R1',sh{1},x,s,ob,struct('dt',0.025,...
          'gamma',0.2,'substeps',3,'maxIterations',35));
       status=z.status;res=z.hardSampleResidual;ef=z.solverExitflag;
       minh=z.minGeometricH;mint=z.minTension;T=z.decision(1);
       torque=norm(z.decision(2:4),Inf);ms=z.timeMilliseconds;
       err=z.errorText;
    catch ME,err=[ME.identifier ':' ME.message];end
    n=n+1;R(n,:)={id,sh{1},typ{1},d,status,res,ef,minh,mint,T,torque,ms};
   end
  end
 end
end
tbl=cell2table(R,'VariableNames',{'stateId','shape','obstacleType',...
 'obstacleDistance_m','status','hardSampleResidual','exitflag',...
 'minGeometryH','minTension_N','physicalThrust_N','physicalTorqueInf','solveMs'});
writetable(tbl,fullfile(CBF_V11_OutputFolder(),'v16_r1_zocbf_sampled.csv'));
fprintf('[V16 R1] %d sampled ZOCBF comparison rows, no intersample theorem.\n',height(tbl));
end
