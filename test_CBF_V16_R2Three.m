function tbl=test_CBF_V16_R2Three(opts)
%TEST_CBF_V16_R2THREE R2 three approaches, common 19/23 physical state.
% ReCBF: actual R1 physical row and derivative-channel lift obstruction.
% Backstepping: proved-only-on-vertical-linear-surrogate invariant ellipsoid.
% ZOCBF: actual R2 RK4 sampled one-step filter and unproven delta.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if opts.full,ids=[0 2 3 4];dset=[0.45 0.65 100];kinds={'sphere','ellipsoid'};
else,ids=[0 2];dset=[0.45 100];kinds={'sphere'};end
shapes={'C','B','D'};R=cell(0,28);n=0;
for id=ids
 for shape=shapes
  for kind=kinds
   for distance=dset
    if id==0,[x,~]=CBF_V12_PureHover(s);
    else,[x,~]=CBF_V11_Examples(id,s,kind{1},distance);end
    if id==0,x(10)=0.35;else,x(10)=0.50;end
    T0=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
    xx=[x;T0;zeros(3,1)];
    if strcmp(kind{1},'sphere')
      ob=struct('type','sphere','center',[distance;0;x(10)],'radius',0.25);
    else
      Rq=[cos(pi/5),-sin(pi/5),0;sin(pi/5),cos(pi/5),0;0,0,1];
      ob=struct('type','ellipsoid','center',[distance;0;x(10)],...
         'axes',[0.25;0.38;0.50],'R',Rq);
    end
    status='OK';err='';H=NaN;rb=NaN;r1h=NaN;mis=NaN;
    rectRes=NaN;rectInitial=NaN;rectAff=NaN;
    zoRes=NaN;zoExit=NaN;zoMs=NaN;zoDecision=NaN;
    hrH=NaN;hrRes=NaN;hrJ=NaN;hrActive=NaN;hrMismatch=NaN;hrStatus='NOT_RUN';
    try
        ro=CBF_V16_R1_RectifiedGround(shape{1},x,s,struct('k',1.2));
        rectRes=ro.rectifiedQPResidual;
        rectInitial=double(ro.rectifiedInitialValid);
        rectAff=double(ro.affinePass);
    catch ME
        status='RECTIFIED_ERROR';err=[err ' RE:' ME.identifier ' ' ME.message];
    end
    try
        bs=CBF_V16_R2_CascadeEllipsoid(xx,s);
        H=bs.Hsurrogate;rb=bs.cbfResidual;mis=bs.modelAccelerationMismatch;
        r1h=bs.insideSurrogateSet;
    catch ME
        status='BACKSTEPPING_SURROGATE_ERROR';err=[err ' BS:' ME.identifier ' ' ME.message];
    end
    try
        hr=CBF_V16_R2_HighOrderRectified(xx,s);
        hrH=hr.hRectified;hrRes=hr.QPResidual;hrJ=hr.jChosen;
        hrActive=hr.activeStages;hrMismatch=hr.full3DLoadGroundJetMismatch;
        hrStatus=hr.status;
    catch ME
        err=[err ' HR:' ME.identifier ' ' ME.message];
    end
    try
        zo=CBF_V16_ZOCBFStep('R2',shape{1},xx,s,ob,struct('dt',0.025,...
            'gamma',0.2,'substeps',3,'maxIterations',35));
        zoRes=zo.hardSampleResidual;zoExit=zo.solverExitflag;
        zoMs=zo.timeMilliseconds;zoDecision=zo.decision(1);
        statusZ=zo.status;
    catch ME
        statusZ='TEST_ERROR';err=[err ' ZO:' ME.identifier ' ' ME.message];
    end
    n=n+1;R(n,:)={id,shape{1},kind{1},distance,status,...
      rectRes,rectInitial,rectAff,...
      H,rb,r1h,mis,hrH,hrRes,hrJ,hrActive,hrMismatch,hrStatus,...
      zoRes,zoExit,zoMs,zoDecision,statusZ,...
      0.025,0.2,0,0,err};
   end
  end
 end
end
tbl=cell2table(R,'VariableNames',{'stateId','shape','obstacleType','obstacleDistance_m',...
 'testStatus','r1RectifiedGroundQPResidual','r1RectifiedInitialValid','r1RectifiedAffineCheck',...
 'backstepSurrogateH','backstepSurrogateCBFResidual','backstepInsideSurrogateSet',...
 'backstepVerticalModelMismatch',...
 'highOrderRectifiedSurrogateH','highOrderRectifiedQpResidual',...
 'highOrderRectifiedSnap','highOrderRectifiedActivatedStages',...
 'highOrderRectifiedFull3DJetMismatch','highOrderRectifiedStatus','zocbfR2HardSampleResidual','zocbfExitflag',...
 'zocbfSolveMs','zocbfSnapDecision','zocbfStatus','zocbfSamplingPeriod_s',...
 'zocbfGamma','zocbfDeltaGround_UNPROVEN','zocbfDeltaObstacle_UNPROVEN','errorText'});
writetable(tbl,fullfile(CBF_V11_OutputFolder(),'v16_r2_rectified_backstep_zocbf.csv'));
fprintf('[V16 R2] 3-method STRUCTURAL comparison %d rows. No full3D safety certification.\n',height(tbl));
end
