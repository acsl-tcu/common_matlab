function report=test_CBF_V15_R1R2Predictive(opts)
%TEST_CBF_V15_R1R2PREDICTIVE Compare R1/R2 with direct hard C/B/D geometry.
% IMPORTANT: limited-horizon nonlinear prediction, not validated CBF theorem.
% No motor/torque clipping, no slack, no torque augmentation.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if ~isfield(opts,'horizon'),opts.horizon=0.35;end
if ~isfield(opts,'nSegments'),opts.nSegments=7;end
if ~isfield(opts,'substeps'),opts.substeps=2;end
if ~isfield(opts,'maxIterations'),opts.maxIterations=45;end
test_CBF_SetupPaths();s=test_CBF_Fixture();
shapeNames={'C','B','D'};
if opts.full
    cases=[0 1 2];distances=[0.45 0.65 100];kinds={'sphere','ellipsoid'};
else
    cases=[0 2];distances=[0.45 100];kinds={'sphere'};
end
R=cell(0,19);n=0;
for caseId=cases
  for si=1:numel(shapeNames)
    for kindId=1:numel(kinds)
      for distance=distances
        if caseId==0
            [x19,~]=CBF_V12_PureHover(s);
            x19(10)=0.35;
        else
            [x19,~]=CBF_V11_Examples(caseId,s,kinds{kindId},distance);
            x19(10)=0.50;
        end
        if strcmp(kinds{kindId},'sphere')
            obs=struct('type','sphere','center',[distance;0;x19(10)],'radius',0.25);
        else
            obs=struct('type','ellipsoid','center',[distance;0;x19(10)], ...
                'axes',[0.25;0.38;0.50], 'R',eye(3));
        end
        T0=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
        % Identical physical pose and current thrust for R1 and R2.
        x23=[x19;T0;zeros(3,1)];
        for mid=1:2
            if mid==1,mode='R1';xx=x19;
            else,mode='R2';xx=x23;end
            try
                out=CBF_V15_PredictiveFilter(mode,shapeNames{si},xx,s,{obs},opts);
                n=n+1;
                R(n,:)={caseId,mode,shapeNames{si},kinds{kindId},distance, ...
                    opts.horizon,out.status,out.exitflag, ...
                    out.minConstraintResidual,out.minimumGeometricH, ...
                    out.minimumTension_N,out.minimumThrust_N,out.maximumThrust_N, ...
                    out.decision(1),norm(out.decision(2:4)),out.solverMilliseconds, ...
                    out.nSamples,out.snapDecisionScaleForCost,out.errorText};
            catch ME
                n=n+1;
                R(n,:)={caseId,mode,shapeNames{si},kinds{kindId},distance, ...
                    opts.horizon,'TEST_ERROR',-99,NaN,NaN,NaN,NaN,NaN, ...
                    NaN,NaN,NaN,0,NaN,[ME.identifier ': ' ME.message]};
            end
        end
      end
    end
  end
end
report=cell2table(R,'VariableNames',{ ...
  'stateId','mode','shape','obstacleType','obstacleDistance_m','predictionHorizon_s', ...
  'status','solverExitflag','minimumHardResidual','minimumGeometricH', ...
  'minimumTension_N','minimumThrust_N','maximumThrust_N', ...
  'chosenFirstDecision','chosenTorqueNorm','solverMilliseconds', ...
  'numberOfGeometricChecks','snapCostScale','errorText'});
outdir=CBF_V11_OutputFolder();
writetable(report,fullfile(outdir,'v15_r1_r2_predictive_hard_geometry.csv'));
fprintf('[V15] R1/R2 prediction %d rows: hard sampled geometry/inputs, NOT continuous safety certificate.\n',height(report));
end
