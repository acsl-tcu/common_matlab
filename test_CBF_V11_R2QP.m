function report=test_CBF_V11_R2QP(opts)
%TEST_CBF_V11_R2QP  R2候補の幾何CBFとハード入力制約を独立に試す。
% R2は検討候補であり、実際のCBF本体に適用していない。
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(opts,'full') && opts.full,states=1:4;ds=[0.8 1.5 100];
else,states=[1 2 3];ds=[1 100];end
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};
rows=cell(0,13);n=0;
for cid=states
 for si=1:numel(shapes)
  for ki=1:numel(kinds)
   for dist=ds
    [x19,u,obs]=CBF_V11_Examples(cid,s,kinds{ki},dist);
    x23=[x19;u(1);zeros(3,1)];
    try
      R=CBF_V11_R2CandidateQP(shapes{si},x23,s,obs);
      n=n+1;
      rows(n,:)={cid,shapes{si},kinds{ki},dist, ...
       R.validAffineAtPoint,R.maxEarlyInput,R.maxNonaffine, ...
       min(R.minPsi),R.hasUncontrollableViolation, ...
       R.solverStatus,R.solverExitflag,R.minResidual, ...
       numel(R.labels)};
    catch ME
      n=n+1;
      rows(n,:)={cid,shapes{si},kinds{ki},dist,false,NaN,NaN,NaN, ...
       true,['TEST_ERROR_' ME.identifier],NaN,NaN,0};
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames', ...
 {'stateId','shape','obstacleType','distance','inputAffineAtChosenOrders', ...
 'maxEarlyInput','maxNonaffine','minAuxiliaryPsi','uncontrollableRow', ...
 'solverStatus','exitflag','minQPResidual','nGeometryRows'});
writetable(report,fullfile(CBF_V11_OutputFolder(),'v11_r2_hard_qp_candidates.csv'));
fprintf('[V11] R2 [T4,tau] 候補QP：%d条件。不可行は隠しません。\n',height(report));
end
