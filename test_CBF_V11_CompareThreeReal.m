function report=test_CBF_V11_CompareThreeReal(opts)
%TEST_CBF_V11_COMPARETHREEREAL
% R0=現行[T4,tau2], R1=[T,tau] (作用次数のみ), R2=[T4,tau] (試験QP)。
% 3つのシステム形状C/B/Dと障害物球/楕円体を比較。
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(opts,'full') && opts.full
 states=1:4;distances=[0.8,1.5,3,100];
else
 states=[1,2];distances=[1,100];
end
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};
row=cell(0,13);n=0;
for cid=states
 for si=1:3
  for ki=1:2
   for dist=distances
     [x19,u,obs]=CBF_V11_Examples(cid,s,kinds{ki},dist);
     x23=[x19;u(1);zeros(3,1)];x29=[x23;u(2:4);zeros(3,1)];
     R0status='NOT_RUN';R2status='NOT_RUN';R0psi=NaN;R2psi=NaN;
     R0deg=NaN;R2deg=NaN;geometryError=NaN;r0Ms=NaN;r2Ms=NaN;
     try
       t=tic;
       q0=cbf_hocbf_scene('real',shapes{si},x29,s,{obs});
       r0Ms=1000*toc(t);
       R0psi=min(cellfun(@(z)min(z),q0.psi));
       R0deg=sum(q0.uncontrollableViolation);
       % 旧実装を正しいと仮定せず、現行実装の可行性をそのまま記録。
       R0status=localLP(q0.A,q0.b);
     catch ME
       R0status=['ERROR_' ME.identifier];
     end
     try
       t=tic;
       q2=CBF_V11_R2CandidateQP(shapes{si},x23,s,obs);
       r2Ms=1000*toc(t);
       R2status=q2.solverStatus;R2psi=min(q2.minPsi);
       R2deg=sum(q2.hasUncontrollableViolation);
       [h0,~]=cbf_evaluate_case('real',shapes{si},obs.type,x29,s,obs);
       [H,~]=CBF_V11_JetRows('R2',shapes{si},x23,[0;0;0;0],s,obs,6);
       geometryError=norm(h0-H(:,1),Inf);
     catch ME
       R2status=['ERROR_' ME.identifier];
     end
     n=n+1;
     row(n,:)={cid,shapes{si},kinds{ki},dist,geometryError, ...
       R0status,R2status,R0psi,R2psi,R0deg,R2deg,r0Ms,r2Ms};
   end
  end
 end
end
report=cell2table(row,'VariableNames', ...
 {'stateId','shape','obstacleType','distance','sameGeometricHError', ...
 'R0SolverStatus','R2SolverStatus','R0MinPsi','R2MinPsi', ...
 'R0Uncontrollable','R2Uncontrollable','R0CoefficientMs','R2CoefficientMs'});
writetable(report,fullfile(CBF_V11_OutputFolder(),'v11_compare_R0_R2_CBD.csv'));
fprintf('[V11] R0現行・R2候補のQP比較：%d条件、R1は別途入力作用次数で比較。\n',height(report));
end

function status=localLP(A,b)
opts=optimoptions('linprog','Display','off');
[nu,~,flag]=linprog(zeros(size(A,2),1),-A,-b,[],[],[],[],opts);
if flag>0 && ~isempty(nu) && min(A*nu-b)>=-1e-6
 status='FEASIBLE_AT_THIS_STATE';
else
 status=sprintf('INFEASIBLE_OR_SOLVER_FAILURE_%d',flag);
end
end
