function OUT = CURRENT_C6_STEP3_COMMON_14CASE_THREE_BACKEND_MASTER_V8()
% Unified 14-case Step3 robustness audit.
%
% Fixed backend architectures from C02 V6:
%   BEZIER_D13 : literature baseline / Bernstein corridor certificate
%   POLY_D13   : minimal Hermite jets / Bernstein image certificate
%   BSPLINE_D7 : 28 CP / 21 spans / native-CP local convex-hull certificate
%
% No architecture tuning per case.
% No safety relaxation.
% Any negative original hard slack => FAIL.
% No text-file logging.

fprintf('========================================================================================================================\n');
fprintf(' STEP 3 COMMON / 14-CASE / THREE-BACKEND MASTER V8\n');
fprintf(' FIXED ARCHITECTURE ROBUSTNESS AUDIT | BEZIER_D13 | POLY_D13 | BSPLINE_D7\n');
fprintf('========================================================================================================================\n');

P0=aud_params(); %#ok<NASGU>
CASES0=aud_build_cases();
nCase=numel(CASES0);

requiredCaseFields={'case_id','case_desc','p0','pg','v0','e_nom','T','obs'};
for kk=1:numel(requiredCaseFields)
    if ~isfield(CASES0,requiredCaseFields{kk})
        error('MASTER14:CaseSchema','Missing case field: %s',requiredCaseFields{kk});
    end
end

ids={CASES0.case_id};
if numel(unique(ids))~=nCase
    error('MASTER14:CaseSchema','case_id values must be unique.');
end

REC=repmat(struct(),1,nCase);

for ic=1:nCase
    fprintf('\n\n########################################################################################################################\n');
    fprintf(' CASE %02d / %02d : %s\n',ic,nCase,CASES0(ic).case_id);
    fprintf('########################################################################################################################\n');

    fprintf('\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> BEZIER_D13 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n');
    B=run_bezier_d13(ic,false);

    fprintf('\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POLY_D13 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n');
    P=run_poly_d13(ic,false);

    fprintf('\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> BSPLINE_D7 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n');
    S=run_bspline_d7(ic,false);

    REC(ic).case_index=ic;
    REC(ic).case_id=CASES0(ic).case_id;
    REC(ic).case_desc=CASES0(ic).case_desc;
    REC(ic).bezier=B;
    REC(ic).poly=P;
    REC(ic).bspline=S;

    REC(ic).all_pass=B.pass && P.pass && S.pass;
    REC(ic).d13_max_sample_position_difference=norm(B.sample_position-P.sample_position,inf);
end

fprintf('\n\n========================================================================================================================\n');
fprintf(' 14-CASE THREE-BACKEND COMPACT SUMMARY\n');
fprintf('========================================================================================================================\n');
fprintf('case | Bpass Ppass Spass | Bsolve  Psolve  Ssolve [ms] | Bslack       Pslack       Sslack       | Bxi       Pxi       Sxi\n');

for ic=1:nCase
    B=REC(ic).bezier; P=REC(ic).poly; S=REC(ic).bspline;
    fprintf('%4d | %5d %5d %5d | %7.3f %7.3f %7.3f | %+11.3e %+11.3e %+11.3e | %8.4f %8.4f %8.4f\n', ...
        ic,B.pass,P.pass,S.pass, ...
        B.solve_time_ms,P.solve_time_ms,S.solve_time_ms, ...
        B.min_original_hard_slack,P.min_original_hard_slack,S.min_original_hard_slack, ...
        B.xi_chosen,P.xi_chosen,S.xi_chosen);
end

Bpass=[REC.bezier]; Bpass=[Bpass.pass];
Ppass=[REC.poly];    Ppass=[Ppass.pass];
Spass=[REC.bspline]; Spass=[Spass.pass];

fprintf('\n[AGGREGATE]\n');
fprintf('BEZIER pass count           = %d / %d\n',sum(Bpass),nCase);
fprintf('POLY pass count             = %d / %d\n',sum(Ppass),nCase);
fprintf('BSPLINE pass count          = %d / %d\n',sum(Spass),nCase);
fprintf('ALL-three pass count        = %d / %d\n',sum([REC.all_pass]),nCase);

Bsolve=arrayfun(@(r)r.bezier.solve_time_ms,REC);
Psolve=arrayfun(@(r)r.poly.solve_time_ms,REC);
Ssolve=arrayfun(@(r)r.bspline.solve_time_ms,REC);

fprintf('medianSolveMs [B P S]       = [%.6f %.6f %.6f]\n',median(Bsolve),median(Psolve),median(Ssolve));
fprintf('maxSolveMs    [B P S]       = [%.6f %.6f %.6f]\n',max(Bsolve),max(Psolve),max(Ssolve));

Bsl=arrayfun(@(r)r.bezier.min_original_hard_slack,REC);
Psl=arrayfun(@(r)r.poly.min_original_hard_slack,REC);
Ssl=arrayfun(@(r)r.bspline.min_original_hard_slack,REC);
fprintf('minHardSlack  [B P S]       = [%+.17e %+.17e %+.17e]\n',min(Bsl),min(Psl),min(Ssl));

d13=arrayfun(@(r)r.d13_max_sample_position_difference,REC);
fprintf('max D13 representation diff = %.17e\n',max(d13));

OUT=struct();
OUT.mode='STEP3_COMMON_14CASE_THREE_BACKEND_MASTER_V8';
OUT.records=REC;
OUT.aggregate=struct();
OUT.aggregate.n_cases=nCase;
OUT.aggregate.bezier_pass_count=sum(Bpass);
OUT.aggregate.poly_pass_count=sum(Ppass);
OUT.aggregate.bspline_pass_count=sum(Spass);
OUT.aggregate.all_three_pass_count=sum([REC.all_pass]);
OUT.aggregate.median_solve_ms=[median(Bsolve),median(Psolve),median(Ssolve)];
OUT.aggregate.max_solve_ms=[max(Bsolve),max(Psolve),max(Ssolve)];
OUT.aggregate.min_hard_slack=[min(Bsl),min(Psl),min(Ssl)];
OUT.aggregate.max_d13_sample_difference=max(d13);

assignin('base','c6_step3_14case_three_backend_master_v8_result',OUT);
end


function OUT = run_bezier_d13(caseIndex,verbose)
% STEP 3 COMMON / C02 / BEZIER_D13_MIN7
% Literature-aligned baseline:
%   - Piecewise degree-13 Bezier/Bernstein trajectory over safe corridor cells
%   - Continuous geometric safety by Bernstein control-point convex hull
%     (Gao et al., ICRA 2018 style)
%   - Shared physical knot jets eliminate redundant continuity variables only;
%     this is an implementation reduction, not a new safety method.
%
% Minimal knot-jet parameterization inspired by MINCO-style elimination.
%
% SAME trajectory space and SAME hard corridor constraints as the previous
% 3-piece degree-13 C6 Bezier formulation.
%
% IMPORTANT CHANGE:
%   J7 is NOT optimized in Step 3.
%   The only objective is geometric guide proximity:
%
%       sum over all Bezier CPs ||P_ij - Pguide_ij||^2
%
%   No arbitrary weights are used.
%
% Decision variables (43 total):
%   - internal knot #2 jets r=0..6, xyz : 21
%   - internal knot #3 jets r=0..6, xyz : 21
%   - recovery coordinate xi             : 1
%
% Start/end jets are affine functions of xi.
% Every degree-13 segment is reconstructed uniquely from its two endpoint
% jets, therefore:
%   - C6 is structural
%   - no equality constraints are required
%   - no redundant Bezier variables remain
%
% Numerical scaling is algebraic only and does not alter the feasible set,
% geometric objective, or safety boundary.
%
% ONE quadprog call. No search, no rescue, no safety relaxation.


P=aud_params();
CASES=aud_build_cases();
S=CASES(caseIndex);
fid=1;

fprintf('========================================================================================================================\n');
fprintf(' STEP 3 COMMON / CASE / BEZIER_D13 LITERATURE BASELINE V5\n');
fprintf(' 43 minimal variables | structural C6 | guide proximity | exact-support | one QP\n');
fprintf('========================================================================================================================\n');

G=aud_run_case(S,P,fid);
if ~G.pass
    error('MINCOJET:Guide','Step1 failed.');
end

obs=S.obs(G.front.known_ids_at_trigger);
C=step2_build_and_audit_corridor(G,obs,P,fid);
if ~C.pass
    error('MINCOJET:Corridor','Step2B failed.');
end

ticBuild=tic;
M=bez13_geo_build_model(G,C);
buildMs=1e3*toc(ticBuild);

% Algebraic row scaling only.
[As,bs]=bez13_jet_scale_ineq(M.Aineq,M.bineq);

lb=-inf(M.nvar,1);
ub= inf(M.nvar,1);
lb(M.iXi)=G.rejoin_xi/M.xiScale;

qpopt=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

ticSolve=tic;
[z,objScaled,exitflag,solverOutput]=quadprog( ...
    M.Hscaled,M.fscaled,As,bs,[],[],lb,ub,[],qpopt);
solveMs=1e3*toc(ticSolve);

if isempty(z)
    fprintf('\n[QP]\n');
    fprintf('solverCalls                = 1\n');
    fprintf('buildTimeMs                = %.6f\n',buildMs);
    fprintf('solveTimeMs                = %.6f\n',solveMs);
    fprintf('nDecisionVariables         = %d\n',M.nvar);
    fprintf('nIneq                      = %d\n',size(M.Aineq,1));
    fprintf('nEq                        = 0\n');
    fprintf('exitflag                   = %d\n',exitflag);
    fprintf('BEZIER_D13_GEOMETRIC_PASS  = 0\n');

    OUT=struct();
    OUT.mode='STEP3_COMMON_C02_BEZIER_D13_LITERATURE_BASELINE_V5';
OUT.case_index=caseIndex;
OUT.case_id=S.case_id;
OUT.case_desc=S.case_desc;
    OUT.backend='BEZIER_D13_MIN7';
    OUT.parameterization='MINIMAL_KNOT_JET_GEOMETRIC_OBJECTIVE';
    OUT.solver_calls=1;
    OUT.build_time_ms=buildMs;
    OUT.solve_time_ms=solveMs;
    OUT.exitflag=exitflag;
    OUT.solver_output=solverOutput;
    OUT.n_decision_variables=M.nvar;
    OUT.n_ineq=size(M.Aineq,1);
    OUT.n_eq=0;
    OUT.pass=false;
    OUT.reason='NO_QP_SOLUTION';

    return
end

ticAudit=tic;

CP=bez13_jet_reconstruct_cp(z,M);
xi=z(M.iXi)*M.xiScale;

% Original unscaled hard inequalities.
slack=M.bineq-M.Aineq*z;
[minSlack,worstRow]=min(slack);
worst=M.safeMeta(worstRow);

% Structural C6 / endpoint audit in physical jet coordinates.
[continuityResidual,firstJetResidual,lastJetResidual]= ...
    bez13_jet_full_audit(CP,xi,M,G);

% Recovery point.
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);
pRec=p0+xi*e;
recoverySlack=min(C.cells{3}.A*pRec-C.cells{3}.b);

% Geometric objective in physical coordinates.
Jgeo=bez13_geo_objective_value(CP,G);

% Diagnostic J7 only -- NOT optimized in Step 3.
J7diag=bez13_jet_J7(CP,M);

% Diagnostic dense exact Euclidean clearance only.
[sDense,pDense]=bez13_eval_dense(CP,M,301);
denseExactSlack=bez13_dense_exact_ellipsoid_slack(pDense,obs,P);

auditMs=1e3*toc(ticAudit);
totalMs=buildMs+solveMs+auditMs;

scaledViolation=max([0;As*z-bs]);
coordInf=max(abs(CP(:)));

% Conditioning of the minimal Hessian.
Hz=full(M.Hphysical);
Hz=0.5*(Hz+Hz.');
ev=eig(Hz);
ev=sort(real(ev));
tolH=max(size(Hz))*eps(max(abs(ev)));
pos=ev>tolH;
if any(pos)
    condH=max(ev(pos))/min(ev(pos));
else
    condH=Inf;
end

c6StructuralResidual=0;

pass=(exitflag>0) && ...
     (c6StructuralResidual==0) && ...
     (minSlack>=0) && ...
     (recoverySlack>=0);

fprintf('\n[COMMON INPUT]\n');
fprintf('case                       = C02_TWO_BASE\n');
fprintf('xiGeom                     = %.17g\n',G.rejoin_xi);
fprintf('segment_h                  = %s\n',mat2str(M.h,17));
fprintf('corridorPlanes             = [%d %d %d]\n', ...
    size(C.cells{1}.A,1),size(C.cells{2}.A,1),size(C.cells{3}.A,1));

fprintf('\n[MINIMAL QP SIZE / RUNTIME]\n');
fprintf('solverCalls                = 1\n');
fprintf('buildTimeMs                = %.6f\n',buildMs);
fprintf('solveTimeMs                = %.6f\n',solveMs);
fprintf('auditTimeMs                = %.6f\n',auditMs);
fprintf('step3TotalTimeMs           = %.6f\n',totalMs);
fprintf('nDecisionVariables         = %d\n',M.nvar);
fprintf('nIneq                      = %d\n',size(M.Aineq,1));
fprintf('nEq                        = 0\n');
fprintf('exitflag                   = %d\n',exitflag);
fprintf('geometricHessianCond         = %.17e\n',condH);

fprintf('\n[ORIGINAL HARD POST-CHECK]\n');
fprintf('xiChosen                   = %.17g\n',xi);
fprintf('additionalRunout           = %.17e\n',xi-G.rejoin_xi);
fprintf('controlPointInfNorm        = %.17e\n',coordInf);
fprintf('scaledMaxIneqViolation     = %.17e\n',scaledViolation);
fprintf('C6StructuralResidual       = %.17e\n',c6StructuralResidual);
fprintf('C6CPReconstructionResidual = %.17e  [diagnostic only]\n',continuityResidual);
fprintf('firstJetResidual           = %.17e\n',firstJetResidual);
fprintf('lastJetResidual            = %.17e\n',lastJetResidual);
fprintf('minOriginalHardSlack       = %+.17e  (seg=%d cp=%d plane=%d)\n', ...
    minSlack,worst.segment,worst.cp,worst.plane);
fprintf('recoveryFinalCellSlack     = %+.17e\n',recoverySlack);
fprintf('Jgeo                       = %.17e\n',Jgeo);
fprintf('J7Diagnostic               = %.17e  [not optimized]\n',J7diag);
fprintf('denseExactEllipsoidSlack   = %+.17e  [diagnostic only]\n',denseExactSlack);
fprintf('BEZIER_D13_GEOMETRIC_PASS  = %d\n',pass);
fprintf('LITERATURE_BASELINE         = Gao-style Bernstein corridor certificate; no new safety construction.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='STEP3_COMMON_C02_BEZIER_D13_LITERATURE_BASELINE_V5';
OUT.backend='BEZIER_D13_MIN7';
OUT.parameterization='MINIMAL_KNOT_JET_GEOMETRIC_OBJECTIVE';
OUT.solver_calls=1;
OUT.build_time_ms=buildMs;
OUT.solve_time_ms=solveMs;
OUT.audit_time_ms=auditMs;
OUT.step3_total_time_ms=totalMs;
OUT.exitflag=exitflag;
OUT.solver_output=solverOutput;
OUT.n_decision_variables=M.nvar;
OUT.n_ineq=size(M.Aineq,1);
OUT.n_eq=0;
OUT.geometric_hessian_condition=condH;
OUT.xi_geometric=G.rejoin_xi;
OUT.xi_chosen=xi;
OUT.additional_runout=xi-G.rejoin_xi;
OUT.control_point_inf_norm=coordInf;
OUT.scaled_max_ineq_violation=scaledViolation;
OUT.c6_structural_residual=c6StructuralResidual;
OUT.c6_cp_reconstruction_residual=continuityResidual;
OUT.first_jet_residual=firstJetResidual;
OUT.last_jet_residual=lastJetResidual;
OUT.min_original_hard_slack=minSlack;
OUT.recovery_final_cell_slack=recoverySlack;
OUT.Jgeo=Jgeo;
OUT.J7_diagnostic=J7diag;
OUT.dense_exact_ellipsoid_slack=denseExactSlack;
OUT.control_points=CP;
OUT.sample_s=sDense;
OUT.sample_position=pDense;
OUT.pass=pass;

end

function OUT = run_poly_d13(caseIndex,verbose)
% STEP 3 COMMON / C02 / BEZIER_D13_MIN7
% Literature-aligned baseline:
%   - Piecewise degree-13 polynomial with minimal/shared knot-jet variables
%     following the MINCO/GCOPTER design philosophy
%   - Geometry is certified through the Bernstein image of each polynomial
%   - Raw power-basis coefficients are output-only because they are not a
%     numerically suitable online optimization basis.
%   - No new safety construction is claimed here.
%
% Minimal knot-jet parameterization inspired by MINCO-style elimination.
%
% SAME trajectory space and SAME hard corridor constraints as the previous
% 3-piece degree-13 C6 Bezier formulation.
%
% POLY_D13 stable implementation:
%   The polynomial is parameterized by shared physical endpoint jets.
%   Optimization/safety are assembled through the numerically stable
%   Bernstein image of the same degree-13 polynomial. Raw power
%   coefficients are generated only after solving.
%
%   J7 is NOT optimized in Step 3.
%   The only objective is geometric guide proximity:
%
%       sum over all Bezier CPs ||P_ij - Pguide_ij||^2
%
%   No arbitrary weights are used.
%
% Decision variables (43 total):
%   - internal knot #2 jets r=0..6, xyz : 21
%   - internal knot #3 jets r=0..6, xyz : 21
%   - recovery coordinate xi             : 1
%
% Start/end jets are affine functions of xi.
% Every degree-13 segment is reconstructed uniquely from its two endpoint
% jets, therefore:
%   - C6 is structural
%   - no equality constraints are required
%   - no redundant Bezier variables remain
%
% Numerical scaling is algebraic only and does not alter the feasible set,
% geometric objective, or safety boundary.
%
% ONE quadprog call. No search, no rescue, no safety relaxation.


P=aud_params();
CASES=aud_build_cases();
S=CASES(caseIndex);
fid=1;

fprintf('========================================================================================================================\n');
fprintf(' STEP 3 COMMON / CASE / POLY_D13 LITERATURE BASELINE V5\n');
fprintf(' 43 minimal variables | structural C6 | guide proximity | exact-support | one QP\n');
fprintf('========================================================================================================================\n');

G=aud_run_case(S,P,fid);
if ~G.pass
    error('MINCOJET:Guide','Step1 failed.');
end

obs=S.obs(G.front.known_ids_at_trigger);
C=step2_build_and_audit_corridor(G,obs,P,fid);
if ~C.pass
    error('MINCOJET:Corridor','Step2B failed.');
end

ticBuild=tic;
M=poly13_stable_build_model(G,C);
buildMs=1e3*toc(ticBuild);

% Algebraic row scaling only.
[As,bs]=bez13_jet_scale_ineq(M.Aineq,M.bineq);

lb=-inf(M.nvar,1);
ub= inf(M.nvar,1);
lb(M.iXi)=G.rejoin_xi/M.xiScale;

qpopt=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

ticSolve=tic;
[z,objScaled,exitflag,solverOutput]=quadprog( ...
    M.Hscaled,M.fscaled,As,bs,[],[],lb,ub,[],qpopt);
solveMs=1e3*toc(ticSolve);

if isempty(z)
    fprintf('\n[QP]\n');
    fprintf('solverCalls                = 1\n');
    fprintf('buildTimeMs                = %.6f\n',buildMs);
    fprintf('solveTimeMs                = %.6f\n',solveMs);
    fprintf('nDecisionVariables         = %d\n',M.nvar);
    fprintf('nIneq                      = %d\n',size(M.Aineq,1));
    fprintf('nEq                        = 0\n');
    fprintf('exitflag                   = %d\n',exitflag);
    fprintf('POLY_D13_HERMITE_PASS  = 0\n');

    OUT=struct();
    OUT.mode='STEP3_COMMON_C02_POLY_D13_LITERATURE_BASELINE_V5';
OUT.case_index=caseIndex;
OUT.case_id=S.case_id;
OUT.case_desc=S.case_desc;
    OUT.backend='POLY_D13';
    OUT.parameterization='MINIMAL_PHYSICAL_HERMITE_JETS';
    OUT.solver_calls=1;
    OUT.build_time_ms=buildMs;
    OUT.solve_time_ms=solveMs;
    OUT.exitflag=exitflag;
    OUT.solver_output=solverOutput;
    OUT.n_decision_variables=M.nvar;
    OUT.n_ineq=size(M.Aineq,1);
    OUT.n_eq=0;
    OUT.pass=false;
    OUT.reason='NO_QP_SOLUTION';

    return
end

ticAudit=tic;

CP=bez13_jet_reconstruct_cp(z,M);
PowerCoef=poly13_stable_bernstein_to_power(CP);
xi=z(M.iXi)*M.xiScale;

% Original unscaled hard inequalities.
slack=M.bineq-M.Aineq*z;
[minSlack,worstRow]=min(slack);
worst=M.safeMeta(worstRow);

% Structural C6 / endpoint audit in physical jet coordinates.
[continuityResidual,firstJetResidual,lastJetResidual]= ...
    bez13_jet_full_audit(CP,xi,M,G);

% Recovery point.
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);
pRec=p0+xi*e;
recoverySlack=min(C.cells{3}.A*pRec-C.cells{3}.b);

% Geometric objective in physical coordinates.
Jgeo=bez13_geo_objective_value(CP,G);

% Diagnostic J7 only -- NOT optimized in Step 3.
J7diag=bez13_jet_J7(CP,M);

% Diagnostic dense exact Euclidean clearance only.
[sDense,pDense]=bez13_eval_dense(CP,M,301);
denseExactSlack=bez13_dense_exact_ellipsoid_slack(pDense,obs,P);

auditMs=1e3*toc(ticAudit);
totalMs=buildMs+solveMs+auditMs;

scaledViolation=max([0;As*z-bs]);
coordInf=max(abs(CP(:)));

% Conditioning of the minimal Hessian.
Hz=full(M.Hphysical);
Hz=0.5*(Hz+Hz.');
ev=eig(Hz);
ev=sort(real(ev));
tolH=max(size(Hz))*eps(max(abs(ev)));
pos=ev>tolH;
if any(pos)
    condH=max(ev(pos))/min(ev(pos));
else
    condH=Inf;
end

c6StructuralResidual=0;

pass=(exitflag>0) && ...
     (c6StructuralResidual==0) && ...
     (minSlack>=0) && ...
     (recoverySlack>=0);

fprintf('\n[COMMON INPUT]\n');
fprintf('case                       = C02_TWO_BASE\n');
fprintf('xiGeom                     = %.17g\n',G.rejoin_xi);
fprintf('segment_h                  = %s\n',mat2str(M.h,17));
fprintf('corridorPlanes             = [%d %d %d]\n', ...
    size(C.cells{1}.A,1),size(C.cells{2}.A,1),size(C.cells{3}.A,1));

fprintf('\n[MINIMAL QP SIZE / RUNTIME]\n');
fprintf('solverCalls                = 1\n');
fprintf('buildTimeMs                = %.6f\n',buildMs);
fprintf('solveTimeMs                = %.6f\n',solveMs);
fprintf('auditTimeMs                = %.6f\n',auditMs);
fprintf('step3TotalTimeMs           = %.6f\n',totalMs);
fprintf('nDecisionVariables         = %d\n',M.nvar);
fprintf('nIneq                      = %d\n',size(M.Aineq,1));
fprintf('nEq                        = 0\n');
fprintf('exitflag                   = %d\n',exitflag);
fprintf('geometricHessianCond         = %.17e\n',condH);

fprintf('\n[ORIGINAL HARD POST-CHECK]\n');
fprintf('xiChosen                   = %.17g\n',xi);
fprintf('additionalRunout           = %.17e\n',xi-G.rejoin_xi);
fprintf('bernsteinCPInfNorm         = %.17e\n',coordInf);
fprintf('powerCoeffInfNorm          = %.17e  [output representation only]\n',max(abs(PowerCoef(:))));
fprintf('scaledMaxIneqViolation     = %.17e\n',scaledViolation);
fprintf('C6StructuralResidual       = %.17e\n',c6StructuralResidual);
fprintf('C6CPReconstructionResidual = %.17e  [diagnostic only]\n',continuityResidual);
fprintf('firstJetResidual           = %.17e  [diagnostic only]\n',firstJetResidual);
fprintf('lastJetResidual            = %.17e  [diagnostic only]\n',lastJetResidual);
fprintf('minOriginalHardSlack       = %+.17e  (seg=%d cp=%d plane=%d)\n', ...
    minSlack,worst.segment,worst.cp,worst.plane);
fprintf('recoveryFinalCellSlack     = %+.17e\n',recoverySlack);
fprintf('Jgeo                       = %.17e\n',Jgeo);
fprintf('J7Diagnostic               = %.17e  [not optimized]\n',J7diag);
fprintf('denseExactEllipsoidSlack   = %+.17e  [diagnostic only]\n',denseExactSlack);
fprintf('POLY_D13_HERMITE_PASS      = %d\n',pass);
fprintf('LITERATURE_BASELINE         = MINCO/GCOPTER-style minimal polynomial representation + Bernstein geometry.\n');
fprintf('NOTE: QP uses stable physical-Hermite/Bernstein map; raw power coefficients are output-only.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='STEP3_COMMON_C02_POLY_D13_LITERATURE_BASELINE_V5';
OUT.backend='POLY_D13';
OUT.parameterization='MINIMAL_PHYSICAL_HERMITE_JETS';
OUT.solver_calls=1;
OUT.build_time_ms=buildMs;
OUT.solve_time_ms=solveMs;
OUT.audit_time_ms=auditMs;
OUT.step3_total_time_ms=totalMs;
OUT.exitflag=exitflag;
OUT.solver_output=solverOutput;
OUT.n_decision_variables=M.nvar;
OUT.n_ineq=size(M.Aineq,1);
OUT.n_eq=0;
OUT.geometric_hessian_condition=condH;
OUT.xi_geometric=G.rejoin_xi;
OUT.xi_chosen=xi;
OUT.additional_runout=xi-G.rejoin_xi;
OUT.bernstein_cp_inf_norm=coordInf;
OUT.power_coefficient_inf_norm=max(abs(PowerCoef(:)));
OUT.scaled_max_ineq_violation=scaledViolation;
OUT.c6_structural_residual=c6StructuralResidual;
OUT.c6_cp_reconstruction_residual=continuityResidual;
OUT.first_jet_residual=firstJetResidual;
OUT.last_jet_residual=lastJetResidual;
OUT.min_original_hard_slack=minSlack;
OUT.recovery_final_cell_slack=recoverySlack;
OUT.Jgeo=Jgeo;
OUT.J7_diagnostic=J7diag;
OUT.dense_exact_ellipsoid_slack=denseExactSlack;
OUT.bernstein_control_points=CP;
OUT.power_coefficients=PowerCoef;
OUT.sample_s=sDense;
OUT.sample_position=pDense;
OUT.pass=pass;

end

function OUT = run_bspline_d7(caseIndex,verbose)
% STEP 3 COMMON SPEC -- BACKEND 3 / 3
% C02 only: BSPLINE_D7
%
% Common requirements preserved:
%   - Step1 guide + Step2B exact-support corridor
%   - degree 7 B-spline
%   - simple interior knots -> structural C6
%   - recovery coordinate xi is optimized in the SAME one-shot QP
%   - continuous corridor certificate through exact Bezier extraction
%   - geometric guide-proximity objective
%   - original, unscaled hard safety post-check must be >= 0
%   - no J7 optimization, no time allocation, no dynamics
%   - no scan / bisection / rescue / safety relaxation
%
% Representation:
%   degree p = 7
%   ncoef    = 22
%   nspan    = 15
%
% The 15 knot spans are distributed across the three Step1 guide segments
% by path length, with guide/corridor transitions placed exactly at knots.
%
% Decision variables:
%   Literature Variant-2 gives 16 native CPs. After exact endpoint-jet
%   elimination, only CP 8:9 (two CPs per axis) + xi remain = 7 variables.
%
% Endpoint spatial jets 0..6 are affine in xi, matching the same current
% common start/recovery semantics used by the degree-13 backend.
%
% Geometric objective:
%   exact integral of squared Euclidean deviation from the piecewise-linear
%   Step1 guide using 8-point Gauss-Legendre per degree-7 span.
%
% NOTE:
%   This is a basis-invariant curve-to-guide geometric objective for the
%   B-spline backend. Its numerical magnitude is not directly comparable
%   to the degree-13 control-point Jgeo magnitude; geometry/safety/runtime
%   are the primary common comparisons at this stage.


P=aud_params();
CASES=aud_build_cases();
S=CASES(caseIndex); % C02_TWO_BASE
fid=1;

fprintf('========================================================================================================================\n');
fprintf(' STEP 3 COMMON / CASE / BSPLINE_D7 LITERATURE VARIANT-2 V5\n');
fprintf(' degree7 | 28 CP / 21 spans | native-CP convex-hull safety | structural C6 | one QP\n');
fprintf('========================================================================================================================\n');

G=aud_run_case(S,P,fid);
if ~G.pass
    error('BSD7:Guide','Step1 failed.');
end

obs=S.obs(G.front.known_ids_at_trigger);
C=step2_build_and_audit_corridor(G,obs,P,fid);
if ~C.pass
    error('BSD7:Corridor','Step2B failed.');
end

ticBuild=tic;
M=bsd7_build_model(G,C);

% Standard B-spline local convex-hull safety certificate.
% Each native CP is constrained to every corridor cell touched by its
% supported spans. The exact extracted-Bezier post-check remains mandatory.
[M.Aineq,M.bineq,M.safeMeta,nativeCert]=bsd7_native_cp_certificate(M,C);

buildMs=1e3*toc(ticBuild);

R=bsd7_build_equality_elimination(M);

% Reduced variable y contains:
%   fourteen interior native CPs per axis + scaled xi = 43 variables.
% Full variable is x = R.x0 + R.T*y.
Ar=M.Aineq*R.T;
br=M.bineq-M.Aineq*R.x0;

Hr=R.T.'*M.Hscaled*R.T;
Hr=0.5*(Hr+Hr.');
fr=R.T.'*(M.Hscaled*R.x0+M.fscaled);

[Ais,bis]=bsd7_scale_rows(Ar,br);

lb=-inf(R.nred,1);
ub= inf(R.nred,1);
lb(R.iXiRed)=G.rejoin_xi/M.xiScale;

opts=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

ticSolve=tic;
[y,objScaled,exitflag,solverOutput]=quadprog( ...
    sparse(Hr),fr,Ais,bis,[],[],lb,ub,[],opts);
solveMs=1e3*toc(ticSolve);

if isempty(y)
    x=[];
else
    x=R.x0+R.T*y;

    % Explicit reduced/full dimension guards.
    if numel(y)~=R.nred
        error('BSD7:ReducedDimension','Reduced solution y has unexpected length.');
    end
    if numel(x)~=M.nvar
        error('BSD7:FullDimension','Reconstructed full solution x has unexpected length.');
    end
    if size(Ais,2)~=numel(y)
        error('BSD7:ReducedIneqDimension','Reduced inequality matrix does not match y.');
    end
    if size(M.Aineq,2)~=numel(x)
        error('BSD7:FullIneqDimension','Full inequality matrix does not match x.');
    end
    if size(M.Aeq,2)~=numel(x)
        error('BSD7:FullEqDimension','Full equality matrix does not match x.');
    end
end

if isempty(x)
    fprintf('\n[QP]\n');
    fprintf('solverCalls                = 1\n');
    fprintf('buildTimeMs                = %.6f\n',buildMs);
    fprintf('solveTimeMs                = %.6f\n',solveMs);
    fprintf('nDecisionVariablesFull     = %d\n',M.nvar);
fprintf('nDecisionVariablesReduced  = %d\n',R.nred);
    fprintf('nIneq                      = %d\n',size(M.Aineq,1));
    fprintf('nEqInQP                    = 0\n');
    fprintf('exitflag                   = %d\n',exitflag);
    fprintf('BSPLINE_D7_GEOMETRIC_PASS  = 0\n');

    OUT=struct();
    OUT.mode='STEP3_COMMON_C02_BSPLINE_D7_LITERATURE_VARIANT2_V5';
OUT.case_index=caseIndex;
OUT.case_id=S.case_id;
OUT.case_desc=S.case_desc;
    OUT.backend='BSPLINE_D7';
    OUT.solver_calls=1;
    OUT.build_time_ms=buildMs;
    OUT.solve_time_ms=solveMs;
    OUT.exitflag=exitflag;
    OUT.solver_output=solverOutput;
    OUT.n_decision_variables_full=M.nvar;
    OUT.n_decision_variables_reduced=R.nred;
    OUT.n_ineq=size(M.Aineq,1);
    OUT.n_eq_original=size(M.Aeq,1);
    OUT.n_eq_in_qp=0;

    OUT.pass=false;
    OUT.reason='NO_QP_SOLUTION';

    return
end

ticAudit=tic;

xi=x(M.iXi)*M.xiScale;
cp=bsd7_native_cp(x,M);
Bcp=bsd7_extract_bezier(cp,M);

D=bsd7_bottleneck_diagnostics(x,M,C,G);

eqResidual=norm(M.Aeq*x-M.beq,inf);
slack=M.bineq-M.Aineq*x;
[minHardSlack,worstRow]=min(slack);
worst=M.safeMeta(worstRow);

[minBezierSlack,worstSpan,worstCell,worstCP,worstPlane]= ...
    bsd7_original_bezier_slack(Bcp,M,C);

% Structural C6 from degree 7 + simple interior knots.
C6StructuralResidual=0;

% Endpoint Bezier reconstruction diagnostic.
endpointResidual=bsd7_endpoint_diagnostic(Bcp,xi,M,G);

% Recovery point hard post-check.
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);
pRec=p0+xi*e;
recoverySlack=min(C.cells{3}.A*pRec-C.cells{3}.b);

% Exact integrated curve-to-guide objective.
JgeoCurve=bsd7_curve_guide_objective(cp,M,G);

% Dense diagnostic only.
[sDense,pDense]=bsd7_eval_dense(cp,M,301);
denseExactSlack=bez13_dense_exact_ellipsoid_slack(pDense,obs,P);

% Extraction structural audit.
minExtractEntry=min(M.Tbez,[],'all');
extractRowSumErr=max(abs(sum(M.Tbez,2)-1));

scaledIneqViolation=max([0;Ais*y-bis]);
scaledEqResidual=eqResidual;
cpInf=max(abs(cp(:)));
bezInf=max(abs(Bcp(:)));

auditMs=1e3*toc(ticAudit);
totalMs=buildMs+solveMs+auditMs;

pass=(exitflag>0) && ...
     (C6StructuralResidual==0) && ...
     (minHardSlack>=0) && ...
     (minBezierSlack>=0) && ...
     (recoverySlack>=0);

fprintf('\n[COMMON INPUT]\n');
fprintf('case                       = C02_TWO_BASE\n');
fprintf('xiGeom                     = %.17g\n',G.rejoin_xi);
fprintf('guideFractions             = %s\n',mat2str(M.frac,17));
fprintf('spanCountByCell            = %s  [C6 boundary-padded Variant-2]\n',mat2str(M.spanCount));
fprintf('controlPointRule           = n=d(q-1)+2+2r = %d, r=6\n',M.ncoef);
fprintf('intervalCount              = %d\n',M.nspan);
fprintf('corridorPlanes             = [%d %d %d]\n', ...
    size(C.cells{1}.A,1),size(C.cells{2}.A,1),size(C.cells{3}.A,1));
fprintf('safetyCertificate          = native B-spline CP local convex hull\n');
fprintf('nativeCPCellMemberships    = %d\n',nativeCert.uniqueMemberships);
fprintf('nativeCPMultiCellCount     = %d\n',nativeCert.multiCellCPs);
fprintf('nativeCPRows               = %d\n',nativeCert.constraintRows);

fprintf('\n[QP SIZE / RUNTIME]\n');
fprintf('solverCalls                = 1\n');
fprintf('buildTimeMs                = %.6f\n',buildMs);
fprintf('solveTimeMs                = %.6f\n',solveMs);
fprintf('auditTimeMs                = %.6f\n',auditMs);
fprintf('step3TotalTimeMs           = %.6f\n',totalMs);
fprintf('nDecisionVariablesFull     = %d\n',M.nvar);
fprintf('nDecisionVariablesReduced  = %d\n',R.nred);
fprintf('nIneq                      = %d\n',size(M.Aineq,1));
fprintf('nEqOriginal                = %d\n',size(M.Aeq,1));
fprintf('nEqInQP                    = 0\n');
fprintf('exitflag                   = %d\n',exitflag);
fprintf('equalityEliminationCond    = %.17e\n',R.condDependentBlock);
fprintf('reconstructionEqResidual  = %.17e\n',norm(M.Aeq*x-M.beq,inf));

fprintf('\n[BOTTLENECK ANATOMY]\n');
fprintf('rankAeq                    = %d\n',D.rankAeq);
fprintf('nullityAfterEquality       = %d\n',D.nullityAfterEquality);
fprintf('condAeqNonzeroSV           = %.17e\n',D.condAeq);
fprintf('reducedHessianCond         = %.17e\n',D.reducedHessianCond);
fprintf('activeIneq_1e12            = %d / %d\n',D.active1e12,size(M.Aineq,1));
fprintf('activeIneq_1e10            = %d / %d\n',D.active1e10,size(M.Aineq,1));
fprintf('activeIneqByCell           = %s\n',mat2str(D.activeByCell));
fprintf('ineqRowsByCell             = %s\n',mat2str(D.ineqRowsByCell));
fprintf('meanNativeSupportPerBezier = %.6f\n',D.meanNativeSupport);
fprintf('maxNativeSupportPerBezier  = %d\n',D.maxNativeSupport);
fprintf('endpointEqNativeCPUsed     = %d / %d\n',D.endpointNativeCPUsed,M.ncoef);
fprintf('safetyNativeCPUsed         = %d / %d\n',D.safetyNativeCPUsed,M.ncoef);
fprintf('allNativeCPConstrained     = %d\n',D.allNativeCPConstrained);
fprintf('nativeCertificateRowsCheck = %d\n',nativeCert.constraintRows);
fprintf('boundaryWorstIsTransition  = %d\n',D.worstIsTransition);
fprintf('boundaryWorstSharedPoint   = %d\n',D.worstIsSharedBezierPoint);
fprintf('\n[ORIGINAL HARD POST-CHECK]\n');
fprintf('xiChosen                   = %.17g\n',xi);
fprintf('additionalRunout           = %.17e\n',xi-G.rejoin_xi);
fprintf('nativeCPInfNorm            = %.17e\n',cpInf);
fprintf('bezierCPInfNorm            = %.17e\n',bezInf);
fprintf('scaledMaxIneqViolation     = %.17e\n',scaledIneqViolation);
fprintf('scaledEqResidual           = %.17e\n',scaledEqResidual);
fprintf('originalEqResidual         = %.17e  [numerical diagnostic]\n',eqResidual);
fprintf('C6StructuralResidual       = %.17e\n',C6StructuralResidual);
fprintf('endpointBezierResidual     = %.17e  [diagnostic only]\n',endpointResidual);
fprintf('minOriginalHardSlack       = %+.17e  (span=%d cell=%d cp=%d plane=%d)\n', ...
    minHardSlack,worst.span,worst.cell,worst.cp,worst.plane);
fprintf('minExtractedBezierSlack    = %+.17e  (span=%d cell=%d cp=%d plane=%d)\n', ...
    minBezierSlack,worstSpan,worstCell,worstCP,worstPlane);
fprintf('recoveryFinalCellSlack     = %+.17e\n',recoverySlack);
fprintf('JgeoCurveIntegral          = %.17e\n',JgeoCurve);
fprintf('extractMinEntry            = %+.17e\n',minExtractEntry);
fprintf('extractRowSumError         = %.17e\n',extractRowSumErr);
fprintf('denseExactEllipsoidSlack   = %+.17e  [diagnostic only]\n',denseExactSlack);
fprintf('BSPLINE_D7_GEOMETRIC_PASS  = %d\n',pass);
fprintf('NOTE: Nguyen Variant-2 is retained as the base; 2r extra CPs are added only to preserve its free DOF under endpoint jets 0..6.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='STEP3_COMMON_C02_BSPLINE_D7_LITERATURE_VARIANT2_V5';
OUT.backend='BSPLINE_D7';
OUT.degree=M.p;
OUT.n_native_control_points=M.ncoef;
OUT.n_spans=M.nspan;
OUT.span_count_by_cell=M.spanCount;
OUT.knots=M.U;
OUT.solver_calls=1;
OUT.build_time_ms=buildMs;
OUT.solve_time_ms=solveMs;
OUT.audit_time_ms=auditMs;
OUT.step3_total_time_ms=totalMs;
OUT.exitflag=exitflag;
OUT.solver_output=solverOutput;
OUT.n_decision_variables_full=M.nvar;
OUT.n_decision_variables_reduced=R.nred;
OUT.n_ineq=size(M.Aineq,1);
OUT.safety_certificate='NATIVE_BSPLINE_CP_LOCAL_CONVEX_HULL';
OUT.native_cp_cell_memberships=nativeCert.uniqueMemberships;
OUT.native_cp_multi_cell_count=nativeCert.multiCellCPs;
OUT.native_cp_constraint_rows=nativeCert.constraintRows;
OUT.n_eq_original=size(M.Aeq,1);
OUT.n_eq_in_qp=0;
OUT.equality_elimination_condition=R.condDependentBlock;
OUT.reconstruction_eq_residual=norm(M.Aeq*x-M.beq,inf);
OUT.xi_geometric=G.rejoin_xi;
OUT.xi_chosen=xi;
OUT.additional_runout=xi-G.rejoin_xi;
OUT.native_cp_inf_norm=cpInf;
OUT.bezier_cp_inf_norm=bezInf;
OUT.scaled_max_ineq_violation=scaledIneqViolation;
OUT.scaled_eq_residual=scaledEqResidual;
OUT.original_eq_residual=eqResidual;
OUT.c6_structural_residual=C6StructuralResidual;
OUT.endpoint_bezier_residual=endpointResidual;
OUT.min_original_hard_slack=minHardSlack;
OUT.min_extracted_bezier_slack=minBezierSlack;
OUT.recovery_final_cell_slack=recoverySlack;
OUT.Jgeo_curve_integral=JgeoCurve;
OUT.extraction_min_entry=minExtractEntry;
OUT.extraction_row_sum_error=extractRowSumErr;
OUT.dense_exact_ellipsoid_slack=denseExactSlack;
OUT.native_control_points=cp;
OUT.extracted_bezier_control_points=Bcp;
OUT.sample_s=sDense;
OUT.sample_position=pDense;
OUT.pass=pass;

end

function OUT=run_bspline_d7_literature_position_only(caseIndex,verbose)
% Literature-faithful diagnostic:
% Nguyen Variant-2 architecture with START/END POSITION constraints only.
%
% This deliberately removes OUR additional endpoint spatial jets 1..6.
% Purpose: determine whether the current Variant-2 infeasibility is caused
% by those extra endpoint-jet constraints.
%
% Same:
%   - C02
%   - Step1 guide / Step2B exact-support corridor
%   - degree 7
%   - 16 native CP / 9 spans / [1 7 1]
%   - extracted-Bezier continuous safety certificate
%   - geometric curve-to-guide objective
%   - variable recovery xi
%   - strict original hard post-check
%
% Different:
%   - endpoint constraints are only p(start)=p_trigger and
%     p(end)=p_trigger+e*xi, matching the literature path problem.

fprintf('========================================================================================================================\n');
fprintf(' C02 / BSPLINE_D7 / NGUYEN VARIANT-2 / POSITION-ONLY DIAGNOSTIC\n');
fprintf(' 16 CP / 9 spans | literature endpoint positions only | exact-support | one QP\n');
fprintf('========================================================================================================================\n');

P=aud_params();
CASES=aud_build_cases();
S=CASES(caseIndex);
fid=1;

G=aud_run_case(S,P,fid);
if ~G.pass
    error('BSD7LIT:Guide','Step1 failed.');
end
obs=S.obs(G.front.known_ids_at_trigger);
if isempty(obs)
    error('BSD7LIT:Obs','No known obstacle available at trigger.');
end
C=step2_build_and_audit_corridor(G,obs,P,fid);
if ~isstruct(C) || ~isfield(C,'pass') || ~isfield(C,'cells')
    error('BSD7LIT:CorridorStruct','Unexpected Step2B corridor structure.');
end
if ~C.pass
    error('BSD7LIT:Corridor','Step2B failed.');
end
if numel(C.cells)~=3
    error('BSD7LIT:CorridorCells','Expected exactly three C02 corridor cells.');
end

ticBuild=tic;
M=bsd7_build_model_position_only(G,C);

% Exact elimination of clamped start/end position constraints:
% P1 = p0, Pn = p0 + e*xi.
R=bsd7_position_only_elimination(M,G);

Ar=M.Aineq*R.T;
br=M.bineq-M.Aineq*R.x0;

Hr=R.T.'*M.Hscaled*R.T;
Hr=0.5*(Hr+Hr.');
fr=R.T.'*(M.Hscaled*R.x0+M.fscaled);

[Ais,bis]=bsd7_scale_rows(Ar,br);

lb=-inf(R.nred,1);
ub= inf(R.nred,1);
lb(R.iXiRed)=G.rejoin_xi/M.xiScale;

opts=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

buildMs=1e3*toc(ticBuild);

ticSolve=tic;
[y,~,exitflag,solverOutput]=quadprog(sparse(Hr),fr,Ais,bis,[],[],lb,ub,[],opts);
solveMs=1e3*toc(ticSolve);

if isempty(y)
    error('BSD7LIT:NoSolution','quadprog returned empty solution.');
end
x=R.x0+R.T*y;

ticAudit=tic;
cp=bsd7_native_cp(x,M);
Bcp=bsd7_extract_bezier(cp,M);

scaledIneqViolation=max([0;Ais*y-bis]);
[minHardSlack,~,~,~,~]=bsd7_original_bezier_slack(Bcp,M,C);
minBezierSlack=minHardSlack;

xi=M.xiScale*x(M.iXi);
recoverySlack=min(C.cells{3}.A*(G.front.payload_at_trigger + ...
    (G.waypoints(:,3)-G.waypoints(:,2))/norm(G.waypoints(:,3)-G.waypoints(:,2))*xi)-C.cells{3}.b);

Jgeo=bsd7_curve_guide_objective(cp,M,G);
[sSample,pSample]=bsd7_eval_dense(cp,M,301);
if size(pSample,1)~=3
    error('BSD7LIT:DenseShape','Dense position samples must be 3xN.');
end
denseSlack=bez13_dense_exact_ellipsoid_slack(pSample,obs,P);

startResidual=norm(cp(:,1)-G.front.payload_at_trigger,inf);
e=G.waypoints(:,3)-G.waypoints(:,2); e=e/norm(e);
endResidual=norm(cp(:,end)-(G.front.payload_at_trigger+e*xi),inf);

auditMs=1e3*toc(ticAudit);
totalMs=buildMs+solveMs+auditMs;

pass=(exitflag>0) && ...
     (minHardSlack>=0) && ...
     (minBezierSlack>=0) && ...
     (recoverySlack>=0);

fprintf('\n[LITERATURE POSITION-ONLY RESULT]\n');
fprintf('solverCalls                = 1\n');
fprintf('buildTimeMs                = %.6f\n',buildMs);
fprintf('solveTimeMs                = %.6f\n',solveMs);
fprintf('auditTimeMs                = %.6f\n',auditMs);
fprintf('step3TotalTimeMs           = %.6f\n',totalMs);
fprintf('nDecisionVariablesFull     = %d\n',M.nvar);
fprintf('nDecisionVariablesReduced  = %d\n',R.nred);
fprintf('nIneq                      = %d\n',size(M.Aineq,1));
fprintf('nEqInQP                    = 0\n');
fprintf('exitflag                   = %d\n',exitflag);
fprintf('xiChosen                   = %.17g\n',xi);
fprintf('additionalRunout           = %.17e\n',xi-G.rejoin_xi);
fprintf('startPositionResidual      = %.17e\n',startResidual);
fprintf('endPositionResidual        = %.17e\n',endResidual);
fprintf('scaledMaxIneqViolation     = %.17e\n',scaledIneqViolation);
fprintf('minOriginalHardSlack       = %+.17e\n',minHardSlack);
fprintf('recoveryFinalCellSlack     = %+.17e\n',recoverySlack);
fprintf('JgeoCurveIntegral          = %.17e\n',Jgeo);
fprintf('denseExactEllipsoidSlack   = %+.17e  [diagnostic only]\n',denseSlack);
fprintf('BSPLINE_LITERATURE_PASS    = %d\n',pass);
fprintf('NOTE: this diagnostic reproduces the literature endpoint-position problem; it does NOT yet enforce our added endpoint jets 1..6.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='BSPLINE_D7_NGUYEN_VARIANT2_POSITION_ONLY_DIAGNOSTIC';
OUT.backend='BSPLINE_D7';
OUT.solver_calls=1;
OUT.build_time_ms=buildMs;
OUT.solve_time_ms=solveMs;
OUT.audit_time_ms=auditMs;
OUT.step3_total_time_ms=totalMs;
OUT.exitflag=exitflag;
OUT.solver_output=solverOutput;
OUT.n_decision_variables_full=M.nvar;
OUT.n_decision_variables_reduced=R.nred;
OUT.n_ineq=size(M.Aineq,1);
OUT.n_eq_in_qp=0;
OUT.xi_geometric=G.rejoin_xi;
OUT.xi_chosen=xi;
OUT.additional_runout=xi-G.rejoin_xi;
OUT.start_position_residual=startResidual;
OUT.end_position_residual=endResidual;
OUT.scaled_max_ineq_violation=scaledIneqViolation;
OUT.min_original_hard_slack=minHardSlack;
OUT.recovery_final_cell_slack=recoverySlack;
OUT.Jgeo_curve_integral=Jgeo;
OUT.dense_exact_ellipsoid_slack=denseSlack;
OUT.native_control_points=cp;
OUT.extracted_bezier_control_points=Bcp;
OUT.sample_s=sSample;
OUT.sample_position=pSample;
OUT.pass=pass;
end


function M=bsd7_build_model_position_only(G,C)
% Same Nguyen Variant-2 geometry/objective/safety as bsd7_build_model,
% but no endpoint jet equalities are built here.
M=bsd7_build_model(G,C);
M.Aeq=sparse(0,M.nvar);
M.beq=zeros(0,1);
end


function R=bsd7_position_only_elimination(M,G)
% For an open clamped B-spline:
% curve start = native CP1, curve end = native CPn.
% Eliminate those six endpoint coordinates exactly, retaining xi.

n=M.ncoef;
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

dep=[1, n, n+1, 2*n, 2*n+1, 3*n];
allVars=1:M.nvar;
free=setdiff(allVars,dep,'stable');

x0=zeros(M.nvar,1);
T=zeros(M.nvar,numel(free));
T(free,:)=eye(numel(free));

% start CP coordinates
x0(1)=p0(1);
x0(n+1)=p0(2);
x0(2*n+1)=p0(3);

% end CP = p0 + e * xiScale*zXi
x0(n)=p0(1);
x0(2*n)=p0(2);
x0(3*n)=p0(3);

iXiRed=find(free==M.iXi,1);
if isempty(iXiRed)
    error('BSD7LIT:Xi','xi must remain a reduced variable.');
end
T(n,iXiRed)=e(1)*M.xiScale;
T(2*n,iXiRed)=e(2)*M.xiScale;
T(3*n,iXiRed)=e(3)*M.xiScale;

R=struct();
R.dep=dep;
R.free=free;
R.x0=x0;
R.T=T;
R.nred=numel(free);
R.iXiRed=iXiRed;
end


function M=bsd7_build_model(G,C)
p=7;
qcell=3;
rBoundary=6;

% Literature-guided C6 boundary extension.
% Base Nguyen Variant-2 uses n0=d(q-1)+2 and only endpoint positions.
% Our global-C6 requirement fixes endpoint spatial jets 0..rBoundary.
% To preserve the SAME reduced design freedom as the literature position-only
% problem, add 2*rBoundary control points:
%   n = d(q-1)+2 + 2*rBoundary.
% For d=7, q=3, rBoundary=6:
%   ncoef = 28, nspan = 21, reduced variables = 43.
ncoef=p*(qcell-1)+2+2*rBoundary;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
Ltot=sum(segLen);
if ~(Ltot>0)
    error('BSD7:GuideLength','Zero guide length.');
end
frac=segLen/Ltot;

% Boundary-jet padded corridor assignment.
% Degree p endpoint-jet control points are kept inside the first/last
% corridor before any transition. Hence allocate p spans to each end cell;
% the remaining p spans stay in the middle cell:
%   [p p p] = [7 7 7].
spanCount=[p,p,p];
if sum(spanCount)~=nspan
    error('BSD7:C6Variant2','C6-extended span count is inconsistent.');
end

ub=[0,cumsum(frac)];
ub(end)=1;

% Keep guide/corridor transitions at knot breaks and distribute the seven
% middle intervals uniformly within the middle guide fraction.
breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx);
    b=ub(cidx+1);
    nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1
        breaks=local;
    else
        breaks=[breaks,local(2:end)]; %#ok<AGROW>
    end
    cellOfSpan(ss+(1:nc))=cidx;
    ss=ss+nc;
end

U=[repmat(breaks(1),1,p+1),breaks(2:end-1),repmat(breaks(end),1,p+1)];

[~,Tbez]=step3a_bs_bezier_decomposition_transform(U,p,ncoef);
expected=nspan*p+1;
if size(Tbez,1)~=expected
    error('BSD7:BezierExtract','Unexpected extraction row count.');
end

pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

% x = [Px(22); Py(22); Pz(22); zXi], xi = xiScale*zXi.
nvar=3*ncoef+1;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);
iXi=nvar;
xiScale=max(1,G.path_length);

p0=G.front.payload_at_trigger;
e=W(:,3)-W(:,2);
e=e/norm(e);

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

Aeq=zeros(0,nvar);
beq=zeros(0,1);
axisIds={ix,iy,iz};

% Endpoint Bezier CP identities for spatial jets 0..6.
% Same semantics as the current common degree-13 implementation.
for ax=1:3
    ia=axisIds{ax};

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        rr(iXi)=-(j*h0/p)*e(ax)*xiScale;
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        rr(iXi)=-(1-j*h1/p)*e(ax)*xiScale;
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end
end

% Continuous safety via exact extracted local Bezier control points.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            n=CC.A(hh,:).';
            rr=zeros(1,nvar);
            rr(ix)=-n(1)*er;
            rr(iy)=-n(2)*er;
            rr(iz)=-n(3)*er;

            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
        end
    end
end

% Exact integrated squared curve-to-guide objective.
[xg,wg]=step3a_gauss8_01();
Cobj=zeros(3*nspan*numel(xg),nvar);
dobj=zeros(3*nspan*numel(xg),1);
rr=0;

for q=1:nspan
    ua=breaks(q);
    uq=breaks(q+1);
    hs=uq-ua;
    cellIdx=cellOfSpan(q);
    ga=ub(cellIdx);
    gb=ub(cellIdx+1);

    for k=1:numel(xg)
        u=ua+hs*xg(k);
        N=step3a_bs_basis_all(u,U,p,ncoef);

        alpha=(u-ga)/(gb-ga);
        alpha=min(1,max(0,alpha));
        pg=(1-alpha)*W(:,cellIdx)+alpha*W(:,cellIdx+1);

        sw=sqrt(wg(k)*hs);

        rr=rr+1;
        Cobj(rr,ix)=sw*N;
        dobj(rr)=sw*pg(1);

        rr=rr+1;
        Cobj(rr,iy)=sw*N;
        dobj(rr)=sw*pg(2);

        rr=rr+1;
        Cobj(rr,iz)=sw*N;
        dobj(rr)=sw*pg(3);
    end
end

H=2*(Cobj.'*Cobj);
H=0.5*(H+H.');
f=-2*(Cobj.'*dobj);

% Positive scalar objective scaling only.
d=max(abs(diag(H)));
if ~(d>0), d=1; end

M=struct();
M.p=p;
M.ncoef=ncoef;
M.nspan=nspan;
M.nvar=nvar;
M.ix=ix;
M.iy=iy;
M.iz=iz;
M.iXi=iXi;
M.xiScale=xiScale;
M.frac=frac;
M.spanCount=spanCount;
M.breaks=breaks;
M.U=U;
M.ub=ub;
M.cellOfSpan=cellOfSpan;
M.Tbez=Tbez;
M.pieceMaps=pieceMaps;
M.Aeq=sparse(Aeq);
M.beq=beq;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hscaled=sparse(H/d);
M.fscaled=f/d;
end

function [Aineq,bineq,safeMeta,CERT]=bsd7_native_cp_certificate(M,C)
% Native B-spline control-point local convex-hull certificate.
%
% Every span of a B-spline lies in the convex hull of its active native
% control points. For every native CP, collect all corridor cells touched
% by spans in its support and constrain that CP to all those cells.
%
% This is a continuous-time sufficient safety certificate.
% No safety relaxation and no obstacle approximation are introduced.

n=M.ncoef;
nvar=M.nvar;

cellsPerCP=cell(1,n);
spansPerCP=cell(1,n);

for q=1:M.nspan
    E=M.pieceMaps{q};
    active=find(any(abs(E)>0,1));
    ci=M.cellOfSpan(q);

    for kk=1:numel(active)
        cp=active(kk);
        cellsPerCP{cp}(end+1)=ci; %#ok<AGROW>
        spansPerCP{cp}(end+1)=q; %#ok<AGROW>
    end
end

Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});

for cp=1:n
    cellsPerCP{cp}=unique(cellsPerCP{cp},'stable');
    spansPerCP{cp}=unique(spansPerCP{cp},'stable');

    if isempty(cellsPerCP{cp})
        error('BSD7:NativeCert','A native CP has no corridor membership.');
    end

    for ci=cellsPerCP{cp}
        CC=C.cells{ci};

        qrep=[];
        for q=spansPerCP{cp}
            if M.cellOfSpan(q)==ci
                qrep=q;
                break
            end
        end
        if isempty(qrep)
            error('BSD7:NativeCert','Support/cell mapping inconsistency.');
        end

        for hh=1:size(CC.A,1)
            nv=CC.A(hh,:).';
            rr=zeros(1,nvar);

            rr(M.ix(cp))=-nv(1);
            rr(M.iy(cp))=-nv(2);
            rr(M.iz(cp))=-nv(3);

            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct( ...
                'span',qrep,'cell',ci,'cp',cp,'plane',hh); %#ok<AGROW>
        end
    end
end

nCells=cellfun(@numel,cellsPerCP);

CERT=struct();
CERT.cellsPerCP=cellsPerCP;
CERT.spansPerCP=spansPerCP;
CERT.uniqueMemberships=sum(nCells);
CERT.multiCellCPs=sum(nCells>1);
CERT.maxCellsPerCP=max(nCells);
CERT.constraintRows=size(Aineq,1);

rowCheck=0;
for cp=1:n
    for ci=cellsPerCP{cp}
        rowCheck=rowCheck+size(C.cells{ci}.A,1);
    end
end
if rowCheck~=CERT.constraintRows
    error('BSD7:NativeCert','Certificate row-count mismatch.');
end
end


function R=bsd7_build_equality_elimination(M)
% Exact algebraic elimination of endpoint jet equalities.
%
% Degree-7 endpoint jets 0..6 determine seven native control points at
% each end. The remaining middle native CPs and xi are the reduced QP vars.
%
% For the C6-extended variant with ncoef=28:
%   dependent CPs = 1:7 and 22:28
%   free CPs      = 8:21
%   reduced vars  = 14 CP * 3 axes + xi = 43.

n=M.ncoef;
leftDep=1:7;
rightDep=(n-6):n;
middle=8:(n-7);

if isempty(middle)
    error('BSD7:EqElim','No interior native control points remain.');
end
if ~isempty(intersect(leftDep,rightDep))
    error('BSD7:EqElim','Endpoint-dependent control-point sets overlap.');
end

dep=[];
free=[];
for ax=0:2
    base=ax*n;
    dep=[dep,base+leftDep,base+rightDep]; %#ok<AGROW>
    free=[free,base+middle]; %#ok<AGROW>
end
free=[free,M.iXi];

Aeq=full(M.Aeq);
Ad=Aeq(:,dep);
Af=Aeq(:,free);

if size(Ad,1)~=size(Ad,2)
    error('BSD7:EqElim','Dependent endpoint block is not square.');
end
if rank(Ad)~=size(Ad,1)
    error('BSD7:EqElim','Dependent endpoint block is rank deficient.');
end

x0=zeros(M.nvar,1);
T=zeros(M.nvar,numel(free));
T(free,:)=eye(numel(free));
x0(dep)=Ad\M.beq;
T(dep,:)=-Ad\Af;

iXiRed=find(free==M.iXi,1);
if isempty(iXiRed)
    error('BSD7:EqElim','xi was not retained.');
end

R=struct();
R.dep=dep;
R.free=free;
R.middleCP=middle;
R.x0=x0;
R.T=T;
R.nred=numel(free);
R.iXiRed=iXiRed;
R.condDependentBlock=cond(Ad);
end

function D=bsd7_bottleneck_diagnostics(x,M,C,G)
% Diagnose WHY the current B-spline formulation is large/nonlocal.
% No change to optimization or safety logic.

Aeq=full(M.Aeq);
sA=svd(Aeq);
tolA=max(size(Aeq))*eps(max(sA));
rankA=sum(sA>tolA);
nullity=size(Aeq,2)-rankA;
if rankA>0
    condA=sA(1)/sA(rankA);
else
    condA=NaN;
end

% Equality-reduced Hessian conditioning.
Z=null(Aeq);
H=full(M.Hscaled);
Hr=0.5*(Z.'*H*Z+(Z.'*H*Z).');
ev=eig(Hr);
ev=sort(real(ev));
tolH=max(size(Hr))*eps(max(abs(ev)));
pos=ev>tolH;
if any(pos)
    condHr=max(ev(pos))/min(ev(pos));
else
    condHr=Inf;
end

% Active hard inequalities.
sl=M.bineq-M.Aineq*x;
act12=find(sl<=1e-12);
act10=find(sl<=1e-10);

activeByCell=zeros(1,3);
ineqRowsByCell=zeros(1,3);
for ii=1:numel(M.safeMeta)
    ci=M.safeMeta(ii).cell;
    ineqRowsByCell(ci)=ineqRowsByCell(ci)+1;
end
for kk=1:numel(act10)
    ci=M.safeMeta(act10(kk)).cell;
    activeByCell(ci)=activeByCell(ci)+1;
end

% Local support of exact Bezier extraction.
supportCount=sum(abs(M.Tbez)>0,2);
meanSupport=mean(supportCount);
maxSupport=max(supportCount);

% Which native CPs are touched by endpoint equalities?
eqNative=false(1,M.ncoef);
for axBlock=0:2
    cols=axBlock*M.ncoef+(1:M.ncoef);
    eqNative=eqNative | any(abs(Aeq(:,cols))>0,1);
end

% Which native CPs participate in ANY safety row?
Ai=full(M.Aineq);
safeNative=false(1,M.ncoef);
for axBlock=0:2
    cols=axBlock*M.ncoef+(1:M.ncoef);
    safeNative=safeNative | any(abs(Ai(:,cols))>0,1);
end

% Exact row count implied by span allocation/corridor plane counts.
cnt=0;
for q=1:M.nspan
    ci=M.cellOfSpan(q);
    cnt=cnt+(M.p+1)*size(C.cells{ci}.A,1);
end

% Worst hard row anatomy.
[minSlack,iw]=min(sl); %#ok<ASGLU>
meta=M.safeMeta(iw);
isTransition = (meta.span>1) && (M.cellOfSpan(meta.span)~=M.cellOfSpan(meta.span-1));
isShared = isTransition && (meta.cp==1);

D=struct();
D.rankAeq=rankA;
D.nullityAfterEquality=nullity;
D.condAeq=condA;
D.reducedHessianCond=condHr;
D.active1e12=numel(act12);
D.active1e10=numel(act10);
D.activeByCell=activeByCell;
D.ineqRowsByCell=ineqRowsByCell;
D.meanNativeSupport=meanSupport;
D.maxNativeSupport=maxSupport;
D.endpointNativeCPUsed=sum(eqNative);
D.safetyNativeCPUsed=sum(safeNative);
D.allNativeCPConstrained=all(safeNative);
D.ineqFormulaCount=cnt;
D.worstIsTransition=isTransition;
D.worstIsSharedBezierPoint=isShared;
end

function [As,bs]=bsd7_scale_rows(A,b)
if isempty(A)
    As=A;
    bs=b;
    return
end
sc=max(abs(A),[],2);
sc=max(sc,abs(b));
sc=max(sc,1);
As=A./sc;
bs=b./sc;
end

function cp=bsd7_native_cp(x,M)
cp=[x(M.ix).';x(M.iy).';x(M.iz).'];
end

function Bcp=bsd7_extract_bezier(cp,M)
Qx=M.Tbez*cp(1,:).';
Qy=M.Tbez*cp(2,:).';
Qz=M.Tbez*cp(3,:).';

Bcp=zeros(3,M.p+1,M.nspan);
for q=1:M.nspan
    ids=(q-1)*M.p+(1:M.p+1);
    Bcp(:,:,q)=[Qx(ids),Qy(ids),Qz(ids)].';
end
end

function [ms,ws,wc,wj,wp]=bsd7_original_bezier_slack(Bcp,M,C)
ms=Inf;
ws=NaN; wc=NaN; wj=NaN; wp=NaN;

for q=1:M.nspan
    ci=M.cellOfSpan(q);
    CC=C.cells{ci};

    for j=1:M.p+1
        sl=CC.A*Bcp(:,j,q)-CC.b;
        [v,ip]=min(sl);
        if v<ms
            ms=v;
            ws=q;
            wc=ci;
            wj=j;
            wp=ip;
        end
    end
end
end

function r=bsd7_endpoint_diagnostic(Bcp,xi,M,G)
p0=G.front.payload_at_trigger;
W=G.waypoints;
e=W(:,3)-W(:,2);
e=e/norm(e);

h0=M.breaks(2)-M.breaks(1);
h1=M.breaks(end)-M.breaks(end-1);
p=M.p;

r=0;
for ax=1:3
    for j=0:6
        t0=p0(ax)+(j*h0/p)*e(ax)*xi;
        t1=p0(ax)+(1-j*h1/p)*e(ax)*xi;
        r=max(r,abs(Bcp(ax,j+1,1)-t0));
        r=max(r,abs(Bcp(ax,p-j+1,M.nspan)-t1));
    end
end
end

function J=bsd7_curve_guide_objective(cp,M,G)
[xg,wg]=step3a_gauss8_01();
W=G.waypoints;
J=0;

for q=1:M.nspan
    ua=M.breaks(q);
    uq=M.breaks(q+1);
    hs=uq-ua;
    ci=M.cellOfSpan(q);
    ga=M.ub(ci);
    gb=M.ub(ci+1);

    for k=1:numel(xg)
        u=ua+hs*xg(k);
        N=step3a_bs_basis_all(u,M.U,M.p,M.ncoef);
        p=N*cp.';

        alpha=(u-ga)/(gb-ga);
        alpha=min(1,max(0,alpha));
        pg=(1-alpha)*W(:,ci)+alpha*W(:,ci+1);

        d=p.'-pg;
        J=J+wg(k)*hs*(d.'*d);
    end
end
end

function [sAll,pAll]=bsd7_eval_dense(cp,M,Ns)
sAll=linspace(0,1,Ns);
pAll=zeros(3,Ns);

for ii=1:Ns
    N=step3a_bs_basis_all(sAll(ii),M.U,M.p,M.ncoef);
    pAll(:,ii)=cp*N.';
end
end

function M=poly13_stable_build_model(G,C)
n=13;
nseg=3;
ncp=n+1;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
h=segLen/sum(segLen);
h=h(:).';

p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

% Minimal variables:
% knot 2 jet orders 0..6 xyz = 21
% knot 3 jet orders 0..6 xyz = 21
% xi = 1
nvar=43;
iXi=43;

% Structural numerical variable scaling.
% For an internal knot derivative of order r, use the geometric mean of
% adjacent segment lengths. The scale makes a unit z-variable induce O(L)
% Bezier-control-point motion through h^r/fallingFactorial(n,r).
L=max(1,G.path_length);
jetScale=zeros(2,7);
for kk=1:2
    href=sqrt(h(kk)*h(kk+1));
    for r=0:6
        ff=factorial(n)/factorial(n-r);
        jetScale(kk,r+1)=L*ff/(href^r);
    end
end
xiScale=L;

% Fixed finite-difference endpoint interpolation matrix:
% maps 14 CPs -> [left normalized differences 0..6;
%                  right normalized differences 0..6].
F=bez13_jet_endpoint_difference_matrix(n);
if rank(F)~=14
    error('MINCOJET:Interpolation','Endpoint-difference matrix is singular.');
end

% Affine CP map: cpVec = c0 + Gcp*z
% cpVec ordering is x/y/z contiguous per segment CP via bez13_jet_cpidx.
ncpall=3*nseg*ncp;
Gcp=zeros(ncpall,nvar);
c0=zeros(ncpall,1);

for q=1:nseg
    for ax=1:3
        D0=zeros(14,1);
        Dz=zeros(14,nvar);

        % Left knot q, right knot q+1.
        [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,1,q,h(q),ax,p0,e, ...
            jetScale,xiScale,n,iXi);
        [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,8,q+1,h(q),ax,p0,e, ...
            jetScale,xiScale,n,iXi);

        % CP = F \ normalized endpoint differences.
        cp0=F\D0;
        cpG=F\Dz;

        ids=arrayfun(@(j) bez13_jet_cpidx(q,j,ax,ncp),1:ncp);
        c0(ids)=cp0;
        Gcp(ids,:)=cpG;
    end
end

% Hard corridor inequalities on actual CPs.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('segment',{},'cp',{},'plane',{});

for q=1:nseg
    CC=C.cells{q};

    for j=1:ncp
        for pl=1:size(CC.A,1)
            a=zeros(1,ncpall);
            for ax=1:3
                a(bez13_jet_cpidx(q,j,ax,ncp))=-CC.A(pl,ax);
            end

            Aineq(end+1,:)=a*Gcp; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(pl)-a*c0; %#ok<AGROW>

            safeMeta(end+1)=struct( ...
                'segment',q,'cp',j,'plane',pl); %#ok<AGROW>
        end
    end
end

% -------------------------------------------------------------------------
% Geometric guide-proximity objective in minimal coordinates.
%
% For each Bezier CP, define the target point by uniform local parameter
% interpolation along the corresponding Step1 guide segment.
%
%   Jgeo = sum ||Pcp - Pguide||^2
%
% Since Pcp = c0 + Gcp*z:
%
%   Jgeo = 0.5*z'*H*z + f'*z + const
%   H = 2*Gcp'*Gcp
%   f = 2*Gcp'*(c0-target)
%
% No arbitrary physical or tuning weight is introduced.
% -------------------------------------------------------------------------
target=zeros(ncpall,1);

for q=1:nseg
    for j=1:ncp
        tau=(j-1)/(ncp-1);
        pt=(1-tau)*W(:,q)+tau*W(:,q+1);

        for ax=1:3
            target(bez13_jet_cpidx(q,j,ax,ncp))=pt(ax);
        end
    end
end

Hphys=2*(Gcp.'*Gcp);
Hphys=0.5*(Hphys+Hphys.');
fphys=2*Gcp.'*(c0-target);

% Positive scalar scaling only; minimizer is unchanged.
d=max(abs(diag(Hphys)));
if ~(d>0), d=1; end
Hscaled=sparse(Hphys/d);
fscaled=fphys/d;

M=struct();
M.degree=n;
M.nseg=nseg;
M.ncp=ncp;
M.nvar=nvar;
M.iXi=iXi;
M.h=h;
M.jetScale=jetScale;
M.xiScale=xiScale;
M.Gcp=Gcp;
M.c0=c0;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hphysical=sparse(Hphys);
M.fphysical=fphys;
M.Hscaled=Hscaled;
M.fscaled=fscaled;
end

function [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,row0,knot,hseg,ax,p0,e,jetScale,xiScale,n,iXi)
% Insert normalized endpoint differences for one knot and one axis.
%
% normalized r-th difference = h^r/fallingFactorial(n,r) * global derivative.

for r=0:6
    ff=factorial(n)/factorial(n-r);
    mult=(hseg^r)/ff;
    row=row0+r;

    if knot==1
        % start:
        % r0 = p0, r1 = e*xi, r>=2 = 0
        if r==0
            D0(row)=p0(ax);
        elseif r==1
            Dz(row,iXi)=mult*e(ax)*xiScale;
        end

    elseif knot==4
        % end:
        % r0 = p0 + e*xi, r1 = e*xi, r>=2 = 0
        if r==0
            D0(row)=p0(ax);
            Dz(row,iXi)=e(ax)*xiScale;
        elseif r==1
            Dz(row,iXi)=mult*e(ax)*xiScale;
        end

    else
        kk=knot-1; % internal knot 2->1, 3->2
        vidx=bez13_jet_varidx(kk,r,ax);
        Dz(row,vidx)=mult*jetScale(kk,r+1);
    end
end
end

function idx=bez13_jet_varidx(internalKnot,r,axis)
% internalKnot=1,2 ; r=0..6 ; axis=1..3
idx=(internalKnot-1)*21 + r*3 + axis;
end

function idx=bez13_jet_cpidx(seg,cp,axis,ncp)
idx=3*((seg-1)*ncp+(cp-1))+axis;
end

function F=bez13_jet_endpoint_difference_matrix(n)
% 14x14 matrix:
% rows 1..7  : left forward differences Delta^r P_0, r=0..6
% rows 8..14 : right forward differences Delta^r P_{n-r}, r=0..6
F=zeros(14,n+1);

for r=0:6
    for k=0:r
        c=(-1)^(r-k)*nchoosek(r,k);
        F(r+1,k+1)=c;
        F(8+r,n-r+k+1)=c;
    end
end
end

function CP=bez13_jet_reconstruct_cp(z,M)
v=M.c0+M.Gcp*z;
CP=zeros(3,M.ncp,M.nseg);

for q=1:M.nseg
    for j=1:M.ncp
        for ax=1:3
            CP(ax,j,q)=v(bez13_jet_cpidx(q,j,ax,M.ncp));
        end
    end
end
end

function [As,bs]=bez13_jet_scale_ineq(A,b)
s=max(abs(A),[],2);
s=max(s,abs(b));
s=max(s,1);
As=A./s;
bs=b./s;
end

function [c6res,r0,r1]=bez13_jet_full_audit(CP,xi,M,G)
n=M.degree;
c6res=0;

for q=1:M.nseg-1
    ratio=M.h(q)/M.h(q+1);

    for r=0:6
        dL=zeros(3,1);
        dR=zeros(3,1);

        for k=0:r
            c=(-1)^(r-k)*nchoosek(r,k);
            dL=dL+c*CP(:,n-r+k+1,q);
            dR=dR+c*CP(:,k+1,q+1);
        end

        c6res=max(c6res,norm(dL-ratio^r*dR,inf));
    end
end

p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);
r0=0;
r1=0;

for j=0:6
    t0=p0+(j*M.h(1)/n)*e*xi;
    t1=p0+(1-j*M.h(end)/n)*e*xi;
    r0=max(r0,norm(CP(:,j+1,1)-t0,inf));
    r1=max(r1,norm(CP(:,n-j+1,M.nseg)-t1,inf));
end
end

function Jgeo=bez13_geo_objective_value(CP,G)
% Exact geometric guide-proximity objective used by Step 3.
Jgeo=0;
nseg=size(CP,3);
ncp=size(CP,2);
W=G.waypoints;

for q=1:nseg
    for j=1:ncp
        tau=(j-1)/(ncp-1);
        target=(1-tau)*W(:,q)+tau*W(:,q+1);
        d=CP(:,j,q)-target;
        Jgeo=Jgeo+d.'*d;
    end
end
end

function A=poly13_stable_bernstein_to_power(B)
% Convert degree-13 Bernstein coefficients to ordinary power coefficients.
% This conversion is OUTPUT/DIAGNOSTIC ONLY and is not used to construct
% the optimization problem or certify safety.
%
% B_j^n(t) = C(n,j) t^j (1-t)^(n-j)
%
% Therefore the coefficient of t^k is
%   sum_{j=0}^k B_j C(n,j) C(n-j,k-j) (-1)^(k-j).

n=size(B,2)-1;
nseg=size(B,3);
A=zeros(size(B));

for q=1:nseg
    for ax=1:3
        for k=0:n
            ak=0;
            for j=0:k
                ak=ak + B(ax,j+1,q) * nchoosek(n,j) * ...
                    nchoosek(n-j,k-j) * (-1)^(k-j);
            end
            A(ax,k+1,q)=ak;
        end
    end
end
end

function J7=bez13_jet_J7(CP,M)
n=M.degree;
D7=bez13_difference_matrix(n,7);
M6=bez13_bernstein_gram(6);
fac=factorial(n)/factorial(n-7);
J7=0;

for q=1:M.nseg
    for ax=1:3
        p=CP(ax,:,q).';
        d=D7*p;
        J7=J7+(fac^2)/(M.h(q)^13)*(d.'*M6*d);
    end
end
end

function M=bez13_build_model(G,C)
n=13;
nseg=3;
ncp=n+1;

% Segment parameter lengths follow the three certified guide segments.
W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
h=segLen/sum(segLen);
h=h(:).';

ncpvar=3*nseg*ncp;
iXi=ncpvar+1;
nvar=iXi;

% ------------------------------
% Equality constraints
% ------------------------------
Aeq=zeros(0,nvar);
beq=zeros(0,1);

p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

% Start and final spatial jets 0..6.
% They represent a straight nominal extension scaled by recovery xi:
% p_s = e*xi, p_ss ... p^(6) = 0.
for ax=1:3
    for j=0:6
        r=zeros(1,nvar);
        r(bez13_idx(1,j+1,ax,ncp))=1;
        r(iXi)=-(j*h(1)/n)*e(ax);
        Aeq(end+1,:)=r; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end

    for j=0:6
        r=zeros(1,nvar);
        r(bez13_idx(nseg,n-j+1,ax,ncp))=1;
        r(iXi)=-(1-j*h(end)/n)*e(ax);
        Aeq(end+1,:)=r; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end
end

% C0..C6 between neighboring degree-13 pieces.
% Finite-difference form with span-length ratios avoids explicit h^-6.
for q=1:nseg-1
    ratio=h(q)/h(q+1);

    for rord=0:6
        rp=ratio^rord;

        for ax=1:3
            row=zeros(1,nvar);

            % left end r-th forward difference
            for k=0:rord
                c=(-1)^(rord-k)*nchoosek(rord,k);
                cpidx=n-rord+k+1;
                id=bez13_idx(q,cpidx,ax,ncp);
                row(id)=row(id)+c;
            end

            % right start r-th forward difference
            for k=0:rord
                c=(-1)^(rord-k)*nchoosek(rord,k);
                cpidx=k+1;
                id=bez13_idx(q+1,cpidx,ax,ncp);
                row(id)=row(id)-rp*c;
            end

            Aeq(end+1,:)=row; %#ok<AGROW>
            beq(end+1,1)=0; %#ok<AGROW>
        end
    end
end

% ------------------------------
% Corridor hard inequalities
% ------------------------------
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('segment',{},'cp',{},'plane',{});

for q=1:nseg
    CC=C.cells{q};

    for j=1:ncp
        for pl=1:size(CC.A,1)
            row=zeros(1,nvar);

            for ax=1:3
                row(bez13_idx(q,j,ax,ncp))=-CC.A(pl,ax);
            end

            Aineq(end+1,:)=row; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(pl); %#ok<AGROW>

            safeMeta(end+1)=struct( ...
                'segment',q,'cp',j,'plane',pl); %#ok<AGROW>
        end
    end
end

% ------------------------------
% Exact J7 Hessian
% ------------------------------
D7=bez13_difference_matrix(n,7);    % 7 x 14
M6=bez13_bernstein_gram(6);         % integral B^6_i B^6_j
fac=factorial(n)/factorial(n-7);

H=zeros(nvar);
for q=1:nseg
    Hseg=(fac^2)/(h(q)^13) * (D7.'*M6*D7);

    for ax=1:3
        ids=arrayfun(@(j) bez13_idx(q,j,ax,ncp),1:ncp);
        H(ids,ids)=H(ids,ids)+2*Hseg;
    end
end

% xi has no direct J7 penalty; it is selected through endpoint/corridor coupling.
% Explicitly remove floating-point assembly asymmetry before quadprog.
% This does not change the quadratic form.
H=0.5*(H+H.');

% Scale the objective by one positive scalar only; optimum is unchanged.
d=max(abs(diag(H)));
if d>0
    Hscaled=sparse(H/d);
else
    Hscaled=sparse(H);
end

M=struct();
M.degree=n;
M.nseg=nseg;
M.ncp=ncp;
M.ncpvar=ncpvar;
M.iXi=iXi;
M.nvar=nvar;
M.h=h;
M.Aeq=sparse(Aeq);
M.beq=beq;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hphysical=sparse(H);
M.Hscaled=Hscaled;
end

function idx=bez13_idx(seg,cp,axis,ncp)
idx=3*((seg-1)*ncp+(cp-1))+axis;
end

function D=bez13_difference_matrix(n,r)
% r-th forward differences of n+1 control points.
D=zeros(n-r+1,n+1);
for i=0:n-r
    for k=0:r
        D(i+1,i+k+1)=(-1)^(r-k)*nchoosek(r,k);
    end
end
end

function Gm=bez13_bernstein_gram(m)
% Exact Gram matrix integral_0^1 B_i^m(t) B_j^m(t) dt.
Gm=zeros(m+1);
for i=0:m
    for j=0:m
        Gm(i+1,j+1)= ...
            nchoosek(m,i)*nchoosek(m,j) / ...
            ((2*m+1)*nchoosek(2*m,i+j));
    end
end
end

function [As,bs,Aeqs,beqs]=bez13_scale_rows(A,b,Aeq,beq)
s=max(abs(A),[],2);
s=max(s,abs(b));
s=max(s,1);
As=A./s;
bs=b./s;

se=max(abs(Aeq),[],2);
se=max(se,abs(beq));
se=max(se,1);
Aeqs=Aeq./se;
beqs=beq./se;
end

function CP=bez13_unpack_cp(w,M)
CP=zeros(3,M.ncp,M.nseg);
for q=1:M.nseg
    for j=1:M.ncp
        for ax=1:3
            CP(ax,j,q)=w(bez13_idx(q,j,ax,M.ncp));
        end
    end
end
end

function rmax=bez13_continuity_audit(CP,M)
n=M.degree;
rmax=0;

for q=1:M.nseg-1
    ratio=M.h(q)/M.h(q+1);

    for r=0:6
        dL=zeros(3,1);
        dR=zeros(3,1);

        for k=0:r
            c=(-1)^(r-k)*nchoosek(r,k);
            dL=dL+c*CP(:,n-r+k+1,q);
            dR=dR+c*CP(:,k+1,q+1);
        end

        rmax=max(rmax,norm(dL-ratio^r*dR,inf));
    end
end
end

function [r0,r1]=bez13_endpoint_audit(CP,xi,M,G)
n=M.degree;
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

r0=0;
r1=0;

for j=0:6
    t0=p0+(j*M.h(1)/n)*e*xi;
    t1=p0+(1-j*M.h(end)/n)*e*xi;

    r0=max(r0,norm(CP(:,j+1,1)-t0,inf));
    r1=max(r1,norm(CP(:,n-j+1,M.nseg)-t1,inf));
end
end

function [sAll,pAll]=bez13_eval_dense(CP,M,N)
% Diagnostic sampling only.
n=M.degree;
bounds=[0,cumsum(M.h)];
bounds(end)=1;

sAll=linspace(0,1,N);
pAll=zeros(3,N);

for ii=1:N
    s=sAll(ii);

    q=find(s<=bounds(2:end)+16*eps,1,'first');
    if isempty(q), q=M.nseg; end

    a=bounds(q);
    b=bounds(q+1);
    if b>a
        t=(s-a)/(b-a);
    else
        t=0;
    end
    t=min(1,max(0,t));

    B=zeros(n+1,1);
    for j=0:n
        B(j+1)=nchoosek(n,j)*(t^j)*((1-t)^(n-j));
    end

    pAll(:,ii)=CP(:,:,q)*B;
end
end

function mslack=bez13_dense_exact_ellipsoid_slack(Pts,obs,P)
% Diagnostic only.
%
% Exact Euclidean payload-center clearance to each oriented ellipsoid:
%
%   slack = dist(point, ellipsoid) - (R_payload + d_margin)
%
% aud_point_ellipsoid_distance() already uses ob.center / ob.radii / ob.R
% and computes the exact Euclidean point-to-ellipsoid distance.
%
% Continuous-time safety is still certified by Bezier control-point
% containment in the exact-support corridor; this dense check is NOT used
% as the certificate.

Rreq=P.R_payload+P.d_margin;
mslack=Inf;

for k=1:numel(obs)
    for ii=1:size(Pts,2)
        dist=aud_point_ellipsoid_distance(Pts(:,ii),obs(k));
        val=dist-Rreq;
        mslack=min(mslack,val);
    end
end
end

function [xRef,INFO]=step3a_local_build_guide_reference(M,G,xi)
% Deterministic least-squares guide fit under endpoint equalities.
% Linear algebra only; no optimization solver.
%
% We fit all local Bezier control points to the certified guide polyline
% while exactly satisfying endpoint-jet equalities for fixed xi.

n=M.ncoef;
nvar=M.nvar_curve;

% Fix xi in joint endpoint equalities:
Aeq=M.Aeq(:,1:nvar);
beq=M.beq-M.Aeq(:,M.iXi)*xi;

x0=lsqminnorm(Aeq,beq);
Z=null(Aeq);

% Local Bezier target points along each assigned guide segment.
T=M.Tbez;
Bmat=blkdiag(T,T,T);

target=zeros(3*size(T,1),1);
spanCounter=zeros(1,3);

for q=1:M.nspan
    c=M.cellOfSpan(q);
    spanCounter(c)=spanCounter(c)+1;
    kLocal=spanCounter(c);
    nLocal=M.spanCount(c);

    ids=(q-1)*M.p+(1:M.p+1);

    for j=1:M.p+1
        tau=((kLocal-1)+(j-1)/M.p)/nLocal;
        pt=(1-tau)*G.waypoints(:,c)+tau*G.waypoints(:,c+1);

        target(ids(j))=pt(1);
        target(size(T,1)+ids(j))=pt(2);
        target(2*size(T,1)+ids(j))=pt(3);
    end
end

if isempty(Z)
    xRef=x0;
else
    AZ=Bmat*Z;
    rhs=target-Bmat*x0;
    y=lsqminnorm(AZ,rhs);
    xRef=x0+Z*y;
end

INFO=struct();
INFO.eq_residual=norm(Aeq*xRef-beq,inf);
INFO.guide_fit_inf=norm(Bmat*xRef-target,inf);
INFO.guide_fit_rms=sqrt(mean((Bmat*xRef-target).^2));
INFO.curve_inf_norm=norm(xRef,inf);
end

function idx=step3a_local_active_xyz_indices(activeCP,ncoef)
cp=find(activeCP);
idx=[cp, ncoef+cp, 2*ncoef+cp];
idx=idx(:).';
end

function [As,bs,Aeqs,beqs]=step3a_local_scale_rows(A,b,Aeq,beq)
% Algebraic row scaling only; feasible set is unchanged.
s=max(abs(A),[],2);
s=max(s,abs(b));
s=max(s,1);
As=A./s;
bs=b./s;

se=max(abs(Aeq),[],2);
se=max(se,abs(beq));
se=max(se,1);
Aeqs=Aeq./se;
beqs=beq./se;
end

function rmax=step3a_local_c6_bezier_audit(x,M)
% Independent C0..C6 audit across neighboring local Bezier pieces.
% Uses derivative-equivalent finite differences with span-length ratios.

n=M.ncoef;
Qx=M.Tbez*x(1:n);
Qy=M.Tbez*x(n+(1:n));
Qz=M.Tbez*x(2*n+(1:n));

rmax=0;

for q=1:M.nspan-1
    idsL=(q-1)*M.p+(1:M.p+1);
    idsR=q*M.p+(1:M.p+1);

    BL=[Qx(idsL),Qy(idsL),Qz(idsL)].';
    BR=[Qx(idsR),Qy(idsR),Qz(idsR)].';

    hL=M.breaks(q+1)-M.breaks(q);
    hR=M.breaks(q+2)-M.breaks(q+1);

    for r=0:6
        dL=zeros(3,1);
        dR=zeros(3,1);

        for k=0:r
            c=(-1)^(r-k)*nchoosek(r,k);
            dL=dL+c*BL(:,M.p-r+k+1);
            dR=dR+c*BR(:,k+1);
        end

        rr=dL-(hL/hR)^r*dR;
        rmax=max(rmax,norm(rr,inf));
    end
end
end

function [As,bs,Aeqs,beqs,sIneq,sEq]=step3a_c02_scale_rows(A,b,Aeq,beq)
% Pure algebraic row scaling. No constraint relaxation.
sIneq=max(abs(A),[],2);
sIneq=max(sIneq,abs(b));
sIneq=max(sIneq,1);
As=A./sIneq;
bs=b./sIneq;

sEq=max(abs(Aeq),[],2);
sEq=max(sEq,abs(beq));
sEq=max(sEq,1);
Aeqs=Aeq./sEq;
beqs=beq./sEq;
end

function [H,f]=step3a_c02_guide_deviation_qp(M,G)
% Sum squared Euclidean deviations of local Bezier CPs from the fixed
% certified Step-1 guide polyline. This is a secondary selector only.

n=M.ncoef;
H=zeros(3*n);
f=zeros(3*n,1);

W=G.waypoints;
cellSpanCounter=zeros(1,3);

for q=1:M.nspan
    c=M.cellOfSpan(q);
    cellSpanCounter(c)=cellSpanCounter(c)+1;
    kLocal=cellSpanCounter(c);
    nLocal=M.spanCount(c);

    E=M.pieceMaps{q};

    for j=1:M.p+1
        tau=((kLocal-1)+(j-1)/M.p)/nLocal;
        target=(1-tau)*W(:,c)+tau*W(:,c+1);

        er=E(j,:);

        for ax=1:3
            ia=(ax-1)*n+(1:n);
            H(ia,ia)=H(ia,ia)+2*(er.'*er);
            f(ia)=f(ia)-2*target(ax)*er.';
        end
    end
end

% Symmetrize only against floating accumulation noise.
H=0.5*(H+H.');
end

function M=step3a_c02_build_joint_lp(G,C)
p=7;
ncoef=22;
nspan=ncoef-p;
W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
frac=segLen/sum(segLen);
spanCount=step3a_allocate_spans(nspan,frac);
ub=[0,cumsum(frac)];
ub(end)=1;
breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx); b=ub(cidx+1); nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1, breaks=local; else, breaks=[breaks,local(2:end)]; end %#ok<AGROW>
    cellOfSpan(ss+(1:nc))=cidx;
    ss=ss+nc;
end
U=[repmat(breaks(1),1,p+1),breaks(2:end-1),repmat(breaks(end),1,p+1)];
[~,Tbez]=step3a_bs_bezier_decomposition_transform(U,p,ncoef);
pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

nvarCurve=3*ncoef;
nvarJoint=nvarCurve+1;
iXi=nvarJoint;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
idxAxis={ix,iy,iz};
Aeq=zeros(0,nvarJoint); beq=zeros(0,1);
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2); e=e/norm(e);
h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1}; E1=pieceMaps{end};

for ax=1:3
    ia=idxAxis{ax};
    for j=0:6
        rr=zeros(1,nvarJoint);
        rr(ia)=E0(j+1,:);
        rr(iXi)=-(j*h0/p)*e(ax);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end
    for j=0:6
        rr=zeros(1,nvarJoint);
        rr(ia)=E1(p-j+1,:);
        rr(iXi)=-(1-j*h1/p)*e(ax);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end
end

Aineq=zeros(0,nvarJoint); bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});
for q=1:nspan
    E=pieceMaps{q}; cellIdx=cellOfSpan(q); CC=C.cells{cellIdx};
    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            nn=CC.A(hh,:).';
            rr=zeros(1,nvarJoint);
            rr(ix)=-nn(1)*er; rr(iy)=-nn(2)*er; rr(iz)=-nn(3)*er;
            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
        end
    end
end

M=struct('p',p,'ncoef',ncoef,'nspan',nspan,'nvar_curve',nvarCurve, ...
    'nvar_joint',nvarJoint,'iXi',iXi,'spanCount',spanCount, ...
    'cellOfSpan',cellOfSpan,'breaks',breaks,'knots',U,'Tbez',Tbez, ...
    'pieceMaps',{pieceMaps},'Aeq',Aeq,'beq',beq,'Aineq',Aineq, ...
    'bineq',bineq,'safeMeta',safeMeta);
end

function [minSlack,meta,Qbez]=step3a_c02_bezier_postcheck(x,M,C)
ncoef=M.ncoef;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
Qx=M.Tbez*x(ix); Qy=M.Tbez*x(iy); Qz=M.Tbez*x(iz);
Qbez=[Qx(:),Qy(:),Qz(:)].';
minSlack=Inf;
meta=struct('span',NaN,'cell',NaN,'cp',NaN,'plane',NaN);
for q=1:M.nspan
    ids=(q-1)*M.p+(1:M.p+1);
    B=[Qx(ids),Qy(ids),Qz(ids)].'; CC=C.cells{M.cellOfSpan(q)};
    for j=1:M.p+1
        sl=CC.A*B(:,j)-CC.b; [v,k]=min(sl);
        if v<minSlack
            minSlack=v; meta.span=q; meta.cell=M.cellOfSpan(q); meta.cp=j; meta.plane=k;
        end
    end
end
end

function [r0,r1]=step3a_c02_endpoint_jet_residuals(x,xi,M,G)
p=M.p; ncoef=M.ncoef;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
Qx=M.Tbez*x(ix); Qy=M.Tbez*x(iy); Qz=M.Tbez*x(iz);
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2); e=e/norm(e);
h0=M.breaks(2)-M.breaks(1); h1=M.breaks(end)-M.breaks(end-1);
Bfirst=[Qx(1:p+1),Qy(1:p+1),Qz(1:p+1)].';
idsLast=(M.nspan-1)*p+(1:p+1);
Blast=[Qx(idsLast),Qy(idsLast),Qz(idsLast)].';
r0=0; r1=0;
for j=0:6
    target0=p0+(j*h0/p)*e*xi;
    target1=p0+(1-j*h1/p)*e*xi;
    r0=max(r0,norm(Bfirst(:,j+1)-target0,inf));
    r1=max(r1,norm(Blast(:,p-j+1)-target1,inf));
end
end

function R=step3a14_run_one_case(S,P,fid)
R=struct();
R.step1_pass=false;
R.corridor_initial_pass=false;
R.xi_geom=NaN;
R.xi_used=NaN;
R.additional_runout=NaN;
R.terminal_slack=NaN;
R.corridor_rebuilt_pass=false;
R.fixed_conflict_count=NaN;
R.candidate_exitflag=NaN;
R.direction_exitflag=NaN;
R.min_directional_gain=NaN;
R.alpha_required=NaN;
R.alpha_final=NaN;
R.analytic_correction_iters=NaN;
R.eq_residual=NaN;
R.min_original_hard_slack=NaN;
R.min_bezier_hard_slack=NaN;
R.pass=false;
R.failure_reason='';
R.control_points=[];

G0=aud_run_case(S,P,fid);
R.step1_pass=G0.pass;
if ~G0.pass
    R.failure_reason='STEP1_FAILED';
    return
end

obs=S.obs(G0.front.known_ids_at_trigger);
C0=step2_build_and_audit_corridor(G0,obs,P,fid);
R.corridor_initial_pass=C0.pass;
if ~C0.pass
    R.failure_reason='INITIAL_CORRIDOR_FAILED';
    return
end

p0=G0.front.payload_at_trigger;
e=G0.waypoints(:,3)-G0.waypoints(:,2);
e=e/norm(e);
n=G0.bypass_direction(:);
n=n-e*(e.'*n);
n=n/norm(n);
dBypass=G0.bypass_offset;
xiGeom=G0.rejoin_xi;

R.xi_geom=xiGeom;

% Minimum terminal-C6 rejoin.
[xiStar,Rterm]=step3a_whole_find_terminal_xi(xiGeom,p0,e,n,dBypass,C0.cells{3});

R.xi_used=xiStar;
R.additional_runout=xiStar-xiGeom;
R.terminal_slack=Rterm.minSlack;

aud_log(fid,'');
aud_log(fid,'[VARIABLE REJOIN]');
aud_log(fid,'xiGeom=%.17g xiUsed=%.17g additionalRunout=%.17e terminalSlack=%+.17e', ...
    xiGeom,xiStar,xiStar-xiGeom,Rterm.minSlack);

% Rebuild guide/corridor at variable xi.
G=G0;
G.rejoin_xi=xiStar;
W1=p0+dBypass*n;
W2=W1+xiStar*e;
W3=p0+xiStar*e;
G.waypoints=[p0,W1,W2,W3];
G.path_length=sum(vecnorm(diff(G.waypoints,1,2),2,1));

C=step2_build_and_audit_corridor(G,obs,P,fid);
R.corridor_rebuilt_pass=C.pass;
if ~C.pass
    R.failure_reason='REBUILT_CORRIDOR_FAILED';
    return
end

M=step3a_whole_build_matrices(G,C);

% Equality elimination.
x0=lsqminnorm(M.Aeq,M.beq);
Z=null(M.Aeq);
Ar=M.Aineq*Z;
br=M.bineq-M.Aineq*x0;

rowInf=max(abs(Ar),[],2);
matrixScale=max(1,norm(Ar,inf));
tauRow=512*eps(matrixScale)*max(size(Ar));

fixed=(rowInf<=tauRow);
active=~fixed;
fixedConflict=fixed & (br<0);
R.fixed_conflict_count=sum(fixedConflict);

aud_log(fid,'[REDUCED] nEq=%d rankEq=%d nullity=%d nIneq=%d fixed=%d active=%d fixedConflict=%d', ...
    size(M.Aeq,1),rank(M.Aeq,1e-10),size(Z,2),size(M.Aineq,1), ...
    sum(fixed),sum(active),R.fixed_conflict_count);

if any(fixedConflict)
    R.failure_reason='FIXED_CONFLICT';
    return
end

Aa=Ar(active,:);
ba=br(active);

% Moderate norm candidate.
[~,y0,ef0,~]=step3a_row_active_qp_feasibility(Aa,ba);
R.candidate_exitflag=ef0;
if isempty(y0)
    R.failure_reason='NO_ACTIVE_CANDIDATE';
    return
end

s0=ba-Aa*y0;

% Common improving direction.
rs=max(abs(Aa),[],2);
An=Aa./rs;

H=eye(size(Aa,2));
f=zeros(size(Aa,2),1);

opts=optimoptions('quadprog','Display','none', ...
    'ConstraintTolerance',1e-9, ...
    'OptimalityTolerance',1e-9, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',5000);

[dDir,~,efDir,~]=quadprog(H,f,An,-ones(size(An,1),1),[],[],[],[],[],opts);
R.direction_exitflag=efDir;

if isempty(dDir)
    R.failure_reason='NO_COMMON_IMPROVING_DIRECTION';
    return
end

gain=-Aa*dDir;
R.min_directional_gain=min(gain);

if ~(R.min_directional_gain>0)
    R.failure_reason='DIRECTION_NOT_STRICTLY_IMPROVING';
    return
end

neg=(s0<0);
if any(neg)
    alphaReq=max((-s0(neg))./gain(neg));
else
    alphaReq=0;
end
R.alpha_required=alphaReq;

alpha=alphaReq;
[y,x,eqRes,origSlack,bezSlack,~]=step3a_ci_eval_alpha( ...
    alpha,y0,dDir,x0,Z,M,C);

fullGain=-M.Aineq*(Z*dDir);

correctionIters=0;
while min(origSlack)<0 || bezSlack<0
    correctionIters=correctionIters+1;

    negRows=find(origSlack<0);

    if isempty(negRows)
        R.failure_reason='BEZIER_LINEAR_MISMATCH';
        return
    end

    if any(fullGain(negRows)<=0)
        R.failure_reason='NO_CORRECTION_DIRECTION';
        return
    end

    deltaReq=max((-origSlack(negRows))./fullGain(negRows));
    alphaTrial=aud_next_up(alpha+deltaReq);

    if alphaTrial<=alpha
        R.failure_reason='ALPHA_CORRECTION_STALLED';
        return
    end

    alpha=alphaTrial;
    [y,x,eqRes,origSlack,bezSlack,~]=step3a_ci_eval_alpha( ...
        alpha,y0,dDir,x0,Z,M,C);

    if correctionIters>20
        R.failure_reason='ALPHA_CORRECTION_LIMIT';
        return
    end
end

R.alpha_final=alpha;
R.analytic_correction_iters=correctionIters;
R.eq_residual=eqRes;
R.min_original_hard_slack=min(origSlack);
R.min_bezier_hard_slack=bezSlack;

ncoef=M.ncoef;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
R.control_points=[x(ix),x(iy),x(iz)].';

R.pass=(eqRes<=1e-8) && ...
       (R.min_original_hard_slack>=0) && ...
       (R.min_bezier_hard_slack>=0);

if ~R.pass
    R.failure_reason='FINAL_HARD_POSTCHECK_FAILED';
end

aud_log(fid,'[BSPLINE_D7 RESULT] candExit=%d dirExit=%d minGain=%.17e alphaReq=%.17e alphaFinal=%.17e corrIters=%d', ...
    R.candidate_exitflag,R.direction_exitflag,R.min_directional_gain, ...
    R.alpha_required,R.alpha_final,R.analytic_correction_iters);
aud_log(fid,'[BSPLINE_D7 RESULT] eqResidual=%.17e minOriginal=%+.17e minBezier=%+.17e PASS=%d', ...
    R.eq_residual,R.min_original_hard_slack,R.min_bezier_hard_slack,R.pass);
end

function [y,x,eqRes,origSlack,minBezier,meta]=step3a_ci_eval_alpha(alpha,y0,dDir,x0,Z,M,C)
y=y0+alpha*dDir;
x=x0+Z*y;

eqRes=norm(M.Aeq*x-M.beq,inf);
origSlack=M.bineq-M.Aineq*x;

ncoef=M.ncoef;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);

Qx=M.Tbez*x(ix);
Qy=M.Tbez*x(iy);
Qz=M.Tbez*x(iz);

minBezier=Inf;
meta=struct('span',NaN,'cell',NaN,'cp',NaN,'plane',NaN);

for q=1:M.nspan
    ids=(q-1)*M.p+(1:M.p+1);
    Bcp=[Qx(ids),Qy(ids),Qz(ids)].';
    CC=C.cells{M.cellOfSpan(q)};

    for j=1:M.p+1
        sl=CC.A*Bcp(:,j)-CC.b;
        [v,k]=min(sl);

        if v<minBezier
            minBezier=v;
            meta.span=q;
            meta.cell=M.cellOfSpan(q);
            meta.cp=j;
            meta.plane=k;
        end
    end
end
end

function [pass,y,exitflag,msg]=step3a_row_active_qp_feasibility(A,b)
% Find any moderate-norm point satisfying active reduced inequalities.
% Rows are normalized ONLY for the numerical solve. Original hard
% inequalities are always checked afterward.

nv=size(A,2);
if isempty(A)
    y=zeros(nv,1);
    exitflag=1;
    msg='No active rows.';
    pass=true;
    return
end

rs=max(abs(A),[],2);
rs(rs==0)=1;
As=A./rs;
bs=b./rs;

H=eye(nv);
f=zeros(nv,1);

opts=optimoptions('quadprog','Display','none', ...
    'ConstraintTolerance',1e-9, ...
    'OptimalityTolerance',1e-9, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',5000);

[y,~,exitflag,out]=quadprog(H,f,As,bs,[],[],[],[],[],opts);

if isempty(y)
    pass=false;
else
    pass=all(b-A*y>=0);
end
msg=out.message;
end

function [xiStar,Rstar]=step3a_whole_find_terminal_xi(xiGeom,p0,e,n,d,CC)
R0=step3a_xi_terminal_eval(xiGeom,p0,e,n,d,CC);

if R0.minSlack>=0
    xiStar=xiGeom;
    Rstar=R0;
    return
end

lo=xiGeom;
hi=max(aud_next_up(lo),2*lo);
Rhi=step3a_xi_terminal_eval(hi,p0,e,n,d,CC);

nExpand=0;
while Rhi.minSlack<0
    lo=hi;
    hi=2*hi;
    Rhi=step3a_xi_terminal_eval(hi,p0,e,n,d,CC);
    nExpand=nExpand+1;
    if nExpand>60
        error('STEP3AWHOLE:Bracket','Could not bracket terminal-feasible xi.');
    end
end

for it=1:100
    mid=0.5*(lo+hi);
    if mid==lo || mid==hi
        break
    end

    Rmid=step3a_xi_terminal_eval(mid,p0,e,n,d,CC);

    if Rmid.minSlack>=0
        hi=mid;
        Rhi=Rmid;
    else
        lo=mid;
    end

    if hi-lo <= 16*eps(max(1,abs(hi)))
        break
    end
end

xiStar=hi;
Rstar=step3a_xi_terminal_eval(xiStar,p0,e,n,d,CC);

while Rstar.minSlack<0
    xiStar=aud_next_up(xiStar);
    Rstar=step3a_xi_terminal_eval(xiStar,p0,e,n,d,CC);
end
end

function M=step3a_whole_build_matrices(G,C)
p=7;
ncoef=22;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
frac=segLen/sum(segLen);
spanCount=step3a_allocate_spans(nspan,frac);

ub=[0,cumsum(frac)];
ub(end)=1;

breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx);
    b=ub(cidx+1);
    nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1
        breaks=local;
    else
        breaks=[breaks,local(2:end)]; %#ok<AGROW>
    end
    cellOfSpan(ss+(1:nc))=cidx;
    ss=ss+nc;
end

U=[repmat(breaks(1),1,p+1),breaks(2:end-1),repmat(breaks(end),1,p+1)];
[~,Tbez]=step3a_bs_bezier_decomposition_transform(U,p,ncoef);

pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

nvar=3*ncoef;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);
idxAxis={ix,iy,iz};

Aeq=zeros(0,nvar);
beq=zeros(0,1);

p0=G.front.payload_at_trigger;
e=W(:,3)-W(:,2);
e=e/norm(e);
xi=G.rejoin_xi;

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

for ax=1:3
    ia=idxAxis{ax};

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax)+(j*h0/p)*e(ax)*xi; %#ok<AGROW>
    end

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax)+(1-j*h1/p)*e(ax)*xi; %#ok<AGROW>
    end
end

Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});
ineqCell=zeros(0,1);
ineqSpan=zeros(0,1);

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            nn=CC.A(hh,:).';
            rr=zeros(1,nvar);
            rr(ix)=-nn(1)*er;
            rr(iy)=-nn(2)*er;
            rr(iz)=-nn(3)*er;

            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
            ineqCell(end+1,1)=cellIdx; %#ok<AGROW>
            ineqSpan(end+1,1)=q; %#ok<AGROW>
        end
    end
end

M=struct();
M.p=p;
M.ncoef=ncoef;
M.nspan=nspan;
M.nvar=nvar;
M.spanCount=spanCount;
M.cellOfSpan=cellOfSpan;
M.breaks=breaks;
M.knots=U;
M.Tbez=Tbez;
M.Aeq=Aeq;
M.beq=beq;
M.Aineq=Aineq;
M.bineq=bineq;
M.safeMeta=safeMeta;
M.ineqCell=ineqCell;
M.ineqSpan=ineqSpan;
end

function R=step3a_xi_terminal_eval(xi,p0,e,n,d,CC)
% Rebuild only the path-length parameterization quantities that depend on xi.
% Geometry:
%   [d, xi, d]

p=7;
nspan=15;
segLen=[d,xi,d];
frac=segLen/sum(segLen);
spanCount=step3a_allocate_spans(nspan,frac);

% Final guide segment occupies fraction frac(3) and has spanCount(3) spans.
hFinal=frac(3)/spanCount(3);

factor=zeros(1,7);
minSlackByJ=zeros(1,7);
worstPlaneByJ=zeros(1,7);

minSlack=Inf;
worstJ=NaN;
worstPlane=NaN;

for j=0:6
    factor(j+1)=1-j*hFinal/p;
    Bj=p0+factor(j+1)*xi*e;

    sl=CC.A*Bj-CC.b;
    [v,k]=min(sl);

    minSlackByJ(j+1)=v;
    worstPlaneByJ(j+1)=k;

    if v<minSlack
        minSlack=v;
        worstJ=j;
        worstPlane=k;
    end
end

% Also report the shifted guide points for later reconstruction.
W1=p0+d*n;
W2=W1+xi*e;
W3=p0+xi*e;

R=struct();
R.xi=xi;
R.segment_lengths=segLen;
R.fractions=frac;
R.spanCount=spanCount;
R.hFinal=hFinal;
R.factor=factor;
R.minSlackByJ=minSlackByJ;
R.worstPlaneByJ=worstPlaneByJ;
R.minSlack=minSlack;
R.worstJ=worstJ;
R.worstPlane=worstPlane;
R.waypoints=[p0,W1,W2,W3];
end

function M=step3a_jet_build_matrices(G,C,finalOrder)
p=7;
ncoef=22;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
frac=segLen/sum(segLen);
spanCount=step3a_allocate_spans(nspan,frac);

ub=[0,cumsum(frac)];
ub(end)=1;

breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx); b=ub(cidx+1); nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1, breaks=local; else, breaks=[breaks,local(2:end)]; end %#ok<AGROW>
    cellOfSpan(ss+(1:nc))=cidx;
    ss=ss+nc;
end

U=[repmat(breaks(1),1,p+1),breaks(2:end-1),repmat(breaks(end),1,p+1)];
[~,Tbez]=step3a_bs_bezier_decomposition_transform(U,p,ncoef);

pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

nvar=3*ncoef;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
idxAxis={ix,iy,iz};

Aeq=zeros(0,nvar); beq=zeros(0,1);

p0=G.front.payload_at_trigger;
e=W(:,3)-W(:,2); e=e/norm(e);
xi=G.rejoin_xi;

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

for ax=1:3
    ia=idxAxis{ax};

    % START always full jet 0..6.
    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax)+(j*h0/p)*e(ax)*xi; %#ok<AGROW>
    end

    % FINAL progressively 0..finalOrder.
    for j=0:finalOrder
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax)+(1-j*h1/p)*e(ax)*xi; %#ok<AGROW>
    end
end

Aineq=zeros(0,nvar); bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});
ineqCell=zeros(0,1);

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            n=CC.A(hh,:).';
            rr=zeros(1,nvar);
            rr(ix)=-n(1)*er;
            rr(iy)=-n(2)*er;
            rr(iz)=-n(3)*er;

            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
            ineqCell(end+1,1)=cellIdx; %#ok<AGROW>
        end
    end
end

M=struct('Aeq',Aeq,'beq',beq,'Aineq',Aineq,'bineq',bineq, ...
    'safeMeta',safeMeta,'ineqCell',ineqCell,'cellOfSpan',cellOfSpan);
end

function [rho,y,exitflag,msg]=step3a_diag_phase1(A,b)
% Robust phase-I LP:
%
%   min rho
%   s.t. A*y <= b + rho
%        rho >= 0
%
% rho has the SAME physical units as the original inequality slack.
%
% We scale each row but scale the rho coefficient consistently:
%
%   (A_i/s_i)y - (1/s_i)rho <= b_i/s_i
%
% so the problem is algebraically identical.

nv=size(A,2);

if isempty(A)
    rho=0;
    y=zeros(nv,1);
    exitflag=1;
    msg='No inequality rows.';
    return
end

srow=max(abs(A),[],2);
srow=max(srow,abs(b));
srow=max(srow,1);
srow(srow==0)=1;

As=A./srow;
bs=b./srow;
rhoCol=-1./srow;

f=[zeros(nv,1);1];
Aaug=[As,rhoCol];
lb=[-inf(nv,1);0];

opts=optimoptions('linprog','Display','none', ...
    'ConstraintTolerance',1e-9, ...
    'OptimalityTolerance',1e-9);

[z,~,exitflag,out]=linprog(f,Aaug,bs,[],[],lb,[],opts);

if isempty(z)
    rho=NaN;
    y=[];
else
    y=z(1:nv);
    rho=z(end);
end
msg=out.message;
end

function M=step3a_diag_build_matrices(G,C)
p=7;
ncoef=22;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
Ltot=sum(segLen);
frac=segLen/Ltot;
spanCount=step3a_allocate_spans(nspan,frac);

ub=[0,cumsum(frac)];
ub(end)=1;

breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx);
    b=ub(cidx+1);
    nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1
        breaks=local;
    else
        breaks=[breaks,local(2:end)]; %#ok<AGROW>
    end
    cellOfSpan(ss+(1:nc))=cidx;
    ss=ss+nc;
end

U=[repmat(breaks(1),1,p+1),breaks(2:end-1),repmat(breaks(end),1,p+1)];
[~,Tbez]=step3a_bs_bezier_decomposition_transform(U,p,ncoef);

pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

nvar=3*ncoef;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);

Aeq=zeros(0,nvar);
beq=zeros(0,1);

p0=G.front.payload_at_trigger;
e=W(:,3)-W(:,2);
e=e/norm(e);
xi=G.rejoin_xi;

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};
idxAxis={ix,iy,iz};

for ax=1:3
    ia=idxAxis{ax};
    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr;
        beq(end+1,1)=p0(ax)+(j*h0/p)*e(ax)*xi;
    end
    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr;
        beq(end+1,1)=p0(ax)+(1-j*h1/p)*e(ax)*xi;
    end
end

Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});
ineqCell=zeros(0,1);
ineqSpan=zeros(0,1);

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            n=CC.A(hh,:).';
            bb=CC.b(hh);

            rr=zeros(1,nvar);
            rr(ix)=-n(1)*er;
            rr(iy)=-n(2)*er;
            rr(iz)=-n(3)*er;

            Aineq(end+1,:)=rr;
            bineq(end+1,1)=-bb;
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
            ineqCell(end+1,1)=cellIdx; %#ok<AGROW>
            ineqSpan(end+1,1)=q; %#ok<AGROW>
        end
    end
end

M=struct();
M.p=p;
M.ncoef=ncoef;
M.nspan=nspan;
M.nvar=nvar;
M.spanCount=spanCount;
M.cellOfSpan=cellOfSpan;
M.breaks=breaks;
M.knots=U;
M.Tbez=Tbez;
M.Aeq=Aeq;
M.beq=beq;
M.Aineq=Aineq;
M.bineq=bineq;
M.safeMeta=safeMeta;
M.ineqCell=ineqCell;
M.ineqSpan=ineqSpan;
end

function C=step2_build_and_audit_corridor(G,obs,P,fid)
% STEP 2B
% Multi-plane convex corridor from the Step-1 V3 certified guide.
%
% No artificial bounding box.
% No obstacle sphere approximation.
% No support-boundary relaxation.
%
% Unbounded cells are allowed when the actual free space is open.
%
% Candidate normals are generated only from deterministic geometric
% features of each guide segment and exact ellipsoid:
%   - Step-1 certified support direction
%   - exact point-to-ellipsoid closest normal at start / midpoint / end
%   - exact segment-to-ellipsoid closest-pair normal
%
% Every retained plane is independently certified to contain the COMPLETE
% guide segment by endpoint evaluation. Because halfspace slack is affine
% on the segment, endpoint nonnegativity certifies the full segment.

W=G.waypoints;
Rreq=P.R_payload+P.d_margin;
nseg=size(W,2)-1;

if nseg~=3
    error('STEP2B:UnexpectedGuide','Step-1 guide must have exactly 3 segments.');
end

mid=W(:,3)-W(:,2);
if norm(mid)<=0
    error('STEP2B:ZeroProgress','Middle guide segment has zero length.');
end
e=mid/norm(mid);

nBy=G.bypass_direction(:);
nBy=nBy-e*(e.'*nBy);
nBy=nBy/norm(nBy);

baseDIR=[-e,nBy,e];
srcBase={'STEP1_BEFORE_-e','STEP1_BYPASS','STEP1_AFTER_+e'};

CELLS=cell(1,nseg);
PLANELOG=cell(1,nseg);

aud_log(fid,'');
aud_log(fid,'[STEP2B] MULTI-PLANE EXACT-SUPPORT CORRIDOR');
aud_log(fid,'[STEP2B] no artificial box | no b relaxation | unbounded allowed if free space is open');

for sidx=1:nseg
    P0=W(:,sidx);
    P1=W(:,sidx+1);
    PM=0.5*(P0+P1);

    candN=[];
    candSrc={};
    candObs=[];

    for i=1:numel(obs)
        candN(:,end+1)=baseDIR(:,sidx); %#ok<AGROW>
        candSrc{end+1}=[srcBase{sidx} '_OBS']; %#ok<AGROW>
        candObs(end+1)=i; %#ok<AGROW>
    end

    for i=1:numel(obs)
        Q={P0,PM,P1};
        qname={'START','MID','END'};

        for qk=1:3
            [dist,pEll]=step2b_point_ellipsoid_closest(Q{qk},obs(i));
            if dist>0
                n=(Q{qk}-pEll)/dist;
                candN(:,end+1)=n; %#ok<AGROW>
                candSrc{end+1}=['POINT_' qname{qk}]; %#ok<AGROW>
                candObs(end+1)=i; %#ok<AGROW>
            end
        end

        [distSeg,~,pSeg,pEll]=step2b_segment_ellipsoid_closest_pair(P0,P1,obs(i));
        if distSeg>0
            n=(pSeg-pEll)/distSeg;
            candN(:,end+1)=n; %#ok<AGROW>
            candSrc{end+1}='SEGMENT_CLOSEST'; %#ok<AGROW>
            candObs(end+1)=i; %#ok<AGROW>
        end
    end

    A=[];
    b=[];
    kept=struct('obstacle_id',{},'source',{},'normal',{},'support_b',{}, ...
        'min_segment_slack',{},'scalar_slack',{},'expanded_slack',{});
    rejected=struct('obstacle_id',{},'source',{},'reason',{},'min_slack',{});

    aud_log(fid,'');
    aud_log(fid,'[CELL %d] candidatePlanes=%d',sidx-1,size(candN,2));

    for k=1:size(candN,2)
        i=candObs(k);
        n=candN(:,k);

        if norm(n)==0 || any(~isfinite(n))
            rejected(end+1)=struct('obstacle_id',obs(i).id,'source',candSrc{k}, ...
                'reason','INVALID_NORMAL','min_slack',NaN); %#ok<AGROW>
            continue
        end
        n=n/norm(n);

        bi=aud_support(obs(i),n)+Rreq;

        sc=[n.'*P0-bi,n.'*P1-bi];
        ex=[n(1)*P0(1)+n(2)*P0(2)+n(3)*P0(3)-bi, ...
            n(1)*P1(1)+n(2)*P1(2)+n(3)*P1(3)-bi];
        minsl=min([sc,ex]);

        if minsl<0
            rejected(end+1)=struct('obstacle_id',obs(i).id,'source',candSrc{k}, ...
                'reason','CUTS_GUIDE','min_slack',minsl); %#ok<AGROW>
            continue
        end

        isdup=false;
        for j=1:numel(kept)
            if kept(j).obstacle_id==obs(i).id && ...
               norm(kept(j).normal-n)<=1e-12 && ...
               abs(kept(j).support_b-bi)<=1e-12
                isdup=true;
                break
            end
        end
        if isdup
            rejected(end+1)=struct('obstacle_id',obs(i).id,'source',candSrc{k}, ...
                'reason','DUPLICATE','min_slack',minsl); %#ok<AGROW>
            continue
        end

        A=[A;n.']; %#ok<AGROW>
        b=[b;bi]; %#ok<AGROW>
        kept(end+1)=struct('obstacle_id',obs(i).id,'source',candSrc{k}, ...
            'normal',n,'support_b',bi,'min_segment_slack',minsl, ...
            'scalar_slack',sc,'expanded_slack',ex); %#ok<AGROW>
    end

    if isempty(A)
        C=step2_fail_case(sprintf('CELL_%d_NO_VALID_PLANES',sidx-1));
        return
    end

    guideMin=min([A*P0-b;A*P1-b]);
    rankA=rank(A,1e-12);
    [r,strict,unboundedCheb]=step2_chebyshev_radius(A,b);
    [isBounded,recessionWitness]=step2b_boundedness(A);

    CELLS{sidx}=struct( ...
        'A',A,'b',b,'segment_index',sidx, ...
        'guide_start',P0,'guide_end',P1, ...
        'n_planes',size(A,1),'rankA',rankA, ...
        'strict_interior',strict,'cheb_radius',r, ...
        'cheb_unbounded',unboundedCheb, ...
        'is_bounded',isBounded, ...
        'recession_witness',recessionWitness);

    PLANELOG{sidx}=struct('kept',kept,'rejected',rejected);

    aud_log(fid,'[CELL %d] kept=%d rejected=%d rank(A)=%d guideMinSlack=%+.17e', ...
        sidx-1,numel(kept),numel(rejected),rankA,guideMin);
    aud_log(fid,'[CELL %d] strictInterior=%d chebRadius=%s bounded=%d', ...
        sidx-1,strict,step2b_numstr(r),isBounded);

    for i=1:numel(obs)
        if isempty(kept)
            kk=[];
        else
            kk=find([kept.obstacle_id]==obs(i).id);
        end
        if isempty(rejected)
            rr=[];
        else
            rr=find([rejected.obstacle_id]==obs(i).id);
        end
        if isempty(kk)
            minK=NaN;
        else
            minK=min([kept(kk).min_segment_slack]);
        end
        aud_log(fid,'[CELL %d OBS %d] keptPlanes=%d rejected=%d minKeptSlack=%+.17e', ...
            sidx-1,obs(i).id,numel(kk),numel(rr),minK);
    end
end

guideMin=zeros(1,nseg);
guidePass=false(1,nseg);
strictCellPass=true;
supportStructurePass=true;

aud_log(fid,'');
aud_log(fid,'[AUDIT] CELL GEOMETRY');

for sidx=1:nseg
    Cc=CELLS{sidx};
    vals=step2b_segment_kernel_slacks(Cc,W(:,sidx),W(:,sidx+1));
    guideMin(sidx)=min(vals);
    guidePass(sidx)=guideMin(sidx)>=0;

    rn=vecnorm(Cc.A,2,2);
    if any(abs(rn-1)>128*eps(1))
        supportStructurePass=false;
    end
    strictCellPass=strictCellPass && Cc.strict_interior;

    aud_log(fid,'[CELL %d] nPlanes=%d rank=%d bounded=%d strict=%d minGuideSlack=%+.17e', ...
        sidx-1,Cc.n_planes,Cc.rankA,Cc.is_bounded,Cc.strict_interior,guideMin(sidx));
end

npair=nseg-1;
OVER=repmat(struct('left_cell',0,'right_cell',0,'radius',NaN, ...
    'strict',false,'unbounded',false,'bounded',false, ...
    'shared_waypoint_slack',NaN),1,npair);

sharedPass=false(1,npair);
sharedSlack=zeros(1,npair);
strictOverlapPass=true;

aud_log(fid,'');
aud_log(fid,'[AUDIT] CONSECUTIVE OVERLAPS');

for k=1:npair
    A=[CELLS{k}.A;CELLS{k+1}.A];
    b=[CELLS{k}.b;CELLS{k+1}.b];

    [r,strict,unbounded]=step2_chebyshev_radius(A,b);
    [isBounded,~]=step2b_boundedness(A);

    q=W(:,k+1);
    vals=[step2_all_kernel_slacks(CELLS{k},q); ...
          step2_all_kernel_slacks(CELLS{k+1},q)];
    sl=min(vals);

    sharedSlack(k)=sl;
    sharedPass(k)=sl>=0;
    strictOverlapPass=strictOverlapPass && strict;

    OVER(k)=struct('left_cell',k-1,'right_cell',k,'radius',r, ...
        'strict',strict,'unbounded',unbounded,'bounded',isBounded, ...
        'shared_waypoint_slack',sl);

    aud_log(fid,'[%d->%d] strict=%d radius=%s bounded=%d sharedSlack=%+.17e sharedPass=%d', ...
        k-1,k,strict,step2b_numstr(r),isBounded,sl,sharedPass(k));
end

guideContainmentPass=all(guidePass);
sharedWaypointPass=all(sharedPass);
minGuideSlack=min(guideMin);
minSharedSlack=min(sharedSlack);

pass=guideContainmentPass && supportStructurePass && ...
    strictCellPass && strictOverlapPass && sharedWaypointPass;

nBounded=sum(cellfun(@(c)c.is_bounded,CELLS));
nUnbounded=nseg-nBounded;
nBoundedOverlap=sum([OVER.bounded]);
nUnboundedOverlap=npair-nBoundedOverlap;

aud_log(fid,'');
aud_log(fid,'[STEP2B-RESULT] guideContainment=%d supportStructure=%d strictCell=%d strictOverlap=%d sharedWaypoint=%d PASS=%d', ...
    guideContainmentPass,supportStructurePass,strictCellPass,strictOverlapPass,sharedWaypointPass,pass);
aud_log(fid,'[STEP2B-RESULT] minGuideSlack=%+.17e minSharedSlack=%+.17e', ...
    minGuideSlack,minSharedSlack);
aud_log(fid,'[STEP2B-RESULT] boundedCells=%d/%d boundedOverlaps=%d/%d', ...
    nBounded,nseg,nBoundedOverlap,npair);

C=struct();
C.pass=pass;
C.failure_reason='';
C.cell_count=nseg;
C.cells=CELLS;
C.plane_log=PLANELOG;
C.overlaps=OVER;
C.guide_containment_pass=guideContainmentPass;
C.support_structure_pass=supportStructurePass;
C.strict_cell_pass=strictCellPass;
C.strict_overlap_pass=strictOverlapPass;
C.shared_waypoint_pass=sharedWaypointPass;
C.min_guide_cell_slack=minGuideSlack;
C.min_shared_waypoint_slack=minSharedSlack;
C.min_finite_cell_radius=step2b_min_finite_cells(CELLS);
C.min_finite_overlap_radius=step2b_min_finite_overlap(OVER);
C.n_unbounded_cells=nUnbounded;
C.n_unbounded_overlaps=nUnboundedOverlap;
C.shared_waypoint_slack=sharedSlack;
end

function vals=step2b_segment_kernel_slacks(C,P0,P1)
m=size(C.A,1);
scalar0=zeros(m,1); scalar1=zeros(m,1);
expanded0=zeros(m,1); expanded1=zeros(m,1);

for i=1:m
    n=C.A(i,:).';
    scalar0(i)=n.'*P0-C.b(i);
    scalar1(i)=n.'*P1-C.b(i);
    expanded0(i)=n(1)*P0(1)+n(2)*P0(2)+n(3)*P0(3)-C.b(i);
    expanded1(i)=n(1)*P1(1)+n(2)*P1(2)+n(3)*P1(3)-C.b(i);
end

matrix0=C.A*P0-C.b;
matrix1=C.A*P1-C.b;
vals=[scalar0;scalar1;matrix0;matrix1;expanded0;expanded1];
end

function [isBounded,witness]=step2b_boundedness(A)
opts=optimoptions('linprog','Display','none', ...
    'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-10);

isBounded=true;
witness=zeros(3,1);

Aineq=-A;
bineq=zeros(size(A,1),1);
lb=-ones(3,1);
ub= ones(3,1);

for j=1:3
    for sgn=[-1,1]
        f=zeros(3,1);
        f(j)=-sgn;
        [d,~,ef]=linprog(f,Aineq,bineq,[],[],lb,ub,opts);
        if ef==1
            val=sgn*d(j);
            if val>1e-9
                isBounded=false;
                witness=d;
                return
            end
        end
    end
end
end

function s=step2b_numstr(x)
if isinf(x)
    s='+Inf';
else
    s=sprintf('%.17e',x);
end
end

function v=step2b_min_finite_cells(CELLS)
r=[];
for i=1:numel(CELLS)
    x=CELLS{i}.cheb_radius;
    if isfinite(x)
        r(end+1)=x; %#ok<AGROW>
    end
end
if isempty(r), v=Inf; else, v=min(r); end
end

function v=step2b_min_finite_overlap(O)
r=[];
for i=1:numel(O)
    if isfinite(O(i).radius)
        r(end+1)=O(i).radius; %#ok<AGROW>
    end
end
if isempty(r), v=Inf; else, v=min(r); end
end

function [dist,tau,pSeg,pEll]=step2b_segment_ellipsoid_closest_pair(A,B,ob)
A=A(:);
B=B(:);
dir=B-A;

if norm(dir)==0
    [dist,pEll]=step2b_point_ellipsoid_closest(A,ob);
    tau=0;
    pSeg=A;
    return
end

fun=@(t) step2b_point_ellipsoid_distance_only(A+t*dir,ob);
opts=optimset('Display','off','TolX',1e-13);
[tau,dist]=fminbnd(fun,0,1,opts);

d0=fun(0);
d1=fun(1);
if d0<dist
    dist=d0;
    tau=0;
end
if d1<dist
    dist=d1;
    tau=1;
end

pSeg=A+tau*dir;
[dist,pEll]=step2b_point_ellipsoid_closest(pSeg,ob);
end

function d=step2b_point_ellipsoid_distance_only(p,ob)
[d,~]=step2b_point_ellipsoid_closest(p,ob);
end

function [dist,pWorld]=step2b_point_ellipsoid_closest(p,ob)
q=ob.R.'*(p(:)-ob.center);
a=ob.radii(:);
a2=a.^2;

if sum((q./a).^2)<=1
    dist=0;
    pWorld=p(:);
    return
end

F=@(lam) sum((a2.*(q.^2))./((lam+a2).^2))-1;

lo=0;
hi=max([1;norm(q)^2;max(a2)]);
while F(hi)>0
    hi=2*hi;
end

for k=1:100
    mid=0.5*(lo+hi);
    if F(mid)>0
        lo=mid;
    else
        hi=mid;
    end
end

lam=0.5*(lo+hi);
x=(a2.*q)./(lam+a2);
pWorld=ob.center+ob.R*x;
dist=norm(p(:)-pWorld);
end

function vals=step2_all_kernel_slacks(C,q)
m=size(C.A,1);
scalar=zeros(m,1);
expanded=zeros(m,1);

for i=1:m
    n=C.A(i,:).';
    scalar(i)=n.'*q-C.b(i);
    expanded(i)=n(1)*q(1)+n(2)*q(2)+n(3)*q(3)-C.b(i);
end

matrix=C.A*q-C.b;
vals=[scalar;matrix;expanded];
end

function B=step3a_bspline_d7_feasibility(G,C,fid)
% BSPLINE_D7 ONLY.
%
% Representation held fixed from the earlier S2-R baseline:
%   degree p = 7
%   ncoef    = 22
%   nspan    = 15
%   simple interior knots -> structural C6.
%
% Cell transitions are no longer assigned by arbitrary uniform quarters.
% The 15 spans are allocated to the three guide segments according to guide
% arclength, with at least one span per segment; the knot locations align
% exactly with the guide/cell transition arclengths.
%
% Continuous cell containment:
%   native B-spline -> exact local Bezier extraction by standard knot
%   insertion; every local Bezier control point is constrained to the
%   corresponding convex cell.
%
% Objective is NOT J7. It is the exact integrated squared Euclidean
% deviation from the piecewise-linear Step-1 guiding path. 8-point
% Gauss-Legendre is exact for the squared degree-7-minus-linear integrand
% (degree <= 14) on every knot span.

p=7;
ncoef=22;
nspan=ncoef-p; % 15, held fixed for direct comparison with old S2-R.

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
Ltot=sum(segLen);
if Ltot<=0
    B=step3a_fail_bspline('ZERO_GUIDE_LENGTH');
    return
end

frac=segLen/Ltot;
spanCount=step3a_allocate_spans(nspan,frac);
if any(spanCount<1) || sum(spanCount)~=nspan
    error('STEP3A:SpanAllocation','Invalid path-driven span allocation.');
end

% Guide arclength parameter boundaries.
ub=[0,cumsum(frac)];
ub(end)=1;

% Build nonuniform unique break sequence aligned to guide transitions.
breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx);
    b=ub(cidx+1);
    nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1
        breaks=local;
    else
        breaks=[breaks,local(2:end)]; %#ok<AGROW>
    end
    cellOfSpan(ss+(1:nc))=cidx;
    ss=ss+nc;
end

if numel(breaks)~=nspan+1
    error('STEP3A:BreakCount','Expected %d breaks.',nspan+1);
end

U=[repmat(breaks(1),1,p+1),breaks(2:end-1),repmat(breaks(end),1,p+1)];

% Exact B-spline -> piecewise Bezier extraction.
[Ubez,Tbez]=step3a_bs_bezier_decomposition_transform(U,p,ncoef); %#ok<ASGLU>
expected=nspan*p+1;
if size(Tbez,1)~=expected
    error('STEP3A:BezierExtract','Expected %d decomposed points, got %d.',expected,size(Tbez,1));
end

pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

nvar=3*ncoef;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);

Aeq=zeros(0,nvar);
beq=zeros(0,1);

% Endpoint jets 0..6, same semantics as earlier S2-R.
p0=G.front.payload_at_trigger;
e=(W(:,3)-W(:,2));
e=e/norm(e);
xi=G.rejoin_xi;

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

for ax=1:3
    idxAxis={ix,iy,iz};
    ia=idxAxis{ax};

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr;
        beq(end+1,1)=p0(ax)+(j*h0/p)*e(ax)*xi;
    end

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr;
        beq(end+1,1)=p0(ax)+(1-j*h1/p)*e(ax)*xi;
    end
end

% Continuous safety via local Bezier convex hull.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            n=CC.A(hh,:).';
            bb=CC.b(hh);

            rr=zeros(1,nvar);
            rr(ix)=-n(1)*er;
            rr(iy)=-n(2)*er;
            rr(iz)=-n(3)*er;

            Aineq(end+1,:)=rr;
            bineq(end+1,1)=-bb;
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
        end
    end
end

% Exact L2 guide objective by 8-point Gauss-Legendre per knot span.
[xg,wg]=step3a_gauss8_01();
Cobj=zeros(3*nspan*numel(xg),nvar);
dobj=zeros(3*nspan*numel(xg),1);
rr=0;

for q=1:nspan
    ua=breaks(q);
    ubq=breaks(q+1);
    hs=ubq-ua;
    cellIdx=cellOfSpan(q);

    ga=ub(cellIdx);
    gb=ub(cellIdx+1);

    for k=1:numel(xg)
        u=ua+hs*xg(k);
        N=step3a_bs_basis_all(u,U,p,ncoef);

        if gb>ga
            alpha=(u-ga)/(gb-ga);
        else
            alpha=0;
        end
        alpha=min(1,max(0,alpha));
        pg=(1-alpha)*W(:,cellIdx)+alpha*W(:,cellIdx+1);

        sw=sqrt(wg(k)*hs);

        rr=rr+1;
        Cobj(rr,ix)=sw*N;
        dobj(rr)=sw*pg(1);

        rr=rr+1;
        Cobj(rr,iy)=sw*N;
        dobj(rr)=sw*pg(2);

        rr=rr+1;
        Cobj(rr,iz)=sw*N;
        dobj(rr)=sw*pg(3);
    end
end

% Row scaling for numerical conditioning. This does not change constraints.
[Ais,bis,ineqScale]=step3a_scale_rows(Aineq,bineq);
[Aes,bes,eqScale]=step3a_scale_rows(Aeq,beq);

opts=optimoptions('lsqlin', ...
    'Display','none', ...
    'Algorithm','interior-point', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',5000);

[x,resnorm,~,exitflag,output]=lsqlin(Cobj,dobj,Ais,bis,Aes,bes,[],[],[],opts);

if isempty(x) || exitflag<=0
    B=step3a_fail_bspline(['LSQLIN_FAILED_' num2str(exitflag)]);
    B.exitflag=exitflag;
    B.message=output.message;
    aud_log(fid,'[BSPLINE_D7] SOLVER FAIL exit=%d message=%s',exitflag,output.message);
    return
end

% ORIGINAL, unscaled independent certificate.
eqResidual=norm(Aeq*x-beq,inf);
safeSlack=bineq-Aineq*x;
minSafe=min(safeSlack);

% Endpoint extracted-Bezier residual.
cpX=x(ix); cpY=x(iy); cpZ=x(iz);
Qx=Tbez*cpX;
Qy=Tbez*cpY;
Qz=Tbez*cpZ;

endpointResidual=0;
for ax=1:3
    QQ={Qx,Qy,Qz};
    qv=QQ{ax};
    for j=0:6
        expected0=p0(ax)+(j*h0/p)*e(ax)*xi;
        expected1=p0(ax)+(1-j*h1/p)*e(ax)*xi;
        endpointResidual=max(endpointResidual,abs(qv(j+1)-expected0));
        endpointResidual=max(endpointResidual,abs(qv(end-j)-expected1));
    end
end

% Continuous span-cell certificate from extracted Bezier CPs using ORIGINAL
% cell inequalities. No sampling is needed for the proof.
minBezierSlack=Inf;
worstSpan=NaN;
worstCell=NaN;
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    Bcp=[Qx(ids),Qy(ids),Qz(ids)].';
    CC=C.cells{cellOfSpan(q)};
    for j=1:p+1
        sl=CC.A*Bcp(:,j)-CC.b;
        v=min(sl);
        if v<minBezierSlack
            minBezierSlack=v;
            worstSpan=q;
            worstCell=cellOfSpan(q);
        end
    end
end

% Structural extraction audit.
minEntry=min(Tbez,[],'all');
rowSumErr=max(abs(sum(Tbez,2)-1));
minNZ=step3a_min_nonzero(Tbez);

% Objective is already quadrature-weighted exact L2 integral.
l2obj=resnorm;

% This stage's hard pass uses original constraints. Solver tolerance is not
% used to excuse negative safety.
pass=(eqResidual<=1e-8) && (endpointResidual<=1e-8) && ...
     (minSafe>=0) && (minBezierSlack>=0) && ...
     (minEntry>=-1e-14) && (rowSumErr<=1e-12);

aud_log(fid,'');
aud_log(fid,'========================================================================================================================');
aud_log(fid,' [BSPLINE_D7] STEP 3A SAFE-SPATIAL FEASIBILITY');
aud_log(fid,'========================================================================================================================');
aud_log(fid,'[BSPLINE_D7] degree=%d ncoef=%d nspan=%d spanCountByCell=%s', ...
    p,ncoef,nspan,mat2str(spanCount));
aud_log(fid,'[BSPLINE_D7] guideFractions=%s',mat2str(frac,15));
aud_log(fid,'[BSPLINE_D7] knotBreaks=%s',mat2str(breaks,15));
aud_log(fid,'[BSPLINE_D7] nEq=%d rankEq=%d nIneq=%d', ...
    size(Aeq,1),rank(Aeq,1e-10),size(Aineq,1));
aud_log(fid,'[BSPLINE_D7] extraction minEntry=%+.17e minNonzero=%+.17e rowSumErr=%.17e', ...
    minEntry,minNZ,rowSumErr);
aud_log(fid,'[BSPLINE_D7] solver exit=%d resnorm(L2guide)=%.17e',exitflag,l2obj);
aud_log(fid,'[BSPLINE_D7] originalEqResidual=%.17e endpointResidual=%.17e', ...
    eqResidual,endpointResidual);
aud_log(fid,'[BSPLINE_D7] originalMinLinearSafeSlack=%+.17e',minSafe);
aud_log(fid,'[BSPLINE_D7] extractedBezierMinSafeSlack=%+.17e worstSpan=%d worstCell=%d', ...
    minBezierSlack,worstSpan,worstCell);
aud_log(fid,'[BSPLINE_D7] CONTINUOUS_CORRIDOR_CERT=%d C6_STRUCTURAL=1 PASS=%d', ...
    minBezierSlack>=0,pass);

B=struct();
B.pass=pass;
B.failure_reason='';
B.exitflag=exitflag;
B.message=output.message;
B.degree=p;
B.ncoef=ncoef;
B.nspan=nspan;
B.span_count_by_cell=spanCount;
B.breaks=breaks;
B.knots=U;
B.span_cell_assignment=cellOfSpan;
B.control_points=[cpX,cpY,cpZ].';
B.eq_residual=eqResidual;
B.endpoint_residual=endpointResidual;
B.min_safe_slack=min(minSafe,minBezierSlack);
B.linear_min_safe_slack=minSafe;
B.bezier_min_safe_slack=minBezierSlack;
B.l2_objective=l2obj;
B.C6_structural=true;
B.continuous_corridor_cert=minBezierSlack>=0;
B.extraction_min_entry=minEntry;
B.extraction_min_nonzero=minNZ;
B.extraction_row_sum_error=rowSumErr;
B.ineq_scale=ineqScale;
B.eq_scale=eqScale;
end

function B=step3a_fail_bspline(reason)
B=struct();
B.pass=false;
B.failure_reason=reason;
B.exitflag=NaN;
B.message='';
B.degree=7;
B.ncoef=22;
B.nspan=15;
B.span_count_by_cell=[];
B.breaks=[];
B.knots=[];
B.span_cell_assignment=[];
B.control_points=[];
B.eq_residual=NaN;
B.endpoint_residual=NaN;
B.min_safe_slack=NaN;
B.linear_min_safe_slack=NaN;
B.bezier_min_safe_slack=NaN;
B.l2_objective=NaN;
B.C6_structural=true;
B.continuous_corridor_cert=false;
B.extraction_min_entry=NaN;
B.extraction_min_nonzero=NaN;
B.extraction_row_sum_error=NaN;
B.ineq_scale=[];
B.eq_scale=[];
end

function n=step3a_allocate_spans(nspan,frac)
% Largest-remainder allocation with one span guaranteed per guide segment.
m=numel(frac);
n=ones(1,m);
remain=nspan-m;
if remain<0
    error('STEP3A:SpanCount','Too few spans.');
end

raw=remain*frac/sum(frac);
base=floor(raw);
n=n+base;
left=remain-sum(base);

rem=raw-base;
[~,ord]=sort(rem,'descend');
for k=1:left
    n(ord(k))=n(ord(k))+1;
end
end

function [A2,b2,scale]=step3a_scale_rows(A,b)
if isempty(A)
    A2=A;
    b2=b;
    scale=zeros(0,1);
    return
end
scale=max(abs(A),[],2);
scale=max(scale,abs(b));
scale(scale==0)=1;
A2=A./scale;
b2=b./scale;
end

function [Uout,T]=step3a_bs_bezier_decomposition_transform(U,p,ncp)
% Standard repeated knot insertion applied to the identity control polygon.
% This is the same tested construction used in S2-R V6.

Uout=U;
T=eye(ncp);

interior=unique(U(p+2:end-p-1));
interior=interior(interior>U(1) & interior<U(end));

for jj=1:numel(interior)
    u=interior(jj);
    while sum(Uout==u)<p
        [Uout,T]=step3a_bs_insert_knot_once(Uout,T,u,p);
    end
end
end

function [Up,Q]=step3a_bs_insert_knot_once(U,P,u,p)
% One Boehm knot insertion. Rows of P are control points (or transformation
% rows); the operation is a local convex combination.
%
% This indexing is copied from the previously validated S2-R V6 helper.

n=size(P,1)-1; % zero-based last control index
k=find(U<=u,1,'last')-1; % zero-based knot span index
s=sum(U==u);

Q=zeros(size(P,1)+1,size(P,2));

% Q_i = P_i, i=0..k-p
for i0=0:(k-p)
    Q(i0+1,:)=P(i0+1,:);
end

% Q_{i+1} = P_i, i=k-s..n
for i0=(k-s):n
    Q(i0+2,:)=P(i0+1,:);
end

% Blended control points.
for i0=(k-p+1):(k-s)
    den=U(i0+p+1)-U(i0+1);
    if den==0
        alpha=0;
    else
        alpha=(u-U(i0+1))/den;
    end
    Q(i0+1,:)=alpha*P(i0+1,:)+(1-alpha)*P(i0,:);
end

Up=[U(1:k+1),u,U(k+2:end)];
end

function N=step3a_bs_basis_all(u,U,p,ncoef)
% Cox-de Boor, endpoint-safe.
N0=zeros(1,ncoef+p);

if abs(u-U(end))<=16*eps(max(1,abs(U(end))))
    N=zeros(1,ncoef);
    N(end)=1;
    return
end

for i=1:(numel(U)-1)
    if U(i)<=u && u<U(i+1)
        N0(i)=1;
    end
end

for d=1:p
    N1=zeros(size(N0));
    for i=1:(numel(U)-d-1)
        a=0; b=0;
        den1=U(i+d)-U(i);
        den2=U(i+d+1)-U(i+1);
        if den1>0
            a=(u-U(i))/den1*N0(i);
        end
        if den2>0
            b=(U(i+d+1)-u)/den2*N0(i+1);
        end
        N1(i)=a+b;
    end
    N0=N1;
end
N=N0(1:ncoef);
end

function [x,w]=step3a_gauss8_01()
% 8-point Gauss-Legendre mapped from [-1,1] to [0,1].
z=[ ...
 -0.9602898564975363
 -0.7966664774136267
 -0.5255324099163290
 -0.1834346424956498
  0.1834346424956498
  0.5255324099163290
  0.7966664774136267
  0.9602898564975363];
ww=[ ...
 0.1012285362903763
 0.2223810344533745
 0.3137066458778873
 0.3626837833783620
 0.3626837833783620
 0.3137066458778873
 0.2223810344533745
 0.1012285362903763];
x=(z+1)/2;
w=ww/2;
end

function v=step3a_min_nonzero(A)
a=abs(A(:));
a=a(a>0);
if isempty(a)
    v=0;
else
    v=min(a);
end
end

function C=step2_fail_case(reason)
C=struct();
C.pass=false;
C.failure_reason=reason;
C.cell_count=0;
C.cells={};
C.separators=[];
C.cell_interiors=[];
C.overlaps=[];
C.guide_containment_pass=false;
C.support_structure_pass=false;
C.strict_cell_pass=false;
C.strict_overlap_pass=false;
C.shared_waypoint_pass=false;
C.min_guide_cell_slack=NaN;
C.min_shared_waypoint_slack=NaN;
C.min_finite_cell_radius=NaN;
C.min_finite_overlap_radius=NaN;
C.n_unbounded_cells=0;
C.n_unbounded_overlaps=0;
C.shared_waypoint_slack=[];
end

function [r,strict,unbounded]=step2_chebyshev_radius(A,b)
% A p >= b. Rows are unit normals.
%
% max r
% s.t. A p - r >= b, r>=0.
%
% r>0 proves strict interior.
% exitflag=-3 means the radius is unbounded, hence strict interior exists.

rowNorm=vecnorm(A,2,2);

f=[0;0;0;-1];
Aineq=[-A,rowNorm];
bineq=-b;
lb=[-Inf;-Inf;-Inf;0];

opts=optimoptions('linprog', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10);

[z,~,ef,out]=linprog(f,Aineq,bineq,[],[],lb,[],opts); %#ok<ASGLU>

if ef==1
    r=z(4);
    unbounded=false;
    strict=r>0;
elseif ef==-3
    r=Inf;
    unbounded=true;
    strict=true;
else
    error('STEP2AUDIT:ChebyshevLP','Chebyshev-radius LP failed: %s',out.message);
end
end

function R=aud_run_case(S,P,fid)
R=struct();
R.failure_reason='';

F=aud_sensor_trigger_frontend(S,P);
R.front=F;

aud_log(fid,'[FRONT] initialOutside=%d detectedAt=%s memory=%d trigger=%d t=%.3f TTC=%.3f known=%s', ...
    F.initial_outside_all,mat2str(F.detected_time,4),F.memory_pass,F.triggered, ...
    F.trigger_time,F.ttc,mat2str(F.known_ids_at_trigger));

if ~F.initial_outside_all
    R=aud_fail_result(R,'INITIAL_SENSOR_OUTSIDE_FAIL'); return
end
if ~F.memory_pass
    R=aud_fail_result(R,'DETECTION_MEMORY_FAIL'); return
end
if ~F.triggered
    R=aud_fail_result(R,'NO_TRIGGER'); return
end
if isempty(F.known_ids_at_trigger)
    R=aud_fail_result(R,'NO_KNOWN_OBSTACLE_AT_TRIGGER'); return
end

p0=F.payload_at_trigger;
e=aud_unit(S.e_nom);
obs=S.obs(F.known_ids_at_trigger);
Rreq=P.R_payload+P.d_margin;

[b1,b2]=aud_transverse_basis(e);

% Rejoin progress.
xiReq=zeros(1,numel(obs));
for i=1:numel(obs)
    xiReq(i)=aud_support(obs(i),e) ...
        + aud_vertical_nominal_system_support(e,P) ...
        + P.d_margin ...
        - e.'*p0;
end
[xiRejoin,kLim]=max(xiReq);
xiRejoin=max(0,xiRejoin);

% One common bypass direction.
obj=@(phi) aud_common_offset(phi,b1,b2,p0,obs,Rreq);
opts=optimset('Display','off','TolX',1e-12);
[phi0,~,~,~]=fminbnd(obj,0,2*pi,opts);
phiCand=[phi0,0,pi/2,pi,3*pi/2,2*pi];
dCand=arrayfun(obj,phiCand);
[dStar,jbest]=min(dCand);
phiStar=phiCand(jbest);

n=cos(phiStar)*b1+sin(phiStar)*b2;
n=n-e*(e.'*n);
n=n/norm(n);

% Recompute exact analytical requirements after projection.
req=zeros(1,numel(obs));
for i=1:numel(obs)
    req(i)=aud_support(obs(i),n)+Rreq-n.'*p0;
end
dStar=max([0,req]);

d0=dStar;
xi0=xiRejoin;

% =====================================================================
% V3 NUMERICAL ROBUSTIFICATION
% =====================================================================
% DO NOT relax b = h_E(n)+Rreq.
%
% Instead, move the GUIDE outward by the minimum representable amount
% until the original support inequalities are STRICTLY POSITIVE under
% several algebraically equivalent floating-point evaluation kernels.
%
% This is not an added physical margin or tuning threshold.
% It is a representability requirement: the mathematically safe boundary
% must not change sign merely because a later stage evaluates A*p-b via a
% different summation order.
% =====================================================================

dSteps=0;
while true
    W1=p0+dStar*n;
    W2=W1+e*xiRejoin;

    [minScalar,minMatrix,minExpanded]=aud_bypass_multikernel_slack( ...
        W1,W2,n,obs,Rreq);

    if min([minScalar,minMatrix,minExpanded])>0
        break
    end

    dNew=aud_next_up(dStar);
    if dNew<=dStar
        error('AUDIT:OutwardRoundD','Cannot outward-round bypass offset.');
    end
    dStar=dNew;
    dSteps=dSteps+1;

    if dSteps>10000
        error('AUDIT:OutwardRoundDLoop', ...
            'Bypass robustification exceeded 10000 representable steps.');
    end
end

% Rejoin robustification. Use BOTH the original nominal-system rejoin
% certificate and the payload +e corridor certificate, with multiple
% floating evaluation kernels.
xiSteps=0;
while true
    W3=p0+e*xiRejoin;

    [minNominal,minPayloadScalar,minPayloadMatrix,minPayloadExpanded]= ...
        aud_rejoin_multikernel_slack(W3,e,obs,P,Rreq);

    if min([minNominal,minPayloadScalar,minPayloadMatrix,minPayloadExpanded])>0
        break
    end

    xiNew=aud_next_up(xiRejoin);
    if xiNew<=xiRejoin
        error('AUDIT:OutwardRoundXi','Cannot outward-round rejoin xi.');
    end
    xiRejoin=xiNew;
    xiSteps=xiSteps+1;

    if xiSteps>10000
        error('AUDIT:OutwardRoundXiLoop', ...
            'Rejoin robustification exceeded 10000 representable steps.');
    end
end

% Final guide.
W0=p0;
W1=p0+dStar*n;
W2=W1+e*xiRejoin;
W3=p0+e*xiRejoin;
W=[W0,W1,W2,W3];

segLen=vecnorm(diff(W,1,2),2,1);
pathLength=sum(segLen);

% Original exact-support certificate, no safety tolerance.
supportSlack=zeros(3,numel(obs));
for i=1:numel(obs)
    supportSlack(1,i)=(-e).'*W0-aud_support(obs(i),-e)-Rreq;
    supportSlack(2,i)=n.'*W1-aud_support(obs(i),n)-Rreq;
    supportSlack(3,i)=e.'*W3-aud_support(obs(i),e)-Rreq;
end
supportPass=all(supportSlack(:)>0);

% Multi-kernel final audit.
[bSc,bMx,bEx]=aud_bypass_multikernel_slack(W1,W2,n,obs,Rreq);
[rNom,rSc,rMx,rEx]=aud_rejoin_multikernel_slack(W3,e,obs,P,Rreq);
multiKernelPass=min([bSc,bMx,bEx,rNom,rSc,rMx,rEx])>0;

% Exact Euclidean segment audit against ORIGINAL ellipsoids.
minSlack=Inf;
worstSeg=NaN;
worstObs=NaN;
for sidx=1:3
    A=W(:,sidx); B=W(:,sidx+1);
    for i=1:numel(obs)
        [dist,~,~]=aud_segment_ellipsoid_distance(A,B,obs(i));
        sl=dist-Rreq;
        if sl<minSlack
            minSlack=sl;
            worstSeg=sidx;
            worstObs=obs(i).id;
        end
    end
end
exactPass=minSlack>=0;

% Equality/progress audits remain separate from hard safety.
xiW=e.'*(W-p0);
dXi=diff(xiW);
eqScale=max(1,max(abs(xiW)));
eqTol=128*eps(eqScale);
progressPass=all(dXi>=-eqTol);

transverseResidual=norm((eye(3)-e*e.')*(W3-p0));
rejoinProgress=e.'*(W3-p0);
rejoinResidual=abs(rejoinProgress-xiRejoin);
rejoinPass=transverseResidual<=eqTol && rejoinResidual<=eqTol;

pass=supportPass && multiKernelPass && exactPass && progressPass && rejoinPass;

aud_log(fid,'[GUIDE] pTrigger=[%.6f %.6f %.6f] xiRejoin=%.17g limitingObs=%d', ...
    p0(1),p0(2),p0(3),xiRejoin,obs(kLim).id);
aud_log(fid,'[GUIDE] phi=%.12f n=[%.12f %.12f %.12f] eDotN=%+.17e', ...
    phiStar,n(1),n(2),n(3),e.'*n);
aud_log(fid,'[GUIDE] bypass=%.17g pathLength=%.12f',dStar,pathLength);

aud_log(fid,'[ROUND-V3] d original=%.17g final=%.17g ULPsteps=%d delta=%+.17e', ...
    d0,dStar,dSteps,dStar-d0);
aud_log(fid,'[ROUND-V3] xi original=%.17g final=%.17g ULPsteps=%d delta=%+.17e', ...
    xi0,xiRejoin,xiSteps,xiRejoin-xi0);

aud_log(fid,'[MULTIKERNEL-BYPASS] scalar=%+.17e matrix=%+.17e expanded=%+.17e pass=%d', ...
    bSc,bMx,bEx,min([bSc,bMx,bEx])>0);
aud_log(fid,'[MULTIKERNEL-REJOIN] nominal=%+.17e payloadScalar=%+.17e payloadMatrix=%+.17e payloadExpanded=%+.17e pass=%d', ...
    rNom,rSc,rMx,rEx,min([rNom,rSc,rMx,rEx])>0);

aud_log(fid,'[CERT] supportPass=%d minSupportSlack=%+.17e', ...
    supportPass,min(supportSlack(:)));
aud_log(fid,'[CERT] multiKernelPass=%d',multiKernelPass);
aud_log(fid,'[CERT] exactPass=%d minExactSlack=%+.17e worstSeg=%d worstObs=%d', ...
    exactPass,minSlack,worstSeg,worstObs);
aud_log(fid,'[EQUALITY] xiWaypoints=%s dXi=%s eqTol=%.17e', ...
    mat2str(xiW,17),mat2str(dXi,17),eqTol);
aud_log(fid,'[EQUALITY] transverseResidual=%.17e rejoinResidual=%.17e', ...
    transverseResidual,rejoinResidual);
aud_log(fid,'[CERT] progressPass=%d rejoinPass=%d GUIDE_PASS=%d', ...
    progressPass,rejoinPass,pass);

R.pass=pass;
R.support_pass=supportPass;
R.exact_pass=exactPass;
R.progress_pass=progressPass;
R.rejoin_pass=rejoinPass;
R.bypass_offset=dStar;
R.bypass_direction=n;
R.path_length=pathLength;
R.exact_min_clearance_slack=minSlack;
R.rejoin_xi=xiRejoin;
R.waypoints=W;
R.support_slack=supportSlack;
R.failure_reason='';
R.numeric=struct( ...
    'd0',d0,'d_final',dStar,'d_ulp_steps',dSteps, ...
    'xi0',xi0,'xi_final',xiRejoin,'xi_ulp_steps',xiSteps, ...
    'bypass_scalar_slack',bSc,'bypass_matrix_slack',bMx, ...
    'bypass_expanded_slack',bEx,'rejoin_nominal_slack',rNom, ...
    'rejoin_payload_scalar_slack',rSc,'rejoin_payload_matrix_slack',rMx, ...
    'rejoin_payload_expanded_slack',rEx,'multi_kernel_pass',multiKernelPass, ...
    'eq_tol',eqTol,'transverse_residual',transverseResidual, ...
    'rejoin_residual',rejoinResidual);
end

function R=aud_fail_result(R,reason)
R.pass=false;
R.support_pass=false;
R.exact_pass=false;
R.progress_pass=false;
R.rejoin_pass=false;
R.bypass_offset=NaN;
R.bypass_direction=nan(3,1);
R.path_length=NaN;
R.exact_min_clearance_slack=NaN;
R.rejoin_xi=NaN;
R.waypoints=nan(3,4);
R.support_slack=NaN;
R.failure_reason=reason;
end

function CASES=aud_build_cases()
% Deterministic scenario suite. All obstacle centers are created in a local
% frame tied to the nominal direction, then mapped to world coordinates.

CASES=repmat(aud_empty_case(),1,14);

% 1 obstacle baseline
CASES(1)=aud_make_case('C01_ONE', ...
    'single oriented ellipsoid, horizontal +x nominal motion', ...
    [1;0;0],1.0, ...
    [20,0.30,0.10], ...
    [1.00,0.75,1.10], ...
    [20,10,-15]);

% 2 obstacle baseline-like
CASES(2)=aud_make_case('C02_TWO_BASE', ...
    'two nearby oriented ellipsoids with opposite transverse offsets', ...
    [1;0;0],1.0, ...
    [20,0.20,0.05;24,-0.25,0.18], ...
    [1.00,0.72,1.05;0.92,0.78,1.12], ...
    [22,8,-10;-31,-12,20]);

% 4 obstacle mixed sizes/orientations
CASES(3)=aud_make_case('C03_FOUR_MIX', ...
    'four-obstacle cluster, mixed sizes and orientations', ...
    [1;0;0],1.0, ...
    [18,0.5,0.2;21,-0.4,-0.1;24,0.3,0.4;27,-0.2,0.0], ...
    [0.7,0.9,1.0;1.2,0.6,1.1;0.9,1.0,0.7;1.3,0.8,1.2], ...
    [15,20,-5;-35,10,25;48,-18,12;70,25,-20]);

% 8 obstacle dense cluster
CASES(4)=aud_make_dense_case('C04_EIGHT_DENSE', ...
    'eight-obstacle dense cluster', ...
    [1;0;0],1.0,8,18.0,1.55);

% 12 obstacle heavier-load front-end case
CASES(5)=aud_make_dense_case('C05_TWELVE_DENSE', ...
    'twelve-obstacle dense cluster', ...
    [1;0;0],1.0,12,17.0,1.20);

% Large ellipsoids
CASES(6)=aud_make_case('C06_LARGE', ...
    'large obstacles, mixed aspect ratios', ...
    [1;0;0],1.0, ...
    [20,0.5,0.2;25,-0.7,-0.3;30,0.2,0.5], ...
    [1.8,1.3,1.7;2.1,1.0,1.6;1.5,2.0,1.2], ...
    [25,10,30;-40,35,-15;65,-20,40]);

% Small ellipsoids
CASES(7)=aud_make_case('C07_SMALL', ...
    'small obstacles with mixed orientations', ...
    [1;0;0],1.0, ...
    [19,0.2,0.1;21,-0.2,-0.1;23,0.25,0.15;25,-0.15,0.0], ...
    [0.35,0.45,0.50;0.40,0.30,0.55;0.50,0.42,0.32;0.28,0.50,0.40], ...
    [10,45,-20;55,-30,15;-70,20,50;90,10,-35]);

% Slender/high-aspect-ratio ellipsoids
CASES(8)=aud_make_case('C08_SLENDER', ...
    'high-aspect-ratio ellipsoids, strongly rotated', ...
    [1;0;0],1.0, ...
    [20,0.0,0.0;24,0.3,-0.2;28,-0.4,0.3], ...
    [2.2,0.35,0.40;0.30,2.0,0.45;0.40,0.35,2.3], ...
    [55,35,70;-65,50,20;80,-40,60]);

% Horizontal diagonal path
CASES(9)=aud_make_case('C09_DIAG_XY', ...
    'diagonal horizontal nominal path', ...
    [1;1;0]/sqrt(2),1.2, ...
    [20,0.3,0.1;24,-0.4,0.2;28,0.2,-0.2], ...
    [1.0,0.8,1.0;0.8,1.1,0.9;1.2,0.7,1.1], ...
    [30,20,-10;-45,15,35;75,-25,10]);

% 3-D climbing nominal direction
CASES(10)=aud_make_case('C10_CLIMB', ...
    '3-D climbing nominal direction', ...
    aud_unit([1;0.25;0.20]),1.0, ...
    [20,0.4,0.2;24,-0.3,-0.2;28,0.2,0.3], ...
    [1.1,0.7,1.0;0.9,1.2,0.8;1.0,0.8,1.3], ...
    [25,35,10;-55,20,40;70,-30,-15]);

% 3-D descending nominal direction
CASES(11)=aud_make_case('C11_DESCEND', ...
    '3-D descending nominal direction', ...
    aud_unit([1;-0.20;-0.18]),1.1, ...
    [19,0.2,0.3;23,-0.35,-0.1;27,0.4,0.0], ...
    [0.9,1.0,0.8;1.2,0.7,1.1;0.8,1.3,1.0], ...
    [15,-25,45;-60,40,20;85,15,-30]);

% Close longitudinal spacing
CASES(12)=aud_make_case('C12_CLOSE_SPACING', ...
    'four obstacles with close longitudinal spacing', ...
    [1;0;0],0.9, ...
    [20.0,0.35,0.1;21.2,-0.3,-0.2;22.4,0.25,0.25;23.6,-0.2,-0.1], ...
    [0.9,0.8,1.0;0.8,1.0,0.9;1.0,0.75,1.1;0.85,1.05,0.8], ...
    [10,20,30;-25,35,-15;50,-20,45;-70,10,25]);

% Strong orientation sweep
CASES(13)=aud_make_case('C13_ORIENT_SWEEP', ...
    'six obstacles spanning strong orientation changes', ...
    [1;0;0],1.0, ...
    [18,0.4,0.1;20,-0.3,0.2;22,0.2,-0.2;24,-0.4,0.3;26,0.3,0.0;28,-0.2,-0.3], ...
    repmat([1.0,0.65,1.25],6,1), ...
    [0,0,0;30,15,20;60,30,40;90,45,60;120,60,80;150,75,100]);

% Different speed, same geometric architecture
CASES(14)=aud_make_case('C14_SPEED_1P8', ...
    'higher nominal speed with five mixed obstacles', ...
    aud_unit([1;0.35;0.05]),1.8, ...
    [20,0.2,0.2;23,-0.5,-0.1;26,0.4,0.3;29,-0.2,-0.2;32,0.3,0.1], ...
    [0.8,1.0,1.1;1.2,0.7,0.9;0.9,1.3,0.8;1.0,0.8,1.2;1.1,0.9,1.0], ...
    [20,10,15;-35,25,-20;55,-30,40;80,15,-35;110,40,25]);
end

function S=aud_empty_case()
S=struct('case_id','','case_desc','','p0',[0;0;0],'pg',[40;0;0], ...
    'v0',[1;0;0],'e_nom',[1;0;0],'T',40,'obs',struct([]));
end

function S=aud_make_dense_case(id,desc,e,speed,nobs,xi0,dx)
centers=zeros(nobs,3);
radii=zeros(nobs,3);
angles=zeros(nobs,3);
for k=1:nobs
    centers(k,1)=xi0+(k-1)*dx;
    centers(k,2)=0.55*sin(0.9*k);
    centers(k,3)=0.40*cos(0.7*k);
    radii(k,:)=[0.65+0.05*mod(k,4), 0.55+0.07*mod(k+1,3), 0.75+0.06*mod(k+2,5)];
    angles(k,:)=[17*k, -11*k, 23*k];
end
S=aud_make_case(id,desc,e,speed,centers,radii,angles);
end

function S=aud_make_case(id,desc,e,speed,localCenters,radii,anglesDeg)
e=aud_unit(e);
[b1,b2]=aud_transverse_basis(e);
B=[e,b1,b2];

p0=[0;0;0];
pg=p0+e*45;

n=size(localCenters,1);
obs=repmat(struct('center',[],'radii',[],'R',[],'S',[],'id',0),1,n);

for i=1:n
    q=localCenters(i,:).';
    c=p0+B*q;

    a=anglesDeg(i,:);
    Rloc=aud_Rz(deg2rad(a(1)))*aud_Ry(deg2rad(a(2)))*aud_Rx(deg2rad(a(3)));
    R=B*Rloc*B.';

    rr=radii(i,:).';
    obs(i)=struct('center',c,'radii',rr,'R',R,'S',R*diag(rr.^2)*R','id',i);
end

S=struct();
S.case_id=id;
S.case_desc=desc;
S.p0=p0;
S.pg=pg;
S.v0=e*speed;
S.e_nom=e;
S.T=norm(pg-p0)/speed;
S.obs=obs;
end

function P=aud_params()
P=struct();
P.d_margin=0.15;
P.R_payload=0.24;
P.R_uav=0.30;
P.R_cable=0.045;
P.L_cable=2.0;
P.sensor_R=15.0;
P.dt_sensor=0.05;
P.lookahead=5.0;
P.pred_dt=0.10;
end

function F=aud_sensor_trigger_frontend(S,P)
v=S.v0;
speed=norm(v);
if speed<=0
    error('AUDIT:ZeroSpeed','Nonzero nominal speed required.');
end

Tnom=norm(S.pg-S.p0)/speed;
no=numel(S.obs);

detected=false(1,no);
detect_t=nan(1,no);
initial_out=true(1,no);

% Nominal cable stays vertical in this Step-1 trigger replay.
pU0=S.p0+P.L_cable*[0;0;1];

for o=1:no
    rs=max(S.obs(o).radii); % sensor gate only
    initial_out(o)=norm(S.obs(o).center-pU0)>P.sensor_R+rs;
end

triggered=false;
trig_t=NaN;
ttc=NaN;
trig_obs=0;
trig_body='';
known_at=[];
payload_trig=[NaN;NaN;NaN];
first_detect=NaN;
memory_pass=true;
prev_known=false(1,no);

for t=0:P.dt_sensor:Tnom
    pL=S.p0+v*t;
    pU=pL+P.L_cable*[0;0;1];

    for o=1:no
        if ~detected(o)
            rs=max(S.obs(o).radii); % SENSOR ONLY
            if norm(S.obs(o).center-pU)<=P.sensor_R+rs
                detected(o)=true;
                detect_t(o)=t;
                if isnan(first_detect)
                    first_detect=t;
                end
            end
        end
    end

    if any(prev_known & ~detected)
        memory_pass=false;
    end
    prev_known=detected;

    ids=find(detected);
    if ~isempty(ids)
        [hit,tau,oid,body]=aud_predict_nominal_collision(pL,v,S.obs(ids),ids,P);
        if hit
            triggered=true;
            trig_t=t;
            ttc=tau;
            trig_obs=oid;
            trig_body=body;
            known_at=ids;
            payload_trig=pL;
            break
        end
    end
end

F=struct( ...
    'initial_outside_each',initial_out, ...
    'initial_outside_all',all(initial_out), ...
    'detected_time',detect_t, ...
    'first_detect_time',first_detect, ...
    'memory_pass',memory_pass, ...
    'triggered',triggered, ...
    'trigger_time',trig_t, ...
    'ttc',ttc, ...
    'trigger_obstacle_id',trig_obs, ...
    'trigger_body',trig_body, ...
    'known_ids_at_trigger',known_at, ...
    'payload_at_trigger',payload_trig);
end

function [hit,tau,globalObsId,bodyName]=aud_predict_nominal_collision(pNow,v,knownObs,globalIds,P)
hit=false; tau=Inf; globalObsId=0; bodyName='';

for dt=0:P.pred_dt:P.lookahead
    pL=pNow+v*dt;
    pU=pL+P.L_cable*[0;0;1];

    for io=1:numel(knownObs)
        [g,b]=aud_point_system_margin_nominal(pL,pU,knownObs(io),P);
        if g<=0
            hit=true;
            tau=dt;
            globalObsId=globalIds(io);
            bodyName=b;
            return
        end
    end
end
end

function [g,bodyName]=aud_point_system_margin_nominal(pL,pU,ob,P)
N=aud_add_cardinal(aud_fibonacci_sphere(722));
K=size(N,2);
H=zeros(K,1);

for k=1:K
    H(k)=aud_support(ob,N(:,k));
end

nPL=N.'*pL;
nPU=N.'*pU;

gL=max(nPL-H-P.R_payload-P.d_margin);
gU=max(nPU-H-P.R_uav-P.d_margin);
gC=max(min(nPL,nPU)-H-P.R_cable-P.d_margin);

parts=[gL gU gC];
[g,idx]=min(parts);
names={'Payload','UAV','Cable'};
bodyName=names{idx};
end

function d=aud_common_offset(phi,b1,b2,p0,obs,Rreq)
n=cos(phi)*b1+sin(phi)*b2;
n=n/norm(n);
req=zeros(1,numel(obs));
for i=1:numel(obs)
    req(i)=aud_support(obs(i),n)+Rreq-n.'*p0;
end
d=max([0,req]);
end

function [dmin,tmin,pmin]=aud_segment_ellipsoid_distance(A,B,ob)
A=A(:); B=B(:); dir=B-A;

if norm(dir)==0
    dmin=aud_point_ellipsoid_distance(A,ob);
    tmin=0; pmin=A; return
end

fun=@(t) aud_point_ellipsoid_distance(A+t*dir,ob);
opts=optimset('Display','off','TolX',1e-13);
[tmin,dmin]=fminbnd(fun,0,1,opts);

d0=fun(0); d1=fun(1);
if d0<dmin, dmin=d0; tmin=0; end
if d1<dmin, dmin=d1; tmin=1; end

pmin=A+tmin*dir;
end

function dist=aud_point_ellipsoid_distance(p,ob)
q=ob.R.'*(p(:)-ob.center);
a=ob.radii(:);
a2=a.^2;

if sum((q./a).^2)<=1
    dist=0;
    return
end

F=@(lam) sum((a2.*(q.^2))./((lam+a2).^2))-1;
lo=0;
hi=max([1;norm(q)^2;max(a2)]);
while F(hi)>0
    hi=2*hi;
end

for k=1:100
    mid=0.5*(lo+hi);
    if F(mid)>0
        lo=mid;
    else
        hi=mid;
    end
end
lam=0.5*(lo+hi);
x=(a2.*q)./(lam+a2);
dist=norm(q-x);
end

function h=aud_support(ob,n)
n=n(:);
h=n.'*ob.center+sqrt(max(0,n.'*ob.S*n));
end

function d=aud_vertical_nominal_system_support(n,P)
nz=n(3);
dL=P.R_payload;
dU=P.L_cable*nz+P.R_uav;
dC=max(0,P.L_cable*nz)+P.R_cable;
d=max([dL,dU,dC]);
end

function [b1,b2]=aud_transverse_basis(e)
e=aud_unit(e);

ref=[0;0;1];
if abs(e.'*ref)>0.9
    ref=[0;1;0];
end

b1=cross(e,ref);
b1=b1-e*(e.'*b1);
b1=b1/norm(b1);

b2=cross(e,b1);
b2=b2-e*(e.'*b2)-b1*(b1.'*b2);
b2=b2/norm(b2);
end

function v=aud_unit(v)
v=v(:);
v=v/norm(v);
end

function R=aud_Rx(a)
R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
end

function R=aud_Ry(a)
R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];
end

function R=aud_Rz(a)
R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
end

function D=aud_fibonacci_sphere(N)
k=(0:N-1)';
z=1-2*(k+0.5)/N;
phi=pi*(3-sqrt(5))*k;
r=sqrt(max(0,1-z.^2));
D=[r.*cos(phi) r.*sin(phi) z].';
end

function D=aud_add_cardinal(D)
C=[eye(3) -eye(3)];
D=[D C];
D=D./vecnorm(D,2,1);
end

function [mScalar,mMatrix,mExpanded]=aud_bypass_multikernel_slack(W1,W2,n,obs,Rreq)
% Evaluate the SAME exact-support inequality using several algebraically
% equivalent floating-point kernels. All must remain strictly positive.

mScalar=Inf;
mMatrix=Inf;
mExpanded=Inf;

A=repmat(n(:).',numel(obs),1);
b=zeros(numel(obs),1);

for i=1:numel(obs)
    b(i)=aud_support(obs(i),n)+Rreq;

    s1=n.'*W1-b(i);
    s2=n.'*W2-b(i);
    mScalar=min([mScalar,s1,s2]);

    ex1=n(1)*W1(1)+n(2)*W1(2)+n(3)*W1(3)-b(i);
    ex2=n(1)*W2(1)+n(2)*W2(2)+n(3)*W2(3)-b(i);
    mExpanded=min([mExpanded,ex1,ex2]);
end

v1=A*W1-b;
v2=A*W2-b;
mMatrix=min([v1;v2]);
end

function [mNominal,mScalar,mMatrix,mExpanded]=aud_rejoin_multikernel_slack(W3,e,obs,P,Rreq)
mNominal=Inf;
mScalar=Inf;
mMatrix=Inf;
mExpanded=Inf;

A=repmat(e(:).',numel(obs),1);
b=zeros(numel(obs),1);

for i=1:numel(obs)
    % Original nominal suspended-system rejoin certificate.
    sn=e.'*W3-aud_support(obs(i),e)- ...
        aud_vertical_nominal_system_support(e,P)-P.d_margin;
    mNominal=min(mNominal,sn);

    % Payload corridor certificate.
    b(i)=aud_support(obs(i),e)+Rreq;

    ss=e.'*W3-b(i);
    mScalar=min(mScalar,ss);

    ex=e(1)*W3(1)+e(2)*W3(2)+e(3)*W3(3)-b(i);
    mExpanded=min(mExpanded,ex);
end

mMatrix=min(A*W3-b);
end

function y=aud_next_up(x)
% Smallest practical representable increase constructed from MATLAB eps.
% This is used only when the ORIGINAL hard inequality evaluates negative.
% It is numerical outward rounding, not an added physical margin.
if ~isfinite(x)
    y=x;
    return
end
dx=eps(x);
if dx==0
    dx=realmin;
end
y=x+dx;
if y<=x
    dx=eps(max(1,abs(x)));
    y=x+dx;
end
end

function aud_log(varargin)
% Silent in Step3 backend audit.
end

function M=bez13_geo_build_model(G,C)
n=13;
nseg=3;
ncp=n+1;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
h=segLen/sum(segLen);
h=h(:).';

p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

% Minimal variables:
% knot 2 jet orders 0..6 xyz = 21
% knot 3 jet orders 0..6 xyz = 21
% xi = 1
nvar=43;
iXi=43;

% Structural numerical variable scaling.
% For an internal knot derivative of order r, use the geometric mean of
% adjacent segment lengths. The scale makes a unit z-variable induce O(L)
% Bezier-control-point motion through h^r/fallingFactorial(n,r).
L=max(1,G.path_length);
jetScale=zeros(2,7);
for kk=1:2
    href=sqrt(h(kk)*h(kk+1));
    for r=0:6
        ff=factorial(n)/factorial(n-r);
        jetScale(kk,r+1)=L*ff/(href^r);
    end
end
xiScale=L;

% Fixed finite-difference endpoint interpolation matrix:
% maps 14 CPs -> [left normalized differences 0..6;
%                  right normalized differences 0..6].
F=bez13_jet_endpoint_difference_matrix(n);
if rank(F)~=14
    error('MINCOJET:Interpolation','Endpoint-difference matrix is singular.');
end

% Affine CP map: cpVec = c0 + Gcp*z
% cpVec ordering is x/y/z contiguous per segment CP via bez13_jet_cpidx.
ncpall=3*nseg*ncp;
Gcp=zeros(ncpall,nvar);
c0=zeros(ncpall,1);

for q=1:nseg
    for ax=1:3
        D0=zeros(14,1);
        Dz=zeros(14,nvar);

        % Left knot q, right knot q+1.
        [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,1,q,h(q),ax,p0,e, ...
            jetScale,xiScale,n,iXi);
        [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,8,q+1,h(q),ax,p0,e, ...
            jetScale,xiScale,n,iXi);

        % CP = F \ normalized endpoint differences.
        cp0=F\D0;
        cpG=F\Dz;

        ids=arrayfun(@(j) bez13_jet_cpidx(q,j,ax,ncp),1:ncp);
        c0(ids)=cp0;
        Gcp(ids,:)=cpG;
    end
end

% Hard corridor inequalities on actual CPs.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('segment',{},'cp',{},'plane',{});

for q=1:nseg
    CC=C.cells{q};

    for j=1:ncp
        for pl=1:size(CC.A,1)
            a=zeros(1,ncpall);
            for ax=1:3
                a(bez13_jet_cpidx(q,j,ax,ncp))=-CC.A(pl,ax);
            end

            Aineq(end+1,:)=a*Gcp; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(pl)-a*c0; %#ok<AGROW>

            safeMeta(end+1)=struct( ...
                'segment',q,'cp',j,'plane',pl); %#ok<AGROW>
        end
    end
end

% -------------------------------------------------------------------------
% Geometric guide-proximity objective in minimal coordinates.
%
% For each Bezier CP, define the target point by uniform local parameter
% interpolation along the corresponding Step1 guide segment.
%
%   Jgeo = sum ||Pcp - Pguide||^2
%
% Since Pcp = c0 + Gcp*z:
%
%   Jgeo = 0.5*z'*H*z + f'*z + const
%   H = 2*Gcp'*Gcp
%   f = 2*Gcp'*(c0-target)
%
% No arbitrary physical or tuning weight is introduced.
% -------------------------------------------------------------------------
target=zeros(ncpall,1);

for q=1:nseg
    for j=1:ncp
        tau=(j-1)/(ncp-1);
        pt=(1-tau)*W(:,q)+tau*W(:,q+1);

        for ax=1:3
            target(bez13_jet_cpidx(q,j,ax,ncp))=pt(ax);
        end
    end
end

Hphys=2*(Gcp.'*Gcp);
Hphys=0.5*(Hphys+Hphys.');
fphys=2*Gcp.'*(c0-target);

% Positive scalar scaling only; minimizer is unchanged.
d=max(abs(diag(Hphys)));
if ~(d>0), d=1; end
Hscaled=sparse(Hphys/d);
fscaled=fphys/d;

M=struct();
M.degree=n;
M.nseg=nseg;
M.ncp=ncp;
M.nvar=nvar;
M.iXi=iXi;
M.h=h;
M.jetScale=jetScale;
M.xiScale=xiScale;
M.Gcp=Gcp;
M.c0=c0;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hphysical=sparse(Hphys);
M.fphysical=fphys;
M.Hscaled=Hscaled;
M.fscaled=fscaled;
end

