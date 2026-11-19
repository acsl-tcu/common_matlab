function result = generate_CBF_REAL_HOCBF_ZOH()
% GENERATE_CBF_REAL_HOCBF_ZOH
% ONE COMMAND: generate three .m files, audit all four real inputs, smoke-test.
% The required jet and inequality source code is embedded BELOW as local
% functions; no CBF_REAL_ZOH_*_SOURCE.m companion files are necessary.
%
%   generate_CBF_REAL_HOCBF_ZOH
%
% Depends ONLY on sibling ../.. /COMMON/CBF_COMMON_MODEL.m and
% CBF_COMMON_GEOMETRY.m. Generator adds that folder temporarily.
% All verification appears in the Command Window (fprintf(2) for warnings).
% No .txt, .mat, CSV, or independent validation script is generated.
% Actual output: GENERATED/CBF_REAL_ZOH_DIAG_EVAL.m,
% GENERATED/CBF_REAL_ZOH_HOCBF_INEQUALITY.m and
% GENERATED/CBF_REAL_ZOH_JOINT_FILTER.m.
%
% IMPORTANT: These are four-input constant-input residuals, not a certified
% real-input HOCBF safety controller. Thrust acts already at order 2,
% torque generally at order 4; joint residuals may contain T*tau.
% HLC is NOT modified.
root=fileparts(mfilename('fullpath'));
project=fileparts(fileparts(root));
common=fullfile(project,'COMMON');
for item={'CBF_COMMON_MODEL.m','CBF_COMMON_GEOMETRY.m'}
    needed=fullfile(common,item{1});
    if exist(needed,'file')~=2
        error('CBFRealZOH:MissingCommon', ...
            'Required COMMON file is missing: %s',needed);
    end
end
if ~any(strcmp(strsplit(path,pathsep),common))
    addpath(common,'-begin');
    commonCleanup=onCleanup(@()rmpath(common)); %#ok<NASGU>
end

outdir=fullfile(root,'GENERATED');
if ~exist(outdir,'dir'), mkdir(outdir); end
% Copy source text from THIS generator, not missing external .m files.
selfFile=[mfilename('fullpath') '.m'];
jetSource=extract_embedded_source(selfFile,'JET');
jetSource=strrep(jetSource,'CBF_REAL_ZOH_JET_SOURCE', ...
    'CBF_REAL_ZOH_DIAG_EVAL');
diagnosticFile=fullfile(outdir,'CBF_REAL_ZOH_DIAG_EVAL.m');
write_file(diagnosticFile,jetSource);

hocbfSource=extract_embedded_source(selfFile,'HOCBF');
hocbfSource=strrep(hocbfSource,'CBF_REAL_ZOH_HOCBF_SOURCE', ...
    'CBF_REAL_ZOH_HOCBF_INEQUALITY');
hocbfFile=fullfile(outdir,'CBF_REAL_ZOH_HOCBF_INEQUALITY.m');
write_file(hocbfFile,hocbfSource);
jointSource=extract_embedded_source(selfFile,'JOINT');
jointSource=strrep(jointSource,'CBF_REAL_ZOH_JOINT_SOURCE', ...
    'CBF_REAL_ZOH_JOINT_FILTER');
jointFile=fullfile(outdir,'CBF_REAL_ZOH_JOINT_FILTER.m');
write_file(jointFile,jointSource);
fprintf('[CBF GENERATED] %s\n',diagnosticFile);
fprintf('[CBF GENERATED] %s\n',hocbfFile);
fprintf('[CBF GENERATED] %s\n',jointFile);

% Numerical and symbolic diagnostics use the inline source, same math as
% the files just generated. All checks run without additional commands.
P=struct('m',0.762,'mL',0.01,'cableL',2.0,'gravity',9.81, ...
    'J',[0.02237568;0.02985236;0.0480374],'dstx',0,'dsty',0);
G=CBF_COMMON_GEOMETRY(P);
% Test fixtures are numerical probes, NOT operating-point assumptions.
parts=struct('kind','ellipsoid','center',[1.25,0.70,1.45], ...
    'R',rotation_z(0.35), ...
    'params',struct('axes',[0.7 0.45 0.95]), ...
    'obstacleName','degree_probe_obstacle');

cases={ ...
    sample_state([0.25;-0.20;-0.947],[0.18;-0.11;0.07], ...
                 [0.4;-0.1;1.35]), ...
    sample_state([-0.37;0.22;-0.903],[-0.15;0.12;0.25], ...
                 [-0.3;0.5;1.75]), ...
    sample_state([0.12;0.38;-0.917],[0.26;0.17;-0.09], ...
                 [0.5;0.9;2.10])};
thrusts=[7.5 9.2 11.3];
step=0.25; % N*m; symmetric difference of exact jet coefficients
S=zeros(7,6,3,3);
ST=zeros(7,6,3); % thrust sensitivity (early physical input action)
scale=zeros(7,6,3);
modelError=zeros(1,3);
% Optional direct check against the ACTUAL 24-state physical plant RHS.
plantError=nan(1,numel(cases));
plantFound=(exist('with_load_model_euler_for_HL','file')==2);
analyticError=zeros(1,3);
analyticBeta=zeros(3,3);
for c=1:numel(cases)
    x=cases{c};u=[thrusts(c);0;0;0];
    base=CBF_REAL_ZOH_JET_SOURCE(x,u,parts,P,G,6);
    rhs=CBF_COMMON_MODEL(x,u,P);
    modelError(c)=max(abs(rhs-base.stateTaylor(:,2)));
    if plantFound
        plantError(c)=compare_to_active_24state_plant(x,u,P,rhs);
    end
    for j=1:numel(base.names)
        scale(:,j,c)=max(1,abs(base.derivatives(:,j)));
    end
    utp=u;utm=u;utp(1)=u(1)+step;utm(1)=u(1)-step;
    apt=CBF_REAL_ZOH_JET_SOURCE(x,utp,parts,P,G,6);
    amt=CBF_REAL_ZOH_JET_SOURCE(x,utm,parts,P,G,6);
    ST(:,:,c)=(apt.derivatives-amt.derivatives)/(2*step);
    for a=1:3
        up=u;um=u;up(a+1)=step;um(a+1)=-step;
        ap=CBF_REAL_ZOH_JET_SOURCE(x,up,parts,P,G,6);
        am=CBF_REAL_ZOH_JET_SOURCE(x,um,parts,P,G,6);
        S(:,:,a,c)=(ap.derivatives-am.derivatives)/(2*step);
    end
    % Independent closed-form certificate of fourth-order torque action
    % on the load/ellipsoid obstacle barrier (row index 3).
    analyticBeta(:,c)=load_fourth_torque_beta(x,u,parts,P,G);
    analyticError(c)=max(abs(analyticBeta(:,c)-reshape(S(5,3,:,c),3,1)));
end
% Separate reference-output audit: the original hierarchy differentiates
% h2=pL_x-xd_x, h3=pL_y-xd_y six times AFTER the first-layer transform.
% Their order is not automatically the torque order of the geometric h.
referenceProbe=reference_position_probe(cases,P,G,parts,thrusts,step);
expected=[4 4 4 4 4 4]; % direct physical torque action; not 4-input relative degree
expectedThrust=[2 2 2 2 2 2];
labels=base.names;
observed=zeros(1,6);
firstMagnitude=zeros(1,6);
observedThrust=zeros(1,6);
for j=1:6
    for ord=1:6
        % S slices have 3 torque axes x 3 sample states, while scale
        % has one scalar per sample.  Do NOT flatten them to 9 vs 3.
        sig=reshape(S(ord+1,j,:,:),3,numel(cases));
        ref=reshape(scale(ord+1,j,:),1,numel(cases));
        threshold=repmat(1e-7.*ref,3,1);
        if any(any(abs(sig)>threshold))
            observed(j)=ord;
            firstMagnitude(j)=max(abs(sig(:)));
            break
        end
    end
    for ord=1:6
        sig=reshape(ST(ord+1,j,:),1,numel(cases));
        ref=reshape(scale(ord+1,j,:),1,numel(cases));
        if any(abs(sig)>1e-7.*ref)
            observedThrust(j)=ord;break
        end
    end
end

% The requested relative degrees and the directly observed torque
% sensitivities are deliberately SEPARATE diagnostics.
loadBetaNonzero=any(abs(analyticBeta(:))>1e-7);
symbolicStatus='SKIPPED_NO_SYMBOLIC_TOOLBOX';
if exist('sym','file')==2
    try
        proof=check_beta4_symbolic_inline();
        if proof.hasGenericOrder4Action
            symbolicStatus='GENERIC_BETA4_NONZERO';
            loadBetaNonzero=true;
        else
            symbolicStatus='INCONCLUSIVE';
        end
    catch ME
        symbolicStatus=['FAILED: ' ME.message];
        fprintf(2,'[CBF SYMBOLIC WARNING] %s\n',ME.message);
    end
end

expectedThrust=[2 2 2 2 2 2];
modelMax=max(modelError);
analyticMax=max(analyticError);
modelOK=modelMax<1e-9;
betaOK=analyticMax<1e-4;
degreeOK=isequal(observed,expected);
thrustOK=isequal(observedThrust,expectedThrust);
% Verify BOTH halves of the first-appearance claim, for EACH safety
% function: no torque in derivative orders 0..3, nonzero at 4; no thrust
% in orders 0..1, nonzero at 2. These are multi-state numerical probes,
% not a global symbolic proof; scale normalized to catch false zero values.
torqueLower=zeros(1,6); torqueAt4=zeros(1,6);
thrustLower=zeros(1,6); thrustAt2=zeros(1,6);
for j=1:6
    for c=1:numel(cases)
        for k=0:3
            denom=scale(k+1,j,c);
            a=reshape(S(k+1,j,:,c),[],1);
            torqueLower(j)=max(torqueLower(j),max(abs(a))/denom);
        end
        denom=scale(5,j,c);
        a=reshape(S(5,j,:,c),[],1);
        torqueAt4(j)=max(torqueAt4(j),max(abs(a))/denom);
        for k=0:1
            thrustLower(j)=max(thrustLower(j),abs(ST(k+1,j,c))/scale(k+1,j,c));
        end
        thrustAt2(j)=max(thrustAt2(j),abs(ST(3,j,c))/scale(3,j,c));
    end
end
appearanceOK=all(torqueLower<1e-7) && all(torqueAt4>1e-7) && ...
             all(thrustLower<1e-7) && all(thrustAt2>1e-7);
if plantFound
    plantOK=all(plantError<1e-9);
else
    plantOK=true; % missing optional plant comparison is reported separately
end

% Joint T/tau dependence: evaluate the exact ZOH jet at four corners.
% This is a central-difference *audit* of exact Taylor coefficients, not a
% symbolic derivative and not a solver-ready convex approximation.
stepT=0.25; stepTau=step;
mixed=zeros(3,numel(cases));
for c=1:numel(cases)
    x=cases{c};u0=[thrusts(c);0;0;0];
    for a=1:3
        v=zeros(1,4);
        for st=[-1,1]
            for sw=[-1,1]
                uc=u0;uc(1)=uc(1)+st*stepT;
                uc(a+1)=uc(a+1)+sw*stepTau;
                z=CBF_REAL_ZOH_JET_SOURCE(x,uc,parts,P,G,4);
                v((st+1)/2*2+(sw+1)/2+1)=z.derivatives(5,3);
            end
        end
        % v order: --, -+, +-, ++
        mixed(a,c)=(v(4)-v(3)-v(2)+v(1))/(4*stepT*stepTau);
    end
end
mixedExpected=analyticBeta./reshape(thrusts,1,[]);
mixedError=max(abs(mixed(:)-mixedExpected(:)));
mixedNonzero=any(abs(mixed(:))>1e-7);
mixedOK=mixedError<1e-4 && mixedNonzero;
fprintf('\n========== CBF REAL / ZOH GENERATION CHECK ==========\n');
fprintf('Model: x=[q4,omega3,pL3,vL3,pT3,omegaCable3], u=[T,tauX,tauY,tauZ]\n');
fprintf('Assumption: real inputs held constant in local Taylor expansion.\n');
fprintf('Generic torque first appearance (drone,cable,load): [4 4 4]\n');
fprintf('Thrust generally appears at order 2 for all six barriers.\n');
fprintf('Structural basis: h depends on pL and pT (not attitude directly); h_dot is input-free,\n');
fprintf('  h_ddot depends on T; h^(3) has body rate but no torque; h^(4) uses body angular acceleration (tau).\n');
fprintf('Probe parameters: DIATONE m=%.3f, J=[%.6g %.6g %.6g], mL=%.3f (controller initial); true plant mL=0.14.\n', ...
    P.m,P.J,P.mL);
fprintf('\n%-28s %10s %10s %12s\n','Constraint','Expected','Observed','First beta');
for j=1:numel(labels)
    if observed(j)==expected(j)
        fprintf('  %-26s %10d %10d %12.3g\n', ...
            labels{j},expected(j),observed(j),firstMagnitude(j));
    else
        fprintf(2,'  %-26s %10d %10d %12.3g  <== DEGREE WARNING\n', ...
            labels{j},expected(j),observed(j),firstMagnitude(j));
    end
end
fprintf('Torques first appearance: [%s] (expected [%s])\n',num2str(observed),num2str(expected));
fprintf('Thrust first appearance: [%s] (expected [%s])\n', ...
    num2str(observedThrust),num2str(expectedThrust));
fprintf('Taylor-jet finite-perturbation input sensitivity (normalized, 3 states):\n');
fprintf('  %-26s %11s %11s %11s %11s\n', ...
    'Barrier','T_0to1','T_2','tau_0to3','tau_4');
for j=1:6
    lowOK=torqueLower(j)<1e-7 && thrustLower(j)<1e-7;
    firstOK=torqueAt4(j)>1e-7 && thrustAt2(j)>1e-7;
    if lowOK && firstOK
        fprintf('  %-26s %11.3g %11.3g %11.3g %11.3g PASS\n', ...
            labels{j},thrustLower(j),thrustAt2(j),torqueLower(j),torqueAt4(j));
    else
        fprintf(2,'  %-26s %11.3g %11.3g %11.3g %11.3g FAIL\n', ...
            labels{j},thrustLower(j),thrustAt2(j),torqueLower(j),torqueAt4(j));
    end
end
fprintf('Model/jet agreement max: %.4g\n',modelMax);
if plantFound
    fprintf('Actual with_load_model_euler_for_HL (24-state) RHS agreement: %.4g\n', ...
        max(plantError));
else
    fprintf(2,'[CBF PLANT AUDIT NOT RUN] with_load_model_euler_for_HL.m not on MATLAB path.\n');
end
fprintf('Load beta4 analytic-vs-jet max error: %.4g\n',analyticMax);
fprintf('Load beta4 symbolic status: %s\n',symbolicStatus);
fprintf('Joint T*tau load beta4 cross-sensitivity check max error: %.3e\n',mixedError);
if exist('fmincon','file')==2 || exist('fmincon','file')==6
    fprintf('[CBF NLP] fmincon available; infeasible-nominal cases can be optimized.\n');
else
    fprintf(2,'[CBF NLP WARNING] fmincon unavailable: only nominal-feasible checks will run.\n');
end
if ~mixedOK
    fprintf(2,'[CBF MIXED-INPUT WARNING] Unexpected T*tau sensitivity or analytic mismatch.\n');
end
fprintf('Reference pL(x,y,z) direct physical torque order: [%s]\n', ...
    num2str(referenceProbe.generic));
fprintf('Reference pL hover probe order: [%s]\n', ...
    num2str(referenceProbe.hover));
fprintf('NOTE: HLC sixth-order horizontal-load design is after first-layer thrust feedback;\n');
fprintf('      direct physical T,tau safety-output sensitivity is a different system.\n');
if ~appearanceOK
    fprintf(2,'[CBF ORDER FAILURE] Lower-order effect or required first action mismatch!\n');
end
if ~plantOK
    fprintf(2,'[CBF PLANT MISMATCH] Generated model differs from actual 24-state physical plant!\n');
end
if ~modelOK
    fprintf(2,'[CBF MODEL ERROR] Model and Taylor RHS disagree!\n');
end
if ~betaOK
    fprintf(2,'[CBF BETA ERROR] Analytic beta4 and jet disagree!\n');
end
if ~thrustOK
    fprintf(2,'[CBF THRUST WARNING] Unexpected thrust input appearance orders.\n');
end
if ~degreeOK
    fprintf(2,'[CBF DEGREE WARNING] Expected generic torque order [4 4 4] differs from probe.\n');
end
if ~loadBetaNonzero
    fprintf(2,'[CBF DEGENERACY WARNING] Load beta4 was not identified as generically nonzero.\n');
end

% Automatic generated-file smoke test. This does not certify safety.
smoke=run_generated_smoke(outdir);
fprintf('[CBF SMOKE PASS] obstacle+ground %d, ground-only %d, model error %.3e, coupled filter: %s\n', ...
    smoke.numObstacleAndGroundBarriers,smoke.numGroundOnlyBarriers, ...
    smoke.modelRhsDifference,smoke.jointStatus);

if modelOK && plantOK && appearanceOK && betaOK && degreeOK && thrustOK && mixedOK && plantFound
    status='GENERATED_FOUR_INPUT_STRUCTURE_AND_PLANT_CHECKED_NOT_CERTIFIED';
elseif modelOK && plantOK && appearanceOK && betaOK && degreeOK && thrustOK && mixedOK
    status='GENERATED_FOUR_INPUT_STRUCTURE_CHECKED_PLANT_UNAVAILABLE_NOT_CERTIFIED';
else
    status='GENERATED_WITH_INPUT_STRUCTURE_WARNINGS_NOT_CERTIFIED';
end
fprintf(2,'[CBF SAFETY NOTICE] The r=4 residual is NOT a classical HOCBF for all four real inputs.\n');
fprintf(2,'[CBF SAFETY NOTICE] Experimental coupled nonlinear filter does NOT guarantee continuous or sampled-data safety.\n');
fprintf(2,'[CBF SAFETY NOTICE] T enters at r=2 and T*tau enters higher derivatives: NOT a linear QP; sampled-data safety unproven.\n');
fprintf('[CBF STATUS] %s\n',status);
fprintf('=====================================================\n');
result=struct('status',status,'diagnosticFile',diagnosticFile, ...
    'inequalityFile',hocbfFile,'jointFilterFile',jointFile, ...
    'observed',observed,'expected',expected, ...
    'observedThrust',observedThrust,'expectedThrust',expectedThrust, ...
    'modelError',modelError,'loadBeta4',analyticBeta, ...
    'analyticBetaError',analyticError,'referenceProbe',referenceProbe, ...
    'symbolicStatus',symbolicStatus,'mixedInputError',mixedError, ...
    'mixedInputNonzero',mixedNonzero,'appearanceOK',appearanceOK, ...
    'torqueLower',torqueLower,'torqueAt4',torqueAt4, ...
    'thrustLower',thrustLower,'thrustAt2',thrustAt2, ...
    'actualPlantComparisonAvailable',plantFound,'actualPlantError',plantError, ...
    'inputSensitivity',struct('thrust',ST,'torque',S), ...
    'smoke',smoke,'classicalHOCBFCertified',false);
end

function err=compare_to_active_24state_plant(x,u,P,rhs)
% Independent numeric RHS check against the actual plant selected by
% Model_Suspended_Load -> get_model_name('Load_HL'). Only physically shared
% state derivatives are compared (not quaternion vs Euler derivatives).
q=x(1:4);
q0=q(1);q1=q(2);q2=q(3);q3=q(4);
roll=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
pitch=asin(max(-1,min(1,2*(q0*q2-q3*q1))));
yaw=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
pL=x(8:10);vL=x(11:13);pT=x(14:16);wL=x(17:19);
pDrone=pL-P.cableL*pT;
vDrone=vL-P.cableL*cross(wL,pT);
xPlant=[pDrone;roll;pitch;yaw;vDrone;x(5:7);pL;vL;pT;wL];
pp=zeros(1,21);
pp(1)=P.m;pp(6:8)=P.J(:).';pp(9)=P.gravity;
pp(20)=P.mL;pp(21)=P.cableL;
dPlant=with_load_model_euler_for_HL(xPlant,u(:),pp);
physical=[rhs(5:7);rhs(8:10);rhs(11:13);rhs(14:16);rhs(17:19)];
physicalPlant=[dPlant(10:12);dPlant(13:15);dPlant(16:18); ...
               dPlant(19:21);dPlant(22:24)];
err=max(abs(physical-physicalPlant));
end

function audit=reference_position_probe(cases,P,G,parts,thrusts,step)
% Probe direct physical Cartesian load positions for comparison with HLC
% reference h2/h3, which uses a transformed input system f1+g1*v.
N=6; generic=zeros(1,3); hover=zeros(1,3);
for mode=1:2
    if mode==1
        probes=cases;Tvals=thrusts;
    else
        q=[1;0;0;0]; cable=[0;0;-1];
        x=[q;zeros(3,1);[0;0;2];zeros(3,1);cable;zeros(3,1)];
        probes={x};Tvals=(P.m+P.mL)*P.gravity;
    end
    signal=zeros(N+1,3,3,numel(probes));
    magnitude=zeros(N+1,3,numel(probes));
    for c=1:numel(probes)
        x=probes{c};u=[Tvals(c);0;0;0];
        base=CBF_REAL_ZOH_JET_SOURCE(x,u,parts,P,G,N);
        for ord=0:N
            magnitude(ord+1,:,c)=max(1,abs(factorial(ord)*base.stateTaylor(8:10,ord+1).'));
        end
        for a=1:3
            up=u;um=u;up(a+1)=step;um(a+1)=-step;
            Xp=CBF_REAL_ZOH_JET_SOURCE(x,up,parts,P,G,N);
            Xm=CBF_REAL_ZOH_JET_SOURCE(x,um,parts,P,G,N);
            for ord=0:N
                signal(ord+1,:,a,c)=factorial(ord)* ...
                    (Xp.stateTaylor(8:10,ord+1)-Xm.stateTaylor(8:10,ord+1)).'/(2*step);
            end
        end
    end
    observed=zeros(1,3);
    for axis=1:3
        for ord=1:N
            sig=reshape(signal(ord+1,axis,:,:),3,numel(probes));
            sc=reshape(magnitude(ord+1,axis,:),1,numel(probes));
            if any(any(abs(sig)>repmat(1e-7.*sc,3,1)))
                observed(axis)=ord;break
            end
        end
    end
    if mode==1,generic=observed;else,hover=observed;end
end
audit=struct('generic',generic,'hover',hover, ...
    'meaning','Direct torque effects on physical Cartesian load coordinates; not HLC transformed output');
end

function R=rotation_z(a)
R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
end

function x=sample_state(cable,omega,loadpos)
q=[0.965;0.115;0.19;-0.13];q=q/norm(q);
pT=cable/norm(cable);
x=[q;omega;loadpos;0.4;-0.25;0.1;pT;0.15;-0.11;0.2];
end

function beta=load_fourth_torque_beta(x,u,part,P,G)
q=x(1:4);pT=x(14:16);pL=x(8:10);
R=quat_rotation(q);
axes=part.params.axes(:);
Qo=part.R*diag(axes.^2)*part.R.';
M=((1+G.obstacleOuterEta)*G.loadSafetyRadius^2*eye(3) ...
    +(1+1/G.obstacleOuterEta)*Qo)\eye(3);
D=[-R(:,2)/P.J(1),R(:,1)/P.J(2),zeros(3,1)];
beta=(2*u(1)/(P.m+P.mL)) ...
    *(pT.'*M*(pL-part.center(:)))*(D.'*pT);
end

function R=quat_rotation(q)
a=q(1);b=q(2);c=q(3);d=q(4);
R=[1-2*(c*c+d*d),2*(b*c-a*d),2*(b*d+a*c);...
   2*(b*c+a*d),1-2*(b*b+d*d),2*(c*d-a*b);...
   2*(b*d-a*c),2*(c*d+a*b),1-2*(b*b+c*c)];
end

function write_file(path,contents)
fid=fopen(path,'w');
if fid<0,error('CBFRealZOH:Write','Cannot write %s',path);end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,contents,'char');
end

function info=check_beta4_symbolic_inline()
% Symbolic Math Toolbox optional. No text report is written.
syms q0 q1 q2 q3 t1 t2 t3 p1 p2 p3 c1 c2 c3 real
syms M11 M12 M13 M22 M23 M33 T m mL Jx Jy Jz real
q=[q0;q1;q2;q3]; %#ok<NASGU>
pT=[t1;t2;t3];
pL=[p1;p2;p3]; c=[c1;c2;c3];
M=[M11 M12 M13;M12 M22 M23;M13 M23 M33];
R=[1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2); ...
   2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1); ...
   2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1*q1+q2*q2)];
D=[-R(:,2)/Jx R(:,1)/Jy sym(zeros(3,1))];
beta4=(2*T/(m+mL))*(pT.'*M*(pL-c))*(D.'*pT);
info=struct();
info.beta4=beta4;
info.isIdenticallyZero=[isequal(simplify(beta4(1)),sym(0)), ...
                        isequal(simplify(beta4(2)),sym(0)), ...
                        isequal(simplify(beta4(3)),sym(0))];
info.hasGenericOrder4Action=~all(info.isIdenticallyZero);
end

function smoke=run_generated_smoke(outdir)
% Verify generated .m functions from the current MATLAB run; no file report.
addpath(outdir,'-begin');
cleanup=onCleanup(@()rmpath(outdir)); %#ok<NASGU>
rehash;
P=struct('m',0.762,'mL',0.01,'cableL',2.0, 'gravity',9.81, ...
    'J',[0.02237568;0.02985236;0.0480374],'dstx',0,'dsty',0);
G=CBF_COMMON_GEOMETRY(P);
x=[1;0;0;0; zeros(3,1); 0;0;2; zeros(3,1); 0;0;-1; zeros(3,1)];
u=[(P.m+P.mL)*P.gravity;0;0;0];
part=struct('kind','ellipsoid','center',[2 0 2], ...
    'R',eye(3),'params',struct('axes',[0.5 0.5 0.8]), ...
    'obstacleName','smoke_test');
a=CBF_REAL_ZOH_HOCBF_INEQUALITY(x,u,part,P,G);
assert(numel(a.values)==6, ...
    'CBF generated smoke: expected 6 constraints with one obstacle');
assert(isequal(a.requestedOrder,[4 4 4 4 4 4]), ...
    'CBF generated smoke: expected order-4 torque diagnostics');
assert(all(isfinite(a.values)) && all(isfinite(a.derivatives(:))), ...
    'CBF generated smoke: nonfinite constraints');
assert(all(cellfun(@(v)all(isfinite(v)),a.stages)), ...
    'CBF generated smoke: nonfinite intermediate stages');
% Both thrust and torque must actually change ZOH residuals at generic states.
x2=sample_state([0.25;-0.20;-0.947],[0.18;-0.11;0.07],[0.4;-0.1;1.35]);
u2=[7.5;0;0;0];
a2=CBF_REAL_ZOH_HOCBF_INEQUALITY(x2,u2,part,P,G);
uT=u2;uT(1)=uT(1)+0.2;
uTau=u2;uTau(2)=uTau(2)+0.05;
aT=CBF_REAL_ZOH_HOCBF_INEQUALITY(x2,uT,part,P,G);
aTau=CBF_REAL_ZOH_HOCBF_INEQUALITY(x2,uTau,part,P,G);
assert(any(abs(aT.values-a2.values)>1e-9), ...
    'CBF smoke: thrust does not affect residuals');
assert(any(abs(aTau.values-a2.values)>1e-9), ...
    'CBF smoke: torque does not affect residuals');
b=CBF_REAL_ZOH_HOCBF_INEQUALITY(x,u,[],P,G);
assert(numel(b.values)==3, ...
    'CBF generated smoke: expected 3 ground constraints for empty sensor');
% Coupled filter test: a nominal input satisfying all constraints must be
% returned unchanged, without requiring the Optimization Toolbox.
joint=CBF_REAL_ZOH_JOINT_FILTER(x,u,part,P,G);
assert(joint.success && norm(joint.input-u)<1e-9, ...
    'CBF smoke: joint nominal-feasible path did not return nominal u');
assert(~isempty(joint.constraintValues) && all(isfinite(joint.constraintValues)), ...
    'CBF smoke: joint nonlinear constraints nonfinite');
% Empty obstacles AND disabled ground must yield bounded nominal.
G0=G;G0.includeGround=false;
joint0=CBF_REAL_ZOH_JOINT_FILTER(x,u,[],P,G0);
assert(joint0.success && norm(joint0.input-u)<1e-9, ...
    'CBF smoke: empty environment not supported');
jet=CBF_REAL_ZOH_DIAG_EVAL(x,u,part,P,G,6);
modelErr=max(abs(CBF_COMMON_MODEL(x,u,P)-jet.stateTaylor(:,2)));
assert(modelErr<1e-9, 'CBF generated smoke: model/jet mismatch');
smoke=struct('status','PASS_NOT_SAFETY_CERTIFICATE', ...
    'modelRhsDifference',modelErr, ...
    'numObstacleAndGroundBarriers',numel(a.values), ...
    'numGroundOnlyBarriers',numel(b.values), ...
    'jointStatus',joint.status);
end

function body=extract_embedded_source(selfFile,kind)
% Build the unique tag by concatenation: tags occur only at source blocks.
raw=fileread(selfFile);
beginTag=['%<<< ' 'CBF_' kind '_BEGIN' ' >>>'];
endTag=['%<<< ' 'CBF_' kind '_END' ' >>>'];
b=strfind(raw,beginTag);
e=strfind(raw,endTag);
if numel(b)~=1 || numel(e)~=1 || e<=b+numel(beginTag)
    error('CBFRealZOH:InlineSource', ...
        'Embedded %s source markers missing or duplicated in %s',kind,selfFile);
end
body=strtrim(raw(b+numel(beginTag):e-1));
if ~startsWith(body,'function ')
    error('CBFRealZOH:InlineSource','Embedded %s source malformed',kind);
end
end

%<<< CBF_JET_BEGIN >>>
function data = CBF_REAL_ZOH_JET_SOURCE(x, u, parts, P, G, order)
% CBF_REAL_ZOH_JET_SOURCE  Exact time-Taylor coefficients (to roundoff).
% Source algorithm used by generate_CBF_REAL_HOCBF_ZOH.m.
% NOT a validated safety filter. No numerical finite differentiation.
% This source is copied into GENERATED under a distinctive name after
% generator checks; method-specific ZOH assumption lives ONLY here.
%
% x (19x1) [q(4);w(3);pL(3);vL(3);pT(3);wL(3)], u(4x1)=[T;tau].
% All 4 physical inputs are held constant for the Taylor expansion.
% This is independent of the nominal output order in the HLC hierarchy.
% The generator checks lower-order input effects on each geometric barrier.
if nargin<6, order=6; end
validateattributes(x,{'double'},{'vector','numel',19,'finite','real'});
validateattributes(u,{'double'},{'vector','numel',4,'finite','real'});
validateattributes(order,{'double'},{'scalar','integer','>=',1,'<=',6});
validateattributes(P.m,{'double'},{'scalar','positive','finite'});
validateattributes(P.mL,{'double'},{'scalar','nonnegative','finite'});
validateattributes(P.cableL,{'double'},{'scalar','positive','finite'});
validateattributes(P.J,{'double'},{'vector','numel',3,'positive','finite'});
x=x(:);u=u(:);
if abs(norm(x(1:4))-1)>1e-5
    error('CBFRealZOH:Quaternion','Input quaternion must be unit length');
end
if abs(norm(x(14:16))-1)>1e-5
    error('CBFRealZOH:CableDirection','Cable unit direction pT must have norm 1');
end
% No detections is a normal sensor result and must be handled safely.
if isempty(parts), parts=struct([]); end
if isstruct(parts) && isscalar(parts) && isfield(parts,'parts')
    parts=parts.parts;
end
if ~isstruct(parts)
    error('CBFRealZOH:Obstacle','parts must be a struct array (may be empty)');
end
N=order;
X=zeros(19,N+1);
X(:,1)=x;
for k=0:N-1
    X(:,k+2)=physical_rhs_coefficient(X(:,1:k+1),u,P,k)/(k+1);
end
pL=X(8:10,:); pT=X(14:16,:);
pD=pL-P.cableL*pT;
pC=pL-(P.cableL/2)*pT;
a=G.cableAxisShort; b=G.cableAxisLong;
Qc=zeros(3,3,N+1);
for k=0:N
    Qc(:,:,k+1)=(b*b-a*a)*outer_coefficient(pT,pT,k);
end
Qc(:,:,1)=Qc(:,:,1)+a*a*eye(3);

nObs=numel(parts);
ng=double(logical(G.includeGround));
nb=3*nObs+3*ng;
D=zeros(N+1,nb);
names=cell(1,nb);
components=cell(1,nb);
kinds=cell(1,nb);
idx=0;
for j=1:nObs
    [ctr,Qo,label]=parse_obstacle(parts(j),j);
    pose={pD,pC,pL};
    labels={'drone','cable','load'};
    for k=1:3
        idx=idx+1;
        Qs=zeros(3,3,N+1);
        switch k
            case 1, Qs(:,:,1)=G.droneSafetyRadius^2*eye(3);
            case 2, Qs=Qc;
            case 3, Qs(:,:,1)=G.loadSafetyRadius^2*eye(3);
        end
        U= (1+G.obstacleOuterEta)*Qs;
        U(:,:,1)=U(:,:,1)+(1+1/G.obstacleOuterEta)*Qo;
        delta=pose{k};delta(:,1)=delta(:,1)-ctr;
        D(:,idx)=quad_separation_jet(delta,U,N);
        names{idx}=sprintf('%s/%s',label,labels{k});
        components{idx}=labels{k};
        kinds{idx}='obstacle';
    end
end
if G.includeGround
    idx=idx+1;
    D(:,idx)=deriv_from_coefficient(pD(3,:)-[G.groundZ+G.droneSafetyRadius zeros(1,N)]);
    names{idx}='ground/drone';components{idx}='drone';kinds{idx}='ground';
    idx=idx+1;
    zSeries=Qc(3,3,:);zSeries=reshape(zSeries,1,[]);
    radiusSeries=positive_sqrt_series(zSeries,N);
    D(:,idx)=deriv_from_coefficient(pC(3,:)-radiusSeries-[G.groundZ zeros(1,N)]);
    names{idx}='ground/cable';components{idx}='cable';kinds{idx}='ground';
    idx=idx+1;
    D(:,idx)=deriv_from_coefficient(pL(3,:)-[G.groundZ+G.loadSafetyRadius zeros(1,N)]);
    names{idx}='ground/load';components{idx}='load';kinds{idx}='ground';
end

data=struct('derivatives',D,'names',{names},'components',{components}, ...
    'kinds',{kinds},'stateTaylor',X,'order',N, ...
    'inputZOH',u,'model','generateModel_SupendedLoad physical x19', ...
    'warning','ZOH jets only: intersample and global HOCBF safety unverified');
end

function f=physical_rhs_coefficient(X,u,P,k)
q=X(1:4,:);w=X(5:7,:);vL=X(11:13,:);
pT=X(14:16,:);wL=X(17:19,:);
m=P.m;L=P.cableL;mL=P.mL;J=P.J(:);
T=u(1);
f=zeros(19,1);
% qdot = 1/2 q tensor omega
f(1)=-.5*(conv_at(q(2,:),w(1,:),k)+conv_at(q(3,:),w(2,:),k)+conv_at(q(4,:),w(3,:),k));
f(2)= .5*(conv_at(q(1,:),w(1,:),k)+conv_at(q(3,:),w(3,:),k)-conv_at(q(4,:),w(2,:),k));
f(3)= .5*(conv_at(q(1,:),w(2,:),k)+conv_at(q(4,:),w(1,:),k)-conv_at(q(2,:),w(3,:),k));
f(4)= .5*(conv_at(q(1,:),w(3,:),k)+conv_at(q(2,:),w(2,:),k)-conv_at(q(3,:),w(1,:),k));
Jw=J.*w;
f(5:7)=-cross_at(w,Jw,k)./J;
if k==0, f(5:7)=f(5:7)+u(2:4)./J; end
f(8:10)=vL(:,k+1);
b3=zeros(3,k+1);
for j=0:k,b3(:,j+1)=body_z_axis_coeff(q,j);end
vT=zeros(3,k+1);
for j=0:k,vT(:,j+1)=cross_at(wL,pT,j);end
% aL=-g e3 + (T*dot(pT,b3)-mL*dot(vT,vT))*pT/(m+mL)
s=zeros(1,k+1);
for j=0:k
    s(j+1)=T*dot_at(pT,b3,j)-m*L*dot_at(vT,vT,j);
end
for j=0:k
    f(11:13)=f(11:13)+s(j+1)*pT(:,k-j+1)/(m+mL);
end
if k==0
    f(13)=f(13)-P.gravity;
    if isfield(P,'dstx'),f(11)=f(11)+P.dstx;end
    if isfield(P,'dsty'),f(12)=f(12)+P.dsty;end
end
f(14:16)=vT(:,k+1);
f(17:19)=-(T/(m*L))*cross_at(pT,b3,k);
end

function b=body_z_axis_coeff(q,k)
b=zeros(3,1);
b(1)=2*(conv_at(q(2,:),q(4,:),k)+conv_at(q(1,:),q(3,:),k));
b(2)=2*(conv_at(q(3,:),q(4,:),k)-conv_at(q(1,:),q(2,:),k));
b(3)=conv_at(q(1,:),q(1,:),k)-conv_at(q(2,:),q(2,:),k)...
    -conv_at(q(3,:),q(3,:),k)+conv_at(q(4,:),q(4,:),k);
end

function v=conv_at(a,b,k)
v=0;
for j=0:k
    v=v+a(j+1)*b(k-j+1);
end
end

function v=dot_at(a,b,k)
v=0;
for j=0:k
    v=v+a(:,j+1).'*b(:,k-j+1);
end
end

function v=cross_at(a,b,k)
v=zeros(3,1);
for j=0:k
    v=v+cross(a(:,j+1),b(:,k-j+1));
end
end

function v=outer_coefficient(a,b,k)
v=zeros(3);
for j=0:k
    v=v+a(:,j+1)*b(:,k-j+1).';
end
end

function [center,Q,label]=parse_obstacle(part,index)
if ~isfield(part,'center')
    error('CBFRealZOH:ObstacleCenter','parts(%d) has no center',index);
end
center=part.center(:);
if ~isequal(size(center),[3,1]) || any(~isfinite(center))
    error('CBFRealZOH:ObstacleCenter','Obstacle center must be finite 3x1');
end
if isfield(part,'kind')
    kind=char(part.kind);
else
    kind='ellipsoid';
end
switch kind
    case 'ellipsoid'
        if isfield(part,'params')
            axes=part.params.axes(:);
        elseif isfield(part,'axes')
            axes=part.axes(:);
        else
            error('CBFRealZOH:ObstacleAxes','Ellipsoid axes missing');
        end
        R=part.R;
    case 'sphere'
        if isfield(part,'params'),r=part.params.radius;
        else,r=part.radius;end
        axes=repmat(r,3,1);R=eye(3);
    otherwise
        error('CBFRealZOH:ObstacleKind', ...
            'Expected single ellipsoid or sphere, got %s',kind);
end
if ~isequal(size(R),[3,3]) || any(~isfinite(R(:))) || ...
        norm(R.'*R-eye(3),'fro')>1e-5 || ...
        numel(axes)~=3 || any(~isfinite(axes)) || any(axes<=0)
    error('CBFRealZOH:BadObstacle','Invalid obstacle orientation or axes');
end
Q=R*diag(axes.^2)*R.';
if isfield(part,'obstacleName')
    label=char(part.obstacleName);
elseif isfield(part,'name')
    label=char(part.name);
else
    label=sprintf('obstacle%d',index);
end
end

function d=quad_separation_jet(delta,U,N)
% E_system - E_obstacle is contained in E_outer with
% Qouter=(1+eta)Qsystem + (1+1/eta)Qobstacle.
% h=delta'*Qouter^-1*delta-1 >= 0 => ellipsoids are separated.
% This outer approximation is conservative but shape-aware and smooth.
J=zeros(3,3,N+1);
Q0=U(:,:,1);
if rcond(Q0)<1e-12
    error('CBFRealZOH:OuterEllipsoid','Outer ellipsoid matrix is singular');
end
J(:,:,1)=Q0\eye(3);
for k=1:N
    S=zeros(3);
    for i=1:k
        S=S+U(:,:,i+1)*J(:,:,k-i+1);
    end
    J(:,:,k+1)=-(Q0\S);
end
y=zeros(3,N+1);
for k=0:N
    for i=0:k
        y(:,k+1)=y(:,k+1)+J(:,:,i+1)*delta(:,k-i+1);
    end
end
h=zeros(1,N+1);
for k=0:N
    h(k+1)=dot_at(delta,y,k);
end
h(1)=h(1)-1;
d=deriv_from_coefficient(h);
end

function s=positive_sqrt_series(z,N)
if z(1)<=0
    error('CBFRealZOH:CableGeometry','Cable ellipsoid cross-section is invalid');
end
s=zeros(1,N+1);s(1)=sqrt(z(1));
for k=1:N
    v=0;
    for j=1:k-1,v=v+s(j+1)*s(k-j+1);end
    s(k+1)=(z(k+1)-v)/(2*s(1));
end
end

function d=deriv_from_coefficient(z)
N=numel(z)-1;
d=zeros(N+1,1);
for j=0:N,d(j+1)=factorial(j)*z(j+1);end
end
%<<< CBF_JET_END >>>

%<<< CBF_HOCBF_BEGIN >>>
function out = CBF_REAL_ZOH_HOCBF_SOURCE(x, u, parts, P, G, options)
% CBF_REAL_ZOH_HOCBF_SOURCE  Experimental r=[4,4,4] ZOH residuals.
% GENERATED locally as CBF_REAL_ZOH_HOCBF_INEQUALITY.m.
% Inputs: x(19x1), real u=[T;tx;ty;tz], parts (single ellipsoid array
% or world struct with .parts), physical P, geometry G, options.
%
% h: conservative nonintersection function for drone sphere, cable
% ellipsoid and load sphere against obstacle ellipsoids, plus ground.
%
% gamma_i = (d/dt + lambda)^i h for i=0,...,r along the fixed-input
% time Taylor jet. Top-order residual >=0 is the candidate inequality.
% r=4 for drone/cable/load is the generic FIRST TORQUE ACTION ORDER.
% Thrust appears at order 2, so r=4 is NOT a classical full-input
% relative degree. This is a diagnostic, NOT a certified HOCBF.
%
% IMPORTANT: If torques act in lower derivatives, this is an experimental
% constant-input high-order surrogate, NOT a certified standard HOCBF.
% Also the constraints can be nonlinear in real inputs (including T*tau);
% do not insert into a linear-QP solver or claim intersample safety.
%
% options.lambda: scalar or [drone,cable,load], default [1 1 1] [1/s].
% Outputs: out.values (1xn), out.stages (1xn cells), out.currentSafe,
% out.lowerStagesSatisfied, out.requestedOrder, out.names, out.derivatives.

if nargin<6 || isempty(options), options=struct(); end
if nargin<5 || isempty(G), G=CBF_COMMON_GEOMETRY(P); end
if isempty(parts), parts=struct([]); end
if isstruct(parts) && isscalar(parts) && isfield(parts,'parts')
    parts=parts.parts;
end
if ~isfield(options,'lambda'), options.lambda=[1 1 1]; end
lambda=options.lambda(:).';
if numel(lambda)==1, lambda=repmat(lambda,1,3); end
validateattributes(lambda,{'numeric'},{'vector','numel',3,'real','finite','positive'});
if any(~isfinite(lambda)), error('CBFRealZOH:Lambda','Invalid lambda'); end
J=CBF_REAL_ZOH_DIAG_EVAL(x,u,parts,P,G,4);
n=numel(J.names);
values=zeros(1,n);
ranks=zeros(1,n);
gains=zeros(1,n);
stages=cell(1,n);
lowerOK=true(1,n);
for j=1:n
    switch J.components{j}
        case 'drone', r=4; lam=lambda(1);
        case 'cable', r=4; lam=lambda(2);
        case 'load', r=4; lam=lambda(3);
        otherwise, error('CBFRealZOH:Component','Unknown component');
    end
    ranks(j)=r;
    gains(j)=lam;
    stage=zeros(r+1,1);
    for i=0:r
        v=0;
        for k=0:i
            v=v+nchoosek(i,k)*lam^(i-k)*J.derivatives(k+1,j);
        end
        stage(i+1)=v;
    end
    stages{j}=stage;
    values(j)=stage(end);
    lowerOK(j)=all(stage(1:end-1)>=0);
end
out=struct('values',values,'names',{J.names},'components',{J.components}, ...
    'kinds',{J.kinds},'requestedOrder',ranks,'gains',gains, ...
    'stages',{stages},'derivatives',J.derivatives, ...
    'h',J.derivatives(1,:), 'currentSafe',J.derivatives(1,:)>=0, ...
    'lowerStagesSatisfied',lowerOK, 'topStageSatisfied',values>=0, ...
    'input',u(:),'state',x(:), ...
    'status','EXPERIMENTAL_ZOH_FOUR_INPUT_NONLINEAR_NOT_SAFETY_CERTIFIED');
end
%<<< CBF_HOCBF_END >>>

%<<< CBF_JOINT_BEGIN >>>
function result = CBF_REAL_ZOH_JOINT_SOURCE(x, uNom, parts, P, G, options)
% CBF_REAL_ZOH_JOINT_FILTER  Experimental coupled T,torque nonlinear filter.
% GENERATED by generate_CBF_REAL_HOCBF_ZOH (no separate source needed).
%
% Usage:
%   out=CBF_REAL_ZOH_JOINT_FILTER(x19,uNom4,detectedParts,P,G);
%   if out.success, uToPlant=out.input; else, handle failure explicitly; end
%
% Optimizes all 4 physical commands together [T;tauX;tauY;tauZ].
% Uses nonlinear SQP (fmincon), NOT a QP. The coupled fourth-order
% derivative contains T*tau, and thrust already occurs at order 2.
% Constraints used here are (A) the r=4 experimental high-order residual,
% and (B) h >= 0 at samples in the next 25 ms ZOH interval, evaluated
% from the fourth-order time Taylor polynomial. These tests are NOT a
% formal sampled-data forward-invariance proof: missing fifth+ terms,
% obstacle motion between poses and future input viability remain open.
%
% Input G and parts match CBF_REAL_ZOH_HOCBF_INEQUALITY.
% 'parts' is the CURRENTLY OBSERVED world, not future ground truth.
% Ground is handled independently by G.includeGround.
%
% options fields (all optional):
%   dt=0.025, sampleFractions=[0 .25 .5 .75 1],
%   lambda=[1 1 1], enforceLowerStages=false,
%   inputLower=[0;-1;-1;-1], inputUpper=[20;1;1;1],
%   inputScale=[10;1;1;1], inputWeights=[1;1;1;1],
%   constraintTolerance=1e-7, maxIterations=65,
%   maxFunctionEvaluations=400, useRK4Check=true, rk4Steps=12,
%   allowUnsafeStart=false.
%
% IMPORTANT: result.input is EMPTY on failure. Never send unchecked input
% directly to the plant. result.candidate and nominal are for diagnostics.
if nargin<6 || isempty(options), options=struct(); end
if nargin<5 || isempty(G), G=CBF_COMMON_GEOMETRY(P); end
validateattributes(x,{'double'},{'vector','numel',19,'real','finite'});
validateattributes(uNom,{'double'},{'vector','numel',4,'real','finite'});
x=x(:); uNom=uNom(:);
opts=cbf_joint_defaults(options);
lb=opts.inputLower(:); ub=opts.inputUpper(:);
scale=opts.inputScale(:); weights=opts.inputWeights(:);
if any(lb>=ub), error('CBFJoint:Bounds','Each input lower bound must be < upper bound');end
if any(scale<=0) || any(weights<=0)
    error('CBFJoint:Weight','Scale and weights must be positive');
end
if opts.dt<=0 || opts.dt>1 || ~isfinite(opts.dt)
    error('CBFJoint:Dt','Expected 0 < dt <= 1 second');
end
if any(opts.sampleFractions<0) || any(opts.sampleFractions>1) || ...
        any(~isfinite(opts.sampleFractions))
    error('CBFJoint:Samples','sampleFractions must lie in [0,1]');
end
% Also inspect the endpoints irrespective of requested samples.
times=unique([0 opts.sampleFractions(:).' 1])*opts.dt;
% Current geometry does not depend on any current control command.
initialJet=CBF_REAL_ZOH_DIAG_EVAL(x,[0;0;0;0],parts,P,G,1);
result=struct('success',false,'status','UNINITIALIZED','input',[], ...
    'candidate',[],'nominal',uNom,'fval',NaN,'exitflag',NaN, ...
    'maxViolation',Inf,'constraintNames',{{}},'constraintValues',[], ...
    'names',{initialJet.names},'nominalFeasible',false, ...
    'solved',false,'rk4MinimumH',NaN,'solverOutput',struct(), ...
    'warning','Experimental nonlinear ZOH surrogate, NOT certified CBF');
if isempty(initialJet.names)
    candidate=min(max(uNom,lb),ub);
    result.candidate=candidate;
    result.input=candidate;
    result.success=true;
    result.status='NO_ACTIVE_CONSTRAINTS_BOUNDS_ONLY';
    result.nominalFeasible=all(uNom>=lb)&all(uNom<=ub);
    result.maxViolation=0;
    return
end
if ~opts.allowUnsafeStart && any(initialJet.derivatives(1,:) < -opts.constraintTolerance)
    result.status='CURRENTLY_UNSAFE_REQUIRES_EXPLICIT_RECOVERY_POLICY';
    result.candidate=min(max(uNom,lb),ub);
    result.maxViolation=max(0,-min(initialJet.derivatives(1,:)));
    return
end
[u0,nominalInBounds]=cbf_joint_clip_nominal(uNom,lb,ub);
[g0,names0]=cbf_joint_eval(u0,x,parts,P,G,opts,times);
result.constraintNames=names0;
result.constraintValues=g0;
% Nominal must be IN bounds as well as satisfying nonlinear constraints.
if nominalInBounds && all(g0>=-opts.constraintTolerance)
    candidate=uNom;
    flag=1;
    output=struct('iterations',0,'funcCount',1,'message','Nominal feasible; no NLP required');
    fval=0;
    result.nominalFeasible=true;
    result.status='NOMINAL_FEASIBLE';
else
    if exist('fmincon','file')~=2 && exist('fmincon','file')~=6
        result.status='MISSING_OPTIMIZATION_TOOLBOX_FMINCON';
        result.candidate=u0;
        return
    end
    solverOptions=optimoptions('fmincon', ...
        'Algorithm','sqp','Display','off', ...
        'MaxIterations',opts.maxIterations, ...
        'MaxFunctionEvaluations',opts.maxFunctionEvaluations, ...
        'ConstraintTolerance',opts.constraintTolerance, ...
        'OptimalityTolerance',1e-8);
    objective=@(u)sum(weights.*((u(:)-uNom)./scale).^2);
    constraints=@(u)cbf_joint_nlcon(u,x,parts,P,G,opts,times);
    try
        [candidate,fval,flag,output]=fmincon(objective,u0,[],[],[],[], ...
            lb,ub,constraints,solverOptions);
    catch ME
        result.status='SOLVER_EXCEPTION';
        result.solverOutput=struct('message',ME.message,'identifier',ME.identifier);
        result.candidate=u0;
        return
    end
    result.solved=true;
    result.status='OPTIMIZATION_FINISHED_CHECKING';
end
result.candidate=candidate(:);
result.fval=fval;
result.exitflag=flag;
result.solverOutput=output;
if any(~isfinite(candidate)) || numel(candidate)~=4
    result.status='NONFINITE_CANDIDATE';return
end
[g,names]=cbf_joint_eval(candidate,x,parts,P,G,opts,times);
result.constraintValues=g;
result.constraintNames=names;
result.maxViolation=max([0; -g(:); lb-candidate(:); candidate(:)-ub]);
if result.maxViolation>opts.constraintTolerance
    result.status='INFEASIBLE_OR_UNSATISFIED_CONSTRAINTS';
    return
end
if flag<=0
    result.status='SOLVER_NOT_CONVERGED';
    return
end
if opts.useRK4Check
    % Independent, discrete numerical check of the claimed 25 ms interval.
    % It is a diagnostic only (finite samples, model P, frozen obstacles).
    minH=cbf_joint_rk4_minimum(x,candidate,parts,P,G,opts.dt,opts.rk4Steps);
    result.rk4MinimumH=minH;
    if minH<-opts.constraintTolerance
        result.status='RK4_CROSSCHECK_FAILED';
        return
    end
end
result.input=candidate(:);
result.success=true;
if ~result.nominalFeasible
    result.status='EXPERIMENTAL_FEASIBLE_NONLINEAR_CANDIDATE';
end
end

function opts=cbf_joint_defaults(user)
opts=struct('dt',0.025,'sampleFractions',[0 .25 .5 .75 1], ...
    'lambda',[1 1 1],'enforceLowerStages',false, ...
    'inputLower',[0;-1;-1;-1],'inputUpper',[20;1;1;1], ...
    'inputScale',[10;1;1;1],'inputWeights',[1;1;1;1], ...
    'constraintTolerance',1e-7,'maxIterations',65, ...
    'maxFunctionEvaluations',400,'useRK4Check',true, ...
    'rk4Steps',12,'allowUnsafeStart',false);
names=fieldnames(user);
for k=1:numel(names)
    name=names{k};
    if ~isfield(opts,name)
        error('CBFJoint:Option','Unknown option: %s',name);
    end
    opts.(name)=user.(name);
end
validateattributes(opts.inputLower,{'numeric'},{'vector','numel',4,'real','finite'});
validateattributes(opts.inputUpper,{'numeric'},{'vector','numel',4,'real','finite'});
validateattributes(opts.inputScale,{'numeric'},{'vector','numel',4,'real','finite'});
validateattributes(opts.inputWeights,{'numeric'},{'vector','numel',4,'real','finite'});
validateattributes(opts.maxIterations,{'numeric'},{'scalar','integer','positive'});
validateattributes(opts.maxFunctionEvaluations,{'numeric'},{'scalar','integer','positive'});
validateattributes(opts.rk4Steps,{'numeric'},{'scalar','integer','positive'});
validateattributes(opts.constraintTolerance,{'numeric'},{'scalar','positive','finite'});
end

function [v,inBounds]=cbf_joint_clip_nominal(u,lb,ub)
inBounds=all(u>=lb)&all(u<=ub);
v=min(max(u,lb),ub);
end

function [g,names]=cbf_joint_eval(u,x,parts,P,G,opts,times)
% Evaluate all components with ALL FOUR current candidate inputs together.
% Order-four Taylor coefficients are exact derivatives of the frozen-input
% physical model; time continuation is truncated and not a safety proof.
res=CBF_REAL_ZOH_HOCBF_INEQUALITY(x,u(:),parts,P,G, ...
    struct('lambda',opts.lambda));
d=res.derivatives(1:5,:);
n=numel(res.names);
g=zeros(n*(1+numel(times)+3*double(opts.enforceLowerStages)),1);
names=cell(numel(g),1);
pos=0;
for j=1:n
    pos=pos+1;
    g(pos)=res.values(j);
    names{pos}=['r4_top/' res.names{j}];
end
if opts.enforceLowerStages
    for stage=1:3
        for j=1:n
            pos=pos+1;
            g(pos)=res.stages{j}(stage+1);
            names{pos}=sprintf('stage%d/%s',stage,res.names{j});
        end
    end
end
for it=1:numel(times)
    t=times(it);
    b=[1 t t^2/2 t^3/6 t^4/24];
    ht=b*d;
    for j=1:n
        pos=pos+1;
        g(pos)=ht(j);
        names{pos}=sprintf('zoh_h_t%.6g/%s',t,res.names{j});
    end
end
if pos~=numel(g)
    error('CBFJoint:Internal','Constraint count bookkeeping failed');
end
end

function [c,ceq]=cbf_joint_nlcon(u,x,parts,P,G,opts,times)
g=cbf_joint_eval(u,x,parts,P,G,opts,times);
c=-g;
ceq=[];
end

function minH=cbf_joint_rk4_minimum(x,u,parts,P,G,dt,N)
x=x(:);u=u(:);
h=dt/N;
minH=inf;
for k=0:N
    j=CBF_REAL_ZOH_DIAG_EVAL(x,u,parts,P,G,1);
    if ~isempty(j.names)
        minH=min(minH,min(j.derivatives(1,:)));
    end
    if k==N, break;end
    k1=CBF_COMMON_MODEL(x,u,P);
    k2=CBF_COMMON_MODEL(x+(h/2)*k1,u,P);
    k3=CBF_COMMON_MODEL(x+(h/2)*k2,u,P);
    k4=CBF_COMMON_MODEL(x+h*k3,u,P);
    x=x+(h/6)*(k1+2*k2+2*k3+k4);
    % Keep numeric drift from quaternion/cable norm out of the geometry
    % check. This correction only affects the audit, not the candidate.
    x(1:4)=x(1:4)/norm(x(1:4));
    x(14:16)=x(14:16)/norm(x(14:16));
end
if isinf(minH), minH=NaN;end
end
%<<< CBF_JOINT_END >>>
