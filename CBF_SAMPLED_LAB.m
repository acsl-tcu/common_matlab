function report = CBF_SAMPLED_LAB(action, varargin)
% CBF_SAMPLED_LAB  Independent, read-only Scheme02 research checks.
%
% PURPOSE
%   Inspect the generator currently used by Scheme02, exercise the
%   mathematical jets on one and two obstacles, and summarize an existing
%   FOURWAY results ZIP. This file NEVER patches Sim/CBF/Generator sources.
%
% USAGE (run in the folder containing mainGUI.m)
%   r1 = CBF_SAMPLED_LAB('audit');
%   r1 = CBF_SAMPLED_LAB('audit',pwd,'C:\path\CBF_GENERATOR');
%   r2 = CBF_SAMPLED_LAB('selftest');
%   r3 = CBF_SAMPLED_LAB('results','C:\path\CBF_FOURWAY_RESULTS.zip');
%
% All outputs are diagnostics, NOT a forward-invariance certificate.
% Needs MATLAB and the original Scheme02 generator for 'selftest'.
% Copyright: project-specific review script, 2026-11-22.

if nargin == 0 || isempty(action), action = 'audit'; end
switch lower(char(action))
    case 'audit'
        root = choose_root(varargin);
        report = audit_generator(root, choose_generator(root,varargin));
    case 'selftest'
        root = choose_root(varargin);
        report = run_selftest(root, choose_generator(root,varargin));
    case 'results'
        if isempty(varargin)
            error('CBFLAB:Argument','Provide the existing results ZIP path.');
        end
        report = audit_results(char(varargin{1}));
    otherwise
        error('CBFLAB:Action','Unknown action: %s',char(action));
end
end

function root = choose_root(args)
if isempty(args), root = pwd; else, root = char(args{1}); end
if ~isfile(fullfile(root,'mainGUI.m'))
    error('CBFLAB:Root','Expected mainGUI.m in project root: %s',root);
end
end

function gen = choose_generator(root, args)
% The active-GUI export may include only GENERATED, not SOURCE.
% An original CBF_GENERATOR ZIP can be unpacked outside the project.
if numel(args)>=2 && ~isempty(args{2})
    gen = char(args{2});
else
    gen = fullfile(root,'model','substance','CBF_GENERATOR');
    if ~isfile(fullfile(gen,'REAL','XY','CBF_REAL_XY_EVAL_SOURCE.m'))
        alt = fullfile(root,'CBF_GENERATOR');
        if isfile(fullfile(alt,'REAL','XY','CBF_REAL_XY_EVAL_SOURCE.m'))
            gen = alt;
        end
    end
end
if isfolder(fullfile(gen,'CBF_GENERATOR'))
    gen = fullfile(gen,'CBF_GENERATOR');
end
if ~isfile(fullfile(gen,'REAL','XY','CBF_REAL_XY_EVAL_SOURCE.m'))
    error('CBFLAB:GeneratorSourceMissing', ...
        ['Cannot find XY SOURCE. Unpack CBF_GENERATOR.zip outside the ' ...
         'project and pass its CBF_GENERATOR folder as third argument.']);
end
end

function report = audit_generator(root, gen)
% Exact comparison with the XY copy generator's strrep operation.
xy = fullfile(gen,'REAL','XY');
pairs = { ...
    'CBF_REAL_XY_JET_SOURCE.m', 'CBF_REAL_XY_JET.m'; ...
    'CBF_REAL_XY_EVAL_SOURCE.m','CBF_REAL_XY_EVAL.m'};
checks = struct('source',{},'generated',{},'status',{}, ...
    'runtimeFile',{},'runtimeMatch',{});
for k=1:size(pairs,1)
    source = fullfile(xy,pairs{k,1});
    dest = fullfile(xy,'GENERATED',pairs{k,2});
    item = struct('source',source,'generated',dest,'status','MISSING');
    if isfile(source) && isfile(dest)
        originalName = erase(pairs{k,1},'.m');
        generatedName = erase(pairs{k,2},'.m');
        expected = strrep(fileread(source),originalName,generatedName);
        current = fileread(dest);
        if strcmp(normalize_newlines(expected),normalize_newlines(current))
            item.status = 'SYNC';
        else
            item.status = 'DRIFT';
        end
    end
    runtime = fullfile(root,'model','substance','CBF_GENERATOR', ...
        'REAL','XY','GENERATED',pairs{k,2});
    item.runtimeFile = runtime;
    item.runtimeMatch = 'MISSING';
    if isfile(runtime) && isfile(dest)
        if strcmp(normalize_newlines(fileread(runtime)), ...
                normalize_newlines(fileread(dest)))
            item.runtimeMatch = 'SYNC';
        else
            item.runtimeMatch = 'DRIFT';
        end
    end
    checks(end+1)=item; %#ok<AGROW>
end

scheme = fullfile(root,'controller','HLC_CBF_CONTROLLER','CBF_REAL', ...
    'SCHEMES','CBF_REAL_SCHEME_02_XY.m');
sim = fullfile(root,'mode','SuspendedLoad','SimSuspendedLoadCBFIntegrated.m');
report = struct();
report.kind = 'READ_ONLY_GENERATOR_SOURCE_AUDIT';
report.projectRoot = root;
report.generatorRoot = gen;
report.sourceChecks = checks;
report.schemeFound = isfile(scheme);
report.simFound = isfile(sim);
report.fourwayHookInScheme = false;
report.fourwayHookInSim = false;
if report.schemeFound
    report.fourwayHookInScheme = contains(fileread(scheme), ...
        'CBF_FOURWAY_HOOK_BEGIN');
end
if report.simFound
    report.fourwayHookInSim = contains(fileread(sim), ...
        'CBF_FOURWAY_HOOK_BEGIN');
end
report.proofStatus = 'NOT_PROVEN';
report.proofReason = ['Generator source agreement verifies provenance only. ' ...
    'It does not bound ZOH remainders, estimator error, sensing coverage ' ...
    'or recursive feasibility.'];

fprintf('\n[LAB] Read-only generator audit\n');
for k=1:numel(checks)
    fprintf('  source %-7s  runtime %-7s  %s\n', ...
        checks(k).status,checks(k).runtimeMatch,checks(k).generated);
end
fprintf('  FOURWAY hook in Scheme02: %d\n',report.fourwayHookInScheme);
fprintf('  FOURWAY hook in Sim:      %d\n',report.fourwayHookInSim);
fprintf('  Intersample and recursive guarantee: %s\n',report.proofStatus);
end

function report = run_selftest(root, gen)
% Read-only numerical sanity checks at illustrative states (NOT a proof).
common = fullfile(gen,'COMMON');
xy = fullfile(gen,'REAL','XY');
generated = fullfile(xy,'GENERATED');
if ~isfile(fullfile(xy,'CBF_REAL_XY_SELFTEST.m'))
    error('CBFLAB:GeneratorMissing','Missing Scheme02 XY selftest.');
end
originalPath = path;
cleanupPath = onCleanup(@()path(originalPath)); %#ok<NASGU>
addpath(common,xy,generated,'-begin');

% Existing tests check physical RHS, Lie jet, affine XY torque,
% rotation-dependent cable shape and a short RK4 cross-check.
existing = CBF_REAL_XY_SELFTEST();

% Reproducible TWO-OBSTACLE test, using a single applied torque candidate.
q=[0.98;0.08;-0.12;0.10];q=q/norm(q);
pT=[0.08;-0.11;-0.99];pT=pT/norm(pT);
x=[q;0.1;-0.12;0.08;0;0;3;0.4;-0.2;0.1;pT;0.02;0.03;-0.04];
P=struct('m',0.85,'J',[0.012;0.013;0.022], ...
    'gravity',9.81,'mL',0.14,'cableL',2,'dstx',0,'dsty',0);
G=CBF_COMMON_GEOMETRY(P,struct('includeGround',false, ...
    'trimCableWithBodySpheres',false));
G.cableCenterFromLoad=0.5;
G.cableAxisShort=0.2;
G.cableAxisLong=P.cableL/2+0.02;
lambda=[1;2;8;16];
pa=struct('kind','ellipsoid','center',[1.4;-0.9;2.6], ...
    'R',eye(3),'params',struct('axes',[0.5;0.8;1.2]), ...
    'obstacleName','left');
pb=pa;
pb.center=[-1.6;0.9;2.8];
pb.obstacleName='right';
parts=[pa pb];
u0=[9.5;0;0;0.04];
E0=CBF_REAL_XY_EVAL(x,u0,P,G,parts,lambda);
Ex=CBF_REAL_XY_EVAL(x,u0+[0;1;0;0],P,G,parts,lambda);
Ey=CBF_REAL_XY_EVAL(x,u0+[0;0;1;0],P,G,parts,lambda);
z=[0.38;-0.22];
Ez=CBF_REAL_XY_EVAL(x,u0+[0;z;0],P,G,parts,lambda);
B=[Ex.stages(5,:)-E0.stages(5,:); ...
   Ey.stages(5,:)-E0.stages(5,:)].';
affineError=max(abs(Ez.stages(5,:).'-(E0.stages(5,:).'+B*z)));
assert(isequal(size(E0.stages),[5 2]),'CBFLAB:MultiObs', ...
    'The two obstacles did not produce two complete stage columns.');
assert(affineError<1e-7*max(1,max(abs(Ez.stages(5,:)))), ...
    'CBFLAB:Affine','Two-obstacle XY affine test failed.');

% Same state, different thrust: low-order psi can change at input switches.
uAfter=u0;uAfter(1)=u0(1)+0.25;
Eafter=CBF_REAL_XY_EVAL(x,uAfter,P,G,parts,lambda);
deltaPsi3=Eafter.stages(4,:)-E0.stages(4,:);

report=struct('kind','SYNTHETIC_NUMERICAL_CHECK', ...
    'existingSelftest',existing,'obstacleCount',2, ...
    'xyAffineMaxError',affineError, ...
    'thrustStepN',uAfter(1)-u0(1), ...
    'psi3JumpForSameState',deltaPsi3, ...
    'stateAndParameters','SYNTHETIC_NOT_A_FLIGHT_TEST', ...
    'proofStatus','NOT_PROVEN');
fprintf('\n[LAB] Synthetic tests: PASS\n');
fprintf('  Obstacles evaluated simultaneously: %d\n',report.obstacleCount);
fprintf('  XY affine check, max residual: %.3g\n',affineError);
fprintf('  psi3 change after +%.3g N thrust at SAME state: [%g %g]\n', ...
    report.thrustStepN,deltaPsi3(1),deltaPsi3(2));
fprintf('  Full safety certificate: %s\n',report.proofStatus);
end

function report = audit_results(zipFile)
% Analyze a previously generated ZIP without installing diagnostics.
if ~isfile(zipFile),error('CBFLAB:ResultMissing','ZIP not found: %s',zipFile);end
work=tempname;
mkdir(work);
cleanupFiles=onCleanup(@()delete_temporary(work)); %#ok<NASGU>
unzip(zipFile,work);
required={'PER_SAMPLE.csv','PER_OBSTACLE.csv', ...
    'TRANSITIONS.csv','QP_COMMON.csv'};
for k=1:numel(required)
    if ~isfile(fullfile(work,required{k}))
        error('CBFLAB:BadResult','Missing %s in ZIP.',required{k});
    end
end
S=readtable(fullfile(work,required{1}));
O=readtable(fullfile(work,required{2}));
R=readtable(fullfile(work,required{3}));
Q=readtable(fullfile(work,required{4}));
report=struct();
report.kind='OBSERVED_TRACE_ONLY';
report.samples=height(S);
report.samplesWithObservedObstacles=sum(S.parts>0);
report.maxSimultaneousObstacles=max(S.parts);
report.samplesWithTwoOrMoreObstacles=sum(S.parts>=2);
report.negativeGeometricH=sum(O.h_m<0);
report.negativePsi3=sum(O.psi3<0);
report.currentCommonXYInfeasible=sum(Q.common_box_feasible==0);
report.estimatedAnalyticBoundPass=sum(S.analytic_pass_all~=0 & S.parts>0);
report.estimatedAnalyticBoundFail=sum(S.analytic_pass_all==0 & S.parts>0);
report.psi3PositiveToNegative=sum(R.psi3_0>=0 & R.psi3_1<0);
validJump=isfinite(R.jump_thrust) & isfinite(R.flow_defect) & ...
    R.psi3_0>=0 & R.psi3_1<0;
report.transitionsWithAttribution=sum(validJump);
report.ofThoseThrustJumpLargerThanFlow=sum(validJump & ...
    abs(R.jump_thrust)>abs(R.flow_defect));
report.proofStatus='NOT_PROVEN';
report.proofReason=['A finite simulation trace does not establish ' ...
    'continuous-time invariance or a recursively viable set.'];

fprintf('\n[LAB] Existing FOURWAY ZIP (no Sim modifications)\n');
fprintf('  Logged samples:                    %d\n',report.samples);
fprintf('  Samples with an observed obstacle: %d\n',report.samplesWithObservedObstacles);
fprintf('  Maximum simultaneous obstacles:   %d\n',report.maxSimultaneousObstacles);
fprintf('  Negative support h samples:       %d\n',report.negativeGeometricH);
fprintf('  Negative psi3 entries:            %d\n',report.negativePsi3);
fprintf('  Infeasible common current XY-QP:  %d\n',report.currentCommonXYInfeasible);
fprintf('  One-step lower bound PASS / FAIL: %d / %d\n', ...
    report.estimatedAnalyticBoundPass,report.estimatedAnalyticBoundFail);
fprintf('  Controlled invariance proof:      %s\n',report.proofStatus);
end

function s=normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function delete_temporary(work)
if isfolder(work), rmdir(work,'s'); end
end
