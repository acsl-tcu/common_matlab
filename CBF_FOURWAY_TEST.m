function out = CBF_FOURWAY_TEST(action, varargin)
% CBF_FOURWAY_TEST  Scheme02 passive four-way safety audit (2026-11-21).
%
% ONE M-file. Put in the folder containing mainGUI.m, add project paths,
% then run ONCE before the normal GUI simulation:
%    CBF_FOURWAY_TEST('selftest');
%    CBF_FOURWAY_TEST('install');
% Run mainGUI.m and the usual SimSuspendedLoadCBFIntegrated (true mass case).
% At the end a SINGLE CBF_FOURWAY_RESULTS.zip is saved next to this M-file.
% If the simulation stops early: CBF_FOURWAY_TEST('save');
% Status: CBF_FOURWAY_TEST('status');  Undo hooks: 'uninstall'.
%
% If install cannot locate source files, pass the project folder explicitly:
%    CBF_FOURWAY_TEST('install','C:\path\to\common_matlab');
%
% The installer creates .cbf4way.bak backups of two existing M-files and
% inserts PASSIVE HOOKS only. It does not alter controller computation.
% Results are not a plant-wide, sampled-data or recursive safety proof.
% In particular this scheme lacks body sphere, load sphere, ground CBFs.
%
% Instrumentation is intentionally inside the full-successful Scheme02
% execution path; early returns are detectable only as missing records.
% This M-file also performs substantial OFFLINE checks after simulation.

persistent snapshots captureErrors started
if isempty(snapshots), snapshots=cell(0,1); end
if isempty(captureErrors), captureErrors=cell(0,1); end
if isempty(started), started=datestr(now,31); end
if nargin==0,action='status';end
out=[];
switch lower(char(action))
    case 'reset'
        snapshots=cell(0,1);captureErrors=cell(0,1);started=datestr(now,31);
        fprintf('[CBF 4WAY] reset. Controller logic untouched.\n');
    case 'record'
        try
            if numel(varargin)~=11
                error('CBF4:Arguments','record needs 11 arguments');
            end
            r=make_record(varargin{:});
            snapshots{end+1,1}=r;
        catch ME
            if numel(captureErrors)<100
                captureErrors{end+1,1}=sprintf('RECORD %d: %s: %s', ...
                    numel(snapshots)+1,ME.identifier,ME.message);
            end
        end
    case 'status'
        out=struct('snapshots',numel(snapshots), ...
            'captureErrors',{captureErrors},'start',started);
        fprintf('[CBF 4WAY] snapshots=%d, errors=%d\n', ...
            numel(snapshots),numel(captureErrors));
    case {'save','analyze'}
        out=save_report(snapshots,captureErrors,started);
    case 'install'
        if isempty(varargin),root='';else,root=varargin{1};end
        out=install_hooks(root,false);
    case 'uninstall'
        if isempty(varargin),root='';else,root=varargin{1};end
        out=install_hooks(root,true);
    case 'selftest'
        out=run_selftest();
    otherwise
        error('CBF4:UnknownAction','Unknown action: %s',char(action));
end
end

function r=make_record(agent,x,u,P,G,parts,lambda,tiltCfg,info,dt,varargCell)
assert(isnumeric(x)&&numel(x)==19&&all(isfinite(x(:))),'Bad x');
assert(isnumeric(u)&&numel(u)==4&&all(isfinite(u(:))),'Bad u');
assert(isstruct(info)&&isfield(info,'stageValues'),'Bad info');
assert(isstruct(parts),'Bad obstacles');
r=struct();
r.x=double(x(:));r.u=double(u(:));r.P=P;r.G=G;r.parts=parts;
r.lambda=double(lambda(:));r.tiltCfg=tiltCfg;r.info=info;
r.dt=double(dt);r.time=NaN;
% Read the actual SIM PLANT state BEFORE plant.do at the same control step.
% This is not an estimator and does not modify either state object.
r.trueX=nan(19,1);r.trueStateStatus='MISSING';
try
    st=agent.plant.state;
    z=[double(st.getq('compact'));double(st.w(:));double(st.pL(:)); ...
       double(st.vL(:));double(st.pT(:));double(st.wL(:))];
    if numel(z)==19 && all(isfinite(z))
        r.trueX=z(:);r.trueStateStatus='PLANT_STATE_BEFORE_STEP';
    end
catch ME
    r.trueStateStatus=['PLANT_ACCESS_' ME.identifier];
end
if iscell(varargCell)&&~isempty(varargCell)
    try
        timeArg=varargCell{1};
        if isobject(timeArg),r.time=double(timeArg.t);
        elseif isstruct(timeArg)&&isfield(timeArg,'t'),r.time=double(timeArg.t);end
    catch
    end
end
r.names=info.constraintNames;
r.stages=double(info.stageValues);
assert(isnumeric(r.stages)&&size(r.stages,1)==5,'Invalid HOCBF stages');
assert(size(r.stages,2)==numel(parts),'Inconsistent obstacle stages');
assert(numel(r.lambda)==4&&isfinite(r.dt)&&r.dt>0,'Bad dt/lambda');
end

function out=install_hooks(root,undo)
[scheme,sim]=locate_source(root);
marker1='% CBF_FOURWAY_HOOK_BEGIN';marker2='% CBF_FOURWAY_HOOK_END';
sc=fileread(scheme);sm=fileread(sim);
if undo
    targets={scheme,sim};
    for i=1:2
        backup=[targets{i} '.cbf4way.bak'];
        if ~isfile(backup)
            error('CBF4:NoBackup','Cannot undo; backup missing: %s',backup);
        end
        current=fileread(targets{i});
        if ~contains(current,marker1)||~contains(current,marker2)
            error('CBF4:ModifiedSource','Refusing to overwrite non-instrumented file: %s',targets{i});
        end
        copyfile(backup,targets{i},'f');
        delete(backup);
    end
    rehash;
    fprintf('[CBF 4WAY] Restored Scheme02 and Sim from backups.\n');
    out='UNINSTALLED';return
end
if contains(sc,marker1)&&contains(sm,marker1)
    fprintf('[CBF 4WAY] Hooks already installed; no changes.\n');
    out='ALREADY_INSTALLED';return
end
if contains(sc,marker1)||contains(sm,marker1)
    error('CBF4:PartialInstall','Partial hooks found; inspect files before install');
end
% The anchor matches the FINAL return of the CURRENT Scheme02. Every
% successful regular cycle is logged. No arithmetic or input assignment
% is modified, and all audit exceptions are contained inside try/catch.
sc0=normalize_lf(sc);
anchor=sprintf('info.solverExitflag=double(isFeasible);\ninfo.elapsedMs=1000*toc(start);\n');
if numel(strfind(sc0,anchor))~=1
    error('CBF4:Version','Scheme02 version does not match audited source (final-result anchor)');
end
hook=[marker1 newline ...
    'try' newline ...
    '    CBF_FOURWAY_TEST(''record'',agent,xModel,u,P,G,observed.parts,lambda,tiltCfg,info,dt,varargin);' newline ...
    'catch CBF4HOOKENV' newline ...
    '    warning(''CBF4:CaptureHook'',''Passive capture failed: %s'',CBF4HOOKENV.message);' newline ...
    'end' newline marker2 newline];
scNew=strrep(sc0,anchor,[anchor hook]);
sm0=normalize_lf(sm);
resetAnchor="CBF_REAL_DIAG_VIEW('reset');";
postAnchor='function post(app, worldObstacles, cbfDiagGraphs)';
if ~contains(sm0,resetAnchor)||~contains(sm0,postAnchor)
    error('CBF4:Version','Simulation version does not match audited source');
end
smNew=strrep(sm0,resetAnchor, ...
    [char(resetAnchor) newline marker1 newline ...
    'CBF_FOURWAY_TEST(''reset'');' newline marker2]);
smNew=strrep(smNew,postAnchor, ...
    [postAnchor newline marker1 newline ...
    'try' newline ...
    '    CBF_FOURWAY_TEST(''save'');' newline ...
    'catch CBF4SAVEERR' newline ...
    '    warning(''CBF4:SaveHook'',''Report save failed: %s'',CBF4SAVEERR.message);' newline ...
    'end' newline marker2]);
% Never silently overwrite previous backups.
if isfile([scheme '.cbf4way.bak'])||isfile([sim '.cbf4way.bak'])
    error('CBF4:ExistingBackup','A prior backup exists; inspect before installing');
end
copyfile(scheme,[scheme '.cbf4way.bak']);
copyfile(sim,[sim '.cbf4way.bak']);
try
    put_text(scheme,scNew);put_text(sim,smNew);
catch ME
    copyfile([scheme '.cbf4way.bak'],scheme,'f');
    copyfile([sim '.cbf4way.bak'],sim,'f');
    rethrow(ME)
end
rehash;
fprintf('[CBF 4WAY] Installed ONLY passive hooks into:\n%s\n%s\n',scheme,sim);
fprintf('[CBF 4WAY] Run mainGUI and normal simulation. Final report: %s\n', ...
    fullfile(fileparts(mfilename('fullpath')),'CBF_FOURWAY_RESULTS.zip'));
out='INSTALLED';
end

function [scheme,sim]=locate_source(root)
if nargin==0||isempty(root),root=fileparts(mfilename('fullpath'));end
root=char(root);
scheme=fullfile(root,'controller','HLC_CBF_CONTROLLER','CBF_REAL', ...
    'SCHEMES','CBF_REAL_SCHEME_02_XY.m');
sim=fullfile(root,'mode','SuspendedLoad','SimSuspendedLoadCBFIntegrated.m');
if ~isfile(scheme)||~isfile(sim)
    a=which('CBF_REAL_SCHEME_02_XY');b=which('SimSuspendedLoadCBFIntegrated');
    if ~isempty(a)&&~isempty(b),scheme=a;sim=b;end
end
if ~isfile(scheme)||~isfile(sim)
    error('CBF4:NotFound',[ ...
        'Cannot locate Scheme02 and Sim. Place test M next to mainGUI, ' ...
        'addpath(genpath(pwd)), or use install(projectRoot).']);
end
end

function s=normalize_lf(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function put_text(filename,s)
fid=fopen(filename,'w');
if fid<0,error('CBF4:IO','Cannot write %s',filename);end
cleanup=onCleanup(@() fclose(fid));
fwrite(fid,s,'char');
end

function zipPath=save_report(R,err,started)
folder=fileparts(mfilename('fullpath'));
zipPath=fullfile(folder,'CBF_FOURWAY_RESULTS.zip');
work=tempname;mkdir(work);
workCleanup=onCleanup(@() try_clean(work));
N=numel(R);
summary={... 
    'CBF 4WAY / Scheme02 / four-part safety diagnostic', ...
    ['Started: ' started], ...
    sprintf('Captured successful Scheme02 cycles: %d',N), ...
    sprintf('Capture errors: %d',numel(err)), ...
    '', ...
    'CRITICAL SCOPE:', ...
    'Cable ellipsoid / observed obstacle support h ONLY; sampled CBF tilt constraints.', ...
    'No ground, drone sphere, payload sphere or unobserved-obstacle guarantee.', ...
    'h<0 means failure of this chosen support-axis sufficient separation test;', ...
    'it does NOT by itself establish physical collision.', ...
    'Outputs labelled MODEL_CONDITIONAL are sufficient ONLY when model,', ...
    'state, stationary obstacles, held applied input and arithmetic are validated.', ...
    'There is NO unconditional plant interval or recursive certificate.', ...
    'Observed multi-QP feasibility is a sample-time 2D convex condition only.', ...
    'CURRENT active Scheme02 uses SINGLE_QP_GUIDANCE (no next-step preview).', ...
    'Its previewBaseNextReserve and previewNextReserve are initialized NaN;', ...
    'QP_COMMON.csv computes independent current max-min reserves offline.'};
stepHead={'k','t','phase','parts','status','qp_common','qp_reported','all_lower_nonnegative', ...
    'min_h_m','min_psi1','min_psi2','min_psi3','min_psi4','min_qp_psi_final', ...
    'guard_status','guard_normalized_min','used_recovery', ...
    'guide_status','guide_activation','guide_deltaNm','preview_base_reserve', ...
    'preview_chosen_reserve','preview_base_lower','preview_chosen_lower', ...
    'preview_next_feasible','analytic_h_lower_min','analytic_pass_all', ...
    'analytic_unknown_count','simultaneous_obstacle_constraints','joint_with_tilt', ...
    'full_vehicle_certified','plant_true_min_h_m','plant_state_delta_norm','plant_true_analytic_pass'};
stepRows=cell(0,numel(stepHead));
obsHead={'k','t','obstacle','h_m','psi1','psi2','psi3','psi4', ...
    'analytic_ZOH_lower_m','analytically_sufficient','boundary_reason', ...
    'plant_true_h_m','plant_true_analytic_lower_m','plant_true_analytic_pass'};
obsRows=cell(0,numel(obsHead));
pairHead={'t0','t1','obstacle','h0','h1','h_crossed', ...
    'psi1_0','psi1_1','psi2_0','psi2_1','psi3_0','psi3_1', ...
    'psi4_0','psi4_1','psi3_exp_lower','psi3_defect', ...
    'flow_defect','jump_thrust','jump_yaw','jump_XY','jump_geometry_model', ...
    'identity_error','conditional_min_psi4_upper', ...
    'old_obstacle_matching','same_input_fixed_TYaw','guard0','guard1', ...
    'QP0_feasible','QP1_feasible','analysis_kind', ...
    'plant_true_h0','plant_true_h1','plant_true_h_crossed', ...
    'estimated_true_sign_mismatch_at_next','same_part_geometry', ...
    'same_model_parameters','same_lambda'};
pairRows=cell(0,numel(pairHead));
qpHead={'k','t','all_constraints','obstacle_constraints','individual_rows_box_feasible', ...
    'common_box_feasible','common_polygon_vertices','active_tilt_rows', ...
    'outside_lower_set','min_affine_residual_at_selected', ...
    'conflicting_constraints_2_or_3','margin_normalized_approx'};
qpRows=cell(0,numel(qpHead));
if N==0
    summary{end+1}='NO SNAPSHOTS. Check install, reset timing, scheme id=2, and MATLAB paths.';
end
allT=nan(N,1);
for i=1:N
    if isfinite(R{i}.time),allT(i)=R{i}.time;
    else,allT(i)=(i-1)*R{i}.dt;end
end
[allT,ix]=sort(allT);R=R(ix);
countNegH=0;countTrueNegH=0;countPsi3=0;countQPEmpty=0;countGuardFall=0;
countModelPass=0;countModelUnknown=0;countPair=0;countJumpEval=0;
countJumpFailed=0;countHcross=0;countTrueHcross=0;countFeasLoss=0;countGuide=0;
maxJumpEval=250;maxWitness=20;detailedFailures=cell(0,1);
qFeas=false(N,1);sameStatePass=false(N,1);
for k=1:N
    a=R{k};I=a.info;t=allT(k);
    n=numel(a.parts);h=colstage(a.stages,1);ps1=colstage(a.stages,2);
    ps2=colstage(a.stages,3);ps3=colstage(a.stages,4);ps4=colstage(a.stages,5);
    B=getf(I,'torqueInfluenceMatrix',zeros(0,2));
    bias=getf(I,'psi4Bias',zeros(0,1));
    names=getf(I,'constraintNames',cell(0,1));
    if ~isnumeric(B)||size(B,2)~=2||size(B,1)~=numel(bias)
        B=zeros(0,2);bias=zeros(0,1);names=cell(0,1);
        err{end+1}=sprintf('k=%d invalid B/bias dimensions',k);
    end
    [poly,ok]=halfspace_polygon(B,bias,0);
    qFeas(k)=ok;
    countQPEmpty=countQPEmpty+double(~ok);
    minSelected=localmin(bias+B*a.u(2:3));
    outside=any(a.stages(1:4,:)<-1e-8,'all') || ...
        low_tilt(getf(I,'tiltBodyStages',[])) || ...
        low_tilt(getf(I,'tiltCableStages',[]));
    lowerAll=~outside;
    ng=min(n,size(B,1));
    individual=all(bias>=-sum(abs(B),2)-1e-9);
    mismatch=NaN;
    if isfield(I,'previewCurrentFeasible'),mismatch=double(I.previewCurrentFeasible);end
    guard=char(getf(I,'zohGuardStatus','UNKNOWN'));
    countGuardFall=countGuardFall+double(strcmp(guard,'NO_COMMON_GUARD_INPUT_FALLBACK'));
    guide=char(getf(I,'previewStatus','UNKNOWN'));
    activation=getf(I,'previewGuideActivation',NaN);
    countGuide=countGuide+double(isfinite(activation)&&activation>0);
    boundMin=Inf;unknown=0;passedAll=true;trueHs=nan(n,1);truePassAll=true;trueKnown=0;
    for j=1:n
        [lb,flag,reason]=analytic_interval_bound(a,j);
        if ~isfinite(lb),unknown=unknown+1;passedAll=false;
        else,boundMin=min(boundMin,lb);if lb<0,passedAll=false;end,end
        if j<=size(a.stages,2),v=num2cell(a.stages(:,j).');
        else,v=num2cell(nan(1,5));end
        trueLB=NaN;trueFlag=false;
        if all(isfinite(a.trueX))
            trueHs(j)=direct_h_geometry(a.trueX,a.P,a.G,a.parts(j));
            if isfinite(trueHs(j))
                rTrue=a;rTrue.x=a.trueX;
                rTrue.info.stageValues(1,j)=trueHs(j);
                [trueLB,trueFlag]=analytic_interval_bound(rTrue,j);
                trueKnown=trueKnown+1;
                if ~trueFlag,truePassAll=false;end
            else
                truePassAll=false;
            end
        else
            truePassAll=false;
        end
        obsRows(end+1,:)={k,t,obs_name(a,j),v{:},lb,double(flag),reason, ...
            trueHs(j),trueLB,double(trueFlag)}; %#ok<AGROW>
    end
    if n==0,boundMin=NaN;passedAll=false;truePassAll=false;end
    if trueKnown~=n,truePassAll=false;end
    countTrueNegH=countTrueNegH+sum(trueHs<-1e-9);
    sameStatePass(k)=passedAll;
    countModelPass=countModelPass+double(passedAll);
    countModelUnknown=countModelUnknown+double(unknown>0);
    countNegH=countNegH+sum(h<-1e-9);
    countPsi3=countPsi3+sum(ps3<-1e-9);
    nTilt=max(0,numel(bias)-n);
    witness='NOT_COMPUTED';
    if ~ok&&countQPEmpty<=maxWitness
        witness=conflict_witness(B,bias,names);
    end
    reserve=NaN;
    if mod(k,10)==1||~ok||minSelected<0.05|| ...
            (isfinite(activation)&&activation>0)
        reserve=approx_maxmin_margin(B,bias);
    end
    qpRows(end+1,:)={k,t,numel(bias),n,double(individual),double(ok), ...
        size(poly,1),nTilt,double(outside),minSelected,witness,reserve}; %#ok<AGROW>
    stepRows(end+1,:)={k,t,char(getf(I,'phase','')),n,char(getf(I,'status','')), ...
        double(ok),mismatch,double(lowerAll),localmin(h),localmin(ps1), ...
        localmin(ps2),localmin(ps3),localmin(ps4),minSelected,guard, ...
        getf(I,'zohGuardMinNormalized',NaN),double(getf(I,'usedRecovery',false)), ...
        guide,activation,norm(getf(I,'previewExtraTorque',[0;0])), ...
        getf(I,'previewBaseNextReserve',NaN),getf(I,'previewNextReserve',NaN), ...
        getf(I,'previewBaselineLowerMargin',NaN), ...
        getf(I,'previewChosenLowerMargin',NaN), ...
        double(getf(I,'previewNextFeasible',false)),boundMin, ...
        double(passedAll),unknown,n,double(ok),0, ...
        localmin(trueHs),norm_or_nan(a.x-a.trueX),double(truePassAll)}; %#ok<AGROW>
end
for k=1:max(0,N-1)
    a=R{k};b=R{k+1};t0=allT(k);t1=allT(k+1);stepDt=t1-t0;
    if ~isfinite(stepDt)||stepDt<=0|| ...
            abs(stepDt-a.dt)>max(1e-6,1e-3*a.dt)
        continue
    end
    countPair=countPair+1;
    if qFeas(k)&&~qFeas(k+1),countFeasLoss=countFeasLoss+1;end
    for j=1:numel(a.parts)
        name=obs_name(a,j);jj=find_obs(b,name);
        if isnan(jj),continue;end
        h0=a.stages(1,j);h1=b.stages(1,jj);
        p0=a.stages(4,j);p1=b.stages(4,jj);
        L=exp(-a.lambda(4)*stepDt)*p0;
        defect=p1-L;
        crossing=(h0>=0&&h1<0);
        countHcross=countHcross+double(crossing);
        vals={t0,t1,name,h0,h1,double(crossing), ...
            a.stages(2,j),b.stages(2,jj),a.stages(3,j),b.stages(3,jj), ...
            p0,p1,a.stages(5,j),b.stages(5,jj),L,defect};
        flow=NaN;jt=NaN;jy=NaN;jxy=NaN;jg=NaN;ident=NaN;
        minPsiUpper=NaN;kind='SAMPLE_ONLY';
        detailed=(crossing||h1<0||p1<0||defect<-1e-7||mod(k,40)==1);
        if detailed&&countJumpEval<maxJumpEval
            try
                % Exact algebraic telescoping at the SAME measured endpoint
                % state. Flow attribution is conditional on true flow=logged.
                z0=eval_old(b.x,a.u,a,name);
                ut=a.u;ut(1)=b.u(1);
                zt=eval_old(b.x,ut,a,name);
                uy=ut;uy(4)=b.u(4);
                zy=eval_old(b.x,uy,a,name);
                zn=eval_old(b.x,b.u,a,name);
                flow=z0-L;jt=zt-z0;jy=zy-zt;jxy=zn-zy;jg=p1-zn;
                ident=defect-(flow+jt+jy+jxy+jg);
                K=-expm1(-a.lambda(4)*stepDt)/a.lambda(4);
                if flow<0&&K>0,minPsiUpper=flow/K;end
                kind='ENDPOINT_IDENTITY_CONDITIONAL';countJumpEval=countJumpEval+1;
            catch ME
                countJumpFailed=countJumpFailed+1;
                if numel(detailedFailures)<150
                    detailedFailures{end+1}=sprintf('%.8g %s %s: %s', ...
                        t0,name,ME.identifier,ME.message);
                end
                kind='EVAL_FAILED';
            end
        elseif detailed
            kind='EVAL_BUDGET_EXCEEDED';
        end
        trueH0=direct_h_geometry(a.trueX,a.P,a.G,a.parts(j));
        trueH1=direct_h_geometry(b.trueX,b.P,b.G,b.parts(jj));
        trueCross=isfinite(trueH0)&&isfinite(trueH1)&&trueH0>=0&&trueH1<0;
        countTrueHcross=countTrueHcross+double(trueCross);
        signMismatch=double(isfinite(trueH1)&&(h1<0)~=(trueH1<0));
        geoSame=isequaln(a.parts(j),b.parts(jj))&&isequaln(a.G,b.G);
        modelSame=isequaln(a.P,b.P);gainSame=isequaln(a.lambda,b.lambda);
        pairRows(end+1,:)=[vals,{flow,jt,jy,jxy,jg,ident,minPsiUpper, ...
            double(~isnan(jj)),double(all(abs(a.u([1 4])-b.u([1 4]))<1e-12)), ...
            char(getf(a.info,'zohGuardStatus','')), ...
            char(getf(b.info,'zohGuardStatus','')), ...
            double(qFeas(k)),double(qFeas(k+1)),kind, ...
            trueH0,trueH1,double(trueCross),signMismatch,double(geoSame), ...
            double(modelSame),double(gainSame)}]; %#ok<AGROW>
    end
end
summary=[summary, { ...
    '', 'PART 1 / negative separation and boundary effects:', ...
    sprintf('Negative estimated support-h obstacle samples: %d',countNegH), ...
    sprintf('Negative direct PLANT-STATE support-h obstacle samples: %d',countTrueNegH), ...
    sprintf('Negative psi3 obstacle samples: %d',countPsi3), ...
    sprintf('Crossings estimated h>=0 to h<0 at sampled endpoints: %d',countHcross), ...
    sprintf('Crossings plant-state h>=0 to h<0 at endpoints: %d',countTrueHcross), ...
    sprintf('Valid adjacent actual-sample pairs: %d',countPair), ...
    sprintf('Detailed same-endpoint decompositions OK: %d',countJumpEval), ...
    sprintf('Detailed same-endpoint decomposition failures: %d',countJumpFailed), ...
    sprintf('Expensive attribution capped at %d successful calls (more are SAMPLE_ONLY).',maxJumpEval), ...
    '', 'PART 2 / one ZOH analytic enclosure:', ...
    sprintf('Samples with all observed cable-obstacle h bounds >= 0 under model assumptions: %d',countModelPass), ...
    sprintf('Samples with at least one undefined enclosure: %d',countModelUnknown), ...
    'Analytic formula covers ALL s in [0,dt] under exact 19-state model:', ...
    'Omegamax=norm(wL0)+abs(T)*dt/(m*L);', ...
    'amax=g+(abs(T)+m*L*Omegamax^2)/(m+mL)+norm([dstx,dsty]);', ...
    'Vmax=norm(vL0)+amax*dt+(centerFraction*L)*Omegamax;', ...
    'rmin=norm(C0-obsCenter)-Vmax*dt (>0 required);', ...
    'M=Vmax + ((aLong^2-aShort^2)/aShort+obsMaxAxis)*Vmax/rmin', ...
    '  + ((aLong^2-aShort^2)/aShort)*Omegamax;', ...
    'h(t) >= h(0)-M*dt for all t in interval. No Taylor remainder discarded.', ...
    'This bound is sufficient but conservative; failure does not imply collision.', ...
    'Plant state BEFORE model integration is ALSO sampled in each record.', ...
    'Direct actual-state geometric h and its bound are in PER_OBSTACLE.csv.', ...
    'Model/true-state consistency, obstacle coverage and interval-rounded', ...
    'arithmetic are NOT validated by this numeric audit.', ...
    '', 'PART 3 / recursive feasibility:', ...
    sprintf('Observed feasible(k) -> infeasible(k+1) transitions: %d',countFeasLoss), ...
    'A finite feasible trace, or a positive one-step reserve, is NOT a proof', ...
    'of a controlled-invariant set K or recursive feasibility.', ...
    'Proof obligation: for every x in K exists actual bounded u s.t.', ...
    'Phi_s(x,u) in SAFE for ALL s in [0,dt], Phi_dt(x,u) in K,', ...
    'including input jumps, uncertainty, detection and physical bounds.', ...
    'No validated invariant K is supplied here; verdict = NOT_PROVEN.', ...
    '', 'PART 4 / all-obstacle common feasible XY input:', ...
    sprintf('Common XY QP infeasible samples (incl. tilt): %d',countQPEmpty), ...
    sprintf('Model ZOH guard fallback cycles: %d',countGuardFall), ...
    sprintf('Cycles with nonzero guidance activation: %d',countGuide), ...
    'BOX_POLYGON test uses all affine rows at the SAME torque and [-1,1]^2.', ...
    'A nonempty polygon is only numerically verified sampled affine feasibility.', ...
    'Source scheme already has observed-obstacle cap error rather than silent drop.', ...
    'Fixed thrust and yaw may make common XY feasible set empty.', ...
    'ALL_GEOMETRY or whole-system safety is NEVER certified by Scheme02.', ...
    '', 'FILES: PER_SAMPLE.csv, PER_OBSTACLE.csv, TRANSITIONS.csv, QP_COMMON.csv,', ...
    'ERRORS.txt and RAW_SNAPSHOTS.mat for later exact analysis.', ...
    'IMPORTANT: no changes to HLC, controller gains, input or soft guidance.'}];
write_lines(fullfile(work,'SUMMARY.txt'),summary);
write_csv(fullfile(work,'PER_SAMPLE.csv'),stepHead,stepRows);
write_csv(fullfile(work,'PER_OBSTACLE.csv'),obsHead,obsRows);
write_csv(fullfile(work,'TRANSITIONS.csv'),pairHead,pairRows);
write_csv(fullfile(work,'QP_COMMON.csv'),qpHead,qpRows);
allErr=[err(:);detailedFailures(:)];
if isempty(allErr),allErr={'No snapshot/evaluation exceptions.'};end
write_lines(fullfile(work,'ERRORS.txt'),allErr);
try
    snapshot=R; auditTimes=allT; %#ok<NASGU>
    try
        save(fullfile(work,'RAW_SNAPSHOTS.mat'),'snapshot','auditTimes','-v7.3');
    catch
        save(fullfile(work,'RAW_SNAPSHOTS.mat'),'snapshot','auditTimes','-v7');
    end
catch ME
    allErr{end+1}=sprintf('MAT serialization FAILED %s: %s',ME.identifier,ME.message);
    write_lines(fullfile(work,'ERRORS.txt'),allErr);
end
if isfile(zipPath),delete(zipPath);end
fileList={'SUMMARY.txt','PER_SAMPLE.csv','PER_OBSTACLE.csv', ...
    'TRANSITIONS.csv','QP_COMMON.csv','ERRORS.txt'};
if isfile(fullfile(work,'RAW_SNAPSHOTS.mat'))
    fileList{end+1}='RAW_SNAPSHOTS.mat';
end
zip(zipPath,fileList,work);
clear workCleanup
fprintf('[CBF 4WAY] ZIP only: %s\n',zipPath);
fprintf('[CBF 4WAY] snapshots=%d, negative h=%d, sample h crossings=%d, QP empty=%d, detailed=%d\n', ...
    N,countNegH,countHcross,countQPEmpty,countJumpEval);
end

function [lo,pass,reason]=analytic_interval_bound(a,j)
lo=NaN;pass=false;reason='UNKNOWN';
try
    x=a.x;u=a.u;P=a.P;G=a.G;part=a.parts(j);dt=a.dt;
    m=P.m;L=P.cableL;ml=P.mL;
    if any(~isfinite(x))||any(~isfinite(u))||m<=0||L<=0||ml<0 ...
            ||dt<=0||~isfinite(dt)
        reason='MODEL_PARAMS_INVALID';return
    end
    Om=norm(x(17:19))+abs(u(1))*dt/(m*L);
    amax=P.gravity+(abs(u(1))+m*L*Om^2)/(m+ml)+hypot(P.dstx,P.dsty);
    Vmax=norm(x(11:13))+amax*dt+G.cableCenterFromLoad*L*Om;
    center=x(8:10)-G.cableCenterFromLoad*L*x(14:16);
    d=center-double(part.center(:));r0=norm(d);rmin=r0-Vmax*dt;
    if rmin<=0,reason='DISTANCE_LOWER_BOUND_NONPOSITIVE';return,end
    if isfield(part,'kind')&&strcmp(char(part.kind),'sphere')
        if isfield(part,'params'),axes=part.params.radius*ones(3,1);
        else,axes=part.radius*ones(3,1);end
    else
        if isfield(part,'params'),axes=part.params.axes(:);
        else,axes=part.axes(:);end
    end
    aobs=max(double(axes(:)));
    if ~isfinite(aobs)||aobs<=0||G.cableAxisShort<=0|| ...
            G.cableAxisLong<G.cableAxisShort
        reason='OBSTACLE_OR_CABLE_GEOMETRY_INVALID';return
    end
    along=G.cableAxisLong;ashort=G.cableAxisShort;ratio=(along*along-ashort*ashort)/ashort;
    M=Vmax+(ratio+aobs)*Vmax/rmin+ratio*Om;
    lo=a.info.stageValues(1,j)-M*dt;
    pass=isfinite(lo)&&lo>=0;
    if pass,reason='MODEL_CONDITIONAL_SUFFICIENT';
    else,reason='MODEL_BOUND_NOT_SUFFICIENT';end
catch ME
    reason=['BOUND_ERROR_' ME.identifier];
end
end

function h=direct_h_geometry(x,P,G,part)
% Same chosen centre-to-centre support axis as generated CBF_REAL_XY_EVAL.
h=NaN;
try
    pT=double(x(14:16));pT=pT/norm(pT);
    C=x(8:10)-G.cableCenterFromLoad*P.cableL*pT;
    d=C-double(part.center(:));dd=norm(d);
    if dd<1e-6,return;end
    n=d/dd;
    sc=sqrt(G.cableAxisShort^2+ ...
        (G.cableAxisLong^2-G.cableAxisShort^2)*(n.'*pT)^2);
    if isfield(part,'kind')&&strcmp(char(part.kind),'sphere')
        if isfield(part,'params'),rad=part.params.radius;
        else,rad=part.radius;end
        so=rad;
    else
        if isfield(part,'params'),ax=part.params.axes(:);
        else,ax=part.axes(:);end
        R=part.R;
        so=norm(diag(ax)*R.'*n);
    end
    h=dd-sc-so;
catch
    h=NaN;
end
end

function n=norm_or_nan(x)
if any(~isfinite(x)),n=NaN;else,n=norm(x);end
end

function val=eval_old(x,u,a,name)
e=CBF_REAL_XY_EVAL(x,u,a.P,a.G,a.parts,a.lambda,a.tiltCfg);
idx=find(strcmp(e.names,name),1);
if isempty(idx),error('CBF4:Name','Cannot match %s',name);end
val=e.stages(4,idx);
end

function name=obs_name(a,j)
if j<=numel(a.names),name=char(a.names{j});
else,name=sprintf('part%d',j);end
end

function j=find_obs(a,name)
j=find(strcmp(a.names(1:min(numel(a.parts),numel(a.names))),name),1);
if isempty(j),j=NaN;end
end

function [poly,ok]=halfspace_polygon(B,b,margin)
% Feasible set = {z in [-1,1]^2 : B*z+b >= margin}.
poly=[-1 -1;1 -1;1 1;-1 1];
if isempty(b),ok=true;return;end
if size(B,1)~=numel(b)||size(B,2)~=2|| ...
        any(~isfinite([B(:);b(:)]))
    poly=zeros(0,2);ok=false;return
end
for k=1:numel(b)
    prev=poly(end,:);fp=B(k,:)*prev.'+b(k)-margin;
    next=zeros(0,2);
    for j=1:size(poly,1)
        q=poly(j,:);fq=B(k,:)*q.'+b(k)-margin;
        in0=(fp>=-1e-10);in1=(fq>=-1e-10);
        if in0~=in1
            d=fp-fq;
            if abs(d)>1e-15
                alpha=fp/d;
                next(end+1,:)=prev+alpha*(q-prev); %#ok<AGROW>
            end
        end
        if in1,next(end+1,:)=q;end %#ok<AGROW>
        prev=q;fp=fq;
    end
    if isempty(next),poly=zeros(0,2);ok=false;return;end
    % Duplicate vertices do not alter feasibility, but cause wasted work.
    keep=true(size(next,1),1);
    for j=2:size(next,1)
        if norm(next(j,:)-next(j-1,:))<1e-11,keep(j)=false;end
    end
    poly=next(keep,:);
end
ok=~isempty(poly);
end

function w=conflict_witness(B,b,names)
n=numel(b);w='NO_SMALL_WITNESS_FOUND';
if n>35,w='SKIPPED_MANY_ROWS';return;end
% In R^2, empty intersection of convex box and halfspaces admits a Helly
% witness with at most 3 added halfspaces (box considered one convex set).
for m=1:min(3,n)
    combos=nchoosek(1:n,m);
    for k=1:size(combos,1)
        ids=combos(k,:);
        [~,ok]=halfspace_polygon(B(ids,:),b(ids),0);
        if ~ok
            label=cell(1,numel(ids));
            for j=1:numel(ids)
                if ids(j)<=numel(names),label{j}=char(names{ids(j)});
                else,label{j}=sprintf('row%d',ids(j));end
            end
            w=strjoin(label,' + ');return
        end
    end
end
end

function margin=approx_maxmin_margin(B,b)
% Bisection with feasible polygon in two variables. Numerical, NOT proof.
if isempty(b),margin=Inf;return;end
scale=max(1,sum(abs(B),2));
B=bsxfun(@rdivide,B,scale);b=b(:)./scale;
low=min(b);high=min(b+sum(abs(B),2));
if ~isfinite(low)||~isfinite(high),margin=NaN;return,end
for k=1:20
    mid=(low+high)/2;
    [~,ok]=halfspace_polygon(B,b,mid);
    if ok,low=mid;else,high=mid;end
end
margin=low;
end

function v=colstage(stages,j)
if isempty(stages),v=[];else,v=stages(j,:).';end
end

function v=localmin(x)
if isempty(x),v=NaN;else,v=min(x(:));end
end

function bad=low_tilt(x)
if isempty(x),bad=false;else,bad=any(x(1:end-1)<-1e-8);end
end

function value=getf(s,key,default)
if isstruct(s)&&isfield(s,key),value=s.(key);else,value=default;end
end

function write_csv(file,headers,rows)
fid=fopen(file,'w','n','UTF-8');
if fid<0,error('CBF4:IO','Failed to open %s',file);end
cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',strjoin(headers,','));
for i=1:size(rows,1)
    s=cell(1,numel(headers));
    for j=1:numel(headers)
        v=rows{i,j};
        if isnumeric(v)||islogical(v)
            if isempty(v)||numel(v)~=1,s{j}='NaN';
            else,s{j}=sprintf('%.17g',double(v));end
        else
            try,v=char(v);catch,v='UNPRINTABLE';end
            v=strrep(v,'"','""');v=strrep(v,sprintf('\r'),' ');
            v=strrep(v,sprintf('\n'),' ');
            s{j}=['"' v '"'];
        end
    end
    fprintf(fid,'%s\n',strjoin(s,','));
end
end

function write_lines(file,lines)
fid=fopen(file,'w','n','UTF-8');
if fid<0,error('CBF4:IO','Failed to open %s',file);end
cleanup=onCleanup(@() fclose(fid));
for i=1:numel(lines),fprintf(fid,'%s\n',char(lines{i}));end
end

function try_clean(folder)
if isfolder(folder),try,rmdir(folder,'s');catch,end,end
end

function out=run_selftest()
B=[1 0;-1 0;0 1;0 -1];b=0.2*ones(4,1);
[p,ok]=halfspace_polygon(B,b,0);
assert(ok&&size(p,1)>=3,'Feasible intersection should exist');
B=[1 0;-1 0];b=[-0.6;-0.6];
[~,ok]=halfspace_polygon(B,b,0);
assert(~ok,'Contradictory constraints should be infeasible');
assert(~isempty(strfind(conflict_witness(B,b,{'left','right'}),'left')));
B=[1 0;-1 0];b=[0.4;0.4];
[~,ok]=halfspace_polygon(B,b,0);
assert(ok,'Feasible strip');
m=approx_maxmin_margin(B,b);
assert(abs(m-0.4)<1e-4,'Max-min margin should be 0.4');
% Verify algebraic decomposition with a synthetic exact telescoping sum.
L=0.3;z0=0.5;zt=0.7;zy=1.0;zn=0.6;pnext=0.65;
assert(abs((pnext-L)-((z0-L)+(zt-z0)+(zy-zt)+(zn-zy)+(pnext-zn)))<1e-12);
% Static positive clearance bound on an axisymmetric obstacle.
a=struct();a.x=[1;zeros(3,1);zeros(3,1);0;0;2;zeros(3,1);0;0;-1;zeros(3,1)];
a.u=[9;0;0;0];a.P=struct('m',1,'mL',0.14,'cableL',2, ...
    'gravity',9.81,'dstx',0,'dsty',0);
a.G=struct('cableCenterFromLoad',0.5,'cableAxisShort',0.2,'cableAxisLong',1.02);
a.parts=struct('center',{[50;0;0]},'kind',{'sphere'},'params',struct('radius',1));
a.dt=0.025;a.info=struct('stageValues',[40;0;0;0;0]);
[lb,ok]=analytic_interval_bound(a,1);
assert(ok&&isfinite(lb)&&lb>0,'Static distant clearance should pass');
out='PASS';
fprintf('[CBF 4WAY] PASS: polygon feasible/infeasible, witness, margin, identity, conditional interval bound.\n');
end
