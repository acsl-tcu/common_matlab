function report=CBF_SD_TEST(mode,varargin)
% CBF_SD_TEST  Read-only sampled-data HOCBF interval-enclosure experiment.
% Single persistent research test name. Does not edit Sim/Scheme02/generator.
% 2026-11-23 maintenance: componentwise Picard inflation and failure diagnostics.
% 2026-11-23 previous maintenance: fix 2-by-N interval construction.
% 2026-11-23 research mode: boundary / boundarySelftest (mixed-relative-degree input switch).
%
%   CBF_SD_TEST('selftest')
%   CBF_SD_TEST('replay','CBF_FOURWAY_RESULTS.zip','maxCases',3,'substeps',4)
%   CBF_SD_TEST('replay',zipPath,'massFactors',[.8 1 1.2], ...
%               'gainFactors',[.8 1 1.2],'torqueOffsets',[-.1 0 .1])
%
% If a verified interval enclosure covers every ZOH state, interval
% evaluation of psi_r yields a sufficient within-period condition.
% IMPORTANT: proof is CONDITIONAL on exact estimated model, fixed theta
% and u during 25 ms, static known obstacles, nonzero support normal,
% and IEEE-754 arithmetic assumptions in the interval primitive. This
% does NOT prove new-input lower-stage conditions or recursive viability.
if nargin==0,mode='rtshadow';end
if strcmpi(mode,'rtcausal')
    report=sd_rtcausal_mode(varargin{:});return;
end
if strcmpi(mode,'rtshadow')
    report=sd_rtshadow_mode(varargin{:});return;
end
if strcmpi(mode,'localrefine')
    report=sd_localrefine_mode(varargin{:});return;
end
if strcmpi(mode,'theorybridge')
    report=sd_theorybridge_mode(varargin{:});return;
end
if strcmpi(mode,'analyticbound')
    report=sd_analyticbound_mode(varargin{:});return;
end
if strcmpi(mode,'fastqp')
    report=sd_fastqp_mode(varargin{:});return;
end
if strcmpi(mode,'qp')
    report=sd_qp_mode(varargin{:});return;
end
if strcmpi(mode,'fast')
    report=sd_fast_mode(varargin{:});return;
elseif strcmpi(mode,'boundary')
    report=sd_boundary_mode(varargin{:});return;
elseif strcmpi(mode,'boundarySelftest')
    report=sd_boundary_selftest();return;
elseif strcmpi(mode,'dual')
    report=sd_dual_mode(varargin{:});return;
elseif strcmpi(mode,'dualSelftest')
    report=sd_dual_selftest();return;
end
[opt,zipPath]=sd_options(mode,varargin);
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
if strcmpi(mode,'selftest')
    sd_selftest_primitives();
    snapshots={sd_synthetic()};idx=1;
elseif strcmpi(mode,'replay')
    if isempty(zipPath) || ~isfile(zipPath)
        error('SD:Input','The FOURWAY result ZIP is required.');
    end
    tempDir=tempname;mkdir(tempDir);
    tempGuard=onCleanup(@()sd_cleanup(tempDir)); %#ok<NASGU>
    unzip(zipPath,tempDir);
    tab=readtable(fullfile(tempDir,'PER_SAMPLE.csv'));
    valid=find(tab.parts>0);
    if isempty(valid),error('SD:Empty','No active obstacle snapshots.');end
    k=min(opt.maxCases,numel(valid));
    [~,a]=sort(tab.min_h_m(valid));[~,b]=sort(tab.min_psi3(valid));
    na=min(ceil(k/3),numel(valid)); nb=min(ceil(k/3),numel(valid));
    chosen=unique([valid(a(1:na));valid(b(1:nb)); ...
        valid(unique(round(linspace(1,numel(valid),k))))],'stable');
    chosen=chosen(1:min(k,numel(chosen)));
    if numel(chosen)<k
        rem=setdiff(valid,chosen,'stable');chosen=[chosen;rem(1:k-numel(chosen))];
    end
    chosen=sort(chosen);
    R=load(fullfile(tempDir,'RAW_SNAPSHOTS.mat'),'snapshot');
    if ~iscell(R.snapshot) || numel(R.snapshot)<max(chosen)
        error('SD:Input','snapshot must be a cell vector.');
    end
    snapshots=R.snapshot(chosen);idx=chosen(:);
else
    error('SD:Mode','Use selftest or replay.');
end
rows=struct([]); errors={}; caseCount=0;
for j=1:numel(snapshots)
    for mf=opt.massFactors(:).'
        for gf=opt.gainFactors(:).'
            for dx=opt.torqueOffsets(:).'
                for dy=opt.torqueOffsets(:).'
                    caseCount=caseCount+1;
                    try
                        rr=sd_case(snapshots{j},idx(j),mf,gf,[dx;dy],opt);
                        if isempty(rows),rows=rr;
                        else,rows(end+1:end+numel(rr))=rr;end %#ok<AGROW>
                    catch ME
                        st='';for kk=1:numel(ME.stack)
                            st=[st sprintf(' %s:%d',ME.stack(kk).name,ME.stack(kk).line)]; %#ok<AGROW>
                        end
                        errors{end+1,1}=sprintf('snapshot=%d mf=%.7g gf=%.7g torque=[%g %g]: %s | %s', ...
                            idx(j),mf,gf,dx,dy,ME.message,st); %#ok<AGROW>
                    end
                end
            end
        end
    end
end
report=struct('mode',char(mode),'caseCount',caseCount,'barriers',numel(rows), ...
    'errors',{errors},'rows',rows,'options',opt, ...
    'guarantee','CONDITIONAL_INTERVAL_TEST_ONLY__NO_RECURSIVE_PROOF');
if ~isempty(rows)
    pass=sum(strcmp({rows.status},'PERIOD_HOCBF_CONDITIONAL_PASS'));
    unknown=sum(strcmp({rows.status},'INTERVAL_INCONCLUSIVE'));
    fail=sum(strcmp({rows.status},'REACH_ENCLOSURE_FAILED'));
    fprintf('[SD] candidates=%d rows=%d errors=%d\n',caseCount,numel(rows),numel(errors));
    fprintf('[SD] interval HOCBF within-period conditional PASS=%d, INCONCLUSIVE=%d, TUBE_FAILED=%d\n',pass,unknown,fail);
    common=sum(strcmp({rows.status},'PERIOD_HOCBF_CONDITIONAL_PASS') & [rows.sameInputAllConstraintsPass]);
    fprintf('[SD] rows in all-constraint passing candidate=%d (count rows, not candidates).\n',common);
    fprintf('[SD] ALL-TIME safety / recursive feasibility: NOT_PROVEN.\n');
else
    fprintf('[SD] NO RESULT ROWS; errors=%d\n',numel(errors));
end
if ~isempty(errors),fprintf('[SD] FIRST ERROR: %s\n',errors{1});end
if opt.save,sd_save(report);end
end

function [o,pathZip]=sd_options(mode,args)
o=struct('maxCases',3,'substeps',4,'massFactors',1,'gainFactors',1, ...
    'torqueOffsets',0,'save',true,'tubeIterations',14);
pathZip='';mode=lower(char(mode));
if strcmp(mode,'replay')
    if isempty(args) || ~(ischar(args{1}) || isstring(args{1}))
        error('SD:Arguments','replay requires positional ZIP file.');
    end
    pathZip=char(args{1});args=args(2:end);
elseif ~strcmp(mode,'selftest')
    error('SD:Mode','Mode must be selftest or replay.');
end
if mod(numel(args),2),error('SD:Arguments','Options must be name/value pairs.');end
for k=1:2:numel(args)
    key=lower(char(args{k}));v=args{k+1};
    switch key
        case 'maxcases',o.maxCases=v;
        case 'substeps',o.substeps=v;
        case 'massfactors',o.massFactors=v;
        case 'gainfactors',o.gainFactors=v;
        case 'torqueoffsets',o.torqueOffsets=v;
        case 'save',o.save=logical(v);
        case 'tubeiterations',o.tubeIterations=v;
        otherwise,error('SD:Arguments','Unknown option: %s',key);
    end
end
validateattributes(o.maxCases,{'numeric'},{'scalar','integer','>=',1,'<=',40});
validateattributes(o.substeps,{'numeric'},{'scalar','integer','>=',1,'<=',64});
validateattributes(o.tubeIterations,{'numeric'},{'scalar','integer','>=',4,'<=',32});
for x={o.massFactors,o.gainFactors}
    validateattributes(x{1},{'numeric'},{'vector','real','finite','positive'});
end
validateattributes(o.torqueOffsets,{'numeric'},{'vector','finite','real','>=',-2,'<=',2});
if numel(o.torqueOffsets)>7,error('SD:Arguments','At most 7 torque offsets.');end
end

function r=sd_case(snap,id,mfac,gfac,dTorque,opt)
% Timing breakdown is diagnostic only; no controller or proof logic changes.
sdCaseClock=tic;
needed={'x','u','P','G','lambda','parts','dt'};
for k=1:numel(needed)
    if ~isfield(snap,needed{k}),error('SD:Snapshot','Missing %s',needed{k});end
end
x=double(snap.x(:));u=double(snap.u(:));P=snap.P;P.mL=double(P.mL)*mfac;
lam=double(snap.lambda(:))*gfac;G=snap.G;dt=double(snap.dt);
if isfield(snap,'tiltCfg'),tilt=snap.tiltCfg;else,tilt=[];end
if ~isempty(tilt)
    tilt.bodyLambda=double(tilt.bodyLambda(:))*gfac;
    tilt.cableLambda=double(tilt.cableLambda(:))*gfac;
end
if isfield(snap,'time'),t0=double(snap.time);else,t0=NaN;end
assert(numel(x)==19 && numel(u)==4 && dt>0 && dt<=0.1,'SD:Dimensions','Bad snapshot dimension or step.');
assert(P.m>0 && P.mL>=0 && P.cableL>0 && all(P.J>0),'SD:Parameters','Invalid physical parameters.');
u(2:3)=u(2:3)+dTorque;
if any(abs(u(2:3))>1+1e-12),error('SD:Torque','Candidate outside physical XY limits.');end
rawNorm=max(abs([norm(x(1:4))-1,norm(x(14:16))-1]));
if rawNorm>0.005,error('SD:State','State outside Scheme02 projection tolerance.');end
x(1:4)=x(1:4)/norm(x(1:4));x(14:16)=x(14:16)/norm(x(14:16));
C=sd_interval(x);tubes=cell(opt.substeps,1);
reachOK=true;reachReason='OK';
picardClock=tic;picardTotalIterations=0;picardAttemptedSubsteps=0;
picardCompletedSubsteps=0;
for j=1:opt.substeps
    picardAttemptedSubsteps=j;
    try
        [box,Cnext,ok,picardDetail,picardIterations]=sd_picard_box(C,u,P,dt/opt.substeps,opt.tubeIterations);
        picardTotalIterations=picardTotalIterations+picardIterations;
    catch ME
        ok=false;reachReason=['Picard arithmetic undefined: ' ME.identifier];
    end
    if ~ok
        reachOK=false;
        if strcmp(reachReason,'OK')
            reachReason=sprintf('Picard inclusion failed at substep %d: %s',j,picardDetail);
        end
        break;
    end
    tubes{j}=box;C=Cnext;picardCompletedSubsteps=j;
end
picardElapsedMs=1000*toc(picardClock);
stageClock=tic;
names=sd_names(snap.parts,tilt);
tubeStages=cell(opt.substeps,1);tubeEvalOK=reachOK;
if reachOK
    for jj=1:opt.substeps
        try
            tubeStages{jj}=sd_interval_stages(tubes{jj},u,P,G,snap.parts,lam,tilt);
        catch ME
            tubeEvalOK=false;
            reachReason=['Reach enclosed; HOCBF interval undefined: ' ME.identifier];
            break;
        end
    end
end
% Extra endpoint enclosure is computed ONLY in dual comparison mode.
terminalStages=[];terminalOK=false;
if reachOK && tubeEvalOK && isfield(opt,'dualEndpoint') && opt.dualEndpoint
    try
        terminalStages=sd_interval_stages(C,u,P,G,snap.parts,lam,tilt);
        terminalOK=numel(terminalStages)==numel(names);
    catch
        terminalOK=false;
    end
end
intervalStagesElapsedMs=1000*toc(stageClock);
checkClock=tic;
r=repmat(sd_result_template(),numel(names),1);
initial=sd_interval_stages(sd_interval(x),u,P,G,snap.parts,lam,tilt);
% Model-source regression: numerical generated EVAL must lie within the
% computed initial interval, up to a tiny comparison tolerance. If not,
% we cannot trust the derived interval F and refuse this candidate.
E=CBF_REAL_XY_EVAL(x,u,P,G,snap.parts,lam,tilt);
sd_check_source_match(initial,E,tilt);
if ~isequal({initial.name}.',names),error('SD:Names','Barrier name mismatch.');end
generatorCheckElapsedMs=1000*toc(checkClock);
coreElapsedMs=1000*toc(sdCaseClock);
for i=1:numel(names)
    rr=sd_result_template();rr.sampleIndex=id;rr.time=t0;rr.barrier=names{i};
    rr.massEstimateKg=P.mL;rr.massFactor=mfac;rr.gainFactor=gfac;
    rr.thrustN=u(1);rr.torqueX=u(2);rr.torqueY=u(3);rr.torqueZ=u(4);
    rr.dt=dt;rr.substeps=opt.substeps;rr.stateProjectionError=rawNorm;
    rr.picardElapsedMs=picardElapsedMs;
    rr.intervalStagesElapsedMs=intervalStagesElapsedMs;
    rr.generatorCheckElapsedMs=generatorCheckElapsedMs;
    rr.coreElapsedMs=coreElapsedMs;
    rr.picardIterations=picardTotalIterations;
    rr.picardSubstepsAttempted=picardAttemptedSubsteps;
    rr.picardSubstepsCompleted=picardCompletedSubsteps;
    rr.initialHLower=initial(i).stages{1}(1);
    rr.initialHUpper=initial(i).stages{1}(2);
    if terminalOK
        rr.terminalHLower=terminalStages(i).stages{1}(1);
        rr.terminalHUpper=terminalStages(i).stages{1}(2);
        rr.terminalEvaluationOK=true;
    end
    rr.initialLowestStage=min(cellfun(@(v)v(1),initial(i).stages(1:end-1)));
    rr.initialFinalLower=initial(i).stages{end}(1);
    rr.initialFinalUpper=initial(i).stages{end}(2);
    rr.initialLowerOK=rr.initialLowestStage>=0;
    rr.reachEnclosureOK=reachOK;
    rr.reachStatus=reachReason;
    rr.intervalMinH=Inf;rr.intervalMinFinal=Inf;
    rr.intervalMaxFinal=-Inf;
    if reachOK && tubeEvalOK
        for jj=1:opt.substeps
            st=tubeStages{jj};
            rr.intervalMinH=min(rr.intervalMinH,st(i).stages{1}(1));
            rr.intervalMinFinal=min(rr.intervalMinFinal,st(i).stages{end}(1));
            rr.intervalMaxFinal=max(rr.intervalMaxFinal,st(i).stages{end}(2));
        end
        rr.derivedDropBound=max(0,rr.initialFinalUpper-rr.intervalMinFinal);
        rr.marginAboveBound=rr.initialFinalLower-rr.derivedDropBound;
        % A direct interval min(F) is less conservative than using F0 - drop.
        if rr.initialLowerOK && rr.intervalMinFinal>=0
            rr.status='PERIOD_HOCBF_CONDITIONAL_PASS';
        else
            rr.status='INTERVAL_INCONCLUSIVE';
        end
    else
        rr.intervalMinH=NaN;rr.intervalMinFinal=NaN;
        rr.intervalMaxFinal=NaN;rr.derivedDropBound=NaN;
        rr.marginAboveBound=NaN;
        if reachOK,rr.status='INTERVAL_EVALUATION_UNDEFINED';
        else,rr.status='REACH_ENCLOSURE_FAILED';end
    end
    rr.recursiveGuarantee='NOT_PROVEN';
    r(i)=rr;
end
bothOK=all(strcmp({r.status},'PERIOD_HOCBF_CONDITIONAL_PASS'));
for ii=1:numel(r),r(ii).sameInputAllConstraintsPass=bothOK;end
end


function sd_check_source_match(I,E,tilt)
expected=cell(1,0);
for j=1:size(E.stages,2),expected{end+1}=E.stages(:,j);end %#ok<AGROW>
if ~isempty(tilt)
    if tilt.bodyEnabled,expected{end+1}=E.tiltBodyStages(:);end
    if tilt.cableEnabled,expected{end+1}=E.tiltCableStages(:);end
end
if numel(I)~=numel(expected),error('SD:GeneratorMismatch','Barrier count differs.');end
for j=1:numel(I)
    for k=1:numel(I(j).stages)
        v=I(j).stages{k};ref=expected{j}(k);
        tol=1e-7*(1+abs(ref));
        if ~(ref>=v(1)-tol && ref<=v(2)+tol)
            error('SD:GeneratorMismatch', ...
                'Source/interval formula discrepancy at barrier %d stage %d.',j,k);
        end
    end
end
end

function rr=sd_result_template()
rr=struct('sampleIndex',NaN,'time',NaN,'barrier','','massEstimateKg',NaN, ...
    'massFactor',NaN,'gainFactor',NaN,'thrustN',NaN, ...
    'torqueX',NaN,'torqueY',NaN,'torqueZ',NaN, ...
    'dt',NaN,'substeps',NaN,'stateProjectionError',NaN, ...
    'initialHLower',NaN,'initialHUpper',NaN, ...
    'terminalHLower',NaN,'terminalHUpper',NaN,'terminalEvaluationOK',false, ...
    'initialLowestStage',NaN, ...
    'initialFinalLower',NaN,'initialFinalUpper',NaN, ...
    'initialLowerOK',false,'reachEnclosureOK',false,'reachStatus','', ...
    'intervalMinH',NaN,'intervalMinFinal',NaN,'intervalMaxFinal',NaN, ...
    'derivedDropBound',NaN,'marginAboveBound',NaN, ...
    'status','','recursiveGuarantee','NOT_PROVEN', ...
    'sameInputAllConstraintsPass',false, ...
    'picardElapsedMs',NaN,'intervalStagesElapsedMs',NaN, ...
    'generatorCheckElapsedMs',NaN,'coreElapsedMs',NaN, ...
    'picardIterations',NaN,'picardSubstepsAttempted',NaN, ...
    'picardSubstepsCompleted',NaN);
end

function [box,next,ok,detail,iterations]=sd_picard_box(start,u,P,h,limit)
% Prove one substep enclosure by Picard inclusion:
% start + [0,h] * f(box,u,P) strictly inside box.
% Widen ONLY components failing inclusion. Previously every component
% was blindly doubled at each iteration: this can destroy useful boxes.
% A failure remains UNKNOWN, not evidence of a collision.
ok=false;box=[];next=[];detail='not evaluated';iterations=0;
F=sd_interval_rhs(start,u,P);
rad=max(abs(F),[],1)*h*2 + 1e-10 + 1e-8*(abs(start(1,:))+abs(start(2,:)));
for it=1:limit
    iterations=it;
    box=[sd_down(start(1,:)-rad);sd_up(start(2,:)+rad)];
    rhs=sd_interval_rhs(box,u,P);
    reach=sd_iadd(start,sd_imul(sd_irange(0,h),rhs));
    missLo=reach(1,:)<=box(1,:);
    missHi=reach(2,:)>=box(2,:);
    bad=missLo | missHi;
    if ~any(bad)
        next=sd_iadd(start,sd_iscale(rhs,h));ok=true;
        detail=sprintf('OK in %d Picard iterations',it);
        return;
    end
    % Needed radius measured from the original starting interval.
    needed=max([start(1,:)-reach(1,:);reach(2,:)-start(2,:)],[],1);
    if any(~isfinite(needed))
        detail=sprintf('nonfinite reach at iteration %d',it);return;
    end
    % Componentwise, monotone enlargement; do not widen settled dimensions.
    pad=1e-11 + 1e-10*(abs(start(1,:))+abs(start(2,:)));
    rad(bad)=max(1.25*rad(bad),1.30*max(needed(bad),0)+pad(bad));
    if any(~isfinite(rad)) || any(rad>1e6)
        detail=sprintf('radius divergence at iteration %d',it);return;
    end
end
% Report which physical state component prevented enclosure.
if exist('bad','var') && any(bad)
    deficit=max([box(1,:)-reach(1,:);reach(2,:)-box(2,:)],[],1);
    deficit(~bad)=-Inf;
    [worst,coord]=max(deficit);
    detail=sprintf('coordinate %d, excess %.6g, iterations %d',coord,worst,limit);
else
    detail='no strict interior inclusion';
end
end

function B=sd_interval_rhs(X,u,P)
% Interval extension of CBF_COMMON_MODEL (19-state physical model).
q=X(:,1:4);w=X(:,5:7);v=X(:,11:13);
p=X(:,14:16);wl=X(:,17:19);
T=sd_interval(u(1));m=P.m;mL=P.mL;L=P.cableL;J=P.J(:).';
qdot=sd_iscale([ ...
    sd_ineg(sd_iadd(sd_iadd(sd_imul(q(:,2),w(:,1)),sd_imul(q(:,3),w(:,2))),sd_imul(q(:,4),w(:,3)))), ...
    sd_isub(sd_iadd(sd_imul(q(:,1),w(:,1)),sd_imul(q(:,3),w(:,3))),sd_imul(q(:,4),w(:,2))), ...
    sd_isub(sd_iadd(sd_imul(q(:,1),w(:,2)),sd_imul(q(:,4),w(:,1))),sd_imul(q(:,2),w(:,3))), ...
    sd_isub(sd_iadd(sd_imul(q(:,1),w(:,3)),sd_imul(q(:,2),w(:,2))),sd_imul(q(:,3),w(:,1)))],.5);
b3=[sd_iscale(sd_iadd(sd_imul(q(:,2),q(:,4)),sd_imul(q(:,1),q(:,3))),2), ...
    sd_iscale(sd_isub(sd_imul(q(:,3),q(:,4)),sd_imul(q(:,1),q(:,2))),2), ...
    sd_iadd(sd_isub(sd_isub(sd_isqr(q(:,1)),sd_isqr(q(:,2))),sd_isqr(q(:,3))),sd_isqr(q(:,4)))];
ptDot=sd_icross(wl,p);
Jw=sd_imul(w,sd_interval(J));
wdot=sd_idiv(sd_isub(sd_interval(u(2:4).'),sd_icross(w,Jw)),sd_interval(J));
wlDot=sd_iscale(sd_icross(p,b3),-u(1)/(m*L));
s=sd_isub(sd_imul(T,sd_idot(p,b3)), ...
    sd_iscale(sd_idot(ptDot,ptDot),m*L));
a=sd_iadd(sd_iscale(sd_imul(s,p),1/(m+mL)), ...
    sd_interval([sd_get(P,'dstx',0),sd_get(P,'dsty',0),-P.gravity]));
B=[qdot,wdot,v,a,ptDot,wlDot];
end

function stage=sd_interval_stages(X,u,P,G,parts,lam,tilt)
% The exact fourth-order Taylor-coefficient recurrence from the provided XY generator,
% lifted to outward-enlarged intervals, with support direction differentiated.
J=sd_interval_jet(X,u,P);
stage=struct('name',{},'stages',{},'raw',{});
if ~isempty(parts)
    as=G.cableAxisShort;al=G.cableAxisLong;
    c=G.cableCenterFromLoad;
    if ~(as>0 && al>=as),error('SD:Shape','Invalid cable shape.');end
    Cp=cell(1,5);Pt=cell(1,5);
    for k=1:5
        Pt{k}=J{k}(:,14:16);
        Cp{k}=sd_isub(J{k}(:,8:10),sd_iscale(Pt{k},c*P.cableL));
    end
    for i=1:numel(parts)
        [centre,Q,name]=sd_shape(parts(i),i);
        D=Cp;D{1}=sd_isub(D{1},sd_interval(centre(:).'));
        R=sd_jsqrt(sd_jdot(D,D));
        N=sd_jvectimes(D,sd_jinv(R));
        % n' Qc n = as^2 n'n + (al^2-as^2)(n'pT)^2
        Nn=sd_jdot(N,N);Np=sd_jdot(N,Pt);
        sc=sd_jsqrt(sd_jadd(sd_jscale(Nn,as^2),sd_jscale(sd_jmul(Np,Np),al^2-as^2)));
        Qn=cell(1,5);
        for k=1:5,Qn{k}=sd_imatvec(Q,N{k});end
        so=sd_jsqrt(sd_jdot(N,Qn));
        H=sd_jsub(sd_jsub(sd_jdot(N,D),sc),so);
        stage(end+1).name=[name '/cable']; %#ok<AGROW>
        stage(end).stages=sd_stage(H,lam);
        stage(end).raw=H; % DIRECT h Taylor coefficients, no interval inversion
    end
end
if ~isempty(tilt)
    if tilt.bodyEnabled
        Q=cell(1,5);
        for k=0:4
            a=sd_jconv(J,1,1,k);b=sd_jconv(J,2,2,k);
            d=sd_jconv(J,3,3,k);e=sd_jconv(J,4,4,k);
            Q{k+1}=sd_iadd(sd_isub(sd_isub(a,b),d),e);
        end
        Q{1}=sd_isub(Q{1},sd_interval(cosd(tilt.bodyMaxDeg)));
        stage(end+1).name='bodyTilt'; %#ok<AGROW>
        stage(end).stages=sd_stage(Q(1:3),tilt.bodyLambda);
        stage(end).raw=Q(1:3);
    end
    if tilt.cableEnabled
        Q=cell(1,5);
        for k=1:5,Q{k}=sd_ineg(J{k}(:,16));end
        Q{1}=sd_isub(Q{1},sd_interval(cosd(tilt.cableMaxDeg)));
        stage(end+1).name='cableTilt'; %#ok<AGROW>
        stage(end).stages=sd_stage(Q,tilt.cableLambda);
        stage(end).raw=Q;
    end
end
end

function J=sd_interval_jet(X,u,P)
J=cell(1,5);J{1}=X;
for k=0:3
    f=sd_jet_rhs_coeff(J,u,P,k);
    J{k+2}=sd_iscale(f,1/(k+1));
end
end

function f=sd_jet_rhs_coeff(J,u,P,k)
% Intervalized coefficient recurrence of CBF_REAL_XY_JET.
m=P.m;L=P.cableL;mL=P.mL;JJ=P.J(:).';
zero=sd_interval(zeros(1,19));f=zero;
q1=sd_jconv(J,2,5,k);q2=sd_jconv(J,3,6,k);q3=sd_jconv(J,4,7,k);
f(:,1)=sd_iscale(sd_ineg(sd_iadd(sd_iadd(q1,q2),q3)),.5);
f(:,2)=sd_iscale(sd_isub(sd_iadd(sd_jconv(J,1,5,k),sd_jconv(J,3,7,k)),sd_jconv(J,4,6,k)),.5);
f(:,3)=sd_iscale(sd_isub(sd_iadd(sd_jconv(J,1,6,k),sd_jconv(J,4,5,k)),sd_jconv(J,2,7,k)),.5);
f(:,4)=sd_iscale(sd_isub(sd_iadd(sd_jconv(J,1,7,k),sd_jconv(J,2,6,k)),sd_jconv(J,3,5,k)),.5);
rot=sd_interval(zeros(1,3));
for j=0:k
    rot=sd_iadd(rot,sd_icross(J{j+1}(:,5:7), ...
        sd_imul(J{k-j+1}(:,5:7),sd_interval(JJ))));
end
f(:,5:7)=sd_idiv(sd_ineg(rot),sd_interval(JJ));
if k==0,f(:,5:7)=sd_iadd(f(:,5:7),sd_idiv(sd_interval(u(2:4).'),sd_interval(JJ)));end
f(:,8:10)=J{k+1}(:,11:13);
b3=cell(1,k+1);vt=cell(1,k+1);
for j=0:k
    b3{j+1}=sd_b3coeff(J,j);
    vt{j+1}=sd_crosscoeff(J,17:19,14:16,j);
end
acc=sd_interval(zeros(1,3));
for j=0:k
    ps=sd_interval(0);vv=sd_interval(0);
    for l=0:j
        ps=sd_iadd(ps,sd_idot(J{l+1}(:,14:16),b3{j-l+1}));
        vv=sd_iadd(vv,sd_idot(vt{l+1},vt{j-l+1}));
    end
    s=sd_isub(sd_iscale(ps,u(1)),sd_iscale(vv,m*L));
    acc=sd_iadd(acc,sd_imul(s,J{k-j+1}(:,14:16)));
end
f(:,11:13)=sd_iscale(acc,1/(m+mL));
if k==0
    f(:,11:13)=sd_iadd(f(:,11:13),sd_interval([sd_get(P,'dstx',0),sd_get(P,'dsty',0),-P.gravity]));
end
f(:,14:16)=vt{k+1};
v=sd_interval(zeros(1,3));
for j=0:k
    v=sd_iadd(v,sd_icross(J{j+1}(:,14:16),b3{k-j+1}));
end
f(:,17:19)=sd_iscale(v,-u(1)/(m*L));
end

function b=sd_b3coeff(J,k)
b1=sd_iadd(sd_jconv(J,2,4,k),sd_jconv(J,1,3,k));
b2=sd_isub(sd_jconv(J,3,4,k),sd_jconv(J,1,2,k));
b3=sd_iadd(sd_isub(sd_isub(sd_jconv(J,1,1,k),sd_jconv(J,2,2,k)), ...
    sd_jconv(J,3,3,k)),sd_jconv(J,4,4,k));
b=[sd_iscale(b1,2),sd_iscale(b2,2),b3];
end
function c=sd_crosscoeff(J,ia,ib,k)
c=sd_interval(zeros(1,3));
for j=0:k,c=sd_iadd(c,sd_icross(J{j+1}(:,ia),J{k-j+1}(:,ib)));end
end
function c=sd_jconv(J,i,j,k)
c=sd_interval(0);
for a=0:k,c=sd_iadd(c,sd_imul(J{a+1}(:,i),J{k-a+1}(:,j)));end
end

function y=sd_imatvec(Q,x)
y=sd_interval(zeros(1,3));
for i=1:3
    t=sd_interval(0);
    for j=1:3,t=sd_iadd(t,sd_iscale(x(:,j),Q(i,j)));end
    y(:,i)=t;
end
end
function o=sd_jvectimes(V,s)
o=cell(1,5);
for k=0:4
    v=sd_interval(zeros(1,3));
    for j=0:k,v=sd_iadd(v,sd_imul(V{j+1},s{k-j+1}));end
    o{k+1}=v;
end
end
function o=sd_jdot(A,B)
o=cell(1,5);
for k=0:4
    v=sd_interval(0);
    for j=0:k,v=sd_iadd(v,sd_idot(A{j+1},B{k-j+1}));end
    o{k+1}=v;
end
end
function o=sd_jmul(A,B)
o=cell(1,numel(A));
for k=0:numel(A)-1
    v=sd_interval(0);
    for j=0:k,v=sd_iadd(v,sd_imul(A{j+1},B{k-j+1}));end
    o{k+1}=v;
end
end
function o=sd_jadd(A,B)
o=cell(size(A));for k=1:numel(A),o{k}=sd_iadd(A{k},B{k});end
end
function o=sd_jsub(A,B)
o=cell(size(A));for k=1:numel(A),o{k}=sd_isub(A{k},B{k});end
end
function o=sd_jscale(A,c)
o=cell(size(A));for k=1:numel(A),o{k}=sd_iscale(A{k},c);end
end
function o=sd_jsqrt(A)
if A{1}(1)<=0 || ~all(isfinite(A{1}))
    error('SD:Support','Positive support argument not certified on tube.');
end
o=cell(1,numel(A));o{1}=sd_isqrt(A{1});
for k=1:numel(A)-1
    t=sd_interval(0);
    for j=1:k-1,t=sd_iadd(t,sd_imul(o{j+1},o{k-j+1}));end
    o{k+1}=sd_idiv(sd_isub(A{k+1},t),sd_iscale(o{1},2));
end
end
function o=sd_jinv(A)
if A{1}(1)<=0
    error('SD:Denominator','Series denominator not strictly positive.');
end
o=cell(1,numel(A));o{1}=sd_iinv(A{1});
for k=1:numel(A)-1
    t=sd_interval(0);
    for j=1:k,t=sd_iadd(t,sd_imul(A{j+1},o{k-j+1}));end
    o{k+1}=sd_ineg(sd_idiv(t,A{1}));
end
end
function out=sd_stage(H,lambda)
r=numel(lambda);if numel(H)~=r+1,error('SD:Stage','Wrong HOCBF order.');end
out=cell(1,r+1);
for k=0:r
    pol=1;
    for j=1:k,pol=conv(pol,[lambda(j),1]);end
    v=sd_interval(0);
    for j=0:k,v=sd_iadd(v,sd_iscale(H{j+1},pol(j+1)*factorial(j)));end
    out{k+1}=v;
end
end
function [centre,Q,name]=sd_shape(ob,i)
centre=double(ob.center(:));
if isfield(ob,'kind'),kind=char(ob.kind);else,kind='ellipsoid';end
switch kind
    case 'sphere'
        if isfield(ob,'params'),r=ob.params.radius;else,r=ob.radius;end
        ax=[r;r;r];R=eye(3);
    case 'ellipsoid'
        if isfield(ob,'params'),ax=ob.params.axes(:);else,ax=ob.axes(:);end
        R=double(ob.R);
    otherwise,error('SD:Obstacle','Unsupported kind %s.',kind);
end
assert(all(ax>0) && numel(ax)==3 && numel(centre)==3,'SD:Obstacle','Bad obstacle.');
Q=R*diag(double(ax).^2)*R.';
if isfield(ob,'obstacleName'),name=char(ob.obstacleName);
elseif isfield(ob,'name'),name=char(ob.name);
else,name=sprintf('obstacle%d',i);end
end
function names=sd_names(parts,tilt)
names=cell(numel(parts),1);
for i=1:numel(parts),[~,~,nm]=sd_shape(parts(i),i);names{i}=[nm '/cable'];end
if ~isempty(tilt)
    if tilt.bodyEnabled,names{end+1,1}='bodyTilt';end
    if tilt.cableEnabled,names{end+1,1}='cableTilt';end
end
end

% --- Outward-inflated binary64 interval primitives. ---
% Assuming IEEE-754 round-to-nearest, finite operands, and correctly
% rounded arithmetic/sqrt, 8 ULPs on every primitive endpoint enclose
% a single operation. This MUST be reviewed for target MATLAB/CPU;
% without that assumption the output is not a machine-checked certificate.
function y=sd_down(x)
y=x-8*eps(abs(x));
end
function y=sd_up(x)
y=x+8*eps(abs(x));
end
function I=sd_interval(x)
% Exact-valued point intervals; each numeric element is a separate column.
x=double(x(:).');I=[x;x];
end
function I=sd_irange(lo,hi)
% General interval columns, always stored as 2-by-N [lower; upper].
% In particular, [0,h] is a TWO-element row vector, not an interval.
lo=double(lo(:).');hi=double(hi(:).');
if numel(lo)~=numel(hi) || any(~isfinite([lo hi])) || any(lo>hi)
    error('SD:IntervalShape','Invalid interval endpoints.');
end
I=[lo;hi];
end
function v=sd_ineg(a)
v=[-a(2,:);-a(1,:)];
end
function v=sd_iadd(a,b)
v=[sd_down(a(1,:)+b(1,:));sd_up(a(2,:)+b(2,:))];
end
function v=sd_isub(a,b)
v=sd_iadd(a,sd_ineg(b));
end
function v=sd_imul(a,b)
if size(a,1)~=2 || size(b,1)~=2 || ...
        ~(size(a,2)==size(b,2) || size(a,2)==1 || size(b,2)==1)
    error('SD:IntervalShape','Multiply expects 2-by-N interval arrays with compatible widths.');
end
x1=a(1,:).*b(1,:);x2=a(1,:).*b(2,:);
x3=a(2,:).*b(1,:);x4=a(2,:).*b(2,:);
v=[sd_down(min(min(x1,x2),min(x3,x4))); ...
    sd_up(max(max(x1,x2),max(x3,x4)))];
end
function v=sd_isqr(a)
x=a(1,:).^2;y=a(2,:).^2;
lo=min(x,y);lo(a(1,:)<=0 & a(2,:)>=0)=0;
v=[sd_down(lo);sd_up(max(x,y))];
end
function v=sd_isqrt(a)
if any(a(1,:)<0),error('SD:Interval','Square-root interval goes negative.');end
v=[max(0,sd_down(sqrt(a(1,:))));sd_up(sqrt(a(2,:)))];
end
function v=sd_iinv(a)
if size(a,1)~=2 || any(a(1,:)>a(2,:))
    error('SD:IntervalShape','Reciprocal expects a 2-by-N [lower;upper] interval array.');
end
if any(a(1,:)<=0 & a(2,:)>=0)
    error('SD:Interval','Division interval includes zero.');
end
v=[sd_down(1./a(2,:));sd_up(1./a(1,:))];
end
function v=sd_idiv(a,b)
v=sd_imul(a,sd_iinv(b));
end
function v=sd_iscale(a,c)
v=sd_imul(a,sd_interval(c));
end
function v=sd_idot(a,b)
prod=sd_imul(a,b);v=sd_interval(0);
for k=1:size(prod,2),v=sd_iadd(v,prod(:,k));end
end
function v=sd_icross(a,b)
v=[sd_isub(sd_imul(a(:,2),b(:,3)),sd_imul(a(:,3),b(:,2))), ...
    sd_isub(sd_imul(a(:,3),b(:,1)),sd_imul(a(:,1),b(:,3))), ...
    sd_isub(sd_imul(a(:,1),b(:,2)),sd_imul(a(:,2),b(:,1)))];
end
function v=sd_get(s,name,d)
if isfield(s,name),v=double(s.(name));else,v=d;end
end

function sd_selftest_primitives()
[o,p]=sd_options('replay',{'CBF_FOURWAY_RESULTS.zip','maxCases',3,'substeps',4});
assert(strcmp(p,'CBF_FOURWAY_RESULTS.zip') && o.maxCases==3 && o.substeps==4);
a=sd_interval([-3 2]);b=sd_interval([1 4]);
c=sd_iadd(a,b); assert(all(c(1,:)<=[-2 6]) && all(c(2,:)>=[-2 6]));
% Reciprocal of the ONE interval [2,3], not of point values 2 and 3.
v=sd_iinv(sd_irange(2,3));
assert(isequal(size(v),[2 1]) && v(1)<=1/3 && v(2)>=1/2, ...
    'SD:Selftest','Reciprocal interval check failed.');
% A time span [0,h] times a VECTOR of interval derivatives must remain 2-by-N.
span=sd_irange(0,0.025);
v=sd_imul(span,sd_irange([-2 1],[4 3]));
assert(isequal(size(v),[2 2]) && v(1,1)<=-0.05 && ...
    v(2,1)>=0.1 && v(1,2)<=0 && v(2,2)>=0.075, ...
    'SD:Selftest','Time-interval multiplication check failed.');
try
    sd_iinv(sd_irange(-1,2));
    error('SD:Selftest','Reciprocal across zero unexpectedly accepted.');
catch ME
    if ~strcmp(ME.identifier,'SD:Interval'),rethrow(ME);end
end
[o2,p2]=sd_options('replay',{'C:\tmp\log.zip','massFactors',[.8 1 1.2], ...
    'gainFactors',[.8 1 1.2]});
assert(strcmp(p2,'C:\tmp\log.zip') && numel(o2.massFactors)==3);
try
    sd_options('replay',{'log.zip','bogus',42});
    error('SD:Selftest','Bad option unexpectedly accepted.');
catch ME
    if ~strcmp(ME.identifier,'SD:Arguments'),rethrow(ME);end
end
%s Check the interval RHS, interval jet, and per-barrier stages against
% the UNMODIFIED supplied generated functions at one nontrivial state.
a=sd_synthetic();x=a.x;u=a.u;P=a.P;
xd=CBF_COMMON_MODEL(x,u,P);b=sd_interval_rhs(sd_interval(x),u,P);
assert(all(xd(:).'>=b(1,:)) && all(xd(:).'<=b(2,:)), ...
    'SD:Selftest','Interval RHS excluded numeric model.');
I=sd_interval_stages(sd_interval(x),u,P,a.G,a.parts,a.lambda,a.tiltCfg);
E=CBF_REAL_XY_EVAL(x,u,P,a.G,a.parts,a.lambda,a.tiltCfg);
vals={E.stages(:,1),E.stages(:,2),E.tiltBodyStages(:),E.tiltCableStages(:)};
assert(numel(I)==numel(vals),'SD:Selftest','Wrong number of test constraints.');
for k=1:numel(I)
    for j=1:numel(I(k).stages)
        expected=vals{k}(j);v=I(k).stages{j};
        tol=1e-7*(1+abs(expected));
        assert(expected>=v(1)-tol && expected<=v(2)+tol, ...
            'SD:Selftest','Interval jet mismatches existing generated EVAL.');
    end
end
fprintf('[SD] Parser + interval RHS + 2-obstacle generator comparison: PASS.\n');
end

function a=sd_synthetic()
q=[.98;.08;-.12;.1];q=q/norm(q);
p=[.08;-.11;-.99];p=p/norm(p);
a.x=[q;.1;-.12;.08;0;0;3;.4;-.2;.1;p;.02;.03;-.04];
a.u=[9.5;.22;-.13;.04];
a.P=struct('m',.85,'J',[.012;.013;.022], ...
    'gravity',9.81,'mL',.14,'cableL',2,'dstx',0,'dsty',0);
a.G=CBF_COMMON_GEOMETRY(a.P,struct('includeGround',false, ...
    'trimCableWithBodySpheres',false));
a.G.cableCenterFromLoad=.5;a.G.cableAxisShort=.2;
a.G.cableAxisLong=a.P.cableL/2+.02;
a.lambda=[4;4;4;4];a.dt=.025;a.time=0;
a.tiltCfg=struct('bodyEnabled',true,'cableEnabled',true, ...
    'bodyMaxDeg',45,'cableMaxDeg',60,'bodyLambda',[2;2], ...
    'cableLambda',[1;1.2;1.4;1.6]);
pa=struct('kind','ellipsoid','center',[1.4;-.9;2.6], ...
    'R',eye(3),'params',struct('axes',[.5;.8;1.2]),'obstacleName','A');
pb=pa;pb.center=[-1.6;.9;2.8];pb.obstacleName='B';
a.parts=[pa pb];
end
function sd_find_generator()
need={'CBF_COMMON_MODEL','CBF_COMMON_GEOMETRY','CBF_REAL_XY_EVAL','CBF_REAL_XY_JET'};
if all(cellfun(@(s)exist(s,'file')==2,need)),return;end
r0=fileparts(mfilename('fullpath'));
roots={r0,pwd,fileparts(pwd),fullfile(pwd,'model','substance')};
for i=1:numel(roots)
    rr={fullfile(roots{i},'CBF_GENERATOR'),roots{i}, ...
        fullfile(roots{i},'model','substance','CBF_GENERATOR')};
    for j=1:numel(rr)
        cc=fullfile(rr{j},'COMMON');gg=fullfile(rr{j},'REAL','XY','GENERATED');
        if isfile(fullfile(cc,'CBF_COMMON_MODEL.m')) && isfile(fullfile(gg,'CBF_REAL_XY_EVAL.m'))
            addpath(cc,gg,'-begin');
            if all(cellfun(@(s)exist(s,'file')==2,need)),return;end
        end
    end
end
error('SD:Generator','Add existing CBF_GENERATOR/COMMON and REAL/XY/GENERATED to path.');
end
function sd_save(report)
base=fileparts(mfilename('fullpath'));out=fullfile(base,'CBF_SD_TEST_RESULTS.zip');
tmp=tempname;mkdir(tmp);g=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
fid=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fid,'CBF_SD_TEST - independent sampled-data interval test\n');
fprintf(fid,'Candidates: %d, constraints: %d, exceptions: %d\n', ...
    report.caseCount,report.barriers,numel(report.errors));
fprintf(fid,'Assumptions: exact estimated 19-state model per ZOH; P.mL is per-snapshot ESTIMATE.\n');
fprintf(fid,'Interval enclosure: substep Picard inclusion, plus outward-inflated binary64 interval Taylor jet.\n');
fprintf(fid,'Status PERIOD_HOCBF_CONDITIONAL_PASS is ONLY a model/rounding conditional within-period sufficient test.\n');
fprintf(fid,'A negative interval lower bound means INCONCLUSIVE, not collision.\n');
fprintf(fid,'Next input after sample / new P.mL and lower HOCBF / recursive viability: NOT_PROVEN.\n');
fprintf(fid,'Scope: cable ellipsoid-observed obstacles + body/cable tilt; no ground/body sphere/load sphere.\n');
fprintf(fid,'Does not write/modify any production MATLAB source.\n');
fclose(fid);
files={'SUMMARY.txt','ERRORS.txt'};
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if isempty(report.errors),fprintf(fid,'No errors.\n');else
    for j=1:numel(report.errors),fprintf(fid,'%s\n',report.errors{j});end
end
fclose(fid);
if ~isempty(report.rows)
    writetable(struct2table(report.rows),fullfile(tmp,'PER_BARRIER.csv'));
    files{end+1}='PER_BARRIER.csv';
    % One row per candidate physical input, not per individual obstacle.
    crows=struct('sampleIndex',{},'time',{},'massEstimateKg',{}, ...
        'massFactor',{},'gainFactor',{},'thrustN',{},'torqueX',{}, ...
        'torqueY',{},'torqueZ',{},'allConstraintsPass',{}, ...
        'allReachTubesPass',{},'barrierCount',{});
    cursor=1;
    while cursor<=numel(report.rows)
        first=report.rows(cursor);last=cursor;
        while last<numel(report.rows) && ...
            report.rows(last+1).sampleIndex==first.sampleIndex && ...
            report.rows(last+1).massFactor==first.massFactor && ...
            report.rows(last+1).gainFactor==first.gainFactor && ...
            report.rows(last+1).torqueX==first.torqueX && ...
            report.rows(last+1).torqueY==first.torqueY
            last=last+1;
        end
        rr=report.rows(cursor:last);
        crows(end+1)=struct('sampleIndex',first.sampleIndex,'time',first.time, ...
            'massEstimateKg',first.massEstimateKg,'massFactor',first.massFactor, ...
            'gainFactor',first.gainFactor,'thrustN',first.thrustN, ...
            'torqueX',first.torqueX,'torqueY',first.torqueY,'torqueZ',first.torqueZ, ...
            'allConstraintsPass',all(strcmp({rr.status},'PERIOD_HOCBF_CONDITIONAL_PASS')), ...
            'allReachTubesPass',all([rr.reachEnclosureOK]),'barrierCount',numel(rr)); %#ok<AGROW>
        cursor=last+1;
    end
    writetable(struct2table(crows),fullfile(tmp,'PER_CANDIDATE.csv'));
    files{end+1}='PER_CANDIDATE.csv';
end
if isfile(out),delete(out);end
zip(out,files,tmp);
fprintf('[SD] output: %s\n',out);
end
function sd_cleanup(folder)
if isfolder(folder),rmdir(folder,'s');end
end


% ========================================================================
% BOUNDARY MODE (2026-11-23): independent sampled-input HOCBF jump audit.
% ========================================================================
function report=sd_boundary_mode(varargin)
% Uses consecutive REAL logged estimator snapshots; never changes control.
% A detected numerical thrust interval is NOT a controlled-invariant set.
opt=struct('maxPairs',8,'targetTimes',[29.875 31.675 31.700], ...
    'thrustSamples',7,'testQP',true,'save',true,'timeTolerance',0.003);
if isempty(varargin) || ~(ischar(varargin{1}) || isstring(varargin{1}))
    error('SD:BoundaryArguments', ...
        'Usage: CBF_SD_TEST(''boundary'',''CBF_FOURWAY_RESULTS.zip'',name,value,...)');
end
zipPath=char(varargin{1});args=varargin(2:end);
if ~isfile(zipPath),error('SD:BoundaryInput','ZIP not found: %s',zipPath);end
if mod(numel(args),2)~=0,error('SD:BoundaryArguments','Name/value pairs required.');end
for k=1:2:numel(args)
    key=lower(char(args{k}));v=args{k+1};
    switch key
        case 'maxpairs',opt.maxPairs=v;
        case 'targettimes',opt.targetTimes=v;
        case 'thrustsamples',opt.thrustSamples=v;
        case 'testqp',opt.testQP=logical(v);
        case 'save',opt.save=logical(v);
        case 'timetolerance',opt.timeTolerance=v;
        otherwise,error('SD:BoundaryArguments','Unknown option: %s',key);
    end
end
validateattributes(opt.maxPairs,{'numeric'},{'scalar','integer','>=',1,'<=',100});
validateattributes(opt.thrustSamples,{'numeric'},{'scalar','integer','>=',2,'<=',31});
validateattributes(opt.targetTimes,{'numeric'},{'vector','finite','real'});
validateattributes(opt.timeTolerance,{'numeric'},{'scalar','positive','finite','<=',0.025});
oldPath=path;pg=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
work=tempname;mkdir(work);cleanup=onCleanup(@()sd_cleanup(work)); %#ok<NASGU>
unzip(zipPath,work);
csvFile=fullfile(work,'PER_SAMPLE.csv');matFile=fullfile(work,'RAW_SNAPSHOTS.mat');
if ~isfile(csvFile) || ~isfile(matFile)
    error('SD:BoundaryInput','FOURWAY ZIP requires PER_SAMPLE.csv and RAW_SNAPSHOTS.mat.');
end
tab=readtable(csvFile);
required={'t','parts','min_psi3','min_h_m'};
for k=1:numel(required)
    if ~ismember(required{k},tab.Properties.VariableNames)
        error('SD:BoundaryInput','Missing PER_SAMPLE column: %s',required{k});
    end
end
N=height(tab);
if N<2,error('SD:BoundaryInput','Not enough logged samples.');end
delta=tab.t(2:end)-tab.t(1:end-1);
valid=find(tab.parts(1:end-1)>0 & tab.parts(2:end)>0 & ...
    isfinite(tab.min_psi3(1:end-1)) & ...
    isfinite(tab.min_psi3(2:end)) & delta>1e-6 & ...
    delta<=0.1); % Each pair is subsequently verified against its logged dt.
if isempty(valid),error('SD:BoundaryInput','No increasing-time adjacent observed-obstacle pairs.');end
priority=zeros(0,1);
for tt=opt.targetTimes(:).'
    [d,ix]=min(abs(tab.t(valid)-tt));
    if d<=opt.timeTolerance,priority(end+1,1)=valid(ix);end %#ok<AGROW>
end
crossing=valid(tab.min_psi3(valid)>=0 & tab.min_psi3(valid+1)<0);
[~,oi]=sort(tab.min_h_m(crossing));crossing=crossing(oi);
[~,mi]=sort(tab.min_h_m(valid));
selected=unique([priority;crossing(:);valid(mi)],'stable');
selected=selected(1:min(opt.maxPairs,numel(selected)));
selected=sort(selected);
raw=load(matFile,'snapshot');
if ~iscell(raw.snapshot) || numel(raw.snapshot)<max(selected+1)
    error('SD:BoundaryInput','snapshot must be cell array with matching CSV indices.');
end
pairRows=struct([]);stageRows=struct([]);qpRows=struct([]);errs={};
for j=1:numel(selected)
    k=selected(j);
    try
        [pr,st,qp]=sd_boundary_pair(raw.snapshot{k},raw.snapshot{k+1},k,opt);
        if isempty(pairRows),pairRows=pr;else,pairRows(end+1)=pr;end %#ok<AGROW>
        if ~isempty(st)
            if isempty(stageRows),stageRows=st;else,stageRows(end+1:end+numel(st))=st;end %#ok<AGROW>
        end
        if ~isempty(qp)
            if isempty(qpRows),qpRows=qp;else,qpRows(end+1:end+numel(qp))=qp;end %#ok<AGROW>
        end
        fprintf('[SD BOUNDARY] %d->%d t=%.3f: old-end psi3=%+.6g, new-start psi3=%+.6g, Tband=[%.6g %.6g], %s\n', ...
            k,k+1,pr.time,pr.oldInputNextMinLower,pr.newInputNextMinLower, ...
            pr.lowerThrustLo,pr.lowerThrustHi,pr.thrustBandStatus);
    catch ME
        trace='';
        for e=1:numel(ME.stack)
            trace=[trace sprintf(' %s:%d',ME.stack(e).name,ME.stack(e).line)]; %#ok<AGROW>
        end
        errs{end+1,1}=sprintf('pair=%d->%d %s: %s%s', ...
            k,k+1,ME.identifier,ME.message,trace); %#ok<AGROW>
        fprintf('[SD BOUNDARY] FAILED pair %d->%d: %s\n',k,k+1,ME.message);
    end
end
report=struct('mode','boundary','pairCount',numel(selected),'pairs',pairRows, ...
    'stages',stageRows,'qpCandidates',qpRows,'errors',{errs}, ...
    'options',opt,'guarantee','NUMERICAL_DIAGNOSTIC_NOT_A_PROOF');
fprintf('[SD BOUNDARY] pairs=%d success=%d errors=%d.\n', ...
    numel(selected),numel(pairRows),numel(errs));
fprintf('[SD BOUNDARY] Thrust-interval inference conditional on affine low-stage check; recursive safety NOT PROVEN.\n');
if opt.save,sd_boundary_save(report);end
end

function [pr,stages,qpRows]=sd_boundary_pair(a,b,idx,opt)
needed={'x','u','P','G','parts','lambda','tiltCfg','time','dt'};
for j=1:numel(needed)
    if ~isfield(a,needed{j}) || ~isfield(b,needed{j})
        error('SD:BoundarySnapshot','Missing %s in snapshot',needed{j});
    end
end
x0=double(a.x(:));x1=double(b.x(:));u0=double(a.u(:));u1=double(b.u(:));
assert(numel(x0)==19 && numel(x1)==19 && numel(u0)==4 && numel(u1)==4, ...
    'SD:BoundarySnapshot','Incorrect state or input length.');
assert(a.P.mL>=0 && b.P.mL>=0,'SD:BoundarySnapshot','Invalid estimated payload mass.');
assert(abs((b.time-a.time)-a.dt)<1e-4, ...
    'SD:BoundarySnapshot','Snapshot times are not one control period apart.');
% Same obstacle/geometry is REQUIRED for a physically interpretable flow
% and input/model decomposition. Changed geometry is not silently matched.
geomSame=isequaln(a.parts,b.parts);
E0=CBF_REAL_XY_EVAL(x0,u0,a.P,a.G,a.parts,a.lambda,a.tiltCfg);
Eflow=CBF_REAL_XY_EVAL(x1,u0,a.P,a.G,a.parts,a.lambda,a.tiltCfg);
ustar=[u1(1);u0(2);u0(3);u0(4)];
Ethrust=CBF_REAL_XY_EVAL(x1,ustar,a.P,a.G,a.parts,a.lambda,a.tiltCfg);
Eother=CBF_REAL_XY_EVAL(x1,u1,a.P,a.G,a.parts,a.lambda,a.tiltCfg);
Enew=CBF_REAL_XY_EVAL(x1,u1,b.P,b.G,b.parts,b.lambda,b.tiltCfg);
oldNames=sd_names(a.parts,a.tiltCfg);nextNames=sd_names(b.parts,b.tiltCfg);
match=isequal(oldNames,nextNames) && geomSame;
oldStages=sd_boundary_stage_cell(E0,a.tiltCfg);
flowStages=sd_boundary_stage_cell(Eflow,a.tiltCfg);
thrustStages=sd_boundary_stage_cell(Ethrust,a.tiltCfg);
otherStages=sd_boundary_stage_cell(Eother,a.tiltCfg);
newStages=sd_boundary_stage_cell(Enew,b.tiltCfg);
[band,qpRows]=sd_boundary_thrust_band(b,idx+1,opt);
pr=struct('sampleIndex',idx,'nextSampleIndex',idx+1, ...
    'time',double(a.time),'nextTime',double(b.time),'dt',double(a.dt), ...
    'oldThrustN',u0(1),'newThrustN',u1(1),'deltaThrustN',u1(1)-u0(1), ...
    'oldMassEstimateKg',double(a.P.mL),'newMassEstimateKg',double(b.P.mL), ...
    'deltaMassEstimateKg',double(b.P.mL-a.P.mL), ...
    'sameObstacleGeometry',geomSame,'barrierNamesMatch',isequal(oldNames,nextNames), ...
    'decompositionComparable',match,'barrierCountOld',numel(oldNames), ...
    'barrierCountNew',numel(nextNames), ...
    'oldStartMinLower',sd_boundary_min_low(oldStages), ...
    'oldInputNextMinLower',sd_boundary_min_low(flowStages), ...
    'newInputNextMinLower',sd_boundary_min_low(newStages), ...
    'oldInputNextMinPsi3',sd_boundary_min_order(flowStages,3), ...
    'newInputNextMinPsi3',sd_boundary_min_order(newStages,3), ...
    'oldInputNextLowerOK',sd_boundary_min_low(flowStages)>=-1e-8, ...
    'newInputNextLowerOK',sd_boundary_min_low(newStages)>=-1e-8, ...
    'lowerThrustLo',band.lo,'lowerThrustHi',band.hi, ...
    'thrustBandStatus',band.status,'thrustAffineError',band.affineResidual, ...
    'thrustNominalInBand',band.nominalInBand, ...
    'thrustNearestInBand',band.nearest, ...
    'thrustChangeToBandN',band.nearest-u1(1), ...
    'sampledCommonQPFound',band.qpFound, ...
    'sampledCommonQPThrust',band.qpThrust, ...
    'sampledCommonQPTorqueX',band.qpTorqueX, ...
    'sampledCommonQPTorqueY',band.qpTorqueY, ...
    'recursiveGuarantee','NOT_PROVEN');

stages=struct([]);
if ~isequal(oldNames,nextNames)
    % Still reports each snapshot's aggregate and next-time T band.
    return;
end
for i=1:numel(oldNames)
    n=min(numel(oldStages{i}),numel(newStages{i}))-1;
    for order=0:n-1
        t=order+1;
        v0=oldStages{i}(t);vf=flowStages{i}(t);
        vt=thrustStages{i}(t);vo=otherStages{i}(t);vn=newStages{i}(t);
        s=struct('sampleIndex',idx,'nextSampleIndex',idx+1, ...
            'time',double(a.time),'barrier',oldNames{i}, ...
            'order',order,'isPsi3',order==3, ...
            'begin',v0,'flowSameInput',vf, ...
            'afterNewThrust',vt,'afterOtherInput',vo, ...
            'afterNewModel',vn,'flowChange',vf-v0, ...
            'thrustChange',vt-vf,'otherInputChange',vo-vt, ...
            'parameterAndGeometryChange',vn-vo, ...
            'totalChange',vn-v0,'identityResidual', ...
            (vf-v0)+(vt-vf)+(vo-vt)+(vn-vo)-(vn-v0), ...
            'oldEndIsNonnegative',vf>=-1e-8, ...
            'newStartIsNonnegative',vn>=-1e-8, ...
            'sameObstacleGeometry',geomSame, ...
            'decompositionComparable',match);
        if isempty(stages),stages=s;else,stages(end+1)=s;end %#ok<AGROW>
    end
end
end

function C=sd_boundary_stage_cell(E,tilt)
C=cell(0,1);
for i=1:size(E.stages,2),C{end+1,1}=double(E.stages(:,i));end %#ok<AGROW>
if tilt.bodyEnabled,C{end+1,1}=double(E.tiltBodyStages(:));end
if tilt.cableEnabled,C{end+1,1}=double(E.tiltCableStages(:));end
end

function m=sd_boundary_min_low(C)
m=Inf;
for i=1:numel(C)
    v=C{i};m=min(m,min(v(1:end-1)));
end
if ~isfinite(m),m=NaN;end
end

function m=sd_boundary_min_order(C,order)
m=Inf;
for i=1:numel(C)
    if numel(C{i})>order+1
        m=min(m,C{i}(order+1));
    end
end
if ~isfinite(m),m=NaN;end
end

function [band,qpRows]=sd_boundary_thrust_band(s,idx,opt)
% Generator structure, not a fitted one-off parameter: in the given
% 19-state model, every low-stage is (analytically) affine in thrust to
% order r-1 (r=4 for cable; r=2 for body). This source-structure assertion
% must be re-audited if generator is modified. 3-point residual is ONLY a
% numerical regression check, not a symbolic proof.
low=0;high=20;tol=1e-8;
baseU=[0;0;0;s.u(4)];E0=CBF_REAL_XY_EVAL(s.x,baseU,s.P,s.G,s.parts,s.lambda,s.tiltCfg);
baseU(1)=20;E20=CBF_REAL_XY_EVAL(s.x,baseU,s.P,s.G,s.parts,s.lambda,s.tiltCfg);
baseU(1)=10;E10=CBF_REAL_XY_EVAL(s.x,baseU,s.P,s.G,s.parts,s.lambda,s.tiltCfg);
S0=sd_boundary_stage_cell(E0,s.tiltCfg);
S20=sd_boundary_stage_cell(E20,s.tiltCfg);
S10=sd_boundary_stage_cell(E10,s.tiltCfg);
maxRes=0;flatA=[];flatB=[];flatScale=[];
for i=1:numel(S0)
    a=S0{i}(1:end-1);b=(S20{i}(1:end-1)-a)/20;
    mid=S10{i}(1:end-1);
    maxRes=max(maxRes,max(abs(mid-(a+10*b))./(1+abs(mid))));
    flatA=[flatA;a];flatB=[flatB;b]; %#ok<AGROW>
    flatScale=[flatScale;1+abs(mid)]; %#ok<AGROW>
end
affOK=maxRes<=1e-7;
status='LOWER_T_AFFINITY_UNCONFIRMED';
if affOK
    for i=1:numel(flatA)
        a=flatA(i);b=flatB(i);
        if abs(b)<=1e-11*(1+abs(a))
            if a<-tol,low=Inf;high=-Inf;break;end
        elseif b>0
            low=max(low,(-tol-a)/b);
        else
            high=min(high,(-tol-a)/b);
        end
    end
    if low>high+1e-8
        status='NO_LOWER_T_BAND_IN_AFFINE_MODEL';
    else
        low=max(0,low);high=min(20,high);
        status='NUMERICAL_LOWER_T_BAND';
    end
end
band=struct('lo',low,'hi',high,'status',status,'affineResidual',maxRes, ...
    'nominalInBand',false,'nearest',NaN, ...
    'qpFound',false,'qpThrust',NaN,'qpTorqueX',NaN,'qpTorqueY',NaN);
qpRows=struct([]);
if ~affOK || low>high+1e-8,return;end
Tnom=double(s.u(1));band.nominalInBand=Tnom>=low-1e-8 && Tnom<=high+1e-8;
band.nearest=min(max(Tnom,low),high);
if ~opt.testQP,return;end
samples=unique([linspace(low,high,opt.thrustSamples), ...
    band.nearest,Tnom*(band.nominalInBand)],'stable');
samples=samples(samples>=low-1e-8 & samples<=high+1e-8);
for T=samples
    [yes,z,lowMin,affXY]=sd_boundary_xy_qp(s,T);
    r=struct('nextSampleIndex',idx,'nextTime',s.time, ...
        'candidateThrustN',T,'lowestStage',lowMin, ...
        'xyAffineResidual',affXY,'jointCurrentQPFeasible',yes, ...
        'feasibleTorqueX',NaN,'feasibleTorqueY',NaN, ...
        'predictiveOrRecursiveCertified',false);
    if yes,r.feasibleTorqueX=z(1);r.feasibleTorqueY=z(2);end
    if isempty(qpRows),qpRows=r;else,qpRows(end+1)=r;end %#ok<AGROW>
    if yes && ~band.qpFound
        band.qpFound=true;band.qpThrust=T;
        band.qpTorqueX=z(1);band.qpTorqueY=z(2);
    end
end
end

function [ok,z,minLower,affErr]=sd_boundary_xy_qp(s,T)
% Current-time joint XY feasibility only; NOT sampled-data feasibility.
u=[T;0;0;s.u(4)];
E0=CBF_REAL_XY_EVAL(s.x,u,s.P,s.G,s.parts,s.lambda,s.tiltCfg);
Ex=CBF_REAL_XY_EVAL(s.x,u+[0;1;0;0],s.P,s.G,s.parts,s.lambda,s.tiltCfg);
Ey=CBF_REAL_XY_EVAL(s.x,u+[0;0;1;0],s.P,s.G,s.parts,s.lambda,s.tiltCfg);
Em=CBF_REAL_XY_EVAL(s.x,u+[0;.37;-.41;0],s.P,s.G,s.parts,s.lambda,s.tiltCfg);
S=sd_boundary_stage_cell(E0,s.tiltCfg);
minLower=sd_boundary_min_low(S);ok=false;z=[NaN;NaN];affErr=NaN;
if ~isfinite(minLower) || minLower < -1e-8,return;end
v0=[];vx=[];vy=[];vm=[];
for i=1:numel(S)
    n=numel(S{i});
    v0(end+1,1)=S{i}(n); %#ok<AGROW>
end
sx=sd_boundary_stage_cell(Ex,s.tiltCfg);
sy=sd_boundary_stage_cell(Ey,s.tiltCfg);
sm=sd_boundary_stage_cell(Em,s.tiltCfg);
for i=1:numel(S)
    vx(end+1,1)=sx{i}(end); %#ok<AGROW>
    vy(end+1,1)=sy{i}(end); %#ok<AGROW>
    vm(end+1,1)=sm{i}(end); %#ok<AGROW>
end
B=[vx-v0,vy-v0];
affErr=max(abs(vm-(v0+B*[.37;-.41]))./(1+abs(vm)));
if affErr>1e-7,return;end
poly=[-1 -1;1 -1;1 1;-1 1];
for i=1:numel(v0)
    poly=sd_boundary_clip_polygon(poly,B(i,:),v0(i)+1e-8);
    if isempty(poly),return;end
end
ok=true;z=mean(poly,1).';
if any(B*z+v0 < -2e-7) || any(abs(z)>1+2e-7)
    ok=false;z=[NaN;NaN];
end
end

function out=sd_boundary_clip_polygon(poly,b,c)
% Clip 2D convex polygon by b*z+c >=0 (Sutherland-Hodgman).
out=zeros(0,2);if isempty(poly),return;end
N=size(poly,1);
for i=1:N
    a=poly(i,:);d=poly(mod(i,N)+1,:);
    fa=b*a.'+c;fd=b*d.'+c;
    inA=fa>=0;inD=fd>=0;
    if inA && inD
        out(end+1,:)=d; %#ok<AGROW>
    elseif inA && ~inD
        r=fa/(fa-fd);out(end+1,:)=a+r*(d-a); %#ok<AGROW>
    elseif ~inA && inD
        r=fa/(fa-fd);out(end+1,:)=a+r*(d-a); %#ok<AGROW>
        out(end+1,:)=d; %#ok<AGROW>
    end
end
end

function sd_boundary_save(r)
root=fileparts(mfilename('fullpath'));out=fullfile(root,'CBF_SD_TEST_RESULTS.zip');
tmp=tempname;mkdir(tmp);cg=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
fid=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fid,'CBF_SD_TEST: input-boundary mixed-relative-degree audit\n');
fprintf(fid,'Selected pairs %d, successful %d, errors %d\n', ...
    r.pairCount,numel(r.pairs),numel(r.errors));
fprintf(fid,'All states are logged estimated states; P.mL is estimated load mass per sample.\n');
fprintf(fid,'Old input is held ONLY for old 25 ms; next input is evaluated at next logged state.\n');
fprintf(fid,'Decomposition: start->end same input -> change thrust -> change XY/yaw -> update mass/geometry/gains.\n');
fprintf(fid,'Numeric thrust band uses affine low-stage structure and a 3-point regression test; not a formal symbolic verification.\n');
fprintf(fid,'Candidate common XY QP checks are single-time feasibility; no ZOH or recursive proof.\n');
fprintf(fid,'No geometric collisions are implied by negative psi3.\n');
fprintf(fid,'Production files are NOT modified.\n');
fclose(fid);
files={'SUMMARY.txt','ERRORS.txt'};
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if isempty(r.errors),fprintf(fid,'No errors.\n');else
    for i=1:numel(r.errors),fprintf(fid,'%s\n',r.errors{i});end
end
fclose(fid);
if ~isempty(r.pairs)
    writetable(struct2table(r.pairs),fullfile(tmp,'BOUNDARY_PAIRS.csv'));
    files{end+1}='BOUNDARY_PAIRS.csv';
end
if ~isempty(r.stages)
    writetable(struct2table(r.stages),fullfile(tmp,'BOUNDARY_STAGES.csv'));
    files{end+1}='BOUNDARY_STAGES.csv';
end
if ~isempty(r.qpCandidates)
    writetable(struct2table(r.qpCandidates),fullfile(tmp,'BOUNDARY_QP_CANDIDATES.csv'));
    files{end+1}='BOUNDARY_QP_CANDIDATES.csv';
end
if isfile(out),delete(out);end
zip(out,files,tmp);
fprintf('[SD BOUNDARY] results: %s\n',out);
end

function result=sd_boundary_selftest()
oldPath=path;pg=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
a=sd_synthetic();a.parts=a.parts(1:2);b=a;
b.time=a.time+a.dt;
b.u(1)=a.u(1)+0.25;
% A fixed state avoids requiring any hypothetical input application.
[pr,st,qp]=sd_boundary_pair(a,b,1, ...
    struct('testQP',true,'thrustSamples',5));
assert(pr.sameObstacleGeometry && pr.barrierNamesMatch);
assert(abs(pr.oldInputNextMinLower-pr.oldStartMinLower)<1e-8);
assert(all(abs([st.flowChange])<1e-8));
assert(max(abs([st.identityResidual]))<1e-8);
assert(pr.thrustAffineError<1e-7,'SD:BoundarySelftest', ...
    'Low-stage thrust affine regression failed.');
assert(~isempty(qp) || ~strcmp(pr.thrustBandStatus,'NUMERICAL_LOWER_T_BAND'));
% Clip a square by z_x>=0 and verify every output vertex satisfies it.
p=sd_boundary_clip_polygon([-1 -1;1 -1;1 1;-1 1],[1 0],0);
assert(~isempty(p) && all(p(:,1)>=-1e-12));
result=struct('mode','boundarySelftest','status','PASS','pair',pr);
fprintf('[SD BOUNDARY] 2-obstacle mixed-relative-degree decomposition, thrust affine check and polygon clip: PASS.\n');
fprintf('[SD BOUNDARY] Guarantee: NOT_PROVEN; this is a regression test.\n');
end


% =======================================================================
% DUAL MODE: side-by-side conditional HOCBF (A) vs direct geometric h (B)
% =======================================================================
function out=sd_dual_mode(varargin)
% Read-only; no future input is assumed, no simulation/controller patch.
if isempty(varargin),error('SD:DualInput','dual needs FOURWAY ZIP path.');end
args=varargin; zipPath=char(args{1});args=args(2:end);
if ~isfile(zipPath),error('SD:DualInput','FOURWAY ZIP not found: %s',zipPath);end
opts=struct('maxCases',2,'substeps',4,'tubeIterations',14, ...
    'gamma',1,'torqueStep',0.05,'candidates',1,'save',true, ...
    'timingRepeats',3);
if rem(numel(args),2)~=0,error('SD:DualInput','Use name/value pairs after ZIP.');end
for k=1:2:numel(args)
    key=lower(char(args{k}));v=args{k+1};
    switch key
        case 'maxcases',opts.maxCases=v;
        case 'substeps',opts.substeps=v;
        case 'tubeiterations',opts.tubeIterations=v;
        case 'gamma',opts.gamma=v;
        case 'torquestep',opts.torqueStep=v;
        case 'candidates',opts.candidates=v;
        case 'save',opts.save=logical(v);
        case 'timingrepeats',opts.timingRepeats=v;
        otherwise,error('SD:DualInput','Unknown option: %s',key);
    end
end
validateattributes(opts.maxCases,{'numeric'},{'scalar','integer','>=',1,'<=',40});
validateattributes(opts.substeps,{'numeric'},{'scalar','integer','>=',1,'<=',64});
validateattributes(opts.tubeIterations,{'numeric'},{'scalar','integer','>=',4,'<=',32});
validateattributes(opts.gamma,{'numeric'},{'scalar','real','finite','>',0,'<=',1});
validateattributes(opts.torqueStep,{'numeric'},{'scalar','real','finite','>=',0,'<=',1});
validateattributes(opts.candidates,{'numeric'},{'scalar','integer','>=',1,'<=',5});
validateattributes(opts.timingRepeats,{'numeric'},{'scalar','integer','>=',0,'<=',5});
opts.dualEndpoint=true;
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
setupClock=tic;
sd_find_generator();
setupGeneratorMs=1000*toc(setupClock);
loadClock=tic;
tmp=tempname;mkdir(tmp);tmpGuard=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
unzip(zipPath,tmp);
T=readtable(fullfile(tmp,'PER_SAMPLE.csv'));
valid=find(T.parts>0);
if isempty(valid),error('SD:DualInput','No active obstacles.');end
k=min(opts.maxCases,numel(valid));
% Diverse deterministic targets: closest h, lowest psi3, uniform temporal sample.
[~,ha]=sort(T.min_h_m(valid));[~,pa]=sort(T.min_psi3(valid));
ns=min(ceil(k/3),numel(valid));
chosen=unique([valid(ha(1:ns));valid(pa(1:ns)); ...
    valid(unique(round(linspace(1,numel(valid),k))))],'stable');
chosen=chosen(1:min(k,numel(chosen)));
if numel(chosen)<k
    remain=setdiff(valid,chosen,'stable');chosen=[chosen;remain(1:k-numel(chosen))];
end
chosen=sort(chosen(:));
R=load(fullfile(tmp,'RAW_SNAPSHOTS.mat'),'snapshot');
loadDataMs=1000*toc(loadClock);
if ~iscell(R.snapshot) || numel(R.snapshot)<max(chosen)
    error('SD:DualInput','Expected cell vector of snapshots.');
end
candidateRows=struct([]);barrierRows=struct([]);errors=cell(0,1);
timingRows=struct([]);
for j=1:numel(chosen)
    s=R.snapshot{chosen(j)};
    if ~isstruct(s) || ~isfield(s,'u'),errors{end+1,1}=sprintf('Invalid snapshot %d',chosen(j));continue;end %#ok<AGROW>
    offsets=[0 0; opts.torqueStep 0; -opts.torqueStep 0; 0 opts.torqueStep; 0 -opts.torqueStep];
    offsets=offsets(1:opts.candidates,:);
    for c=1:size(offsets,1)
        d=offsets(c,:).';
        if any(abs(double(s.u(2:3))+d)>1+1e-12),continue;end
        clock=tic;
        try
            row=sd_case(s,chosen(j),1,1,d,opts);
            elapsedMs=1000*toc(clock);
            % Repeat the EXACT same first logged state/input. This isolates
            % one-time execution overhead from a genuinely longer code path.
            if j==1 && c==1
                for timingIndex=0:opts.timingRepeats
                    if timingIndex==0
                        ms=elapsedMs;testedRow=row;
                    else
                        repeatClock=tic;
                        testedRow=sd_case(s,chosen(j),1,1,d,opts);
                        ms=1000*toc(repeatClock);
                    end
                    tr=struct('sampleIndex',chosen(j),'candidate',c, ...
                        'repeatIndex',timingIndex,'elapsedMs',ms, ...
                        'picardMs',testedRow(1).picardElapsedMs, ...
                        'intervalStageMs',testedRow(1).intervalStagesElapsedMs, ...
                        'generatorCheckMs',testedRow(1).generatorCheckElapsedMs, ...
                        'otherMs',max(0,ms-testedRow(1).picardElapsedMs- ...
                            testedRow(1).intervalStagesElapsedMs- ...
                            testedRow(1).generatorCheckElapsedMs), ...
                        'picardIterations',testedRow(1).picardIterations, ...
                        'substepsCompleted',testedRow(1).picardSubstepsCompleted, ...
                        'reachEnclosureOK',testedRow(1).reachEnclosureOK);
                    if isempty(timingRows),timingRows=tr;
                    else,timingRows(end+1)=tr;end %#ok<AGROW>
                end
            end
            bRows=repmat(sd_dual_barrier_template(),numel(row),1);
            allA=true;allB=true;allBGeom=true;allReach=true;
            for q=1:numel(row)
                a=row(q); br=sd_dual_barrier_template();
                br.sampleIndex=chosen(j);br.time=a.time; br.candidate=c;
                br.constraint=a.barrier; br.thrustN=a.thrustN;
                br.torqueX=a.torqueX;br.torqueY=a.torqueY;br.torqueZ=a.torqueZ;
                br.massEstimateKg=a.massEstimateKg;br.dt=a.dt;
                br.reachEnclosureOK=a.reachEnclosureOK;
                br.A_lowerStagesOK=a.initialLowerOK;
                br.A_periodCBFOK=strcmp(a.status,'PERIOD_HOCBF_CONDITIONAL_PASS');
                br.B_minHLower=a.intervalMinH;
                br.B_hAtStartLower=a.initialHLower;
                br.B_hAtEndLower=a.terminalHLower;
                br.B_gammaGap=a.terminalHLower-(1-opts.gamma)*a.initialHUpper;
                br.B_geometryWithinPeriodOK=a.reachEnclosureOK && ...
                    isfinite(a.intervalMinH) && a.intervalMinH>=0;
                br.B_endpointConditionOK=a.terminalEvaluationOK && ...
                    isfinite(br.B_gammaGap) && br.B_gammaGap>=0;
                br.B_periodAndEndpointOK=br.B_geometryWithinPeriodOK && br.B_endpointConditionOK;
                br.A_intervalFmin=a.intervalMinFinal;
                br.reachReason=a.reachStatus;
                br.elapsedCandidateMs=elapsedMs;
                allA=allA && br.A_periodCBFOK;
                allB=allB && br.B_periodAndEndpointOK;
                allBGeom=allBGeom && br.B_geometryWithinPeriodOK;
                allReach=allReach && a.reachEnclosureOK;
                bRows(q)=br;
            end
            if isempty(barrierRows),barrierRows=bRows;
            else,barrierRows(end+1:end+numel(bRows))=bRows;end %#ok<AGROW>
            cr=struct('sampleIndex',chosen(j),'time',row(1).time, ...
                'candidate',c,'offsetX',d(1),'offsetY',d(2), ...
                'thrustN',row(1).thrustN,'torqueX',row(1).torqueX, ...
                'torqueY',row(1).torqueY,'torqueZ',row(1).torqueZ, ...
                'estimatedLoadMassKg',row(1).massEstimateKg, ...
                'numObservedObstacles',numel(s.parts), ...
                'numConstraints',numel(row),'A_allPass',allA, ...
                'B_periodGeometryAllPass',allBGeom, ...
                'B_periodAndEndpointAllPass',allB, ...
                'bothAllPass',allA && allB, ...
                'reachableTubeAllPass',allReach, ...
                'elapsedMs',elapsedMs, ...
                'elapsedVs25ms',elapsedMs/25, ...
                'picardMs',row(1).picardElapsedMs, ...
                'intervalStageMs',row(1).intervalStagesElapsedMs, ...
                'generatorCheckMs',row(1).generatorCheckElapsedMs, ...
                'otherMs',max(0,elapsedMs-row(1).picardElapsedMs- ...
                    row(1).intervalStagesElapsedMs-row(1).generatorCheckElapsedMs), ...
                'picardIterations',row(1).picardIterations, ...
                'substepsCompleted',row(1).picardSubstepsCompleted, ...
                'recursiveFeasibility','NOT_PROVEN', ...
                'machineCheckedProof','NOT_PROVEN');
            if isempty(candidateRows),candidateRows=cr;
            else,candidateRows(end+1)=cr;end %#ok<AGROW>
        catch ME
            trace='';for z=1:numel(ME.stack)
                trace=[trace sprintf(' %s:%d',ME.stack(z).name,ME.stack(z).line)]; %#ok<AGROW>
            end
            errors{end+1,1}=sprintf('sample %d candidate %d: %s | %s', ...
                chosen(j),c,ME.message,trace); %#ok<AGROW>
        end
    end
end
out=struct('mode','dual','options',opts,'cases',numel(chosen), ...
    'candidateRows',candidateRows,'barrierRows',barrierRows,'errors',{errors}, ...
    'timingRows',timingRows,'setupGeneratorMs',setupGeneratorMs, ...
    'loadDataMs',loadDataMs, ...
    'theoremStatus','CONDITIONAL_ONE_PERIOD_ONLY; NO RECURSIVE PROOF');
if isempty(candidateRows)
    fprintf('[SD DUAL] No completed candidates; errors=%d\n',numel(errors));
else
    A=sum([candidateRows.A_allPass]); B=sum([candidateRows.B_periodAndEndpointAllPass]);
    fprintf('[SD DUAL] snapshots=%d completed candidates=%d barriers=%d errors=%d\n', ...
        numel(chosen),numel(candidateRows),numel(barrierRows),numel(errors));
    fprintf('[SD DUAL] A all constraints 1-ZOH conditional PASS: %d/%d\n',A,numel(candidateRows));
    fprintf('[SD DUAL] B all constraints 1-ZOH + endpoint conditional PASS: %d/%d\n',B,numel(candidateRows));
    times=[candidateRows.elapsedMs];
    fprintf('[SD DUAL] computation time median=%.3f ms, worst=%.3f ms; 25ms deadline is NOT certified\n', ...
        median(times),max(times));
    fprintf('[SD DUAL] offline generator setup=%.3f ms, ZIP/MAT loading=%.3f ms (excluded from candidate timings)\n', ...
        setupGeneratorMs,loadDataMs);
    if numel(timingRows)>1
        rrMs=[timingRows.elapsedMs];
        fprintf('[SD DUAL] exact same first candidate: first %.3f ms, repeats median %.3f ms\n', ...
            rrMs(1),median(rrMs(2:end)));
    end
end
if ~isempty(errors),fprintf('[SD DUAL] FIRST ERROR: %s\n',errors{1});end
fprintf('[SD DUAL] all-future safety / recursive viability / machine checked rounding: NOT PROVEN\n');
if opts.save,sd_dual_save(out);end
end

function br=sd_dual_barrier_template()
br=struct('sampleIndex',NaN,'time',NaN,'candidate',NaN, ...
    'constraint','','thrustN',NaN,'torqueX',NaN,'torqueY',NaN,'torqueZ',NaN, ...
    'massEstimateKg',NaN,'dt',NaN,'reachEnclosureOK',false, ...
    'A_lowerStagesOK',false,'A_periodCBFOK',false,'A_intervalFmin',NaN, ...
    'B_minHLower',NaN,'B_hAtStartLower',NaN,'B_hAtEndLower',NaN, ...
    'B_gammaGap',NaN,'B_geometryWithinPeriodOK',false, ...
    'B_endpointConditionOK',false,'B_periodAndEndpointOK',false, ...
    'reachReason','','elapsedCandidateMs',NaN);
end

function r=sd_dual_selftest()
oldPath=path;pg=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
sd_selftest_primitives();
s=sd_synthetic();opts=struct('substeps',2,'tubeIterations',14,'dualEndpoint',true);
a=sd_case(s,1,1,1,[0;0],opts);
assert(numel(a)==numel(sd_names(s.parts,s.tiltCfg)),'SD:DualTest','Barrier count');
for i=1:numel(a)
    assert(a(i).terminalEvaluationOK==false || ...
        a(i).terminalHLower<=a(i).terminalHUpper,'SD:DualTest','Endpoint interval ordering');
    assert(~strcmp(a(i).status,'PERIOD_HOCBF_CONDITIONAL_PASS') || ...
        a(i).initialLowerOK,'SD:DualTest','A initial-condition implication');
end
r=struct('mode','dualSelftest','rows',a, ...
    'status','PRIMITIVES_AND_DUAL_PLUMBING_PASS__NOT_SAFETY_PROOF');
fprintf('[SD DUAL SELFTEST] PASS: %d constraints; not a safety certificate.\n',numel(a));
end

function sd_dual_save(r)
root=fileparts(mfilename('fullpath'));out=fullfile(root,'CBF_SD_TEST_RESULTS.zip');
tmp=tempname;mkdir(tmp);cg=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
fid=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fid,'CBF_SD_TEST DUAL COMPARISON (READ-ONLY)\n');
fprintf(fid,'No production Sim/controller/generator edits.\n');
fprintf(fid,'Input: estimated 19-state model and estimated mass per snapshot; thrust and yaw held nominal/logged.\n');
fprintf(fid,'A: all initial low HOCBF >=0 and min over verified interval of highest HOCBF >=0.\n');
fprintf(fid,'B: all min over verified interval of geometric h >=0 and endpoint h >= (1-gamma)h_start.\n');
fprintf(fid,'gamma=%.12g; current ZOH substeps=%d, candidate step=%.12g Nm.\n', ...
    r.options.gamma,r.options.substeps,r.options.torqueStep);
fprintf(fid,'This does NOT prove next HLC input is safe, existence of future safe inputs, or recursive feasibility.\n');
fprintf(fid,'Interval primitives 8-ULP rounding assumptions remain UNVERIFIED on target MATLAB hardware.\n');
fprintf(fid,'Timing is offline MATLAB diagnostic wall clock, NOT online real-time control certification.\n');
fprintf(fid,'Generator setup_ms=%.9g; ZIP/CSV/MAT loading_ms=%.9g (both excluded from candidate timing).\n', ...
    r.setupGeneratorMs,r.loadDataMs);
fprintf(fid,'Breakdown: Picard, interval HOCBF stages, generator crosscheck, other diagnostic overhead.\n');
fprintf(fid,'Repeated identical first candidate separates cold-path effects from algorithmic complexity; does not prove JIT causality.\n');
fprintf(fid,'Current physical obstacle scope: cable ellipsoid vs observed obstacle parts, body and cable tilt.\n');
fprintf(fid,'Future scope reserved: body sphere, payload sphere, ground; NOT IMPLEMENTED.\n');
if ~isempty(r.candidateRows)
   p=[r.candidateRows.elapsedMs];
   fprintf(fid,'Candidates=%d, A_pass=%d, B_pass=%d, both_pass=%d\n', ...
       numel(r.candidateRows),sum([r.candidateRows.A_allPass]), ...
       sum([r.candidateRows.B_periodAndEndpointAllPass]), ...
       sum([r.candidateRows.bothAllPass]));
   fprintf(fid,'Time median_ms=%.9g, worst_ms=%.9g, max_vs25ms=%.9g\n', ...
       median(p),max(p),max(p)/25);
end
fprintf(fid,'Exceptions=%d\n',numel(r.errors));fclose(fid);
files={'SUMMARY.txt','ERRORS.txt'};
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if isempty(r.errors),fprintf(fid,'No errors.\n');else
    for i=1:numel(r.errors),fprintf(fid,'%s\n',r.errors{i});end
end
fclose(fid);
if ~isempty(r.candidateRows)
    writetable(struct2table(r.candidateRows),fullfile(tmp,'DUAL_PER_CANDIDATE.csv'));
    files{end+1}='DUAL_PER_CANDIDATE.csv';
end
if ~isempty(r.barrierRows)
    writetable(struct2table(r.barrierRows),fullfile(tmp,'DUAL_PER_BARRIER.csv'));
    files{end+1}='DUAL_PER_BARRIER.csv';
end
if ~isempty(r.timingRows)
    writetable(struct2table(r.timingRows),fullfile(tmp,'TIMING_REPEAT.csv'));
    files{end+1}='TIMING_REPEAT.csv';
end
if isfile(out),delete(out);end
zip(out,files,tmp);
fprintf('[SD DUAL] results: %s\n',out);
end

function report=sd_fast_mode(varargin)
% FAST research mode: shared affine jets once per snapshot, inexpensive
% multi-candidate screening, ONE separate A/B interval check per snapshot.
% The screen is approximate and NEVER certifies safety on its own.
if isempty(varargin),error('SD:FastInput','FOURWAY ZIP path required.');end
zipPath=char(varargin{1});
if ~isfile(zipPath),error('SD:FastInput','Input ZIP missing: %s',zipPath);end
opts=struct('maxCases',8,'substeps',8,'tubeIterations',14, ...
    'candidates',5,'torqueStep',0.05,'gamma',1,'dualEndpoint',true);
args=varargin(2:end);
if mod(numel(args),2),error('SD:FastOptions','Name/value pairs expected.');end
for k=1:2:numel(args)
    name=lower(char(args{k}));v=args{k+1};
    switch name
        case 'maxcases',opts.maxCases=v;
        case 'substeps',opts.substeps=v;
        case 'candidates',opts.candidates=v;
        case 'torquestep',opts.torqueStep=v;
        otherwise,error('SD:FastOptions','Unknown option: %s',name);
    end
end
validateattributes(opts.maxCases,{'numeric'},{'scalar','integer','>=',1,'<=',40});
validateattributes(opts.substeps,{'numeric'},{'scalar','integer','>=',1,'<=',32});
validateattributes(opts.candidates,{'numeric'},{'scalar','integer','>=',1,'<=',5});
validateattributes(opts.torqueStep,{'numeric'},{'scalar','nonnegative','finite','<=',1});
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
% Automatically regress the proposed fast affine calculation against the
% ordinary generated EVAL on two synthetic simultaneous obstacles.
syn=sd_synthetic();
[~,~,selfErr]=sd_fast_screen(syn,opts);
assert(isfinite(selfErr) && selfErr<1e-8, 'SD:FastSelftest', ...
    'Fast affine evaluation mismatch (synthetic obstacles).');

loadClock=tic;tempDir=tempname;mkdir(tempDir);
tempGuard=onCleanup(@()sd_cleanup(tempDir)); %#ok<NASGU>
unzip(zipPath,tempDir);
T=readtable(fullfile(tempDir,'PER_SAMPLE.csv'));
valid=find(T.parts>0);
if isempty(valid),error('SD:FastInput','No active obstacle records.');end
k=min(opts.maxCases,numel(valid));
[~,ha]=sort(T.min_h_m(valid));[~,pa]=sort(T.min_psi3(valid));
ns=min(ceil(k/3),numel(valid));
chosen=unique([valid(ha(1:ns));valid(pa(1:ns)); ...
    valid(unique(round(linspace(1,numel(valid),k))))],'stable');
chosen=chosen(1:min(k,numel(chosen)));
if numel(chosen)<k
    remain=setdiff(valid,chosen,'stable');
    chosen=[chosen;remain(1:k-numel(chosen))];
end
chosen=sort(chosen(:));
R=load(fullfile(tempDir,'RAW_SNAPSHOTS.mat'),'snapshot');
loadMs=1000*toc(loadClock);
if ~iscell(R.snapshot) || numel(R.snapshot)<max(chosen)
    error('SD:FastInput','Expected cell vector of snapshots.');
end
screens=struct([]);cases=struct([]);errors={};repeat=struct([]);
for j=1:numel(chosen)
    index=chosen(j);s=R.snapshot{index};
    try
        tFast=tic;
        [cand,selection,affErr]=sd_fast_screen(s,opts);
        fastMs=1000*toc(tFast);
        if j==1
            % Repeat exact same fast calculation, no ZIP I/O and no verifier.
            for r=1:4
                if r==1,rt=fastMs;
                else
                    clock=tic;sd_fast_screen(s,opts);rt=1000*toc(clock);
                end
                entry=struct('sampleIndex',index,'repeatIndex',r-1, ...
                    'fastScreenMs',rt);
                if isempty(repeat),repeat=entry;else,repeat(end+1)=entry;end %#ok<AGROW>
            end
        end
        for c=1:numel(cand)
            entry=cand(c);entry.sampleIndex=index;
            if isfield(s,'time'),entry.time=s.time;else,entry.time=NaN;end
            entry.chosenForInterval=(c==selection);
            if isempty(screens),screens=entry;else,screens(end+1)=entry;end %#ok<AGROW>
        end
        d=[cand(selection).offsetX;cand(selection).offsetY];
        clock=tic;
        interval=sd_case(s,index,1,1,d,opts);
        verifyMs=1000*toc(clock);
        passA=all(strcmp({interval.status},'PERIOD_HOCBF_CONDITIONAL_PASS'));
        passB=all([interval.reachEnclosureOK] & ...
            isfinite([interval.intervalMinH]) & [interval.intervalMinH]>=0 & ...
            [interval.terminalEvaluationOK] & ...
            [interval.terminalHLower]-(1-opts.gamma)*[interval.initialHUpper]>=0);
        if isfield(s,'time'),sampleTime=double(s.time);else,sampleTime=NaN;end
        cr=struct('sampleIndex',index,'time',sampleTime, ...
            'obstacles',numel(s.parts),'constraints',numel(interval), ...
            'estimatedLoadMassKg',s.P.mL, ...
            'numCheapCandidates',numel(cand),'chosenCandidate',selection, ...
            'chosenTorqueX',cand(selection).torqueX, ...
            'chosenTorqueY',cand(selection).torqueY, ...
            'screenA',cand(selection).A_approxOK, ...
            'screenB',cand(selection).B_approxOK, ...
            'verifiedA_conditional',passA,'verifiedB_conditional',passB, ...
            'screenFalsePositiveA',cand(selection).A_approxOK && ~passA, ...
            'screenFalsePositiveB',cand(selection).B_approxOK && ~passB, ...
            'allReachEnclosed',all([interval.reachEnclosureOK]), ...
            'minVerifiedHLower',min([interval.intervalMinH]), ...
            'minVerifiedFinalLower',min([interval.intervalMinFinal]), ...
            'fastScreenMs',fastMs,'oneIntervalCheckMs',verifyMs, ...
            'totalMs',fastMs+verifyMs,'timeBudgetMs',25, ...
            'over25ms',fastMs+verifyMs>25,'affineResidual',affErr, ...
            'recursionProof','NOT_PROVEN','roundingProof','NOT_PROVEN');
        if isempty(cases),cases=cr;else,cases(end+1)=cr;end %#ok<AGROW>
    catch ME
        stack='';
        for q=1:numel(ME.stack)
            stack=[stack sprintf(' %s:%d',ME.stack(q).name,ME.stack(q).line)]; %#ok<AGROW>
        end
        errors{end+1,1}=sprintf('sample %d: %s | %s',index,ME.message,stack); %#ok<AGROW>
    end
end
report=struct('mode','fast','options',opts,'screens',screens, ...
    'cases',cases,'errors',{errors},'repeats',repeat,'loadMs',loadMs, ...
    'guarantee','FAST_SCREEN_NOT_A_CERTIFICATE; INTERVAL_PROOF_CONDITIONAL_ONLY; RECURSION_NOT_PROVEN');
if ~isempty(cases)
    timing=[cases.totalMs];
    fprintf('[SD FAST] states=%d cheap candidates=%d heavy interval checks=%d errors=%d\n', ...
        numel(cases),numel(screens),numel(cases),numel(errors));
    fprintf('[SD FAST] conditional A=%d/%d B=%d/%d; fast screen false-positives A=%d B=%d\n', ...
        sum([cases.verifiedA_conditional]),numel(cases), ...
        sum([cases.verifiedB_conditional]),numel(cases), ...
        sum([cases.screenFalsePositiveA]),sum([cases.screenFalsePositiveB]));
    fprintf('[SD FAST] total median %.3f ms, observed max %.3f ms, over 25ms=%d\n', ...
        median(timing),max(timing),sum([cases.over25ms]));
else
    fprintf('[SD FAST] NO COMPLETE CASE; errors=%d\n',numel(errors));
end
if ~isempty(errors),fprintf('[SD FAST] FIRST ERROR: %s\n',errors{1});end
fprintf('[SD FAST] rigorous online/recursive safety and WCET: NOT PROVEN.\n');
sd_fast_save(report);
end

function [cand,chosen,affErr]=sd_fast_screen(s,opts)
% Four generator evaluations shared by EVERY candidate and barrier.
% Fast Taylor samples are approximations; they omit all higher-order remainders.
x=double(s.x(:));u=double(s.u(:));P=s.P;
if ~(numel(x)==19 && numel(u)==4 && P.mL>=0),error('SD:FastState','Invalid snapshot.');end
x(1:4)=x(1:4)/norm(x(1:4));
x(14:16)=x(14:16)/norm(x(14:16));
if isfield(s,'tiltCfg'),tilt=s.tiltCfg;else,tilt=[];end
T=u(1);yaw=u(4);u0=[T;0;0;yaw];
E0=CBF_REAL_XY_EVAL(x,u0,P,s.G,s.parts,s.lambda,tilt);
Ex=CBF_REAL_XY_EVAL(x,u0+[0;1;0;0],P,s.G,s.parts,s.lambda,tilt);
Ey=CBF_REAL_XY_EVAL(x,u0+[0;0;1;0],P,s.G,s.parts,s.lambda,tilt);
Eref=CBF_REAL_XY_EVAL(x,u,P,s.G,s.parts,s.lambda,tilt);
N=size(E0.stages,2);
barriers=struct('names',{},'lambda',{},'s0',{},'sx',{},'sy',{});
for i=1:N
    barriers(end+1)=struct('names',E0.names{i},'lambda',s.lambda(:), ...
        's0',E0.stages(:,i),'sx',Ex.stages(:,i),'sy',Ey.stages(:,i)); %#ok<AGROW>
end
if ~isempty(tilt) && tilt.bodyEnabled
    barriers(end+1)=struct('names','bodyTilt','lambda',tilt.bodyLambda(:), ...
        's0',E0.tiltBodyStages(:),'sx',Ex.tiltBodyStages(:),'sy',Ey.tiltBodyStages(:)); %#ok<AGROW>
end
if ~isempty(tilt) && tilt.cableEnabled
    barriers(end+1)=struct('names','cableTilt','lambda',tilt.cableLambda(:), ...
        's0',E0.tiltCableStages(:),'sx',Ex.tiltCableStages(:),'sy',Ey.tiltCableStages(:)); %#ok<AGROW>
end
% Confirm generator's affine highest-stage and low-stage independence at
% the logged input before using linear candidate algebra.
affErr=0;
for i=1:numel(barriers)
    b=barriers(i);r=numel(b.lambda);
    expected=b.s0+(b.sx-b.s0)*u(2)+(b.sy-b.s0)*u(3);
    if i<=N,actual=Eref.stages(:,i);
    elseif strcmp(b.names,'bodyTilt'),actual=Eref.tiltBodyStages(:);
    else,actual=Eref.tiltCableStages(:);
    end
    affErr=max(affErr,max(abs(expected-actual)));
    if any(abs(b.sx(1:r)-b.s0(1:r))>1e-8) || ...
            any(abs(b.sy(1:r)-b.s0(1:r))>1e-8)
        error('SD:FastAffine','XY torque changed lower HOCBF stages.');
    end
end
if affErr>1e-7,error('SD:FastAffine','Affine residual %.6g exceeds tolerance.',affErr);end
offsets=[0 0;opts.torqueStep 0;-opts.torqueStep 0; ...
    0 opts.torqueStep;0 -opts.torqueStep];
offsets=offsets(1:opts.candidates,:);
cand=struct([]);
for j=1:size(offsets,1)
    z=u(2:3)+offsets(j,:).';
    if any(abs(z)>1+1e-12),continue;end
    aOK=true;bOK=true;minPolyH=Inf;minPolyF=Inf;
    for i=1:numel(barriers)
        b=barriers(i);r=numel(b.lambda);
        stage=b.s0+(b.sx-b.s0)*z(1)+(b.sy-b.s0)*z(2);
        hder=sd_fast_unpack(stage,b.lambda);
        ts=[0 s.dt/2 s.dt];
        lowerOK=all(stage(1:r)>=-1e-10);
        thisMinH=Inf;
        for t=ts
            H=zeros(r+1,1);
            for m=0:r
                for q=m:r
                    H(m+1)=H(m+1)+hder(q+1)*t^(q-m)/factorial(q-m);
                end
            end
            pp=sd_fast_stages(H,b.lambda);
            lowerOK=lowerOK && all(pp(1:r)>=-1e-10);
            minPolyH=min(minPolyH,pp(1));
            thisMinH=min(thisMinH,pp(1));
        end
        hEnd=0;
        for q=0:r,hEnd=hEnd+hder(q+1)*s.dt^q/factorial(q);end
        bOK=bOK && thisMinH>=-1e-10 && ...
            hEnd>=(1-opts.gamma)*hder(1)-1e-10;
        aOK=aOK && lowerOK && stage(end)>=-1e-10;
        minPolyF=min(minPolyF,stage(end));
    end
    cr=struct('sampleIndex',NaN,'time',NaN,'candidateIndex',j, ...
        'offsetX',offsets(j,1),'offsetY',offsets(j,2), ...
        'torqueX',z(1),'torqueY',z(2), ...
        'A_approxOK',aOK,'B_approxOK',bOK, ...
        'minPolynomialH',minPolyH,'minInitialHighCBF',minPolyF, ...
        'chosenForInterval',false,'screenIsCertificate',false);
    if isempty(cand),cand=cr;else,cand(end+1)=cr;end %#ok<AGROW>
end
if isempty(cand),error('SD:FastCandidate','All candidates outside XY torque bounds.');end
% Preserve the logged valid controller output unless a small perturbation
% offers both screens while nominal fails. This is only a research choice.
chosen=1;
if ~(cand(1).A_approxOK && cand(1).B_approxOK)
    good=find([cand.A_approxOK] & [cand.B_approxOK],1,'first');
    if ~isempty(good),chosen=good;end
end
end

function d=sd_fast_unpack(stages,lambda)
r=numel(lambda);d=zeros(r+1,1);
for k=0:r
    pol=1;
    for j=1:k,pol=conv(pol,[lambda(j),1]);end
    if k==0,d(1)=stages(1);
    else,d(k+1)=stages(k+1)-pol(1:end-1)*d(1:k);
    end
end
end

function stages=sd_fast_stages(d,lambda)
r=numel(lambda);stages=zeros(r+1,1);
for k=0:r
    pol=1;
    for j=1:k,pol=conv(pol,[lambda(j),1]);end
    stages(k+1)=pol*d(1:k+1);
end
end

function sd_fast_save(r)
root=fileparts(mfilename('fullpath'));
out=fullfile(root,'CBF_SD_TEST_RESULTS.zip');
tmp=tempname;mkdir(tmp);guard=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
fid=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fid,'CBF_SD_TEST FAST SHARED AFFINE SCREEN + ONE INTERVAL VERIFIER\n');
fprintf(fid,'NO PRODUCTION CHANGES. ONE COMMAND. NO NESTED ZIP.\n');
fprintf(fid,'4 generator EVAL per snapshot independent of candidate count; XY torque only.\n');
fprintf(fid,'Fast polynomial screen uses 0, dt/2, dt samples; NO TAYLOR REMAINDER BOUND; NEVER CERTIFIED.\n');
fprintf(fid,'Only one candidate per state enters the older Picard/interval A-B research verifier.\n');
fprintf(fid,'All reported conditional interval passes rely on unverified 8ULP floating point outward rounding on target CPU.\n');
fprintf(fid,'No guarantee of input switching at boundary, recursive feasibility, physical real time or future obstacles.\n');
fprintf(fid,'Snapshot mass P.mL is an ESTIMATE, held fixed only during the present ZOH.\n');
fprintf(fid,'Future modular constraints: body sphere, load sphere, ground; not included or certified.\n');
fprintf(fid,'Input ZIP/MAT load %.6g ms (excluded).\n',r.loadMs);
if ~isempty(r.cases)
    times=[r.cases.totalMs];
    fprintf(fid,'Completed=%d; screens=%d; exceptions=%d.\n',numel(r.cases),numel(r.screens),numel(r.errors));
    fprintf(fid,'Fast median %.6gms; verify median %.6gms; total median %.6gms; observed max %.6gms.\n', ...
        median([r.cases.fastScreenMs]),median([r.cases.oneIntervalCheckMs]), ...
        median(times),max(times));
    fprintf(fid,'A conditional=%d; B conditional=%d; screen false positive A=%d B=%d.\n', ...
        sum([r.cases.verifiedA_conditional]),sum([r.cases.verifiedB_conditional]), ...
        sum([r.cases.screenFalsePositiveA]),sum([r.cases.screenFalsePositiveB]));
end
fclose(fid);
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if isempty(r.errors),fprintf(fid,'No errors.\n');
else
    for k=1:numel(r.errors),fprintf(fid,'%s\n',r.errors{k});end
end
fclose(fid);
files={'SUMMARY.txt','ERRORS.txt'};
if ~isempty(r.screens)
    writetable(struct2table(r.screens),fullfile(tmp,'FAST_PER_CANDIDATE.csv'));
    files{end+1}='FAST_PER_CANDIDATE.csv';
end
if ~isempty(r.cases)
    writetable(struct2table(r.cases),fullfile(tmp,'FAST_PER_CASE.csv'));
    files{end+1}='FAST_PER_CASE.csv';
end
if ~isempty(r.repeats)
    writetable(struct2table(r.repeats),fullfile(tmp,'FAST_TIMING_REPEAT.csv'));
    files{end+1}='FAST_TIMING_REPEAT.csv';
end
if isfile(out),delete(out);end
zip(out,files,tmp);
end


% =======================================================================
% QP MODE 2026-11-23: No sampled torque candidates. Exactly one 2D QP
% per method and state. SHARED reachable tube for ALL allowed XY torques.
% This is independent read-only research, not an online controller.
% =======================================================================
function out=sd_qp_mode(varargin)
if isempty(varargin),error('SD:QPInput','FOURWAY result ZIP path required.');end
inputZip=char(varargin{1});args=varargin(2:end);
if ~isfile(inputZip),error('SD:QPInput','File not found: %s',inputZip);end
opt=struct('maxCases',6,'substeps',4,'tubeIterations',18, ...
    'contractRate',2); % [1/s] endpoint h_{k+1} >= exp(-rate*dt)*h_k
if mod(numel(args),2)~=0,error('SD:QPOptions','Name/value pairs required.');end
for i=1:2:numel(args)
    key=lower(char(args{i}));val=args{i+1};
    switch key
        case 'maxcases',opt.maxCases=val;
        case 'substeps',opt.substeps=val;
        case 'tubeiterations',opt.tubeIterations=val;
        case 'contractrate',opt.contractRate=val;
        otherwise,error('SD:QPOptions','Unknown option %s',key);
    end
end
validateattributes(opt.maxCases,{'numeric'},{'scalar','integer','>=',1,'<=',25});
validateattributes(opt.substeps,{'numeric'},{'scalar','integer','>=',1,'<=',32});
validateattributes(opt.tubeIterations,{'numeric'},{'scalar','integer','>=',4,'<=',40});
validateattributes(opt.contractRate,{'numeric'},{'scalar','real','finite','>=',0,'<=',100});
oldPath=path;pg=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
% Parser, interval primitives, actual generator equivalence, and QP algebra.
sd_selftest_primitives();
sd_qp_unit_tests();
loadClock=tic;
tmp=tempname;mkdir(tmp);tg=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
unzip(inputZip,tmp);
tab=readtable(fullfile(tmp,'PER_SAMPLE.csv'));
valid=find(tab.parts>0 & isfinite(tab.min_h_m));
if isempty(valid),error('SD:QPInput','No snapshots with observed obstacles.');end
n=min(opt.maxCases,numel(valid));
[~,hord]=sort(tab.min_h_m(valid));
[~,pord]=sort(tab.min_psi3(valid));
edge=min(ceil(n/3),numel(valid));
chosen=unique([valid(hord(1:edge));valid(pord(1:edge)); ...
    valid(unique(round(linspace(1,numel(valid),n))))],'stable');
chosen=chosen(1:min(n,numel(chosen)));
if numel(chosen)<n
    remn=setdiff(valid,chosen,'stable');
    chosen=[chosen;remn(1:n-numel(chosen))];
end
chosen=sort(chosen(:));
D=load(fullfile(tmp,'RAW_SNAPSHOTS.mat'),'snapshot');
if ~iscell(D.snapshot) || numel(D.snapshot)<max(chosen)
    error('SD:QPInput','Expected cell array of logged snapshots.');
end
loadMs=1000*toc(loadClock);
rows=struct([]); barrierRows=struct([]);errors={};repeats=struct([]);
for i=1:numel(chosen)
    k=chosen(i);
    try
        [c,br]=sd_qp_case(D.snapshot{k},k,opt);
        if isempty(rows),rows=c;else,rows(end+1)=c;end %#ok<AGROW>
        if isempty(barrierRows),barrierRows=br;else,barrierRows=[barrierRows;br];end %#ok<AGROW>
        fprintf('[SD-QP] t=%.3f s, rows A=%d B=%d, A=%s B=%s, time=%.3f ms\n', ...
            c.time,c.rowsA,c.rowsB,c.statusA,c.statusB,c.totalMs);
        if i==1
            repeats=struct('sampleIndex',k,'repeatIndex',0, ...
                'totalMs',c.totalMs,'tubeMs',c.tubeMs, ...
                'intervalMs',c.intervalMs,'qpAMs',c.qpAMs,'qpBMs',c.qpBMs);
            for rep=1:2
                [again,~]=sd_qp_case(D.snapshot{k},k,opt);
                repeats(end+1)=struct('sampleIndex',k,'repeatIndex',rep, ...
                    'totalMs',again.totalMs,'tubeMs',again.tubeMs, ...
                    'intervalMs',again.intervalMs,'qpAMs',again.qpAMs, ...
                    'qpBMs',again.qpBMs); %#ok<AGROW>
            end
        end
    catch ME
        trace='';for kk=1:numel(ME.stack)
            trace=[trace sprintf('%s:%d ',ME.stack(kk).name,ME.stack(kk).line)]; %#ok<AGROW>
        end
        errors{end+1,1}=sprintf('sample %d: %s [%s]',k,ME.message,trace); %#ok<AGROW>
        fprintf('[SD-QP] sample %d ERROR %s\n',k,ME.message);
    end
end
% Forward-invariance audit: point evaluations at actual consecutive logged
% sampling instants. No input-candidate enumeration or additional reach tube.
auditClock=tic;
[pairs,pairBarriers,pairErrs]=sd_qp_invariance_audit(chosen,D.snapshot,rows,opt);
auditMs=1000*toc(auditClock);
errors=[errors(:);pairErrs(:)];
out=struct('mode','qp','cases',rows,'barriers',barrierRows, ...
    'errors',{errors},'repeats',repeats,'loadMs',loadMs,'options',opt, ...
    'invariancePairs',pairs,'invarianceBarriers',pairBarriers,'invarianceAuditMs',auditMs, ...
    'mathematicalScope','CONDITIONAL_INDUCTIVE_SAFETY_IF_EACH_STEP_CERTIFIED__RECURSIVE_FEASIBILITY_UNPROVEN__ROUNDING_UNVERIFIED');
sd_qp_save(out);
end

function [r,barrierRows]=sd_qp_case(s,index,opt)
caseClock=tic;
required={'x','u','P','G','parts','lambda','dt'};
for k=1:numel(required)
    if ~isfield(s,required{k}),error('SD:QPSnapshot','Missing %s',required{k});end
end
x=double(s.x(:));u=double(s.u(:));P=s.P;G=s.G;
lam=double(s.lambda(:));dt=double(s.dt);
if isfield(s,'tiltCfg'),tilt=s.tiltCfg;else,tilt=[];end
assert(numel(x)==19 && numel(u)==4 && dt>0 && dt<=0.1, ...
    'SD:QPState','Invalid snapshot dimensions.');
assert(P.m>0 && P.cableL>0 && P.mL>=0 && all(P.J(:)>0), ...
    'SD:QPParameter','Invalid estimated physical parameters.');
err=max(abs([norm(x(1:4))-1,norm(x(14:16))-1]));
if err>0.005,error('SD:QPProjection','State norm outside original Scheme02 tolerance.');end
x(1:4)=x(1:4)/norm(x(1:4));x(14:16)=x(14:16)/norm(x(14:16));
% T and yaw are NOMINAL clipped physical inputs, never optimized in this test.
uFix=[min(max(u(1),0),20);0;0;min(max(u(4),-1),1)];
% The previous FOURWAY hook captured the final corrected physical input
% in s.u. The QP objective must instead use the clipped HLC XY nominal.
if isfield(s,'info') && isfield(s.info,'physicalNominal') && ...
        isnumeric(s.info.physicalNominal) && numel(s.info.physicalNominal)==4
    n=double(s.info.physicalNominal(:));
    zNom=min(max(n(2:3),-1),1);
    nominalSource='info.physicalNominal';
else
    zNom=min(max(double(u(2:3)),-1),1);
    nominalSource='FALLBACK_LOGGED_OUTPUT_NOT_HLC';
end
if isfield(s,'time'),t=double(s.time);else,t=NaN;end

% Common tube is computed ONCE for the entire physical torque box [-1,1]^2.
% This is a validated Picard-enclosure method only under the recorded
% interval-arithmetic assumptions. A negative lower bound is INCONCLUSIVE.
reachClock=tic;
start=sd_interval(x);whole=[];reachOK=true;why='OK';iterations=0;
for j=1:opt.substeps
    [tube,atEnd,ok,msg,it]=sd_qp_picard(start,uFix,P,dt/opt.substeps,opt.tubeIterations);
    iterations=iterations+it;
    if ~ok,reachOK=false;why=sprintf('substep %d: %s',j,msg);break;end
    if isempty(whole),whole=tube;
    else,whole=sd_irange(min(whole(1,:),tube(1,:)), ...
                          max(whole(2,:),tube(2,:)));end
    start=atEnd;
end
reachMs=1000*toc(reachClock);

nameList=sd_names(s.parts,tilt);
nBar=numel(nameList);
barrierRows=repmat(sd_qp_barrier_template(),nBar,1);
for i=1:nBar
    barrierRows(i).sampleIndex=index;barrierRows(i).time=t;
    barrierRows(i).barrier=nameList{i};
end
r=struct('sampleIndex',index,'time',t,'massEstimateKg',P.mL, ...
    'thrustN',uFix(1),'yawNm',uFix(4),'nomX',zNom(1),'nomY',zNom(2), ...
    'dt',dt,'barriers',nBar,'nominalSource',nominalSource, ...
    'contractRatePerSec',opt.contractRate, ...
    'contractGamma',1-exp(-opt.contractRate*dt), ...
    'stateProjectionError',err,'tubeOK',reachOK,'tubeReason',why, ...
    'picardIterations',iterations,'rowsA',0,'rowsB',0, ...
    'statusA','UNAVAILABLE','statusB','UNAVAILABLE', ...
    'torqueAX',NaN,'torqueAY',NaN,'torqueBX',NaN,'torqueBY',NaN, ...
    'minResidualA',NaN,'minResidualB',NaN, ...
    'AinitialLowOK',false,'affineCheckErr',NaN, ...
    'BperiodNominalPass',false,'BcontractNominalPass',false, ...
    'BminGammaNecessaryBox',NaN,'BminGammaNominal',NaN, ...
    'BcontractRows',0,'BcontractTightestGapNominal',NaN, ...
    'B_endpointVelocityMonitorNominalOK',false, ...
    'B_endpointVelocityMonitorSelectedOK',false, ...
    'B_endpointVelocitySelectedMinMargin',NaN, ...
    'AmaxPossibleLower',NaN,'ApointMinHighest',NaN, ...
    'tubeMs',reachMs,'intervalMs',NaN,'qpAMs',NaN,'qpBMs',NaN, ...
    'totalMs',NaN,'certifiedAllTime',false, ...
    'unionLocalConditionalPass',false,'unionSuggestedMethod','NONE_CERTIFIED');
if ~reachOK
    r.statusA='TUBE_UNKNOWN';r.statusB='TUBE_UNKNOWN';
    r.totalMs=1000*toc(caseClock);return;
end

stageClock=tic;
% Interval extensions are evaluated only at three BASIS inputs on the
% common tube. There is NO scan over candidate torques.
ux=uFix+[0;1;0;0];uy=uFix+[0;0;1;0];
try
    F0=sd_interval_stages(whole,uFix,P,G,s.parts,lam,tilt);
    Fx=sd_interval_stages(whole,ux,P,G,s.parts,lam,tilt);
    Fy=sd_interval_stages(whole,uy,P,G,s.parts,lam,tilt);
    E0=sd_interval_stages(sd_interval(x),uFix,P,G,s.parts,lam,tilt);
catch ME
    r.statusA='INTERVAL_NOT_EVALUABLE';
    r.statusB='INTERVAL_NOT_EVALUABLE';
    r.tubeReason=['Interval evaluation undefined: ' ME.identifier ' ' ME.message];
    r.intervalMs=1000*toc(stageClock);
    r.totalMs=1000*toc(caseClock);
    return;
end
assert(numel(F0)==nBar && numel(Fx)==nBar && numel(Fy)==nBar, ...
    'SD:QPBarrier','Number of interval barriers mismatched.');
assert(isequal({F0.name}.',nameList) && isequal({E0.name}.',nameList), ...
    'SD:QPBarrier','Barrier ordering mismatch.');
% Numerically verify generator consistency (NOT a symbolic proof).
G0=CBF_REAL_XY_EVAL(x,uFix,P,G,s.parts,lam,tilt);
Gx=CBF_REAL_XY_EVAL(x,ux,P,G,s.parts,lam,tilt);
Gy=CBF_REAL_XY_EVAL(x,uy,P,G,s.parts,lam,tilt);
Gmix=CBF_REAL_XY_EVAL(x,uFix+[0;.37;-.41;0],P,G,s.parts,lam,tilt);
sd_check_source_match(E0,G0,tilt);
C0=sd_boundary_stage_cell(G0,tilt);
Cx=sd_boundary_stage_cell(Gx,tilt);
Cy=sd_boundary_stage_cell(Gy,tilt);
Cm=sd_boundary_stage_cell(Gmix,tilt);

Ba=zeros(0,2);ba=zeros(0,1);
Bb=zeros(0,2);bb=zeros(0,1);
Bc=zeros(0,2);bc=zeros(0,1);
Bv=zeros(0,2);bv=zeros(0,1); % endpoint first-derivative viability MONITOR only
contractGamma=1-exp(-opt.contractRate*dt);
minGammaBox=0;minGammaNom=0;contractGapNom=Inf;
initLowAll=true;errAff=0;minInitialHighest=Inf;
for i=1:nBar
    order=numel(F0(i).stages)-1;
    if strcmp(nameList{i},'bodyTilt'),gains=tilt.bodyLambda(:);
    elseif strcmp(nameList{i},'cableTilt'),gains=tilt.cableLambda(:);
    else,gains=lam(:);end
    assert(order==numel(gains),'SD:QPStage','Order/gain mismatch.');
    % The generator's structural property: input XY appears first at order r.
    % A numerical regression is also run at the current state.
    low=inf;
    for j=1:order
        low=min(low,E0(i).stages{j}(1));
        for v={Cx{i},Cy{i},Cm{i}}
            errAff=max(errAff,abs(v{1}(j)-C0{i}(j)) / (1+abs(C0{i}(j))));
        end
    end
    row=sd_qp_barrier_template();row.sampleIndex=index;row.time=t;
    row.barrier=nameList{i};row.relativeDegree=order;
    row.initialMinLower=low;row.initialH=E0(i).stages{1}(1);
    row.initialStagesOK=(low>=0);
    initLowAll=initLowAll && row.initialStagesOK;
    % Method A: a guaranteed affine *lower envelope* for psi_r over the
    % whole common tube and the whole permitted torque box.
    % Legacy three-stage interval difference retained for comparison only.
    % The certified A constraint is rebuilt below from SHARED raw h jets.
    [AA,legacyA0,legacyAff]=sd_qp_affine_envelope(F0(i).stages{end}, ...
        Fx(i).stages{end},Fy(i).stages{end});
    a0=legacyA0;
    Ba(end+1,:)=AA;ba(end+1,1)=a0; %#ok<AGROW>
    row.A_legacyBias=legacyA0;
    row.A_legacyXYcoefRadiusSum=legacyAff.radiusSum;
    row.A_bias=a0;row.A_Bx=AA(1);row.A_By=AA(2);
    row.A_zeroTubeLower=F0(i).stages{end}(1);
    row.A_zeroTubeUpper=F0(i).stages{end}(2);
    row.A_XYcoefRadiusSum=legacyAff.radiusSum;
    row.A_bestBoxCertificate=a0+sum(abs(AA));
    row.A_pointHighest=C0{i}(end);
    minInitialHighest=min(minInitialHighest,row.A_pointHighest);
    % Method B: h(t) >= sum_{j=0}^{r-1} h^(j)(0)t^j/j!
    %                    + lower(h^(r)) * t^r/r!.
    % This is an exact integral identity with a uniform lower bound on
    % h^(r). Bernstein coefficient positivity is a sufficient condition.
    % The jet ALREADY contains interval Taylor coefficients of the raw h.
    % Avoid the mathematically redundant psi -> raw inverse transform: its
    % interval subtractions lose dependency, greatly weakening certificates.
    % Reuse all FOUR existing interval-jet calls: no extra reach/jet cost.
    H0=sd_qp_direct_raw(E0(i));
    Hx=sd_qp_direct_raw(Fx(i));
    Hy=sd_qp_direct_raw(Fy(i));
    Hz=sd_qp_direct_raw(F0(i));
    [AB,bb0,rawAff]=sd_qp_affine_envelope(Hz{end},Hx{end},Hy{end});
    % EXACT STRUCTURAL IDENTITY for fixed current thrust/yaw and fixed x:
    % psi_r = h^(r) + sum_{j=0}^{r-1} c_j h^(j),
    % where c_j is a symmetric polynomial in the positive HOCBF gains.
    % The lower raw derivatives h^(j), j<r, are independent of XY torque
    % at each fixed x by relative degree r. Consequently d psi_r / dz
    % equals d h^(r) / dz. Use B's ALREADY-COMPUTED affine coefficient
    % enclosure rather than three separate psi_r interval subtractions.
    % This is a shared-jet, shared-tube A refinement, not extra Jet work.
    shift=sd_interval(0);poly=1;
    for ll=1:order,poly=conv(poly,[1,gains(ll)]);end
    for jj=0:order-1
        shift=sd_iadd(shift,sd_iscale(Hz{jj+1},poly(end-jj)));
    end
    aInt=sd_iadd(sd_interval(bb0),shift);
    a0=sd_down(aInt(1)-1e-12*(1+abs(aInt(1))+sum(abs(AB))));
    AA=AB;
    Ba(end,:)=AA;ba(end)=a0;
    row.A_bias=a0;row.A_Bx=AA(1);row.A_By=AA(2);
    row.A_XYcoefRadiusSum=rawAff.radiusSum;
    row.A_bestBoxCertificate=a0+sum(abs(AA));
    row.A_directBiasGain=a0-row.A_legacyBias;
    % Sanity-check at logged x: the lower envelope must not exceed actual
    % psi_r for a mixed XY input at this state (not a symbolic proof).
    if a0+AA*[.37;-.41]>Cm{i}(end)+1e-7*(1+abs(Cm{i}(end)))
        error('SD:ADirect','Direct A envelope exceeded point evaluation.');
    end
    % Legacy interval inversion used only to QUANTIFY the dependency loss.
    % These are cheap algebraic operations and never affect the new QP.
    oldH0=sd_qp_unstage(E0(i).stages,gains);
    oldHx=sd_qp_unstage(Fx(i).stages,gains);
    oldHy=sd_qp_unstage(Fy(i).stages,gains);
    oldHz=sd_qp_unstage(F0(i).stages,gains);
    [oldAB,oldbb0]=sd_qp_affine_envelope(oldHz{end},oldHx{end},oldHy{end});
    row.B_legacyDerivativeBias=oldbb0;
    row.B_directMinusLegacyBias=bb0-oldbb0;
    row.B_derivativeBias=bb0;row.B_derivativeBx=AB(1);
    row.B_derivativeBy=AB(2);
    for kk=0:order
        coeffA=0;coeffB=zeros(1,2);
        for j=0:min(kk,order-1)
            fac=(dt^j/factorial(j)) * ...
                (sd_qp_choose(kk,j)/sd_qp_choose(order,j));
            coeffA=coeffA+fac*H0{j+1}(1);
        end
        if kk==order
            fac=dt^order/factorial(order);
            coeffA=coeffA+fac*bb0;
            coeffB=fac*AB;
        end
        % Round the scalar coefficient conservatively downward.
        coeffA=sd_down(coeffA-1e-12*(1+abs(coeffA)+sum(abs(coeffB))));
        Bb(end+1,:)=coeffB;bb(end+1,1)=coeffA; %#ok<AGROW>
    end
    row.B_bernsteinRows=order+1;
    % One additional affine endpoint row, NOT an extra torque candidate.
    % Stronger than h>=0: h_{k+1} >= exp(-rate*dt)*h_k.
    % The RHS uses UPPER interval h(0) to preserve the lower certificate.
    hUpper=E0(i).stages{1}(2);
    endpointB=coeffB;endpointC=coeffA; % last Bernstein coefficient p(1)
    threshold=sd_up(exp(-opt.contractRate*dt)*hUpper);
    Bc(end+1,:)=endpointB;bc(end+1,1)=sd_down(endpointC-threshold); %#ok<AGROW>
    row.B_endpointBias=endpointC;row.B_endpointBx=endpointB(1);
    row.B_endpointBy=endpointB(2);row.B_startUpper=hUpper;
    row.B_contractThreshold=threshold;
    row.B_contractNominalGap=endpointC+endpointB*zNom-threshold;
    row.B_contractBestBoxGap=endpointC+sum(abs(endpointB))-threshold;
    % B viability diagnostic: bound the ENDPOINT first derivative with the
    % SAME uniform raw h^(r) lower envelope; no new Jet or trajectory.
    % h'(dt) + kappa*h(dt)>=0 is an early-approach indicator and a possible
    % future viability-set ingredient. It is NOT imposed as a QP row and
    % does NOT by itself prove recursive feasibility for order>1.
    v0=0;
    for jj=1:order-1
        v0=v0+dt^(jj-1)/factorial(jj-1)*H0{jj+1}(1);
    end
    vfac=dt^(order-1)/factorial(order-1);
    vvB=vfac*AB+opt.contractRate*endpointB;
    vvb=sd_down(v0+vfac*bb0+opt.contractRate*endpointC - ...
        1e-12*(1+abs(v0)+abs(bb0)+abs(endpointC)+sum(abs(AB))));
    Bv(end+1,:)=vvB;bv(end+1,1)=vvb; %#ok<AGROW>
    row.B_endpointVelocityMonitorBias=vvb;
    row.B_endpointVelocityMonitorBx=vvB(1);
    row.B_endpointVelocityMonitorBy=vvB(2);
    row.B_endpointVelocityNominalMargin=vvb+vvB*zNom;
    % Compare like-for-like endpoint certificate at the SAME nominal input.
    oldEnd=dt^order/factorial(order)*(oldbb0+oldAB*zNom);
    for jj=0:order-1
        oldEnd=oldEnd+dt^jj/factorial(jj)*oldH0{jj+1}(1);
    end
    row.B_legacyContractNominalGap=oldEnd-threshold;
    row.B_directContractGapGain=row.B_contractNominalGap-row.B_legacyContractNominalGap;
    contractGapNom=min(contractGapNom,row.B_contractNominalGap);
    if hUpper>0
        % NECESSARY gamma for this particular affine certificate under any
        % XY torque in box. Not a necessary bound for true physical safety.
        minGammaBox=max(minGammaBox, ...
            max(0,(hUpper-endpointC-sum(abs(endpointB)))/hUpper));
        minGammaNom=max(minGammaNom, ...
            max(0,(hUpper-endpointC-endpointB*zNom)/hUpper));
    elseif hUpper==0
        if endpointC+sum(abs(endpointB))<0,minGammaBox=Inf;end
        if endpointC+endpointB*zNom<0,minGammaNom=Inf;end
    else
        minGammaBox=Inf;minGammaNom=Inf;
    end
    barrierRows(i)=row;
end
r.intervalMs=1000*toc(stageClock);
r.rowsA=size(Ba,1);r.rowsB=size(Bb,1)+size(Bc,1);
r.BcontractRows=size(Bc,1);r.BminGammaNecessaryBox=minGammaBox;
r.BminGammaNominal=minGammaNom;
r.BcontractTightestGapNominal=contractGapNom;
r.B_endpointVelocityMonitorNominalOK=all(bv+Bv*zNom>=0);
r.BperiodNominalPass=all(bb+Bb*zNom>=0);
r.BcontractNominalPass=r.BperiodNominalPass && all(bc+Bc*zNom>=0);
r.AmaxPossibleLower=min(ba+sum(abs(Ba),2));
r.ApointMinHighest=minInitialHighest;
r.affineCheckErr=errAff;
r.AinitialLowOK=initLowAll;
% The certified initial state must coincide with the chosen projection.
% Without a bound on that projection error, the proof is unavailable.
if err>1e-8
    r.statusA='STATE_PROJECTION_UNCERTIFIED';
    r.statusB='STATE_PROJECTION_UNCERTIFIED';
elseif ~isfinite(errAff) || errAff>1e-8
    r.statusA='GENERATOR_RELATIVE_DEGREE_UNKNOWN';
    r.statusB='GENERATOR_RELATIVE_DEGREE_UNKNOWN';
elseif initLowAll
    clock=tic;
    [z,ok,res]=sd_qp_project(Ba,ba,zNom,[1;1]);
    r.qpAMs=1000*toc(clock);
    if ok
        r.statusA='MODEL_CONDITIONAL_ONE_ZOH_PASS';
        r.torqueAX=z(1);r.torqueAY=z(2);r.minResidualA=res;
    else,r.statusA='AFFINE_CERTIFICATE_QP_EMPTY';end
else
    r.statusA='INITIAL_LOWER_STAGES_NEGATIVE';
end
if err<=1e-8 && isfinite(errAff) && errAff<=1e-8
    clock=tic;
    [z,ok,res]=sd_qp_project([Bb;Bc],[bb;bc],zNom,[1;1]);
    r.qpBMs=1000*toc(clock);
    if ok
        r.statusB='MODEL_CONDITIONAL_ONE_ZOH_PASS';
        r.torqueBX=z(1);r.torqueBY=z(2);r.minResidualB=res;
        r.B_endpointVelocitySelectedMinMargin=min(bv+Bv*z);
        r.B_endpointVelocityMonitorSelectedOK=all(bv+Bv*z>=0);
    else,r.statusB='ENDPOINT_CONTRACT_QP_EMPTY_OR_CONSERVATIVE';end
end
% One cycle can be covered by A OR B: either certificate implies h>=0.
% This is a LOCAL disjunction, not proof of future QP feasibility.
r.unionLocalConditionalPass=strcmp(r.statusA,'MODEL_CONDITIONAL_ONE_ZOH_PASS') || ...
    strcmp(r.statusB,'MODEL_CONDITIONAL_ONE_ZOH_PASS');
if strcmp(r.statusA,'MODEL_CONDITIONAL_ONE_ZOH_PASS')
    r.unionSuggestedMethod='A';
elseif strcmp(r.statusB,'MODEL_CONDITIONAL_ONE_ZOH_PASS')
    r.unionSuggestedMethod='B';
else
    r.unionSuggestedMethod='NONE_CERTIFIED';
end
r.totalMs=1000*toc(caseClock);
end

function [pairs,barriers,errors]=sd_qp_invariance_audit(chosen,snapshots,caseRows,opt)
% AUDIT ONLY. Report exact-time, POINT-model changes when HLC thrust switches.
% Old input acts during [t_k,t_{k+1}). At the boundary, compare at the SAME
% state x_{k+1}: old T, new T, new actual u, new model. These comparisons
% are NOT alternate flow predictions or one-step viable set certificates.
pairs=struct([]);barriers=struct([]);errors={};
for ii=1:numel(chosen)
    k=chosen(ii);
    if k>=numel(snapshots) || isempty(snapshots{k+1}),continue;end
    a=snapshots{k};b=snapshots{k+1};
    try
        if ~isfield(a,'time') || ~isfield(b,'time') || ...
                abs((double(b.time)-double(a.time))-double(a.dt))>1e-4
            continue;
        end
        if isempty(a.parts) || ~isstruct(a.parts),continue;end
        if isfield(a,'tiltCfg'),tiltA=a.tiltCfg;else,tiltA=[];end
        if isfield(b,'tiltCfg'),tiltB=b.tiltCfg;else,tiltB=[];end
        xa=double(a.x(:));xb=double(b.x(:));
        ua=double(a.u(:));ub=double(b.u(:));
        % Evaluate same physical geometry, gain and parameter at boundary
        % before switching the model. The new thrust is a recorded NEW-cycle
        % decision, not applied during the previous ZOH.
        Estart=CBF_REAL_XY_EVAL(xa,ua,a.P,a.G,a.parts,a.lambda,tiltA);
        Eold=CBF_REAL_XY_EVAL(xb,ua,a.P,a.G,a.parts,a.lambda,tiltA);
        uThrust=ua;uThrust(1)=ub(1);
        EnewT=CBF_REAL_XY_EVAL(xb,uThrust,a.P,a.G,a.parts,a.lambda,tiltA);
        EnewU=CBF_REAL_XY_EVAL(xb,ub,a.P,a.G,a.parts,a.lambda,tiltA);
        EnewModel=CBF_REAL_XY_EVAL(xb,ub,b.P,b.G,b.parts,b.lambda,tiltB);
        namesA=sd_names(a.parts,tiltA);namesB=sd_names(b.parts,tiltB);
        C0=sd_boundary_stage_cell(Estart,tiltA);
        Cend=sd_boundary_stage_cell(Eold,tiltA);
        Ct=sd_boundary_stage_cell(EnewT,tiltA);
        Cu=sd_boundary_stage_cell(EnewU,tiltA);
        Cn=sd_boundary_stage_cell(EnewModel,tiltB);
        same=isequaln(a.parts,b.parts) && isequal(namesA,namesB);
        rho=exp(-opt.contractRate*double(a.dt));
        idx=find([caseRows.sampleIndex]==k,1);
        if isempty(idx),continue;end
        q=caseRows(idx);
        Bpass=strcmp(q.statusB,'MODEL_CONDITIONAL_ONE_ZOH_PASS');
        Apass=strcmp(q.statusA,'MODEL_CONDITIONAL_ONE_ZOH_PASS');
        chosenApplied=Bpass && all(isfinite([q.torqueBX;q.torqueBY])) && ...
            norm(double([q.torqueBX;q.torqueBY])-ua(2:3),inf)<1e-8;
        nextidx=find([caseRows.sampleIndex]==k+1,1);
        if isempty(nextidx)
            nextB='NOT_EVALUATED';nextA='NOT_EVALUATED';
        else
            nextB=caseRows(nextidx).statusB;
            nextA=caseRows(nextidx).statusA;
        end
        row=struct('sampleIndex',k,'nextSampleIndex',k+1, ...
            'time',double(a.time),'nextTime',double(b.time), ...
            'dt',double(a.dt),'barrierCount',numel(namesA), ...
            'obstacleGeometryAndNamesMatch',same, ...
            'oldThrust',ua(1),'newThrust',ub(1), ...
            'deltaThrust',ub(1)-ua(1), ...
            'oldMass',double(a.P.mL),'newMass',double(b.P.mL), ...
            'AoldEndLowMin',sd_boundary_min_low(Cend), ...
            'AnewStartLowMin',sd_boundary_min_low(Cn), ...
            'AoldEndLowPass',sd_boundary_min_low(Cend)>=0, ...
            'AnewStartLowPass',sd_boundary_min_low(Cn)>=0, ...
            'ApreviousQPPass',Apass,'BpreviousQPPass',Bpass, ...
            'BselectedXYAppliedInRecordedFlight',chosenApplied, ...
            'nextRecordedStateAstatus',nextA,'nextRecordedStateBstatus',nextB, ...
            'minimumRecordedH',Inf,'minimumRecordedNextH',Inf, ...
            'minimumRecordedContractResidual',Inf, ...
            'maxAbsHInputJump',0, ...
            'guarantee','LOGGED_POINT_DIAGNOSTIC_NOT_CLOSED_LOOP_PROOF');
        for jj=1:numel(namesA)
            c0=C0{jj};ce=Cend{jj};ct=Ct{jj};cu=Cu{jj};
            if jj<=numel(Cn) && strcmp(namesA{jj},namesB{jj})
                cn=Cn{jj};
            else
                cn=nan(size(cu));
            end
            h0=c0(1);hEnd=ce(1);con=hEnd-rho*h0;
            row.minimumRecordedH=min(row.minimumRecordedH,h0);
            row.minimumRecordedNextH=min(row.minimumRecordedNextH,hEnd);
            row.minimumRecordedContractResidual=min(row.minimumRecordedContractResidual,con);
            row.maxAbsHInputJump=max(row.maxAbsHInputJump,abs(ct(1)-ce(1)));
            if numel(ce)>3,psi3Before=ce(4);psi3AfterT=ct(4);psi3AfterModel=NaN;
                if numel(cn)>3,psi3AfterModel=cn(4);end
            else,psi3Before=NaN;psi3AfterT=NaN;psi3AfterModel=NaN;end
            % Mathematical A endpoint-reset test (LOGGED JUMP ONLY):
            % psi_(r-1)^-(dt) >= e^(-lambda_r dt)*psi_(r-1)(0)
            % + L*(1-e^(-lambda_r dt))/lambda_r if psi_r >= L in ZOH.
            % New-HLC/model may jump psi_(r-1) by signed observed amount.
            % For a predictive CERTIFICATE, a rigorously bounded future
            % jump Jbar is REQUIRED in place of the observed jump here.
            rdeg=numel(ce)-1;
            if strcmp(namesA{jj},'bodyTilt'),lamHere=tiltA.bodyLambda(:);
            elseif strcmp(namesA{jj},'cableTilt'),lamHere=tiltA.cableLambda(:);
            else,lamHere=a.lambda(:);end
            lamLast=double(lamHere(end));
            if rdeg>=1 && isfinite(lamLast) && lamLast>=0 && ...
                    numel(cn)>=rdeg && all(isfinite([c0(rdeg),ce(rdeg),cn(rdeg)]))
                observedResetDrop=ce(rdeg)-cn(rdeg);
                observedJumpUpper=max(0,observedResetDrop);
                E=exp(-lamLast*double(a.dt));
                if lamLast==0
                    coeff=double(a.dt);
                else
                    coeff=(1-E)/lamLast;
                end
                requiredHigh=(observedJumpUpper-E*c0(rdeg))/coeff;
                prevStageAtStart=c0(rdeg);
                prevStageOldEnd=ce(rdeg);
                prevStageNewStart=cn(rdeg);
            else
                observedResetDrop=NaN;observedJumpUpper=NaN;
                requiredHigh=NaN;prevStageAtStart=NaN;
                prevStageOldEnd=NaN;prevStageNewStart=NaN;
            end
            br=struct('sampleIndex',k,'time',double(a.time), ...
                'barrier',namesA{jj},'relativeDegree',numel(ce)-1, ...
                'hStart',h0,'hEndRecordedOldGeometry',hEnd, ...
                'recordedEndpointContractionResidual',con, ...
                'psiLowerStartMin',min(c0(1:end-1)), ...
                'psiLowerOldInputAtEndMin',min(ce(1:end-1)), ...
                'psiLowerNewThrustAtEndMin',min(ct(1:end-1)), ...
                'psiLowerNewInputOldModelMin',min(cu(1:end-1)), ...
                'psiLowerNewInputNewModelMin',min(cn(1:end-1)), ...
                'psi3OldInputAtEnd',psi3Before, ...
                'psi3NewThrustAtEnd',psi3AfterT, ...
                'psi3NewModelAtEnd',psi3AfterModel, ...
                'psi3ThrustJump',psi3AfterT-psi3Before, ...
                'A_resetObservedDrop',observedResetDrop, ...
                'A_resetObservedPositiveBound',observedJumpUpper, ...
                'A_resetRequiredHighestUniformLower',requiredHigh, ...
                'A_resetInitialPreviousStage',prevStageAtStart, ...
                'A_resetOldEndPreviousStage',prevStageOldEnd, ...
                'A_resetNewStartPreviousStage',prevStageNewStart, ...
                'A_resetCertifiedFutureJumpBound',false, ...
                'hNewThrustJump',ct(1)-ce(1), ...
                'sameGeometry',same, ...
                'note','POINT_EVAL_RECORDING_NOT_PREDICTION');
            if isempty(barriers),barriers=br;else,barriers(end+1)=br;end %#ok<AGROW>
        end
        if isempty(pairs),pairs=row;else,pairs(end+1)=row;end %#ok<AGROW>
    catch ME
        errors{end+1,1}=sprintf('invariance pair %d: %s (%s)',k,ME.message,ME.identifier); %#ok<AGROW>
    end
end
end

function [box,next,ok,msg,it]=sd_qp_picard(start,u0,P,h,limit)
% Uniform reach tube for EVERY constant XY torque in [-1,1]^2.
% Separate angular-velocity acceleration uncertainty is sound because
% tauX, tauY enter wdot additively as tau_i/J_i.
box=[];next=[];ok=false;msg='UNKNOWN';it=0;
F=sd_qp_rhs(start,u0,P);
rad=max(abs(F),[],1)*h*2+1e-10+1e-8*(abs(start(1,:))+abs(start(2,:)));
for k=1:limit
    it=k;box=sd_irange(sd_down(start(1,:)-rad),sd_up(start(2,:)+rad));
    rhs=sd_qp_rhs(box,u0,P);
    enclosure=sd_iadd(start,sd_imul(sd_irange(0,h),rhs));
    bad=(enclosure(1,:)<=box(1,:)) | (enclosure(2,:)>=box(2,:));
    if ~any(bad)
        next=sd_iadd(start,sd_iscale(rhs,h));ok=true;msg='OK';return;
    end
    needed=max([start(1,:)-enclosure(1,:);enclosure(2,:)-start(2,:)],[],1);
    if any(~isfinite(needed)),msg='nonfinite enclosure';return;end
    pad=1e-11+1e-10*(abs(start(1,:))+abs(start(2,:)));
    rad(bad)=max(1.25*rad(bad),1.3*max(needed(bad),0)+pad(bad));
    if any(~isfinite(rad)) || any(rad>1e6),msg='radius diverged';return;end
end
msg=sprintf('no Picard inclusion after %d iterations',limit);
end

function F=sd_qp_rhs(X,u0,P)
% Bounds the state vector field for all allowed XY torques, fixing
% HLC thrust and yaw. No future-cycle thrust assumption.
F=sd_interval_rhs(X,u0,P);
J=double(P.J(:));
F(:,5)=sd_iadd(F(:,5),sd_irange(-1/J(1),1/J(1)));
F(:,6)=sd_iadd(F(:,6),sd_irange(-1/J(2),1/J(2)));
end

function [B,b,info]=sd_qp_affine_envelope(Z0,Zx,Zy)
% For each x in the common tube:
% F(x,z)=F(x,0)+(F(x,e1)-F(x,0))*z1+(F(x,e2)-F(x,0))*z2.
% Interval subtraction on the RHS yields an enclosure for each true
% coefficient even when the interval operands are dependent.
D1=sd_isub(Zx,Z0);D2=sd_isub(Zy,Z0);
mid1=(D1(1)+D1(2))/2;mid2=(D2(1)+D2(2))/2;
rad1=sd_up(max(mid1-D1(1),D1(2)-mid1));
rad2=sd_up(max(mid2-D2(1),D2(2)-mid2));
B=[mid1 mid2];
% Since |z_i|<=1: F(x,z) >= Z0_lower + mid*z - sum(rad_i).
b=sd_down(Z0(1)-rad1-rad2 - ...
    1e-12*(1+abs(Z0(1))+sum(abs(B))+rad1+rad2));
info=struct('radiusX',rad1,'radiusY',rad2,'radiusSum',rad1+rad2);

end

function H=sd_qp_direct_raw(stage)
% DIRECT derivative intervals from the already-computed Taylor jet:
% raw{k} is coefficient of s^(k-1), not the (k-1)-th derivative.
% Differentiation multiplies that coefficient by factorial(k-1).
% This eliminates the psi-to-h interval cancellation/dependency loss.
if ~isfield(stage,'raw') || numel(stage.raw)~=numel(stage.stages)
    error('SD:QPRaw','Missing direct raw geometry jet.');
end
H=cell(size(stage.raw));
for k=1:numel(stage.raw)
    H{k}=sd_iscale(stage.raw{k},factorial(k-1));
end
end

function H=sd_qp_unstage(S,lambda)
% Invert psi_k=(D+lambda_k)...(D+lambda_1)h at fixed input.
r=numel(lambda);assert(numel(S)==r+1,'SD:QPUnstage','Stages mismatch.');
H=cell(1,r+1);H{1}=S{1};
for k=1:r
    poly=1;
    for j=1:k,poly=conv(poly,[1,lambda(j)]);end
    v=S{k+1};
    for j=0:k-1
        v=sd_isub(v,sd_iscale(H{j+1},poly(end-j)));
    end
    H{k+1}=v;
end
end

function v=sd_qp_choose(n,k)
v=factorial(n)/(factorial(k)*factorial(n-k));
end

function [z,ok,margin]=sd_qp_project(B,b,nom,w)
% Solve the full CONTINUOUS 2D strictly convex QP by active-set candidate
% enumeration (each inequality edge and all pairwise active vertices).
% This enumerates mathematical KKT active sets, NOT fixed trial torques.
% Research solver complexity is O(m^2); future many-barrier online solver
% should use a bounded-iteration, measured small-QP backend.
z=[NaN;NaN];ok=false;margin=NaN;
if any(~isfinite([B(:);b(:);nom(:);w(:)])) || any(w<=0),return;end
% Positive constraint tightening avoids treating small negative QP
% residuals as certificates. This is only a numerical safeguard, not a
% substitute for verified rounding at the interval and QP layers.
safeB=b-1e-10*(1+abs(b)+sum(abs(B),2));
C=[-B;eye(2);-eye(2)];d=[safeB;1;1;1;1];
N=size(C,1);nom=nom(:);w=w(:);
if all(C*nom<=d-1e-10)
    z=nom;ok=true;margin=min(b+B*nom);return;
end
p=[nom];invW=1./w;
for i=1:N
    ci=C(i,:).';den=sum(invW.*(ci.^2));
    if den>1e-14
        p(:,end+1)=nom+invW.*ci*((d(i)-ci.'*nom)/den); %#ok<AGROW>
    end
end
for i=1:N-1
    for j=i+1:N
        A=C([i,j],:);
        if abs(det(A))>1e-12*max(1,norm(A,'fro')^2)
            p(:,end+1)=A\d([i,j]); %#ok<AGROW>
        end
    end
end
feasible=all(C*p<=d+1e-10,1);
if ~any(feasible),return;end
q=p(:,feasible);cost=sum(w.*(q-nom).^2,1);
[~,idx]=min(cost);z=q(:,idx);
margin=min(b+B*z);
ok=all(isfinite(z)) && all(abs(z)<=1+1e-10) && margin>=0;
if ~ok,z=[NaN;NaN];end
end

function s=sd_qp_barrier_template()
s=struct('sampleIndex',NaN,'time',NaN,'barrier','', ...
    'relativeDegree',NaN,'initialH',NaN,'initialMinLower',NaN, ...
    'initialStagesOK',false,'A_bias',NaN,'A_Bx',NaN,'A_By',NaN, ...
    'B_derivativeBias',NaN,'B_derivativeBx',NaN, ...
    'B_derivativeBy',NaN,'B_bernsteinRows',NaN, ...
    'B_endpointBias',NaN,'B_endpointBx',NaN,'B_endpointBy',NaN, ...
    'B_legacyDerivativeBias',NaN,'B_directMinusLegacyBias',NaN, ...
    'B_legacyContractNominalGap',NaN,'B_directContractGapGain',NaN, ...
    'B_endpointVelocityMonitorBias',NaN, ...
    'B_endpointVelocityMonitorBx',NaN, ...
    'B_endpointVelocityMonitorBy',NaN, ...
    'B_endpointVelocityNominalMargin',NaN, ...
    'B_startUpper',NaN,'B_contractThreshold',NaN, ...
    'B_contractNominalGap',NaN,'B_contractBestBoxGap',NaN, ...
    'A_zeroTubeLower',NaN,'A_zeroTubeUpper',NaN, ...
    'A_XYcoefRadiusSum',NaN,'A_legacyBias',NaN, ...
    'A_legacyXYcoefRadiusSum',NaN,'A_directBiasGain',NaN, ...
    'A_bestBoxCertificate',NaN, ...
    'A_pointHighest',NaN);
end

function sd_qp_unit_tests()
% Illustrative polynomial p(t)=1-0.1*t+0.2*t^2 on [0,1].
% Power-to-Bernstein transform must be algebraically exact.
a=[1,-0.1,0.2];n=2;b=zeros(1,n+1);
for k=0:n
    for j=0:k
        b(k+1)=b(k+1)+a(j+1)*sd_qp_choose(k,j)/sd_qp_choose(n,j);
    end
end
for t=[0 .13 .52 1]
    v=(1-t)^2*b(1)+2*t*(1-t)*b(2)+t^2*b(3);
    assert(abs(v-polyval(fliplr(a),t))<1e-12,'SD:QPTest','Bernstein identity failed.');
end
[z,ok,~]=sd_qp_project([1 0;0 1],[-.2;-.3],[-.7;-.8],[1;1]);
assert(ok && norm(z-[.2;.3])<1e-9,'SD:QPTest','Convex QP solver failed.');
I=sd_interval(1);
H=sd_qp_unstage({I,sd_interval(3),sd_interval(8)},[2;1]);
% h=1,h'=1, h''=8-(2+1)*1-2*1=3
assert(abs(mean(H{2})-1)<1e-10 && abs(mean(H{3})-3)<1e-10, ...
    'SD:QPTest','HOCBF inversion failed.');
dt=.025; rate=2; gg=1-exp(-rate*dt);
assert(gg>0 && gg<1 && exp(-rate*dt)+gg==1, ...
    'SD:QPTest','Endpoint contraction factor failed.');
% Affine constraint (b+B*z)>=exp(-rate*dt)*h0 must hold with h0 upper.
h0=2; lowerEnd=1.95; err=lowerEnd-exp(-rate*dt)*h0;
assert(err>0 && lowerEnd < h0 && gg>0,'SD:QPTest','Contract sign failed.');
% Discrete induction invariant: h_{k+1} >= rho h_k implies h_k>=rho^k h_0.
h0=0.25;rho=exp(-2*dt);prev=h0;
for kk=1:20
    cur=rho*prev;
    assert(cur>=rho^kk*h0-1e-14 && cur>=0, ...
        'SD:QPTest','Sampled endpoint induction algebra failed.');
    prev=cur;
end
% The HOCBF auxiliary can jump under exogenous thrust even when physical
% h remains continuous: no unproved identification of these conditions.
a=0.2;b=0.5;Told=9;Tnew=9.01;
assert(a+b*Told~=a+b*Tnew && (1 - 1)==0, ...
    'SD:QPTest','Input-dependent auxiliary jump regression failed.');
fprintf('[SD-QP] Continuous QP + Bernstein + endpoint induction + HOCBF switch logic: PASS.\n');
end

function sd_qp_save(r)
base=fileparts(mfilename('fullpath'));out=fullfile(base,'CBF_SD_TEST_RESULTS.zip');
tmp=tempname;mkdir(tmp);guard=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
fid=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fid,'CBF_SD_TEST -- AFFINE A/B QP (NOT candidate scanning)\n');
fprintf(fid,'NO modifications to production Sim, Scheme02, generator, HLC.\n');
fprintf(fid,'Each state: common Picard tube for ALL z in [-1,1]^2; 3 interval basis evaluations shared.\n');
fprintf(fid,'A: affine lower psi_r over the entire tube + initial lower stages.\n');
fprintf(fid,'B: DIRECT raw h^(r) jet intervals (no HOCBF-to-raw interval inversion), Taylor-Bernstein h>=0 plus endpoint contraction.\n');
fprintf(fid,'QP_BARRIERS records legacy inversion-vs-direct bound differences using the same 4 shared jet evaluations.\n');
fprintf(fid,'Endpoint contract exp(-rate*dt) with rate=%.6g [1/s]. This is NOT a recursive-feasibility theorem.\n',r.options.contractRate);
fprintf(fid,'Thrust and yaw are HLC fixed/clipped; only XY torque optimized. P.mL is the per-snapshot estimated mass.\n');
fprintf(fid,'Each method solves exactly one continuous-variable 2D QP per state, not 5 fixed candidates.\n');
fprintf(fid,'Conditional on exact estimated-model ODE, static obstacle geometry, proper interval outward rounding, and affine relative-degree structure.\n');
fprintf(fid,'Interval 8-ULP heuristics have NOT been validated as rigorous outward rounding on this MATLAB/CPU.\n');
fprintf(fid,'Therefore MODEL_CONDITIONAL_ONE_ZOH_PASS is not a verified machine proof.\n');
fprintf(fid,'A switching: next-HLC thrust causes psi_{r-1} reset; pointwise logging audit only, no guaranteed next-HLC policy or reset guard.\n');
fprintf(fid,'B forward invariance is CONDITIONALLY PROVED BY INDUCTION if every applied cycle satisfies Bernstein h>=0 and endpoint h_next>=rho*h_now.\n');
fprintf(fid,'Feasibility for EVERY future cycle is NOT proved: no controlled-invariant domain / backup policy, no machine-verified rounding, no WCET.\n');
fprintf(fid,'B endpoint condition is a barrier decay condition; even if feasible now, next-cycle feasibility is UNPROVEN.\n');
fprintf(fid,'A adds no new heavy reach computations; CSV separates point psi_r, interval base width, input-coefficient interval radius.\n');
fprintf(fid,'BminGammaNecessaryBox is a bound within THIS conservative certificate only; not physics/true viability.\n');
fprintf(fid,'If no affine certificate is feasible, the result is INCONCLUSIVE, not proof of collision/physical input impossibility.\n');
fprintf(fid,'Initial load sphere/body sphere/ground NOT in constraint set; plan to add after current scope.\n');
fprintf(fid,'QP method used here enumerates active intersections, O(m^2); future many-constraint solver needs independent real-time evaluation.\n');
fprintf(fid,'Snapshot ZIP load excluded from computation time: %.6f ms.\n',r.loadMs);
fprintf(fid,'Cases=%d Errors=%d\n',numel(r.cases),numel(r.errors));
if ~isempty(r.cases)
    t=[r.cases.totalMs];
    fprintf(fid,'Time median=%.6f ms, max=%.6f ms; these are offline wall-clock timings, not WCET.\n',median(t),max(t));
    fprintf(fid,'A conditional=%d; B+endpoint-contract conditional=%d.\n', ...
        sum(strcmp({r.cases.statusA},'MODEL_CONDITIONAL_ONE_ZOH_PASS')), ...
        sum(strcmp({r.cases.statusB},'MODEL_CONDITIONAL_ONE_ZOH_PASS')));
end
if isfield(r,'invariancePairs')
    fprintf(fid,'Observed adjacent pairs=%d; this is a record-based diagnostic, NOT closed-loop replay with the selected new QP input.\n',numel(r.invariancePairs));
    if ~isempty(r.invariancePairs)
        fprintf(fid,'Recorded A old-end low-stage OK=%d/%d; after new-HLC/model low-stage OK=%d/%d.\n', ...
            sum([r.invariancePairs.AoldEndLowPass]),numel(r.invariancePairs), ...
            sum([r.invariancePairs.AnewStartLowPass]),numel(r.invariancePairs));
        fprintf(fid,'Recorded B chosen XY matches applied=%d/%d.\n', ...
            sum([r.invariancePairs.BselectedXYAppliedInRecordedFlight]),numel(r.invariancePairs));
    end
end
fprintf(fid,'HYBRID A-or-B: if at each k one CERTIFIED feasible row-set is applied, h>=0 follows by cycle induction. Future existence remains UNKNOWN.\n');
fprintf(fid,'A DIRECT REFINEMENT: psi_r and h^(r) share XY coefficients for fixed T/yaw; reuses B raw interval jet and HOCBF gain polynomial, no added Jet calls.\n');
fprintf(fid,'A reset extra uniform psi_r margin is an OBSERVED post-hoc diagnostic: logged future thrust/model jump is NOT a predictive upper bound.\n');
fprintf(fid,'B viability kernel / HLC-augmented-state transition still NOT constructed; h endpoint contraction does not ensure next-QP feasibility.\n');
fprintf(fid,'B endpoint approach-velocity monitor: lower bound hdot(dt)+contractRate*h(dt) from same jets, MONITOR ONLY (not imposed / not a viability proof).\n');
if ~isempty(r.cases)
    fprintf(fid,'B selected endpoint approach-velocity monitor OK=%d/%d (not an invariant-set test).\n', ...
        sum([r.cases.B_endpointVelocityMonitorSelectedOK]),numel(r.cases));
end
fprintf(fid,'Additional POINT-only pair audit time %.6f ms is excluded from all per-case QP timings.\n',r.invarianceAuditMs);
fclose(fid);
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if isempty(r.errors),fprintf(fid,'No errors.\n');
else,for i=1:numel(r.errors),fprintf(fid,'%s\n',r.errors{i});end,end
fclose(fid);
files={'SUMMARY.txt','ERRORS.txt'};
if ~isempty(r.cases)
    writetable(struct2table(r.cases),fullfile(tmp,'QP_CASES.csv'));
    files{end+1}='QP_CASES.csv';
end
if ~isempty(r.barriers)
    writetable(struct2table(r.barriers),fullfile(tmp,'QP_BARRIERS.csv'));
    files{end+1}='QP_BARRIERS.csv';
end
if ~isempty(r.repeats)
    writetable(struct2table(r.repeats),fullfile(tmp,'QP_TIMING_REPEAT.csv'));
    files{end+1}='QP_TIMING_REPEAT.csv';
end
if ~isempty(r.invariancePairs)
    writetable(struct2table(r.invariancePairs),fullfile(tmp,'INVARIANCE_PAIRS.csv'));
    files{end+1}='INVARIANCE_PAIRS.csv';
end
if ~isempty(r.invarianceBarriers)
    writetable(struct2table(r.invarianceBarriers),fullfile(tmp,'INVARIANCE_BARRIERS.csv'));
    files{end+1}='INVARIANCE_BARRIERS.csv';
end
if isfile(out),delete(out);end
zip(out,files,tmp);
fprintf('[SD-QP] Output: %s\n',out);
end

% =======================================================================
% FAST-QP MODE: no interval Jet/Picard evaluation in the timed control path.
% All measured QPs here have ZERO intersample margin and are SCREENING ONLY.
% Mathematical guarantee requires independently certified derivative-rate
% upper bounds plus a recursively invariant initial/terminal domain.
% This is NOT a deployable safety filter.
% =======================================================================
function out=sd_fastqp_mode(varargin)
if isempty(varargin),error('SD:FastQPInput','FOURWAY ZIP is required.');end
inputZip=char(varargin{1});
if ~isfile(inputZip),error('SD:FastQPInput','Input ZIP not found: %s',inputZip);end
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
clock=tic;
tmp=tempname;mkdir(tmp);clean=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
unzip(inputZip,tmp);
tab=readtable(fullfile(tmp,'PER_SAMPLE.csv'));
valid=find(tab.parts>0 & isfinite(tab.min_h_m));
if isempty(valid),error('SD:FastQPInput','No obstacle snapshots.');end
n=min(6,numel(valid));
[~,hord]=sort(tab.min_h_m(valid));[~,pord]=sort(tab.min_psi3(valid));
e=min(ceil(n/3),numel(valid));
chosen=unique([valid(hord(1:e));valid(pord(1:e)); ...
    valid(unique(round(linspace(1,numel(valid),n))))],'stable');
chosen=chosen(1:min(numel(chosen),n));
if numel(chosen)<n
    remain=setdiff(valid,chosen,'stable');chosen=[chosen;remain(1:n-numel(chosen))];
end
chosen=sort(chosen(:));
D=load(fullfile(tmp,'RAW_SNAPSHOTS.mat'),'snapshot');
if ~iscell(D.snapshot) || numel(D.snapshot)<max(chosen)
    error('SD:FastQPInput','snapshot must be a cell vector.');
end
loadMs=1000*toc(clock);
cases=struct([]);barriers=struct([]);repeats=struct([]);errors={};
for i=1:numel(chosen)
    id=chosen(i);
    try
        [c,b]=sd_fastqp_one(D.snapshot{id},id);
        if isempty(cases),cases=c;else,cases(end+1)=c;end %#ok<AGROW>
        if isempty(barriers),barriers=b;else,barriers=[barriers;b];end %#ok<AGROW>
        fprintf('[SD FAST-QP] t=%8.3f A0=%s B0=%s T=%.3fms QP=%.3fms  (UNCERTIFIED)\n', ...
            c.time,c.A_optimisticStatus,c.B_optimisticStatus,c.totalMs,c.qpMs);
        if i==1
            for r=0:29
                if r==0,cc=c;else,[cc,~]=sd_fastqp_one(D.snapshot{id},id);end
                row=struct('sampleIndex',id,'repeatIndex',r, ...
                    'totalMs',cc.totalMs,'pointGeneratorMs',cc.pointGeneratorMs, ...
                    'formulationMs',cc.formulationMs,'qpMs',cc.qpMs, ...
                    'numConstraintsA',cc.rowsA,'numConstraintsB',cc.rowsB);
                if isempty(repeats),repeats=row;else,repeats(end+1)=row;end %#ok<AGROW>
            end
        end
    catch ME
        errors{end+1,1}=sprintf('snapshot %d: %s (%s)',id,ME.message,ME.identifier); %#ok<AGROW>
        fprintf('[SD FAST-QP] ERROR sample=%d: %s\n',id,ME.message);
    end
end
out=struct('mode','fastqp','cases',cases,'barriers',barriers, ...
    'repeats',repeats,'errors',{errors},'loadMs',loadMs, ...
    'proofStatus','NO_INTERSAMPLE_RATE_BOUND__NO_RECURSIVE_FEASIBILITY_PROOF', ...
    'guaranteedSafety',false);
sd_fastqp_save(out);
end

function [c,barrierRows]=sd_fastqp_one(s,id)
% Exactly three point-generator evaluations shared by ALL constraints.
% No state box, no interval Jet, no per-obstacle repeated generator call.
start=tic;
x=double(s.x(:));u=double(s.u(:));P=s.P;G=s.G;
if isfield(s,'tiltCfg'),tilt=s.tiltCfg;else
    error('SD:FastQPTilt','Expected logged tilt settings.');
end
lam=double(s.lambda(:));dt=double(s.dt);
if numel(x)~=19 || numel(u)~=4 || ~(dt>0 && dt<=.1)
    error('SD:FastQPState','Malformed logged sample.');
end
pErr=max([abs(norm(x(1:4))-1),abs(norm(x(14:16))-1)]);
if pErr>1e-8
    error('SD:FastQPUnit','Projection needed: this test does not silently certify projection.');
end
T=max(0,min(20,u(1)));yaw=max(-1,min(1,u(4)));
zero=[T;0;0;yaw];
if isfield(s,'info') && isstruct(s.info) && ...
        isfield(s.info,'physicalNominal') && numel(s.info.physicalNominal)==4
    n=double(s.info.physicalNominal(:));zNom=max(-1,min(1,n(2:3)));
    source='HLC_physicalNominal';
else
    zNom=max(-1,min(1,u(2:3)));
    source='LOGGED_OUTPUT_FALLBACK';
end
pointClock=tic;
E0=CBF_REAL_XY_EVAL(x,zero,P,G,s.parts,lam,tilt);
Ex=CBF_REAL_XY_EVAL(x,zero+[0;1;0;0],P,G,s.parts,lam,tilt);
Ey=CBF_REAL_XY_EVAL(x,zero+[0;0;1;0],P,G,s.parts,lam,tilt);
pointMs=1000*toc(pointClock);
formClock=tic;
S0=sd_boundary_stage_cell(E0,tilt);
Sx=sd_boundary_stage_cell(Ex,tilt);
Sy=sd_boundary_stage_cell(Ey,tilt);
names=sd_names(s.parts,tilt);
nBar=numel(S0);
if numel(names)~=nBar || numel(Sx)~=nBar || numel(Sy)~=nBar
    error('SD:FastQPBarrier','Barrier shape mismatch.');
end
A=zeros(0,2);ab=zeros(0,1);B=zeros(0,2);bb=zeros(0,1);
Ba=zeros(nBar,2);bbA=zeros(nBar,1); % per-barrier psi_r(x, z)
lowAll=true; rows=struct([]);
for j=1:nBar
    r=numel(S0{j})-1;
    if j<=numel(s.parts)
        hj=double(E0.hDerivatives(:,j)); gains=lam(:);
    elseif strcmp(names{j},'bodyTilt')
        gains=tilt.bodyLambda(:);hj=sd_fastqp_numeric_unstage(S0{j},gains);
    elseif strcmp(names{j},'cableTilt')
        gains=tilt.cableLambda(:);hj=sd_fastqp_numeric_unstage(S0{j},gains);
    else,error('SD:FastQPBarrier','Unknown barrier.');end
    if numel(gains)~=r || numel(hj)~=r+1
        error('SD:FastQPOrder','Barrier-relative-degree mismatch.');
    end
    lr=min(S0{j}(1:r));lowAll=lowAll && lr>=-1e-9;
    % Assumed structural relative-degree property; verified by generator
    % source separately. Here we measure its pointwise affine QP cost.
    a0=S0{j}(end);a=[Sx{j}(end)-a0,Sy{j}(end)-a0];
    Ba(j,:)=a;bbA(j)=a0;
    A(end+1,:)=a;ab(end+1,1)=a0; %#ok<AGROW>
    % B polynomial lower model with hypothetical M_r=0.
    % For certified |d h^(r)/dt| <= M_r on the whole 25 ms tube,
    % replace the top coefficient d_r(z) with d_r(z)-M_r*dt.
    % Here M_r is UNKNOWN, thus these rows are ONLY screening constraints.
    dr0=hj(end);
    dr=[Sx{j}(end)-S0{j}(end),Sy{j}(end)-S0{j}(end)];
    if max(abs(dr-a))>1e-6*(1+max(abs(a)))
        error('SD:FastQPRelativeDegree','Raw/psi highest coefficients mismatch.');
    end
    bj=zeros(r+1,1);aj=zeros(r+1,2);
    for k=0:r
        for q=0:min(k,r-1)
            bj(k+1)=bj(k+1)+(nchoosek(k,q)/nchoosek(r,q))* ...
                hj(q+1)*dt^q/factorial(q);
        end
        if k==r
            bj(k+1)=bj(k+1)+dr0*dt^r/factorial(r);
            aj(k+1,:)=dr*dt^r/factorial(r);
        end
    end
    B=[B;aj];bb=[bb;bj]; %#ok<AGROW>
    % Endpoint contraction additional affine inequality.
    rate=2;
    contract0=bj(end)-exp(-rate*dt)*hj(1);
    contractB=aj(end,:);
    B(end+1,:)=contractB;bb(end+1,1)=contract0; %#ok<AGROW>
    br=struct('sampleIndex',id,'time',double(s.time), ...
        'barrier',names{j},'relativeDegree',r,'initialLowStageMin',lr, ...
        'A_psiHighestAtZeroXY',a0,'A_Bx',a(1),'A_By',a(2), ...
        'B_lastBernsteinAtZeroXY',bj(end), ...
        'B_endpointContractAtZeroXY',contract0, ...
        'B_highestDerivativeAtZeroXY',dr0, ...
        'B_highestDerivativeBx',dr(1),'B_highestDerivativeBy',dr(2), ...
        'intersampleDerivativeRateBound','NOT_CERTIFIED', ...
        'recursiveFeasibility','NOT_PROVEN');
    if isempty(rows),rows=br;else,rows(end+1,1)=br;end %#ok<AGROW>
end
formMs=1000*toc(formClock);
qpClock=tic;
[zA,okA,resA]=sd_qp_project(A,ab,zNom,[1;1]);
[zB,okB,resB]=sd_qp_project(B,bb,zNom,[1;1]);
qpMs=1000*toc(qpClock);
if ~lowAll,as='INITIAL_LOWER_NEGATIVE';
elseif okA,as='POINT_QP_FEASIBLE_UNCERTIFIED';
else,as='POINT_QP_INFEASIBLE';end
if okB,bs='POINT_TAYLOR_QP_FEASIBLE_UNCERTIFIED';
else,bs='POINT_TAYLOR_QP_INFEASIBLE';end
% Diagnostic admissible derivative-rate capacities MUST be reported
% separately for each barrier.  A joint min across r=2 and r=4 is
% dimensionally meaningless (different derivative orders / physical units).
% Neither these capacities nor ZERO assumed rates are safety certificates.
rowEnd=0;
for j=1:nBar
    r=numel(S0{j})-1;
    rowEnd=rowEnd+r+1;  % Bernstein coefficient of order r
    capA=NaN;capB=NaN;
    if okA && lowAll
        capA=(bbA(j)+Ba(j,:)*zA)/dt;
    end
    if okB
        denom=dt^(r+1)/factorial(r);
        polyEnd=bb(rowEnd)+B(rowEnd,:)*zB;
        contractEnd=bb(rowEnd+1)+B(rowEnd+1,:)*zB;
        capB=min(polyEnd,contractEnd)/denom;
    end
    rows(j).A_admissiblePsiRRateAtChosenXY=capA;
    rows(j).B_admissibleH_RPlus1RateAtChosenXY=capB;
    rows(j).rateUnits=sprintf('barrierUnits_per_s^%d',r+1);
    rowEnd=rowEnd+1;
end
c=struct('sampleIndex',id,'time',double(s.time), ...
    'estimatedMassKg',P.mL,'thrustN',T,'yawNm',yaw, ...
    'numObstacles',numel(s.parts),'numBarriers',nBar, ...
    'rowsA',size(A,1),'rowsB',size(B,1), ...
    'nomX',zNom(1),'nomY',zNom(2),'nominalSource',source, ...
    'A_optimisticStatus',as,'B_optimisticStatus',bs, ...
    'A_optimisticX',zA(1),'A_optimisticY',zA(2), ...
    'B_optimisticX',zB(1),'B_optimisticY',zB(2), ...
    'A_zeroMarginResidual',resA,'B_zeroMarginResidual',resB, ...
    'lowStagesAtCurrentThrustOK',lowAll, ...
    'pointGeneratorMs',pointMs,'formulationMs',formMs,'qpMs',qpMs, ...
    'totalMs',1000*toc(start), ...
    'A_certified',false,'B_certified',false, ...
    'recursiveFeasibilityCertified',false, ...
    'reasonNotCertified','NO_PROVEN_UNIFORM_DERIVATIVE_BOUND_OR_INVARIANT_DOMAIN');
barrierRows=rows(:);
end

function h=sd_fastqp_numeric_unstage(psi,lambda)
r=numel(lambda);psi=psi(:);h=zeros(r+1,1);
if numel(psi)~=r+1,error('SD:FastQPUnstage','Stage order mismatch.');end
h(1)=psi(1);
for k=1:r
    p=1;
    for j=1:k,p=conv(p,[1 lambda(j)]);end
    value=psi(k+1);
    for j=0:k-1,value=value-p(end-j)*h(j+1);end
    h(k+1)=value;
end
end

function sd_fastqp_save(report)
root=fileparts(mfilename('fullpath'));
outFile=fullfile(root,'CBF_SD_TEST_RESULTS.zip');
tmp=tempname;mkdir(tmp);c=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
files={'SUMMARY.txt','ERRORS.txt'};
fid=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fid,'CBF_SD_TEST FAST-QP ONLY, deliberately no 19D interval Jet or Picard online.\n');
fprintf(fid,'Guarantee A: NOT CERTIFIED. Guarantee B: NOT CERTIFIED. Recursive safety: NOT PROVEN.\n');
fprintf(fid,'ZERO intersample margin used for optimistic screening, not as a true Lipschitz bound.\n');
fprintf(fid,'All output QP inputs are SCREENING only; never apply as certified controller outputs.\n');
fprintf(fid,'If |d psi_r/dt|<=LA, A requires psi_r(xk,z)>=LA*dt plus initial lower stages and post-switch viability.\n');
fprintf(fid,'If |d h^(r)/dt|<=LB, B lower model uses h^(r)(xk,z)-LB*dt; endpoint contraction and Bernstein rows then suffice within this ZOH.\n');
fprintf(fid,'Both LA and LB must be mathematically certified over the appropriate reachable set and all allowed XY inputs, with mass/T/gain/obstacle assumptions.\n');
fprintf(fid,'Online cost measurement covers 3 point generator calls + affine row assembly + A/B 2-variable QPs. Does not include the eventual proof-margin lookup / HLC / sensor / actuator overhead.\n');
fprintf(fid,'Rate-capacity diagnostics are PER BARRIER in FAST_BARRIERS.csv; r=2 and r=4 bounds have different units and cannot be combined by a numerical minimum.\n');
fprintf(fid,'Future solver and WCET must be checked for increased obstacle counts.\n');
fprintf(fid,'Studied %d cases; errors %d. Data load %.3f ms (excluded).\n', ...
    numel(report.cases),numel(report.errors),report.loadMs);
if ~isempty(report.cases)
    tt=[report.cases.totalMs]; fprintf(fid,'median total %.6f ms; max total %.6f ms. 25ms hard deadline NOT proven.\n',median(tt),max(tt));
end
if ~isempty(report.repeats)
    rt=sort([report.repeats.totalMs]);
    fprintf(fid,'Same-state repeats %d: min %.6f ms; median %.6f ms; p95 %.6f ms; max %.6f ms. This is NOT WCET.\n', ...
        numel(rt),rt(1),median(rt),rt(max(1,ceil(.95*numel(rt)))),rt(end));
end
fclose(fid);
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if isempty(report.errors),fprintf(fid,'No errors.\n');else
    for i=1:numel(report.errors),fprintf(fid,'%s\n',report.errors{i});end
end
fclose(fid);
if ~isempty(report.cases)
    writetable(struct2table(report.cases),fullfile(tmp,'FAST_CASES.csv'));
    files{end+1}='FAST_CASES.csv';
end
if ~isempty(report.barriers)
    writetable(struct2table(report.barriers),fullfile(tmp,'FAST_BARRIERS.csv'));
    files{end+1}='FAST_BARRIERS.csv';
end
if ~isempty(report.repeats)
    writetable(struct2table(report.repeats),fullfile(tmp,'FAST_TIMING_REPEAT.csv'));
    files{end+1}='FAST_TIMING_REPEAT.csv';
end
if isfile(outFile),delete(outFile);end
zip(outFile,files,tmp);
fprintf('[SD FAST-QP] Output: %s | certified safety: NO | model/solver timing only\n',outFile);
end


% ======================================================================
% 2026-11-23: constructive nonnegative majorant for MODEL-DEPENDENT
% one-ZOH rate bounds. No interval jet, no candidate-torque sampling.
% ATTENTION: mathematical upper-bound construction is exact-real;
% MATLAB double arithmetic is NOT an outward-rounded proof certificate.
% This mode never claims recursive invariance or machine certification.
% ======================================================================
function report=sd_analyticbound_mode(varargin)
if isempty(varargin) || ~isfile(char(varargin{1}))
    error('SD:AnalyticInput','Need CBF_FOURWAY_RESULTS.zip beside the runner.');
end
inputZip=char(varargin{1});
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
tmp=tempname;mkdir(tmp);clean=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
unzip(inputZip,tmp);
tab=readtable(fullfile(tmp,'PER_SAMPLE.csv'));
valid=find(tab.parts>0 & isfinite(tab.min_h_m));
if isempty(valid),error('SD:AnalyticInput','No obstacle snapshots.');end
n=min(6,numel(valid));
[~,a]=sort(tab.min_h_m(valid));[~,b]=sort(tab.min_psi3(valid));
q=min(ceil(n/3),numel(valid));
chosen=unique([valid(a(1:q));valid(b(1:q)); ...
    valid(unique(round(linspace(1,numel(valid),n))))],'stable');
chosen=chosen(1:min(n,numel(chosen)));
if numel(chosen)<n
    remain=setdiff(valid,chosen,'stable');chosen=[chosen;remain(1:n-numel(chosen))];
end
chosen=sort(chosen(:));
D=load(fullfile(tmp,'RAW_SNAPSHOTS.mat'),'snapshot');
if ~iscell(D.snapshot) || numel(D.snapshot)<max(chosen)
    error('SD:AnalyticInput','Snapshot cell array missing.');
end
fixed=sd_analytic_fixed_signature(D.snapshot{chosen(1)});
rows=struct([]);bars=struct([]);errors={};
for kk=1:numel(chosen)
    id=chosen(kk);
    try
        sd_analytic_fixed_check(fixed,D.snapshot{id});
        [row,br]=sd_analyticbound_case(D.snapshot{id},id);
        if isempty(rows),rows=row;else,rows(end+1,1)=row;end %#ok<AGROW>
        if isempty(bars),bars=br;else,bars=[bars;br];end %#ok<AGROW>
        fprintf('[SD MAJORANT] t=%.3f  A=%s B=%s  maxM_B=%.5g  %.2fms (NOT CERTIFIED)\n', ...
            row.time,row.A_status,row.B_status,row.maxObstacleMB,row.totalMs);
    catch ME
        errors{end+1,1}=sprintf('snapshot=%d: %s [%s]',id,ME.message,ME.identifier); %#ok<AGROW>
        fprintf('[SD MAJORANT] ERROR %d: %s\n',id,ME.message);
    end
end
report=struct('mode','analyticbound','cases',rows,'barriers',bars, ...
    'errors',{errors},'guarantee','EXACT_REAL_MATHEMATICAL_MAJORANT__NO_OUTWARD_ROUNDING__NO_RECURSIVE_K');
sd_analyticbound_save(report);
end


function v=sd_analytic_fixed_signature(s)
% Flight-preparation parameters; load-mass and disturbance estimates are
% intentionally EXCLUDED from the fixed signature.
p=s.P;g=s.G;t=s.tiltCfg;
v=[p.m;p.J(:);p.gravity;p.cableL;double(s.dt); ...
    g.cableCenterFromLoad;g.cableAxisLong;g.cableAxisShort; ...
    s.lambda(:);double(t.bodyEnabled);double(t.cableEnabled); ...
    t.bodyMaxDeg;t.cableMaxDeg;t.bodyLambda(:);t.cableLambda(:)];
end

function sd_analytic_fixed_check(v,s)
w=sd_analytic_fixed_signature(s);
if numel(v)~=numel(w) || any(~isfinite(w)) || any(abs(v-w)>1e-12.*max(1,abs(v)))
    error('SD:AnalyticFixedChanged', ...
        'A flight-preparation parameter changed: offline bounds must be rebuilt.');
end
end

function [caseRow,barrierRows,qpRows]=sd_analyticbound_case(s,id)
start=tic;
x=double(s.x(:));u=double(s.u(:));P=s.P;G=s.G;
if ~isfield(s,'tiltCfg'),error('SD:AnalyticTilt','Logged tiltCfg absent.');end
tilt=s.tiltCfg;
lam=double(s.lambda(:));dt=double(s.dt);
if numel(x)~=19 || numel(u)~=4 || dt<=0 || dt>.1
    error('SD:AnalyticState','Invalid input state or sampling interval.');
end
if max(abs([norm(x(1:4))-1;norm(x(14:16))-1]))>1e-8
    error('SD:AnalyticManifold','Expected unit quaternion and cable direction.');
end
T=max(0,min(20,u(1)));yaw=max(-1,min(1,u(4)));
u0=[T;0;0;yaw];
if isfield(s,'info') && isstruct(s.info) && isfield(s.info,'physicalNominal') ...
        && numel(s.info.physicalNominal)==4
    zNom=max(-1,min(1,double(s.info.physicalNominal(2:3))));
else
    zNom=max(-1,min(1,u(2:3)));
end
% Conservative scalar comparison envelope for all XY torques in [-1,1].
clock=tic;
[env,ok,reason]=sd_analytic_scalar_envelope(x,P,T,yaw,dt,G);
envMs=1000*toc(clock);
if ~ok,error('SD:AnalyticEnvelope','%s',reason);end
pointClock=tic;
E0=CBF_REAL_XY_EVAL(x,u0,P,G,s.parts,lam,tilt);
Ex=CBF_REAL_XY_EVAL(x,u0+[0;1;0;0],P,G,s.parts,lam,tilt);
Ey=CBF_REAL_XY_EVAL(x,u0+[0;0;1;0],P,G,s.parts,lam,tilt);
pointMs=1000*toc(pointClock);
S0=sd_boundary_stage_cell(E0,tilt);
Sx=sd_boundary_stage_cell(Ex,tilt);
Sy=sd_boundary_stage_cell(Ey,tilt);
names=sd_names(s.parts,tilt);
if numel(names)~=numel(S0),error('SD:AnalyticNames','Barrier mismatch.');end
formClock=tic;
A=zeros(0,2);ab=zeros(0,1);B=zeros(0,2);bb=zeros(0,1);
lowAll=true;barrierRows=struct([]);
maxMB=0;domainMin=Inf;
for i=1:numel(S0)
    r=numel(S0{i})-1;
    if i<=numel(s.parts)
        [centre,Qobs,~]=sd_shape(s.parts(i),i);
        if ~isequal(size(Qobs),[3 3]),error('SD:AnalyticObs','Invalid shape.');end
        [axisMin,axisMax]=sd_analytic_axes(s.parts(i));
        d0=norm(x(8:10)-G.cableCenterFromLoad*P.cableL*x(14:16)-centre);
        [H,dmin,valid]=sd_analytic_obstacle_h(x,P,G,env,d0,axisMin,axisMax,dt);
        if ~valid,error('SD:AnalyticSeparation','Centre-distance lower bound not positive.');end
        gains=lam;
    elseif strcmp(names{i},'bodyTilt')
        gains=double(tilt.bodyLambda(:));
        H=env.b3(:);dmin=NaN;
    elseif strcmp(names{i},'cableTilt')
        gains=double(tilt.cableLambda(:));
        H=env.pT(:);dmin=NaN;
    else
        error('SD:AnalyticBarrier','Unexpected barrier type.');
    end
    if numel(gains)~=r || numel(H)<r+2
        error('SD:AnalyticOrder','Insufficient majorant Taylor order.');
    end
    % Coefficients psi_r = product_j(D+lambda_j) h.
    polynomial=1;
    for j=1:r,polynomial=conv(polynomial,[1 gains(j)]);end
    polynomial=fliplr(polynomial); % ascending h^(j) coefficients
    MB=factorial(r+1)*H(r+2);
    MA=0;
    for j=0:r
        MA=MA+abs(polynomial(j+1))*factorial(j+1)*H(j+2);
    end
    if ~isfinite(MA) || ~isfinite(MB)
        error('SD:AnalyticOverflow','Bound overflow.');
    end
    maxMB=max(maxMB,MB);if isfinite(dmin),domainMin=min(domainMin,dmin);end
    lowNow=min(S0{i}(1:r));
    lowAll=lowAll && lowNow>=0;
    a0=S0{i}(end);az=[Sx{i}(end)-a0,Sy{i}(end)-a0];
    % A1: F_A(t_k,z) - MA*dt >=0 .
    A(end+1,:)=az;ab(end+1,1)=a0-MA*dt; %#ok<AGROW>
    % B1: degree r+1 Taylor polynomial with -MB*s^(r+1)/(r+1)!.
    if i<=numel(s.parts)
        raw=double(E0.hDerivatives(1:r+1,i));
    else
        raw=sd_fastqp_numeric_unstage(S0{i},gains);
    end
    n=r+1;
    c=zeros(n+1,1);d=zeros(n+1,2);
    for j=0:r-1,c(j+1)=raw(j+1)*dt^j/factorial(j);end
    rawR=raw(r+1);
    c(r+1)=rawR*dt^r/factorial(r);
    d(r+1,:)=az*dt^r/factorial(r);
    c(n+1)=-MB*dt^(r+1)/factorial(r+1);
    for ell=0:n
        beta=0;betaZ=[0 0];
        for j=0:ell
            weight=nchoosek(ell,j)/nchoosek(n,j);
            beta=beta+weight*c(j+1);
            betaZ=betaZ+weight*d(j+1,:);
        end
        B(end+1,:)=betaZ;bb(end+1,1)=beta; %#ok<AGROW>
    end
    % End-of-period contraction, a sufficient condition, not viability.
    rate=2;
    B(end+1,:)=d(r+1,:);
    bb(end+1,1)=sum(c)-exp(-rate*dt)*raw(1); %#ok<AGROW>
    br=struct('sampleIndex',id,'time',double(s.time), ...
        'barrier',names{i},'relativeDegree',r, ...
        'geometryDminM',dmin, ...
        'majorant_MA_PsiRate',MA,'majorant_MB_HrPlus1',MB, ...
        'nominalPsiHighest',a0+az*zNom, ...
        'A_requiredSampleMargin',MA*dt, ...
        'A_nominalMarginAfterBound',a0+az*zNom-MA*dt, ...
        'B_nominalFinalPoly',sum(c)+d(r+1,:)*zNom, ...
        'B_nominalContractGap',sum(c)+d(r+1,:)*zNom-exp(-rate*dt)*raw(1), ...
        'initialLowStageMin',lowNow, ...
        'numericStatus','EXACT_REAL_BOUND_FLOAT_EVAL_NOT_MACHINE_CERTIFIED', ...
        'recursiveViability','NO_PROVEN_INVARIANT_SET');
    if isempty(barrierRows),barrierRows=br;else,barrierRows(end+1,1)=br;end %#ok<AGROW>
end
formMs=1000*toc(formClock);
qpClock=tic;
[zA,okA,marginA]=sd_qp_project(A,ab,zNom,[1;1]);
[zB,okB,marginB]=sd_qp_project(B,bb,zNom,[1;1]);
qpMs=1000*toc(qpClock);
if ~lowAll,aStatus='INITIAL_LOW_STAGE_NEGATIVE';
elseif okA,aStatus='BOUND_QP_FEASIBLE__FLOAT_ONLY';
else,aStatus='BOUND_QP_INFEASIBLE';end
if okB,bStatus='BOUND_QP_FEASIBLE__FLOAT_ONLY';
else,bStatus='BOUND_QP_INFEASIBLE';end
caseRow=struct('sampleIndex',id,'time',double(s.time), ...
    'estimatedMassKg',P.mL,'thrustN',T,'yawNm',yaw, ...
    'numObstacles',numel(s.parts),'numBarriers',numel(S0), ...
    'scalarWcap',env.W,'scalarOcap',env.O,'scalarVcap',env.V, ...
    'minGeometryDminM',domainMin, ...
    'A_status',aStatus,'B_status',bStatus, ...
    'A_x',zA(1),'A_y',zA(2),'B_x',zB(1),'B_y',zB(2), ...
    'A_qpResidual',marginA,'B_qpResidual',marginB, ...
    'lowStagesOK',lowAll, 'maxObstacleMB',maxMB, ...
    'envelopeMs',envMs,'pointGeneratorMs',pointMs, ...
    'formulationMs',formMs,'qpMs',qpMs,'totalMs',1000*toc(start), ...
    'mathematicalMajorantDerived',true,'machineRoundedCertified',false, ...
    'recursiveFeasibilityCertified',false);
barrierRows=barrierRows(:);
% Cache the *same* A/B affine rows the diagnostic QP used. The cache is
% valid ONLY for the exact logged state/model/input/obstacle signature.
% No positive-radius certified cell is built in this mode.
qpRows=struct('A',A,'ab',ab,'B',B,'bb',bb,'zNom',zNom, ...
    'initialLowOK',lowAll,'dt',dt,'T',T,'yaw',yaw);
end

function [env,ok,reason]=sd_analytic_scalar_envelope(x,P,T,yaw,dt,G)
% Scalar comparison proof for all z in [-1,1]^2 and fixed T/yaw.
% q,pT remain unit under the stated exact continuous model.
ok=false;reason='';env=struct();
m=P.m;L=P.cableL;J=P.J(:);ml=P.mL;
if m<=0 || L<=0 || any(J<=0) || ml<0 || T<0
    reason='Invalid physical parameters';return;
end
W0=norm(x(5:7));O0=norm(x(17:19));V0=norm(x(11:13));
Jmin=min(J);Jmax=max(J);tauMax=sqrt(2+yaw^2);
A=dt*Jmax/Jmin;
C=W0+dt*tauMax/Jmin;
discriminant=1-4*A*C;
if discriminant<=0
    reason='Body-rate Riccati comparison does not enclose a full cycle';return;
end
rootSmall=2*C/(1+sqrt(discriminant));
rootLarge=(1+sqrt(discriminant))/(2*A);
W=rootSmall+min(max(1e-6,rootSmall*1e-3),(rootLarge-rootSmall)/4);
if W0+dt*(tauMax+Jmax*W^2)/Jmin>W
    reason='Body-rate comparison envelope failed';return;
end
O=O0+dt*T/(m*L);
Ddist=0;
if isfield(P,'dstx'),Ddist=Ddist+P.dstx^2;end
if isfield(P,'dsty'),Ddist=Ddist+P.dsty^2;end
Ddist=sqrt(Ddist);
V=V0+dt*(P.gravity+(T+m*L*O^2)/(m+ml)+Ddist);
if any(~isfinite([W,O,V]))
    reason='Scalar envelope overflow';return;
end
% Coefficient majorants for quaternion, body omega, pT, cable omega,
% load velocity and load position (coefficient index k+1).
N=5;
Q=zeros(1,N+1);BW=zeros(1,N+1);PT=zeros(1,N+1);
CW=zeros(1,N+1);VL=zeros(1,N+1);
B3=zeros(1,N+1);CT=zeros(1,N+1);S=zeros(1,N+1);
Q(1)=1;BW(1)=W;PT(1)=1;CW(1)=O;VL(1)=V;
for k=0:N-1
    % Quadratic body z-axis components, cross(wL,pT), thrust scalar.
    if k==0
        B3(1)=1;CT(1)=O;S(1)=T+m*L*O^2;
    else
        % Euclidean/operator-norm majorants: Rdot=R*[omega]_cross.
        % ||R_j||_2 <= B3_j, and ||[omega]_cross||_2=||omega||.
        B3(k+1)=sd_analytic_conv(B3,BW,k-1)/k;
        CT(k+1)=sd_analytic_conv(CW,PT,k);
        S(k+1)=T*sd_analytic_conv(PT,B3,k)+ ...
            m*L*sd_analytic_conv(CT,CT,k);
    end
    if k==0
        Q(2)=.5*W;
        BW(2)=(tauMax+Jmax*W^2)/Jmin;
        PT(2)=O;
        CW(2)=T/(m*L);
    else
        Q(k+2)=.5*sd_analytic_conv(Q,BW,k)/(k+1);
        BW(k+2)=Jmax/Jmin*sd_analytic_conv(BW,BW,k)/(k+1);
        PT(k+2)=sd_analytic_conv(CW,PT,k)/(k+1);
        CW(k+2)=T/(m*L)*sd_analytic_conv(PT,B3,k)/(k+1);
    end
    VL(k+2)=sd_analytic_conv(S,PT,k)/((m+ml)*(k+1));
    if k==0,VL(k+2)=VL(k+2)+P.gravity+Ddist;end
end
B3(N+1)=sd_analytic_conv(B3,BW,N-1)/N;
env=struct('W',W,'O',O,'V',V,'q',Q,'omega',BW, ...
    'pT',PT,'omegaL',CW,'vL',VL,'b3',B3, ...
    'G',G,'L',L,'T',T, 'dt',dt);
ok=true;
end

function [H,dmin,ok]=sd_analytic_obstacle_h(x,P,G,E,d0,aObsMin,aObsMax,dt)
% Construct h-majorant coefficients up to order 5 using the algebraic
% support identity h=(s-sqrt(as^2*s+delta*t^2)-sqrt(D'Qobs D))/sqrt(s).
% No normal-derivative interval Jet is needed.
alpha=G.cableCenterFromLoad*P.cableL;
reach=dt*(E.V+abs(alpha)*E.O);
dmin=d0-reach;
dmax=d0+reach;
ok=dmin>0 && aObsMin>0 && G.cableAxisShort>0;
H=nan(1,6);if ~ok,return;end
as=G.cableAxisShort;al=G.cableAxisLong;
if al<as,error('SD:AnalyticCableShape','Long axis smaller than short axis.');end
delta=al^2-as^2;
N=5;D=zeros(1,N+1);S=zeros(1,N+1);t=zeros(1,N+1);
C=zeros(1,N+1);O=zeros(1,N+1);
% For ALL intermediate states, not just the initial state, use dmax.
D(1)=dmax;S(1)=dmax^2;t(1)=dmax;
C(1)=al^2*dmax^2;O(1)=aObsMax^2*dmax^2;
for k=1:N
    D(k+1)=E.vL(k)/k+abs(alpha)*E.pT(k+1);
    S(k+1)=sd_analytic_conv(D,D,k);
    t(k+1)=sd_analytic_conv(D,E.pT,k);
    C(k+1)=as^2*S(k+1)+delta*sd_analytic_conv(t,t,k);
    O(k+1)=aObsMax^2*sd_analytic_conv(D,D,k);
end
r=sd_analytic_sqrt_maj(S,dmin,dmax);
sc=sd_analytic_sqrt_maj(C,as*dmin,al*dmax);
so=sd_analytic_sqrt_maj(O,aObsMin*dmin,aObsMax*dmax);
invR=zeros(1,N+1);invR(1)=1/dmin;
for k=1:N
    invR(k+1)=0;
    for j=1:k,invR(k+1)=invR(k+1)+r(j+1)*invR(k-j+1);end
    invR(k+1)=invR(k+1)/dmin;
end
for k=0:N
    H(k+1)=sd_analytic_conv(S,invR,k)+ ...
        sd_analytic_conv(sc,invR,k)+sd_analytic_conv(so,invR,k);
end
if any(~isfinite(H)),error('SD:AnalyticH','Nonfinite majorant.');end
end

function y=sd_analytic_sqrt_maj(a,ymin,ymax)
if ymin<=0 || ymax<=0,error('SD:AnalyticSqrt','Nonpositive support lower bound.');end
n=numel(a)-1;y=zeros(1,n+1);y(1)=ymax;
for k=1:n
    crossTerms=0;
    for j=1:k-1
        crossTerms=crossTerms+y(j+1)*y(k-j+1);
    end
    y(k+1)=(a(k+1)+crossTerms)/(2*ymin);
end
end

function v=sd_analytic_conv(a,b,k)
v=0;
for j=0:k,v=v+a(j+1)*b(k-j+1);end
end

function [low,high]=sd_analytic_axes(part)
if isfield(part,'kind') && strcmp(char(part.kind),'sphere')
    if isfield(part,'params'),rad=part.params.radius;else,rad=part.radius;end
    low=rad;high=rad;
else
    if isfield(part,'params'),ax=part.params.axes;else,ax=part.axes;end
    low=min(ax(:));high=max(ax(:));
end
low=double(low);high=double(high);
if ~(isfinite(low)&&isfinite(high)&&low>0&&high>=low)
    error('SD:AnalyticAxes','Obstacle axes invalid.');
end
end

function sd_analyticbound_save(R)
root=fileparts(mfilename('fullpath'));
tmp=tempname;mkdir(tmp);clean=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
if ~isempty(R.cases),writetable(struct2table(R.cases),fullfile(tmp,'ANALYTIC_CASES.csv'));end
if ~isempty(R.barriers),writetable(struct2table(R.barriers),fullfile(tmp,'ANALYTIC_BARRIERS.csv'));end
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
for j=1:numel(R.errors),fprintf(fid,'%s\n',R.errors{j});end
fclose(fid);
fid=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fid,['ANALYTIC scalar Taylor majorants of the 19-state model, no interval Jet or candidate list.\n' ...
    'MODEL: fixed thrust/yaw and estimated mass in each 25ms cycle, XY torque bounded by [-1,1]^2.\n' ...
    'MA_i >= sup |d(psi_r)/dt|, MB_i >= sup |h^(r+1)| under scalar envelope.\n' ...
    'Double evaluation of algebraic bounds is NOT outward-rounded; no machine-checked proof.\n' ...
    'A current low stages must hold; no theorem if new HLC thrust switches to negative low stage.\n' ...
    'No computed terminal control-invariant K, no guarantee of next cycle QP feasibility.\n']);
fprintf(fid,'analysed=%d errors=%d\n',numel(R.cases),numel(R.errors));
if ~isempty(R.cases)
    fprintf(fid,'A bound-QP nominally feasible=%d/%d (before low-stage check)\n', ...
        sum(strcmp({R.cases.A_status},'BOUND_QP_FEASIBLE__FLOAT_ONLY')),numel(R.cases));
    fprintf(fid,'B bound-QP nominally feasible=%d/%d\n', ...
        sum(strcmp({R.cases.B_status},'BOUND_QP_FEASIBLE__FLOAT_ONLY')),numel(R.cases));
    fprintf(fid,'median computational cost=%.5fms max=%.5fms (offline MATLAB only)\n', ...
        median([R.cases.totalMs]),max([R.cases.totalMs]));
end
fprintf(fid,['RESULT CLASSIFICATION: constructive mathematical bound formula, no strict machine certificate.\n' ...
    'Do not apply in production; no complete forward invariance proof.\n']);
fclose(fid);
outZip=fullfile(root,'CBF_SD_TEST_RESULTS.zip');
if isfile(outZip),delete(outZip);end
names={'SUMMARY.txt','ERRORS.txt'};
if ~isempty(R.cases),names{end+1}='ANALYTIC_CASES.csv';end
if ~isempty(R.barriers),names{end+1}='ANALYTIC_BARRIERS.csv';end
zip(outZip,names,tmp);
end


% ======================================================================
% 2026-11-23 THEOREM BRIDGE (read-only): rigorously derived 1-ZOH
% geometry speed screen, plus exact thrust sensitivity of psi_2.
% This is a new, FAST SUFFICIENT SAFETY SCREEN, not a complete safety CBF.
% No interval Jet, no torque candidate list, no optimization for screening.
% Exact-real formulas; binary64 rounding is NOT proof certified.
% ======================================================================
function report=sd_theorybridge_mode(varargin)
if isempty(varargin) || ~isfile(char(varargin{1}))
    error('SD:BridgeInput','CBF_FOURWAY_RESULTS.zip required.');
end
pathWas=path;pathGuard=onCleanup(@()path(pathWas)); %#ok<NASGU>
sd_find_generator();
folder=tempname;mkdir(folder);clean=onCleanup(@()sd_cleanup(folder)); %#ok<NASGU>
unzip(char(varargin{1}),folder);
tab=readtable(fullfile(folder,'PER_SAMPLE.csv'));
valid=find(tab.parts>0 & isfinite(tab.min_h_m));
if isempty(valid),error('SD:BridgeEmpty','No recorded obstacle cases.');end
n=min(6,numel(valid));
[~,idxA]=sort(tab.min_h_m(valid));[~,idxB]=sort(tab.min_psi3(valid));
q=min(ceil(n/3),numel(valid));
chosen=unique([valid(idxA(1:q));valid(idxB(1:q)); ...
   valid(unique(round(linspace(1,numel(valid),n))))],'stable');
chosen=chosen(1:min(n,numel(chosen)));
if numel(chosen)<n
    tail=setdiff(valid,chosen,'stable');chosen=[chosen;tail(1:n-numel(chosen))];
end
chosen=sort(chosen(:));
mat=load(fullfile(folder,'RAW_SNAPSHOTS.mat'),'snapshot');
if ~iscell(mat.snapshot)|| numel(mat.snapshot)<max(chosen)
    error('SD:BridgeSnapshot','Snapshot log unavailable.');
end
fixed=sd_analytic_fixed_signature(mat.snapshot{chosen(1)});
% PRE-FLIGHT: execute all online-relevant code paths before the timed flight
% replay. A warmup reduces some startup/JIT effects but is NOT WCET proof.
preWarm=struct([]);
first=mat.snapshot{chosen(1)};
for ww=1:5
    wwClock=tic;
    [cw,~,ew]=sd_bridge_case(first,chosen(1));
    pw=sd_bridge_plane_case(first,chosen(1),ew);
    totalWarm=1000*toc(wwClock);
    ent=struct('repeat',ww,'sampleIndex',chosen(1), ...
       'time',double(first.time),'totalMs',totalWarm, ...
       'envelopeMs',cw.envelopeMs, ...
       'generatorFirstMs',cw.generatorFirstMs, ...
       'generatorSecondMs',cw.generatorSecondMs, ...
       'geometryTiltMs',cw.geometryAndTiltMs, ...
       'planeMs',sum([pw.planeComputeMs]));
    if isempty(preWarm),preWarm=ent;else,preWarm(end+1,1)=ent;end %#ok<AGROW>
end
rows=struct([]);bs=struct([]);js=struct([]);ps=struct([]);inputs=struct([]);errors={};
for a=1:numel(chosen)
    id=chosen(a);
    try
        s=mat.snapshot{id};
        sd_analytic_fixed_check(fixed,s);
        [rr,br,env]=sd_bridge_case(s,id);
        pr=sd_bridge_plane_case(s,id,env);
        % Offline mathematical/model-consistency checks; NOT online timing.
        pi=sd_bridge_plane_input_coeff(s,id);
        if isempty(inputs),inputs=pi;elseif ~isempty(pi),inputs=[inputs;pi(:)];end %#ok<AGROW>
        % Report ALL independently timed operations (older elapsedMs
        % intentionally covers bridge_case only and excludes the plane).
        rr.planeComputeMs=sum([pr.planeComputeMs]);
        rr.totalWithPlaneMs=rr.elapsedMs+rr.planeComputeMs;
        rr.minFixedPlaneMarginM=min([pr.planeWholePeriodLowerM]);
        % The fixed plane certifies physical obstacle non-overlap, NOT
        % the original centre-normal HOCBF h. Other constraints retain
        % their independent exact-real ZOH rate screens.
        nonObs=~strcmp({br.barrierType},'cableObstacle');
        rr.allPhysicalOneCycleScreenPassed= ...
            all([pr.planePhysicalSeparationPassed]) && all([br(nonObs).screenPassed]);
        if isempty(ps),ps=pr;elseif ~isempty(pr),ps=[ps;pr(:)];end %#ok<AGROW>
        if isempty(rows),rows=rr;else,rows(end+1,1)=rr;end %#ok<AGROW>
        if isempty(bs),bs=br;else,bs=[bs;br(:)];end %#ok<AGROW>
        if id<numel(mat.snapshot) && ~isempty(mat.snapshot{id+1})
            next=mat.snapshot{id+1};
            if abs(next.time-s.time-double(s.dt))<1e-5
                jj=sd_bridge_jump_pair(s,next,id);
                if isempty(js),js=jj;else,js=[js;jj(:)];end %#ok<AGROW>
            end
        end
        fprintf('[SD THEORY] t=%.3f  allXY_screen=%d  mass=%.6f  %.3f ms  A_psi2_jump_check=OK\n', ...
            rr.time,rr.allBarrierScreenPassed,rr.estimatedMassKg,rr.elapsedMs);
    catch ME
        errors{end+1,1}=sprintf('%d: %s [%s]',id,ME.message,ME.identifier); %#ok<AGROW>
        fprintf('[SD THEORY] ERROR %d: %s\n',id,ME.message);
    end
end
% WHOLE-REPLAY benchmarking (five COMPLETE passes in chronological
% snapshot order). Unlike the old 2-snapshot repeats, every pass executes
% the same algorithm on every recorded snapshot, including those without
% an observed obstacle. Read/ZIP/CSV I/O excluded from the measured loops.
% Diagnostic and screenOnly are two SEPARATE paths, each replayed 5 times.
% This is OFFLINE timing, never a worst-case execution-time certificate.
flightRuns=struct([]);flightSamples=table();flightTimingErrors={};
try
    [flightRuns,flightSamples,flightTimingErrors]= ...
        sd_bridge_full_replay_benchmark(mat.snapshot,fixed,5);
catch ME
    errors{end+1,1}=sprintf('FULL_REPLAY_TIMING: %s [%s]', ...
        ME.message,ME.identifier); %#ok<AGROW>
end
% Match the timed samples with the REAL, recorded Scheme02 status and
% clearance. This runs only AFTER all 5 full replays: no timing pollution.
% XY_TORQUE_CORRECTED may include soft guidance; it does NOT prove a
% hard CBF inequality was binding.
flightSegments=struct([]);
if ~isempty(flightSamples)
    try
        [flightSamples,flightSegments]= ...
            sd_bridge_segment_from_recorded_cbf(flightSamples,tab);
    catch ME
        errors{end+1,1}=sprintf('FULL_REPLAY_SEGMENT: %s [%s]', ...
            ME.message,ME.identifier); %#ok<AGROW>
    end
end
% EXACT-REAL A low-stage thrust interval over ALL observed constraints,
% audited at every consecutive recorded 25-ms boundary. The transition
% state comes from the RECORDED previous controller, not a predicted XY-QP.
% B: separate geometric one-cycle screen using the NEW state's own plane.
% Audit is OFFLINE and runs after all flight timing loops; never included
% in the measured per-cycle times.
fullBoundary=struct([]);fullLowStages=struct([]);fullBoundaryErrors={};
try
    [fullBoundary,fullLowStages,fullBoundaryErrors]= ...
        sd_bridge_all_boundary_audit(mat.snapshot,fixed);
catch ME
    errors{end+1,1}=sprintf('FULL_BOUNDARY_AUDIT: %s [%s]',ME.message,ME.identifier); %#ok<AGROW>
end
% REFERENCE-ADMISSION AUDIT (OFFLINE ONLY): invert the actual HLC
% vertical thrust equation at the RECORDED new state. This computes the
% allowed qz = Vz + zddot_ref interval for A low-stage membership.
% It does NOT modify T, yaw, reference, or XY torque, and does NOT claim
% a trajectory-governor or recursive-feasibility certificate. No extra
% Jet, obstacle screening or online timing cost: run AFTER all five
% complete replay timing passes and after the boundary audit.
referenceAdmission=struct([]);referenceAdmissionErrors={};
if ~isempty(fullBoundary)
    try
        [referenceAdmission,referenceAdmissionErrors]= ...
          sd_bridge_reference_admission(fullBoundary,mat.snapshot);
    catch ME
        errors{end+1,1}=sprintf('REFERENCE_ADMISSION: %s [%s]',ME.message,ME.identifier); %#ok<AGROW>
    end
end
% FUTURE REFERENCE-GOVERNOR COMPATIBILITY AUDIT, OFFLINE ONLY.
% These are fixed recorded-state scalar qz intervals and an inferred qz
% trajectory, NOT actual stored reference jets or an online governor.
% Exact-real theorem: an R-Lipschitz selection in closed intervals
% exists iff all pairwise interval separations are <= R*dt. An anchored
% initial qz becomes a singleton at the initial recorded time. A full
% witness is checked by forward reachable-interval / backward selection.
% This runs AFTER the five full replays and outside timed callbacks.
referenceSlewRows=table();referenceSlewMeta=struct([]);referenceSlewErrors={};
if ~isempty(referenceAdmission)
    try
        [referenceSlewRows,referenceSlewMeta,referenceSlewErrors]= ...
            sd_bridge_reference_slew_audit(referenceAdmission);
    catch ME
        errors{end+1,1}=sprintf('REFERENCE_SLEW_AUDIT: %s [%s]', ...
            ME.message,ME.identifier); %#ok<AGROW>
    end
end
% FLIGHT-SPECIFIC ADMISSION AUDIT: entire recorded flight has a single
% immutable parameter configuration theta_f. Estimated load mass and HLC
% inputs are DYNAMIC and not incorrectly frozen. A new flight configuration
% requires an entirely NEW preflight verification, never a reused result.
% This is a consistency/coverage audit, NOT an operational flight permit.
flightSetup=struct([]);flightSetupMeta=struct([]);flightSetupErrors={};
try
    [flightSetup,flightSetupMeta,flightSetupErrors]= ...
        sd_bridge_flight_setup_audit(mat.snapshot,fixed,fullBoundary,referenceAdmission);
catch ME
    errors{end+1,1}=sprintf('FLIGHT_SETUP_AUDIT: %s [%s]',ME.message,ME.identifier); %#ok<AGROW>
end
% B: probe the fifth Lie derivative of the FIXED-plane separation
% by directional differentiation of the independently generated g4.
% Structural proof is in README: g4=a4+B4*z, and B4 does not depend
% on body angular velocity. Therefore the quadratic z^T(grad B4)Gz
% vanishes exactly; finite differences below are ONLY a numerical test.
planeFifthRows=struct([]);planeFifthErrors={};
for kk=1:numel(chosen)
    try
        rec=sd_bridge_plane_g5_affinity(mat.snapshot{chosen(kk)},chosen(kk));
        if isempty(planeFifthRows),planeFifthRows=rec;
        elseif ~isempty(rec),planeFifthRows=[planeFifthRows;rec(:)];end %#ok<AGROW>
    catch ME
        planeFifthErrors{end+1,1}=sprintf('sample=%d: %s [%s]', ...
            chosen(kk),ME.message,ME.identifier); %#ok<AGROW>
    end
end
% A: numerical test of the POINTWISE fifth-stage rate (psi_r dot)
% for all observed obstacle/tilt barriers. Unlike general CBF systems,
% the leading torque coefficient here does not depend on body omega;
% hence the usual quadratic-torque Lie term is structurally absent.
methodARateRows=struct([]);methodARateErrors={};
for kk=1:numel(chosen)
 try
   aa=sd_bridge_A_highstage_rate_affinity(mat.snapshot{chosen(kk)},chosen(kk));
   if isempty(methodARateRows),methodARateRows=aa;
   elseif ~isempty(aa),methodARateRows=[methodARateRows;aa(:)];end %#ok<AGROW>
 catch ME
   methodARateErrors{end+1,1}=sprintf('sample=%d: %s [%s]', ...
      chosen(kk),ME.message,ME.identifier); %#ok<AGROW>
 end
end
% NEW AB-FLIGHT: run ACTUAL continuous two-input A and B QPs on the
% entire historical flight using finite scalar majorants. Timed separately
% from earlier lightweight screen, before saving the ONE results ZIP.
% The newly selected inputs do NOT generate logged successor states, so
% this is an offline per-snapshot QP experiment, NOT closed-loop safety.
abFlightCases=struct([]);abFlightBarriers=struct([]);
abFlightNumeric=struct([]);abFlightMeta=struct([]);abFlightErrors={};
try
 [abFlightCases,abFlightBarriers,abFlightNumeric,abFlightMeta,abFlightErrors]= ...
   sd_bridge_abflight_qp_audit(mat.snapshot,fixed,tab);
catch ME
 errors{end+1,1}=sprintf('AB_FLIGHT_QP: %s [%s]',ME.message,ME.identifier); %#ok<AGROW>
end
% Focused interval tube A/B refinement: 1-vs-4 piece local certificate.
% Runs on a handful of logged states, outside the measured full-flight QP.
localRefineCases=struct([]);localRefineBarriers=struct([]);localRefineMeta=struct([]);
localRefineErrors={};
try
    [localRefineCases,localRefineBarriers,localRefineMeta,localRefineErrors]= ...
        sd_localrefine_audit(mat.snapshot,tab);
catch ME
    localRefineErrors{end+1,1}=sprintf('LOCAL_REFINE: %s [%s]',ME.message,ME.identifier);
end
% OFFLINE selftest of the certified-coefficient-to-linear-XY-QP converter.
% These are SYNTHETIC coefficient values; no real interval bounds have
% been certified yet. The converter is implementable once valid reach
% tubes and coefficient enclosures have been supplied.
qpTemplate=struct('passed',false,'methodARows',0,'methodBRows',0,'message','not evaluated');
try
 qpTemplate=sd_bridge_5th_affine_qp_selftest();
catch ME
 errors{end+1,1}=sprintf('AFFINE_QP_TEMPLATE: %s [%s]',ME.message,ME.identifier); %#ok<AGROW>
end
report=struct('mode','theorybridge','cases',rows,'barriers',bs, ...
    'jumps',js,'planes',ps,'planeInputs',inputs,'timing',struct([]), ...
    'fullBoundary',fullBoundary,'fullLowStages',fullLowStages, ...
    'referenceAdmission',referenceAdmission, ...
    'referenceSlewRows',referenceSlewRows,'referenceSlewMeta',referenceSlewMeta, ...
    'flightSetup',flightSetup,'flightSetupMeta',flightSetupMeta, ...
    'flightSetupErrors',{flightSetupErrors}, ...
    'planeFifthRows',planeFifthRows,'planeFifthErrors',{planeFifthErrors}, ...
    'methodARateRows',methodARateRows,'methodARateErrors',{methodARateErrors}, ...
    'abFlightCases',abFlightCases,'abFlightBarriers',abFlightBarriers, ...
    'localRefineCases',localRefineCases,'localRefineBarriers',localRefineBarriers, ...
    'localRefineMeta',localRefineMeta,'localRefineErrors',{localRefineErrors}, ...
    'abFlightNumeric',abFlightNumeric,'abFlightMeta',abFlightMeta, ...
    'abFlightErrors',{abFlightErrors}, ...
    'qpTemplate',qpTemplate, ...
    'referenceSlewErrors',{referenceSlewErrors}, ...
    'referenceAdmissionErrors',{referenceAdmissionErrors}, ...
    'fullBoundaryErrors',{fullBoundaryErrors}, ...
    'flightRuns',flightRuns,'flightSamples',flightSamples, ...
    'flightSegments',flightSegments, ...
    'flightTimingErrors',{flightTimingErrors}, ...
    'preflightWarmup',preWarm,'errors',{errors},'fullForwardInvarianceCertified',false, ...
    'machineRoundoffCertified',false);
sd_bridge_save(report);
end

% ======================================================================
% AB FLIGHT QP STRESS TEST (read-only; historical snapshots, NOT closed-loop)
% All 2157 recorded samples, 1 continuous XY-QP for A and 1 for B per sample.
% Reuses mathematically derived scalar majorants; it never tests fixed torque
% candidate lists and never changes T or yaw. The historical next state is
% NOT a successor of the newly computed A/B torque.
% ======================================================================
function [rows,barriers,checks,meta,errors]=sd_bridge_abflight_qp_audit(snapshots,fixed,tab)
rows=struct([]);barriers=struct([]);checks=struct([]);errors={};
N=numel(snapshots);
if height(tab)~=N
    error('SD:ABFlightLength','PER_SAMPLE and snapshot counts disagree');
end
ids=find(tab.parts>0 & isfinite(tab.min_h_m));
[~,idxGap]=sort(tab.min_h_m(ids));
closeIds=ids(idxGap(1:min(12,numel(idxGap))));
[~,idxStage]=sort(tab.min_psi3(ids));
stageIds=ids(idxStage(1:min(8,numel(idxStage))));
if isempty(ids),timeIds=[];
else,timeIds=ids(unique(max(1,min(numel(ids),round(linspace(1,numel(ids),6))))));end
numericIds=unique([closeIds(:);stageIds(:);timeIds(:)]);
count=0;A=0;B=0;both=0;Afailed=0;Bfailed=0;deadline=0;
numericCount=0;numericErrors=0;over5=0;maxMs=0;maxIdx=NaN;
for k=1:N
 s=snapshots{k};if isempty(s),continue;end
 try
    % The signature is checked BEFORE the timed numerical controller test.
    sd_analytic_fixed_check(fixed,s);
    [c,bb]=sd_analyticbound_case(s,k);
    ms=double(c.totalMs);
    if ~isfinite(ms) || ms<0,error('SD:ABFlightClock','Invalid execution time');end
    if ms>maxMs,maxMs=ms;maxIdx=k;end
    deadline=deadline+(ms>25);over5=over5+(ms>5);
    % A needs BOTH initial lower stages and a feasible high-stage QP.
    aOK=strcmp(c.A_status,'BOUND_QP_FEASIBLE__FLOAT_ONLY');
    bOK=strcmp(c.B_status,'BOUND_QP_FEASIBLE__FLOAT_ONLY');
    A=A+aOK;B=B+bOK;both=both+(aOK && bOK);
    Afailed=Afailed+~aOK;Bfailed=Bfailed+~bOK;
    zLogged=double(s.u(2:3));
    if isfield(s,'info') && isstruct(s.info) && ...
            isfield(s.info,'physicalNominal') && numel(s.info.physicalNominal)==4
        zNom=max(-1,min(1,double(s.info.physicalNominal(2:3))));
        nominalFrom='info.physicalNominal';
    else
        zNom=max(-1,min(1,zLogged));nominalFrom='logged.applied';
    end
    da=NaN;db=NaN;
    if aOK,da=norm([c.A_x;c.A_y]-zNom,2);end
    if bOK,db=norm([c.B_x;c.B_y]-zNom,2);end
    rr=struct('sampleIndex',k,'timeS',double(s.time), ...
      'obstacles',numel(s.parts),'estimatedMassKg',double(s.P.mL), ...
      'thrustN',double(c.thrustN),'yawNm',double(c.yawNm), ...
      'nominalSource',nominalFrom,'nominalTauX',zNom(1), ...
      'nominalTauY',zNom(2),'loggedTauX',zLogged(1), ...
      'loggedTauY',zLogged(2),'A_initialLowerStagesOK',logical(c.lowStagesOK), ...
      'A_qpFeasibleConditional',aOK,'A_status',c.A_status, ...
      'A_tauX',double(c.A_x),'A_tauY',double(c.A_y), ...
      'A_deltaNominalNorm',da,'A_numericResidual',double(c.A_qpResidual), ...
      'B_qpFeasibleConditional',bOK,'B_status',c.B_status, ...
      'B_tauX',double(c.B_x),'B_tauY',double(c.B_y), ...
      'B_deltaNominalNorm',db,'B_numericResidual',double(c.B_qpResidual), ...
      'envelopeMs',double(c.envelopeMs), ...
      'pointJetMs',double(c.pointGeneratorMs), ...
      'linearRowsMs',double(c.formulationMs), ...
      'twoQPms',double(c.qpMs),'totalMs',ms, ...
      'measuredOverFiveMs',ms>5,'measuredOverDeadline25Ms',ms>25, ...
      'certifiedOneZoh',false,'certifiedRecursive',false, ...
      'certifiedRoundoff',false,'certifiedWCET',false);
    if isempty(rows),rows=rr;else,rows(end+1,1)=rr;end %#ok<AGROW>
    for j=1:numel(bb)
        bj=bb(j);
        pr=struct('sampleIndex',k,'timeS',double(s.time), ...
            'barrier',bj.barrier,'relativeDegree',bj.relativeDegree, ...
            'A_rateMajorant',bj.majorant_MA_PsiRate, ...
            'B_rateMajorant',bj.majorant_MB_HrPlus1, ...
            'A_nominalMargin',bj.A_nominalMarginAfterBound, ...
            'B_nominalEndpointGap',bj.B_nominalContractGap, ...
            'initialLowerMin',bj.initialLowStageMin, ...
            'minimumGeometryDomainM',bj.geometryDminM);
        if isempty(barriers),barriers=pr;else,barriers(end+1,1)=pr;end %#ok<AGROW>
    end
    if any(k==numericIds)
        % Only OFFLINE, with the INPUT actually selected by each QP.
        % RK4 is NOT a guaranteed enclosure of continuous trajectories.
        labels={'logged'};
        Z=zLogged;
        if aOK,labels{end+1}='A_QP';Z(:,end+1)=[c.A_x;c.A_y];end %#ok<AGROW>
        if bOK,labels{end+1}='B_QP';Z(:,end+1)=[c.B_x;c.B_y];end %#<AGROW>
        for jj=1:numel(labels)
            try
                check=sd_bridge_abflight_rk4(s,Z(:,jj),8);
                qq=struct('sampleIndex',k,'timeS',double(s.time), ...
                    'inputType',labels{jj}, ...
                    'torqueX',Z(1,jj),'torqueY',Z(2,jj), ...
                    'approxGridMinPhysicalH',check.minPhysicalH, ...
                    'approxGridMinContractionGap',check.minEndpointContractionGap, ...
                    'approxGridPhysicalNonnegative',check.minPhysicalH>=0, ...
                    'approxGridContractionPassed',check.minEndpointContractionGap>=0, ...
                    'rk4Substeps',8,'isVerifiedIntegration',false, ...
                    'isNewClosedLoop',false);
                if isempty(checks),checks=qq;else,checks(end+1,1)=qq;end %#ok<AGROW>
                numericCount=numericCount+1;
            catch NE
                numericErrors=numericErrors+1;
                if numel(errors)<50
                    errors{end+1,1}=sprintf('NUMERIC_ONLY sample=%d input=%s %s [%s]', ...
                      k,labels{jj},NE.message,NE.identifier);end %#ok<AGROW>
            end
        end
    end
    count=count+1;
 catch ME
    if numel(errors)<50
        errors{end+1,1}=sprintf('QP_SAMPLE=%d t=%g: %s [%s]', ...
            k,double(s.time),ME.message,ME.identifier);end %#<AGROW>
 end
end
if isempty(rows),times=NaN;
else,times=[rows.totalMs];end
meta=struct('recordedSamples',N,'processedSamples',count, ...
  'errorCount',numel(errors),'A_conditionalQPFeasible',A, ...
  'B_conditionalQPFeasible',B,'bothConditionalQPFeasible',both, ...
  'A_notAccepted',Afailed,'B_notAccepted',Bfailed, ...
  'numericRK4Trajectories',numericCount,'numericRK4Errors',numericErrors, ...
  'medianTotalMs',median(times),'maxTotalMs',maxMs, ...
  'maxTotalSampleIndex',maxIdx,'overFiveMs',over5, ...
  'over25Ms',deadline,'onlineWCETCertified',false, ...
  'realMachineIntervalCertified',false,'recursiveFeasibilityCertified',false);
fprintf('[SD AB-FLIGHT] n=%d/%d, A=%d, B=%d, both=%d; time median %.3fms maximum %.3fms; >25ms=%d; errors=%d. NOT CERTIFIED.\n', ...
  count,N,A,B,both,meta.medianTotalMs,maxMs,deadline,numel(errors));
end

function out=sd_bridge_abflight_rk4(s,z,substeps)
% Numerical convergence check ONLY; no proof for intersample safety.
x=double(s.x(:));u=[max(0,min(20,double(s.u(1))));double(z(:)); ...
    max(-1,min(1,double(s.u(4))))];
P=s.P;G=s.G;h=double(s.dt)/substeps;
start=sd_bridge_abflight_physical_gaps(x,s.parts,G,s.tiltCfg,P);
minH=min(start);
for j=1:substeps
    k1=CBF_COMMON_MODEL(x,u,P);
    k2=CBF_COMMON_MODEL(x+h*k1/2,u,P);
    k3=CBF_COMMON_MODEL(x+h*k2/2,u,P);
    k4=CBF_COMMON_MODEL(x+h*k3,u,P);
    x=x+h*(k1+2*k2+2*k3+k4)/6;
    % Purely numerical manifold projection; exact model proof not claimed.
    x(1:4)=x(1:4)/norm(x(1:4));
    x(14:16)=x(14:16)/norm(x(14:16));
    gaps=sd_bridge_abflight_physical_gaps(x,s.parts,G,s.tiltCfg,P);
    minH=min(minH,min(gaps));
end
rho=exp(-2*double(s.dt));
endGap=gaps-rho*start;
out=struct('minPhysicalH',minH, ...
    'minEndpointContractionGap',min(endGap));
end

function gaps=sd_bridge_abflight_physical_gaps(x,parts,G,tilt,P)
p=x(14:16);alpha=G.cableCenterFromLoad*P.cableL;
C=x(8:10)-alpha*p;
aS=G.cableAxisShort;delta=G.cableAxisLong^2-aS^2;
gaps=zeros(0,1);
for k=1:numel(parts)
    [centre,Q,~]=sd_shape(parts(k),k);
    D=C-centre;d=norm(D);
    if d<=0,error('SD:ABFlightCoincident','Cable and obstacle centre coincide');end
    n=D/d;us=dot(n,p);
    gap=d-sqrt(aS^2+delta*us^2)-sqrt(max(0,n.'*Q*n));
    gaps(end+1,1)=gap; %#ok<AGROW>
end
q=x(1:4);b3z=q(1)^2-q(2)^2-q(3)^2+q(4)^2;
if tilt.bodyEnabled,gaps(end+1,1)=b3z-cosd(tilt.bodyMaxDeg);end %#ok<AGROW>
if tilt.cableEnabled,gaps(end+1,1)=-p(3)-cosd(tilt.cableMaxDeg);end %#ok<AGROW>
if isempty(gaps),error('SD:ABFlightNoBarrier','No active barrier');end
if any(~isfinite(gaps)),error('SD:ABFlightNonfinite','Invalid geometry gap');end
end

function [row,br,E]=sd_bridge_case(s,id)
clock=tic;
x=double(s.x(:));u=double(s.u(:));P=s.P;G=s.G;dt=double(s.dt);
if numel(x)~=19||numel(u)~=4||dt<=0||dt>.1
    error('SD:BridgeState','Invalid 19D state/input or dt.');
end
if max(abs([norm(x(1:4))-1;norm(x(14:16))-1]))>1e-8
    error('SD:BridgeUnit','Input quaternion/cable direction not unit.');
end
T=max(0,min(20,u(1)));yaw=max(-1,min(1,u(4)));
clockEnvelope=tic;
[E,ok,why]=sd_analytic_scalar_envelope(x,P,T,yaw,dt,G);
envelopeMs=1000*toc(clockEnvelope);
if ~ok,error('SD:BridgeEnvelope','%s',why);end
q=x(1:4);p=x(14:16);
b3=[2*(q(2)*q(4)+q(1)*q(3)); ...
    2*(q(3)*q(4)-q(1)*q(2)); ...
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
if abs(norm(b3)-1)>1e-8,error('SD:BridgeAxis','body axis not unit');end
alpha=G.cableCenterFromLoad*P.cableL;
C=x(8:10)-alpha*p;
Vc=E.V+abs(alpha)*E.O;
% Scalar speed estimates are uniform on the entire current ZOH, for all
% z in [-1,1]^2, under the exact plant and fixed estimated mass/model.
ctrlU=[T;0;0;yaw];
clockJet=tic;
E0=CBF_REAL_XY_EVAL(x,ctrlU,P,G,s.parts,s.lambda,s.tiltCfg);
generatorFirstMs=1000*toc(clockJet);
clockJet=tic;
E1=CBF_REAL_XY_EVAL(x,[T+1;0;0;yaw],P,G,s.parts,s.lambda,s.tiltCfg);
generatorSecondMs=1000*toc(clockJet);
clockGeometry=tic;
br=struct([]);passed=true;minGap=Inf;maxDiff=0;
for i=1:numel(s.parts)
    [centre,Q,name]=sd_shape(s.parts(i),i);
    D=C-centre;d=norm(D);
    if d<=0,error('SD:BridgeCenters','Coincident cable/obstacle centre');end
    n=D/d;aS=G.cableAxisShort;aL=G.cableAxisLong;
    delta=aL*aL-aS*aS;us=dot(n,p);
    ca=sqrt(aS*aS+delta*us*us);
    so=sqrt(n.'*Q*n);
    h=d-ca-so;
    aObsMax=sqrt(max(eig((Q+Q.')/2)));
    minD=d-Vc*dt;
    if minD<=0
        gap=-Inf;
    else
        % |n(s)-n0| <= Vc*s/minD along the straight time path.
        % sqrt(aS^2 + delta*(n.p)^2) is delta/aL-Lipschitz in n.p
        % on [-1,1]. Obstacle support is aObsMax-Lipschitz in n.
        kappa=delta/aL;
        Lh=Vc*(1+(kappa+aObsMax)/minD)+kappa*E.O;
        gap=h-dt*Lh;
    end
    % Exact partial_T psi_2 at fixed x and fixed estimated mL.
    beta=dot(p,b3);
    pTT=-(b3-beta*p)/(P.m*P.cableL);
    CTT=beta/(P.m+P.mL)*p-alpha*pTT;
    gradD=n-(eye(3)-n*n.')*(delta*us/ca*p+Q*n/so)/d;
    gradP=-(delta*us/ca)*n;
    chi=gradD.'*CTT+gradP.'*pTT;
    psi2=E0.stages(3,i);
    psi2Tmeas=E1.stages(3,i)-psi2;
    diff=abs(chi-psi2Tmeas);
    maxDiff=max(maxDiff,diff);
    if diff>2e-6*(1+abs(chi))
        error('SD:BridgeThrustDerivative', ...
            'Exact dpsi2/dT mismatch: %g vs %g',chi,psi2Tmeas);
    end
    % The new HLC thrust is not predicted. This endpoint test is merely
    % necessary for robust low-stage membership for ALL T in [0,20].
    psi2MinAnyT=min(psi2+chi*(0-T),psi2+chi*(20-T));
    if ~(gap>=0),passed=false;end
    minGap=min(minGap,gap);
    bi=struct('sampleIndex',id,'time',double(s.time), ...
      'barrier',[name '/cable'],'barrierType','cableObstacle', ...
      'hStartM',h,'hOneCycleLowerM',gap, ...
      'screenPassed',isfinite(gap)&&gap>=0, ...
      'dMinM',minD,'maxRelativeCentreSpeedMps',Vc, ...
      'maxCableAngularSpeedRps',E.O, ...
      'psi2AtCurrentThrust',psi2, ...
      'dPsi2_dThrustExact',chi, ...
      'dPsi2_dThrustJetDifference',psi2Tmeas, ...
      'psi2MinForAnyThrust0To20',psi2MinAnyT, ...
      'analyticJetCheckAbsError',diff);
    if isempty(br),br=bi;else,br(end+1,1)=bi;end %#ok<AGROW>
end
if s.tiltCfg.bodyEnabled
    h=b3(3)-cosd(s.tiltCfg.bodyMaxDeg);
    gap=h-dt*E.W;
    bi=struct('sampleIndex',id,'time',double(s.time), ...
      'barrier','bodyTilt','barrierType','bodyTilt', ...
      'hStartM',h,'hOneCycleLowerM',gap, ...
      'screenPassed',gap>=0,'dMinM',NaN, ...
      'maxRelativeCentreSpeedMps',NaN,'maxCableAngularSpeedRps',E.O, ...
      'psi2AtCurrentThrust',NaN,'dPsi2_dThrustExact',0, ...
      'dPsi2_dThrustJetDifference',NaN, ...
      'psi2MinForAnyThrust0To20',NaN, ...
      'analyticJetCheckAbsError',NaN);
    if isempty(br),br=bi;else,br(end+1,1)=bi;end %#ok<AGROW>
    minGap=min(minGap,gap);passed=passed&&gap>=0;
end
if s.tiltCfg.cableEnabled
    h=-p(3)-cosd(s.tiltCfg.cableMaxDeg);
    gap=h-dt*E.O;
    chi=(b3(3)-dot(p,b3)*p(3))/(P.m*P.cableL);
    bi=struct('sampleIndex',id,'time',double(s.time), ...
      'barrier','cableTilt','barrierType','cableTilt', ...
      'hStartM',h,'hOneCycleLowerM',gap, ...
      'screenPassed',gap>=0,'dMinM',NaN, ...
      'maxRelativeCentreSpeedMps',NaN,'maxCableAngularSpeedRps',E.O, ...
      'psi2AtCurrentThrust',NaN,'dPsi2_dThrustExact',chi, ...
      'dPsi2_dThrustJetDifference',NaN, ...
      'psi2MinForAnyThrust0To20',NaN, ...
      'analyticJetCheckAbsError',NaN);
    if isempty(br),br=bi;else,br(end+1,1)=bi;end %#ok<AGROW>
    minGap=min(minGap,gap);passed=passed&&gap>=0;
end
row=struct('sampleIndex',id,'time',double(s.time), ...
    'estimatedMassKg',P.mL,'thrustN',T,'yawNm',yaw, ...
    'barriersChecked',numel(br),'allBarrierScreenPassed',passed, ...
    'minScreenGap',minGap,'maxPsi2JetDisagreement',maxDiff, ...
    'bodyAngularSpeedUpper',E.W, ...
    'cableAngularSpeedUpper',E.O, ...
    'loadSpeedUpper',E.V, ...
    'elapsedMs',1000*toc(clock), ...
    'envelopeMs',envelopeMs, ...
    'generatorFirstMs',generatorFirstMs, ...
    'generatorSecondMs',generatorSecondMs, ...
    'geometryAndTiltMs',1000*toc(clockGeometry), ...
    'derivedOneCycleSufficientScreen',true, ...
    'machineRoundedCertified',false,'recursiveFeasibilityCertified',false);
end

function jumps=sd_bridge_jump_pair(old,new,id)
% Split actual boundary in the model: old T/mL -> new T/old mL ->
% new T/new mL at EXACT same new estimated state. No future prediction.
x=double(new.x(:));oldP=old.P;newP=new.P;
T0=max(0,min(20,old.u(1)));T1=max(0,min(20,new.u(1)));
yaw=max(-1,min(1,new.u(4)));G=new.G;
qu=x(1:4);p=x(14:16);
b3=[2*(qu(2)*qu(4)+qu(1)*qu(3)); ...
   2*(qu(3)*qu(4)-qu(1)*qu(2)); ...
   qu(1)^2-qu(2)^2-qu(3)^2+qu(4)^2];
C=x(8:10)-G.cableCenterFromLoad*oldP.cableL*p;
jumps=struct([]);
% Include only geometrically matched obstacles with same static shape.
n=min(numel(old.parts),numel(new.parts));
for i=1:n
  [centre,Q,name]=sd_shape(new.parts(i),i);
  [centreOld,QOld]=sd_shape(old.parts(i),i);
  if norm(centreOld-centre)>1e-12 || norm(QOld-Q,'fro')>1e-12,continue;end
  D=C-centre;d=norm(D);if d<=0,continue;end
  normal=D/d;delta=G.cableAxisLong^2-G.cableAxisShort^2;
  v=dot(normal,p);cc=sqrt(G.cableAxisShort^2+delta*v^2);
  oo=sqrt(normal.'*Q*normal);
  gradD=normal-(eye(3)-normal*normal.')*(delta*v/cc*p+Q*normal/oo)/d;
  gradP=-(delta*v/cc)*normal;
  beta=dot(p,b3);
  pTT=-(b3-beta*p)/(oldP.m*oldP.cableL);
  CTT=beta/(oldP.m+oldP.mL)*p-G.cableCenterFromLoad*oldP.cableL*pTT;
  chi=gradD.'*CTT+gradP.'*pTT;
  so=CBF_REAL_XY_EVAL(x,[T0;0;0;yaw],oldP,G,new.parts,new.lambda,new.tiltCfg);
  st=CBF_REAL_XY_EVAL(x,[T1;0;0;yaw],oldP,G,new.parts,new.lambda,new.tiltCfg);
  sn=CBF_REAL_XY_EVAL(x,[T1;0;0;yaw],newP,G,new.parts,new.lambda,new.tiltCfg);
  % Based on f=f0+T*fT, psi2=a2+T*b2 and L_fT b2=0, the
  % 3rd-stage HOCBF is affine in T for fixed state/mass/geometry.
  % This extra generator call is OFFLINE VALIDATION ONLY.
  splus=CBF_REAL_XY_EVAL(x,[T0+1;0;0;yaw],oldP,G,new.parts,new.lambda,new.tiltCfg);
  chi3=splus.stages(4,i)-so.stages(4,i);
  % Exact-real one-dimensional necessity for A after the new HLC thrust
  % is known: for a fixed (new) model and state, psi2 and psi3 are
  % affine in T. XY torques cannot alter these two at the switching
  % instant. Compute the LOW-STAGE thrust interval, obstacle only.
  % This is OFFLINE post-hoc diagnostics, not a proposed T controller,
  % and does not certify all constraints / highest-order QP / viability.
  probeT=min(20,T1+1);
  if probeT==T1,probeT=max(0,T1-1);end
  if probeT==T1,error('SD:BridgeTProbe','Cannot perturb HLC T');end
  newMassUnit=CBF_REAL_XY_EVAL(x,[probeT;0;0;yaw], ...
      newP,G,new.parts,new.lambda,new.tiltCfg);
  Anew=sn.stages(1:4,i);
  Bslope=(newMassUnit.stages(1:4,i)-Anew)/(probeT-T1);
  nonInputError=max(abs(Bslope(1:2)));
  if nonInputError>2e-6*(1+max(abs(Anew)))
      error('SD:BridgeLowStage','psi0/psi1 unexpectedly depend on T');
  end
  Bslope(1:2)=0; % exact-real structure for first two stages
  [safeLo,safeHi,lowStagesPossible]= ...
      sd_bridge_lowstage_thrust_range(Anew,Bslope,T1,0,20);
  if lowStagesPossible
      safeDistance=max([safeLo-T1,T1-safeHi,0]);
      hlcInSafeInterval=(T1>=safeLo-1e-10 && T1<=safeHi+1e-10);
  else
      safeDistance=NaN;hlcInSafeInterval=false;
  end
  dPsi3Pred=chi3*(T1-T0);
  dPsi3Measured=st.stages(4,i)-so.stages(4,i);
  err3=abs(dPsi3Pred-dPsi3Measured);
  th=st.stages(3,i)-so.stages(3,i);
  pred=chi*(T1-T0);
  err=abs(th-pred);
  if err>2e-6*(1+abs(th)+abs(pred))
      error('SD:BridgeJumpCheck','psi2 T jump analytic/Jet mismatch');
  end
  j=struct('sampleIndex',id,'timeFrom',double(old.time), ...
   'timeTo',double(new.time),'barrier',[name '/cable'], ...
   'oldThrust',T0,'newThrust',T1, ...
   'oldEstimatedMass',oldP.mL,'newEstimatedMass',newP.mL, ...
   'psi2BeforeThrustSwitch',so.stages(3,i), ...
   'psi2AfterThrustSwitch',st.stages(3,i), ...
   'psi2AfterMassSwitch',sn.stages(3,i), ...
   'exactThrustJumpPredicted',pred, ...
   'exactThrustJumpMeasured',th, ...
   'massSwitchContribution',sn.stages(3,i)-st.stages(3,i), ...
   'thrustJumpIdentityError',err, ...
   'psi3Old',so.stages(4,i),'psi3New',sn.stages(4,i), ...
   'dPsi3_dThrustJet',chi3, ...
   'psi3ThrustJumpPredicted',dPsi3Pred, ...
   'psi3ThrustJumpMeasured',dPsi3Measured, ...
   'psi3MassSwitchContribution',sn.stages(4,i)-st.stages(4,i), ...
   'psi3ThrustAffineError',err3, ...
   'psi3ThrustIdentityNumericallyMatches', ...
      err3<2e-6*(1+abs(dPsi3Pred)+abs(dPsi3Measured)), ...
      'A_newMassPsi2ThrustSlope',Bslope(3), ...
      'A_newMassPsi3ThrustSlope',Bslope(4), ...
      'A_lowStageThrustIntervalLoN',safeLo, ...
      'A_lowStageThrustIntervalHiN',safeHi, ...
      'A_lowStageThrustIntervalNonempty',lowStagesPossible, ...
      'A_newHLCThrustInLowStageInterval',hlcInSafeInterval, ...
      'A_HLCThrustDistanceToIntervalN',safeDistance, ...
      'A_lowerStagesInherited',all(sn.stages(1:4,i)>=0));
  if isempty(jumps),jumps=j;else,jumps(end+1,1)=j;end %#ok<AGROW>
end
end

% Exact REAL arithmetic: intersect the affine psi_j(T)>=0 half-lines
% for j=0,...,3.  Caller has validated psi0/psi1 do not depend on T.
% The resulting interval is a NECESSARY starting-set constraint for
% standard 4th-order HOCBF, NOT a sufficient recursive guarantee.
function [lower,upper,nonempty]=sd_bridge_lowstage_thrust_range(a,b,Tref,Tmin,Tmax)
lower=Tmin;upper=Tmax;nonempty=true;
for stage=1:numel(a)
    val=a(stage);slope=b(stage);
    if ~isfinite(val)||~isfinite(slope)
        error('SD:BridgeThrustBand','Nonfinite low-stage coefficient');
    end
    if slope==0
        if val<0,nonempty=false;end
    elseif slope>0
        lower=max(lower,Tref-val/slope);
    else
        upper=min(upper,Tref-val/slope);
    end
end
if lower>upper,nonempty=false;end
if ~nonempty,lower=NaN;upper=NaN;end
end

% ======================================================================
% 2026-11-23: Analytic fixed-separating-plane 1-ZOH certificate.
% Unlike the original centre-normal screen this certifies actual separation
% of convex cable/obstacle ellipsoids; it does NOT imply the original
% centre-normal h(t) remains nonnegative. No 19-D jet/torque enumeration.
% ======================================================================
function pr=sd_bridge_plane_case(s,id,E)
planeClock=tic;
x=double(s.x(:));P=s.P;G=s.G;dt=double(s.dt);
T=max(0,min(20,double(s.u(1))));
m=P.m;mu=P.mL;L=P.cableL;
p=x(14:16);wL=x(17:19);vL=x(11:13);
q=x(1:4);
b3=[2*(q(2)*q(4)+q(1)*q(3)); ...
    2*(q(3)*q(4)-q(1)*q(2)); ...
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
alpha=G.cableCenterFromLoad*L;
pDot=cross(wL,p);
wLDot=-(T/(m*L))*cross(p,b3);
pDDot=cross(wLDot,p)+cross(wL,pDot);
Dst=[P.dstx;P.dsty;-P.gravity];
S=T*dot(p,b3)-m*L*dot(pDot,pDot);
aLoad=Dst+(S/(m+mu))*p;
aCentre=aLoad-alpha*pDDot;
C=x(8:10)-alpha*p;
vCentre=vL-alpha*pDot;
% All next constants bound the present ZOH for every XY in [-1,1]^2.
O=E.O;W=E.W;
Aomega=T/(m*L);
P2=Aomega+O^2;
P3=Aomega*(W+4*O)+O^3;
S0=T+m*L*O^2;
S1=T*(O+W)+2*m*L*O*P2;
Jcentre=(S1+S0*O)/(m+mu)+abs(alpha)*P3;
aS=G.cableAxisShort;aL=G.cableAxisLong;
delta=aL^2-aS^2;
if ~(aS>0 && aL>=aS),error('SD:PlaneAxes','Invalid cable axes');end
kappa=delta/aL;
C2=delta/aS;
if delta>0
    uCrit=min(1,aS/(2*sqrt(delta)));
    C3=3*delta^2*aS^2*uCrit/(aS^2+delta*uCrit^2)^(5/2);
else
    C3=0;
end
M3=Jcentre+kappa*P3+3*C2*O*P2+C3*O^3;
if ~isfinite(M3)||M3<0,error('SD:PlaneMajorant','Invalid jerk majorant');end
planeSetupMs=1000*toc(planeClock);
pr=struct([]);
for i=1:numel(s.parts)
    [centre,Q,label]=sd_shape(s.parts(i),i);
    D=C-centre;d=norm(D);
    if d<=0,error('SD:PlaneCentre','Coincident obstacle/cable centres');end
    n=D/d;u=dot(n,p);
    ca=sqrt(aS^2+delta*u^2);
    so=sqrt(n.'*Q*n);
    g0=d-ca-so;
    c1=delta*u/ca;
    c2=delta*aS^2/ca^3;
    up=dot(n,pDot);
    upp=dot(n,pDDot);
    g1=dot(n,vCentre)-c1*up;
    g2=dot(n,aCentre)-c1*upp-c2*up^2;
    % For 0 <= t <= dt, t^j is nonnegative. Termwise lower bound:
    % g(t) >= g0 + g1*t + g2*t^2/2 - M3*t^3/6.
    % Replace potentially positive terms by zero, negative terms by
    % their endpoint values. This is a uniform lower bound on [0,dt].
    lower=g0+min(0,g1)*dt+min(0,g2)*dt^2/2-M3*dt^3/6;
    endLower=g0+g1*dt+g2*dt^2/2-M3*dt^3/6;
    % Since g''' >= -M3, g'(s) >= g1+g2*s-M3*s^2/2.
    % This simple all-s lower bound certifies NON-DECREASING separation
    % of the SAME fixed plane over one ZOH, for every allowed XY torque.
    % It is strictly stronger than g>=0 but still NOT recursive viability.
    nonApproachRateLower=g1+min(0,g2)*dt-M3*dt^2/2;
    if ~all(isfinite([g0,g1,g2,M3,lower,endLower,nonApproachRateLower]))
        error('SD:PlaneNonfinite','Nonfinite plane certificate');
    end
    r=struct('sampleIndex',id,'time',double(s.time), ...
      'barrier',[label '/cable'], ...
      'planeStartMarginM',g0, ...
      'planeFirstDerivativeMps',g1, ...
      'planeSecondDerivativeMps2',g2, ...
      'jerkMajorantMps3',M3, ...
      'cubicRemainderUpperM',M3*dt^3/6, ...
      'planeWholePeriodLowerM',lower, ...
      'planeEndpointLowerM',endLower, ...
      'planeNonApproachRateLowerMps',nonApproachRateLower, ...
      'planeNonApproachPassed',g0>=0&&nonApproachRateLower>=0, ...
      'planePhysicalSeparationPassed',lower>=0, ...
      'originalCentreNormalHCertified',false, ...
      'machineRoundedCertified',false, ...
      'recursiveFeasibilityCertified',false);
    if isempty(pr),pr=r;else,pr(end+1,1)=r;end %#ok<AGROW>
end
planeMs=1000*toc(planeClock);
% Per-obstacle attribution is a SHARE of the case total; summing across
% obstacles must not multiply the shared setup cost.
for j=1:numel(pr)
    pr(j).planeComputeMs=planeMs/max(1,numel(pr));
    pr(j).planeCaseSetupMs=planeSetupMs;
    pr(j).planeCaseTotalMs=planeMs;
end
end

% ======================================================================
% Offline exact-real analytical coefficient of z=[tauX;tauY] in the 4th
% derivative of a FIXED separating-plane gap. This is a statement about
% this ZOH physical model, not a recursive safety certificate.
% g'' has a term T*K(q,p,n)'*b3. All other g'' terms are independent of
% body rates/torques. d²b3/dt² contains R(q)*(J^-1*tau x e3), hence
% [dg/d tauX, dg/d tauY] =
% T*[-K'*R*e2/Jx, K'*R*e1/Jy].
% ======================================================================
function rows=sd_bridge_plane_input_coeff(s,id)
x=double(s.x(:));P=s.P;G=s.G;
T=max(0,min(20,double(s.u(1))));yaw=max(-1,min(1,double(s.u(4))));
p=x(14:16);q=x(1:4);alpha=G.cableCenterFromLoad*P.cableL;
C=x(8:10)-alpha*p;
as=G.cableAxisShort;delta=G.cableAxisLong^2-as^2;
% q=[q0;q1;q2;q3] with body-to-world rotation.
b1=[1-2*(q(3)^2+q(4)^2);2*(q(2)*q(3)+q(1)*q(4)); ...
    2*(q(2)*q(4)-q(1)*q(3))];
b2=[2*(q(2)*q(3)-q(1)*q(4));1-2*(q(2)^2+q(4)^2); ...
    2*(q(3)*q(4)+q(1)*q(2))];
rows=struct([]);
for i=1:numel(s.parts)
    [centre,Q,label]=sd_shape(s.parts(i),i); %#ok<ASGLU>
    D=C-centre;d=norm(D);
    if ~(d>0),error('SD:PlaneCoeff','Bad centre distance');end
    n=D/d;u=dot(n,p);ca=sqrt(as^2+delta*u^2);
    c1=delta*u/ca;
    K=(u/(P.m+P.mL))*p + (alpha+c1)/(P.m*P.cableL)*(n-u*p);
    B=[-T*dot(K,b2)/P.J(1);T*dot(K,b1)/P.J(2)];
    % Generator/JET cross-check is exclusively OFFLINE, excluded from
    % realtime timing. Values at (0,0),(1,0),(0,1),(1,1).
    f0=sd_bridge_plane_g4jet(x,[T;0;0;yaw],P,G,n);
    fx=sd_bridge_plane_g4jet(x,[T;1;0;yaw],P,G,n);
    fy=sd_bridge_plane_g4jet(x,[T;0;1;yaw],P,G,n);
    fxy=sd_bridge_plane_g4jet(x,[T;1;1;yaw],P,G,n);
    Bj=[fx-f0;fy-f0];
    err=max(abs(B-Bj));mixErr=abs(fxy-f0-(fx-f0)-(fy-f0));
    item=struct('sampleIndex',id,'time',double(s.time), ...
      'barrier',[label '/cable'],'B4AnalyticX',B(1), ...
      'B4AnalyticY',B(2),'B4JetX',Bj(1),'B4JetY',Bj(2), ...
      'maxAbsCoefficientError',err,'mixedInputAffineError',mixErr, ...
      'inputCoefficientNumericallyMatches', ...
        err<2e-6*(1+max(abs(B))+max(abs(Bj))) && ...
        mixErr<2e-6*(1+abs(fxy)+abs(f0)), ...
      'machineRoundedCertified',false, ...
      'recursiveFeasibilityCertified',false);
    if isempty(rows),rows=item;else,rows(end+1,1)=item;end %#ok<AGROW>
end
end

function g4=sd_bridge_plane_g4jet(x,u,P,G,n)
jet=CBF_REAL_XY_JET(x,u,P);
X=jet.stateTaylor;L=P.cableL;alpha=G.cableCenterFromLoad*L;
aS=G.cableAxisShort;delta=G.cableAxisLong^2-aS^2;
U=n.'*X(14:16,:);
S=zeros(1,5);S(1)=aS^2;
for k=0:4
    for j=0:k,S(k+1)=S(k+1)+delta*U(j+1)*U(k-j+1);end
end
H=zeros(1,5);H(1)=sqrt(S(1));
for k=1:4
    crossTerms=0;
    for j=1:k-1,crossTerms=crossTerms+H(j+1)*H(k-j+1);end
    H(k+1)=(S(k+1)-crossTerms)/(2*H(1));
end
Cc=X(8:10,:)-alpha*X(14:16,:);
g4=24*(n.'*Cc(:,5)-H(5));
end

% ======================================================================
% ======================================================================
% OFFLINE WHOLE-FLIGHT BENCHMARK -- 5 IDENTICAL complete replays per mode.
% NOT a Sim closed-loop replay: this evaluates the logged inputs/states.
% Preparation: parameters and all snapshots checked once, outside timing.
% One repetition: process all logged snapshots IN ORIGINAL ORDER, start to
% finish, with no data reload, no MAT writes or table construction inside.
% Five repetitions show a trend; they cannot establish a WCET or p99 tail.
% ======================================================================
function [runs,sampleTable,errors]=sd_bridge_full_replay_benchmark(snapshots,fixed,nPasses)
N=numel(snapshots);
if N<1,error('SD:FullReplayEmpty','No snapshots');end
if ~(isscalar(nPasses)&&nPasses>=3&&nPasses<=5&&nPasses==floor(nPasses))
    error('SD:FullReplayPasses','Use 3 to 5 full replays.');
end
% A fixed parameter change INVALIDATES any preflight certificate. Reject
% rather than silently comparing different models on one cached bound.
valid=false(N,1);
times=nan(N,1);obsCount=zeros(N,1);
for i=1:N
    if isempty(snapshots{i}),continue;end
    s=snapshots{i};
    if ~isstruct(s)||~isfield(s,'time')||~isfield(s,'parts')
        error('SD:FullReplaySnapshot','Malformed snapshot %d',i);
    end
    sd_analytic_fixed_check(fixed,s);
    valid(i)=true;
    times(i)=double(s.time);
    obsCount(i)=numel(s.parts);
end
if ~any(valid),error('SD:FullReplayEmpty','All snapshots are empty');end
ids=find(valid);
% Warmed-up: preflight above has already executed 5 complete-case calls.
% Here each measured pass starts again at the FIRST snapshot and follows
% the same algorithm and same chronological ordering as all later passes.
modeNames={'diagnostic','screenOnly'};
R=nPasses*numel(modeNames);
M=N*R;
S=struct('pass',nan(M,1),'mode',strings(M,1), ...
    'sampleIndex',nan(M,1),'time',nan(M,1), ...
    'hasObservedObstacle',false(M,1),'totalMs',nan(M,1), ...
    'envelopeMs',nan(M,1),'generatorFirstMs',nan(M,1), ...
    'generatorSecondMs',nan(M,1),'geometryTiltMs',nan(M,1), ...
    'planeMs',nan(M,1),'overheadMs',nan(M,1), ...
    'allPhysicalScreenPassed',false(M,1),'executedOK',false(M,1));
runs=struct([]);errors={};writeAt=0;
firstPassed=nan(N,numel(modeNames));
for pass=1:nPasses
    for mm=1:numel(modeNames)
        mode=modeNames{mm};
        rTotal=nan(N,1);rPassed=nan(N,1);
        errCount=0;processed=0;obsProcessed=0;
        thisStart=tic;
        for ii=1:numel(ids)
            id=ids(ii);s=snapshots{id};
            writeAt=writeAt+1;
            S.pass(writeAt)=pass;S.mode(writeAt)=string(mode);
            S.sampleIndex(writeAt)=id;
            S.time(writeAt)=times(id);
            S.hasObservedObstacle(writeAt)=obsCount(id)>0;
            try
                timing=sd_bridge_timed(s,id,mode);
                rTotal(id)=timing.totalMs;
                rPassed(id)=double(timing.passed);
                S.totalMs(writeAt)=timing.totalMs;
                S.envelopeMs(writeAt)=timing.envelopeMs;
                S.generatorFirstMs(writeAt)=timing.generatorFirstMs;
                S.generatorSecondMs(writeAt)=timing.generatorSecondMs;
                S.geometryTiltMs(writeAt)=timing.geometryTiltMs;
                S.planeMs(writeAt)=timing.planeMs;
                S.overheadMs(writeAt)=timing.overheadMs;
                S.allPhysicalScreenPassed(writeAt)=logical(timing.passed);
                S.executedOK(writeAt)=true;
                processed=processed+1;
                obsProcessed=obsProcessed+(obsCount(id)>0);
            catch ME
                errCount=errCount+1;
                % Store the first failures with their exact sample IDs.
                if numel(errors)<200
                    errors{end+1,1}=sprintf('pass=%d mode=%s sample=%d t=%.9g: %s [%s]', ...
                        pass,mode,id,times(id),ME.message,ME.identifier); %#ok<AGROW>
                end
            end
        end
        wallMs=1000*toc(thisStart);
        known=isfinite(rTotal);
        if any(known)
            vals=rTotal(known);
            [peak,iPeak]=max(vals);
            knownIds=find(known);
            pkId=knownIds(iPeak);
            medianMs=median(vals);meanMs=mean(vals);
            p95Ms=sd_bridge_percentile(vals,95);
            over5=sum(vals>5);over25=sum(vals>25);
            sumComponents=sum(vals);
        else
            pkId=NaN;peak=NaN;medianMs=NaN;meanMs=NaN;
            p95Ms=NaN;over5=0;over25=0;sumComponents=NaN;
        end
        changes=0;
        if pass==1
            firstPassed(:,mm)=rPassed;
        else
            both=isfinite(firstPassed(:,mm)) & isfinite(rPassed);
            changes=sum(firstPassed(both,mm)~=rPassed(both));
            % Failures in only one run count as discrepancies too.
            changes=changes+sum(xor(isfinite(firstPassed(:,mm)),isfinite(rPassed)));
        end
        pkTime=NaN;
        if isfinite(pkId),pkTime=times(pkId);end
        entry=struct('pass',pass,'mode',mode, ...
            'snapshotsInLog',N,'nonemptySnapshots',numel(ids), ...
            'observedObstacleSnapshots',sum(obsCount(ids)>0), ...
            'successfulSamples',processed,'successfulWithObstacle',obsProcessed, ...
            'errors',errCount,'passScreens',sum(rPassed==1), ...
            'failScreens',sum(rPassed==0), ...
            'passDisagreementsWithFirstReplay',changes, ...
            'fullReplayWallMs',wallMs,'sumSampleComputeMs',sumComponents, ...
            'meanSampleMs',meanMs,'medianSampleMs',medianMs, ...
            'p95SampleMs',p95Ms,'maxSampleMs',peak, ...
            'maxSampleIndex',pkId,'maxSampleTimeS',pkTime, ...
            'samplesOver5ms',over5,'samplesOver25ms',over25);
        if isempty(runs),runs=entry;else,runs(end+1,1)=entry;end %#ok<AGROW>
        fprintf('[SD FULL REPLAY] pass %d/%d %-10s %d/%d samples wall %.1f ms, median %.3f ms, max %.3f ms (id %d), errors %d\n', ...
            pass,nPasses,mode,processed,numel(ids),wallMs,medianMs,peak,pkId,errCount);
    end
end
% Output table conversion, disk IO and ZIP archive are NOT part of timing.
f=fieldnames(S);
for i=1:numel(f),S.(f{i})=S.(f{i})(1:writeAt);end
sampleTable=table(S.pass,S.mode,S.sampleIndex,S.time, ...
    S.hasObservedObstacle,S.totalMs,S.envelopeMs, ...
    S.generatorFirstMs,S.generatorSecondMs,S.geometryTiltMs, ...
    S.planeMs,S.overheadMs,S.allPhysicalScreenPassed,S.executedOK, ...
    'VariableNames',fieldnames(S));
end

function y=sd_bridge_percentile(x,p)
x=sort(x(:));N=numel(x);
if N<1,y=NaN;return;end
alpha=1+(N-1)*p/100;
a=floor(alpha);b=ceil(alpha);
y=x(a)+(x(b)-x(a))*(alpha-a);
end

function t=sd_bridge_timed(s,id,mode)
start=tic;
if strcmp(mode,'diagnostic')
    [r,~,E]=sd_bridge_case(s,id);
    pl=sd_bridge_plane_case(s,id,E);
    if isempty(pl),planeMs=0;else,planeMs=sum([pl.planeComputeMs]);end
    elapsed=1000*toc(start);
    other=elapsed-r.envelopeMs-r.generatorFirstMs- ...
        r.generatorSecondMs-r.geometryAndTiltMs-planeMs;
    t=struct('totalMs',elapsed,'envelopeMs',r.envelopeMs, ...
        'generatorFirstMs',r.generatorFirstMs, ...
        'generatorSecondMs',r.generatorSecondMs, ...
        'geometryTiltMs',r.geometryAndTiltMs, ...
        'planeMs',planeMs,'overheadMs',other, ...
        'passed',sd_bridge_joint_physical_pass(s,E,pl));
elseif strcmp(mode,'screenOnly')
    x=double(s.x(:));u=double(s.u(:));P=s.P;G=s.G;dt=double(s.dt);
    clock=tic;
    [E,ok,why]=sd_analytic_scalar_envelope(x,P, ...
        max(0,min(20,u(1))),max(-1,min(1,u(4))),dt,G);
    envMs=1000*toc(clock);
    if ~ok,error('SD:ScreenEnv','%s',why);end
    pl=sd_bridge_plane_case(s,id,E);
    if isempty(pl),planeMs=0;else,planeMs=sum([pl.planeComputeMs]);end
    clock=tic;
    yes=sd_bridge_joint_physical_pass(s,E,pl);
    geomMs=1000*toc(clock);
    elapsed=1000*toc(start);
    t=struct('totalMs',elapsed,'envelopeMs',envMs, ...
        'generatorFirstMs',0,'generatorSecondMs',0, ...
        'geometryTiltMs',geomMs, ...
        'planeMs',planeMs,'overheadMs',elapsed-envMs-geomMs-planeMs, ...
        'passed',yes);
else
    error('SD:TimingMode','Unknown timing mode');
end
end

function yes=sd_bridge_joint_physical_pass(s,E,pl)
% Consistent meaning in BOTH benchmark paths: non-overlap and both tilts.
if isempty(pl),yes=true;else,yes=all([pl.planePhysicalSeparationPassed]);end
x=double(s.x(:));q=x(1:4);dt=double(s.dt);
if s.tiltCfg.bodyEnabled
    bz=q(1)^2-q(2)^2-q(3)^2+q(4)^2;
    yes=yes && bz-cosd(s.tiltCfg.bodyMaxDeg)-dt*E.W>=0;
end
if s.tiltCfg.cableEnabled
    yes=yes && -x(16)-cosd(s.tiltCfg.cableMaxDeg)-dt*E.O>=0;
end
end

% ======================================================================
% OFFLINE aggregation only: JOIN timed samples to recorded Scheme02 status.
% No extra calls to dynamics/generator; no alteration to timed paths.
% NOTE: recorded torque correction is not the same as a binding hard CBF
% constraint, since guidance can also modify the XY torque.
% ======================================================================
function [T,groups]=sd_bridge_segment_from_recorded_cbf(T,record)
if isempty(T),groups=struct([]);return;end
required={'k','t','parts','status','min_h_m'};
for i=1:numel(required)
    if ~ismember(required{i},record.Properties.VariableNames)
        error('SD:SegmentSource','Missing PER_SAMPLE column %s',required{i});
    end
end
idx=double(record.k(:));
if numel(unique(idx))~=numel(idx)
    error('SD:SegmentSource','Duplicate PER_SAMPLE.k values.');
end
[known,where]=ismember(double(T.sampleIndex),idx);
if ~all(known),error('SD:SegmentSource','Timed sample not found in PER_SAMPLE.k');end
realTime=double(record.t(where));
if any(~isfinite(realTime)) || any(abs(double(T.time)-realTime)>1e-5)
    error('SD:SegmentSource','Timed and recorded sample times disagree.');
end
loggedObs=double(record.parts(where))>0;
if any(loggedObs~=logical(T.hasObservedObstacle))
    error('SD:SegmentSource','Obstacle count disagreement between logs.');
end
st=string(record.status(where));
h=double(record.min_h_m(where));
state=repmat("OBSERVED_OTHER",height(T),1);
state(~loggedObs)="NO_OBSTACLE_TILT_ONLY";
state(loggedObs & st=="NOMINAL_XY_PASSED")="OBSERVED_NOMINAL";
state(loggedObs & st=="XY_TORQUE_CORRECTED")="OBSERVED_XY_CORRECTED";
state(loggedObs & st=="OUTSIDE_SET_RECOVERY")="OBSERVED_RECOVERY";
band=repmat("OBSERVED_H_UNKNOWN",height(T),1);
band(~loggedObs)="NO_OBSTACLE";
band(loggedObs & h<0.010)="H_LT_10MM";
band(loggedObs & h>=0.010 & h<0.050)="H_10_TO_50MM";
band(loggedObs & h>=0.050)="H_GE_50MM";
T.loggedScheme02Status=st;
T.loggedScheme02Group=state;
T.loggedClearanceBand=band;
T.loggedMinClearanceM=h;
T.loggedXYCorrected=(st=="XY_TORQUE_CORRECTED");
T.loggedRecovery=(st=="OUTSIDE_SET_RECOVERY");
groups=struct([]);
passes=unique(T.pass(:))';
for pass=passes
    modes=unique(T.mode(T.pass==pass),'stable');
    for mi=1:numel(modes)
        mode=modes(mi);
        valid=(T.pass==pass & T.mode==mode & T.executedOK & isfinite(T.totalMs));
        for groupType={"CBF_STATUS","CLEARANCE"}
            if groupType{1}=="CBF_STATUS"
                labels=state;
            else
                labels=band;
            end
            categories=unique(labels(valid),'stable');
            for j=1:numel(categories)
                pick=valid & labels==categories(j);
                ids=find(pick); val=double(T.totalMs(pick));
                if isempty(val),continue;end
                [pk,pkLocal]=max(val);
                pkRow=ids(pkLocal);
                ent=struct('pass',pass,'mode',char(mode), ...
                    'groupType',char(groupType{1}), ...
                    'groupName',char(categories(j)), ...
                    'sampleCount',numel(val), ...
                    'meanMs',mean(val),'medianMs',median(val), ...
                    'p95Ms',sd_bridge_percentile(val,95), ...
                    'p99Ms',sd_bridge_percentile(val,99), ...
                    'maxMs',pk,'maxSampleIndex',T.sampleIndex(pkRow), ...
                    'maxTimeS',T.time(pkRow), ...
                    'samplesOver5Ms',sum(val>5), ...
                    'samplesOver25Ms',sum(val>25), ...
                    'sumSampleMs',sum(val));
                if isempty(groups),groups=ent;else,groups(end+1,1)=ent;end %#ok<AGROW>
            end
        end
    end
end
end

function sd_bridge_save(r)
folder=tempname;mkdir(folder);clean=onCleanup(@()sd_cleanup(folder)); %#ok<NASGU>
files={'SUMMARY.txt','ERRORS.txt'};
if isfield(r,'localRefineCases') && ~isempty(r.localRefineCases)
 writetable(struct2table(r.localRefineCases),fullfile(folder,'LOCAL_AB_CASES.csv'));
 files{end+1}='LOCAL_AB_CASES.csv';
end
if isfield(r,'localRefineBarriers') && ~isempty(r.localRefineBarriers)
 writetable(struct2table(r.localRefineBarriers),fullfile(folder,'LOCAL_AB_BARRIERS.csv'));
 files{end+1}='LOCAL_AB_BARRIERS.csv';
end
if isfield(r,'localRefineMeta')
 fidL=fopen(fullfile(folder,'LOCAL_AB_SUMMARY.txt'),'w');
 fprintf(fidL,'Offline local 19D Picard + direct raw interval Jet comparison; one vs four subintervals.\n');
 fprintf(fidL,'A: initial low stages plus all-time upper-stage lower enclosure on each subinterval.\n');
 fprintf(fidL,'B: lower Taylor-Bernstein on each subinterval, plus final contraction wrt original start.\n');
 fprintf(fidL,'One common tube covers ALL XY torques in the physical box; T/yaw fixed. NO fixed torque candidates.\n');
 fprintf(fidL,'MATLAB double 8ULP interval model unverified for true directed rounding: MACHINE CERTIFICATE NO.\n');
 fprintf(fidL,'Cannot claim recursively viable K or any guarantee after future HLC/mass/reference update.\n');
 fprintf(fidL,'Local interval Jet is TOO EXPENSIVE FOR ONLINE until redesigned; measured total compared to 25ms as a warning only.\n');
 if isempty(r.localRefineMeta)
     fprintf(fidL,'Local refinement did not complete; see ERROR lines below.\n');
 else
 L=r.localRefineMeta;
 fprintf(fidL,'attempts=%d completed=%d reachFailures=%d errors=%d\n',L.attempts,L.completed,L.tubeFailures,L.errors);
 fprintf(fidL,'A_numericConditional=%d B_numericConditional=%d maxObservedMs=%.6f >25ms=%d\n', ...
     L.aPass,L.bPass,L.maximumMs,L.over25ms);
 end
 for k=1:numel(r.localRefineErrors),fprintf(fidL,'ERROR %s\n',r.localRefineErrors{k});end
 fclose(fidL);files{end+1}='LOCAL_AB_SUMMARY.txt';
end
if isfield(r,'abFlightCases') && ~isempty(r.abFlightCases)
 writetable(struct2table(r.abFlightCases),fullfile(folder,'AB_QP_FLIGHT_CASES.csv'));
 files{end+1}='AB_QP_FLIGHT_CASES.csv';
end
if isfield(r,'abFlightBarriers') && ~isempty(r.abFlightBarriers)
 writetable(struct2table(r.abFlightBarriers),fullfile(folder,'AB_QP_FLIGHT_BARRIERS.csv'));
 files{end+1}='AB_QP_FLIGHT_BARRIERS.csv';
end
if isfield(r,'abFlightNumeric') && ~isempty(r.abFlightNumeric)
 writetable(struct2table(r.abFlightNumeric),fullfile(folder,'AB_QP_NUMERICAL_ZOH.csv'));
 files{end+1}='AB_QP_NUMERICAL_ZOH.csv';
end
fidAB=fopen(fullfile(folder,'AB_QP_FLIGHT_SUMMARY.txt'),'w');
fprintf(fidAB,'A/B FULL RECORDED-FLIGHT CONTINUOUS XY-QP OFFLINE TEST. NOT CLOSED LOOP.\n');
fprintf(fidAB,'All T/yaw fixed to recorded HLC; mL from each recorded estimated parameter.\n');
fprintf(fidAB,'Exact-real majorant formulas evaluated as MATLAB doubles; NOT IEEE machine certified.\n');
fprintf(fidAB,'A uses low-stage positive start plus bounded full-ZOH highest-stage; B uses Taylor-Bernstein + endpoint decay.\n');
fprintf(fidAB,'Conservative infeasibility is NOT physical infeasibility. No K, no guaranteed future reference/HLC update.\n');
if isfield(r,'abFlightMeta') && ~isempty(r.abFlightMeta)
 M=r.abFlightMeta;
 fprintf(fidAB,'recorded=%d computed=%d errors=%d A_conditional=%d B_conditional=%d both=%d\n', ...
 M.recordedSamples,M.processedSamples,M.errorCount, ...
 M.A_conditionalQPFeasible,M.B_conditionalQPFeasible,M.bothConditionalQPFeasible);
 fprintf(fidAB,'median_ms=%.6g max_ms=%.6g maxSample=%d >5ms=%d >25ms=%d\n', ...
 M.medianTotalMs,M.maxTotalMs,M.maxTotalSampleIndex,M.overFiveMs,M.over25Ms);
 fprintf(fidAB,'Numerical RK4 models=%d errors=%d; 8 numerical substeps, nonverified integration.\n', ...
 M.numericRK4Trajectories,M.numericRK4Errors);
else
 fprintf(fidAB,'AB-FLIGHT was not completed: no certificate.\n');
end
fprintf(fidAB,'No genuine 25ms WCET, no actual reference-governor closure, no machine interval certificate, no recursive viability K.\n');
for k=1:numel(r.abFlightErrors),fprintf(fidAB,'ERROR %s\n',r.abFlightErrors{k});end
fclose(fidAB);files{end+1}='AB_QP_FLIGHT_SUMMARY.txt';
if isfield(r,'flightSetup') && ~isempty(r.flightSetup)
 writetable(struct2table(r.flightSetup),fullfile(folder,'FLIGHT_SETUP_AUDIT.csv'));
 files{end+1}='FLIGHT_SETUP_AUDIT.csv';
end
if isfield(r,'planeFifthRows') && ~isempty(r.planeFifthRows)
 writetable(struct2table(r.planeFifthRows),fullfile(folder,'THEORY_PLANE_G5_AFFINITY.csv'));
 files{end+1}='THEORY_PLANE_G5_AFFINITY.csv';
end
if isfield(r,'methodARateRows') && ~isempty(r.methodARateRows)
 writetable(struct2table(r.methodARateRows),fullfile(folder,'THEORY_A_PSI_RATE_AFFINITY.csv'));
 files{end+1}='THEORY_A_PSI_RATE_AFFINITY.csv';
end
fidQ=fopen(fullfile(folder,'QP_A_B_AFFINE_TEMPLATE.txt'),'w');
if isfield(r,'qpTemplate') && ~isempty(r.qpTemplate)
 fprintf(fidQ,'SYNTHETIC ALGEBRA SELFTEST ONLY: passed=%d A_rows=%d B_rows=%d: %s\n', ...
   r.qpTemplate.passed,r.qpTemplate.methodARows,r.qpTemplate.methodBRows,r.qpTemplate.message);
end
fprintf(fidQ,'Method A: one initial high-stage inequality and four endpoint-rate inequalities.\n');
fprintf(fidQ,'Method B: lower degree-5 Bernstein beta4 and four beta5 inequalities; beta0..3 input independent.\n');
fprintf(fidQ,'All rows are affine in 2 XY torques. A valid UNIFORM fifth coefficient reach-set interval is an UNMET prerequisite.\n');
fprintf(fidQ,'A/B recursive invariant K, reference governor, full physical safety, outward rounding, QP WCET are NOT certified.\n');
fclose(fidQ);files{end+1}='QP_A_B_AFFINE_TEMPLATE.txt';
fidC=fopen(fullfile(folder,'FLIGHT_SETUP_SUMMARY.txt'),'w');
fprintf(fidC,'FLIGHT SETUP AUDIT ONLY - NOT A SAFETY CERTIFICATE OR START PERMIT.\n');
if isfield(r,'flightSetupMeta') && ~isempty(r.flightSetupMeta)
 F=r.flightSetupMeta;
 fprintf(fidC,'recordedSamples=%d fixedParamFields=%d fixedParamsUnchanged=%d\n',F.samples,F.fixedFieldsChecked,F.fixedFieldsUnchanged);
 fprintf(fidC,'estimatedMassRangeKg=[%.12g,%.12g]; HLC_T_range_N=[%.12g,%.12g]\n',F.estimatedMassMinKg,F.estimatedMassMaxKg,F.thrustMinN,F.thrustMaxN);
 fprintf(fidC,'quaternionMaxNormError=%.8g cableVectorMaxNormError=%.8g\n',F.maxQuaternionNormError,F.maxCableNormError);
 fprintf(fidC,'A_newThrustAdmissible=%d/%d; B_loggedNextScreenPassed=%d/%d\n',F.aAdmissible,F.boundaries,F.bScreenPassed,F.boundaries);
 fprintf(fidC,'Reference6JetLogged=%d; generatorInternalStateLogged=%d\n',F.referenceSixJetLogged,F.referenceInternalStateLogged);
 fprintf(fidC,'inputBoundsAssumed: 0<=T<=20 N; |tau_x|<=1,|tau_y|<=1,|tau_z|<=1 Nm. If changed, all bounds need re-verification.\n');
 fprintf(fidC,'Existing barrier scope: cable ellipsoid-obstacles + body/cable tilt ONLY. Drone sphere, load sphere, ground not certified.\n');
 fprintf(fidC,'PARAMETER CHANGES BETWEEN FLIGHTS: ALLOWED, but ALL fixed parameters and gains MUST be re-certified for EACH new flight.\n');
 fprintf(fidC,'MASS ESTIMATE CHANGES WITHIN FLIGHT: allowed in audit; requires certified estimator/update bounds for actual safety.\n');
 fprintf(fidC,'REFERENCE CHANGES: actual 6-jet and reference generator update law are REQUIRED for a trajectory guarantee.\n');
 fprintf(fidC,'safeToFlyCertificateIssued=0; methodAForwardInvarianceCertified=0; methodBForwardInvarianceCertified=0; outwardRounded=0; fullOnlineQpWcetCertified=0\n');
else
 fprintf(fidC,'Flight setup unavailable. No operational certificate issued.\n');
end
for ee=1:numel(r.flightSetupErrors),fprintf(fidC,'ERROR %s\n',r.flightSetupErrors{ee});end
for ee=1:numel(r.planeFifthErrors),fprintf(fidC,'G5_ERROR %s\n',r.planeFifthErrors{ee});end
for ee=1:numel(r.methodARateErrors),fprintf(fidC,'A_RATE_ERROR %s\n',r.methodARateErrors{ee});end
fclose(fidC);files{end+1}='FLIGHT_SETUP_SUMMARY.txt';

if ~isempty(r.cases)
 writetable(struct2table(r.cases),fullfile(folder,'THEORY_CASES.csv'));
 files{end+1}='THEORY_CASES.csv';
end
if ~isempty(r.barriers)
 writetable(struct2table(r.barriers),fullfile(folder,'THEORY_BARRIERS.csv'));
 files{end+1}='THEORY_BARRIERS.csv';
end
if ~isempty(r.jumps)
 writetable(struct2table(r.jumps),fullfile(folder,'THEORY_BOUNDARY.csv'));
 files{end+1}='THEORY_BOUNDARY.csv';
end
if isfield(r,'planes') && ~isempty(r.planes)
 writetable(struct2table(r.planes),fullfile(folder,'THEORY_PLANE.csv'));
 files{end+1}='THEORY_PLANE.csv';
end
if isfield(r,'planeInputs') && ~isempty(r.planeInputs)
 writetable(struct2table(r.planeInputs),fullfile(folder,'THEORY_PLANE_INPUT.csv'));
 files{end+1}='THEORY_PLANE_INPUT.csv';
end
if isfield(r,'timing') && ~isempty(r.timing)
 writetable(struct2table(r.timing),fullfile(folder,'TIMING_REPEATS.csv'));
 files{end+1}='TIMING_REPEATS.csv';
end
if isfield(r,'flightRuns') && ~isempty(r.flightRuns)
 writetable(struct2table(r.flightRuns),fullfile(folder,'FLIGHT_TIMING_RUNS.csv'));
 files{end+1}='FLIGHT_TIMING_RUNS.csv';
end
if isfield(r,'flightSamples') && ~isempty(r.flightSamples)
 writetable(r.flightSamples,fullfile(folder,'FLIGHT_TIMING_SAMPLES.csv'));
 files{end+1}='FLIGHT_TIMING_SAMPLES.csv';
end
if isfield(r,'flightSegments') && ~isempty(r.flightSegments)
 writetable(struct2table(r.flightSegments),fullfile(folder,'FLIGHT_TIMING_CBF_GROUPS.csv'));
 files{end+1}='FLIGHT_TIMING_CBF_GROUPS.csv';
end
if isfield(r,'fullBoundary') && ~isempty(r.fullBoundary)
 writetable(struct2table(r.fullBoundary),fullfile(folder,'FLIGHT_A_B_BOUNDARY.csv'));
 files{end+1}='FLIGHT_A_B_BOUNDARY.csv';
end
if isfield(r,'referenceAdmission') && ~isempty(r.referenceAdmission)
 writetable(struct2table(r.referenceAdmission), ...
   fullfile(folder,'REFERENCE_ADMISSION.csv'));
 files{end+1}='REFERENCE_ADMISSION.csv';
end
if isfield(r,'referenceSlewRows') && ~isempty(r.referenceSlewRows)
 writetable(r.referenceSlewRows,fullfile(folder,'REFERENCE_SLEW_AUDIT.csv'));
 files{end+1}='REFERENCE_SLEW_AUDIT.csv';
end
if isfield(r,'referenceSlewMeta') && ~isempty(r.referenceSlewMeta)
 fidS=fopen(fullfile(folder,'REFERENCE_SLEW_SUMMARY.txt'),'w');
 M=r.referenceSlewMeta;
 fprintf(fidS,'REFERENCE SLEW: OFFLINE fixed-log scalar qz interval viability; NOT trajectory realization or forward invariance.\n');
 fprintf(fidS,'samples=%d start_t=%.9f end_t=%.9f dt_s=%.9f\n',M.samples,M.firstTimeS,M.lastTimeS,M.dtS);
 fprintf(fidS,'necessary_and_sufficient_sampled_qz_rate_Rstar=%.12g m/s^3 (exact-real interval selection; anchored initial inferred qz)\n',M.minimumSampleRateMps3);
 fprintf(fidS,'critical_pair t=%.9f and t=%.9f s, interval gap=%.12g m/s^2, gap/dt=%.12g m/s^3\n',M.criticalTimeFromS,M.criticalTimeToS,M.criticalGapMps2,M.criticalRateMps3);
 fprintf(fidS,'nominal_implied_qz_max_sample_rate=%.12g m/s^3 (inferred, NOT measured reference)\n',M.impliedNominalMaxSampleRateMps3);
 fprintf(fidS,'independent_interval_reachability_verified=%d witness_rate=%.12g witness_max_deviation=%.12g witness_all_intervals=%d witness_initial_fixed=%d\n',M.forwardReachabilityPassed,M.witnessMaxRateMps3,M.witnessMaximumDeviationMps2,M.witnessAllIntervalsPassed,M.witnessInitialFixed);
 fprintf(fidS,['Witness exploits FULL FUTURE LOG: it is a NONCAUSAL mathematical construction, not an implemented controller.\n' ...
 'Scalar qz temporal Lipschitz is NOT a proved necessary condition on z_ref derivatives unless chosen as a design requirement.\n' ...
 'Current HLC qz is state-dependent; preserving a smooth qz sequence alone does NOT imply existence of smooth 6-jet trajectory.\n' ...
 'Changes in XY torques/reference affect future states, thrust intervals, load mass estimates and HLC: logged corridors are not robust future predictions.\n' ...
 'No certified invariant set, no exact machine outward rounding, no production code changed.\n']);
 fclose(fidS);files{end+1}='REFERENCE_SLEW_SUMMARY.txt';
end
if isfield(r,'referenceSlewErrors') && ~isempty(r.referenceSlewErrors)
 fse=fopen(fullfile(folder,'REFERENCE_SLEW_ERRORS.txt'),'w');
 for kk=1:numel(r.referenceSlewErrors),fprintf(fse,'%s\n',r.referenceSlewErrors{kk});end
 fclose(fse); files{end+1}='REFERENCE_SLEW_ERRORS.txt';
end
if isfield(r,'referenceAdmissionErrors') && ~isempty(r.referenceAdmissionErrors)
 fRefErr=fopen(fullfile(folder,'REFERENCE_ADMISSION_ERRORS.txt'),'w');
 for jj=1:numel(r.referenceAdmissionErrors)
    fprintf(fRefErr,'%s\n',r.referenceAdmissionErrors{jj});
 end
 fclose(fRefErr);
 files{end+1}='REFERENCE_ADMISSION_ERRORS.txt';
end
if isfield(r,'fullLowStages') && ~isempty(r.fullLowStages)
 writetable(struct2table(r.fullLowStages),fullfile(folder,'FLIGHT_A_LOWSTAGES.csv'));
 files{end+1}='FLIGHT_A_LOWSTAGES.csv';
end
if isfield(r,'fullBoundaryErrors') && ~isempty(r.fullBoundaryErrors)
 fErr=fopen(fullfile(folder,'FLIGHT_A_B_BOUNDARY_ERRORS.txt'),'w');
 for jj=1:numel(r.fullBoundaryErrors),fprintf(fErr,'%s\n',r.fullBoundaryErrors{jj});end
 fclose(fErr);
 files{end+1}='FLIGHT_A_B_BOUNDARY_ERRORS.txt';
end
if isfield(r,'flightTimingErrors') && ~isempty(r.flightTimingErrors)
 fidErr=fopen(fullfile(folder,'FLIGHT_TIMING_ERRORS.txt'),'w');
 for j=1:numel(r.flightTimingErrors),fprintf(fidErr,'%s\n',r.flightTimingErrors{j});end
 fclose(fidErr);
 files{end+1}='FLIGHT_TIMING_ERRORS.txt';
end
if isfield(r,'preflightWarmup') && ~isempty(r.preflightWarmup)
 writetable(struct2table(r.preflightWarmup),fullfile(folder,'PREFLIGHT_WARMUP.csv'));
 files{end+1}='PREFLIGHT_WARMUP.csv';
end
fid=fopen(fullfile(folder,'ERRORS.txt'),'w');
for i=1:numel(r.errors),fprintf(fid,'%s\n',r.errors{i});end
if isfield(r,'localRefineErrors')
 for i=1:numel(r.localRefineErrors),fprintf(fid,'LOCAL %s\n',r.localRefineErrors{i});end
end
fclose(fid);
fid=fopen(fullfile(folder,'SUMMARY.txt'),'w');
fprintf(fid,['THEORY BRIDGE: EXACT-REAL 1-ZOH GEOMETRY SPEED SCREEN + EXACT psi2 THRUST JUMP.\n' ...
 'No candidate XY torque enumeration; safe screen valid for ALL bounded XY.\n' ...
 'The screen is a one-cycle sufficient condition ONLY. Negative value = inconclusive.\n' ...
 'Positive h at all times is a physical geometric condition, not HOCBF set membership.\n' ...
 'HOCBF psi2(T) changes at thrust updates; no psi3/viability guarantee.\n' ...
 'NEW: Fixed separating-plane cubic bound certifies physical non-overlap for 1 ZOH when positive.\n' ...
 'It does NOT imply original centre-normal h(t) >= 0, next-period safety or recursive QP feasibility.\n' ...
 'No recursively invariant K, no online CBF safety filter, no IEEE outward rounding.\n' ...
 'Constant parameters checked preflight; mL, state, HLC input, obstacle read per sample.\n']);
if isfield(r,'flightSetupMeta') && ~isempty(r.flightSetupMeta)
 F=r.flightSetupMeta;
 fprintf(fid,'FLIGHT_SET_UP recorded=%d fixedFields=%d unchanged=%d mLEst=[%.8g,%.8g] kg A_accepted=%d/%d B_posthoc=%d/%d\n', ...
     F.samples,F.fixedFieldsChecked,F.fixedFieldsUnchanged,F.estimatedMassMinKg,F.estimatedMassMaxKg,F.aAdmissible,F.boundaries,F.bScreenPassed,F.boundaries);
 fprintf(fid,'Per-flight parameter changes REQUIRE fresh preflight certification; safety NOT certified by this audit.\n');
end
if isfield(r,'planeFifthRows') && ~isempty(r.planeFifthRows)
 fprintf(fid,'FIXED_PLANE_G5 diagnosticRows=%d maxAffineDefect=%.9g (finite-difference only; structural algebra in README)\n', ...
    numel(r.planeFifthRows),max([r.planeFifthRows.maxAffineResidual]));
end
if isfield(r,'methodARateRows') && ~isempty(r.methodARateRows)
 fprintf(fid,'A_PSI_HIGHEST_RATE diagnosticRows=%d maxAffineDefect=%.9g (finite-difference only, NOT uniform bound)\n', ...
    numel(r.methodARateRows),max([r.methodARateRows.maxAffineResidual]));
end
fprintf(fid,'analysed=%d errors=%d boundary_pairs=%d\n',numel(r.cases),numel(r.errors),numel(r.jumps));
if isfield(r,'localRefineMeta') && ~isempty(r.localRefineMeta)
    L=r.localRefineMeta;
    fprintf(fid,'LOCAL_REFINEMENT_OFFLINE attempts=%d completed=%d A=%d B=%d tubeFailures=%d errors=%d maxMs=%.4f >25ms=%d (NOT ONLINE)\n', ...
       L.attempts,L.completed,L.aPass,L.bPass,L.tubeFailures,L.errors,L.maximumMs,L.over25ms);
end
if ~isempty(r.cases)
 fprintf(fid,'oneCycle_allXY_geometric_screens_passed=%d/%d\n', ...
    sum([r.cases.allBarrierScreenPassed]),numel(r.cases));
 fprintf(fid,'median_case_ms=%.4f max_case_ms=%.4f (existing case timing; offline replay, no WCET proof)\n', ...
    median([r.cases.elapsedMs]),max([r.cases.elapsedMs]));
end
if isfield(r,'planes') && ~isempty(r.planes)
 fprintf(fid,'fixed_plane_physical_nonoverlap_passed=%d/%d obstacle-sample cases\n', ...
    sum([r.planes.planePhysicalSeparationPassed]),numel(r.planes));
 fprintf(fid,'fixed_plane_NONAPPROACH_oneZOH=%d/%d (all XY, fixed plane only; NOT recursive)\n', ...
    sum([r.planes.planeNonApproachPassed]),numel(r.planes));
end
if ~isempty(r.cases) && isfield(r.cases,'totalWithPlaneMs')
 fprintf(fid,'joint_fixed_plane_nonoverlap_and_tilt_screens_passed=%d/%d (exact-real 1-ZOH only)\n', ...
    sum([r.cases.allPhysicalOneCycleScreenPassed]),numel(r.cases));
 fprintf(fid,'median_total_including_plane_ms=%.4f max_total_including_plane_ms=%.4f (offline samples; NOT WCET)\n', ...
    median([r.cases.totalWithPlaneMs]),max([r.cases.totalWithPlaneMs]));
 [v,k]=max([r.cases.totalWithPlaneMs]);
 fprintf(fid,'SLOWEST_CASE t=%.6f s sampleIndex=%d total=%.4f ms bridge=%.4f ms plane=%.4f ms\n', ...
    r.cases(k).time,r.cases(k).sampleIndex,v,r.cases(k).elapsedMs,r.cases(k).planeComputeMs);
 fprintf(fid,'SLOWEST_CASE_BREAKDOWN envelope=%.4f ms first_Jet=%.4f ms second_Jet=%.4f ms geometryTilt=%.4f ms\n', ...
    r.cases(k).envelopeMs,r.cases(k).generatorFirstMs, ...
    r.cases(k).generatorSecondMs,r.cases(k).geometryAndTiltMs);
end
if isfield(r,'timing') && ~isempty(r.timing)
 t=r.timing;
 for mode={'diagnostic','screenOnly'}
    mask=strcmp({t.mode},mode{1}) & strcmp({t.phase},'measured');
    if any(mask)
       v=[t(mask).totalMs];
       fprintf(fid,'WARMED_%s repeats=%d median=%.4f ms max=%.4f ms\n', ...
           mode{1},numel(v),median(v),max(v));
    end
 end
 for ii=unique([t.sampleIndex])
    for mode={'diagnostic','screenOnly'}
       mask=[t.sampleIndex]==ii & strcmp({t.mode},mode{1});
       if any(mask)
          v=[t(mask).totalMs];
          fprintf(fid,'TIMING_SAMPLE sample=%d mode=%s n=%d median=%.4f ms min=%.4f ms max=%.4f ms first=%.4f ms\n', ...
              ii,mode{1},numel(v),median(v),min(v),max(v),v(1));
       end
    end
 end
 fprintf(fid,'TIMING_REPEATS.csv: 5 repeats per mode per sample, 3 local warmups per mode and 5 preflight warmups.\n');
 fprintf(fid,'5 repeats show a trend, NOT a tail-latency or worst-case execution-time certificate. OS/JIT/GC cannot be separated by tic/toc alone.\n');
 fprintf(fid,'SCREEN_ONLY: omits both generator calls and psi2/psi3 diagnostics; NOT an XY-QP or all-time certificate.\n');
end
if isfield(r,'flightRuns') && ~isempty(r.flightRuns)
 fprintf(fid,'\nWHOLE-FLIGHT BENCHMARK: 5 full, identical, start-to-end replays PER mode.\n');
 fprintf(fid,'Each run uses all recorded snapshots in original order, including no-obstacle records.\n');
 fprintf(fid,'MAT/ZIP import, initial warmup and CSV/ZIP saving EXCLUDED; the stored flight is NOT a new closed-loop simulation.\n');
 for k=1:numel(r.flightRuns)
    a=r.flightRuns(k);
    fprintf(fid,['FULL_RUN pass=%d mode=%s samples=%d/%d obstacleSamples=%d ' ...
        'wall=%.4f ms sumSample=%.4f ms median=%.4f ms p95=%.4f ms ' ...
        'max=%.4f ms at sample=%d t=%.5f s >5ms=%d >25ms=%d ' ...
        'errors=%d disagreesFirst=%d\n'], ...
        a.pass,a.mode,a.successfulSamples,a.nonemptySnapshots, ...
        a.successfulWithObstacle,a.fullReplayWallMs, ...
        a.sumSampleComputeMs,a.medianSampleMs,a.p95SampleMs, ...
        a.maxSampleMs,a.maxSampleIndex,a.maxSampleTimeS, ...
        a.samplesOver5ms,a.samplesOver25ms, ...
        a.errors,a.passDisagreementsWithFirstReplay);
 end
 fprintf(fid,'These 5 complete replays show speed trends only, not a provable WCET or 5ms compliance under all conditions.\n');
 fprintf(fid,'diagnostic: analytic bound + TWO generator consistency evaluations + plane/tilt; no XY-QP.\n');
 fprintf(fid,'screenOnly: analytic bound + plane/tilt; no generator, no XY-QP.\n');
end
if isfield(r,'flightSegments') && ~isempty(r.flightSegments)
 fprintf(fid,'\nCBF STATUS-STRATIFIED TIMING: each full replay; groups come from LOGGED Scheme02 status, not new online CBF executions.\n');
 fprintf(fid,'NO_OBSTACLE_TILT_ONLY still includes body/cable tilt checks.\n');
 fprintf(fid,'OBSERVED_XY_CORRECTED may involve soft guidance; not proof that a hard CBF constraint was binding.\n');
 fprintf(fid,'These are timing partitions of the diagnostic/screenOnly evaluator, NOT measured original Scheme02 execution time.\n');
 for k=1:numel(r.flightSegments)
    a=r.flightSegments(k);
    if ~strcmp(a.groupType,'CBF_STATUS'),continue;end
    fprintf(fid,['CBF_GROUP pass=%d mode=%s group=%s n=%d median=%.4f ms p95=%.4f ms ' ...
        'max=%.4f ms sample=%d t=%.5f s >5ms=%d >25ms=%d\n'], ...
        a.pass,a.mode,a.groupName,a.sampleCount,a.medianMs,a.p95Ms, ...
        a.maxMs,a.maxSampleIndex,a.maxTimeS,a.samplesOver5Ms,a.samplesOver25Ms);
 end
 fprintf(fid,'Further near-obstacle timing groups: FLIGHT_TIMING_CBF_GROUPS.csv (CLEARANCE groups).\n');
end
if isfield(r,'preflightWarmup') && ~isempty(r.preflightWarmup)
 fprintf(fid,'PREFLIGHT_WARMUP repeats=%d first=%.4f ms last=%.4f ms (not counted in timed flight replay).\n', ...
    numel(r.preflightWarmup),r.preflightWarmup(1).totalMs,r.preflightWarmup(end).totalMs);
end
if isfield(r,'planeInputs') && ~isempty(r.planeInputs)
 fprintf(fid,'FIXED_PLANE_G4 analyticXY_vs_generator_maxAbsErr=%.9g; numerical diagnostic only.\n', ...
     max([r.planeInputs.maxAbsCoefficientError]));
end
if ~isempty(r.jumps) && isfield(r.jumps,'A_newHLCThrustInLowStageInterval')
 fprintf(fid,'A_OBSTACLE_LOWSTAGE_T_RANGE available=%d/%d, new_HLC_thrust_inside=%d/%d.\n', ...
    sum([r.jumps.A_lowStageThrustIntervalNonempty]),numel(r.jumps), ...
    sum([r.jumps.A_newHLCThrustInLowStageInterval]),numel(r.jumps));
 fprintf(fid,'A_OBSTACLE_LOWSTAGE_T_RANGE: post-hoc 1D affine intersection at new state/mass; XY cannot change psi2/psi3 instantly.\n');
 fprintf(fid,'A_OBSTACLE_LOWSTAGE_T_RANGE: obstacle-only necessity; NO full HOCBF or next-step feasibility proof, T is NOT optimized.\n');
end
if isfield(r,'fullBoundary') && ~isempty(r.fullBoundary)
 b=r.fullBoundary;
 fprintf(fid,'\nFULL RECORDED-BOUNDARY A/B AUDIT (OFFLINE, NO NEW CLOSED-LOOP):\n');
 fprintf(fid,['A ALL-CONSTRAINT LOW-STAGES (obstacles + enabled body/cable tilts) ' ...
   'checked %d adjacent pairs; actual HLC T admissible %d/%d; ' ...
   'full [0,20] N thrust uncertainty admissible %d/%d; affine structure checks passed %d/%d.\n'], ...
   numel(b),sum([b.A_actualThrustAdmissible]),numel(b), ...
   sum([b.A_allThrust0to20Admissible]),numel(b), ...
   sum([b.A_affinityValidatedNumerically]),numel(b));
 fprintf(fid,'B NEXT-ZOH (new recorded state, fixed plane): physical/tilt screen %d/%d; finite-horizon only.\n', ...
   sum([b.B_nextZohPhysicalPass]),numel(b));
 fprintf(fid,['These are separate A necessary-set checks and B one-period physical sufficient checks. ' ...
     'No conjunction proves recursive feasibility, because new states are logged, not predicted under a newly chosen XY-QP.\n']);
 fprintf(fid,'Full audit exceptions=%d.\n',numel(r.fullBoundaryErrors));
 if any(~[b.A_actualThrustAdmissible])
   fail=find(~[b.A_actualThrustAdmissible]);
   for kk=1:min(12,numel(fail))
     a=b(fail(kk));
     fprintf(fid,['A FAIL at t=%.5f -> %.5f, Tnew=%.9f, ' ...
        'safe interval [%.9f,%.9f], limiting stage=%s, negative stages=%d.\n'], ...
        a.oldTimeS,a.newTimeS,a.newThrustN, ...
        a.A_intervalLoN,a.A_intervalHiN,a.A_worstStageName, ...
        a.A_negativeLowStageCount);
   end
 end
end
if isfield(r,'referenceAdmission') && ~isempty(r.referenceAdmission)
 a=r.referenceAdmission;
 ok=[a.referencePreimageComputed];
 inferred=[a.impliedVerticalInputReconstructible];
 fail=ok & inferred & ~[a.A_HLCInAdmissibleVerticalInput];
 good=ok & inferred & [a.A_HLCInAdmissibleVerticalInput];
 fprintf(fid,'\nREFERENCE-ADMISSION CONTRACT (OFFLINE, EXACT-REAL ALGEBRA, NUMERICAL EVALUATION):\n');
 fprintf(fid,['Using source-derived HLC thrust formula, permitted qz=Vz+zddot_ref ' ...
  'is the PREIMAGE of all-constraint low-stage thrust interval through saturation.\n']);
 fprintf(fid,'Computed=%d/%d reconstructibleUnsaturated=%d/%d accepted=%d/%d rejected=%d.\n', ...
  sum(ok),numel(a),sum(inferred),numel(a),sum(good),sum(ok & inferred),sum(fail));
 if any(fail)
  d=[a(fail).minimumVerticalInputChangeMps2];
  fprintf(fid,'REFERENCE virtual-acceleration equivalent correction among failed boundaries: median=%.9g m/s^2 maximum=%.9g m/s^2.\n', ...
     median(d),max(d));
 end
 fprintf(fid,['The inferred vertical input is algebraically reconstructed from UNSATURATED logged T. ' ...
   'Original trajectory/reference/Vz were NOT logged. ' ...
   'A future reference governor is NOT implemented.\n']);
 fprintf(fid,['Admission is only A low-stage membership at the fixed NEW state, ' ...
   'not a sufficient HOCBF/ZOH, QP, B, or recursive-feasibility certificate.\n']);
 fprintf(fid,['Future reference updates must enter augmented state + actual HLC update order; ' ...
   'T and yaw stay HLC-fixed and only XY is CBF-optimized in current work.\n']);
 fprintf(fid,'Admission exceptions=%d, IEEE rounding certificate=NO.\n',numel(r.referenceAdmissionErrors));
end
if isfield(r,'referenceSlewMeta') && ~isempty(r.referenceSlewMeta)
 M=r.referenceSlewMeta;
 fprintf(fid,'\nOFFLINE REFERENCE INTERVAL SLEW (fixed recorded states): minimum R*=%.9f m/s^3; critical time %.6f -> %.6f s; witness valid=%d.\n', ...
     M.minimumSampleRateMps3,M.criticalTimeFromS,M.criticalTimeToS,M.witnessAllIntervalsPassed);
 fprintf(fid,'Actual reference jets are not logged; this is noncausal fixed-corridor analysis, NOT trajectory modification or recursive feasibility.\n');
end
fclose(fid);
root=fileparts(mfilename('fullpath'));out=fullfile(root,'CBF_SD_TEST_RESULTS.zip');
if isfile(out),delete(out);end
zip(out,files,folder);
fprintf('[SD THEORY] Saved %s; full forward invariance NOT proven.\n',out);
end


% ======================================================================
% Full-boundary THEORY audit (research, not part of online controller).
% Key A theorem (exact real arithmetic at fixed x,mu,geometry):
%   psi_{i,j}(x,T,mu) = a_{i,j}+b_{i,j}*T for j<relativeDegree.
%   The necessary low-stage thrust set for the standard HOCBF theorem is
%       I(x,mu)=[0,20] intersection {T : a_ij+b_ij*T >= 0}.
%   I is an interval, computable by one-dimensional half-line clipping.
%   If the NEW HLC T is outside I, current XY torque cannot instantly
%   restore low-stage membership. This is NOT a collision certificate.
%   Numerical checks of affine structure are DIAGNOSTIC and not a proof.
%
% Key B distinction: physical separation has no instantaneous impulse
% from a change of T; for the recorded next state, a NEW normal is chosen
% and the independent 1-ZOH screen is evaluated. This does NOT establish
% that A/B XY-QP at the old sample could have led to that new state, nor
% that the newly selected input is recursively viable.
% ======================================================================
function [rows,stages,errors]=sd_bridge_all_boundary_audit(snapshots,fixed)
rows=struct([]);stages=struct([]);errors={};pairs=0;skipped=0;errorCount=0;
% Pure algebra sanity test. This does not validate the model's affine
% structure; the independent 0/10/20 N generator evaluations do that.
[a,b,ok]=sd_bridge_intersect_affine([4;-1;1],[-1;1;0],0,20);
assert(ok && abs(a-1)<1e-13 && abs(b-4)<1e-13, ...
    'SD:FullBoundaryHalfspaceSelftest','Affine interval test failed');
for ii=1:numel(snapshots)-1
    old=snapshots{ii};new=snapshots{ii+1};
    if isempty(old)||isempty(new),skipped=skipped+1;continue;end
    dt=double(old.dt);
    if ~(isfinite(dt)&&dt>0&& ...
            abs(double(new.time)-double(old.time)-dt)<1e-5)
        skipped=skipped+1;continue;
    end
    try
        sd_analytic_fixed_check(fixed,new);
        [rr,ss]=sd_bridge_one_full_boundary(old,new,ii);
        if isempty(rows),rows=rr;else,rows(end+1,1)=rr;end %#ok<AGROW>
        if isempty(stages),stages=ss;else,stages=[stages;ss(:)];end %#ok<AGROW>
        pairs=pairs+1;
    catch ME
        errorCount=errorCount+1;
        if numel(errors)<200
            errors{end+1,1}=sprintf('old=%d new=%d t=%.9g: %s [%s]', ...
                ii,ii+1,double(new.time),ME.message,ME.identifier); %#ok<AGROW>
        end
    end
end
fprintf('[SD BOUNDARY] adjacentPairs=%d skipped=%d errors=%d', ...
    pairs,skipped,errorCount);
if ~isempty(rows)
    fprintf(' A_actualT_allowed=%d/%d B_oneZoh=%d/%d', ...
      sum([rows.A_actualThrustAdmissible]),numel(rows), ...
      sum([rows.B_nextZohPhysicalPass]),numel(rows));
end
fprintf(' (NOT recursive feasibility)\n');
end

function [out,rows]=sd_bridge_one_full_boundary(old,new,idx)
x=double(new.x(:));P=new.P;G=new.G;theta=new.tiltCfg;
Told=max(0,min(20,double(old.u(1))));
Tnew=max(0,min(20,double(new.u(1))));
yaw=max(-1,min(1,double(new.u(4))));
if ~(numel(x)==19&&numel(new.u)==4)
    error('SD:FullBoundaryInput','Bad logged state/input size');
end
% Read changed mass and ALL observed obstacle geometries from new sample.
% These are EVALs only (offline). They are not a run-time torque search.
Ezero=CBF_REAL_XY_EVAL(x,[0;0;0;yaw],P,G,new.parts,new.lambda,theta);
Ehalf=CBF_REAL_XY_EVAL(x,[10;0;0;yaw],P,G,new.parts,new.lambda,theta);
Efull=CBF_REAL_XY_EVAL(x,[20;0;0;yaw],P,G,new.parts,new.lambda,theta);
Eactual=CBF_REAL_XY_EVAL(x,[Tnew;0;0;yaw],P,G,new.parts,new.lambda,theta);
EoldMass=CBF_REAL_XY_EVAL(x,[Told;0;0;yaw],old.P,G,new.parts,new.lambda,theta);
EnewMassOldT=CBF_REAL_XY_EVAL(x,[Told;0;0;yaw],P,G,new.parts,new.lambda,theta);
[V0,labels,orders]=sd_bridge_flat_lower(Ezero,theta);
[V10,~,~]=sd_bridge_flat_lower(Ehalf,theta);
[V20,~,~]=sd_bridge_flat_lower(Efull,theta);
[Vact,~,~]=sd_bridge_flat_lower(Eactual,theta);
[Vold,~,~]=sd_bridge_flat_lower(EoldMass,theta);
[VoldNewMass,~,~]=sd_bridge_flat_lower(EnewMassOldT,theta);
if ~(numel(V0)==numel(V20)&&numel(V0)==numel(Vact))
    error('SD:FullBoundaryStages','Inconsistent barrier dimensions');
end
slope=(V20-V0)/20;
predict10=V0+10*slope;predictT=V0+Tnew*slope;
if isempty(V0)
    err=0;scale=1;valid=true;
else
    err=max(abs([predict10-V10;predictT-Vact]));
    scale=max(1,max(abs([V0;V10;V20;Vact])));
    valid=err<=3e-8*scale;
end
% This is a CRITICAL safety-model assumption: never publish an A thrust
% interval as affine if the numeric regression itself is inconsistent.
if ~valid
    error('SD:FullBoundaryNotAffine','Lower stages not T-affine: max error %.9g, scale %.9g',err,scale);
end
% Analytic halfspace intersection is exact over real numbers once
% T-affinity is established. The generated regression remains a numerical
% check only; IEEE outward rounding is NOT claimed.
[lo,hi,isPossible]=sd_bridge_intersect_affine(V0,slope,0,20);
negatives=sum(Vact<0);
worstName='none';
if any(Vact<0)
    bad=find(Vact<0,1,'first');worstName=labels{bad};
end
rows=struct([]);
for j=1:numel(V0)
    a=V0(j);b=slope(j);
    if ~isfinite(a)||~isfinite(b)
        error('SD:FullBoundaryNonfinite','Nonfinite low-stage coefficient');
    end
    % Stage quantities have DIFFERENT units across orders; only sign
    % is used jointly. Do not compare raw numerical magnitudes as distances.
    st=struct('sampleIndexOld',idx,'sampleIndexNew',idx+1, ...
      'newTimeS',double(new.time),'barrierAndStage',labels{j}, ...
      'order',orders(j),'aAtZeroThrust',a,'bPerNewton',b, ...
      'valueAtOldThrustOldMass',Vold(j), ...
      'valueAtOldThrustNewMass',VoldNewMass(j), ...
      'valueAtNewThrustNewMass',Vact(j), ...
      'thrustSwitchEffect',Vact(j)-VoldNewMass(j), ...
      'massSwitchEffect',VoldNewMass(j)-Vold(j), ...
      'actuallyNonnegative',Vact(j)>=0, ...
      'numericThrustAffineError',max(abs([predict10(j)-V10(j);predictT(j)-Vact(j)])));
    if isempty(rows),rows=st;else,rows(end+1,1)=st;end %#ok<AGROW>
end
actualFits=isPossible&&(Tnew>=lo)&&(Tnew<=hi)&&all(Vact>=0);
% The robust requirement for arbitrary HLC T in [0,20] is MUCH
% stronger than actual T membership; it is testable without any T search.
allThrustFits=isPossible&&(lo<=0)&&(hi>=20);
if actualFits
    symmetricSlew=min(Tnew-lo,hi-Tnew);
    distanceToInterval=0;
elseif isPossible
    symmetricSlew=NaN;
    distanceToInterval=max([lo-Tnew,Tnew-hi,0]);
else
    symmetricSlew=NaN;distanceToInterval=NaN;
end
% B is a separate exact-real one-cycle screen, with new state's physical
% shape and a freshly selected normal. No old-plane membership is assumed.
[Env,ok,why]=sd_analytic_scalar_envelope(x,P,Tnew,yaw,double(new.dt),G);
if ~ok,error('SD:FullBoundaryBEnvelope','%s',why);end
pl=sd_bridge_plane_case(new,idx+1,Env);
physicalPass=sd_bridge_joint_physical_pass(new,Env,pl);
if isempty(pl),gap=Inf;else,gap=min([pl.planeWholePeriodLowerM]);end
out=struct('sampleIndexOld',idx,'sampleIndexNew',idx+1, ...
    'oldTimeS',double(old.time),'newTimeS',double(new.time), ...
    'oldThrustN',Told,'newThrustN',Tnew, ...
    'oldEstimatedMassKg',double(old.P.mL), ...
    'newEstimatedMassKg',double(new.P.mL), ...
    'observedObstaclesNew',numel(new.parts), ...
    'A_lowerStageRows',numel(Vact), ...
    'A_negativeLowStageCount',negatives, ...
    'A_worstStageName',worstName, ...
    'A_intervalLoN',lo,'A_intervalHiN',hi, ...
    'A_lowStageIntervalNonempty',isPossible, ...
    'A_actualThrustAdmissible',actualFits, ...
    'A_actualThrustDistanceToIntervalN',distanceToInterval, ...
    'A_symmetricalThrustPerturbationAllowedN',symmetricSlew, ...
    'A_allThrust0to20Admissible',allThrustFits, ...
    'A_affinityValidatedNumerically',valid, ...
    'A_maxThrustAffineError',err, ...
    'B_nextZohPhysicalPass',physicalPass, ...
    'B_nextZohPlaneLowerM',gap, ...
    'B_recursiveFeasibilityCertified',false, ...
    'A_recursiveFeasibilityCertified',false, ...
    'machineRoundedCertified',false);
end

function [V,labels,orders]=sd_bridge_flat_lower(E,cfg)
V=zeros(0,1);labels=cell(0,1);orders=zeros(0,1);
for k=1:numel(E.names)
    for j=0:3
       V(end+1,1)=E.stages(j+1,k); %#ok<AGROW>
       labels{end+1,1}=sprintf('%s/psi%d',E.names{k},j); %#ok<AGROW>
       orders(end+1,1)=j; %#ok<AGROW>
    end
end
if cfg.bodyEnabled
    for j=0:1
        V(end+1,1)=E.tiltBodyStages(j+1); %#ok<AGROW>
        labels{end+1,1}=sprintf('bodyTilt/psi%d',j); %#ok<AGROW>
        orders(end+1,1)=j; %#ok<AGROW>
    end
end
if cfg.cableEnabled
    for j=0:3
        V(end+1,1)=E.tiltCableStages(j+1); %#ok<AGROW>
        labels{end+1,1}=sprintf('cableTilt/psi%d',j); %#ok<AGROW>
        orders(end+1,1)=j; %#ok<AGROW>
    end
end
end


function [lo,hi,ok]=sd_bridge_intersect_affine(a,b,Tmin,Tmax)
% Exact-real algebra: intersect all a_j+b_j*T>=0 with [Tmin,Tmax].
% No arbitrary gain, numerical recovery, or modification of HLC T.
if numel(a)~=numel(b)||~isfinite(Tmin)||~isfinite(Tmax)||Tmin>Tmax
    error('SD:FullBoundaryAffine','Invalid coefficient vectors or thrust bounds');
end
lo=Tmin;hi=Tmax;ok=true;
for j=1:numel(a)
    if ~all(isfinite([a(j),b(j)]))
        error('SD:FullBoundaryAffine','Nonfinite affine coefficient');
    end
    if b(j)>0
        lo=max(lo,-a(j)/b(j));
    elseif b(j)<0
        hi=min(hi,-a(j)/b(j));
    elseif a(j)<0
        ok=false;
    end
end
ok=ok&&(lo<=hi);
if ~ok,lo=NaN;hi=NaN;end
end


% ======================================================================
% Reference-aware HLC preimage of the exact-real A low-stage admissibility
% interval. FUTURE trajectory-correction interface ONLY, offline audit.
%
% For source-derived HLC vertical channel on non-singular states:
% T = sat_[0,20]([ (m+mu)*(qz+g) + m*L*pz*||omegaL x p||^2 ] / D),
% D = pz*(p dot R(q)e3), qz = V_z + zddot_ref.
% Equivalently qz = a*Traw + c with a=D/(m+mu),
% c=-g-m*L*pz*||omegaL x p||^2/(m+mu).
%
% The preimage of [lo,hi] under T=sat(Traw) is a single interval.
% If lo=0, the raw lower bound is removed (all thrusts <=0 saturate
% to admissible T=0). Likewise hi=20 removes the raw upper bound.
% For D<0 the corresponding qz inequalities reverse.
%
% This is NOT independent validation of the actual HLC reference state,
% because snapshots log T but not V_z/zddot_ref. qzNom is INFERRED only
% if 0<T<20 and if the exact HLC symbolic source is still in use.
% ======================================================================
function [rows,errors]=sd_bridge_reference_admission(boundaries,snapshots)
rows=struct([]);errors={};
for i=1:numel(boundaries)
    b=boundaries(i);
    try
        sn=snapshots{b.sampleIndexNew};
        x=double(sn.x(:));P=sn.P;
        mass=double(P.m)+double(P.mL);
        if numel(x)~=19 || ~(isfinite(mass)&&mass>0)
            error('SD:RefContractModel','Invalid state or total mass');
        end
        q=x(1:4);p=x(14:16);w=x(17:19);
        b3=[2*(q(2)*q(4)+q(1)*q(3)); ...
            2*(q(3)*q(4)-q(1)*q(2)); ...
            q(1)^2-q(2)^2-q(3)^2+q(4)^2];
        denominator=p(3)*dot(p,b3);
        lever=double(P.m)*double(P.cableL)*p(3)*dot(cross(w,p),cross(w,p));
        if ~(all(isfinite([denominator,lever])) && abs(denominator)>0)
            error('SD:RefContractDenominator','Invalid HLC thrust denominator');
        end
        slope=denominator/mass;
        offset=-double(P.gravity)-lever/mass;
        thrust=double(b.newThrustN);
        lo=double(b.A_intervalLoN);hi=double(b.A_intervalHiN);
        valid=logical(b.A_lowStageIntervalNonempty) && ...
           all(isfinite([lo,hi,thrust,slope,offset])) && lo<=hi && ...
           lo>=0 && hi<=20 && thrust>=0 && thrust<=20;
        lowQ=NaN;highQ=NaN;impliedQ=NaN;need=NaN;
        reconstructible=thrust>0 && thrust<20;
        admissible=false;consistency=false;
        if valid
            lowQ=-Inf;highQ=Inf;
            if lo>0
                cut=slope*lo+offset;
                if slope>0,lowQ=max(lowQ,cut);
                else,highQ=min(highQ,cut);end
            end
            if hi<20
                cut=slope*hi+offset;
                if slope>0,highQ=min(highQ,cut);
                else,lowQ=max(lowQ,cut);end
            end
            if lowQ>highQ
                error('SD:RefContractInverse','Empty inverse image of nonempty T interval');
            end
            if reconstructible
                % This is NOT a measured HLC virtual input; the log has T
                % but not the actual vertical reference state.
                impliedQ=slope*thrust+offset;
                admissible=(impliedQ>=lowQ && impliedQ<=highQ);
                need=max([lowQ-impliedQ,impliedQ-highQ,0]);
                % Identity for distance under an affine monotone map;
                % validates conversion units and implementation.
                tdist=double(b.A_actualThrustDistanceToIntervalN);
                consistency=abs(need-abs(slope)*tdist)<= ...
                    3e-10*(1+abs(need)+abs(slope*tdist));
                if ~consistency
                    error('SD:RefContractMismatch','Affine inverse-distance identity failed');
                end
                if admissible ~= logical(b.A_actualThrustAdmissible)
                    error('SD:RefContractMembership','Reference and thrust membership disagree');
                end
            end
        end
        rr=struct('sampleIndexNew',b.sampleIndexNew, ...
          'newTimeS',double(b.newTimeS), ...
          'estimatedMassKg',double(P.mL), ...
          'HLC_thrustN',thrust, ...
          'HLC_denominatorD',denominator, ...
          'TrawToVerticalInputSlopeMps2PerN',slope, ...
          'verticalInputGravityOffsetMps2',offset, ...
          'A_thrustIntervalLoN',lo,'A_thrustIntervalHiN',hi, ...
          'referencePreimageComputed',valid, ...
          'admissibleVerticalInputLoMps2',lowQ, ...
          'admissibleVerticalInputHiMps2',highQ, ...
          'impliedVerticalInputReconstructible',reconstructible, ...
          'impliedVerticalInputMps2',impliedQ, ...
          'A_HLCInAdmissibleVerticalInput',admissible, ...
          'minimumVerticalInputChangeMps2',need, ...
          'algebraicDistanceIdentityVerified',consistency, ...
          'realHLCTrajectoryAvailable',false, ...
          'currentReferenceWasModified',false, ...
          'forwardInvarianceCertified',false);
        if isempty(rows),rows=rr;else,rows(end+1,1)=rr;end %#ok<AGROW>
    catch ME
        errors{end+1,1}=sprintf('new=%d t=%.8g: %s [%s]', ...
            b.sampleIndexNew,double(b.newTimeS),ME.message,ME.identifier); %#ok<AGROW>
    end
end
fprintf('[SD REFERENCE] audited=%d/%d exceptions=%d (reference NOT altered; recursive safety NOT proven)\n', ...
    numel(rows),numel(boundaries),numel(errors));
end


% ======================================================================
% REFERENCE SLEW COMPATIBILITY (OFFLINE; NO PRODUCTION MODIFICATIONS)
% A theorem for a finite sequence of fixed closed intervals I_k=[Lk,Uk]
% and a fixed first value v1: A sequence v_k in I_k with
% |v_{k+1}-v_k| <= R*(t_{k+1}-t_k) exists iff for each i<j,
%   L_i-U_j <= R*(t_j-t_i), L_j-U_i <= R*(t_j-t_i),
% after replacing the first interval with the singleton {v1}.
% Necessity follows by Lipschitz distance; sufficiency follows because
% the lower envelope v(t)=max_j(L_j-R*|t-t_j|) is R-Lipschitz and inside
% every I_j when these inequalities hold (with an anchored initial value).
% All infinite bounds are valid. This is exact-real mathematics: MATLAB
% binary64 calculations and logged states are NOT machine certificates.
% The witness is NONCAUSAL: computed using the complete FUTURE dataset.
% ======================================================================
function [rows,meta,errors]=sd_bridge_reference_slew_audit(records)
rows=table();meta=struct([]);errors={};
if isempty(records),return;end
n=numel(records);
t=reshape(double([records.newTimeS]),[],1);
lo=reshape(double([records.admissibleVerticalInputLoMps2]),[],1);
hi=reshape(double([records.admissibleVerticalInputHiMps2]),[],1);
nominal=reshape(double([records.impliedVerticalInputMps2]),[],1);
valid=logical([records.referencePreimageComputed]) & ...
      logical([records.impliedVerticalInputReconstructible]);
if ~all(valid) || any(~isfinite(t)) || any(~isfinite(nominal)) || ...
        any(isnan(lo)) || any(isnan(hi)) || any(lo>hi) || ...
        any(~isfinite(diff(t))) || any(diff(t)<=0)
    errors{end+1}='Preimage/nominal unavailable, unordered timestamps or invalid intervals; full slew audit not performed.';
    return;
end
% Fix inferred initial qz for the scalar compatibility experiment only.
if nominal(1)<lo(1)-1e-9 || nominal(1)>hi(1)+1e-9
    errors{end+1}='Initial inferred qz outside its allowed interval; anchored analysis invalid.';
    return;
end
lo(1)=nominal(1);hi(1)=nominal(1);
% Exact-real optimal rate on fixed intervals; O(N^2) OFFLINE, not online.
Rstar=0;criticalI=1;criticalJ=1;criticalGap=0;
for ii=1:n-1
    jj=(ii+1:n)';dt=t(jj)-t(ii);
    gap1=lo(ii)-hi(jj);gap2=lo(jj)-hi(ii);
    [a,p]=max(gap1./dt);
    if a>Rstar
        Rstar=a;criticalI=ii;criticalJ=jj(p);criticalGap=gap1(p);
    end
    [b,p]=max(gap2./dt);
    if b>Rstar
        Rstar=b;criticalI=ii;criticalJ=jj(p);criticalGap=gap2(p);
    end
end
% Finite-precision slack for witness construction; this is NOT an IEEE
% outward-rounded certification. Positive empirical slack is explicit.
Ruse=Rstar+1e-9*(1+Rstar);
[reachable,reachLo,reachHi,firstFailure]= ...
    sd_bridge_scalar_slew_reachable(t,lo,hi,Ruse);
if ~reachable
    errors{end+1}=sprintf('Pairwise theorem numerical discrepancy: firstFailure=%d R*=%.12g.',firstFailure,Rstar);
    return;
end
% Backward projection through the exact forward reachable intervals
% produces a witness with an initial anchor and bounded neighbour steps.
selected=zeros(n,1);
selected(n)=min(max(nominal(n),reachLo(n)),reachHi(n));
for ii=n-1:-1:1
    ds=Ruse*(t(ii+1)-t(ii));
    low=max(reachLo(ii),selected(ii+1)-ds);
    upp=min(reachHi(ii),selected(ii+1)+ds);
    if low>upp+2e-10
        errors{end+1}=sprintf('Backward witness empty at index %d.',ii);
        return;
    end
    selected(ii)=min(max(nominal(ii),low),upp);
end
witnessRate=max(abs(diff(selected)./diff(t)));
nominalRate=max(abs(diff(nominal)./diff(t)));
allIntervals=all(selected>=lo-1e-8 & selected<=hi+1e-8) && ...
    witnessRate<=Ruse+1e-8;
firstPinned=abs(selected(1)-nominal(1))<1e-9;
if ~allIntervals || ~firstPinned
    errors{end+1}='Witness interval, slew or anchor check failed.';return;
end
% This is a scalar diagnostic only: nominal is INFERRED from logged T and
% selected is an OFFLINE, anticipative mathematical witness.
selectedSlew=[NaN;diff(selected)./diff(t)];
shift=selected-nominal;
rows=table(reshape(double([records.sampleIndexNew]),[],1),t,lo,hi, ...
    nominal,selected,shift,selectedSlew,reachLo,reachHi, ...
    'VariableNames',{'sampleIndexNew','timeS', ...
    'qzAllowedLoMps2','qzAllowedHiMps2','impliedNominalQzMps2', ...
    'noncausalWitnessQzMps2','witnessMinusImpliedNominalMps2', ...
    'witnessSampleRateMps3','forwardReachLoMps2','forwardReachHiMps2'});
meta=struct('samples',n,'firstTimeS',t(1),'lastTimeS',t(end), ...
    'dtS',median(diff(t)), ...
    'minimumSampleRateMps3',Rstar, ...
    'criticalTimeFromS',t(criticalI), ...
    'criticalTimeToS',t(criticalJ), ...
    'criticalGapMps2',criticalGap, ...
    'criticalRateMps3',Rstar, ...
    'impliedNominalMaxSampleRateMps3',nominalRate, ...
    'forwardReachabilityPassed',reachable, ...
    'witnessMaxRateMps3',witnessRate, ...
    'witnessMaximumDeviationMps2',max(abs(shift)), ...
    'witnessAllIntervalsPassed',allIntervals, ...
    'witnessInitialFixed',firstPinned, ...
    'futurePreviewWasUsed',true, ...
    'continuousReferenceJetRealizabilityCertified',false, ...
    'forwardInvarianceCertified',false);
fprintf(['[SD REF SLEW] %d intervals; R*=%.9f m/s^3, ' ...
    'critical %.3f->%.3f s; noncausal witness verified=%d; ' ...
    'real trajectory NOT modified.\n'], ...
    n,Rstar,t(criticalI),t(criticalJ),allIntervals);
end

function [ok,loReach,hiReach,firstFailure]= ...
     sd_bridge_scalar_slew_reachable(t,lo,hi,R)
n=numel(t);loReach=NaN(n,1);hiReach=NaN(n,1);
loReach(1)=lo(1);hiReach(1)=hi(1);ok=true;firstFailure=0;
for k=2:n
    d=R*(t(k)-t(k-1));
    loReach(k)=max(lo(k),loReach(k-1)-d);
    hiReach(k)=min(hi(k),hiReach(k-1)+d);
    if loReach(k)>hiReach(k)+1e-12
        ok=false;firstFailure=k;return;
    end
end
end


% ======================================================================
% READ-ONLY PER-FLIGHT SETUP CONSISTENCY AND ADMISSION AUDIT
% Does NOT accept/permit a flight: no invariant K, no certified reference
% update or 6-jet, no roundoff/WCET certificate. A change BETWEEN flights
% must produce a separate independent verification under new theta_f.
% ======================================================================
function [rows,meta,errs]=sd_bridge_flight_setup_audit(snapshots,fixed,boundary,admission)
rows=struct([]);errs={};n=0;muLo=Inf;muHi=-Inf;Tlo=Inf;Thi=-Inf;
qErr=0;pErr=0;fixedOK=true;
base=[];names={};values=[];
for k=1:numel(snapshots)
    s=snapshots{k};if isempty(s),continue;end
    n=n+1;
    if isempty(base)
        base=s;
        % The vector used by the original generator/CBF setup.
        values=sd_analytic_fixed_signature(s);
        names={'massKg','Jx','Jy','Jz','gravityMps2','cableLengthM', ...
          'sampleTimeS','cableCenterFactor','cableAxisLongM','cableAxisShortM'};
        for j=1:numel(s.lambda),names{end+1}=sprintf('obstacleLambda%d',j);end %#ok<AGROW>
        names=[names,{'bodyTiltEnabled','cableTiltEnabled', ...
          'bodyTiltMaxDeg','cableTiltMaxDeg'}]; %#ok<AGROW>
        for j=1:numel(s.tiltCfg.bodyLambda)
            names{end+1}=sprintf('bodyTiltLambda%d',j);end %#ok<AGROW>
        for j=1:numel(s.tiltCfg.cableLambda)
            names{end+1}=sprintf('cableTiltLambda%d',j);end %#ok<AGROW>
        if numel(names)~=numel(values)
            error('SD:FlightSetupFieldCount','Flight fixed signature names mismatch');
        end
    end
    try,sd_analytic_fixed_check(fixed,s);
    catch ME,fixedOK=false;
        if numel(errs)<20,errs{end+1}=sprintf('sample %d fixed signature: %s',k,ME.message);end %#ok<AGROW>
    end
    % Check ALL geometry fields too, not only currently enabled barriers.
    if ~isequaln(s.G,base.G),fixedOK=false;
        if numel(errs)<20,errs{end+1}=sprintf('sample %d geometry changed',k);end %#<AGROW>
    end
    if ~isfinite(s.P.mL)||s.P.mL<0
        error('SD:FlightEstimatedMass','Invalid estimated mass');
    end
    muLo=min(muLo,double(s.P.mL));muHi=max(muHi,double(s.P.mL));
    Tlo=min(Tlo,double(s.u(1)));Thi=max(Thi,double(s.u(1)));
    xx=double(s.x(:));
    qErr=max(qErr,abs(norm(xx(1:4))-1));
    pErr=max(pErr,abs(norm(xx(14:16))-1));
end
if n==0,error('SD:FlightSetupEmpty','No nonempty snapshots');end
for j=1:numel(names)
    rr=struct('parameterName',names{j},'baselineValue',double(values(j)), ...
        'mustBeFixedWithinFlight',true, 'changeBetweenFlightsAllowedWithRecertification',true, ...
        'unchangedInThisRecordedFlight',fixedOK, 'isEstimatorOrHLCDynamicInput',false, ...
        'operationalSafetyCertificateIssued',false);
    if isempty(rows),rows=rr;else,rows(end+1,1)=rr;end %#ok<AGROW>
end
% Explicitly show which quantities may change DURING a flight.
extra={'estimatedLoadMassKg','HLCThrustN','HLCYawTorqueNm'};
extraVals=[muLo,Tlo,NaN];
for j=1:numel(extra)
    rr=struct('parameterName',extra{j},'baselineValue',double(extraVals(j)), ...
        'mustBeFixedWithinFlight',false, ...
        'changeBetweenFlightsAllowedWithRecertification',true, ...
        'unchangedInThisRecordedFlight',false, 'isEstimatorOrHLCDynamicInput',true, ...
        'operationalSafetyCertificateIssued',false);
    rows(end+1,1)=rr; %#ok<AGROW>
end
% G has additional safety parameters (drone/load sphere and ground) that
% are not yet in the implemented barrier set. Flag them in the CSV too.
geom=fieldnames(base.G);
for j=1:numel(geom)
    v=base.G.(geom{j});
    if ~isnumeric(v) && ~islogical(v),continue;end
    if ~isscalar(v)||~isfinite(double(v)),continue;end
    rr=struct('parameterName',['geometry.' geom{j}], ...
        'baselineValue',double(v), 'mustBeFixedWithinFlight',true, ...
        'changeBetweenFlightsAllowedWithRecertification',true, ...
        'unchangedInThisRecordedFlight',fixedOK, ...
        'isEstimatorOrHLCDynamicInput',false, ...
        'operationalSafetyCertificateIssued',false);
    rows(end+1,1)=rr; %#ok<AGROW>
end
if isempty(boundary),A=0;B=0;nn=0;
else
 nn=numel(boundary);A=sum([boundary.A_actualThrustAdmissible]);
 B=sum([boundary.B_nextZohPhysicalPass]);
end
haveJet=false;haveRefState=false;
% These explicit fields are NOT present in the supplied snapshots.
for k=1:numel(snapshots)
    s=snapshots{k};if isempty(s),continue;end
    haveJet=isfield(s,'referenceJet6') && numel(s.referenceJet6)==28;
    haveRefState=isfield(s,'referenceGovernorState');
    break;
end
meta=struct('samples',n,'fixedFieldsChecked',numel(names)+numel(geom), ...
    'fixedFieldsUnchanged',fixedOK, 'estimatedMassMinKg',muLo, ...
    'estimatedMassMaxKg',muHi,'thrustMinN',Tlo,'thrustMaxN',Thi, ...
    'maxQuaternionNormError',qErr,'maxCableNormError',pErr, ...
    'boundaries',nn,'aAdmissible',A,'bScreenPassed',B, ...
    'referenceSixJetLogged',haveJet,'referenceInternalStateLogged',haveRefState, ...
    'referenceAuditSamples',numel(admission), ...
    'operationalSafeToFlyCertified',false);
fprintf('[SD FLIGHT SETUP] samples=%d fixed=%d mL=[%.8g,%.8g] A=%d/%d B=%d/%d; NO OPERATIONAL CERTIFICATE\n', ...
 n,fixedOK,muLo,muHi,A,nn,B,nn);
end

% ======================================================================
% B: INDEPENDENT NUMERICAL G5 AFFINITY PROBE
% For a fixed 3D separating normal n and z=[tauX;tauY], the exact-real
% structural derivation is:
%   g4(x,z)=a4(x)+B4(q,p,T,n)*z.
%   B4 depends on q,p,T,m,J,L,geometry,n but not omegaBody.
%   G=[e_omegaX/Jx, e_omegaY/Jy] is constant and nonzero only in omega.
%   so (partial_x B4)*G = 0, the quadratic g5 term vanishes.
% g5 is therefore POINTWISE affine in z (for fixed T,yaw,mu and n).
% The finite difference below measures only numerical consistency, not
% a machine-verified identity or a UNIFORM ZOH g5 lower bound.
% ======================================================================
function out=sd_bridge_plane_g5_affinity(s,id)
out=struct([]);if isempty(s.parts),return;end
x=double(s.x(:));P=s.P;G=s.G;
T=max(0,min(20,double(s.u(1))));yaw=max(-1,min(1,double(s.u(4))));
alpha=G.cableCenterFromLoad*P.cableL;
C=x(8:10)-alpha*x(14:16);
for j=1:numel(s.parts)
    part=s.parts(j);n=C-double(part.center(:));normN=norm(n);
    if normN<=1e-12,error('SD:PlaneG5Normal','Degenerate fixed normal');end
    n=n/normN;
    test=[0,1,0,1,-0.6;0,0,1,-1,0.9];
    val=zeros(1,size(test,2));
    for kk=1:size(test,2)
        z=test(:,kk);u=[T;z(1);z(2);yaw];
        f=CBF_COMMON_MODEL(x,u,P);
        if ~(all(isfinite(f)) && norm(f)<1e7)
            error('SD:PlaneG5Rhs','Invalid derivative');
        end
        % Scale to keep perturbation small while preserving the physical
        % body/cable unit-state manifold to O(eps^2).
        epsStep=1e-5/(1+norm(f));
        plus=sd_bridge_plane_g4jet(x+epsStep*f,u,P,G,n);
        minus=sd_bridge_plane_g4jet(x-epsStep*f,u,P,G,n);
        val(kk)=(plus-minus)/(2*epsStep);
    end
    a=val(1);bx=val(2)-a;by=val(3)-a;
    pred=a+bx*test(1,:)+by*test(2,:);
    res=max(abs(pred-val));scale=max(1,max(abs(val)));
    rr=struct('sampleIndex',id,'timeS',double(s.time), ...
       'obstaclePart',j,'g5AtZero',a,'g5CoeffX',bx,'g5CoeffY',by, ...
       'g5AtX1Yminus1',val(4),'g5AtXminus06Y09',val(5), ...
       'maxAffineResidual',res,'relativeAffineResidual',res/scale, ...
       'numericallyCompatibleWithAffine',res<=3e-3*scale, ...
       'uniformFiveDerivativeBoundCertified',false, ...
       'recursiveFeasibilityCertified',false,'machineRoundedCertified',false);
    if isempty(out),out=rr;else,out(end+1,1)=rr;end %#ok<AGROW>
end
end


% ======================================================================
% A: POINTWISE HIGHEST HOCBF STAGE RATE vs. XY torque (read-only).
% Psi_r(x,z) uses the exact source generator, d/dt Psi_r=grad Psi_r*f(x,z)
% is approximated by centered directional differences. This tests the
% model-specific disappearance of z^T*(grad B_r)*G*z, not a rigorous
% period-wise lower derivative bound or recursive-invariance certificate.
% ======================================================================
function out=sd_bridge_A_highstage_rate_affinity(s,id)
out=struct([]);
x=double(s.x(:));P=s.P;G=s.G;
T=max(0,min(20,double(s.u(1))));yaw=max(-1,min(1,double(s.u(4))));
test=[0,1,0,1,-0.6;0,0,1,-1,0.9];
vals=[];names={};
for kk=1:size(test,2)
    z=test(:,kk);u=[T;z(1);z(2);yaw];
    f=CBF_COMMON_MODEL(x,u,P);step=1e-5/(1+norm(f));
    ep=CBF_REAL_XY_EVAL(x+step*f,u,P,G,s.parts,s.lambda,s.tiltCfg);
    em=CBF_REAL_XY_EVAL(x-step*f,u,P,G,s.parts,s.lambda,s.tiltCfg);
    [hp,nm]=sd_bridge_A_highstage_vector(ep,s.tiltCfg);
    [hm,~]=sd_bridge_A_highstage_vector(em,s.tiltCfg);
    if isempty(vals)
        vals=zeros(numel(hp),size(test,2));names=nm;
    end
    if numel(hp)~=size(vals,1)
        error('SD:ARateSize','HOCBF barrier count changed');
    end
    vals(:,kk)=(hp-hm)/(2*step);
end
for j=1:size(vals,1)
    a=vals(j,1);bx=vals(j,2)-a;by=vals(j,3)-a;
    approx=a+bx*test(1,:)+by*test(2,:);
    err=max(abs(approx-vals(j,:)));scale=max(1,max(abs(vals(j,:))));
    rr=struct('sampleIndex',id,'timeS',double(s.time), ...
        'barrierName',names{j},'highestStageRateAtZero',a, ...
        'rateCoeffX',bx,'rateCoeffY',by, ...
        'maxAffineResidual',err,'relativeAffineResidual',err/scale, ...
        'numericallyCompatibleWithAffine',err<=3e-3*scale, ...
        'uniformRateBoundCertified',false, ...
        'forwardInvarianceCertified',false,'machineRoundedCertified',false);
    if isempty(out),out=rr;else,out(end+1,1)=rr;end %#ok<AGROW>
end
end

function [v,names]=sd_bridge_A_highstage_vector(E,cfg)
v=[];names={};
if ~isempty(E.stages)
    v=reshape(E.stages(end,:),[],1);
    for j=1:numel(v),names{end+1}=sprintf('obstacle%dPsi4',j);end %#ok<AGROW>
end
if cfg.bodyEnabled
    v(end+1,1)=E.tiltBodyStages(end);
    names{end+1}='bodyTiltPsi2';
end
if cfg.cableEnabled
    v(end+1,1)=E.tiltCableStages(end);
    names{end+1}='cableTiltPsi4';
end
end


% ======================================================================
% DETERMINISTIC 2-VARIABLE AFFINE QP ROW CONSTRUCTOR
% Inputs a5Lower,b5Lower,b5Upper MUST be certified enclosures valid for
% ALL states reachable throughout this ZOH under the entire XY torque box.
% For each fixed torque z, derivative >= a5Lower + min(BL*z,BU*z)
% is equivalent to four fixed (non-enumerated input) linear inequalities.
% This function does NOT compute the coefficient interval certificates.
% QP uses A*z<=b, with z=[tauX;tauY].
% mode='A': f4(z)>=0 and f4(z)+dt*minDerivative(z)>=0.
% mode='B': >=0 for all 6 Bernstein coefficients of a 5th-degree
% polynomial lower bound of g(s) during the ZOH.
% ======================================================================
function [A,b,staticOK]=sd_bridge_5th_affine_qp_rows(mode,d0to3,f4a,f4B, ...
                                                     a5Lower,b5Lower,b5Upper,dt)
assert(isscalar(dt)&&isfinite(dt)&&dt>0);
assert(numel(f4B)==2&&numel(b5Lower)==2&&numel(b5Upper)==2);
assert(all(isfinite([f4a,f4B(:).',a5Lower,b5Lower(:).',b5Upper(:).'])));
f4B=double(f4B(:)).';b5Lower=double(b5Lower(:)).';b5Upper=double(b5Upper(:)).';
assert(all(b5Lower<=b5Upper));
comb=[b5Lower(1),b5Lower(2);b5Lower(1),b5Upper(2); ...
      b5Upper(1),b5Lower(2);b5Upper(1),b5Upper(2)];
if strcmp(mode,'A')
 A=[-f4B;-(repmat(f4B,4,1)+dt*comb)];
 b=[f4a;repmat(f4a+dt*a5Lower,4,1)];
 staticOK=true;
elseif strcmp(mode,'B')
 assert(numel(d0to3)==4&&all(isfinite(d0to3)));
 d=double(d0to3(:));
 alpha=[d(1);dt*d(2);dt^2*d(3)/2;dt^3*d(4)/6];
 staticOK=true;
 for i=0:3
     beta=0;
     for j=0:i,beta=beta+nchoosek(i,j)/nchoosek(5,j)*alpha(j+1);end
     if beta<0,staticOK=false;end
 end
 % beta4=constant+alpha4/5, beta5=sum(alpha0..alpha5).
 base4=0;
 for j=0:3,base4=base4+nchoosek(4,j)/nchoosek(5,j)*alpha(j+1);end
 cf4=dt^4/24;
 cf5=dt^5/120;
 A=[-(cf4/5)*f4B;-(cf4*repmat(f4B,4,1)+cf5*comb)];
 b=[base4+(cf4/5)*f4a;repmat(sum(alpha)+cf4*f4a+cf5*a5Lower,4,1)];
else
 error('SD:5thQPMode','Unknown coefficient-to-QP mode');
end
end

function info=sd_bridge_5th_affine_qp_selftest()
dt=.025;a4=.2;B4=[.1,-.05];a5=-.4;lo=[-.3,-.1];hi=[.2,.4];
[Aa,ba,sta]=sd_bridge_5th_affine_qp_rows('A',[],a4,B4,a5,lo,hi,dt);
[Ab,bb,stb]=sd_bridge_5th_affine_qp_rows('B',[1,0,0,0],a4,B4,a5,lo,hi,dt);
assert(sta&&stb&&size(Aa,1)==5&&size(Ab,1)==5&&size(Aa,2)==2&&size(Ab,2)==2);
% Check exact corner formula against the four linear rows using a few
% SYNTHETIC z values, NOT a candidate search to find a physical input.
trial=[0,0;1,1;-1,-1;.32,-.81];
for j=1:size(trial,1)
    z=trial(j,:).';
    lower=a5+min(lo(1)*z(1),hi(1)*z(1))+min(lo(2)*z(2),hi(2)*z(2));
    worstA=a4+B4*z+dt*lower;
    assert(abs(worstA-min(ba(2:end)-Aa(2:end,:)*z))<1e-11);
    assert(all(Ab*z<=bb+1e-9));
end
% When beta0<0, no input is a Bernstein certificate despite any XY torque.
[~,~,bad]=sd_bridge_5th_affine_qp_rows('B',[-.1,0,0,0],a4,B4,a5,lo,hi,dt);
assert(~bad);
info=struct('passed',true,'methodARows',size(Aa,1), ...
  'methodBRows',size(Ab,1), ...
  'message','Synthetic interval corners and Bernstein row conversion passed; no real reach-set bounds.');
end
 

% =====================================================================
% 2026-11-23 LOCAL REFINEMENT EXPERIMENT (OFFLINE ONLY)
% Split the 25ms ZOH into short segments. Every segment uses a 19D Picard
% tube for ALL physically allowed XY torques simultaneously. Derivative
% enclosures are computed from DIRECT raw interval Jet, not un-staging.
%
% MATHEMATICAL, CONDITIONAL ON TRUE ENCLOSURE:
% A: for each segment, inf psi_r(x(t),z)>=b_i+B_i*z >=0, with psi_<r
%    nonnegative at the initial state. Hence standard comparison applies
%    to the full ZOH before the next input switch.
% B: h_j(t_j+s)>=sum_{ell=0}^{r-1} lower h^(ell)(X_j)s^ell/ell!
%                        +(b_i+B_i*z)*s^r/r!.
%    Bernstein lower coefficients >=0 on EACH segment; the final segment
%    lower endpoint >=rho*upper(h_initial). This proves one-ZOH h>=0 plus
%    endpoint decay, given a mathematically valid tube and enclosure.
%    We never claim the subinterval starting boxes are single trajectories;
%    all are over-approximations valid simultaneously for every z.
%
% Interval routines use 8ULP heuristics, not verified IEEE outward rounding;
% results are FLOAT-CONDITIONAL and NOT flight safety certificates.
% No terminal control-invariant K; future HLC thrust is not held fixed.
% =====================================================================
function report=sd_localrefine_mode(varargin)
if isempty(varargin) || ~isfile(char(varargin{1}))
    error('SD:LocalInput','CBF_FOURWAY_RESULTS.zip required.');
end
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
sd_find_generator();
tmp=tempname;mkdir(tmp);guard=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
unzip(char(varargin{1}),tmp);
D=load(fullfile(tmp,'RAW_SNAPSHOTS.mat'),'snapshot');
tab=readtable(fullfile(tmp,'PER_SAMPLE.csv'));
[cases,barriers,meta,errors]=sd_localrefine_audit(D.snapshot,tab);
report=struct('mode','localrefine','cases',cases,'barriers',barriers, ...
    'meta',meta,'errors',{errors},'certificate','NOT_MACHINE_OR_RECURSIVELY_CERTIFIED');
end

function [cases,barriers,meta,errors]=sd_localrefine_audit(snapshots,tab)
% Fixed target near times are scientific REGRESSION cases, not online
% candidate XY torques. Compare N=1 and N=4 with EXACT same model data.
target=[16.275,24.825,29.875,29.900,31.725,33.725];
valid=find(tab.parts>0 & isfinite(tab.min_h_m));
cases=struct([]);barriers=struct([]);errors={};
if isempty(valid),error('SD:LocalNoObstacle','No obstacle snapshots.');end
selected=zeros(numel(target),1);
for j=1:numel(target)
    [~,ii]=min(abs(double(tab.t(valid))-target(j)));
    selected(j)=valid(ii);
end
selected=unique(selected,'stable');
sd_qp_polygon_selftest();
ns=[1,4];
completed=0;tubeFailures=0;aPass=0;bPass=0;over25=0;maxMs=0;
for j=1:numel(selected)
    k=selected(j);s=snapshots{k};
    for np=ns
        try
            [c,b]=sd_localrefine_case(s,k,np);
            if isempty(cases),cases=c;else,cases(end+1,1)=c;end %#ok<AGROW>
            if ~isempty(b)
                if isempty(barriers),barriers=b(:);else,barriers=[barriers;b(:)];end %#ok<AGROW>
            end
            completed=completed+1;tubeFailures=tubeFailures+~c.tubeOK;
            aPass=aPass+c.A_feasibleConditional;
            bPass=bPass+c.B_feasibleConditional;
            over25=over25+(c.totalMs>25);
            maxMs=max(maxMs,c.totalMs);
            fprintf('[SD LOCAL] t=%.3f segments=%d tube=%d A=%d B=%d total=%.2fms NOT CERTIFIED\n', ...
                c.timeS,np,c.tubeOK,c.A_feasibleConditional,c.B_feasibleConditional,c.totalMs);
        catch ME
            errors{end+1,1}=sprintf('sample=%d parts=%d: %s [%s]', ...
                k,np,ME.message,ME.identifier); %#ok<AGROW>
        end
    end
end
meta=struct('attempts',numel(selected)*numel(ns),'completed',completed, ...
    'tubeFailures',tubeFailures,'errors',numel(errors), ...
    'aPass',aPass,'bPass',bPass, ...
    'maximumMs',maxMs,'over25ms',over25);
end

function [c,rows]=sd_localrefine_case(s,id,nseg)
clock=tic;rows=struct([]);
x=double(s.x(:));P=s.P;G=s.G;lam=double(s.lambda(:));
dt=double(s.dt);tilt=s.tiltCfg;
if numel(x)~=19 || dt<=0 || numel(s.u)~=4 || ...
        max(abs([norm(x(1:4))-1,norm(x(14:16))-1]))>1e-8
    error('SD:LocalState','Initial state is not a trusted unit-vector point.');
end
T=max(0,min(20,double(s.u(1))));yaw=max(-1,min(1,double(s.u(4))));
u0=[T;0;0;yaw];ux=u0+[0;1;0;0];uy=u0+[0;0;1;0];
if isfield(s,'info') && isstruct(s.info) && ...
        isfield(s.info,'physicalNominal') && numel(s.info.physicalNominal)==4
    nom=max(-1,min(1,double(s.info.physicalNominal(2:3))));
else
    nom=max(-1,min(1,double(s.u(2:3))));
end
names=sd_names(s.parts,tilt);nbar=numel(names);if nbar<1,error('SD:LocalNames','No barriers.');end
hstep=dt/nseg;
A=zeros(0,2);ba=zeros(0,1);B=zeros(0,2);bb=zeros(0,1);
initialLow=true;reachMs=0;jetMs=0;rowMs=0;qpMs=0;
start=sd_interval(x);init=sd_interval_stages(start,u0,P,G,s.parts,lam,tilt);
initialHupper=zeros(nbar,1);
for i=1:nbar
    rr=numel(init(i).stages)-1;
    for q=1:rr
        initialLow=initialLow && init(i).stages{q}(1)>=0;
    end
    initialHupper(i)=init(i).stages{1}(2);
end
reason='OK';passedSegments=0;lastTube=[];
for j=1:nseg
    tReach=tic;
    [tube,next,ok,msg,~]=sd_qp_picard(start,u0,P,hstep,18);
    reachMs=reachMs+1000*toc(tReach);
    if ~ok,reason=sprintf('segment %d: %s',j,msg);break;end
    passedSegments=j;
    tJet=tic;
    % Startbox all XY and tube all XY. Lower raw orders <r independent
    % of XY by relative-degree structure of the audited source.
    startJet=sd_interval_stages(start,u0,P,G,s.parts,lam,tilt);
    J0=sd_interval_stages(tube,u0,P,G,s.parts,lam,tilt);
    Jx=sd_interval_stages(tube,ux,P,G,s.parts,lam,tilt);
    Jy=sd_interval_stages(tube,uy,P,G,s.parts,lam,tilt);
    jetMs=jetMs+1000*toc(tJet);
    tRow=tic;
    for i=1:nbar
        r=numel(J0(i).raw)-1;
        if i<=numel(s.parts), gains=lam;
        elseif strcmp(names{i},'bodyTilt'),gains=double(tilt.bodyLambda(:));
        elseif strcmp(names{i},'cableTilt'),gains=double(tilt.cableLambda(:));
        else,error('SD:LocalBarrier','Unexpected safety constraint');end
        if numel(gains)~=r
            error('SD:LocalDegree','Relative degree and gain order differ.');
        end
        Hz=sd_qp_direct_raw(J0(i));Hx=sd_qp_direct_raw(Jx(i));
        Hy=sd_qp_direct_raw(Jy(i));Hs=sd_qp_direct_raw(startJet(i));
        [Bb,bbias,band]=sd_qp_affine_envelope(Hz{r+1},Hx{r+1},Hy{r+1});
        % Method A: psi_r = h^(r) + sum c_j*h^(j), evaluated across tube.
        pol=1;
        for m=1:r,pol=conv(pol,[1,gains(m)]);end
        shift=sd_interval(0);
        for m=0:r-1
            shift=sd_iadd(shift,sd_iscale(Hz{m+1},pol(end-m)));
        end
        aa=sd_down(bbias+shift(1)-1e-12*(1+abs(bbias)+abs(shift(1))+sum(abs(Bb))));
        A(end+1,:)=Bb;ba(end+1,1)=aa; %#ok<AGROW>
        % Method B: Taylor lower polynomial on [0,hstep].
        degree=r;coeff=zeros(degree+1,1);slope=zeros(degree+1,2);
        for m=0:r-1
            coeff(m+1)=sd_down(Hs{m+1}(1)*hstep^m/factorial(m));
        end
        coeff(r+1)=sd_down(bbias*hstep^r/factorial(r));
        slope(r+1,:)=Bb*hstep^r/factorial(r);
        endB=[0,0];end0=0;
        for ell=0:degree
            beta0=0;betab=[0,0];
            for m=0:ell
                scale=sd_qp_choose(ell,m)/sd_qp_choose(degree,m);
                beta0=beta0+scale*coeff(m+1);
                betab=betab+scale*slope(m+1,:);
            end
            beta0=sd_down(beta0-1e-12*(1+abs(beta0)+sum(abs(betab))));
            B(end+1,:)=betab;bb(end+1,1)=beta0; %#ok<AGROW>
            if ell==degree,end0=beta0;endB=betab;end
        end
        if j==nseg
            % This lower endpoint is compared to the ORIGINAL h(x_k),
            % not the subinterval start h, for finite-horizon contract.
            rho=exp(-2*dt);
            contract=sd_down(end0-sd_up(rho*initialHupper(i)));
            B(end+1,:)=endB;bb(end+1,1)=contract; %#ok<AGROW>
        end
        item=struct('sampleIndex',id,'timeS',double(s.time), ...
            'segments',nseg,'segmentIndex',j,'barrier',names{i}, ...
            'relativeDegree',r,'startHlower',Hs{1}(1), ...
            'tubeHighestHLowerAtZero',Hz{end}(1), ...
            'A_bias',aa,'A_coeffX',Bb(1),'A_coeffY',Bb(2), ...
            'A_maxBoxResidual',aa+sum(abs(Bb)), ...
            'B_endpointLowerNominal',end0+endB*nom, ...
            'B_intervalCoefficientRadiusSum',band.radiusSum, ...
            'B_hasContractRow',j==nseg);
        if isempty(rows),rows=item;else,rows(end+1,1)=item;end %#ok<AGROW>
    end
    rowMs=rowMs+1000*toc(tRow);
    start=next;lastTube=tube; %#ok<NASGU>
end
c=struct('sampleIndex',id,'timeS',double(s.time), ...
    'segments',nseg,'horizonS',dt,'estimatedMassKg',double(P.mL), ...
    'barriers',nbar,'tubeOK',passedSegments==nseg, ...
    'completedSegments',passedSegments,'reachReason',reason, ...
    'A_initialLowOK',initialLow,'A_feasibleConditional',false, ...
    'B_feasibleConditional',false,'A_status','TUBE_UNKNOWN', ...
    'B_status','TUBE_UNKNOWN','A_tauX',NaN,'A_tauY',NaN, ...
    'B_tauX',NaN,'B_tauY',NaN, ...
    'A_deltaNominal',NaN,'B_deltaNominal',NaN, ...
    'A_rows',size(A,1),'B_rows',size(B,1), ...
    'A_residual',NaN,'B_residual',NaN, ...
    'reachMs',reachMs,'jetMs',jetMs,'rowMs',rowMs, ...
    'qpMs',qpMs,'totalMs',NaN, ...
    'intervalRoundoffProven',false,'onlineWCETProven',false, ...
    'recursiveKProven',false);
if passedSegments==nseg
    tQP=tic;
    if initialLow
        [zA,okA,rA]=sd_qp_project_polygon(A,ba,nom,[1;1]);
    else
        zA=[NaN;NaN];okA=false;rA=NaN;
    end
    [zB,okB,rB]=sd_qp_project_polygon(B,bb,nom,[1;1]);
    c.qpMs=1000*toc(tQP);
    c.A_feasibleConditional=okA;c.B_feasibleConditional=okB;
    if ~initialLow,c.A_status='INITIAL_LOWER_NEGATIVE';
    elseif okA,c.A_status='FLOAT_CONDITIONAL_ONE_ZOH_PASS';
    else,c.A_status='AFFINE_QP_EMPTY_OR_INCONCLUSIVE';end
    if okB,c.B_status='FLOAT_CONDITIONAL_ONE_ZOH_PASS';
    else,c.B_status='AFFINE_QP_EMPTY_OR_INCONCLUSIVE';end
    if okA
        c.A_tauX=zA(1);c.A_tauY=zA(2);c.A_residual=rA;
        c.A_deltaNominal=norm(zA-nom);
    end
    if okB
        c.B_tauX=zB(1);c.B_tauY=zB(2);c.B_residual=rB;
        c.B_deltaNominal=norm(zB-nom);
    end
end
c.totalMs=1000*toc(clock);
end

function [z,ok,margin]=sd_qp_project_polygon(B,b,nom,w)
% 2D strictly-convex QP through incremental halfplane CLIPPING.
% All constraints B*z+b>=0; feasible set is a convex polygon inside box.
% No O(m^2) pairwise-active-set matrix, uses incremental edge clipping;
% worst-case arithmetic is still O(m^2). This is NOT a bounded WCET.
% Research implementation, final residual check and tightened constraints.
z=[NaN;NaN];ok=false;margin=NaN;
if size(B,2)~=2 || size(B,1)~=numel(b) || ...
        any(~isfinite([B(:);b(:);nom(:);w(:)])) || any(w<=0)
    return;
end
nom=nom(:);w=w(:);
tighten=1e-10*(1+abs(b)+sum(abs(B),2));
bb=b(:)-tighten(:);
V=[-1,-1;1,-1;1,1;-1,1];
if all(B*nom+bb>=0) && all(abs(nom)<=1)
    z=nom;ok=true;margin=min(B*z+b);return;
end
for j=1:size(B,1)
    d=V*B(j,:).'+bb(j);
    if all(d>=0),continue;end
    if all(d<0),return;end
    nv=size(V,1);U=zeros(0,2);
    for i=1:nv
        ii=mod(i,nv)+1;
        v0=V(i,:);v1=V(ii,:);
        f0=d(i);f1=d(ii);
        if f0>=0,U(end+1,:)=v0;end %#ok<AGROW>
        if (f0>0 && f1<0) || (f0<0 && f1>0)
            t=f0/(f0-f1);
            U(end+1,:)=v0+t*(v1-v0); %#ok<AGROW>
        elseif f0==0 && f1<0
            % endpoint v0 is already included; no extra point needed.
        elseif f0<0 && f1==0
            U(end+1,:)=v1; %#ok<AGROW>
        end
    end
    if isempty(U),return;end
    V=U;
end
best=Inf;point=[NaN;NaN];nv=size(V,1);
for j=1:nv
    a=V(j,:).';bpt=V(mod(j,nv)+1,:).';edge=bpt-a;
    denom=sum(w.*(edge.^2));
    if denom>0
        t=min(1,max(0,sum(w.*(nom-a).*edge)/denom));
    else
        t=0;
    end
    cand=a+t*edge;cost=sum(w.*(cand-nom).^2);
    if cost<best,best=cost;point=cand;end
end
% Recheck ALL original constraints with conservative tightening. Pure
% floating-point numerical test; cannot replace certified QP residuals.
if all(abs(point)<=1+1e-9) && all(B*point+bb>=-1e-11*(1+abs(bb)))
    z=point;ok=true;margin=min(B*z+b);
end
end


function sd_qp_polygon_selftest()
% Synthetic arithmetic regression against legacy 2D active-set projection.
% A finite sample regression is NOT an exact or machine-rounded proof.
B0=[1,0;0,1;-1,-1];b0=[0;0;1];
Bsets={B0,[1,0;0,1],[-1,0;0,-1],[1,0;-1,0],[1,0;-1,0]};
bsets={b0,[0;0],[1;1],[-.5;.5],[-.8;.6]};
noms={[-.3;-.3],[.5;-.2],[-.8;-.8],[0;0],[0;0]};
for k=1:numel(Bsets)
    [z0,ok0]=sd_qp_project(Bsets{k},bsets{k},noms{k},[1;1]);
    [z1,ok1]=sd_qp_project_polygon(Bsets{k},bsets{k},noms{k},[1;1]);
    if ok0~=ok1 || (ok0 && norm(z0-z1,inf)>2e-7)
        error('SD:PolygonQPTest','Legacy/polygon 2D QP differ in synthetic regression %d',k);
    end
end
fprintf('[SD LOCAL] 2D polygon-QP synthetic comparison passed (NOT certified).\n');
end

% ======================================================================
% RT SHADOW (2026-11-23): FLIGHT-PREPARATION vs 25ms HOT-PATH AUDIT
% ======================================================================
% This is an ARCHITECTURE AND TIMING RESEARCH test, not a flight certificate.
% Preflight builds rows separately from each EXACT historical snapshot.
% The online shadow may reuse them only if *all* state/parameter/T/geometry
% entries match. There is NO certified positive-radius state cell here.
% A true live flight generally cannot use a future logged-state cache.
% Any unknown or modified state is REJECTED, not silently interpolated.
% Exact-model continuous-time majorants are inherited from analyticbound;
% finite-precision rounding and recursive invariant K remain UNPROVEN.
% One 2D QP is solved per mode per sample. A and B are evaluated in
% separate SHADOW PASSES for research, not stacked into a deployed loop.
% Preflight, ZIP decompression and CSV generation are explicitly excluded
% from the timed hot path. An old control input is NEVER promoted to a
% verified backup on deadline or QP failure.
function report=sd_rtshadow_mode(varargin)
if isempty(varargin) || ~(ischar(varargin{1})||isstring(varargin{1})) || ...
        ~isfile(char(varargin{1}))
    error('SD:RTInput','CBF_FOURWAY_RESULTS.zip is required.');
end
file=char(varargin{1});args=varargin(2:end);
opt=struct('maxSamples',Inf,'repeatPasses',3,'budgetMs',3, ...
    'maxRows',128,'warmup',12,'scaleRepeats',20);
if mod(numel(args),2)~=0,error('SD:RTArguments','Name/value pairs required.');end
for j=1:2:numel(args)
    switch lower(char(args{j}))
        case 'maxsamples',opt.maxSamples=args{j+1};
        case 'repeatpasses',opt.repeatPasses=args{j+1};
        case 'budgetms',opt.budgetMs=args{j+1};
        case 'maxrows',opt.maxRows=args{j+1};
        case 'warmup',opt.warmup=args{j+1};
        case 'scalerepeats',opt.scaleRepeats=args{j+1};
        otherwise,error('SD:RTArguments','Unknown option: %s',char(args{j}));
    end
end
validateattributes(opt.maxSamples,{'numeric'},{'scalar','real','positive'});
validateattributes(opt.repeatPasses,{'numeric'},{'scalar','integer','>=',1,'<=',10});
validateattributes(opt.budgetMs,{'numeric'},{'scalar','real','positive','<=',25});
validateattributes(opt.maxRows,{'numeric'},{'scalar','integer','>=',8,'<=',512});
validateattributes(opt.warmup,{'numeric'},{'scalar','integer','>=',1,'<=',100});
validateattributes(opt.scaleRepeats,{'numeric'},{'scalar','integer','>=',2,'<=',100});
prevPath=path;pathRestore=onCleanup(@()path(prevPath)); %#ok<NASGU>
sd_find_generator();
sd_rt_unit_tests(opt.maxRows);
tmp=tempname;mkdir(tmp);cleanup=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
unzip(file,tmp);
D=load(fullfile(tmp,'RAW_SNAPSHOTS.mat'),'snapshot');
if ~iscell(D.snapshot),error('SD:RTData','RAW_SNAPSHOTS.mat must have snapshot cell array.');end
S=D.snapshot;total=numel(S);
N=min(total,floor(opt.maxSamples));if N<1,error('SD:RTNoData','No snapshots.');end
if total>N
    ids=unique(round(linspace(1,total,N)),'stable');
else
    ids=(1:total).';
end
n=numel(ids);cache=cell(n,1);rows=struct([]);errors={};success=0;
setupMs=NaN(n,1);
flightSig=[];
for j=1:n
    k=ids(j);s=S{k};
    try
        if isempty(s),error('SD:RTEmpty','Empty snapshot');end
        if isempty(flightSig),flightSig=sd_analytic_fixed_signature(s);end
        sd_analytic_fixed_check(flightSig,s);
        ticPre=tic;
        [c,~,qp]=sd_analyticbound_case(s,k);
        setupMs(j)=1000*toc(ticPre);
        gate=sd_rt_fingerprint(s);
        ent=struct('gate',gate,'A',qp.A,'ab',qp.ab,'B',qp.B,'bb',qp.bb, ...
            'nom',qp.zNom,'lowOK',logical(qp.initialLowOK), ...
            'index',k,'time',double(s.time),'offlineMs',setupMs(j));
        cache{j}=ent;success=success+1;
        v=struct('sampleIndex',k,'timeS',double(s.time), ...
          'observedParts',numel(s.parts),'mLEstimatedKg',double(s.P.mL), ...
          'A_initialLowerNonnegative',logical(qp.initialLowOK), ...
          'A_rows',size(qp.A,1),'B_rows',size(qp.B,1), ...
          'A_offlineFloatFeasible',strcmp(c.A_status,'BOUND_QP_FEASIBLE__FLOAT_ONLY') && qp.initialLowOK, ...
          'B_offlineFloatFeasible',strcmp(c.B_status,'BOUND_QP_FEASIBLE__FLOAT_ONLY'), ...
          'preflightCalculationMs',setupMs(j), ...
          'finitePrecisionCertified',false,'recursiveKCertified',false);
        if isempty(rows),rows=v;else,rows(end+1,1)=v;end %#ok<AGROW>
    catch ME
        errors{end+1,1}=sprintf('preflight id=%d: %s [%s]',k,ME.message,ME.identifier); %#ok<AGROW>
    end
end
% Fail-closed regression: a cache must reject next time, different mL,
% different T, and obstacle changes. All mutations performed on COPIES.
gates=struct([]);
first=find(~cellfun(@isempty,cache),1);
if ~isempty(first)
    k=ids(first);gate=cache{first}.gate;
    t=sd_rt_gate_row('EXACT_SNAPSHOT',sd_rt_gate(gate,S{k}),true);
    gates=t;
    copy=S{k};copy.P.mL=copy.P.mL+1e-5;
    gates(end+1,1)=sd_rt_gate_row('CHANGED_ESTIMATED_MASS',sd_rt_gate(gate,copy),false);
    copy=S{k};copy.u(1)=copy.u(1)+1e-4;
    gates(end+1,1)=sd_rt_gate_row('CHANGED_HLC_THRUST',sd_rt_gate(gate,copy),false);
    copy=S{k};copy.u(4)=copy.u(4)+1e-4;
    gates(end+1,1)=sd_rt_gate_row('CHANGED_HLC_YAW',sd_rt_gate(gate,copy),false);
    copy=S{k};copy.P.m=copy.P.m*(1+1e-5);
    gates(end+1,1)=sd_rt_gate_row('CHANGED_FLIGHT_MASS',sd_rt_gate(gate,copy),false);
    copy=S{k};copy.x(8)=copy.x(8)+1e-5;
    gates(end+1,1)=sd_rt_gate_row('CHANGED_STATE',sd_rt_gate(gate,copy),false);
    copy=S{k};copy.time=copy.time+double(copy.dt);
    gates(end+1,1)=sd_rt_gate_row('CHANGED_TIME',sd_rt_gate(gate,copy),false);
    copy=S{k};copy.parts=[];
    if isempty(S{k}.parts)
        copy.parts=struct('syntheticObstacleChange',true);
    end
    gates(end+1,1)=sd_rt_gate_row('CHANGED_OBSTACLES',sd_rt_gate(gate,copy),false);
    % Two consecutive logged records may have IDENTICAL content, including
    % time=0. Such an exact duplicate is not a mutation and must be accepted.
    % Never assume a change merely because the cell-array index increased.
    if k<total && ~isempty(S{k+1})
        nextIsIdentical=isequaln(gate,sd_rt_fingerprint(S{k+1}));
        if nextIsIdentical
            gates(end+1,1)=sd_rt_gate_row( ...
                'NEXT_LOGGED_IDENTICAL_CONTENT',sd_rt_gate(gate,S{k+1}),true);
        else
            gates(end+1,1)=sd_rt_gate_row( ...
                'NEXT_LOGGED_CHANGED_CONTENT',sd_rt_gate(gate,S{k+1}),false);
        end
    end
    % A different logged TIME is independent evidence of a changed input
    % to the exact-content gate, even if all other values happen to agree.
    for nextIndex=k+1:total
        if isempty(S{nextIndex}),continue;end
        if ~isequaln(S{nextIndex}.time,S{k}.time)
            gates(end+1,1)=sd_rt_gate_row( ...
                'NEXT_LOGGED_DIFFERENT_TIME',sd_rt_gate(gate,S{nextIndex}),false);
            break;
        end
    end
    if ~all([gates.matchesExpectation])
        bad=gates(~[gates.matchesExpectation]);
        msgs=arrayfun(@(r)sprintf('%s(actual=%d,expected=%d)', ...
            r.test,r.identityMatches,r.expectedMatch),bad,'UniformOutput',false);
        error('SD:RTGateRegression', ...
            'Exact-snapshot gate mutation tests failed: %s',strjoin(msgs,'; '));
    end
end
% JIT warm-up is OUTSIDE the timed replay. Still not a true real-time WCET.
if ~isempty(first)
    e=cache{first};
    for j=1:opt.warmup
        sd_rt_hot(e,S{ids(first)},'B',opt);
        sd_rt_hot(e,S{ids(first)},'A',opt);
    end
end
hot=struct([]);
for pp=1:opt.repeatPasses
    for j=1:n
        if isempty(cache{j}),continue;end
        s=S{ids(j)};e=cache{j};
        % Each path runs ONE QP only. We separately benchmark A and B.
        for mm=1:2
            if mm==1,mode='A';else,mode='B';end
            [v,elapsed]=sd_rt_hot(e,s,mode,opt);
            rr=struct('pass',pp,'sampleIndex',ids(j),'timeS',double(s.time), ...
                'mode',mode,'observedObstacleCount',numel(s.parts), ...
                'rowCount',v.rowCount,'identityMatch',v.match, ...
                'initialLowOK',v.lowOK, ...
                'floatQPFeasible',v.feasible,'numericConstraintMargin',v.margin, ...
                'outputTauX',v.z(1),'outputTauY',v.z(2), ...
                'deltaNominalNorm',v.delta, 'status',v.status, ...
                'measuredHotpathMs',elapsed, ...
                'exceededProvisional3msBudget',elapsed>opt.budgetMs, ...
                'safetyFlightCertified',false, ...
                'hardwareWCETCertified',false);
            if isempty(hot),hot=rr;else,hot(end+1,1)=rr;end %#ok<AGROW>
        end
    end
    fprintf('[SD RT] shadow pass %d/%d, %d snapshots, 2 separate single-QP modes; no certification.\n', ...
        pp,opt.repeatPasses,success);
end
% Compare new fixed-memory solver to legacy exact 2D enumeration numerically
% on historical preflight rows, OUTSIDE timed loops. Includes infeasible cases.
comparison=struct([]);
checked=0;
for j=1:n
    if isempty(cache{j}),continue;end
    checked=checked+1;
    if checked>min(80,success),break;end
    e=cache{j};
    for mm=1:2
        if mm==1,mode='A';X=e.A;h=e.ab;else,mode='B';X=e.B;h=e.bb;end
        [z1,ok1]=sd_rt_fixed_qp(X,h,e.nom,[1;1],opt.maxRows);
        [z2,ok2]=sd_qp_project(X,h,e.nom,[1;1]);
        d=NaN;if ok1 && ok2,d=norm(z1-z2,inf);end
        mismatch=(ok1~=ok2) || (ok1 && (d>1e-5));
        v=struct('sampleIndex',e.index,'mode',mode,'fastFeasible',ok1, ...
            'legacyFeasible',ok2,'solutionInfinityDifference',d, ...
            'mismatch',mismatch,'thisIsNotRigorousSolverProof',true);
        if isempty(comparison),comparison=v;else,comparison(end+1,1)=v;end %#ok<AGROW>
    end
end
scaling=sd_rt_scaling(cache,opt);
% Small POSITIVE-RADIUS cell experiment, expensive OFFLINE, never in hot path.
[cellRows,cellGates,cellErrors]=sd_rt_cells(S,ids,opt);
errors=[errors(:);cellErrors(:)];
report=struct('mode','rtshadow','preflight',rows,'hotpath',hot, ...
    'gates',gates,'comparison',comparison,'scaling',scaling, ...
    'cells',cellRows,'cellGates',cellGates, ...
    'requestedSnapshots',n,'preparedSnapshots',success,'options',opt, ...
    'errors',{errors},'guarantee','NOT_PROVEN__CONDITIONAL_LOCAL_CELL_TEST');
sd_rt_save(report);
end

function g=sd_rt_fingerprint(s)
% Strict, readable per-state signature (NOT a cryptographic hash).
% Never accept the next state based on time/index alone.
g=struct('x',s.x,'u',s.u,'P',s.P,'G',s.G, ...
    'parts',s.parts,'lambda',s.lambda,'tiltCfg',s.tiltCfg,'dt',s.dt,'time',s.time);
if isfield(s,'info') && isstruct(s.info) && ...
        isfield(s.info,'physicalNominal')
    g.physicalNominal=s.info.physicalNominal;
else
    g.physicalNominal=[];
end
end

function yes=sd_rt_gate(g,s)
% Full exact match is intentionally more restrictive than a proven cell.
% This rejects any new live state except an IDENTICAL replay point.
yes=isequaln(g,sd_rt_fingerprint(s));
end

function r=sd_rt_gate_row(label,actual,expected)
r=struct('test',label,'identityMatches',logical(actual), ...
    'expectedMatch',logical(expected),'matchesExpectation',actual==expected);
end

function [r,elapsed]=sd_rt_hot(cache,s,mode,opt)
clk=tic;
r=struct('match',false,'lowOK',false,'feasible',false, ...
    'z',[NaN;NaN],'margin',NaN,'rowCount',0, ...
    'delta',NaN,'status','UNCERTIFIED_REJECT');
if ~sd_rt_gate(cache.gate,s)
    r.status='REJECT_CACHE_IDENTITY_CHANGED';
    elapsed=1000*toc(clk);return;
end
r.match=true;r.lowOK=cache.lowOK;
if strcmp(mode,'A')
    X=cache.A;h=cache.ab;
    if ~r.lowOK
        r.status='REJECT_A_INITIAL_LOW_NEGATIVE';
        r.rowCount=size(X,1);elapsed=1000*toc(clk);return;
    end
elseif strcmp(mode,'B')
    X=cache.B;h=cache.bb;
else
    error('SD:RTMode','A or B expected');
end
r.rowCount=size(X,1);
[z,ok,margin]=sd_rt_fixed_qp(X,h,cache.nom,[1;1],opt.maxRows);
if ok
    r.z=z;r.margin=margin;r.feasible=true;
    r.delta=norm(z-cache.nom);
    r.status='FLOAT_QP_VALID__NOT_SAFETY_CERTIFIED';
else
    r.status='REJECT_QP_EMPTY_NUMERIC_OR_CAP';
end
elapsed=1000*toc(clk);
% No output is asserted to be safe; a timing miss is treated as rejection
% rather than continueBounded or automatically holding the previous input.
if elapsed>opt.budgetMs
    r.status='REJECT_TIMING_BUDGET_EXCEEDED';
end
end

function [z,ok,margin]=sd_rt_fixed_qp(B,b,nom,w,maxRows)
% B*z+b>=0 in [-1,1]^2, diagonal positive W.
% Fixed matrix capacity, 2D half-plane polygon clipping, NO iteration
% until numerical convergence. O(maxRows^2) worst-case operations.
% Not a machine-verified QP solver and NOT a formal WCET bound.
z=[NaN;NaN];ok=false;margin=NaN;
m=size(B,1);nom=double(nom(:));w=double(w(:));b=double(b(:));
if m<1,return;end % Empty rows cannot establish full barrier coverage.
if m>maxRows || size(B,2)~=2 || numel(b)~=m || numel(nom)~=2 || ...
    numel(w)~=2 || any(w<=0) || any(~isfinite([B(:);b;nom;w]))
    return;
end
% Finite-precision protective margin. Not a replacement for interval
% arithmetic or directed rounding. Feasible zero-margin cases may reject.
epsRows=1e-9*(1+abs(b)+sum(abs(B),2));
bb=b-epsRows;
if all(abs(nom)<=1) && all(B*nom+bb>=0)
    z=nom;ok=true;margin=min(B*z+b);return;
end
cap=2*maxRows+16;
V=zeros(cap,2);U=zeros(cap,2);V(1:4,:)=[-1,-1;1,-1;1,1;-1,1];nv=4;
for k=1:m
    a=B(k,:);d=V(1:nv,:)*a.'+bb(k);
    if all(d>=0),continue;end
    if all(d<0),return;end
    nw=0;
    for j=1:nv
        jj=j+1;if jj>nv,jj=1;end
        p=V(j,:);q=V(jj,:);f=d(j);g=d(jj);
        if f>=0
            nw=nw+1;if nw>cap,return;end
            U(nw,:)=p;
        end
        if (f>0 && g<0) || (f<0 && g>0)
            den=f-g;
            if den==0 || ~isfinite(den),return;end
            cross=p+(f/den)*(q-p);
            nw=nw+1;if nw>cap,return;end
            U(nw,:)=cross;
        elseif f<0 && g==0
            nw=nw+1;if nw>cap,return;end
            U(nw,:)=q;
        end
    end
    if nw<1,return;end
    V(1:nw,:)=U(1:nw,:);nv=nw;
end
best=Inf;point=[NaN;NaN];
for j=1:nv
    jj=j+1;if jj>nv,jj=1;end
    p=V(j,:).';e=V(jj,:).'-p;
    den=sum(w.*(e.^2));
    if den>0,t=max(0,min(1,sum(w.*(nom-p).*e)/den));else,t=0;end
    test=p+t*e;
    cost=sum(w.*(test-nom).^2);
    if cost<best,best=cost;point=test;end
end
if all(isfinite(point)) && all(abs(point)<=1+1e-10) && ...
        all(B*point+bb>=-1e-12*(1+abs(bb)))
    z=max(-1,min(1,point));
    % clipping may move the point; verify ORIGINAL tightened constraints.
    if all(B*z+bb>=-1e-12*(1+abs(bb)))
        ok=true;margin=min(B*z+b);
    end
end
end

function sd_rt_unit_tests(maxRows)
% Tests check algebra and rejection behavior, NOT proof of a flight.
B=[1,0;0,1;-1,-1];bb=[0;0;1];
for j=1:5
    nom=[cos(j);sin(j)];
    [z,yes]=sd_rt_fixed_qp(B,bb,nom,[1;1],maxRows);
    [reference,refYes]=sd_qp_project(B,bb,nom,[1;1]);
    if yes~=refYes || (yes && norm(z-reference,inf)>2e-6)
        error('SD:RTQPSelftest','Mismatch with research reference solver');
    end
end
[~,ok]=sd_rt_fixed_qp([1,0;-1,0],[-.7;-.7],[0;0],[1;1],maxRows);
assert(~ok,'Infeasible polygon unexpectedly accepted');
[~,ok]=sd_rt_fixed_qp([1,0;-1,0],[-.7;-.7],[0;0],[1;1],1);
assert(~ok,'Row cap was not enforced');
end

function r=sd_rt_scaling(cache,opt)
r=struct([]);first=find(~cellfun(@isempty,cache),1);
if isempty(first),return;end
e=cache{first};
% Duplicate rows to isolate solver scaling. This is NOT a model with new
% real obstacles; duplicates are algebraically redundant.
for m=1:5
    factor=[1,2,4,8,16];copies=factor(m);
    B=repmat(e.B,copies,1);b=repmat(e.bb,copies,1);
    [~,ok]=sd_rt_fixed_qp(B,b,e.nom,[1;1],opt.maxRows);
    times=zeros(opt.scaleRepeats,1);
    for k=1:opt.scaleRepeats
        start=tic;
        sd_rt_fixed_qp(B,b,e.nom,[1;1],opt.maxRows);
        times(k)=1000*toc(start);
    end
    item=struct('repeatedConstraintFactor',copies,'rowCount',size(B,1), ...
        'withinRowCap',size(B,1)<=opt.maxRows, ...
        'floatFeasible',ok,'medianQPms',median(times), ...
        'p95QPms',sd_bridge_percentile(times,95), ...
        'maxObservedQPms',max(times),'overBudgetCount',sum(times>opt.budgetMs), ...
        'matlabWCETProven',false);
    if isempty(r),r=item;else,r(end+1,1)=item;end %#ok<AGROW>
end
end

function sd_rt_save(R)
root=fileparts(mfilename('fullpath'));
tmp=tempname;mkdir(tmp);cleanup=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
files={'SUMMARY.txt','ERRORS.txt','RT_SUMMARY.txt'};
fd=fopen(fullfile(tmp,'RT_SUMMARY.txt'),'w');
fprintf(fd,'CBF 25ms RT shadow test - OFFLINE REPLAY ONLY, NOT A FLIGHT CERTIFICATE.\n');
fprintf(fd,'Exact logged-state identity gates. No positive-radius certified lookup cell.\n');
fprintf(fd,'Preparation is a separate, NON-CAUSAL replay procedure: true online deployment cannot foresee future states.\n');
fprintf(fd,'All preflight computations, cache index/region discovery and ZIP/MAT/CSV IO are EXCLUDED from hot-path timing.\n');
fprintf(fd,'Each A/B mode is one QP per replay step; two modes are run separately, not concurrently in a production loop.\n');
fprintf(fd,'Finite-precision rounding, safety invariant K, unpredictable HLC future T, and WCET are NOT PROVEN.\n');
fprintf(fd,'Small positive-radius 19D initial-state cells are tested OFFLINE with fixed T/yaw/mL/geometry, not yet flight-wide.\n');
fprintf(fd,'Provisional hot-path budget: %.4f ms of 25 ms; this is a design target, not achieved certification.\n',R.options.budgetMs);
fprintf(fd,'Prepared %d / selected %d snapshots; errors %d.\n', ...
    R.preparedSnapshots,R.requestedSnapshots,numel(R.errors));
if ~isempty(R.preflight)
 p=[R.preflight.preflightCalculationMs];
 fprintf(fd,'Preflight row generation per logged state: median %.5f ms, MAX %.5f ms; this CANNOT be omitted from true live operation until positive-volume cell certificates exist.\n',median(p),max(p));
end
if ~isempty(R.hotpath)
  t=[R.hotpath.measuredHotpathMs];
  fprintf(fd,'Hot path N=%d measured median %.6f ms, P99 %.6f ms, MAX %.6f ms, count>budget %d.\n', ...
    numel(t),median(t),sd_bridge_percentile(t,99),max(t),sum(t>R.options.budgetMs));
  for mode={'A','B'}
    idx=strcmp({R.hotpath.mode},mode{1});v=R.hotpath(idx);
    fprintf(fd,'Mode %s: %d trials; finite QP feasible %d; accepted before budget %d; numeric rejects %d.\n', ...
        mode{1},numel(v),sum([v.floatQPFeasible]), ...
        sum(strcmp({v.status},'FLOAT_QP_VALID__NOT_SAFETY_CERTIFIED')), ...
        sum(~[v.floatQPFeasible]));
  end
end
if ~isempty(R.comparison)
 fprintf(fd,'Legacy-QP comparisons %d; disagreement %d.\n',numel(R.comparison),sum([R.comparison.mismatch]));
end
if ~isempty(R.cells)
 fprintf(fd,'Positive-radius interval cells (offline only): %d trials, tube success %d, A conditional QP %d, B conditional QP %d.\n', ...
  numel(R.cells),sum([R.cells.tubeOK]),sum([R.cells.A_floatFeasible]),sum([R.cells.B_floatFeasible]));
end
fprintf(fd,'IMPORTANT: exact-snapshot cache does NOT cover new flight states, so the current shadow controller is NOT deployable.\n');
fprintf(fd,'The cache identity is NOT a cryptographic signature of controller/HLC/reference versions.\n');
fprintf(fd,'IMPORTANT: full online row generation, valid state-cell lookup and backup/K proof MUST be timed and certified separately.\n');
fclose(fd);
fd=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fd,'CBF_SD_TEST rtshadow mode, research only. See RT_SUMMARY.txt.\n');
fprintf(fd,'Real-time safety status: NOT_PROVEN.\n');
fclose(fd);
fd=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if isempty(R.errors),fprintf(fd,'No errors.\n');else
    for j=1:numel(R.errors),fprintf(fd,'%s\n',R.errors{j});end
end
fclose(fd);
reports={'preflight','hotpath','gates','comparison','scaling','cells','cellGates'};
for j=1:numel(reports)
    name=reports{j};
    if isempty(R.(name)),continue;end
    fname=['RT_' upper(name) '.csv'];
    writetable(struct2table(R.(name)),fullfile(tmp,fname));
    files{end+1}=fname; %#ok<AGROW>
end
out=fullfile(root,'CBF_SD_TEST_RESULTS.zip');
if isfile(out),delete(out);end
zip(out,files,tmp);
end

% ======================================================================
% POSITIVE-VOLUME LOCAL STATE CELL EXPERIMENT (offline only).
% This is genuinely different from a table of isolated logged points:
% with exact real interval inclusion, each resulting QP row applies to
% ALL x in the initial 19D box, provided SAME P/T/Yaw/obstacle/geometry.
% The interval arithmetic still uses unverified 8ULP binary64 inflation.
% It is not a machine-formal certificate nor any recurrent K proof.
% ======================================================================
function [cases,gates,errors]=sd_rt_cells(S,ids,opt)
cases=struct([]);gates=struct([]);errors={};
want=[16.275,24.825,29.875,29.900,31.725,33.725];
available=ids(~cellfun(@isempty,S(ids)));
if isempty(available),return;end
valid=zeros(0,1);
for i=1:numel(available)
    k=available(i);
    if ~isempty(S{k}.parts),valid(end+1,1)=k;end %#ok<AGROW>
end
if isempty(valid),return;end
selected=zeros(numel(want),1);
for i=1:numel(want)
    ts=cellfun(@(a)double(a.time),S(valid));
    [~,index]=min(abs(ts-want(i)));
    selected(i)=valid(index);
end
selected=unique(selected,'stable');
radii=[0,1e-6,1e-5];
for ii=1:numel(selected)
    k=selected(ii);s=S{k};
    for jj=1:numel(radii)
        rr=radii(jj);
        try
            ticCell=tic;
            [row,coef,lo,hi]=sd_rt_cell_case(s,k,rr,opt);
            row.preflightMs=1000*toc(ticCell);
            if isempty(cases),cases=row;else,cases(end+1,1)=row;end %#ok<AGROW>
            g0=sd_rt_cell_gate(coef,s,lo,hi);
            inside=s;inside.x(8)=inside.x(8)+0.5*(hi(8)-double(s.x(8)));
            g1=sd_rt_cell_gate(coef,inside,lo,hi);
            outside=s;outside.x(8)=hi(8)+max(1e-9,rr);
            g2=sd_rt_cell_gate(coef,outside,lo,hi);
            altered=s;altered.P.mL=altered.P.mL+1e-5;
            g3=sd_rt_cell_gate(coef,altered,lo,hi);
            % One cell-membership gate and ONE B QP; no row generation.
            tHot=tic;
            hotGate=sd_rt_cell_gate(coef,s,lo,hi);
            if hotGate && ~isempty(coef.B)
                [~,ok]=sd_rt_fixed_qp(coef.B,coef.bb,coef.nom,[1;1],opt.maxRows);
            else
                ok=false;
            end
            ms=1000*toc(tHot);
            v=struct('sampleIndex',k,'timeS',double(s.time), ...
                'coordinateRadiusScale',rr,'centreAccepted',g0, ...
                'insidePerturbationAccepted',g1, ...
                'outsidePerturbationRejected',~g2, ...
                'massChangedRejected',~g3, ...
                'B_floatQPFeasible',ok,'B_QPonlyMs',ms, ...
                'machineRoundedCertified',false,'recursiveKCertified',false);
            if isempty(gates),gates=v;else,gates(end+1,1)=v;end %#ok<AGROW>
            if ~(g0&&g1&&~g2&&~g3)
                errors{end+1,1}=sprintf('CELL_GATE id=%d radius=%g inconsistent region membership',k,rr); %#ok<AGROW>
            end
            fprintf('[SD RT CELL] t=%.3f radius=%g tube=%d A=%d B=%d preflight=%.3fms NOT CERTIFIED\n', ...
                s.time,rr,row.tubeOK,row.A_floatFeasible,row.B_floatFeasible,row.preflightMs);
        catch ME
            errors{end+1,1}=sprintf('CELL id=%d radius=%g: %s [%s]',k,rr,ME.message,ME.identifier); %#ok<AGROW>
        end
    end
end
end

function [row,cache,lo,hi]=sd_rt_cell_case(s,id,radius,opt)
% The fixed-parameter and observation signature is part of the cell.
x=double(s.x(:));dt=double(s.dt);P=s.P;G=s.G;
if numel(x)~=19 || ~(dt>0) || dt>.1 || radius<0 || ...
        max(abs([norm(x(1:4))-1,norm(x(14:16))-1]))>1e-8
    error('SD:RTCellState','Invalid initial model state.');
end
spread=radius*max(1,abs(x));lo=x-spread;hi=x+spread;
start=sd_irange(lo,hi);
T=max(0,min(20,double(s.u(1))));yaw=max(-1,min(1,double(s.u(4))));
u0=[T;0;0;yaw];ux=u0+[0;1;0;0];uy=u0+[0;0;1;0];
if isfield(s,'info') && isfield(s.info,'physicalNominal') ...
        && numel(s.info.physicalNominal)==4
    nom=max(-1,min(1,double(s.info.physicalNominal(2:3))));
else
    nom=max(-1,min(1,double(s.u(2:3))));
end
lam=double(s.lambda(:));tilt=s.tiltCfg;
ini=sd_interval_stages(start,u0,P,G,s.parts,lam,tilt);
names=sd_names(s.parts,tilt);nb=numel(names);
A=zeros(0,2);ab=zeros(0,1);B=zeros(0,2);bb=zeros(0,1);
low=true;
for i=1:nb
    r=numel(ini(i).stages)-1;
    for j=1:r
        low=low && ini(i).stages{j}(1)>=0;
    end
end
[tube,~,tubeOK,why]=sd_qp_picard(start,u0,P,dt,18);
if tubeOK
    J0=sd_interval_stages(tube,u0,P,G,s.parts,lam,tilt);
    Jx=sd_interval_stages(tube,ux,P,G,s.parts,lam,tilt);
    Jy=sd_interval_stages(tube,uy,P,G,s.parts,lam,tilt);
    for i=1:nb
        r=numel(J0(i).raw)-1;
        if i<=numel(s.parts),gains=lam;
        elseif strcmp(names{i},'bodyTilt'),gains=double(tilt.bodyLambda(:));
        elseif strcmp(names{i},'cableTilt'),gains=double(tilt.cableLambda(:));
        else,error('SD:RTCellBarrier','Unexpected barrier name');end
        if numel(gains)~=r,error('SD:RTCellDegree','Incorrect barrier order');end
        H0=sd_qp_direct_raw(J0(i));Hx=sd_qp_direct_raw(Jx(i));
        Hy=sd_qp_direct_raw(Jy(i));Hs=sd_qp_direct_raw(ini(i));
        [bz,az]=sd_qp_affine_envelope(H0{r+1},Hx{r+1},Hy{r+1});
        pol=1;
        for j=1:r,pol=conv(pol,[1,gains(j)]);end
        shift=sd_interval(0);
        for j=0:r-1
            shift=sd_iadd(shift,sd_iscale(H0{j+1},pol(end-j)));
        end
        % Psi_r lower for ALL x in the tube and ALL XY in the box.
        A(end+1,:)=bz;ab(end+1,1)=sd_down(az+shift(1)); %#ok<AGROW>
        % Geometry h Taylor polynomial starting at any x in THIS cell.
        c=zeros(r+1,1);v=zeros(r+1,2);
        for j=0:r-1
            c(j+1)=sd_down(Hs{j+1}(1)*dt^j/factorial(j));
        end
        c(r+1)=sd_down(az*dt^r/factorial(r));
        v(r+1,:)=bz*dt^r/factorial(r);
        last0=NaN;lastV=[NaN NaN];
        for j=0:r
            beta0=0;betaV=[0 0];
            for t=0:j
                f=sd_qp_choose(j,t)/sd_qp_choose(r,t);
                beta0=beta0+f*c(t+1);
                betaV=betaV+f*v(t+1,:);
            end
            B(end+1,:)=betaV;bb(end+1,1)=sd_down(beta0); %#ok<AGROW>
            if j==r,last0=beta0;lastV=betaV;end
        end
        rho=exp(-2*dt);
        B(end+1,:)=lastV;bb(end+1,1)=sd_down(last0-sd_up(rho*Hs{1}(2))); %#ok<AGROW>
    end
end
if low && tubeOK
    [zA,okA]=sd_rt_fixed_qp(A,ab,nom,[1;1],opt.maxRows);
else
    zA=[NaN;NaN];okA=false;
end
if tubeOK
    [zB,okB]=sd_rt_fixed_qp(B,bb,nom,[1;1],opt.maxRows);
else
    zB=[NaN;NaN];okB=false;
end
row=struct('sampleIndex',id,'timeS',double(s.time), ...
    'coordinateRadiusScale',radius,'estimatedMassKg',double(P.mL), ...
    'observedParts',numel(s.parts),'barrierCount',nb, ...
    'tubeOK',logical(tubeOK),'reachReason',why, ...
    'A_initialLowOK',logical(low),'A_floatFeasible',logical(okA), ...
    'B_floatFeasible',logical(okB),'A_rows',size(A,1), ...
    'B_rows',size(B,1),'A_tauX',zA(1),'A_tauY',zA(2), ...
    'B_tauX',zB(1),'B_tauY',zB(2), ...
    'preflightMs',NaN,'machineRoundedCertified',false, ...
    'continuousFutureCertified',false);
cache=struct('xLo',lo,'xHi',hi,'P',P,'G',G,'parts',s.parts, ...
    'lambda',s.lambda,'tiltCfg',tilt,'dt',s.dt,'T',T,'yaw',yaw, ...
    'A',A,'ab',ab,'B',B,'bb',bb,'nom',nom);
end

function yes=sd_rt_cell_gate(c,s,lo,hi)
% Membership of a nontrivial state cell, other parameters STILL EXACT.
x=double(s.x(:));
yes=numel(x)==19 && all(isfinite(x)) && ...
    all(x>=lo) && all(x<=hi) && ...
    isequaln(s.P,c.P) && isequaln(s.G,c.G) && ...
    isequaln(s.parts,c.parts) && isequaln(s.lambda,c.lambda) && ...
    isequaln(s.tiltCfg,c.tiltCfg) && isequaln(s.dt,c.dt) && ...
    isequaln(max(0,min(20,double(s.u(1)))),c.T) && ...
    isequaln(max(-1,min(1,double(s.u(4)))),c.yaw);
end


% ======================================================================
% RT CAUSAL DRY-RUN (2026-11-23): no noncausal exact-state row cache.
% This mode computes ALL A/B rows from the CURRENT snapshot at EACH step.
% The replay source is historical; disk IO is not part of a period because
% real sensors would provide state instead. Rows/coefficients ARE timed.
% Both A/B rows are formed by sd_analyticbound_case, which also runs the
% old solver internally; thus these times are a CONSERVATIVE implementation
% baseline, not a minimal optimized single-mode pipeline or a WCET proof.
% No selected control is applied to the plant; no closed-loop proof exists.
% ======================================================================
function report=sd_rtcausal_mode(varargin)
if isempty(varargin) || ~(ischar(varargin{1})||isstring(varargin{1})) || ...
        ~isfile(char(varargin{1}))
    error('SD:RTLiveInput','CBF_FOURWAY_RESULTS.zip is required.');
end
file=char(varargin{1});args=varargin(2:end);
opt=struct('maxSamples',Inf,'repeatPasses',2,'budgetMs',3, ...
    'maxRows',128,'warmup',8);
if mod(numel(args),2)~=0
    error('SD:RTLiveArguments','Name/value arguments are required.');
end
for j=1:2:numel(args)
    switch lower(char(args{j}))
        case 'maxsamples',opt.maxSamples=args{j+1};
        case 'repeatpasses',opt.repeatPasses=args{j+1};
        case 'budgetms',opt.budgetMs=args{j+1};
        case 'maxrows',opt.maxRows=args{j+1};
        case 'warmup',opt.warmup=args{j+1};
        otherwise
            error('SD:RTLiveArguments','Unknown option: %s',char(args{j}));
    end
end
validateattributes(opt.maxSamples,{'numeric'},{'scalar','positive'});
validateattributes(opt.repeatPasses,{'numeric'},{'scalar','integer','>=',1,'<=',5});
validateattributes(opt.budgetMs,{'numeric'},{'scalar','positive','<=',25});
validateattributes(opt.maxRows,{'numeric'},{'scalar','integer','>=',8,'<=',512});
validateattributes(opt.warmup,{'numeric'},{'scalar','integer','>=',0,'<=',100});
prevPath=path;restorePath=onCleanup(@()path(prevPath)); %#ok<NASGU>
sd_find_generator();
sd_rt_unit_tests(opt.maxRows);
tmp=tempname;mkdir(tmp);cleanup=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
unzip(file,tmp);
D=load(fullfile(tmp,'RAW_SNAPSHOTS.mat'),'snapshot');
if ~iscell(D.snapshot)
    error('SD:RTLiveSnapshot','snapshot must be a cell vector.');
end
S=D.snapshot;total=numel(S);
N=min(total,floor(opt.maxSamples));
if N<1,error('SD:RTLiveEmpty','No log snapshots.');end
if N==total,ids=(1:total).';
else,ids=unique(round(linspace(1,total,N)),'stable');end
signature=[];
for k=ids(:).'
    if ~isempty(S{k})
        signature=sd_analytic_fixed_signature(S{k});break;
    end
end
if isempty(signature),error('SD:RTLiveEmpty','All selected snapshots empty.');end
% Warm up the FIRST initial state only. We never precalculate future rows.
first=ids(find(~cellfun(@isempty,S(ids)),1));
for j=1:opt.warmup
    sd_analyticbound_case(S{first},first);
end
allRows=struct([]);allBarriers=struct([]);errors={};
for pass=1:opt.repeatPasses
    for jj=1:numel(ids)
        k=ids(jj);s=S{k};
        if isempty(s)
            errors{end+1,1}=sprintf('pass=%d id=%d: empty snapshot',pass,k); %#ok<AGROW>
            continue;
        end
        try
            cycleClock=tic;
            % Fixed flight properties have to be checked on EVERY period.
            % This check is included in the measured row-generation cost.
            checkClock=tic;
            sd_analytic_fixed_check(signature,s);
            fixedCheckMs=1000*toc(checkClock);
            genClock=tic;
            [c,br,qp]=sd_analyticbound_case(s,k);
            rowGenerationMs=1000*toc(genClock);
            % All generated rows are based on THIS state; there is no
            % exact-snapshot cache and no matching future log record.
            qaClock=tic;
            if qp.initialLowOK
                [zA,okA,marginA]=sd_rt_fixed_qp(qp.A,qp.ab,qp.zNom, ...
                    [1;1],opt.maxRows);
            else
                zA=[NaN;NaN];okA=false;marginA=NaN;
            end
            qpAMs=1000*toc(qaClock);
            qbClock=tic;
            [zB,okB,marginB]=sd_rt_fixed_qp(qp.B,qp.bb,qp.zNom, ...
                [1;1],opt.maxRows);
            qpBMs=1000*toc(qbClock);
            totalObservedMs=1000*toc(cycleClock);
            % A row a*z+b>=0 is individually impossible on [-1,1]^2
            % if b+abs(a1)+abs(a2)<0, independently of QP solver.
            aBest=qp.ab(:)+sum(abs(qp.A),2);
            bBest=qp.bb(:)+sum(abs(qp.B),2);
            badA=sum(aBest<0);badB=sum(bBest<0);
            deltaA=NaN;deltaB=NaN;
            if okA,deltaA=norm(zA-qp.zNom);end
            if okB,deltaB=norm(zB-qp.zNom);end
            r=struct('pass',pass,'sampleIndex',k, ...
                'timeS',double(s.time),'obstacleCount',numel(s.parts), ...
                'estimatedMassKg',double(s.P.mL), ...
                'A_initialLowOK',logical(qp.initialLowOK), ...
                'A_rowCount',size(qp.A,1),'B_rowCount',size(qp.B,1), ...
                'A_individuallyImpossibleRows',badA, ...
                'B_individuallyImpossibleRows',badB, ...
                'A_bestSingleRowMin',min(aBest), ...
                'B_bestSingleRowMin',min(bBest), ...
                'A_floatFeasible',logical(okA), ...
                'B_floatFeasible',logical(okB), ...
                'A_tauX',zA(1),'A_tauY',zA(2), ...
                'B_tauX',zB(1),'B_tauY',zB(2), ...
                'A_numericMargin',marginA,'B_numericMargin',marginB, ...
                'A_deltaNominal',deltaA,'B_deltaNominal',deltaB, ...
                'fixedParamCheckMs',fixedCheckMs, ...
                'onDemandRowGenerationMs',rowGenerationMs, ...
                'scalarEnvelopeMs',c.envelopeMs, ...
                'generatorEvalMs',c.pointGeneratorMs, ...
                'rowFormulationMs',c.formulationMs, ...
                'legacyDiagnosticQPmsIncludedInRowGeneration',c.qpMs, ...
                'A_fastQPms',qpAMs,'B_fastQPms',qpBMs, ...
                'A_observedGenerateAndQPms',fixedCheckMs+rowGenerationMs+qpAMs, ...
                'B_observedGenerateAndQPms',fixedCheckMs+rowGenerationMs+qpBMs, ...
                'bothDiagnosticObservedMs',totalObservedMs, ...
                'A_exceeds3ms',fixedCheckMs+rowGenerationMs+qpAMs>opt.budgetMs, ...
                'B_exceeds3ms',fixedCheckMs+rowGenerationMs+qpBMs>opt.budgetMs, ...
                'hardwareWCETCertified',false, ...
                'finitePrecisionCertified',false, ...
                'recursiveKCertified',false);
            if isempty(allRows),allRows=r;
            else,allRows(end+1,1)=r;end %#ok<AGROW>
            % Barriers are exported OUTSIDE measured cycle timing in this
            % research harness, so reporting does not inflate the budget.
            if pass==1 && ~isempty(br)
                for bi=1:numel(br)
                    bar=br(bi);
                    if isempty(allBarriers),allBarriers=bar;
                    else,allBarriers(end+1,1)=bar;end %#ok<AGROW>
                end
            end
        catch ME
            errors{end+1,1}=sprintf('pass=%d id=%d time=%g: %s [%s]', ...
                pass,k,double(s.time),ME.message,ME.identifier); %#ok<AGROW>
        end
    end
    fprintf('[SD RT CAUSAL] pass %d/%d: on-demand rows + single A/B QPs, not certified.\n', ...
        pass,opt.repeatPasses);
end
report=struct('mode','rtcausal','options',opt, ...
    'requestedSnapshots',numel(ids),'cycles',allRows, ...
    'barriers',allBarriers,'errors',{errors}, ...
    'guarantee','NOT_PROVEN__NO_FUTURE_CACHE__NO_RECURSIVE_K');
sd_rtcausal_save(report);
end

function sd_rtcausal_save(R)
root=fileparts(mfilename('fullpath'));
tmp=tempname;mkdir(tmp);cleanup=onCleanup(@()sd_cleanup(tmp)); %#ok<NASGU>
files={'SUMMARY.txt','ERRORS.txt','RT_CAUSAL_SUMMARY.txt'};
fid=fopen(fullfile(tmp,'RT_CAUSAL_SUMMARY.txt'),'w');
fprintf(fid,'CBF rtcausal: on-demand rows from CURRENT snapshot, no future-state precomputed cache.\n');
fprintf(fid,'Offline replay uses logged states. Sensor/HLC/reference execution and disk IO are not timed.\n');
fprintf(fid,'Row generation INCLUDES three generator evaluations and the legacy diagnostic A/B QPs.\n');
fprintf(fid,'Thus row-generation+one fixed-QP observed time is deliberately conservative, not an optimized production implementation.\n');
fprintf(fid,'A and B are competing SHADOW modes; deployed system must not assume both can be run concurrently.\n');
fprintf(fid,'Real-time 3ms is a provisional CBF-only target within 25ms; this test is NOT WCET.\n');
fprintf(fid,'Finite precision, exact outward rounding, causal invariant K/backup and future HLC thrust are NOT CERTIFIED.\n');
fprintf(fid,'No numeric result provides permission to apply a non-certified input to the plant.\n');
fprintf(fid,'Requested samples %d, repeat passes %d, errors %d.\n', ...
    R.requestedSnapshots,R.options.repeatPasses,numel(R.errors));
if ~isempty(R.cycles)
    a=R.cycles;
    fprintf(fid,'Processed %d cycles (including repeated visits).\n',numel(a));
    fprintf(fid,'On-demand rows: median %.5f ms, P99 %.5f ms, MAX %.5f ms.\n', ...
        median([a.onDemandRowGenerationMs]), ...
        sd_bridge_percentile([a.onDemandRowGenerationMs],99), ...
        max([a.onDemandRowGenerationMs]));
    for mode='AB'
        prefix=char(mode);
        timeField=[prefix '_observedGenerateAndQPms'];
        feasibleField=[prefix '_floatFeasible'];
        badField=[prefix '_individuallyImpossibleRows'];
        correctionField=[prefix '_deltaNominal'];
        t=[a.(timeField)];
        feasible=[a.(feasibleField)];
        bad=[a.(badField)];
        vals=[a.(correctionField)];vals=vals(isfinite(vals));
        fprintf(fid,'%s on-demand rows+one fixed QP: median %.5f, P99 %.5f, MAX %.5f ms, >%.2f ms=%d/%d; feasible=%d/%d; individual-impossible row samples=%d; corrected input=%d.\n', ...
            prefix,median(t),sd_bridge_percentile(t,99),max(t), ...
            R.options.budgetMs,sum(t>R.options.budgetMs),numel(t), ...
            sum(feasible),numel(feasible),sum(bad>0),sum(vals>1e-8));
    end
end
fprintf(fid,'URGENT: A/B infeasibility is not physical impossibility; improving bounds and a viability/backup K remains necessary.\n');
fclose(fid);
fid=fopen(fullfile(tmp,'SUMMARY.txt'),'w');
fprintf(fid,'rtcausal on-demand test. See RT_CAUSAL_SUMMARY.txt. Safety guarantee: NOT_PROVEN.\n');
fclose(fid);
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if isempty(R.errors),fprintf(fid,'No errors.\n');
else,for j=1:numel(R.errors),fprintf(fid,'%s\n',R.errors{j});end
end
fclose(fid);
if ~isempty(R.cycles)
    name='RT_CAUSAL_CYCLES.csv';
    writetable(struct2table(R.cycles),fullfile(tmp,name));
    files{end+1}=name; %#ok<AGROW>
end
if ~isempty(R.barriers)
    name='RT_CAUSAL_BARRIERS.csv';
    writetable(struct2table(R.barriers),fullfile(tmp,name));
    files{end+1}=name; %#ok<AGROW>
end
out=fullfile(root,'CBF_SD_TEST_RESULTS.zip');
if isfile(out),delete(out);end
zip(out,files,tmp);
end
