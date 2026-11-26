function zipPath=CBF_ID3_SAMPLED_THRUST_TEST_REV44(opt)
% REV44: baseline vs lazy whole-period proof; read-only snapshot experiment.
% E_FAST/F_FAST: reject invalid initial stages before reach computation;
% attempt one whole interval, subdivide only if enclosure fails.
% The final candidate must satisfy the same interval certificate as E/F.
% No claim that coarse search failure proves infeasibility.
% Exact ideal quadratic thrust hold; interval lower-bound certification.
% Run after stopped REV38 GUI. Optional struct fields documented below.
% opt.thrustBounds=[0;20], opt.bodyDeg=40, opt.cableDeg=60,
% opt.bodyLambda=[2;2], opt.cableLambda=[1;2;8;16],
% opt.stride=80, opt.maxIter=4, opt.substeps=2, opt.refineSubsteps=4.
% Candidate search is NOT an exhaustive feasibility test. Only a successful
% outward-enclosed certificate for the FINAL chosen held input counts.
if nargin==0,opt=struct();end
outDir=fileparts(mfilename('fullpath'));zipPath=fullfile(outDir,'CBF_ID3_SAMPLED_THRUST_REV44_RESULTS.zip');
summary="REV44 ID3 SAMPLED HOLD / THRUST KERNEL SNAPSHOT TEST"+newline;
act={};replay={};results={};details={};unit={};snap={};
try
    if ~evalin('base','exist(''app'',''var'')'),error('SD44:App','Run GUI, stop flight, keep app.');end
    app=evalin('base','app');
    if ~isa(app,'SimExp')||~isvalid(app)||~strcmp(string(app.cha),"s"),error('SD44:App','Stopped SimExp required');end
    hlc=app.agent(1).controller.hlc_suspended;cfg=hlc.cbfConfig;
    if cfg.cbfId~=3,error('SD44:ID','ID3 required');end
    opt=defaults(opt,cfg);
    [summary,act,replay,results,details,unit,snap]=audit(app.logger,hlc,cfg,opt,summary);
catch ME
    summary=summary+"STATUS=FAILED"+newline+string(getReport(ME,'extended','hyperlinks','off'))+newline;
end
% Single persistent output, overwrite atomically; temp ZIP cleaned on error.
tmp=[tempname(outDir),'.zip'];clean=onCleanup(@()delete_if_exists(tmp));
fos=java.io.FileOutputStream(tmp);zos=java.util.zip.ZipOutputStream(fos);zc=onCleanup(@()close_zip(zos));
put_entry(zos,'SUMMARY.txt',char(summary));
entries={'ACTUATOR.csv',act;'REPLAY.csv',replay;'CANDIDATES.csv',results;'CERTIFICATE_ROWS.csv',details;'UNIT.csv',unit;'SNAPSHOTS.csv',snap};
for i=1:size(entries,1),if ~isempty(entries{i,2}),put_entry(zos,entries{i,1},join_lines(entries{i,2}));end,end
zos.close();clear zc
[ok,msg]=movefile(tmp,zipPath,'f');if ~ok,error('SD44:ZIP','%s',msg);end
clear clean
fprintf('[SD44] %s\n%s\n',zipPath,char(summary));
end

function o=defaults(o,cfg)
d=struct('thrustBounds',cfg.paperThrustBounds(:),'bodyDeg',40,'cableDeg',60,'bodyLambda',[2;2],'cableLambda',[1;2;8;16], ...
    'stride',80,'maxIter',4,'substeps',2,'refineSubsteps',4);
if ~isstruct(o)||~isscalar(o),error('SD44:Options','Expected scalar options struct');end
n=fieldnames(o);if any(~ismember(n,fieldnames(d))),error('SD44:Options','Unknown option');end
for j=fieldnames(d).',k=j{1};if ~isfield(o,k),o.(k)=d.(k);end,end
o.thrustBounds=double(o.thrustBounds(:));o.bodyLambda=double(o.bodyLambda(:));o.cableLambda=double(o.cableLambda(:));
validateattributes(o.thrustBounds,{'double'},{'numel',2,'real','finite','nonnegative'});assert(diff(o.thrustBounds)>0,'SD44:Bounds');
validateattributes(o.bodyDeg,{'numeric'},{'scalar','real','finite','>',0,'<',90});validateattributes(o.cableDeg,{'numeric'},{'scalar','real','finite','>',0,'<',90});
validateattributes(o.bodyLambda,{'double'},{'numel',2,'positive','finite'});validateattributes(o.cableLambda,{'double'},{'numel',4,'positive','finite'});
for k={'stride','maxIter','substeps','refineSubsteps'},validateattributes(o.(k{1}),{'numeric'},{'scalar','integer','positive','finite'});end
% Upper enclosures of exact cos(angle*pi/180), no empirical angle margin.
a=cos_interval(o.bodyDeg);b=cos_interval(o.cableDeg);o.cosBody=a(2);o.cosCable=b(2);
end

function [summary,act,replay,results,details,unit,snap]=audit(logger,hlc,cfg,o,summary)
H=double(hlc.param.dt);assert(isfinite(H)&&H>0,'SD44:dt');
D=logger.Data;L=D.agent(1);n=min([logger.k,numel(D.t),numel(D.phase),numel(L.controller.result),numel(L.estimator.result),numel(L.sensor.result)]);
P0=ascol(hlc.self.parameter.get(["mass","jx","jy","jz","gravity","loadmass","cableL"]),7);
summary=summary+sprintf('H=%.17g bounds=%s bodyDeg=%.17g cableDeg=%.17g bodyLambda=%s cableLambda=%s obsLambda=%s\n',H,mat2str(o.thrustBounds.',17),o.bodyDeg,o.cableDeg,mat2str(o.bodyLambda.'),mat2str(o.cableLambda.'),mat2str(cfg.paperLambda(:).'));
summary=summary+sprintf('cos_threshold_upper_body=%.17g cable=%.17g\n',o.cosBody,o.cosCable);
summary=summary+sprintf('legacy_bounds=%s legacy_caps=[%.17g %.17g] scale=%s (unchanged objective weight, NOT a physical bound)\n',mat2str(cfg.paperThrustBounds(:).'),cfg.paperRateMaxNps,cfg.paperAccelMaxNps2,mat2str(cfg.paperControlScale(:).'));
summary=summary+"Certificate scope: ideal static obstacle polynomial dynamics from the recorded state with exact normalization enclosed along with the runtime rounded normalization, fixed parameters, exact held input. Outward interval elementary arithmetic assumes IEEE binary64 round-to-nearest; no verified physical estimation/ODE solver errors, no recursive joint CBF feasibility. Angle thresholds are upper interval enclosures of real cosine; geometry and other parameters are interpreted as stored numbers. Strict interval checks may reject exact boundary points. No fallback deemed safe."+newline;
summary=summary+"Reconstructed P from live fixed parameters and logged mL; P(8:9)=0 per archived HLC source; replay gate checks input and stage agreement. RK4 endpoint errors are diagnostic, not certified model-error bounds."+newline;
summary=summary+"K_T: one-cycle stopping invariant subset, no fixed q/nu cap, not maximum kernel, not physical bandwidth. Snapshot counterfactuals only; unchanged raw HLC objective, no new closed-loop flight. A legacy recorded, B new thrust CT obstacle, C new thrust CT+angles, D full final-stage bound, E integral contraction, F integral nonnegative (also psi_r at sample >=0). D/E/F use candidate-specific M, repeated QP is search only; final candidate revalidated, search failure NOT proof of infeasibility."+newline;
unit=unit_checks(H,o,cfg,P0);summary=summary+"UNIT_CHECKS=PASS (arithmetic and analytic regression, not full certification)"+newline;
act={strjoin({'t','T','q','nu_logged','legacy_exact_lo','legacy_exact_hi','legacy_adm_lo','legacy_adm_hi','hold_lo','hold_hi','K_lo','K_hi','q_lo','q_hi','K_valid','logged_in_new','stop_witness','Tmin_actual','Tmax_actual','h_logged_min','lower_logged_min','obs_count'},',')};
replay={'t,status,raw_error,hold_error,input_error,stage_error,endpoint_RK4_2_error,endpoint_RK4_16_error,endpoint_RK4_64_error'};
results={'t,case,method,status,certified,iterations,ms,nu,tauX,tauY,tauZ,Tend,qEnd,Tmin,Tmax,initial_lower_min,sample_final_min,cert_margin_min,nominal_cost,grid_h_min,grid_body_h_min,grid_cable_h_min,grid_lower_min,grid_final_min,affine_ms,seed_qp_ms,core_ms'};
details={strjoin([{'t','case','method','row','relative_degree','initial_h_lo','initial_lower_min','final_sample_lo','final_sample_hi','M','required_upper','margin_lo','substeps','reach_boxes','bound_compute_ms'},vecnames('stage_lo_0to4',5),vecnames('stage_hi_0to4',5),vecnames('grid_min_0to4',5),{'reason'}],',')};
snap={strjoin([{'t'},vecnames('x',19),vecnames('P',7),vecnames('raw',4),vecnames('sent',28),vecnames('hold',5),{'obs_json'}],',')};
S={};skips=0;replayFail=0;allneg=0;actK=0;actIn=0;maxNu=0;lastT=NaN;
for k=1:n-1
    if char(D.phase(k))~='f',continue;end
    c=L.controller.result{k};if ~isstruct(c)||~isfield(c,'cbf')||~isstruct(c.cbf)||~isfield(c.cbf,'paperHold')||isempty(c.cbf.paperHold),continue;end
    h=c.cbf.paperHold;if ~h.active||abs(h.dt-H)>1e-10,skips=skips+1;continue;end
    sr=L.sensor.result{k};if ~isstruct(sr)||~isfield(sr,'cbfObservedWorld')||~isfield(sr.cbfObservedWorld,'parts'),skips=skips+1;continue;end
    ob=sr.cbfObservedWorld;if isfield(ob,'dynamicIndices')&&~isempty(ob.dynamicIndices),error('SD44:Dynamic','Only static world');end
    if ~isfield(sr,'cbfSensorTime')||abs(double(sr.cbfSensorTime)-double(D.t(k)))>1.5*H,skips=skips+1;continue;end
    ss=struct('t',double(D.t(k)),'x',ascol(c.x,19),'raw',ascol(c.tmp,4),'sent',ascol(c.xd,28),'u',ascol(c.input,4),'P',P0,'obs',ob,'hold',[h.T0;h.q0;h.nu;h.Tend;h.qEnd],'c',c.cbf,'next',nan(19,1));
    ss.P(6)=double(L.estimator.result{k}.state.mL);
    if any(~isfinite([ss.x;ss.raw;ss.P;ss.hold]))||any(ss.P([1:5,7])<=0)||ss.P(6)<0,error('SD44:Snapshot','Nonfinite or invalid state/parameters');end
    if char(D.phase(k+1))=='f'&&abs(double(D.t(k+1))-ss.t-H)<1e-8
        cc=L.controller.result{k+1};if isstruct(cc)&&isfield(cc,'x'),ss.next=ascol(cc.x,19);end
    end
    if ~isfield(c.cbf,'paperCurrentH')||isempty(c.cbf.paperCurrentH),hm=Inf;else,hm=min(c.cbf.paperCurrentH(:));end
    ss.hm=hm;ss.lm=Inf;if isfield(c.cbf,'paperStageValues')&&~isempty(c.cbf.paperStageValues),ss.lm=min(c.cbf.paperStageValues(:));end;allneg=allneg+(hm<0);T=h.T0;q=h.q0;nu=h.nu;
    [lo,hi,kl,kh,ok,qlo,qhi]=kernel(T,q,H,o.thrustBounds);[mn,mx]=thrust_extrema(T,q,nu,H);
    old1=as2(c.cbf,'paperThrustInterval');old2=as2(c.cbf,'paperAdmissibleNuAfterViability');
    in=ok&&nu>=kl&&nu<=kh;stop=ok&&(-q/H>=kl-1e-8)&&(-q/H<=kh+1e-8);
    actK=actK+ok;actIn=actIn+in;maxNu=max(maxNu,abs(nu));
    act{end+1}=numline([ss.t;T;q;nu;old1;old2;lo;hi;kl;kh;qlo;qhi;ok;in;stop;mn;mx;hm;ss.lm;numel(ob.parts)]); %#ok<AGROW>
    S{end+1}=ss;lastT=ss.t; %#ok<AGROW>
end
if isempty(S),error('SD44:NoHold','No ID3 hold snapshots found');end
summary=summary+sprintf('logged_intervals=%d skipped=%d last_t=%.9g negative_logged_h=%d K_initial=%d logged_nu_inside_new_K=%d max_logged_abs_nu=%.9g lower_negative=%d\n',numel(S),skips,lastT,allneg,actK,actIn,maxNu,sum(cellfun(@(s)s.lm<0,S)));
% Sparse uniform selection plus ALL h-negative records and near-boundary
% neighbours, and the first encounter. No result-dependent cherry picking.
hv=cellfun(@(s)s.hm,S);idx=unique([1:o.stride:numel(S),numel(S),find(hv<0),find(hv<0)-1,find(hv<0)+1]);
first=find(isfinite(hv),1);if ~isempty(first),idx=unique([idx,first,max(1,first-1)]);end
lv=cellfun(@(s)s.lm,S);first=find(lv<0,1);if ~isempty(first),idx=unique([idx,first-2:first+2]);end
[~,ord]=sort(hv);idx=unique([idx,ord(1:min(8,numel(ord)))]);idx=idx(idx>=1&idx<=numel(S));
summary=summary+sprintf('selected=%d stride=%d maxIter=%d substeps=%d refine=%d\n',numel(idx),o.stride,o.maxIter,o.substeps,o.refineSubsteps);
methods={'A_LOGGED','B_CT_THRUST','C_CT_TILT','D_FULL','E_INTEGRAL','F_NONNEG','E_FAST','F_FAST'};count=zeros(8,1);certs=zeros(8,1);msSum=zeros(8,1);msMax=zeros(8,1);coreSum=zeros(8,1);coreMax=zeros(8,1);coreInTime=zeros(8,1);
for ii=1:numel(idx)
    ss=S{idx(ii)};fprintf('[SD44] snapshot %d/%d t=%.3f\n',ii,numel(idx),ss.t);
    P=physical(ss.P);G=geometry(cfg,P);x=ss.x;
    if max(abs([norm(x(1:4))-1;norm(x(14:16))-1]))>cfg.paperStateProjectionLimit,replayFail=replayFail+1;replay{end+1}=[numline(ss.t),',',csvtext('STATE_OUTSIDE_PROJECTION_LIMIT'),',',numline(nan(7,1))];continue;end
    x(1:4)=x(1:4)/norm(x(1:4));x(14:16)=x(14:16)/norm(x(14:16));y=[x;ss.hold(1:2)];Y0=normal_box(ss.x,y);tz=max(-1,min(1,ss.raw(4)));parts=ss.obs.parts;
    % Recompute recorded raw and QP (value-only copy, no live controller).
    c2=cfg;c2.paperThrustState=struct('initialized',true,'T',y(20),'q',y(21));a=struct('sensor',struct('result',struct('cbfObservedWorld',ss.obs)));
    status='OK';err=nan(7,1);
    try
        raw=nominal_hlc(hlc,ss.x,ss.sent,ss.P,false);err(1)=max(abs(raw-ss.raw));
        [u,info]=CBF_REAL_SCHEME_03_PAPER_EXTENSION(a,c2,ss.x,ss.raw,[ss.P;0;0],H);
        if logical(info.success)~=logical(ss.c.success)||~info.success,error('SD44:Replay','Legacy QP not successful or mismatched');end
        h=info.paperHold;err(2)=max(abs([h.T0;h.q0;h.nu;h.Tend;h.qEnd]-ss.hold));err(3)=max(abs(u(:)-ss.u));
        z=CBF_REAL_D2T_OBSTACLE_EVAL(y,[ss.hold(3);ss.u(2:3)],ss.u(4),P,G,parts,cfg.paperLambda(:));
        if isempty(parts),err(4)=0;else,err(4)=max(abs(z.stages(:)-ss.c.stageValues(:)));end
        for m=1:3,ns=[2,16,64];xx=project_state(propagate(ss.x,ss.u,ss.hold(1:3).',ss.P,H,ns(m)));err(4+m)=max(abs(xx-ss.next));end
        if err(1)>1e-7*(1+max(abs(ss.raw)))||err(2)>1e-6||err(3)>1e-7||err(4)>1e-6*(1+max([0;abs(ss.c.stageValues(:))])),error('SD44:Replay','Recorded raw/hold/stage mismatch');end
    catch ME,status=status_text(ME);replayFail=replayFail+1;end
    replay{end+1}=[numline(ss.t),',',csvtext(status),',',numline(err)]; %#ok<AGROW>
    snap{end+1}=[numline([ss.t;ss.x;ss.P;ss.raw;ss.sent;ss.hold]),',',csvtext(jsonencode(parts))]; %#ok<AGROW>
    if ~strcmp(status,'OK'),continue;end
    ta=tic;try,[rr,labels,lam]=affine_rows(y,tz,P,G,parts,cfg,o);catch ME,replayFail=replayFail+1;replay{end+1}=[numline(ss.t),',',csvtext(status_text(ME)),',',numline(nan(7,1))];continue;end
    affineMs=1000*toc(ta);
    [~,~,kl,kh,valid]=kernel(y(20),y(21),H,o.thrustBounds);lb=[kl;-1;-1];ub=[kh;1;1];wn=[2*(ss.raw(1)-y(20)-H*y(21))/H^2;ss.raw(2:3)];scale=cfg.paperControlScale(:);
    wA=[ss.hold(3);ss.u(2:3)];wB=[];wC=[];seedMs=NaN;
    % Independent timings: reuse an identical bound only within this method.
    for m=1:8
        rule=m;if m>=7,rule=m-2;end
        cacheW={};cacheC={};
        tt=tic;w=[];cert=false;its=0;cdata=[];required=nan(numel(lam),1);marg=required;st='NOT_RUN';
        try
            switch m
                case 1,w=wA;st='LOGGED';
                case 2
                    if ~valid,st='K_INITIAL_OUTSIDE';else,[w,st]=qp(rr.B(1:numel(parts),:),rr.b(1:numel(parts)),zeros(numel(parts),1),wn,lb,ub,scale);end;wB=w;
                case 3
                    tseed=tic;if ~valid,st='K_INITIAL_OUTSIDE';else,[w,st]=qp(rr.B,rr.b,zeros(numel(lam),1),wn,lb,ub,scale);end;seedMs=1000*toc(tseed);wC=w;
                otherwise
                    if ~valid,st='K_INITIAL_OUTSIDE';
                    elseif isempty(wC),st='CT_SEED_UNAVAILABLE';
                    else
                        w=wC;st='SEARCH_EXHAUSTED_UNCERTIFIED';
                        for it=1:o.maxIter
                            its=it;
                            if m>=7
                                [cdata,cacheW,cacheC]=fast_cached_bound(y,Y0,w,tz,P,G,parts,lam,o,H,cacheW,cacheC);
                            else
                                [cdata,cacheW,cacheC]=cached_bound(y,Y0,w,tz,P,G,parts,lam,o,H,cacheW,cacheC);
                            end
                            [required,marg,cert,why]=cert_rule(cdata,H,lam,rule,y,w,o.thrustBounds);
                            if cert,st='CERTIFIED_IDEAL_ONE_INTERVAL';break;end
                            if ~cdata.ok||~cdata.lowerOK||any(~isfinite(required)),st=why;break;end
                            % Search-only numerical cushion: row-scaled solver tolerance and
                            % current point evaluation enclosure width. Certification
                            % still uses the unmodified mathematical threshold.
                            cushion=max(cdata.a(:,2)-cdata.a(:,1),1e-9*max([abs(rr.b-required),max(abs(rr.B.*scale.'),[],2),ones(numel(lam),1)],[],2));
                            [wnew,qst]=qp(rr.B,rr.b,required+cushion,wn,lb,ub,scale);
                            if isempty(wnew),st=['SEARCH_QP_',qst];break;end
                            if isequal(wnew,w),st='SEARCH_STALLED_UNCERTIFIED';break;end
                            if it<o.maxIter,w=wnew;end % never report an unvalidated new point
                        end
                    end
            end
            if m<=3&&~isempty(w)
                [cdata,cacheW,cacheC]=cached_bound(y,Y0,w,tz,P,G,parts,lam,o,H,cacheW,cacheC);
                % Diagnose whether unchanged candidate would meet E; do not label CT as certified method.
                [required,marg,cert,why]=cert_rule(cdata,H,lam,5,y,w,o.thrustBounds);
                st=[st,'|E_CHECK:',why];
                if m==2,st=[st,'|ANGLES_NOT_ENFORCED'];end
            end
        catch ME,st=status_text(ME);cert=false;end
        ms=1000*toc(tt);msSum(m)=msSum(m)+ms;msMax(m)=max(msMax(m),ms);count(m)=count(m)+~isempty(w);certs(m)=certs(m)+cert;
        % Core timing includes shared affine evaluation and CT seed solve
        % for D/E/F and FAST; excludes raw HLC, legacy replay, grid, logging.
        addSeed=0;if m>=4,addSeed=seedMs;end
        coreMs=affineMs+addSeed+ms;coreSum(m)=coreSum(m)+coreMs;coreMax(m)=max(coreMax(m),coreMs);coreInTime(m)=coreInTime(m)+(cert&&coreMs<=1000*H);
        vals=nan(1,17);gm=nan(numel(lam),5);if ~isempty(w)
            [mn,mx]=thrust_extrema(y(20),y(21),w(1),H);Tend=y(20)+H*y(21)+0.5*H^2*w(1);qe=y(21)+H*w(1);
            try,[gg,gm]=grid_check(y,w,tz,ss.P,P,G,parts,cfg,o,H);catch,gg=nan(5,1);end
            ini=NaN;fin=NaN;cm=NaN;
            if ~isempty(cdata)&&~isempty(cdata.lowerMin)&&isfield(cdata,'stages'),ini=min(cdata.lowerMin);fin=min(cdata.a(:,1));end
            if ~isempty(cdata)&&cdata.ok,cm=min(marg);end
            vals=[w(:).',tz,Tend,qe,mn,mx,ini,fin,cm,sum(((w-wn)./scale).^2),gg(:).'];
        end
        results{end+1}=[numline(ss.t),',',csvtext('RECORDED'),',',csvtext(methods{m}),',',csvtext(st),',',numline([cert;its;ms;vals(:);affineMs;addSeed;coreMs])]; %#ok<AGROW>
        if ~isempty(cdata)&&~cdata.ok
            details{end+1}=[numline(ss.t),',',csvtext('RECORDED'),',',csvtext(methods{m}),',',csvtext('BOUND_FAILURE'),',',numline([nan(8,1);cdata.substeps;cdata.boxes;cdata.ms;nan(15,1)]),',',csvtext(cdata.reason)]; %#ok<AGROW>
        elseif ~isempty(cdata)&&cdata.ok
            for r=1:numel(lam)
                stg=nan(5,2);stg(1:size(cdata.stages{r},1),:)=cdata.stages{r};
                details{end+1}=[numline(ss.t),',',csvtext('RECORDED'),',',csvtext(methods{m}),',',csvtext(labels{r}),',',numline([numel(lam{r});cdata.h(r,1);cdata.lowerMin(r);cdata.a(r,:).';cdata.M(r);required(r);marg(r);cdata.substeps;cdata.boxes;cdata.ms;stg(:);gm(r,:).']),',',csvtext(st)]; %#ok<AGROW>
            end
        end
    end
end
summary=summary+sprintf('replay_failed=%d\n',replayFail);
summary=summary+"REV44: A-F unchanged baseline search; E_FAST/F_FAST use whole-period first, refine ONLY when reach enclosure fails, and stop immediately at invalid initial lower stages. Coarser M may increase QP cost or search failures; no tolerance relaxation. All final inputs independently checked. Fast substeps=0 denotes early rejection (M unavailable)."+newline;
summary=summary+"Timing: core_ms includes shared affine-row evaluation plus CT seed QP for m>=4 and independent search; excludes raw HLC, sensor, replay, grid and logger; not full control-cycle time. All candidates share recorded parameters and raw HLC. No guarantee of deadlines."+newline;
for m=1:8,summary=summary+sprintf('%s candidates=%d E_or_own_certificates=%d mean_independent_search_ms_per_selected=%.6g max_search_ms=%.6g mean_core_ms=%.6g max_core_ms=%.6g certified_core_within_H=%d\n',methods{m},count(m),certs(m),msSum(m)/numel(idx),msMax(m),coreSum(m)/numel(idx),coreMax(m),coreInTime(m));end
summary=summary+"STATUS=COMPLETE_SNAPSHOT_DIAGNOSTIC_NOT_CLOSED_LOOP_OR_FLIGHT_CERTIFICATION"+newline;
end

function Y=normal_box(raw,y)
% Enclose exact real normalization of the recorded q and pT as well as the
% binary64 normalized point used by the legacy generated row evaluation.
Y=iv(y);for I={1:4,14:16}
    k=I{1};a=iv(raw(k));sq=isq(a);b=sq(1,:);for j=2:numel(k),b=ia(b,sq(j,:));end
    if b(1)<=0,error('SD44:Normalize','Zero or uncertified norm');end
    b=out(sqrt(b(1)),sqrt(b(2)));a=id(a,b);
    Y(k,:)=[min(a(:,1),Y(k,1)),max(a(:,2),Y(k,2))];
end
end
function [lo,hi,kl,kh,ok,ql,qh]=kernel(T,q,H,B)
% Exact real-arithmetic scalar formulas; numeric candidate rechecked with intervals.
a=T-B(1);b=B(2)-T;ql=-2*a/H;qh=2*b/H;lo=Inf;hi=-Inf;kl=Inf;kh=-Inf;ok=false;
if a<0||b<0,return;end
lo=2*(-a-q*H)/H^2;hi=2*(b-q*H)/H^2;
if q<0
    if a==0,lo=Inf;elseif -2*a/q<H,lo=q^2/(2*a);end
elseif q>0
    if b==0,hi=-Inf;elseif 2*b/q<H,hi=-q^2/(2*b);end
end
kl=max(lo,-a/H^2-1.5*q/H);kh=min(hi,b/H^2-1.5*q/H);
ok=q>=ql&&q<=qh&&kl<=kh&&all(isfinite([kl,kh]));
end
function [mn,mx]=thrust_extrema(T,q,v,H)
a=[T,T+q*H+.5*v*H^2];if v~=0,s=-q/v;if s>0&&s<H,a(end+1)=T+q*s+.5*v*s^2;end,end
mn=min(a);mx=max(a);
end
function ok=actuator_cert(y,w,H,B)
% Quadratic Bezier control points all inside the thrust box, endpoint K_T.
% More restrictive only for initial states outside K_T (already excluded).
t=iv(y(20));q=iv(y(21));v=iv(w(1));hh=iv(H);half=iv(.5);
b0=t;b1=ia(t,im(half,im(hh,q)));b2=ia(ia(t,im(hh,q)),im(half,im(im(hh,hh),v)));qe=ia(q,im(hh,v));
% terminal stop point T_end + H q_end/2 also within box.
b3=ia(b2,im(half,im(hh,qe)));z=[b0;b1;b2;b3];ok=all(z(:,1)>=B(1)&z(:,2)<=B(2))&&all(abs(w(2:3))<=1);
end
function P=physical(p)
P=struct('m',p(1),'J',p(2:4),'gravity',p(5),'mL',p(6),'cableL',p(7),'dstx',0,'dsty',0);
end
function G=geometry(cfg,P)
G=struct('collisionMode','cable_ellipsoid','cableAxisShort',cfg.paperCableRadiusM,'cableAxisLong',max(P.cableL/2+cfg.paperCableAxialMarginM,cfg.paperCableRadiusM),'cableCenterFromLoad',.5);
end
function a=as2(s,f)
a=nan(2,1);if isfield(s,f)&&numel(s.(f))==2,v=s.(f);a=double(v(:));end
end
function [rr,labels,lam]=affine_rows(y,tz,P,G,parts,cfg,o)
lam=[repmat({cfg.paperLambda(:)},1,numel(parts)),{o.bodyLambda,o.cableLambda}];
labels=[arrayfun(@(i)sprintf('OBS_%d',i),1:numel(parts),'UniformOutput',false),{'BODY','CABLE'}];
[a,lower]=point_stages(y,zeros(3,1),tz,P,G,parts,lam,o);B=zeros(numel(lam),3);
for j=1:3,v=zeros(3,1);v(j)=1;[b,low]=point_stages(y,v,tz,P,G,parts,lam,o);B(:,j)=b-a;assert(max(abs(low-lower))<1e-7*(1+max(abs(lower))),'SD44:RelativeDegree');end
v=[31;.071;-.043];b=point_stages(y,v,tz,P,G,parts,lam,o);assert(max(abs(b-a-B*v)./(1+abs(b)))<1e-7,'SD44:Affine');
rr=struct('b',a,'B',B,'lower',lower);
end
function [a,lower,hs,full]=point_stages(y,w,tz,P,G,parts,lam,o)
if isempty(parts),obsLambda=ones(4,1);else,obsLambda=lam{1};end
z=CBF_REAL_D2T_OBSTACLE_EVAL(y,w,tz,P,G,parts,obsLambda);
a=zeros(numel(lam),1);lower=[];hs=zeros(numel(lam),1);full=cell(numel(lam),1);
for j=1:numel(parts),a(j)=z.stages(5,j);lower=[lower;z.stages(1:4,j)];hs(j)=z.stages(1,j);full{j}=z.stages(:,j);end %#ok<AGROW>
J=CBF_REAL_D2T_JET(y(1:19),y(20),y(21),w(1),[w(2:3);tz],P);X=J.stateTaylor;
b=zeros(1,5);for k=1:4,sg=[1,-1,-1,1];c=conv(X(k,:),X(k,:));b=b+sg(k)*c(1:5);end
b(1)=b(1)-o.cosBody;c=-X(16,:);c(1)=c(1)-o.cosCable;
for j=1:2,k=numel(parts)+j;if j==1,f=b;else,f=c;end;d=f(:).*factorial((0:4)');q=1;s=d(1);for r=1:numel(lam{k}),q=conv(q,[lam{k}(r),1]);s(end+1)=q*d(1:r+1);end;a(k)=s(end);lower=[lower;s(1:end-1).'];hs(k)=s(1);full{k}=s(:);end %#ok<AGROW>
end
function [w,status]=qp(B,b,req,nom,lo,hi,S)
w=[];status='EMPTY';if any(~isfinite([lo;hi;nom;S]))||any(lo>hi),return;end
if isempty(b),w=min(hi,max(lo,nom));status='OK';return;end
Az=-B.*S.';bz=b-req;sc=max([abs(bz),max(abs(Az),[],2),1e-9*ones(size(bz))],[],2);Az=Az./sc;bz=bz./sc;
if exist('quadprog','file')==0,status='QUADPROG_MISSING';return;end
persistent qo;if isempty(qo),qo=optimoptions('quadprog','Display','off');end
[z,~,e]=quadprog(eye(3),-nom./S,Az,bz,[],[],lo./S,hi./S,[],qo);
if isempty(z)||e<=0||any(~isfinite(z)),status=sprintf('EXIT_%d',e);return;end
w=S.*z(:);if any(w<lo-1e-8)||any(w>hi+1e-8)||any(b+B*w<req-1e-7),w=[];status='RESIDUAL';else,status='OK';end
end
function [c,W,C]=cached_bound(y,Y0,w,tz,P,G,parts,lam,o,H,W,C)
for j=1:numel(W),if isequal(w,W{j}),c=C{j};return;end,end
tm=tic;c=bound_candidate(y,Y0,w,tz,P,G,parts,lam,o,H,o.substeps);
% Refinement also required if coarse bound cannot establish a nonnegative lower stage.
if o.refineSubsteps>o.substeps
    d=bound_candidate(y,Y0,w,tz,P,G,parts,lam,o,H,o.refineSubsteps);
    if d.ok&&(~c.ok||sum(d.M)<sum(c.M)),c=d;end
end
c.ms=1000*toc(tm);W{end+1}=w;C{end+1}=c;
end
function [c,W,C]=fast_cached_bound(y,Y0,w,tz,P,G,parts,lam,o,H,W,C)
for j=1:numel(W),if isequal(w,W{j}),c=C{j};return;end,end
tm=tic;
% First try one complete interval. If the proof of enclosure succeeds,
% its M is valid even if the candidate needs a different QP input.
% Only refine when the enclosure itself fails, not merely to tighten M.
steps=unique([1,o.substeps,o.refineSubsteps]);
for ns=steps
    c=bound_candidate(y,Y0,w,tz,P,G,parts,lam,o,H,ns,true);
    if c.ok,break;end
end
c.ms=1000*toc(tm);W{end+1}=w;C{end+1}=c;
end
function c=bound_candidate(y,Y0,w,tz,P,G,parts,lam,o,H,ns,fast)
if nargin<12,fast=false;end
c=struct('ok',false,'lowerOK',false,'lowerMin',[],'M',[],'a',[],'z',[],'h',[],'substeps',ns,'boxes',0,'reason','NOT_RUN','early',false);
try
    U=iv([w(:);tz]);Y=Y0;
    N=5;if fast,N=4;end
    J=statejet(Y,U,P,N);[st,~]=barrier_jets(J,P,G,parts,lam,o);
    nr=numel(lam);c.h=zeros(nr,2);c.a=c.h;c.z=c.h;c.lowerMin=zeros(nr,1);c.M=zeros(nr,1);
    for j=1:nr,s=st{j};c.h(j,:)=s(1,:);c.a(j,:)=s(end,:);c.z(j,:)=s(end-1,:);c.lowerMin(j)=min(s(1:end-1,1));end
    c.lowerOK=all(c.lowerMin>=0);c.stages=st;
    if fast&&~c.lowerOK
        c.ok=true;c.early=true;c.substeps=0;c.M(:)=NaN;c.reason='INITIAL_LOWER_OUTSIDE_OR_UNCERTAIN';return
    end
    % Each subinterval length is enclosed as exact H/ns. Picard enclosure
    % and Taylor remainder use the same upper time interval; endpoint update
    % uses the exact enclosed step, not an unvalidated numerical integrator.
    dt=id(iv(H),iv(ns));
    for k=1:ns
        B=picard(Y,U,P,dt);
        % The taut-cable model requires positive tension throughout.
        Z=initjet(B,0);[~,rad]=rhsjet(Z,U,P);if rad(1,2)>=0,error('SD44:Tension','Taut-cable domain not enclosed');end
        JB=statejet(B,U,P,5);[~,der]=barrier_jets(JB,P,G,parts,lam,o);
        c.M=max(c.M,max(0,-der(:,1)));c.boxes=c.boxes+1;
        if k<ns
            J0=statejet(Y,U,P,4);endY=Y;pow=iv(1);
            for r=1:5,pow=im(pow,dt);if r<=4,Q=coeff(J0,r);else,Q=coeff(JB,r);end;endY=ia(endY,im(Q,pow));end
            % Intersection with validated Picard enclosure at endpoint.
            Y=[max(endY(:,1),B(:,1)),min(endY(:,2),B(:,2))];if any(Y(:,1)>Y(:,2)),error('SD44:Endpoint','Empty validated intersection');end
        end
    end
    c.ok=true;c.reason='OK';
catch ME,c.reason=status_text(ME);end
end
function [req,margin,ok,why]=cert_rule(c,H,lam,m,y,w,B)
n=numel(lam);req=nan(n,1);margin=req;ok=false;why='BOUND_FAILED';if ~c.ok,why=c.reason;return;end
if c.early,why='INITIAL_LOWER_OUTSIDE_OR_UNCERTAIN';return;end
for j=1:n
    if m==4,r=im(iv(c.M(j)),iv(H));else
        [I0,I1]=weights(H,lam{j}(end));r=im(iv(c.M(j)),id(I1,I0));
        if m==6,r=isub(r,id(c.z(j,:),I0));r=[max(0,r(1)),max(0,r(2))];end
    end
    req(j)=r(2);marg=isub(c.a(j,:),iv(req(j)));margin(j)=marg(1);
end
if ~c.lowerOK,why='INITIAL_LOWER_OUTSIDE_OR_UNCERTAIN';elseif ~actuator_cert(y,w,H,B),why='ACTUATOR_CERT_FAILED';elseif any(margin<0),why='ROW_NOT_CERTIFIED';else,ok=true;why='PASS';end
end
function c=cos_interval(deg)
% Known enclosure of mathematical pi, then Taylor alternating remainder.
% 0 < deg < 90; remainder of 20 even powers bounded by next magnitude.
p=out(pi,pi);x=id(im(iv(deg),p),iv(180));x2=isq(x);c=iv(1);v=iv(1);
for k=1:20,v=id(im(v,x2),iv((2*k-1)*(2*k)));if mod(k,2)==1,c=isub(c,v);else,c=ia(c,v);end,end
v=id(im(v,x2),iv(41*42));c=ia(c,[-v(2),0]);
end
function [I0,I1]=weights(H,l)
% Positive series, interval-rounded with geometric tail. No exp subtraction.
z=im(iv(H),iv(l));N=max(24,ceil(2*z(2))+8);if N>256,error('SD44:Weight','lambda*H too large for this test');end
term=iv(1);a=term;b=iv(.5);
for k=1:N,term=id(im(term,z),iv(k));a=ia(a,id(term,iv(k+1)));b=ia(b,id(term,iv(k+2)));end
next=id(im(term,z),iv(N+1));rat=id(z,iv(N+2));tail=id(next,isub(iv(1),rat));a=ia(a,[0,max(0,tail(2))]);b=ia(b,[0,max(0,tail(2))]);I0=im(iv(H),a);I1=im(im(iv(H),iv(H)),b);
end
function B=picard(Y,U,P,dt)
% Strict containment => validated local existence and enclosure for smooth polynomial f.
f=rhsjet(initjet(Y,0),U,P);F=coeff(f,0);span=[0,dt(2)];box=ia(Y,im(F,span));B=inflate([min(Y(:,1),box(:,1)),max(Y(:,2),box(:,2))]);
for j=1:14
    F=coeff(rhsjet(initjet(B,0),U,P),0);box=ia(Y,im(F,span));
    % Enlarge ONLY coordinates that fail strict inclusion. The enclosure
    % acceptance criterion remains unchanged and is checked for ALL states.
    bad=~(box(:,1)>B(:,1)&box(:,2)<B(:,2));
    if ~any(bad),return;end
    B(bad,:)=inflate([min(B(bad,1),box(bad,1)),max(B(bad,2),box(bad,2))]);
end
error('SD44:Picard','Reach enclosure did not close');
end
function b=inflate(a)
% This expansion affects only proposed enclosure, never the proof criterion.
d=max(1e-12,.15*(a(:,2)-a(:,1)));b=[a(:,1)-d,a(:,2)+d];if any(~isfinite(b(:))),error('SD44:Reach','Nonfinite');end
end
function J=statejet(Y,U,P,N)
J=initjet(Y,N);
for k=1:N
    A=cellfun(@(j)j(1:k,:),J,'UniformOutput',false);F=rhsjet(A,U,P);
    for i=1:21,J{i}(k+1,:)=id(F{i}(k,:),iv(k));end
end
end
function J=initjet(Y,N)
J=cell(size(Y,1),1);for i=1:numel(J),J{i}=[Y(i,:);zeros(N,2)];end
end
function C=coeff(J,k)
C=zeros(numel(J),2);for i=1:numel(J),C(i,:)=J{i}(k+1,:);end
end
function [F,rad]=rhsjet(X,U,P)
% Exact polynomial ID3 ideal dynamics, ordinary Taylor coefficients.
q=X(1:4);w=X(5:7);p=X(14:16);wl=X(17:19);T=X{20};N=size(T,1)-1;
b=bodyjet(q);vt=jcross(wl,p);Jw=cell(3,1);for j=1:3,Jw{j}=js(w{j},iv(P.J(j)));end
c=jcross(w,Jw);d=jcross(q(2:4),w);F=cell(21,1);F{1}=js(jdot(q(2:4),w),iv(-.5));
for j=1:3,F{j+1}=js(ia(jm(q{1},w{j}),d{j}),iv(.5));F{4+j}=js(isub(jconst(U(j+1,:),N),c{j}),id(iv(1),iv(P.J(j))));end
ml=ia(iv(P.m),iv(P.mL));mL=im(iv(P.m),iv(P.cableL));rad=js(isub(jm(T,jdot(p,b)),js(jdot(vt,vt,true),mL)),id(iv(1),ml));
cp=jcross(p,b);
for j=1:3,F{7+j}=X{10+j};F{10+j}=jm(rad,p{j});F{13+j}=vt{j};F{16+j}=js(jm(T,cp{j}),id(iv(-1),mL));end
F{13}=isub(F{13},jconst(iv(P.gravity),N));F{20}=X{21};F{21}=jconst(U(1,:),N);
end
function b=bodyjet(q)
b={js(ia(jm(q{2},q{4}),jm(q{1},q{3})),iv(2));js(isub(jm(q{3},q{4}),jm(q{1},q{2})),iv(2));ia(isub(isub(jm(q{1},q{1},true),jm(q{2},q{2},true)),jm(q{3},q{3},true)),jm(q{4},q{4},true))};
end
function [st,der]=barrier_jets(X,P,G,parts,lam,o)
N=size(X{1},1)-1;h=cell(numel(lam),1);p=X(14:16);C=cell(3,1);
for j=1:3,C{j}=isub(X{7+j},js(p{j},im(iv(G.cableCenterFromLoad),iv(P.cableL))));end
as=isq(iv(G.cableAxisShort));delta=isub(isq(iv(G.cableAxisLong)),as);
for i=1:numel(parts)
    [oc,a,R]=obs_shape(parts(i));D=cell(3,1);for j=1:3,D{j}=isub(C{j},jconst(iv(oc(j)),N));end
    r2=jdot(D,D,true);r=jsqrt(r2);inv=jinv(r2);pd=jdot(p,D);
    supC=jsqrt(ia(jconst(as,N),js(jm(jm(pd,pd,true),inv),delta)));
    E=cell(3,1);for j=1:3,E{j}=zeros(N+1,2);for k=1:3,E{j}=ia(E{j},js(D{k},iv(R(k,j))));end;E{j}=js(E{j},iv(a(j)));end
    supO=jsqrt(jm(jdot(E,E,true),inv));h{i}=isub(isub(r,supC),supO);
end
b=bodyjet(X(1:4));h{end-1}=isub(b{3},jconst(iv(o.cosBody),N));h{end}=isub(-flip(X{16},2),jconst(iv(o.cosCable),N));
st=cell(size(h));der=zeros(numel(h),2);
for i=1:numel(h)
    z=h{i};s=z(1,:);
    for j=1:numel(lam{i}),k=size(z,1)-1;z=ia(im(z(2:end,:),iv((1:k)')),js(z(1:end-1,:),iv(lam{i}(j))));s=[s;z(1,:)];end %#ok<AGROW>
    st{i}=s;if size(z,1)>=2,der(i,:)=z(2,:);else,der(i,:)=NaN;end
end
end
function [c,a,R]=obs_shape(p)
c=double(p.center(:));kind='ellipsoid';if isfield(p,'kind'),kind=char(p.kind);end
switch kind
    case 'sphere',if isfield(p,'params'),a=p.params.radius;else,a=p.radius;end;a=repmat(a,3,1);R=eye(3);
    case 'ellipsoid',if isfield(p,'params'),a=p.params.axes;else,a=p.axes;end;if isfield(p,'R'),R=double(p.R);else,R=eye(3);end
    otherwise,error('SD44:Obstacle','Unsupported type');
end
a=double(a(:));if numel(c)~=3||numel(a)~=3||any(a<=0)||~isequal(size(R),[3,3])||any(~isfinite([c;a;R(:)])),error('SD44:Shape','Invalid geometry');end
end
% ---- scalar interval arithmetic; arrays are N-by-2 [lower, upper] ----
function r=iv(x),x=double(x(:));r=[x,x];end
function r=out(l,h)
if any(~isfinite([l(:);h(:)]))||any(l(:)>h(:)),error('SD44:Interval','Invalid interval arithmetic');end
% One outward ulp at the computed value (at least realmin*eps relative
% to normal scale; avoids relying on exact directed hardware rounding).
r=[l-eps(max(abs(l),realmin)),h+eps(max(abs(h),realmin))];
if any(~isfinite(r(:))),error('SD44:Interval','Overflow');end
end
function r=ia(a,b)
l=a(:,1)+b(:,1);h=a(:,2)+b(:,2);r=out(l,h);
% All ia callers supply the same row counts. Exact zero is identity.
z=all(a==0,2);r(z,:)=b(z,:);z=all(b==0,2);r(z,:)=a(z,:);
end
function r=isub(a,b),r=ia(a,-flip(b,2));end
function r=im(a,b)
c=[a(:,1).*b(:,1),a(:,1).*b(:,2),a(:,2).*b(:,1),a(:,2).*b(:,2)];r=out(min(c,[],2),max(c,[],2));z=all(a==0,2)|all(b==0,2);r(z,:)=0;
end
function r=isq(a)
x=a(:,1).*a(:,1);y=a(:,2).*a(:,2);lo=min(x,y);lo(a(:,1)<=0&a(:,2)>=0)=0;r=out(lo,max(x,y));r(:,1)=max(r(:,1),0);r(all(a==0,2),:)=0;
end
function r=id(a,b)
if any(b(:,1)<=0&b(:,2)>=0),error('SD44:Division','Interval denominator crosses zero');end
r=im(a,out(1./b(:,2),1./b(:,1)));
end
function r=jconst(a,N),r=[a;zeros(N,2)];end
function r=js(a,s),r=im(a,s);end
function r=jm(a,b,same)
if nargin<3,same=false;end
N=size(a,1);r=zeros(N,2);
for k=1:N
    c=im(a(1:k,:),b(k:-1:1,:));if same&&mod(k,2)==1,j=(k+1)/2;c(j,:)=isq(a(j,:));end
    z=c(1,:);for j=2:k,z=ia(z,c(j,:));end;r(k,:)=z;
end
end
function r=jdot(a,b,same)
if nargin<3,same=false;end
r=jm(a{1},b{1},same);for j=2:numel(a),r=ia(r,jm(a{j},b{j},same));end
end
function r=jcross(a,b)
r={isub(jm(a{2},b{3}),jm(a{3},b{2}));isub(jm(a{3},b{1}),jm(a{1},b{3}));isub(jm(a{1},b{2}),jm(a{2},b{1}))};
end
function r=jsqrt(a)
if a(1,1)<=0,error('SD44:Sqrt','Unverified positive square root');end
N=size(a,1);r=zeros(N,2);r(1,:)=out(sqrt(a(1,1)),sqrt(a(1,2)));
for k=2:N,s=[0,0];for j=2:k-1,s=ia(s,im(r(j,:),r(k-j+1,:)));end;r(k,:)=id(isub(a(k,:),s),im(iv(2),r(1,:)));end
end
function r=jinv(a)
N=size(a,1);r=zeros(N,2);r(1,:)=id(iv(1),a(1,:));
for k=2:N,s=[0,0];for j=2:k,s=ia(s,im(a(j,:),r(k-j+1,:)));end;r(k,:)=id(-flip(s,2),a(1,:));end
end
function [g,gm]=grid_check(y,w,tz,p,P,G,parts,cfg,o,H)
% NONCERTIFIED numerical cross-check; never supplies M or proof.
lam=[repmat({cfg.paperLambda(:)},1,numel(parts)),{o.bodyLambda,o.cableLambda}];g=inf(5,1);gm=nan(numel(lam),5);x=y(1:19);h=H/16;u=[y(20);w(2:3);tz];
for j=0:16,s=j*h;Y=[x;y(20)+s*y(21)+.5*s^2*w(1);y(21)+s*w(1)];[a,l,hs,full]=point_stages(Y,w,tz,P,G,parts,lam,o);for k=1:numel(full),m=numel(full{k});if j==0,gm(k,1:m)=full{k}(:).';else,gm(k,1:m)=min(gm(k,1:m),full{k}(:).');end,end;g=min(g,[min([Inf;hs(1:numel(parts))]);hs(end-1:end);min(l);min(a)]);if j<16,x=project_state(propagate(x,u,[Y(20),Y(21),w(1)],p,h,1));end,end
end
function rows=unit_checks(H,o,cfg,p)
rows={'test,value,pass'};
assert(eps(0)>0 && realmin/2>0 && 1+eps(1)>1 && 1+eps(1)/2==1,'SD44:IEEE');
rows{end+1}='BINARY64_BASIC_SANITY,1,1';
for d=[1,20,40,60,89],a=cos_interval(d);assert(cosd(d)>=a(1)-1e-15&&cosd(d)<=a(2)+1e-15,'SD44:Cosine');rows{end+1}=sprintf('COS_INTERVAL_%d,%.17g,1',d,a(2)-a(1));end
% Deterministic stop witness and endpoints for varying thrust bounds and periods.
n=0;worst=0;
for B={[0;20],[2;9],[0;7],[3;3.5]}
 for dt=[H,H/2,2*H]
  for a=linspace(.01,.99,11),T=B{1}(1)+a*diff(B{1});for qf=linspace(-.99,.99,9),if qf>=0,q=qf*2*(B{1}(2)-T)/dt;else,q=qf*2*(T-B{1}(1))/dt;end
    [~,~,l,u,ok]=kernel(T,q,dt,B{1});v=-q/dt;assert(ok&&v>=l-1e-8&&v<=u+1e-8,'SD44:KernelWitness');
    for f=[0,.25,.5,.75,1],w=l+f*(u-l);[mn,mx]=thrust_extrema(T,q,w,dt);T2=T+dt*q+.5*dt^2*w;q2=q+dt*w;vi=max([B{1}(1)-mn,mx-B{1}(2),-2*(T2-B{1}(1))/dt-q2,q2-2*(B{1}(2)-T2)/dt,0]);worst=max(worst,vi);assert(vi<1e-7,'SD44:KernelEndpoint');n=n+1;end
  end,end
 end
end
rows{end+1}=sprintf('KERNEL_INTERVAL_ENDPOINTS,%d,1',n);rows{end+1}=sprintf('KERNEL_MAX_NUMERIC_RESIDUAL,%.17g,1',worst);
for l=[.01,2,16,32], [a,b]=weights(H,l);r=id(b,a);ref=H/(-expm1(-l*H))-1/l;assert(ref>=r(1)-1e-10&&ref<=r(2)+1e-10,'SD44:WeightCheck');rows{end+1}=sprintf('INTEGRAL_WEIGHT_%g,%.17g,1',l,mean(r));end
P=physical(p);G=geometry(cfg,P);y=[1;zeros(12,1);0;0;-1;zeros(3,1);(P.m+P.mL)*P.gravity;0];
% Quaternion/cable conventions and mixed order, empty obstacles must still yield two rows.
parts=struct('center',{},'kind',{},'radius',{});lam={o.bodyLambda,o.cableLambda};w=[.3;.01;-.02];tz=.03;
[a,lower,hs]=point_stages(y,w,tz,P,G,parts,lam,o);[rr,~,~]=affine_rows(y,tz,P,G,parts,cfg,o);assert(numel(a)==2&&numel(lower)==6&&abs(hs(1)-(1-o.cosBody))<1e-12&&abs(hs(2)-(1-o.cosCable))<1e-12,'SD44:Tilt');
J=statejet(iv(y),iv([w;tz]),P,5);[ss,~]=barrier_jets(J,P,G,parts,lam,o);for k=1:2,assert(a(k)>=ss{k}(end,1)-1e-9&&a(k)<=ss{k}(end,2)+1e-9,'SD44:JetCrossCheck');end
assert(all(isfinite(rr.B(:))),'SD44:TiltAffine');rows{end+1}='EMPTY_OBSTACLE_TILT_AND_AFFINE,2,1';
% Nonzero orientation/rate, ellipsoid shape and input; cross-check every stage.
y(1:4)=[cos(.2);sin(.2);0;0];y(5:7)=[.13;-.21;.05];y(14:16)=[.3;.1;-sqrt(.9)];y(17:19)=[.04;.1;.02];y(8:10)=[.2;1;1];y(11:13)=[.1;.5;-.1];
parts=struct('center',[1;5;2],'kind','ellipsoid','axes',[.5;.8;1.3],'R',eye(3));lam=[{cfg.paperLambda(:)},lam];z=CBF_REAL_D2T_OBSTACLE_EVAL(y,w,tz,P,G,parts,lam{1});
J=statejet(iv(y),iv([w;tz]),P,5);[ss,der]=barrier_jets(J,P,G,parts,lam,o);assert(all(z.stages(:)>=ss{1}(:,1)-1e-8*(1+abs(z.stages(:))))&&all(z.stages(:)<=ss{1}(:,2)+1e-8*(1+abs(z.stages(:)))),'SD44:ObstacleJet');assert(all(isfinite(der(:))),'SD44:Derivative');
% Check interval against point coefficients for ALL 21 states through order 4.
ref=CBF_REAL_D2T_JET(y(1:19),y(20),y(21),w(1),[w(2:3);tz],P);for k=0:4,v=coeff(J,k);r=ref.extendedTaylor(:,k+1);assert(all(r>=v(:,1)-1e-9*(1+abs(r))&r<=v(:,2)+1e-9*(1+abs(r))),'SD44:StateJet');end
rows{end+1}='OBSTACLE_AND_STATE_JET_CROSSCHECK,1,1';
% Cap change test: current angle near 30deg, same jet evaluated at new limits.
a=o;a.cosBody=cosd(20);a.cosCable=cosd(10);[~,~,h1]=point_stages(y,w,tz,P,G,parts,lam,o);[~,~,h2]=point_stages(y,w,tz,P,G,parts,lam,a);assert(abs((h2(end-1)-h1(end-1))-(o.cosBody-a.cosBody))<1e-12&&abs((h2(end)-h1(end))-(o.cosCable-a.cosCable))<1e-12,'SD44:RuntimeCaps');rows{end+1}='RUNTIME_CAP_CHANGE,1,1';
% Check actual control authority near both limits, independent of obstacles.
p0=y;p0(1:4)=[cosd(19.5);sind(19.5);0;0];p0(5:7)=0;
p0(14:16)=[sind(59);0;-cosd(59)];p0(17:19)=0;empty=struct('center',{},'kind',{},'radius',{});
[rr,~,ll]=affine_rows(p0,tz,P,G,empty,cfg,o);
assert(abs(rr.B(1,1))<1e-7&&rr.B(1,2)<0&&norm(rr.B(2,:))>1e-8,'SD44:AngleAuthority');
[~,~,h1]=point_stages(p0,w,tz,P,G,empty,ll,o);
assert(abs(h1(1)-(cosd(39)-o.cosBody))<1e-12&&abs(h1(2)-(cosd(59)-o.cosCable))<1e-12,'SD44:AngleGeometry');
rows{end+1}='NEAR_LIMIT_CONTROL_AUTHORITY,1,1';
% New fast path regression: same stage enclosures using the required order.
U=iv([w;tz]);J4=statejet(iv(y),U,P,4);J5=statejet(iv(y),U,P,5);
[s4,~]=barrier_jets(J4,P,G,parts,lam,o);[s5,~]=barrier_jets(J5,P,G,parts,lam,o);
for k=1:numel(s4),assert(isequal(s4{k},s5{k}),'SD44:InitialStageOrder');end
rows{end+1}='INITIAL_STAGE_ORDER4_EQUALS_ORDER5,1,1';
% Hover at a genuine strictly interior actuator state; test full interval.
yh=[1;zeros(12,1);0;0;-1;zeros(3,1);(P.m+P.mL)*P.gravity;0];
if yh(20)>o.thrustBounds(1)&&yh(20)<o.thrustBounds(2)
    [~,~,ll]=affine_rows(yh,0,P,G,empty,cfg,o);wh=zeros(3,1);
    ch=bound_candidate(yh,iv(yh),wh,0,P,G,empty,ll,o,H,1,true);
    [~,~,pass]=cert_rule(ch,H,ll,5,yh,wh,o.thrustBounds);
    assert(pass,'SD44:WholeIntervalHover');rows{end+1}='WHOLE_INTERVAL_HOVER,1,1';
else,rows{end+1}='WHOLE_INTERVAL_HOVER_NOT_IN_THRUST_RANGE,NaN,1';end
end

function x=ascol(x,n)
if ~isnumeric(x)||~isreal(x)||numel(x)~=n,error('CBFBasis:SnapshotShape','Unexpected numeric vector shape');end
x=double(x(:));
end
function n=vecnames(s,k)
n=arrayfun(@(j)sprintf('%s_%d',s,j),1:k,'UniformOutput',false);
end
function s=numline(v)
s=sprintf('%.17g,',v(:));s(end)=[];
end
function s=csvtext(s)
s=['"',strrep(char(s),'"','""'),'"'];
end
function s=status_text(ME)
s=[ME.identifier,': ',strrep(strrep(ME.message,newline,' '),char(13),' ')];
end
function s=join_lines(c)
s=[strjoin(c,newline),newline];
end
function put_entry(z,name,s)
e=java.util.zip.ZipEntry(name);z.putNextEntry(e);
b=unicode2native(s,'UTF-8');b=typecast(uint8(b(:)),'int8');
z.write(b,0,numel(b));z.closeEntry();
end
function close_zip(z)
try,z.close();catch,end
end
function delete_if_exists(p)
if exist(p,'file'),delete(p);end
end

function raw=nominal_hlc(hlc,x,xd,P,adjustYaw)
% Verbatim generated pure-function sequence of HLC_SUSPENDED_LOAD_CBF_REAL.do.
% The methods take all numerical inputs explicitly; no live controller do().
if adjustYaw
    q=x(1:4);yaw=atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2));
    xd(4)=yaw-atan2(sin(yaw-xd(4)),cos(yaw-xd(4)));
end
P=[P(:).',0,0];a=hlc.param;
vf=hlc.Vfd_SuspendedLoadxyDst(a.dt,x,xd.',a.F1);
vs=hlc.Vs_SuspendedLoadxyDst(x,xd.',vf,P,a.F2,a.F3,a.F4);
uf=hlc.Uf_SuspendedLoadxyDst(x,xd.',vf,P);
beta=hlc.Beta2_SuspendedLoadxyDst(x,xd.',vf,P);
b=hlc.V2_alpha2_SuspendedLoadxyDst(x,xd.',vf,vs.',P);
if any(~isfinite(beta(:))) || rcond(beta)<1e-12,error('CBFTraj:ComparisonHLCIllConditioned','Ill-conditioned HLC decoupling matrix');end
raw=[uf(1);beta\b];
if ~isreal(raw) || any(~isfinite(raw)),error('CBFTraj:ComparisonHLCNonfinite','Nonfinite or nonreal HLC result');end
end

function y=propagate(x,u,thrust,P,dt,nSub)
h=dt/nSub;y=x;
for k=1:nSub
    t=(k-1)*h;
    k1=rhs(y,at(u,thrust,t),P);
    k2=rhs(y+h*k1/2,at(u,thrust,t+h/2),P);
    k3=rhs(y+h*k2/2,at(u,thrust,t+h/2),P);
    k4=rhs(y+h*k3,at(u,thrust,t+h),P);
    y=y+h*(k1+2*k2+2*k3+k4)/6;
end
end

function uk=at(u,profile,s)
uk=u;
uk(1)=profile(1)+profile(2)*s+0.5*profile(3)*s^2;
end

function [out,radial]=rhs(x,u,p)
% Same 19-state suspended-load ideal dynamics used in V81 research.
% x=[quaternion(4); bodyOmega(3); loadP(3); loadV(3);
%    cableDirection(3); cableOmega(3)].
m=p(1);J=p(2:4);g=p(5);ml=p(6);L=p(7);
q=x(1:4);omega=x(5:7);dir=x(14:16);omegaL=x(17:19);
b3=[2*(q(2)*q(4)+q(1)*q(3)); ...
    2*(q(3)*q(4)-q(1)*q(2)); ...
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
qd=0.5*[-q(2:4)'*omega; q(1)*omega+cross(q(2:4),omega)];
wd=(u(2:4)-cross(omega,J.*omega))./J;
crossLD=cross(omegaL,dir);
radial=(u(1)*dot(dir,b3)-m*L*dot(crossLD,crossLD))/(m+ml);
out=[qd;wd;x(11:13);-g*[0;0;1]+radial*dir; ...
    crossLD;-(u(1)/(m*L))*cross(dir,b3)];
end


function y=project_state(y)
if any(~isfinite(y)) || abs(norm(y(1:4))-1)>0.01 || ...
        abs(norm(y(14:16))-1)>0.01
    error('CBFTraj:ComparisonManifold','Invalid integrated manifold state');
end
y(1:4)=y(1:4)/norm(y(1:4));y(14:16)=y(14:16)/norm(y(14:16));
end

