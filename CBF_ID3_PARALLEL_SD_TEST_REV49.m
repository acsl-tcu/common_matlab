function zipPath=CBF_ID3_PARALLEL_SD_TEST_REV49(opt)
% REV49: recorded-parameter replay, batched interval geometry and direct-stage screening.
% CT, F_INC_REFERENCE, F_BATCH, F_SCREEN, approximate endpoint h/z.
% No production modifications. Independent verification outside core.
% Synthetic extra parts are timing stress only; not actual sensed world.
if nargin==0,opt=struct();end
outDir=fileparts(mfilename('fullpath'));zipPath=fullfile(outDir,'CBF_ID3_PARALLEL_SD_REV49_RESULTS.zip');
summary="REV49 ID3 SHARED ANALYTIC AFFINE ROWS / CERTIFICATE CROSSCHECK"+newline;
act={};replay={};results={};details={};unit={};snap={};
try
    if ~evalin('base','exist(''app'',''var'')'),error('SD49:App','Run GUI, stop flight, keep app.');end
    app=evalin('base','app');
    if ~isa(app,'SimExp')||~isvalid(app)||~strcmp(string(app.cha),"s"),error('SD49:App','Stopped SimExp required');end
    hlc=app.agent(1).controller.hlc_suspended;cfg=hlc.cbfConfig;
    if cfg.cbfId~=3,error('SD49:ID','ID3 required');end
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
[ok,msg]=movefile(tmp,zipPath,'f');if ~ok,error('SD49:ZIP','%s',msg);end
clear clean
fprintf('[SD49] %s\n%s\n',zipPath,char(summary));
end

function o=defaults(o,cfg)
d=struct('thrustBounds',cfg.paperThrustBounds(:),'bodyDeg',cfgvalue48(cfg,'paperBodyTiltMaxDeg',40),'cableDeg',cfgvalue48(cfg,'paperCableTiltMaxDeg',60),'bodyLambda',cfgvalue48(cfg,'paperBodyTiltLambda',[2;2]),'cableLambda',cfgvalue48(cfg,'paperCableTiltLambda',[1;2;8;16]), ...
    'stride',80,'maxIter',4,'substeps',2,'refineSubsteps',4,'stressCounts',unique([1 4 8 16 cfg.paperMaxParts]),'stressSamples',3);
if ~isstruct(o)||~isscalar(o),error('SD49:Options','Expected scalar options struct');end
n=fieldnames(o);if any(~ismember(n,fieldnames(d))),error('SD49:Options','Unknown option');end
for j=fieldnames(d).',k=j{1};if ~isfield(o,k),o.(k)=d.(k);end,end
o.thrustBounds=double(o.thrustBounds(:));o.bodyLambda=double(o.bodyLambda(:));o.cableLambda=double(o.cableLambda(:));
validateattributes(o.thrustBounds,{'double'},{'numel',2,'real','finite','nonnegative'});assert(diff(o.thrustBounds)>0,'SD49:Bounds');
validateattributes(o.bodyDeg,{'numeric'},{'scalar','real','finite','>',0,'<',90});validateattributes(o.cableDeg,{'numeric'},{'scalar','real','finite','>',0,'<',90});
validateattributes(o.bodyLambda,{'double'},{'numel',2,'positive','finite'});validateattributes(o.cableLambda,{'double'},{'numel',4,'positive','finite'});
validateattributes(o.stressCounts,{'numeric'},{'vector','integer','positive','finite'});
for k={'stride','maxIter','substeps','refineSubsteps','stressSamples'},validateattributes(o.(k{1}),{'numeric'},{'scalar','integer','positive','finite'});end
% Upper enclosures of exact cos(angle*pi/180), no empirical angle margin.
a=cos_interval(o.bodyDeg);b=cos_interval(o.cableDeg);o.cosBody=a(2);o.cosCable=b(2);
[o.torqueLo,o.torqueHi,o.schemePath]=read_torque48();
end

function v=cfgvalue48(cfg,k,fallback)
if isfield(cfg,k),v=cfg.(k);else,v=fallback;end
end
function [lo,hi,p]=read_torque48()
% ID3 legacy has literal torque bounds, not exposed in cfg. Read active
% source, only accept the known explicit literal grammar; fail if changed.
p=which('CBF_REAL_SCHEME_03_PAPER_EXTENSION');if isempty(p),error('SD49:TorqueSource','ID3 source unavailable');end
s=fileread(p);num='([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)';
a=regexp(s,['lo\s*=\s*\[\s*nuLo\s*;\s*',num,'\s*;\s*',num,'\s*\]'],'tokens','once');
b=regexp(s,['hi\s*=\s*\[\s*nuHi\s*;\s*',num,'\s*;\s*',num,'\s*\]'],'tokens','once');
c=regexp(s,['tauZ\s*=\s*min\(\s*max\(\s*raw\(4\)\s*,\s*',num,'\s*\)\s*,\s*',num,'\s*\)'],'tokens','once');
if isempty(a)||isempty(b)||isempty(c),error('SD49:TorqueSource','Cannot read literal ID3 torque bounds; expose actual runtime config before testing');end
lo=[str2double(a(:));str2double(c{1})];hi=[str2double(b(:));str2double(c{2})];
if any(~isfinite([lo;hi]))||any(lo>=hi),error('SD49:TorqueSource','Invalid torque bounds');end
end

function [summary,act,replay,results,details,unit,snap]=audit(logger,hlc,cfg,o,summary)
H=double(hlc.param.dt);assert(isfinite(H)&&H>0,'SD49:dt');
D=logger.Data;L=D.agent(1);n=min([logger.k,numel(D.t),numel(D.phase),numel(L.controller.result),numel(L.estimator.result),numel(L.sensor.result)]);
P0=ascol(hlc.self.parameter.get(["mass","jx","jy","jz","gravity","loadmass","cableL"]),7);
summary=summary+sprintf('H=%.17g bounds=%s bodyDeg=%.17g cableDeg=%.17g bodyLambda=%s cableLambda=%s obsLambda=%s\n',H,mat2str(o.thrustBounds.',17),o.bodyDeg,o.cableDeg,mat2str(o.bodyLambda.'),mat2str(o.cableLambda.'),mat2str(cfg.paperLambda(:).'));
summary=summary+sprintf('PARAM_SOURCE live HLC.param.dt, live CBF config, current fixed agent parameter vector, logged estimator mL, logged geometry and hold; historical fixed parameters not logged. P0=%s\n',mat2str(P0.',17));
summary=summary+sprintf('THRUST_BOUNDS_MATCH_LIVE=%d (0 means explicit parameter sweep)\n',isequal(o.thrustBounds,double(cfg.paperThrustBounds(:))));
summary=summary+sprintf('TORQUE_SOURCE %s bounds=%s %s\n',o.schemePath,mat2str(o.torqueLo.'),mat2str(o.torqueHi.'));
summary=summary+sprintf('ANGLE_SOURCE cfg fields if present, otherwise user design 40/60 and provisional gains; explicit opt override permitted. bodyCfg=%d cableCfg=%d bodyGainCfg=%d cableGainCfg=%d\n',isfield(cfg,'paperBodyTiltMaxDeg'),isfield(cfg,'paperCableTiltMaxDeg'),isfield(cfg,'paperBodyTiltLambda'),isfield(cfg,'paperCableTiltLambda'));
summary=summary+sprintf('GEOMETRY_SOURCE radius=%.17g axialMargin=%.17g physicalRadius=%.17g maxParts=%d\n',cfg.paperCableRadiusM,cfg.paperCableAxialMarginM,cfg.paperPhysicalCableRadiusM,cfg.paperMaxParts);
summary=summary+sprintf('cos_threshold_upper_body=%.17g cable=%.17g\n',o.cosBody,o.cosCable);
summary=summary+sprintf('legacy_bounds=%s legacy_caps=[%.17g %.17g] scale=%s (unchanged objective weight, NOT a physical bound)\n',mat2str(cfg.paperThrustBounds(:).'),cfg.paperRateMaxNps,cfg.paperAccelMaxNps2,mat2str(cfg.paperControlScale(:).'));
summary=summary+"Certificate scope: ideal static obstacle polynomial dynamics from the recorded state with exact normalization enclosed along with the runtime rounded normalization, fixed parameters, exact held input. Outward interval elementary arithmetic assumes IEEE binary64 round-to-nearest; no verified physical estimation/ODE solver errors, no recursive joint CBF feasibility. Angle thresholds are upper interval enclosures of real cosine; geometry and other parameters are interpreted as stored numbers. Strict interval checks may reject exact boundary points. No fallback deemed safe."+newline;
summary=summary+"Reconstructed P from live fixed parameters and logged mL; P(8:9)=0 per archived HLC source; replay gate checks input and stage agreement. RK4 endpoint errors are diagnostic, not certified model-error bounds."+newline;
summary=summary+"K_T: one-cycle stopping invariant subset, no fixed q/nu cap, not maximum kernel, not physical bandwidth. Snapshot counterfactuals only; unchanged raw HLC objective, no new closed-loop flight. Search failure is NOT proof of infeasibility."+newline;
unit=unit_checks(H,o,cfg,P0);summary=summary+"UNIT_CHECKS=PASS (arithmetic and analytic regression, not full certification)"+newline;
act={strjoin({'t','T','q','nu_logged','legacy_exact_lo','legacy_exact_hi','legacy_adm_lo','legacy_adm_hi','hold_lo','hold_hi','K_lo','K_hi','q_lo','q_hi','K_valid','logged_in_new','stop_witness','Tmin_actual','Tmax_actual','h_logged_min','lower_logged_min','obs_count'},',')};
replay={'t,status,raw_error,hold_error,input_error,stage_error,endpoint_RK4_2_error,endpoint_RK4_16_error,endpoint_RK4_64_error'};
results={'t,case,parts,method,status,qp_count,candidate,online_ms,prepare_ms,lookup_qp_ms,verify_ms,cert_h,cert_lower,cert_own,nu,tauX,tauY,Tend,qEnd,objective,grid_h_min,grid_body_h_min,grid_cable_h_min,grid_lower_min,grid_final_min,initial_lower_min,model_residual,min_row_margin,direct_screened_rows_last_certificate,affine_scaled_error,lower_scaled_error,affine_audit_ms,initial_affine_ms,certificate_ms,solver_ms'};
details={'t,case,parts,method,row,M,initial_lower_lo,required,margin,reason'};
snap={strjoin([{'t'},vecnames('x',19),vecnames('P',7),vecnames('raw',4),vecnames('sent',28),vecnames('hold',5),{'obs_json'}],',')};
S={};skips=0;replayFail=0;allneg=0;actK=0;actIn=0;maxNu=0;lastT=NaN;
for k=1:n-1
    if char(D.phase(k))~='f',continue;end
    c=L.controller.result{k};if ~isstruct(c)||~isfield(c,'cbf')||~isstruct(c.cbf)||~isfield(c.cbf,'paperHold')||isempty(c.cbf.paperHold),continue;end
    h=c.cbf.paperHold;if ~h.active||abs(h.dt-H)>1e-10,skips=skips+1;continue;end
    sr=L.sensor.result{k};if ~isstruct(sr)||~isfield(sr,'cbfObservedWorld')||~isfield(sr.cbfObservedWorld,'parts'),skips=skips+1;continue;end
    ob=sr.cbfObservedWorld;if isfield(ob,'dynamicIndices')&&~isempty(ob.dynamicIndices),error('SD49:Dynamic','Only static world');end
    if ~isfield(sr,'cbfSensorTime')||abs(double(sr.cbfSensorTime)-double(D.t(k)))>1.5*H,skips=skips+1;continue;end
    ss=struct('t',double(D.t(k)),'x',ascol(c.x,19),'raw',ascol(c.tmp,4),'sent',ascol(c.xd,28),'u',ascol(c.input,4),'P',P0,'obs',ob,'hold',[h.T0;h.q0;h.nu;h.Tend;h.qEnd],'c',c.cbf,'next',nan(19,1));
    ss.P(6)=double(L.estimator.result{k}.state.mL);
    if isfield(ss.c,'paperControlScale')&&~isequal(double(ss.c.paperControlScale(:)),double(cfg.paperControlScale(:))),error('SD49:ConfigMismatch','Recorded objective scale differs from live config');end
    if isfield(ss.c,'paperThrustCap')&&ss.c.paperThrustCap~=cfg.paperAccelMaxNps2,error('SD49:ConfigMismatch','Recorded legacy nu cap differs from live config');end
    if isfield(ss.c,'paperRateCap')&&ss.c.paperRateCap~=cfg.paperRateMaxNps,error('SD49:ConfigMismatch','Recorded legacy rate cap differs from live config');end
    if any(~isfinite([ss.x;ss.raw;ss.P;ss.hold]))||any(ss.P([1:5,7])<=0)||ss.P(6)<0,error('SD49:Snapshot','Nonfinite or invalid state/parameters');end
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
if isempty(S),error('SD49:NoHold','No ID3 hold snapshots found');end
summary=summary+sprintf('logged_intervals=%d skipped=%d last_t=%.9g negative_logged_h=%d K_initial=%d logged_nu_inside_new_K=%d max_logged_abs_nu=%.9g lower_negative=%d\n',numel(S),skips,lastT,allneg,actK,actIn,maxNu,sum(cellfun(@(s)s.lm<0,S)));
% Sparse uniform selection plus ALL h-negative records and near-boundary
% neighbours, and the first encounter. No result-dependent cherry picking.
hv=cellfun(@(s)s.hm,S);idx=unique([1:o.stride:numel(S),numel(S),find(hv<0),find(hv<0)-1,find(hv<0)+1]);
first=find(isfinite(hv),1);if ~isempty(first),idx=unique([idx,first,max(1,first-1)]);end
lv=cellfun(@(s)s.lm,S);first=find(lv<0,1);if ~isempty(first),idx=unique([idx,first-2:first+2]);end
[~,ord]=sort(hv);idx=unique([idx,ord(1:min(8,numel(ord)))]);idx=idx(idx>=1&idx<=numel(S));
summary=summary+sprintf('selected=%d stride=%d maxIter=%d substeps=%d refine=%d\n',numel(idx),o.stride,o.maxIter,o.substeps,o.refineSubsteps);
methods={'CT','F_INC_REFERENCE','F_BATCH','F_SCREEN','ENDPOINT_H','ENDPOINT_Z','CT_SHARED','F_SHARED','F_SHARED_SCREEN'};
% Selection of recorded states and synthetic scaling cases.
metrics=zeros(0,11);stressIdx=[];for j=idx,if ~isempty(S{j}.obs.parts)&&S{j}.lm>=0,stressIdx(end+1)=j;end,end
if isempty(stressIdx),for j=idx,if ~isempty(S{j}.obs.parts),stressIdx(end+1)=j;end,end,end
if numel(stressIdx)>o.stressSamples
    [~,near]=min(cellfun(@(s)abs(s.hm),S(stressIdx)));sel=unique([near,round(linspace(1,numel(stressIdx),max(1,o.stressSamples-1)))],'stable');stressIdx=stressIdx(sel(1:min(numel(sel),o.stressSamples)));
end
for ii=1:numel(idx)
    ss=S{idx(ii)};fprintf('[SD49] snapshot %d/%d t=%.3f\n',ii,numel(idx),ss.t);
    P=physical(ss.P);G=geometry(cfg,P);x=ss.x;
    if max(abs([norm(x(1:4))-1;norm(x(14:16))-1]))>cfg.paperStateProjectionLimit,replayFail=replayFail+1;replay{end+1}=[numline(ss.t),',',csvtext('STATE_OUTSIDE_PROJECTION_LIMIT'),',',numline(nan(7,1))];continue;end
    x(1:4)=x(1:4)/norm(x(1:4));x(14:16)=x(14:16)/norm(x(14:16));y=[x;ss.hold(1:2)];Y0=normal_box(ss.x,y);tz=max(o.torqueLo(3),min(o.torqueHi(3),ss.raw(4)));parts=ss.obs.parts;
    % Recompute recorded raw and QP (value-only copy, no live controller).
    c2=cfg;c2.paperThrustState=struct('initialized',true,'T',y(20),'q',y(21));a=struct('sensor',struct('result',struct('cbfObservedWorld',ss.obs)));
    status='OK';err=nan(7,1);
    try
        raw=nominal_hlc(hlc,ss.x,ss.sent,ss.P,false);err(1)=max(abs(raw-ss.raw));
        [u,info]=CBF_REAL_SCHEME_03_PAPER_EXTENSION(a,c2,ss.x,ss.raw,[ss.P;0;0],H);
        if logical(info.success)~=logical(ss.c.success)||~info.success,error('SD49:Replay','Legacy QP not successful or mismatched');end
        h=info.paperHold;err(2)=max(abs([h.T0;h.q0;h.nu;h.Tend;h.qEnd]-ss.hold));err(3)=max(abs(u(:)-ss.u));
        z=CBF_REAL_D2T_OBSTACLE_EVAL(y,[ss.hold(3);ss.u(2:3)],ss.u(4),P,G,parts,cfg.paperLambda(:));
        if isempty(parts),err(4)=0;else,err(4)=max(abs(z.stages(:)-ss.c.stageValues(:)));end
        for m=1:3,ns=[2,16,64];xx=project_state(propagate(ss.x,ss.u,ss.hold(1:3).',ss.P,H,ns(m)));err(4+m)=max(abs(xx-ss.next));end
        if err(1)>1e-7*(1+max(abs(ss.raw)))||err(2)>1e-6||err(3)>1e-7||err(4)>1e-6*(1+max([0;abs(ss.c.stageValues(:))])),error('SD49:Replay','Recorded raw/hold/stage mismatch');end
    catch ME,status=status_text(ME);replayFail=replayFail+1;end
    replay{end+1}=[numline(ss.t),',',csvtext(status),',',numline(err)]; %#ok<AGROW>
    snap{end+1}=[numline([ss.t;ss.x;ss.P;ss.raw;ss.sent;ss.hold]),',',csvtext(jsonencode(parts))]; %#ok<AGROW>
    if ~strcmp(status,'OK'),continue;end
    cases={parts}; tags={'RECORDED'};
    if ismember(idx(ii),stressIdx)
        for np=unique(o.stressCounts(:).')
            if np<=numel(parts),continue;end
            cases{end+1}=stress_parts(parts,np,y,P,G);tags{end+1}='SYNTHETIC_PARTS';
        end
    end
    for cs=1:numel(cases)
        pp=cases{cs};tag=tags{cs};
        for m=1:numel(methods)
            try
                [r,d]=compare47(methods{m},y,Y0,tz,ss,P,G,pp,cfg,o,H);
            catch ME
                r=blank47();r.status=status_text(ME);d={};
            end
            v=[r.nqp;~isempty(r.w);r.online;r.prep;r.lookup;r.verify;r.hcert;r.lowercert;r.owncert];
            u=nan(3,1);te=NaN;qe=NaN;cost=NaN;gg=nan(5,1);
            if ~isempty(r.w)
                u=r.w;te=y(20)+H*y(21)+.5*H^2*u(1);qe=y(21)+H*u(1);
                wn=[2*(ss.raw(1)-y(20)-H*y(21))/H^2;ss.raw(2:3)];cost=sum(((u-wn)./cfg.paperControlScale(:)).^2);
                try,gg=grid_check(y,u,tz,ss.P,P,G,pp,cfg,o,H);catch,end
            end
            prefix=[numline(ss.t),',',csvtext(tag),',',numline(numel(pp)),',',csvtext(methods{m}),','];
            results{end+1}=[prefix,csvtext(r.status),',',numline([v;u;te;qe;cost;gg(:);r.ini;r.res;r.margin;r.coverage;r.afferr;r.lowerr;r.auditms;r.affms;r.certms;r.qpms])];
            for j=1:numel(d),details{end+1}=[prefix,d{j}];end
            metrics(end+1,:)=[cs>1,numel(pp),m,r.online,r.prep,r.lookup,r.verify,~isempty(r.w),r.hcert,r.lowercert,r.owncert];
        end
    end
end
summary=summary+sprintf('replay_failed=%d stress_selected=%d\n',replayFail,numel(stressIdx));
summary=summary+"Methods CT, F_INC_REFERENCE, F_BATCH (same interval operations batched over parts), F_SCREEN (prove z>=0 directly on validated state box at order 3; original integral rule for unresolved rows), ENDPOINT_H/Z (approximate, not certified). No fixed local input search radii. Native proof does not require final psi_r>=0 inside cycle. Independent verification outside online timing; synthetic translated parts are stress tests only. No joint recursive feasibility or estimator error guarantee."+newline;
for k=0:1
    if k==0,groups=NaN;tag='RECORDED';else,groups=unique(metrics(metrics(:,1)==1,2)).';tag='SYNTHETIC_PARTS';end
    for np=groups
        for m=1:numel(methods)
            mask=metrics(:,1)==k&metrics(:,3)==m;if k==1,mask=mask&metrics(:,2)==np;end
            a=metrics(mask,:);if isempty(a),continue;end
            t=a(isfinite(a(:,4)),4);u=a(isfinite(a(:,6)),6);v=a(isfinite(a(:,5)),5);
            summary=summary+sprintf('%s parts=%g %s n=%d candidate=%d native_cert=%d independent_h=%d independent_lower=%d actual_core_median_ms=%.6g max_ms=%.6g within_H=%d native_cert_within_H=%d prep_median_ms=%.6g unused_lookup_median_ms=%.6g\n',tag,np,methods{m},size(a,1),sum(a(:,8)),sum(a(:,11)),sum(a(:,9)),sum(a(:,10)),stat47(t,false),stat47(t,true),sum(a(:,4)<=1000*H),sum(a(:,4)<=1000*H&a(:,11)>0),stat47(v,false),stat47(u,false));
        end
    end
end
summary=summary+"SHARED paths: one generated zero-input state jet, analytic input coefficients, vectorized point geometry; box projection bypass only when it satisfies all rows. Independent legacy affine audit is separately timed; same interval certificate as F_BATCH/F_SCREEN. No new physical constants."+newline;
summary=summary+"STATUS=COMPLETE_PARALLEL_SNAPSHOT_DIAGNOSTIC"+newline;
end

function x=stat47(v,mx)
if isempty(v),x=NaN;elseif mx,x=max(v);else,x=median(v);end
end

function r=blank47()
r=struct('status','NOT_RUN','nqp',0,'w',[],'online',NaN,'prep',0,'lookup',NaN,'verify',NaN,'hcert',false,'lowercert',false,'owncert',false,'ini',NaN,'res',NaN,'margin',NaN,'coverage',NaN,'afferr',NaN,'lowerr',NaN,'auditms',NaN,'affms',NaN,'certms',0,'qpms',0);
end

function [r,d]=compare47(method,y,Y0,tz,ss,P,G,parts,cfg,o,H)
r=blank47();d={};lastCert=[];lastReq=[];lastMar=[];tm=tic;[~,~,kl,kh,valid]=kernel(y(20),y(21),H,o.thrustBounds);
if ~valid,r.status='K_INITIAL_OUTSIDE';r.online=1000*toc(tm);return;end
lo=[kl;o.torqueLo(1:2)];hi=[kh;o.torqueHi(1:2)];wn=[2*(ss.raw(1)-y(20)-H*y(21))/H^2;ss.raw(2:3)];sc=cfg.paperControlScale(:);
lam=[repmat({cfg.paperLambda(:)},1,numel(parts)),{o.bodyLambda,o.cableLambda}];
if startsWith(method,'ENDPOINT_')
    useZ=strcmp(method,'ENDPOINT_Z');packed=[];if ~useZ,packed=pack_geometry47(parts);end
    % Finite difference of full held-input RK4 map. Shared propagation per
    % input probe, not per obstacle. No claim that finite diff is exact.
    ref=min(hi,max(lo,wn));[h0,z0]=endpoint_value(y,zeros(3,1),tz,ss.P,P,G,parts,lam,o,0,useZ,packed);
    [f,~]=endpoint_value(y,ref,tz,ss.P,P,G,parts,lam,o,H,useZ,packed);
    D=zeros(numel(f),3);for j=1:3
        a=ref;b=ref;step=1e-3*sc(j);a(j)=max(lo(j),ref(j)-step);b(j)=min(hi(j),ref(j)+step);
        if b(j)>a(j),D(:,j)=(endpoint_value(y,b,tz,ss.P,P,G,parts,lam,o,H,useZ,packed)-endpoint_value(y,a,tz,ss.P,P,G,parts,lam,o,H,useZ,packed))/(b(j)-a(j));end
    end
    kap=cellfun(@(l)l(end),lam(:));gamma=H*kap./(1+H*kap);req=(1-gamma).*h0;
    [r.w,r.status]=qp(D,f-D*ref,req,wn,lo,hi,sc);r.nqp=1;r.ini=min(z0);
    if ~isempty(r.w),f2=endpoint_value(y,r.w,tz,ss.P,P,G,parts,lam,o,H,useZ,packed);r.res=min(f2-req);r.status=['APPROX_',r.status];end
else
    shared=ismember(method,{'CT_SHARED','F_SHARED','F_SHARED_SCREEN'});
    ta=tic;
    if shared,rr=affine_shared49(y,tz,P,G,parts,lam,o);
    else,[rr,~,~]=affine_rows(y,tz,P,G,parts,cfg,o);end
    r.affms=1000*toc(ta);r.ini=min(rr.lower);
    tq=tic;
    if shared
        [r.w,r.status,nq]=qp_shared49(rr.B,rr.b,zeros(numel(lam),1),wn,lo,hi,sc);r.nqp=nq;
    else
        [r.w,r.status]=qp(rr.B,rr.b,zeros(numel(lam),1),wn,lo,hi,sc);r.nqp=1;
    end
    r.qpms=1000*toc(tq);
    if ismember(method,{'F_INC_REFERENCE','F_BATCH','F_SCREEN','F_SHARED','F_SHARED_SCREEN'})&&~isempty(r.w)
        certMethod=method;
        if strcmp(method,'F_SHARED'),certMethod='F_BATCH';elseif strcmp(method,'F_SHARED_SCREEN'),certMethod='F_SCREEN';end
        for it=1:o.maxIter
            tc=tic;c=bound_candidate(y,Y0,r.w,tz,P,G,parts,lam,o,H,1,true,true,certMethod);
            r.coverage=0;if isfield(c,'directMask'),r.coverage=sum(c.directMask);end
            [req,mar,r.owncert,why]=cert_rule(c,H,lam,6,y,r.w,o,true);r.status=why;r.margin=min(mar);r.certms=r.certms+1000*toc(tc);
            lastCert=c;lastReq=req;lastMar=mar;
            if r.owncert||~c.ok||~c.lowerOK||it==o.maxIter,break;end
            cushion=max(c.a(:,2)-c.a(:,1),1e-9*max([abs(rr.b-req),max(abs(rr.B.*sc.'),[],2),ones(numel(lam),1)],[],2));
            tq=tic;
            if shared,[v,st,nq]=qp_shared49(rr.B,rr.b,req+cushion,wn,lo,hi,sc);r.nqp=r.nqp+nq;
            else,[v,st]=qp(rr.B,rr.b,req+cushion,wn,lo,hi,sc);r.nqp=r.nqp+1;end
            r.qpms=r.qpms+1000*toc(tq);
            if isempty(v)||isequal(v,r.w),r.status=['SEARCH_',st];break;end;r.w=v;
        end
    end
end
if isnan(r.online),r.online=1000*toc(tm);end
% Independent regression of analytic rows against generated legacy
% evaluator. NOT part of new online path, explicitly timed separately.
if ismember(method,{'CT_SHARED','F_SHARED','F_SHARED_SCREEN'})
    ta=tic;
    try
        [old,~,~]=affine_rows(y,tz,P,G,parts,cfg,o);
        [r.afferr,r.lowerr]=affine_error49(rr,old);
        if ~isempty(r.w)
            [a,l]=point_stages(y,r.w,tz,P,G,parts,lam,o);
            r.afferr=max(r.afferr,max(abs(a-rr.b-rr.B*r.w)./(1+abs(a))));
            r.lowerr=max(r.lowerr,max(abs(l-rr.lower)./(1+abs(l))));
        end
        if max(r.afferr,r.lowerr)>1e-7
            r.status=['AFFINE_REGRESSION_FAILED|',r.status];r.owncert=false;
        end
    catch ME
        r.status=['AFFINE_AUDIT_FAILED|',status_text(ME),'|',r.status];r.owncert=false;
    end
    r.auditms=1000*toc(ta);
end
% CSV formatting is not part of the timed control path.
if ~isempty(lastCert)&&lastCert.ok&&~lastCert.early
    c=lastCert;for jj=1:numel(lam),mm=c.M(jj);reason='INTEGRAL';if isfield(c,'directMask')&&c.directMask(jj),mm=NaN;reason='DIRECT_Z_BOX';end;d{end+1}=[numline([jj;mm;c.lowerMin(jj);lastReq(jj);lastMar(jj)]),',',csvtext(reason)];end
end
% Independent OFFLINE certificate. No feedback into candidate selection.
if ~isempty(r.w)
    tv=tic;
    try
        [r.hcert,r.lowercert]=verify47(y,Y0,r.w,tz,P,G,parts,lam,o,H);
    catch ME,r.status=[r.status,'|OFFLINE_',status_text(ME)];end
    r.verify=1000*toc(tv);
end
end

function [f,l]=endpoint_value(y,w,tz,p,P,G,parts,lam,o,H,useZ,packed)
if H>0,x=project_state(propagate(y(1:19),[y(20);w(2:3);tz],[y(20),y(21),w(1)],p,H,2));y=[x;y(20)+H*y(21)+.5*H^2*w(1);y(21)+H*w(1)];end
if useZ
    [~,l,~,st]=point_stages(y,w,tz,P,G,parts,lam,o);f=cellfun(@(s)s(end-1),st);
else
    f=h_packed47(y,P,G,packed,o);l=f;
end
f=f(:);
end
function pack=pack_geometry47(parts)
% Static geometry prepack once per solve; vectorized over ALL obstacle parts.
n=numel(parts);pack.C=zeros(3,n);pack.Q=zeros(9,n);
for j=1:n,[c,a,R]=obs_shape(parts(j));Q=R*diag(a.^2)*R.';pack.C(:,j)=c;pack.Q(:,j)=Q(:);end
end
function f=h_geometry47(y,P,G,parts,o)
f=h_packed47(y,P,G,pack_geometry47(parts),o);
end
function f=h_packed47(y,P,G,pack,o)
p=y(14:16);C=y(8:10)-G.cableCenterFromLoad*P.cableL*p;
d=C-pack.C;r=sqrt(sum(d.^2,1));if any(r<=0),error('SD49:Geometry','Coincident centres');end
n=d./r;v=[n(1,:).*n;n(2,:).*n;n(3,:).*n];sc=sqrt(G.cableAxisShort^2+(G.cableAxisLong^2-G.cableAxisShort^2)*(p.'*n).^2);so=sqrt(sum(pack.Q.*v,1));
q=y(1:4);f=[(r-sc-so).';q(1)^2-q(2)^2-q(3)^2+q(4)^2-o.cosBody;-p(3)-o.cosCable];
if ~isreal(f)||any(~isfinite(f)),error('SD49:Geometry','Nonfinite endpoint geometry');end
end
function [hc,lc]=verify47(y,Y,w,tz,P,G,parts,lam,o,H)
% Candidate-specific OFFLINE proof: lower hierarchy via quadratic lower bound;
% physical h via validated endpoint and chord curvature. No grid-as-proof.
hc=false;lc=false;if ~actuator_cert(y,w,H,o),return;end
U=iv([w;tz]);B=picard(Y,U,P,iv(H));[~,rad]=rhsjet(initjet(B,0),U,P);if rad(1,2)>=0,return;end
J0=statejet_incremental(Y,U,P,4);JB=statejet_incremental(B,U,P,5);
[s,~]=barrier_jets(J0,P,G,parts,lam,o);[~,~,ddz,hj]=barrier_jets(JB,P,G,parts,lam,o);
lc=true;for j=1:numel(lam),z=s{j}(end-1,:);l=iv(lam{j}(end));kap=id(l,ia(iv(1),im(iv(H),l)));a=ia(s{j}(end,:),im(isub(kap,l),z));m=im(iv(.5),im(iv(H),iv(max(0,-ddz(j,1)))));margin=isub(a,m);lc=lc&&all(s{j}(1:end-1,1)>=0)&&margin(1)>=0;end
hc=lc;endY=Y;pow=iv(1);for k=1:5,pow=im(pow,iv(H));if k<=4,Q=coeff(J0,k);else,Q=coeff(JB,k);end;endY=ia(endY,im(Q,pow));end
endY=[max(endY(:,1),B(:,1)),min(endY(:,2),B(:,2))];if any(endY(:,1)>endY(:,2)),return;end
emptyLam=repmat({zeros(0,1)},1,numel(lam));[se,~]=barrier_jets(initjet(endY,0),P,G,parts,emptyLam,o);
hc=true;for j=1:numel(lam),M=max(abs(im(iv(2),hj{j}(3,:))));delta=im(iv(.125),im(im(iv(H),iv(H)),iv(M)));hc=hc&&min(s{j}(1,1),se{j}(1,1))>=delta(2);end
hc=hc||lc;
end
function p=stress_parts(p,n,y,P,G)
% Keep original parts; append distinct translated copies away from the vehicle.
% Synthetic stress not an obstacle detector test. No duplicated row shortcuts.
base=p(1);C=y(8:10)-G.cableCenterFromLoad*P.cableL*y(14:16);[~,a,~]=obs_shape(base);
for j=numel(p)+1:n,v=base;ang=2*pi*(j-1)/max(n,1);v.center=C+(max(a)+G.cableAxisLong+3+j*.15)*[cos(ang);sin(ang);.2];p(end+1)=v;end
end

function Y=normal_box(raw,y)
% Enclose exact real normalization of the recorded q and pT as well as the
% binary64 normalized point used by the legacy generated row evaluation.
Y=iv(y);for I={1:4,14:16}
    k=I{1};a=iv(raw(k));sq=isq(a);b=sq(1,:);for j=2:numel(k),b=ia(b,sq(j,:));end
    if b(1)<=0,error('SD49:Normalize','Zero or uncertified norm');end
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
function ok=actuator_cert(y,w,H,o)
B=o.thrustBounds;
% Quadratic Bezier control points all inside the thrust box, endpoint K_T.
% More restrictive only for initial states outside K_T (already excluded).
t=iv(y(20));q=iv(y(21));v=iv(w(1));hh=iv(H);half=iv(.5);
b0=t;b1=ia(t,im(half,im(hh,q)));b2=ia(ia(t,im(hh,q)),im(half,im(im(hh,hh),v)));qe=ia(q,im(hh,v));
% terminal stop point T_end + H q_end/2 also within box.
b3=ia(b2,im(half,im(hh,qe)));z=[b0;b1;b2;b3];ok=all(z(:,1)>=B(1)&z(:,2)<=B(2))&&all(w(2:3)>=o.torqueLo(1:2)&w(2:3)<=o.torqueHi(1:2));
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
for j=1:3,v=zeros(3,1);v(j)=1;[b,low]=point_stages(y,v,tz,P,G,parts,lam,o);B(:,j)=b-a;assert(max(abs(low-lower))<1e-7*(1+max(abs(lower))),'SD49:RelativeDegree');end
v=[31;.071;-.043];b=point_stages(y,v,tz,P,G,parts,lam,o);assert(max(abs(b-a-B*v)./(1+abs(b)))<1e-7,'SD49:Affine');
rr=struct('b',a,'B',B,'lower',lower);
end

function rr=affine_shared49(y,tz,P,G,parts,lam,o)
% Exact affine structure of the final HOCBF stage (up to floating point).
% All lower derivatives are input-independent. Compute drift from one
% generated zero-input jet; input sensitivities from the equations.
% No finite differencing and no fixed physical parameters.
J=CBF_REAL_D2T_JET(y(1:19),y(20),y(21),0,[0;0;tz],P);
X=J.stateTaylor;n=numel(parts);p=y(14:16);T=y(20);
q=y(1:4);a=q(1);b=q(2);c=q(3);d=q(4);
R=[a*a+b*b-c*c-d*d,2*(b*c-a*d),2*(b*d+a*c); ...
   2*(b*c+a*d),a*a-b*b+c*c-d*d,2*(c*d-a*b); ...
   2*(b*d-a*c),2*(c*d+a*b),a*a-b*b-c*c+d*d];
e=R(:,3);Db=[zeros(3,1),-R(:,2)/P.J(1),R(:,1)/P.J(2)];
% Dp and DL are sensitivities of the FOURTH time derivative,
% Db is sensitivity of the SECOND derivative of thrust direction.
Dp=zeros(3,3);DL=Dp;
for j=1:3
    dT2=double(j==1);
    Dp(:,j)=cross(-(dT2*cross(p,e)+T*cross(p,Db(:,j)))/(P.m*P.cableL),p);
    DL(:,j)=p*(dT2*dot(p,e)+T*dot(p,Db(:,j)))/(P.m+P.mL);
end
DC=DL-G.cableCenterFromLoad*P.cableL*Dp;
C=X(8:10,:)-G.cableCenterFromLoad*P.cableL*X(14:16,:);
B=zeros(n+2,3);dr=zeros(n+2,1);lower=zeros(4*n+6,1);
if n>0
    oc=zeros(n,3);ax=oc;rot=zeros(n,3,3);
    for i=1:n
        [ci,ai,Ri]=obs_shape(parts(i));oc(i,:)=ci.';ax(i,:)=ai.';rot(i,:,:)=reshape(Ri,1,3,3);
    end
    D=cell(3,1);pj=D;
    for j=1:3,D{j}=repmat(C(j,:),n,1);D{j}(:,1)=D{j}(:,1)-oc(:,j);pj{j}=repmat(X(13+j,:),n,1);end
    r2=dot_series49(D,D);if any(r2(:,1)<1e-12),error('SD49:Coincident','Coincident centres');end
    rad=sqrt_series49(r2);inv=inv_series49(r2);pd=dot_series49(pj,D);
    delta=G.cableAxisLong^2-G.cableAxisShort^2;
    sC=delta*mul_series49(mul_series49(pd,pd),inv);sC(:,1)=sC(:,1)+G.cableAxisShort^2;
    sC=sqrt_series49(sC);
    E=cell(3,1);
    for j=1:3,E{j}=zeros(n,5);for k=1:3,E{j}=E{j}+D{k}.*rot(:,k,j);end;E{j}=E{j}.*ax(:,j);end
    sO=sqrt_series49(mul_series49(dot_series49(E,E),inv));
    hs=rad-sC-sO;
    S=stages49(hs,lam{1});dr(1:n)=S(:,end);lower(1:4*n)=reshape(S(:,1:4).',[],1);
    % h(D,p)=||D||-sqrt(as^2+delta*(p'n)^2)-sqrt(n'Qo*n).
    % n=D/||D||, d n/dD=(I-n*n')/||D||.
    N=[D{1}(:,1),D{2}(:,1),D{3}(:,1)]./rad(:,1);
    pn=N*p;Qn=zeros(n,3);
    for j=1:3
        v=zeros(n,1);for k=1:3,v=v+N(:,k).*rot(:,k,j);end
        v=v.*ax(:,j).^2;
        for k=1:3,Qn(:,k)=Qn(:,k)+v.*rot(:,k,j);end
    end
    g=delta*(pn./sC(:,1)).*p.'+Qn./sO(:,1);
    gD=N-(g-sum(g.*N,2).*N)./rad(:,1);
    gp=-delta*(pn./sC(:,1)).*N;
    B(1:n,:)=gD*DC+gp*Dp;
end
hB=zeros(1,5);sg=[1,-1,-1,1];
for j=1:4,v=conv(X(j,:),X(j,:));hB=hB+sg(j)*v(1:5);end
hB(1)=hB(1)-o.cosBody;hC=-X(16,:);hC(1)=hC(1)-o.cosCable;
SB=stages49(hB,lam{end-1});SC=stages49(hC,lam{end});
dr(n+1:n+2)=[SB(end);SC(end)];lower(4*n+1:end)=[SB(1:end-1).';SC(1:end-1).'];
B(n+1,:)=Db(3,:);B(n+2,:)=-Dp(3,:);
rr=struct('b',dr,'B',B,'lower',lower);
assert(all(isfinite([dr;B(:);lower])),'SD49:SharedRows');
end

function s=stages49(h,l)
% Each row is a separate barrier's ordinary Taylor coefficients.
d=h.*factorial(0:size(h,2)-1);s=zeros(size(h,1),numel(l)+1);
s(:,1)=d(:,1);pol=1;
for k=1:numel(l),pol=conv(pol,[l(k),1]);s(:,k+1)=d(:,1:k+1)*pol.';end
end
function c=mul_series49(a,b)
c=zeros(size(a));for k=1:size(a,2),c(:,k)=sum(a(:,1:k).*b(:,k:-1:1),2);end
end
function c=dot_series49(a,b)
c=mul_series49(a{1},b{1});for j=2:numel(a),c=c+mul_series49(a{j},b{j});end
end
function r=sqrt_series49(a)
if any(a(:,1)<=0),error('SD49:PointSqrt','Nonpositive support');end
r=zeros(size(a));r(:,1)=sqrt(a(:,1));
for k=2:size(a,2),r(:,k)=(a(:,k)-sum(r(:,2:k-1).*r(:,k-1:-1:2),2))./(2*r(:,1));end
end
function r=inv_series49(a)
r=zeros(size(a));r(:,1)=1./a(:,1);
for k=2:size(a,2),r(:,k)=-sum(a(:,2:k).*r(:,k-1:-1:1),2)./a(:,1);end
end
function [e,l]=affine_error49(a,b)
e=max([abs(a.b-b.b)./(1+abs(b.b));reshape(abs(a.B-b.B)./(1+abs(b.B)),[],1)]);
l=max(abs(a.lower-b.lower)./(1+abs(b.lower)));
end
function [w,status,n]=qp_shared49(B,b,req,nom,lo,hi,S)
% Diagonal positive objective: box projection is its exact minimizer
% over the box. If it satisfies all linear rows it is also optimal for
% the constrained QP. Interval certificate still required in F paths.
n=0;w=min(hi,max(lo,nom));
if all(isfinite([w;b;B(:);req]))&&all(lo<=hi)&&all(b+B*w>=req)
    status='OK';return
end
[w,status]=qp(B,b,req,nom,lo,hi,S);n=1;
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
function c=bound_candidate(y,Y0,w,tz,P,G,parts,lam,o,H,ns,fast,inc,method)
if nargin<14,method='F_INC_REFERENCE';end
bj=@barrier_jets;if ismember(method,{'F_BATCH','F_SCREEN'}),bj=@barrier_jets_batch48;end
if nargin<12,fast=false;end
if nargin<13,inc=false;end
sj=@statejet;if inc,sj=@statejet_incremental;end
c=struct('ok',false,'lowerOK',false,'lowerMin',[],'M',[],'a',[],'z',[],'h',[],'substeps',ns,'boxes',0,'reason','NOT_RUN','early',false);
try
    U=iv([w(:);tz]);Y=Y0;
    N=5;if fast,N=4;end
    J=sj(Y,U,P,N);[st,~]=bj(J,P,G,parts,lam,o);
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
        Z=initjet(B,0);[~,rad]=rhsjet(Z,U,P);if rad(1,2)>=0,error('SD49:Tension','Taut-cable domain not enclosed');end
        if strcmp(method,'F_SCREEN')&&ns==1
            % Direct proof of z=psi_(r-1) throughout the validated state box.
            % No empirical distance gate. The same input, geometry, gains and
            % initial lower stages are used; unresolved rows use original F.
            ls=cellfun(@(v)v(1:end-1),lam,'UniformOutput',false);
            [short,~]=bj(sj(B,U,P,3),P,G,parts,ls,o);
            c.directLower=cellfun(@(v)v(end,1),short);c.directMask=c.directLower>=0;
            if any(~c.directMask)
                keep=~c.directMask(1:numel(parts));
                JB=sj(B,U,P,5);[~,dr]=bj(JB,P,G,parts(keep),[lam(find(keep)),lam(end-1:end)],o);
                ix=[find(keep(:));nr-1;nr];c.M(ix)=max(0,-dr(:,1));
            end
            c.boxes=c.boxes+1;
        else
            JB=sj(B,U,P,5);[~,der]=bj(JB,P,G,parts,lam,o);
            c.M=max(c.M,max(0,-der(:,1)));c.boxes=c.boxes+1;
        end
        if k<ns
            J0=sj(Y,U,P,4);endY=Y;pow=iv(1);
            for r=1:5,pow=im(pow,dt);if r<=4,Q=coeff(J0,r);else,Q=coeff(JB,r);end;endY=ia(endY,im(Q,pow));end
            % Intersection with validated Picard enclosure at endpoint.
            Y=[max(endY(:,1),B(:,1)),min(endY(:,2),B(:,2))];if any(Y(:,1)>Y(:,2)),error('SD49:Endpoint','Empty validated intersection');end
        end
    end
    c.ok=true;c.reason='OK';
catch ME,c.reason=status_text(ME);end
end
function [req,margin,ok,why]=cert_rule(c,H,lam,m,y,w,o,inc)
if nargin<8,inc=false;end
n=numel(lam);req=nan(n,1);margin=req;ok=false;why='BOUND_FAILED';if ~c.ok,why=c.reason;return;end
if c.early,why='INITIAL_LOWER_OUTSIDE_OR_UNCERTAIN';return;end
for j=1:n
    if isfield(c,'directMask')&&c.directMask(j)
        % For a screened row z>=0 is already proved on the whole cycle.
        % Keep the original CT row in any subsequent search, but do not
        % mislabel the screened row as a zero derivative bound.
        req(j)=0;margin(j)=c.directLower(j);continue
    end
    if m==4,r=im(iv(c.M(j)),iv(H));else
        if inc,[I0,I1]=weights_cached(H,lam{j}(end));else,[I0,I1]=weights(H,lam{j}(end));end;r=im(iv(c.M(j)),id(I1,I0));
        if m==6,r=isub(r,id(c.z(j,:),I0));r=[max(0,r(1)),max(0,r(2))];end
    end
    req(j)=r(2);marg=isub(c.a(j,:),iv(req(j)));margin(j)=marg(1);
end
if ~c.lowerOK,why='INITIAL_LOWER_OUTSIDE_OR_UNCERTAIN';elseif ~actuator_cert(y,w,H,o),why='ACTUATOR_CERT_FAILED';elseif any(margin<0),why='ROW_NOT_CERTIFIED';else,ok=true;why='PASS';end
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
z=im(iv(H),iv(l));N=max(24,ceil(2*z(2))+8);if N>256,error('SD49:Weight','lambda*H too large for this test');end
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
error('SD49:Picard','Reach enclosure did not close');
end
function b=inflate(a)
% This expansion affects only proposed enclosure, never the proof criterion.
d=max(1e-12,.15*(a(:,2)-a(:,1)));b=[a(:,1)-d,a(:,2)+d];if any(~isfinite(b(:))),error('SD49:Reach','Nonfinite');end
end
function J=statejet(Y,U,P,N)
J=initjet(Y,N);
for k=1:N
    A=cellfun(@(j)j(1:k,:),J,'UniformOutput',false);F=rhsjet(A,U,P);
    for i=1:21,J{i}(k+1,:)=id(F{i}(k,:),iv(k));end
end
end

function [a,b]=weights_cached(H,l)
% Same outward interval result as weights(); exact parameter key.
persistent keys vals
if isempty(keys),keys=zeros(0,2);vals=cell(0,2);end
j=find(keys(:,1)==H & keys(:,2)==l,1);
if isempty(j)
    [a,b]=weights(H,l);
    keys(end+1,:)=[H,l];vals(end+1,:)={a,b};
else,a=vals{j,1};b=vals{j,2};end
end

function J=statejet_incremental(Y,U,P,N)
% Same formal power series recurrence as statejet:
% X_{k+1}=[s^k]f(X,U)/(k+1).
% For each intermediate product only calculate coefficient k, retaining
% coefficients 0:k-1. Each coefficient uses the same outward operations,
% summation order, and central square enclosure as jm/rhsjet.
J=initjet(Y,N);
qI=1:4;wI=5:7;pI=14:16;wlI=17:19;
zero=zeros(N+1,2);
b=repmat({zero},3,1);vt=b;Jw=b;c=b;d=b;cp=b;
pd=zero;v2=zero;rad=zero;
ml=ia(iv(P.m),iv(P.mL));mL=im(iv(P.m),iv(P.cableL));
invml=id(iv(1),ml);invmL=id(iv(-1),mL);
iJ=cell(3,1);for j=1:3,iJ{j}=id(iv(1),iv(P.J(j)));end
for k=0:N-1
    q=J(qI);w=J(wI);p=J(pI);wl=J(wlI);T=J{20};i=k+1;
    b{1}(i,:)=im(ia(jc(q{2},q{4},k),jc(q{1},q{3},k)),iv(2));
    b{2}(i,:)=im(isub(jc(q{3},q{4},k),jc(q{1},q{2},k)),iv(2));
    b{3}(i,:)=ia(isub(isub(jc(q{1},q{1},k,true),jc(q{2},q{2},k,true)),jc(q{3},q{3},k,true)),jc(q{4},q{4},k,true));
    v=jc_cross(wl,p,k);
    for j=1:3,vt{j}(i,:)=v(j,:);Jw{j}(i,:)=im(w{j}(i,:),iv(P.J(j)));end
    v=jc_cross(w,Jw,k);u=jc_cross(q(2:4),w,k);
    for j=1:3,c{j}(i,:)=v(j,:);d{j}(i,:)=u(j,:);end
    pd(i,:)=jc_dot(p,b,k);v2(i,:)=jc_dot(vt,vt,k,true);
    rad(i,:)=im(isub(jc(T,pd,k),im(v2(i,:),mL)),invml);
    v=jc_cross(p,b,k);for j=1:3,cp{j}(i,:)=v(j,:);end
    F=zeros(21,2);F(1,:)=im(jc_dot(q(2:4),w,k),iv(-.5));
    for j=1:3
        F(j+1,:)=im(ia(jc(q{1},w{j},k),d{j}(i,:)),iv(.5));
        u0=[0,0];if k==0,u0=U(j+1,:);end
        F(4+j,:)=im(isub(u0,c{j}(i,:)),iJ{j});
        F(7+j,:)=J{10+j}(i,:);
        F(10+j,:)=jc(rad,p{j},k);
        F(13+j,:)=vt{j}(i,:);
        F(16+j,:)=im(jc(T,cp{j},k),invmL);
    end
    if k==0,F(13,:)=isub(F(13,:),iv(P.gravity));F(21,:)=U(1,:);end
    F(20,:)=J{21}(i,:);
    V=id(F,iv(i));
    for j=1:21,J{j}(i+1,:)=V(j,:);end
end
end
function z=jc(a,b,k,same)
if nargin<4,same=false;end
i=k+1;c=im(a(1:i,:),b(i:-1:1,:));
if same&&mod(i,2)==1,j=(i+1)/2;c(j,:)=isq(a(j,:));end
z=c(1,:);for j=2:i,z=ia(z,c(j,:));end
end
function r=jc_dot(a,b,k,same)
if nargin<4,same=false;end
r=jc(a{1},b{1},k,same);for j=2:numel(a),r=ia(r,jc(a{j},b{j},k,same));end
end
function r=jc_cross(a,b,k)
r=[isub(jc(a{2},b{3},k),jc(a{3},b{2},k)); ...
   isub(jc(a{3},b{1},k),jc(a{1},b{3},k)); ...
   isub(jc(a{1},b{2},k),jc(a{2},b{1},k))];
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
function [st,der,ddz,h]=barrier_jets(X,P,G,parts,lam,o)
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
st=cell(size(h));der=zeros(numel(h),2);ddz=nan(numel(h),2);
for i=1:numel(h)
    z=h{i};s=z(1,:);
    for j=1:numel(lam{i}),if j==numel(lam{i})&&size(z,1)>=3,ddz(i,:)=im(iv(2),z(3,:));end;k=size(z,1)-1;z=ia(im(z(2:end,:),iv((1:k)')),js(z(1:end-1,:),iv(lam{i}(j))));s=[s;z(1,:)];end %#ok<AGROW>
    st{i}=s;if size(z,1)>=2,der(i,:)=z(2,:);else,der(i,:)=NaN;end
end
end
function [st,der,ddz,h]=barrier_jets_batch48(X,P,G,parts,lam,o)
% Coefficient-major layout: each coefficient has n consecutive parts.
% Same outward elementary operations and summation order as scalar path.
N=size(X{1},1)-1;n=numel(parts);h=cell(n+2,1);p=X(14:16);C=cell(3,1);
for j=1:3,C{j}=isub(X{7+j},js(p{j},im(iv(G.cableCenterFromLoad),iv(P.cableL))));end
if n>0
    oc=zeros(n,3);ax=oc;R=zeros(n,3,3);
    for i=1:n,[c,a,Q]=obs_shape(parts(i));oc(i,:)=c.';ax(i,:)=a.';R(i,:,:)=reshape(Q,1,3,3);end
    D=cell(3,1);Pj=D;for j=1:3,D{j}=isub(repelem(C{j},n,1),[iv(oc(:,j));zeros(n*N,2)]);Pj{j}=repelem(p{j},n,1);end
    as=isq(iv(G.cableAxisShort));delta=isub(isq(iv(G.cableAxisLong)),as);
    r2=bjdot48(D,D,n,true);r=bjsqrt48(r2,n);inv=bjinv48(r2,n);pd=bjdot48(Pj,D,n,false);
    supC=bjsqrt48(ia([repmat(as,n,1);zeros(n*N,2)],im(bjm48(bjm48(pd,pd,n,true),inv,n,false),delta)),n);
    E=cell(3,1);for j=1:3,E{j}=zeros(n*(N+1),2);for k=1:3,E{j}=ia(E{j},im(D{k},iv(repmat(R(:,k,j),N+1,1))));end;E{j}=im(E{j},iv(repmat(ax(:,j),N+1,1)));end
    supO=bjsqrt48(bjm48(bjdot48(E,E,n,true),inv,n,false),n);hh=isub(isub(r,supC),supO);
    for i=1:n,h{i}=hh(i:n:end,:);end
end
b=bodyjet(X(1:4));h{end-1}=isub(b{3},jconst(iv(o.cosBody),N));h{end}=isub(-flip(X{16},2),jconst(iv(o.cosCable),N));
st=cell(size(h));der=zeros(numel(h),2);ddz=nan(numel(h),2);
for i=1:numel(h)
    z=h{i};ss=z(1,:);
    for j=1:numel(lam{i}),if j==numel(lam{i})&&size(z,1)>=3,ddz(i,:)=im(iv(2),z(3,:));end;k=size(z,1)-1;z=ia(im(z(2:end,:),iv((1:k)')),js(z(1:end-1,:),iv(lam{i}(j))));ss=[ss;z(1,:)];end
    st{i}=ss;if size(z,1)>=2,der(i,:)=z(2,:);else,der(i,:)=NaN;end
end
end
function r=bjm48(a,b,n,same)
N=size(a,1)/n;r=zeros(size(a));I=reshape(1:size(a,1),n,N);
for k=1:N
    x=I(:,1:k);y=I(:,k:-1:1);c=im(a(x(:),:),b(y(:),:));
    if same&&mod(k,2)==1,j=(k+1)/2;ix=(j-1)*n+(1:n);c(ix,:)=isq(a(I(:,j),:));end
    z=c(1:n,:);for j=2:k,z=ia(z,c((j-1)*n+(1:n),:));end;r(I(:,k),:)=z;
end
end
function r=bjdot48(a,b,n,same)
r=bjm48(a{1},b{1},n,same);for j=2:numel(a),r=ia(r,bjm48(a{j},b{j},n,same));end
end
function r=bjsqrt48(a,n)
N=size(a,1)/n;I=reshape(1:size(a,1),n,N);r=zeros(size(a));if any(a(I(:,1),1)<=0),error('SD49:Sqrt','Unverified positive square root');end
r(I(:,1),:)=out(sqrt(a(I(:,1),1)),sqrt(a(I(:,1),2)));
for k=2:N,z=zeros(n,2);for j=2:k-1,z=ia(z,im(r(I(:,j),:),r(I(:,k-j+1),:)));end;r(I(:,k),:)=id(isub(a(I(:,k),:),z),im(iv(2),r(I(:,1),:)));end
end
function r=bjinv48(a,n)
N=size(a,1)/n;I=reshape(1:size(a,1),n,N);r=zeros(size(a));r(I(:,1),:)=id(iv(1),a(I(:,1),:));
for k=2:N,z=zeros(n,2);for j=2:k,z=ia(z,im(a(I(:,j),:),r(I(:,k-j+1),:)));end;r(I(:,k),:)=id(-flip(z,2),a(I(:,1),:));end
end

function [c,a,R]=obs_shape(p)
c=double(p.center(:));kind='ellipsoid';if isfield(p,'kind'),kind=char(p.kind);end
switch kind
    case 'sphere',if isfield(p,'params'),a=p.params.radius;else,a=p.radius;end;a=repmat(a,3,1);R=eye(3);
    case 'ellipsoid',if isfield(p,'params'),a=p.params.axes;else,a=p.axes;end;if isfield(p,'R'),R=double(p.R);else,R=eye(3);end
    otherwise,error('SD49:Obstacle','Unsupported type');
end
a=double(a(:));if numel(c)~=3||numel(a)~=3||any(a<=0)||~isequal(size(R),[3,3])||any(~isfinite([c;a;R(:)])),error('SD49:Shape','Invalid geometry');end
end
% ---- scalar interval arithmetic; arrays are N-by-2 [lower, upper] ----
function r=iv(x),x=double(x(:));r=[x,x];end
function r=out(l,h)
if any(~isfinite([l(:);h(:)]))||any(l(:)>h(:)),error('SD49:Interval','Invalid interval arithmetic');end
% One outward ulp at the computed value (at least realmin*eps relative
% to normal scale; avoids relying on exact directed hardware rounding).
r=[l-eps(max(abs(l),realmin)),h+eps(max(abs(h),realmin))];
if any(~isfinite(r(:))),error('SD49:Interval','Overflow');end
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
if any(b(:,1)<=0&b(:,2)>=0),error('SD49:Division','Interval denominator crosses zero');end
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
if a(1,1)<=0,error('SD49:Sqrt','Unverified positive square root');end
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
assert(eps(0)>0 && realmin/2>0 && 1+eps(1)>1 && 1+eps(1)/2==1,'SD49:IEEE');
rows{end+1}='BINARY64_BASIC_SANITY,1,1';
for d=[1,20,40,60,89],a=cos_interval(d);assert(cosd(d)>=a(1)-1e-15&&cosd(d)<=a(2)+1e-15,'SD49:Cosine');rows{end+1}=sprintf('COS_INTERVAL_%d,%.17g,1',d,a(2)-a(1));end
% Deterministic stop witness and endpoints for varying thrust bounds and periods.
n=0;worst=0;
for B={[0;20],[2;9],[0;7],[3;3.5]}
 for dt=[H,H/2,2*H]
  for a=linspace(.01,.99,11),T=B{1}(1)+a*diff(B{1});for qf=linspace(-.99,.99,9),if qf>=0,q=qf*2*(B{1}(2)-T)/dt;else,q=qf*2*(T-B{1}(1))/dt;end
    [~,~,l,u,ok]=kernel(T,q,dt,B{1});v=-q/dt;assert(ok&&v>=l-1e-8&&v<=u+1e-8,'SD49:KernelWitness');
    for f=[0,.25,.5,.75,1],w=l+f*(u-l);[mn,mx]=thrust_extrema(T,q,w,dt);T2=T+dt*q+.5*dt^2*w;q2=q+dt*w;vi=max([B{1}(1)-mn,mx-B{1}(2),-2*(T2-B{1}(1))/dt-q2,q2-2*(B{1}(2)-T2)/dt,0]);worst=max(worst,vi);assert(vi<1e-7,'SD49:KernelEndpoint');n=n+1;end
  end,end
 end
end
rows{end+1}=sprintf('KERNEL_INTERVAL_ENDPOINTS,%d,1',n);rows{end+1}=sprintf('KERNEL_MAX_NUMERIC_RESIDUAL,%.17g,1',worst);
for l=[.01,2,16,32], [a,b]=weights(H,l);r=id(b,a);ref=H/(-expm1(-l*H))-1/l;assert(ref>=r(1)-1e-10&&ref<=r(2)+1e-10,'SD49:WeightCheck');rows{end+1}=sprintf('INTEGRAL_WEIGHT_%g,%.17g,1',l,mean(r));end
P=physical(p);G=geometry(cfg,P);y=[1;zeros(12,1);0;0;-1;zeros(3,1);(P.m+P.mL)*P.gravity;0];
% Quaternion/cable conventions and mixed order, empty obstacles must still yield two rows.
parts=struct('center',{},'kind',{},'radius',{});lam={o.bodyLambda,o.cableLambda};w=[.3;.01;-.02];tz=.03;
[a,lower,hs]=point_stages(y,w,tz,P,G,parts,lam,o);[rr,~,~]=affine_rows(y,tz,P,G,parts,cfg,o);assert(numel(a)==2&&numel(lower)==6&&abs(hs(1)-(1-o.cosBody))<1e-12&&abs(hs(2)-(1-o.cosCable))<1e-12,'SD49:Tilt');
J=statejet(iv(y),iv([w;tz]),P,5);[ss,~]=barrier_jets(J,P,G,parts,lam,o);for k=1:2,assert(a(k)>=ss{k}(end,1)-1e-9&&a(k)<=ss{k}(end,2)+1e-9,'SD49:JetCrossCheck');end
assert(all(isfinite(rr.B(:))),'SD49:TiltAffine');rows{end+1}='EMPTY_OBSTACLE_TILT_AND_AFFINE,2,1';
% Nonzero orientation/rate, ellipsoid shape and input; cross-check every stage.
y(1:4)=[cos(.2);sin(.2);0;0];y(5:7)=[.13;-.21;.05];y(14:16)=[.3;.1;-sqrt(.9)];y(17:19)=[.04;.1;.02];y(8:10)=[.2;1;1];y(11:13)=[.1;.5;-.1];
parts=struct('center',[1;5;2],'kind','ellipsoid','axes',[.5;.8;1.3],'R',eye(3));lam=[{cfg.paperLambda(:)},lam];z=CBF_REAL_D2T_OBSTACLE_EVAL(y,w,tz,P,G,parts,lam{1});
J=statejet(iv(y),iv([w;tz]),P,5);[ss,der]=barrier_jets(J,P,G,parts,lam,o);assert(all(z.stages(:)>=ss{1}(:,1)-1e-8*(1+abs(z.stages(:))))&&all(z.stages(:)<=ss{1}(:,2)+1e-8*(1+abs(z.stages(:)))),'SD49:ObstacleJet');assert(all(isfinite(der(:))),'SD49:Derivative');
% Check interval against point coefficients for ALL 21 states through order 4.
ref=CBF_REAL_D2T_JET(y(1:19),y(20),y(21),w(1),[w(2:3);tz],P);for k=0:4,v=coeff(J,k);r=ref.extendedTaylor(:,k+1);assert(all(r>=v(:,1)-1e-9*(1+abs(r))&r<=v(:,2)+1e-9*(1+abs(r))),'SD49:StateJet');end
rows{end+1}='OBSTACLE_AND_STATE_JET_CROSSCHECK,1,1';
% Cap change test: current angle near 30deg, same jet evaluated at new limits.
a=o;a.cosBody=cosd(20);a.cosCable=cosd(10);[~,~,h1]=point_stages(y,w,tz,P,G,parts,lam,o);[~,~,h2]=point_stages(y,w,tz,P,G,parts,lam,a);assert(abs((h2(end-1)-h1(end-1))-(o.cosBody-a.cosBody))<1e-12&&abs((h2(end)-h1(end))-(o.cosCable-a.cosCable))<1e-12,'SD49:RuntimeCaps');rows{end+1}='RUNTIME_CAP_CHANGE,1,1';
% Check actual control authority near both limits, independent of obstacles.
p0=y;p0(1:4)=[cosd(19.5);sind(19.5);0;0];p0(5:7)=0;
p0(14:16)=[sind(59);0;-cosd(59)];p0(17:19)=0;empty=struct('center',{},'kind',{},'radius',{});
[rr,~,ll]=affine_rows(p0,tz,P,G,empty,cfg,o);
assert(abs(rr.B(1,1))<1e-7&&rr.B(1,2)<0&&norm(rr.B(2,:))>1e-8,'SD49:AngleAuthority');
[~,~,h1]=point_stages(p0,w,tz,P,G,empty,ll,o);
assert(abs(h1(1)-(cosd(39)-o.cosBody))<1e-12&&abs(h1(2)-(cosd(59)-o.cosCable))<1e-12,'SD49:AngleGeometry');
rows{end+1}='NEAR_LIMIT_CONTROL_AUTHORITY,1,1';
% New fast path regression: same stage enclosures using the required order.
U=iv([w;tz]);J4=statejet(iv(y),U,P,4);J5=statejet(iv(y),U,P,5);
[s4,~]=barrier_jets(J4,P,G,parts,lam,o);[s5,~]=barrier_jets(J5,P,G,parts,lam,o);
for k=1:numel(s4),assert(isequal(s4{k},s5{k}),'SD49:InitialStageOrder');end
rows{end+1}='INITIAL_STAGE_ORDER4_EQUALS_ORDER5,1,1';

% The incremental evaluator must return exactly the same interval
% coefficients, not just similar floating-point midpoints.
tests={iv(y),out(y-1e-6,y+1e-6),out(y-1e-3,y+1e-3)};
for k=1:numel(tests)
    UU=U;if k==3,UU=out(U(:,1)-1e-4,U(:,2)+1e-4);end
    for ord=0:5
        j0=statejet(tests{k},UU,P,ord);j1=statejet_incremental(tests{k},UU,P,ord);
        assert(isequal(j0,j1),'SD49:IncrementalCoefficients');
    end
end
rows{end+1}='INCREMENTAL_EXACT_INTERVAL_COEFFICIENTS,18,1';
for l=[.01,2,16,32]
    [a,b]=weights(H,l);[c,d]=weights_cached(H,l);
    assert(isequal(a,c)&&isequal(b,d),'SD49:CachedWeights');
end
rows{end+1}='CACHED_EXACT_INTERVAL_WEIGHTS,4,1';

% Hover at a genuine strictly interior actuator state; test full interval.
yh=[1;zeros(12,1);0;0;-1;zeros(3,1);(P.m+P.mL)*P.gravity;0];
if yh(20)>o.thrustBounds(1)&&yh(20)<o.thrustBounds(2)
    [~,~,ll]=affine_rows(yh,0,P,G,empty,cfg,o);wh=zeros(3,1);
    ch=bound_candidate(yh,iv(yh),wh,0,P,G,empty,ll,o,H,1,true);
    [~,~,pass]=cert_rule(ch,H,ll,5,yh,wh,o);
    assert(pass,'SD49:WholeIntervalHover');rows{end+1}='WHOLE_INTERVAL_HOVER,1,1';
else,rows{end+1}='WHOLE_INTERVAL_HOVER_NOT_IN_THRUST_RANGE,NaN,1';end
% Cross-check fast raw geometry against legacy generated h.
[~,~,hs]=point_stages(y,w,tz,P,G,parts,lam,o);assert(max(abs(hs-h_geometry47(y,P,G,parts,o)))<1e-9,'SD49:FastGeometry');rows{end+1}='FAST_GEOMETRY_MATCH,1,1';
% Verify second derivative of z from Taylor against derivative psi_r-lambda*dot z.
[s,dr,dd]=barrier_jets(statejet_incremental(iv(y),U,P,5),P,G,parts,lam,o);
for j=1:numel(lam),dz=isub(s{j}(end,:),im(iv(lam{j}(end)),s{j}(end-1,:)));a=isub(dr(j,:),im(iv(lam{j}(end)),dz));assert(max(a(1),dd(j,1))<=min(a(2),dd(j,2)),'SD49:QuadraticDerivative');end
rows{end+1}='QUADRATIC_DERIVATIVE_IDENTITY,1,1';
% Batched scalar-equivalence test: distinct positions, axes and orientations.
pp=parts;pp(2)=parts;pp(2).center=parts.center+[2;1;-.2];pp(2).axes=[.7;.4;1];pp(2).R=[0 -1 0;1 0 0;0 0 1];
ll=[{cfg.paperLambda(:),cfg.paperLambda(:)},lam(end-1:end)];
for ord=0:5
    jj=statejet_incremental(out(y-1e-6,y+1e-6),U,P,ord);
    ll2=cellfun(@(v)v(1:min(numel(v),ord)),ll,'UniformOutput',false);
    [a,b,c,d]=barrier_jets(jj,P,G,pp,ll2,o);[aa,bb,cc,dd]=barrier_jets_batch48(jj,P,G,pp,ll2,o);
    assert(isequaln(a,aa)&&isequaln(b,bb)&&isequaln(c,cc)&&isequaln(d,dd),'SD49:BatchExact');
end
rows{end+1}='BATCH_EXACT_INTERVAL_MATCH,6,1';
% Exact relative-degree coefficient formula regression, including near
% angle limits, rotated distinct parts and changed runtime parameters.
worst=0;
for kk=1:3
    Y=y;if kk==2,Y=p0;elseif kk==3,Y=yh;end
    [old,~,ll]=affine_rows(Y,tz,P,G,pp,cfg,o);
    fresh=affine_shared49(Y,tz,P,G,pp,ll,o);
    [ee,el]=affine_error49(fresh,old);worst=max([worst,ee,el]);
    [af,lf]=point_stages(Y,w,tz,P,G,pp,ll,o);
    worst=max([worst;abs(af-fresh.b-fresh.B*w)./(1+abs(af));abs(lf-fresh.lower)./(1+abs(lf))]);
end
assert(worst<1e-7,'SD49:AnalyticAffineRegression');
rows{end+1}=sprintf('SHARED_AFFINE_REGRESSION,%.17g,1',worst);
% Direct proof on strictly interior hover, no obstacles.
if yh(20)>o.thrustBounds(1)&&yh(20)<o.thrustBounds(2)
    ch=bound_candidate(yh,iv(yh),zeros(3,1),0,P,G,empty,{o.bodyLambda,o.cableLambda},o,H,1,true,true,'F_SCREEN');
    [~,~,pass]=cert_rule(ch,H,{o.bodyLambda,o.cableLambda},6,yh,zeros(3,1),o,true);
    assert(pass&&all(ch.directMask),'SD49:DirectHover');rows{end+1}='DIRECT_Z_HOVER,2,1';
end
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

