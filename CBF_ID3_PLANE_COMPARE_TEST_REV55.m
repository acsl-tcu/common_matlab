function zipPath=CBF_ID3_PLANE_COMPARE_TEST_REV55(opt)
% Standalone test BEFORE changing generate. Read stopped GUI app only.
% Existing generated JET and OBSTACLE_EVAL are read-only dependencies;
% new evaluator and input kernel are local functions in this file.
% Same support function and gains for each plane comparison. Numerical
% grids are diagnostics, NOT sampled-data certificates.
if nargin==0,opt=struct();end
folder=fileparts(mfilename('fullpath'));zipPath=fullfile(folder,'CBF_ID3_PLANE_COMPARE_REV55_RESULTS.zip');
sumtxt="REV55 STANDALONE PRE-GENERATOR PLANE / ALL CONSTRAINTS TEST"+newline;
rows={};detail={};replay={};ground={};unit={};
try
    assert(evalin('base','exist(''app'',''var'')')~=0,'REV55:App','Run GUI and stop first');app=evalin('base','app');
    assert(isa(app,'SimExp')&&isvalid(app)&&strcmp(string(app.cha),"s"),'REV55:App','Stopped SimExp required');
    hlc=app.agent(1).controller.hlc_suspended;cfg=hlc.cbfConfig;assert(cfg.cbfId==3,'REV55:ID');
    assert(~isempty(which('CBF_REAL_D2T_JET'))&&~isempty(which('CBF_REAL_D2T_OBSTACLE_EVAL')),'REV55:ExistingGenerated','Existing ID3 generated JET/OBSTACLE_EVAL required; do not regenerate for this test');
    oldpath=path;pc=onCleanup(@()path(oldpath));jetpath=which('CBF_REAL_D2T_JET');root=fileparts(fileparts(fileparts(fileparts(jetpath))));addpath(fullfile(root,'COMMON'),'-begin');
    assert(~isempty(which('CBF_COMMON_GEOMETRY')),'REV55:GeometrySource');
    o=defaults(opt,cfg);H=double(hlc.param.dt);assert(H>0&&isfinite(H),'REV55:dt');
    [sumtxt,rows,detail,replay,ground,unit]=audit55(app.logger,hlc,cfg,o,H,sumtxt);clear pc
catch ME
    sumtxt=sumtxt+"STATUS=FAILED"+newline+string(getReport(ME,'extended','hyperlinks','off'))+newline;
end
% Atomic replace, one persistent ZIP only.
tmp=[tempname(folder),'.zip'];clean=onCleanup(@()delete_if_exists(tmp));
fos=java.io.FileOutputStream(tmp);zos=java.util.zip.ZipOutputStream(fos);zc=onCleanup(@()close_zip(zos));
put_entry(zos,'SUMMARY.txt',char(sumtxt));E={'CANDIDATES.csv',rows;'ROWS.csv',detail;'REPLAY.csv',replay;'GROUND_GATE.csv',ground;'UNIT.csv',unit};
for j=1:size(E,1),if ~isempty(E{j,2}),put_entry(zos,E{j,1},join_lines(E{j,2}));end,end
zos.close();clear zc;[ok,msg]=movefile(tmp,zipPath,'f');assert(ok,'REV55:ZIP','%s',msg);clear clean
fprintf('[REV55] %s\n%s\n',zipPath,char(sumtxt));
end
function [summary,rows,detail,replay,ground,unit]=audit55(logger,hlc,cfg,o,H,summary)
D=logger.Data;L=D.agent(1);n=min([logger.k,numel(D.t),numel(D.phase),numel(L.controller.result),numel(L.estimator.result),numel(L.sensor.result)]);
P0=ascol(hlc.self.parameter.get(["mass","jx","jy","jz","gravity","loadmass","cableL"]),7);
o.obsLambda=cfg.paperLambda(:);o.mode='CENTER';o.spheres=false;o.extras=true;
summary=summary+sprintf('H=%.17g P0=%s thrust=%s torqueLo=%s torqueHi=%s obsLambda=%s\n',H,mat2str(P0.',17),mat2str(o.thrustBounds.',17),mat2str(o.torqueLo.',17),mat2str(o.torqueHi.',17),mat2str(o.obsLambda.'));
summary=summary+sprintf('bodyDeg=%g cableDeg=%g bodyLambda=%s cableLambda=%s groundZ=%g groundLambda=%s tensionMin=%g tensionLambda=%s tensionEnabled=%d\n',o.bodyDeg,o.cableDeg,mat2str(o.bodyLambda.'),mat2str(o.cableLambda.'),o.groundZ,mat2str(o.groundLambda.'),o.tensionMin,mat2str(o.tensionLambda.'),o.tensionEnabled);
G0=geom55(cfg,physical(P0));summary=summary+sprintf('GEOMETRY_SOURCE %s droneSafetyRadius=%.17g loadSafetyRadius=%.17g cableShort=%.17g cableLong=%.17g\n',which('CBF_COMMON_GEOMETRY'),G0.droneSafetyRadius,G0.loadSafetyRadius,G0.cableAxisShort,G0.cableAxisLong);
summary=summary+sprintf('INPUT_SOURCE %s; LEGACY_JET %s; LEGACY_EVAL %s; NEW_EVALUATOR_AND_KERNEL=LOCAL_TEST_FUNCTIONS\n',o.schemePath,which('CBF_REAL_D2T_JET'),which('CBF_REAL_D2T_OBSTACLE_EVAL'));
summary=summary+"Fixed P from current agent, mL from each logged estimator (historical fixed parameters not logged). thrustBounds from live cfg unless explicit test override. New tilt/ground/tension settings are runtime design choices, not physical parameters. Sphere radii from existing COMMON geometry; cable geometry stays ID3. No new margins. All candidates share same thrust kernel and objective. Ground is point constraint. No production edits. All certificates for NEW plane modes remain unestablished; RK4 grid only."+newline;
rows={'t,case,parts,method,status,nrows,initial_h_min,initial_lower_min,obstacle_h_min,cable_h_min,qp_ok,nu,tauX,tauY,tauZ,cost,actuator_cert,affine_error,lower_input_error,last_residual,grid_h_min,grid_lower_min,grid_tension_min,setup_ms,coeff_ms,qp_ms,online_ms,qp_count,ground_active,m,jx,jy,jz,g,mL,L'};
detail={'t,case,parts,method,row,name,degree,h,psi1,psi2,psi3,psi4,nx,ny,nz,eta,grid_h_min,grid_lower_min'};
replay={'t,status,raw_error,input_error,hold_error,legacy_stage_error,center_stage_error'};
ground={'t,active,entry_lower_min,load_z,body_z,reason'};unit={'test,pass,max_scaled_error'};
S={};armed=false;firstArm=NaN;skip=0;
for k=1:n-1
    if char(D.phase(k))~='f',armed=false;continue;end
    c=L.controller.result{k};if ~isstruct(c)||~isfield(c,'cbf')||~isstruct(c.cbf)||~isfield(c.cbf,'paperHold')||isempty(c.cbf.paperHold),continue;end
    h=c.cbf.paperHold;if ~h.active||abs(h.dt-H)>1e-10,skip=skip+1;continue;end
    sr=L.sensor.result{k};if ~isstruct(sr)||~isfield(sr,'cbfObservedWorld')||~isfield(sr.cbfObservedWorld,'parts'),skip=skip+1;continue;end
    ob=sr.cbfObservedWorld;if isfield(ob,'dynamicIndices')&&~isempty(ob.dynamicIndices),error('REV55:Dynamic','Static world only');end
    if ~isfield(sr,'cbfSensorTime')||abs(double(sr.cbfSensorTime)-double(D.t(k)))>1.5*H,skip=skip+1;continue;end
    ss=struct('t',double(D.t(k)),'x',ascol(c.x,19),'raw',ascol(c.tmp,4),'sent',ascol(c.xd,28),'u',ascol(c.input,4),'P',P0,'obs',ob,'hold',[h.T0;h.q0;h.nu;h.Tend;h.qEnd],'c',c.cbf);
    ss.P(6)=double(L.estimator.result{k}.state.mL);if any(~isfinite([ss.x;ss.P;ss.hold;ss.raw])),error('REV55:Snapshot');end
    if isfield(ss.c,'paperControlScale')&&~isequal(double(ss.c.paperControlScale(:)),double(cfg.paperControlScale(:))),error('REV55:ConfigMismatch');end
    if max(abs([norm(ss.x(1:4))-1;norm(ss.x(14:16))-1]))>cfg.paperStateProjectionLimit,error('REV55:Projection');end
    ss.y=[project_state(ss.x);ss.hold(1:2)];P=physical(ss.P);G=geom55(cfg,P);
    oo=o;oo.groundEnabled=true;oo.tensionEnabled=false;oo.spheres=false;oo.mode='CENTER';
    z=plane_eval55(ss.y,zeros(3,1),ss.u(4),P,G,ob.parts([]),oo);v=[z.stages{3}(1:end-1);z.stages{4}(1:end-1)];
    reason='WAIT_ENTRY';if ~o.groundEnabled,reason='OPTION_OFF';elseif armed,reason='LATCHED';elseif all(v>=0),armed=true;reason='ARM';if isnan(firstArm),firstArm=ss.t;end,end
    ss.ground=armed&&o.groundEnabled;ground{end+1}=[numline([ss.t;ss.ground;min(v);ss.y(10);ss.y(10)-P.cableL*ss.y(16)]),',',csvtext(reason)];
    ss.hm=Inf;ss.lm=Inf;if isfield(ss.c,'paperCurrentH')&&~isempty(ss.c.paperCurrentH),ss.hm=min(ss.c.paperCurrentH(:));end
    if isfield(ss.c,'paperStageValues')&&~isempty(ss.c.paperStageValues),ss.lm=min(ss.c.paperStageValues(:));end
    S{end+1}=ss;
end
assert(~isempty(S),'REV55:NoHold');summary=summary+sprintf('logged=%d skipped=%d ground_first_arm=%.17g (phase f plus entry gate; latched until phase exits; diagnostic not certified)\n',numel(S),skip,firstArm);
hv=cellfun(@(s)s.hm,S);lv=cellfun(@(s)s.lm,S);idx=unique([1:o.stride:numel(S),numel(S),find(hv<0),find(hv<0)-1,find(hv<0)+1]);i=find(isfinite(hv),1);if ~isempty(i),idx=unique([idx,i,i-1]);end;i=find(lv<0,1);if ~isempty(i),idx=unique([idx,i-2:i+2]);end
[~,i]=sort(hv);idx=unique([idx,i(1:min(8,numel(i)))]);idx=idx(idx>=1&idx<=numel(S));
si=idx(cellfun(@(s)~isempty(s.obs.parts)&&s.lm>=0,S(idx)));if isempty(si),si=idx(cellfun(@(s)~isempty(s.obs.parts),S(idx)));end
if ~isempty(si),[~,i]=min(abs(hv(si)));si=unique([si(i),si(round(linspace(1,numel(si),max(1,o.stressSamples-1))))],'stable');si=si(1:min(o.stressSamples,numel(si)));end
modes={'CENTER','SHAPE','PW','FIXED_SHAPE'};groups={'CABLE','SHAPES','ALL'};metrics=zeros(0,13);fail=0;maxCenter=0;maxAff=0;maxLow=0;
for ii=1:numel(idx)
    s=S{idx(ii)};P=physical(s.P);G=geom55(cfg,P);p=s.obs.parts;y=s.y;tz=min(o.torqueHi(3),max(o.torqueLo(3),s.raw(4)));wlog=[s.hold(3);s.u(2:3)];
    status='OK';er=nan(5,1);
    try
        raw=nominal_hlc(hlc,s.x,s.sent,s.P,false);er(1)=max(abs(raw-s.raw));
        c2=cfg;c2.paperThrustState=struct('initialized',true,'T',y(20),'q',y(21));a=struct('sensor',struct('result',struct('cbfObservedWorld',s.obs)));
        [u,info]=CBF_REAL_SCHEME_03_PAPER_EXTENSION(a,c2,s.x,s.raw,[s.P;0;0],H);assert(info.success&&s.c.success,'REV55:Replay');
        h=info.paperHold;er(2)=max(abs(u(:)-s.u));er(3)=max(abs([h.T0;h.q0;h.nu;h.Tend;h.qEnd]-s.hold));
        old=CBF_REAL_D2T_OBSTACLE_EVAL(y,wlog,s.u(4),P,G,p,o.obsLambda);if isempty(p),er(4)=0;else,er(4)=max(abs(old.stages(:)-s.c.stageValues(:)));end
        oo=o;oo.spheres=false;oo.extras=false;z=plane_eval55(y,wlog,s.u(4),P,G,p,oo);if isempty(p),er(5)=0;else,v=cell2mat(z.stages);er(5)=max(abs(v(:)-old.stages(:))./(1+abs(old.stages(:))));end
        assert(er(1)<=1e-7*(1+max(abs(s.raw)))&&er(2)<=1e-7&&er(3)<=1e-6&&er(4)<=1e-6*(1+max([0;abs(old.stages(:))]))&&er(5)<=1e-9,'REV55:ReplayMismatch');
    catch ME,status=status_text(ME);fail=fail+1;end
    replay{end+1}=[numline(s.t),',',csvtext(status),',',numline(er)];if ~strcmp(status,'OK'),continue;end;maxCenter=max(maxCenter,er(5));
    pp={p};tags={'RECORDED'};if ismember(idx(ii),si),for np=unique(o.stressCounts(:).'),if np>numel(p),pp{end+1}=stress_parts(p,np,y,P,G);tags{end+1}='SYNTHETIC_PARTS';end,end,end
    fprintf('[REV55] %d/%d t=%.3f\n',ii,numel(idx),s.t);
    for j=1:numel(pp),parts=pp{j};tag=tags{j};for group=1:numel(groups),for m=1:numel(modes)
        oo=o;oo.mode=modes{m};oo.spheres=group>=2;oo.extras=group==3;oo.groundEnabled=group==3&&s.ground;name=[modes{m},'_',groups{group}];
        [r,dr]=candidate55(y,s,P,G,parts,tz,oo,H,cfg);prefix=[numline(s.t),',',csvtext(tag),',',numline(numel(parts)),',',csvtext(name),','];
        rows{end+1}=[prefix,csvtext(r.status),',',numline([r.v;s.P])];for k=1:numel(dr),detail{end+1}=[prefix,dr{k}];end
        metrics(end+1,:)=[j>1,numel(parts),group,m,r.v(1),r.v(3),r.v(5),r.v(6),r.v(12),r.v(16),r.v(17),r.v(22),r.v(23)];
        if isfinite(r.v(13)),maxAff=max(maxAff,r.v(13));end;if isfinite(r.v(14)),maxLow=max(maxLow,r.v(14));end
    end,end,end
end
unit{end+1}=sprintf('LEGACY_CENTER_PARITY,%d,%.17g',fail==0,maxCenter);unit{end+1}=sprintf('AFFINE_AT_SELECTED_INPUT,%d,%.17g',any(metrics(:,8)>0)&&maxAff<=1e-8,maxAff);unit{end+1}=sprintf('LOWER_INPUT_INDEPENDENCE,%d,%.17g',any(metrics(:,8)>0)&&maxLow<=1e-8,maxLow);
summary=summary+sprintf('selected=%d replay_failed=%d maxCenterScaled=%.17g maxAffineScaled=%.17g maxLowerScaled=%.17g\n',numel(idx),fail,maxCenter,maxAff,maxLow);
summary=summary+"Groups: 1=CABLE only (legacy geometry), 2=SHAPES (same cable + body/load spheres), 3=ALL (SHAPES + body/cable tilt + flight-gated ground + optional tension). Each group: same constraints, geometry, mathematical thrust bounds and objective. Only plane law varies; PW holds eta, FIXED holds n for one interval. QP solves CT final-stage rows; grid uses same plan, not reoptimized at sample points. Not a sampled-data proof. Stress synthetic translations, not actual flight. QP status not a proof of infeasibility."+newline;
for stress=0:1
    if stress==0,ns=NaN;else,ns=unique(metrics(metrics(:,1)==1,2)).';end
    for np=ns,for group=1:numel(groups),for m=1:numel(modes)
        mask=metrics(:,1)==stress&metrics(:,3)==group&metrics(:,4)==m;if stress,mask=mask&metrics(:,2)==np;end;v=metrics(mask,:);if isempty(v),continue;end
        summary=summary+sprintf('stress=%d parts=%g group=%d %s n=%d lower_initial_nonnegative=%d cable_h_nonnegative_with_obstacles=%d qp_ok=%d actuator_cert=%d grid_h_nonnegative=%d grid_lower_nonnegative=%d online_median_ms=%.6g max_ms=%.6g online_le5=%d\n',stress,np,group,modes{m},size(v,1),sum(v(:,6)>=0),sum(v(:,2)>0&v(:,7)>=0),sum(v(:,8)>0),sum(v(:,9)>0),sum(v(:,10)>=0),sum(v(:,11)>=0),median(v(:,12),'omitnan'),max(v(:,12),[],'omitnan'),sum(v(:,12)<=5));
    end,end,end
end
if fail>0||isempty(metrics),summary=summary+"STATUS=INCOMPLETE_REPLAY"+newline;elseif maxAff>1e-8||maxLow>1e-8,summary=summary+"STATUS=AFFINE_AUDIT_FAILED"+newline;else,summary=summary+"STATUS=COMPLETE_PLANE_SNAPSHOT_DIAGNOSTIC"+newline;end
end
function G=geom55(cfg,P)
G=geometry(cfg,P);g=CBF_COMMON_GEOMETRY(P,struct());G.droneSafetyRadius=g.droneSafetyRadius;G.loadSafetyRadius=g.loadSafetyRadius;
end
function [r,detail]=candidate55(y,s,P,G,parts,tz,o,H,cfg)
v=nan(24,1);v(6)=0;v(12)=0;v(23)=0;v(24)=o.groundEnabled;detail={};r=struct('v',v,'status','NOT_RUN');t=tic;
try
    a=tic;K=input_kernel55(y(20),y(21),H,o.thrustBounds,o.torqueLo,o.torqueHi);assert(K.ok,'REV55:Kernel');[lo,hi,ok]=inward51(y,H,o,K.nu(1),K.nu(2));assert(ok,'REV55:ActuatorRound');lo=[lo;o.torqueLo(1:2)];hi=[hi;o.torqueHi(1:2)];v(19)=1000*toc(a);
    a=tic;z=plane_eval55(y,zeros(3,1),tz,P,G,parts,o);v(20)=1000*toc(a);n=numel(z.h);nc=numel(parts);no=nc*(1+2*o.spheres);
    v(1:5)=[n;min([Inf;z.h]);min([Inf;z.lower]);min([Inf;z.h(1:no)]);min([Inf;z.h(1:nc)])];
    detail=row_details55(z,no,nan(n,1),nan(n,1));
    wn=[2*(s.raw(1)-y(20)-H*y(21))/H^2;s.raw(2:3)];a=tic;[w,status,nq]=qp_shared49(z.A,z.b,zeros(n,1),wn,lo,hi,cfg.paperControlScale(:));v(21)=1000*toc(a);v(22)=1000*toc(t);v(23)=nq;v(24)=o.groundEnabled;
    r.status=status;if isempty(w),r.v=v;return;end
    v(6:12)=[1;w;tz;sum(((w-wn)./cfg.paperControlScale(:)).^2);actuator_cert(y,w,H,o)];
    % Audit and numerical grid OUTSIDE online timing; reuse exactly the same plan.
    zz=plane_eval55(y,w,tz,P,G,parts,o,z.plan);v(13)=max([0;abs(zz.last-z.b-z.A*w)./(1+abs(zz.last))]);v(14)=max([0;abs(zz.lower-z.lower)./(1+abs(z.lower))]);v(15)=min([Inf;zz.last]);
    gh=z.h;gl=cellfun(@(a)min(a(1:end-1)),z.stages).';gl=gl(:);tg=Inf;
    for k=0:8,dt=H*k/8;x=project_state(propagate(y(1:19),[y(20);w(2:3);tz],[y(20),y(21),w(1)],s.P,dt,max(1,k)));yy=[x;y(20)+y(21)*dt+.5*w(1)*dt^2;y(21)+w(1)*dt];
        g=plane_eval55(yy,w,tz,P,G,parts,o,z.plan);gh=min(gh,g.h);gl=min(gl,reshape(cellfun(@(a)min(a(1:end-1)),g.stages),[],1));[~,rad]=rhs(x,[yy(20);w(2:3);tz],s.P);tg=min(tg,-P.mL*rad);
    end
    v(16:18)=[min([Inf;gh]);min([Inf;gl]);tg];
    detail=row_details55(zz,no,gh,gl);
catch ME,r.status=status_text(ME);if ~isfinite(v(22)),v(22)=1000*toc(t);end;end
r.v=v;
end

function detail=row_details55(zz,no,gh,gl)
detail={};n=numel(zz.stages);
    for k=1:n,st=nan(5,1);st(1:numel(zz.stages{k}))=zz.stages{k};nn=nan(3,1);et=NaN;if k<=no,nn=zz.normal(:,k);et=zz.plan.eta(k);end;detail{end+1}=[numline(k),',',csvtext(zz.names{k}),',',numline([numel(zz.stages{k})-1;st;nn;et;gh(k);gl(k)])];end
end

function o=defaults(o,cfg)
d=struct('thrustBounds',cfg.paperThrustBounds(:),'bodyDeg',cfgvalue48(cfg,'paperBodyTiltMaxDeg',40),'cableDeg',cfgvalue48(cfg,'paperCableTiltMaxDeg',60),'bodyLambda',cfgvalue48(cfg,'paperBodyTiltLambda',[2;2]),'cableLambda',cfgvalue48(cfg,'paperCableTiltLambda',[1;2;8;16]), ...
    'groundZ',cfgvalue48(cfg,'paperGroundZ',0),'groundLambda',cfgvalue48(cfg,'paperGroundLambda',cfg.paperLambda(:)),'tensionMin',cfgvalue48(cfg,'paperTensionMinN',0),'tensionLambda',cfgvalue48(cfg,'paperTensionLambda',cfgvalue48(cfg,'paperBodyTiltLambda',[2;2])),'groundEnabled',true,'tensionEnabled',true,'stride',80,'stressCounts',unique([1 4 8 16 cfg.paperMaxParts]),'stressSamples',3);
if ~isstruct(o)||~isscalar(o),error('SD52:Options','Expected scalar options struct');end
n=fieldnames(o);if any(~ismember(n,fieldnames(d))),error('SD52:Options','Unknown option');end
for j=fieldnames(d).',k=j{1};if ~isfield(o,k),o.(k)=d.(k);end,end
o.thrustBounds=double(o.thrustBounds(:));o.bodyLambda=double(o.bodyLambda(:));o.cableLambda=double(o.cableLambda(:));
validateattributes(o.thrustBounds,{'double'},{'numel',2,'real','finite','nonnegative'});assert(diff(o.thrustBounds)>0,'SD52:Bounds');
validateattributes(o.bodyDeg,{'numeric'},{'scalar','real','finite','>',0,'<',90});validateattributes(o.cableDeg,{'numeric'},{'scalar','real','finite','>',0,'<',90});
validateattributes(o.bodyLambda,{'double'},{'numel',2,'positive','finite'});validateattributes(o.cableLambda,{'double'},{'numel',4,'positive','finite'});
validateattributes(o.stressCounts,{'numeric'},{'vector','integer','positive','finite'});
for k={'stride','stressSamples'},validateattributes(o.(k{1}),{'numeric'},{'scalar','integer','positive','finite'});end
[o.torqueLo,o.torqueHi,o.schemePath]=read_torque48();
o.groundLambda=double(o.groundLambda(:));o.tensionLambda=double(o.tensionLambda(:));
validateattributes(o.groundZ,{'numeric'},{'scalar','real','finite'});
validateattributes(o.tensionMin,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(o.groundLambda,{'double'},{'numel',4,'positive','finite'});
validateattributes(o.tensionLambda,{'double'},{'numel',2,'positive','finite'});
for k={'groundEnabled','tensionEnabled'},validateattributes(o.(k{1}),{'logical','numeric'},{'scalar','real','finite'});assert(ismember(o.(k{1}),[0 1]),'SD52:Flag');end
end

function v=cfgvalue48(cfg,k,fallback)
if isfield(cfg,k),v=cfg.(k);else,v=fallback;end
end

function [lo,hi,p]=read_torque48()
% ID3 legacy has literal torque bounds, not exposed in cfg. Read active
% source, only accept the known explicit literal grammar; fail if changed.
p=which('CBF_REAL_SCHEME_03_PAPER_EXTENSION');if isempty(p),error('SD52:TorqueSource','ID3 source unavailable');end
s=fileread(p);num='([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)';
a=regexp(s,['lo\s*=\s*\[\s*nuLo\s*;\s*',num,'\s*;\s*',num,'\s*\]'],'tokens','once');
b=regexp(s,['hi\s*=\s*\[\s*nuHi\s*;\s*',num,'\s*;\s*',num,'\s*\]'],'tokens','once');
c=regexp(s,['tauZ\s*=\s*min\(\s*max\(\s*raw\(4\)\s*,\s*',num,'\s*\)\s*,\s*',num,'\s*\)'],'tokens','once');
if isempty(a)||isempty(b)||isempty(c),error('SD52:TorqueSource','Cannot read literal ID3 torque bounds; expose actual runtime config before testing');end
lo=[str2double(a(:));str2double(c{1})];hi=[str2double(b(:));str2double(c{2})];
if any(~isfinite([lo;hi]))||any(lo>=hi),error('SD52:TorqueSource','Invalid torque bounds');end
end

function P=physical(p)
P=struct('m',p(1),'J',p(2:4),'gravity',p(5),'mL',p(6),'cableL',p(7),'dstx',0,'dsty',0);
end

function G=geometry(cfg,P)
G=struct('collisionMode','cable_ellipsoid','cableAxisShort',cfg.paperCableRadiusM,'cableAxisLong',max(P.cableL/2+cfg.paperCableAxialMarginM,cfg.paperCableRadiusM),'cableCenterFromLoad',.5);
end

function p=stress_parts(p,n,y,P,G)
% Keep original parts; append distinct translated copies away from the vehicle.
% Synthetic stress not an obstacle detector test. No duplicated row shortcuts.
base=p(1);C=y(8:10)-G.cableCenterFromLoad*P.cableL*y(14:16);[~,a,~]=obs_shape(base);
for j=numel(p)+1:n,v=base;ang=2*pi*(j-1)/max(n,1);v.center=C+(max(a)+G.cableAxisLong+3+j*.15)*[cos(ang);sin(ang);.2];p(end+1)=v;end
end

function [c,a,R]=obs_shape(p)
c=double(p.center(:));kind='ellipsoid';if isfield(p,'kind'),kind=char(p.kind);end
switch kind
    case 'sphere',if isfield(p,'params'),a=p.params.radius;else,a=p.radius;end;a=repmat(a,3,1);R=eye(3);
    case 'ellipsoid',if isfield(p,'params'),a=p.params.axes;else,a=p.axes;end;if isfield(p,'R'),R=double(p.R);else,R=eye(3);end
    otherwise,error('SD52:Obstacle','Unsupported type');
end
a=double(a(:));if numel(c)~=3||numel(a)~=3||any(a<=0)||~isequal(size(R),[3,3])||any(~isfinite([c;a;R(:)])),error('SD52:Shape','Invalid geometry');end
end
% ---- scalar interval arithmetic; arrays are N-by-2 [lower, upper] ----

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

function [lo,hi,ok]=inward51(y,H,o,lo,hi)
% Move only as much as needed to certify the two endpoint inputs of the
% convex actuator set. Bisection towards a certified stopping witness;
% no arbitrary physical rate/acceleration cap. If witness fails, reject.
stop=-y(21)/H;w=[stop;min(o.torqueHi(1:2),max(o.torqueLo(1:2),zeros(2,1)))];
ok=stop>=lo&&stop<=hi&&actuator_cert(y,w,H,o);if ~ok,return;end
for side=1:2
    if side==1,a=lo;else,a=hi;end
    w(1)=a;if actuator_cert(y,w,H,o),continue;end
    bad=a;good=stop;
    for k=1:32
        v=a+sign(stop-a)*eps(max(abs(a),realmin));if v==a||abs(v-stop)>abs(a-stop),break;end
        a=v;w(1)=a;if actuator_cert(y,w,H,o),good=a;break;end;bad=a;
    end
    if good~=stop,if side==1,lo=good;else,hi=good;end;continue;end
    for k=1:64
        mid=bad+(good-bad)/2;if mid==bad||mid==good,break;end
        w(1)=mid;if actuator_cert(y,w,H,o),good=mid;else,bad=mid;end
    end
    if side==1,lo=good;else,hi=good;end
end
ok=lo<=hi;
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

function r=iv(x),x=double(x(:));r=[x,x];end

function r=out(l,h)
if any(~isfinite([l(:);h(:)]))||any(l(:)>h(:)),error('SD52:Interval','Invalid interval arithmetic');end
% One outward ulp at the computed value (at least realmin*eps relative
% to normal scale; avoids relying on exact directed hardware rounding).
r=[l-eps(max(abs(l),realmin)),h+eps(max(abs(h),realmin))];
if any(~isfinite(r(:))),error('SD52:Interval','Overflow');end
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
if any(b(:,1)<=0&b(:,2)>=0),error('SD52:Division','Interval denominator crosses zero');end
r=im(a,out(1./b(:,2),1./b(:,1)));
end

function c=cos_interval(deg)
% Known enclosure of mathematical pi, then Taylor alternating remainder.
% 0 < deg < 90; remainder of 20 even powers bounded by next magnitude.
p=out(pi,pi);x=id(im(iv(deg),p),iv(180));x2=isq(x);c=iv(1);v=iv(1);
for k=1:20,v=id(im(v,x2),iv((2*k-1)*(2*k)));if mod(k,2)==1,c=isub(c,v);else,c=ia(c,v);end,end
v=id(im(v,x2),iv(41*42));c=ia(c,[-v(2),0]);
end

function x=ascol(x,n)
if ~isnumeric(x)||~isreal(x)||numel(x)~=n,error('CBFBasis:SnapshotShape','Unexpected numeric vector shape');end
x=double(x(:));
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


function out=plane_eval55(y,w,tauZ,P,G,parts,opt,plan)
% REV55 additive evaluator; original support gap, unchanged legacy evaluator.
% plan is selected once at the control tick. PW eta and FIXED normal are
% held parameters on the interval. SHAPE and PW normals vary with state.
% All stages are CT HOCBF jets of that particular parameterized function.
% A*w+b is the last stage (direct input sensitivities); no SD certificate.
% Required opts: mode, spheres, extras, groundEnabled, tensionEnabled,
% obsLambda, bodyLambda, cableLambda, groundLambda, tensionLambda,
% bodyDeg, cableDeg, groundZ, tensionMin. G sphere radii supplied by caller.
if nargin<8,plan=[];end
y=double(y(:));w=double(w(:));
validateattributes(y,{'double'},{'numel',21,'finite','real'});
validateattributes(w,{'double'},{'numel',3,'finite','real'});
validateattributes(opt.obsLambda,{'numeric'},{'numel',4,'positive','finite'});
mode=char(opt.mode);assert(ismember(mode,{'CENTER','SHAPE','PW','FIXED_SHAPE'}),'REV55:Mode');
J=CBF_REAL_D2T_JET(y(1:19),y(20),y(21),w(1),[w(2:3);tauZ],P);X=J.extendedTaylor;
p=X(14:16,:);q=y(1:4);a=q(1);b=q(2);c=q(3);d=q(4);
R=[a*a+b*b-c*c-d*d,2*(b*c-a*d),2*(b*d+a*c);2*(b*c+a*d),a*a-b*b+c*c-d*d,2*(c*d-a*b);2*(b*d-a*c),2*(c*d+a*b),a*a-b*b-c*c+d*d];
e=R(:,3);Db=[zeros(3,1),-R(:,2)/P.J(1),R(:,1)/P.J(2)];
Dp=zeros(3,3);DL=Dp;for j=1:3
    Dp(:,j)=cross(-(double(j==1)*cross(p(:,1),e)+y(20)*cross(p(:,1),Db(:,j)))/(P.m*P.cableL),p(:,1));
    DL(:,j)=p(:,1)*(double(j==1)*dot(p(:,1),e)+y(20)*dot(p(:,1),Db(:,j)))/(P.m+P.mL);
end
fractions=G.cableCenterFromLoad;short=G.cableAxisShort;long=G.cableAxisLong;types={'CABLE'};
if opt.spheres
    fractions=[fractions,1,0];short=[short,G.droneSafetyRadius,G.loadSafetyRadius];long=[long,G.droneSafetyRadius,G.loadSafetyRadius];types=[types,{'DRONE_SPHERE','LOAD_SPHERE'}];
end
assert(all(isfinite([short,long,fractions]))&&all(short>0)&&all(long>=short),'REV55:Geometry');
nr=numel(parts)*numel(types);fresh=isempty(plan);
if fresh,plan=struct('mode',mode,'eta',nan(nr,1),'normal',nan(3,nr));else,assert(strcmp(plan.mode,mode)&&numel(plan.eta)==nr&&isequal(size(plan.normal),[3,nr]),'REV55:Plan');end
st={};names={};A=zeros(0,3);normals=zeros(3,nr);hT={};k=0;
for m=1:numel(types)
    C=X(8:10,:)-fractions(m)*P.cableL*p;DC=DL-fractions(m)*P.cableL*Dp;
    delta=long(m)^2-short(m)^2;Q=zeros(3,3,5);Q(:,:,1)=short(m)^2*eye(3);
    for i=0:4,for j=0:i,Q(:,:,i+1)=Q(:,:,i+1)+delta*p(:,j+1)*p(:,i-j+1).';end,end
    for j=1:numel(parts)
        k=k+1;[oc,Qo,label]=obstacle_shape(parts(j),j);D=C;D(:,1)=D(:,1)-oc;D0=D(:,1);
        if D0.'*D0<1e-12,error('REV55:Coincident','Centres too close for legacy normal domain');end
        if fresh
            switch mode
            case 'CENTER',plan.eta(k)=0;
            case {'SHAPE','FIXED_SHAPE'},plan.eta(k)=.5;
            case 'PW'
                % Scalar search only; positivity of actual gap is tested separately.
                f=@(eta)-eta*(1-eta)*(D0.'*(((1-eta)*Q(:,:,1)+eta*Qo)\D0));
                plan.eta(k)=fminbnd(f,0,1,optimset('Display','off','TolX',1e-8));
            end
        end
        eta=plan.eta(k);M=Q.*(1-eta);M(:,:,1)=M(:,:,1)+eta*Qo;
        if strcmp(mode,'CENTER'),M=zeros(3,3,5);M(:,:,1)=eye(3);end
        if strcmp(mode,'FIXED_SHAPE')
            if fresh,v0=M(:,:,1)\D0;plan.normal(:,k)=v0/norm(v0);end
            N=zeros(3,5);N(:,1)=plan.normal(:,k);
        else
            if strcmp(mode,'CENTER'),V=D;else
                V=zeros(3,5);
                for i=0:4
                    t=D(:,i+1);for l=1:i,t=t-M(:,:,l+1)*V(:,i-l+1);end
                    V(:,i+1)=M(:,:,1)\t;
                end
            end
            vn=norm(V(:,1));assert(vn>0&&isfinite(vn),'REV55:Normal');
            N=jetvector(V,jetinverse(jetsqrt(jetdot(V,V))));
            if fresh,plan.normal(:,k)=N(:,1);end
        end
        n=N(:,1);QoS=zeros(3,3,5);QoS(:,:,1)=Qo;
        sc=jetsqrt(jetquad(N,Q));so=jetsqrt(jetquad(N,QoS));
        h=jetdot(N,D)-sc-so;
        % Differential of the exact support gap, including moving normal.
        pn=p(:,1).'*n;gD=n;gp=-delta*pn*n/sc(1);
        if ~strcmp(mode,'FIXED_SHAPE')
            gN=D0-Q(:,:,1)*n/sc(1)-Qo*n/so(1);
            av=M(:,:,1).'\((eye(3)-n*n.')*gN/vn);
            gD=gD+av;
            if ~strcmp(mode,'CENTER'),gp=gp-(1-eta)*delta*((p(:,1).'*V(:,1))*av+(av.'*p(:,1))*V(:,1));end
        end
        A(end+1,:)=gD.'*DC+gp.'*Dp; %#ok<AGROW>
        st{end+1}=stages(h,opt.obsLambda);hT{end+1}=h;names{end+1}=sprintf('%s_%d_%s',types{m},j,label);normals(:,k)=n;
    end
end
if opt.extras
    qb=zeros(1,5);sg=[1,-1,-1,1];for j=1:4,qb=qb+sg(j)*mul_series49(X(j,:),X(j,:));end
    qb(1)=qb(1)-cosd(opt.bodyDeg);hc=-p(3,:);hc(1)=hc(1)-cosd(opt.cableDeg);
    st=[st,{stages(qb,opt.bodyLambda),stages(hc,opt.cableLambda)}];hT=[hT,{qb,hc}];names=[names,{'BODY_TILT','CABLE_TILT'}];A=[A;Db(3,:);-Dp(3,:)];
    if opt.groundEnabled
        hg=X(10,:);hg(1)=hg(1)-opt.groundZ;hq=hg-P.cableL*p(3,:);
        st=[st,{stages(hg,opt.groundLambda),stages(hq,opt.groundLambda)}];hT=[hT,{hg,hq}];names=[names,{'GROUND_LOAD','GROUND_BODY'}];A=[A;DL(3,:);DL(3,:)-P.cableL*Dp(3,:)];
    end
    if opt.tensionEnabled
        [hh,BB]=extra_point51(X,P,struct('groundEnabled',false,'tensionEnabled',true,'tensionMin',opt.tensionMin),DL,Dp,Db);
        st=[st,{stages(hh{1},opt.tensionLambda)}];hT=[hT,hh];names=[names,{'TENSION_MIN'}];A=[A;BB];
    end
end
last=zeros(numel(st),1);low=[];hs=last;for i=1:numel(st),last(i)=st{i}(end);hs(i)=st{i}(1);low=[low;st{i}(1:end-1)];end
out=struct('A',A,'b',last-A*w,'last',last,'lower',low,'h',hs,'stages',{st},'hTaylor',{hT},'names',{names},'plan',plan,'normal',normals,'certifiedSampledData',false);
assert(all(isfinite([A(:);out.b;low(:);normals(:)])),'REV55:Nonfinite');
end
function s=stages(h,l)
l=l(:);assert(ismember(numel(l),[2,4])&&all(isfinite(l))&&all(l>0),'REV55:Gains');d=h(:).*factorial((0:4)');s=zeros(numel(l)+1,1);s(1)=d(1);pol=1;for i=1:numel(l),pol=conv(pol,[l(i),1]);s(i+1)=pol*d(1:i+1);end
end
function [c,Q,label]=obstacle_shape(part,j)
if ~isfield(part,'center'),error('CBFD2T:ObstacleCentre','centre missing');end
c=double(part.center(:));
validateattributes(c,{'double'},{'vector','numel',3,'finite','real'});
kind='ellipsoid';if isfield(part,'kind'),kind=char(part.kind);end
switch kind
    case 'sphere'
        if isfield(part,'params'),a=part.params.radius;else,a=part.radius;end
        a=repmat(a,3,1);R=eye(3);
    case 'ellipsoid'
        if isfield(part,'params'),a=part.params.axes;else,a=part.axes;end
        if isfield(part,'R'),R=double(part.R);else,R=eye(3);end
    otherwise
        error('CBFD2T:ObstacleType','Unknown obstacle kind %s',kind);
end
a=double(a(:));
validateattributes(a,{'double'},{'vector','numel',3,'positive','finite'});
if ~isequal(size(R),[3,3]) || any(~isfinite(R(:))) || ...
        norm(R.'*R-eye(3),'fro')>1e-5 || det(R)<0
    error('CBFD2T:ObstacleRotation','Invalid obstacle rotation');
end
Q=R*diag(a.^2)*R.';
label=sprintf('obstacle%d',j);
if isfield(part,'obstacleName'),label=char(part.obstacleName);
elseif isfield(part,'name'),label=char(part.name);end
end
function s=jetdot(A,B)
s=zeros(1,5);
for k=0:4
    for j=0:k,s(k+1)=s(k+1)+A(:,j+1).'*B(:,k-j+1);end
end
end
function V=jetvector(A,b)
V=zeros(3,5);
for k=0:4
    for j=0:k,V(:,k+1)=V(:,k+1)+A(:,j+1)*b(k-j+1);end
end
end
function s=jetquad(n,Q)
V=zeros(3,5);
for k=0:4
    for j=0:k,V(:,k+1)=V(:,k+1)+Q(:,:,j+1)*n(:,k-j+1);end
end
s=jetdot(n,V);
end
function r=jetsqrt(s)
if s(1)<=0 || ~isfinite(s(1)),error('CBFD2T:Support','Invalid square root');end
r=zeros(1,5);r(1)=sqrt(s(1));
for k=1:4
    crossTerms=0;
    for j=1:k-1,crossTerms=crossTerms+r(j+1)*r(k-j+1);end
    r(k+1)=(s(k+1)-crossTerms)/(2*r(1));
end
end
function b=jetinverse(a)
if abs(a(1))<1e-12,error('CBFD2T:Inverse','Inverse singular');end
b=zeros(1,5);b(1)=1/a(1);
for k=1:4
    acc=0;
    for j=1:k,acc=acc+a(j+1)*b(k-j+1);end
    b(k+1)=-acc/a(1);
end
end
function c=mul_series49(a,b)
c=zeros(size(a));for k=1:size(a,2),c(:,k)=sum(a(:,1:k).*b(:,k:-1:1),2);end
end
function [h,B]=extra_point51(X,P,o,DL,Dp,Db)
% Ordinary Taylor coefficients. Ground positions z_L and z_Q=z_L-L*p_z.
% Tension N=-mL*radial. Relative degree 4/4/2 in the extended system.
h={};B=zeros(0,3);if ~o.groundEnabled&&~o.tensionEnabled,return;end;N=size(X,2);p=X(14:16,:);T=X(20,:);
if o.groundEnabled
    a=X(10,:);a(1)=a(1)-o.groundZ;c=a-P.cableL*p(3,:);h=[h,{a,c}];
    if nargin>=6,B=[DL(3,:);DL(3,:)-P.cableL*Dp(3,:)];end
end
if o.tensionEnabled
    q=X(1:4,:);a=q(1,:);b=q(2,:);c=q(3,:);d=q(4,:);
    e=[2*(mul_series49(b,d)+mul_series49(a,c));2*(mul_series49(c,d)-mul_series49(a,b));mul_series49(a,a)-mul_series49(b,b)-mul_series49(c,c)+mul_series49(d,d)];
    wl=X(17:19,:);v=zeros(3,N);for i=1:3,j=mod(i,3)+1;k=mod(i+1,3)+1;v(i,:)=mul_series49(wl(j,:),p(k,:))-mul_series49(wl(k,:),p(j,:));end
    pe=zeros(1,N);vv=pe;for i=1:3,pe=pe+mul_series49(p(i,:),e(i,:));vv=vv+mul_series49(v(i,:),v(i,:));end
    f=-P.mL/(P.m+P.mL)*(mul_series49(T,pe)-P.m*P.cableL*vv);f(1)=f(1)-o.tensionMin;h=[h,{f}];
    if nargin>=6,B=[B;-P.mL/(P.m+P.mL)*[dot(p(:,1),e(:,1)),T(1)*p(:,1).'*Db(:,2:3)]];end
end
end


function out=input_kernel55(T,q,H,bounds,torqueLo,torqueHi)
% Ideal constant-ddT hold plus one-cycle stopping invariant subset.
% Exact real arithmetic formula, binary64 boundary must be checked separately.
validateattributes(T,{'numeric'},{'scalar','real','finite'});validateattributes(q,{'numeric'},{'scalar','real','finite'});
validateattributes(H,{'numeric'},{'scalar','positive','finite'});validateattributes(bounds,{'numeric'},{'numel',2,'nonnegative','finite'});
validateattributes(torqueLo,{'numeric'},{'numel',3,'finite','real'});validateattributes(torqueHi,{'numeric'},{'numel',3,'finite','real'});
torqueLo=torqueLo(:);torqueHi=torqueHi(:);bounds=bounds(:);
assert(bounds(2)>bounds(1)&&all(torqueHi(:)>torqueLo(:)),'REV55:Bounds');
[a,b,lo,hi,ok,ql,qh]=kernel(T,q,H,bounds);
out=struct('hold',[a;b],'nu',[lo;hi],'qRange',[ql;qh],'ok',ok,'lo',[lo;torqueLo(1:2)],'hi',[hi;torqueHi(1:2)],'tauZ',[torqueLo(3);torqueHi(3)],'physicalBandwidthCertified',false,'roundedBoundsCertified',false);
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
