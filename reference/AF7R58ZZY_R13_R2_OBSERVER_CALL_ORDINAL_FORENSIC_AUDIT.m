function OUT = AF7R58ZZY_R13_R2_OBSERVER_CALL_ORDINAL_FORENSIC_AUDIT()
% AF7R58ZZY-R13-R2 | OBSERVER CALL-ORDINAL / SHAPE FORENSIC
% No repair. No added warmup. Existing 2-event observer preflight and real
% Flight are observed only.
clc;
scriptDir=fileparts(mfilename('fullpath')); addpath(scriptDir,'-begin'); rehash;
EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';
EXPECTED_R8_SHA='5ca886499192ee309c0bfd7a39c2094de708e4a85104fb7a54e99b2b987e97df'; EXPECTED_R5_SHA='2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18'; EXPECTED_R13_SHA='324ddeeb0d27dea14604984bba7d198f9acefcffd86f23aac64eee6cd52b6bb5';
srcPath=which('REPLANNING_SYSTEM'); runnerPath=which('run_flight_headless');
r8Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R8R2_DIAG.m');
r5Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R5R4_DIAG.m');
r13Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R13_DIAG.m');
assert(all(cellfun(@isfile,{srcPath,runnerPath,r8Path,r5Path,r13Path})),'[R13] required file missing.');
sourceSHA=sha256_file_r13(srcPath); runnerSHA=sha256_file_r13(runnerPath); r8SHA=sha256_file_r13(r8Path); r5SHA=sha256_file_r13(r5Path); r13SHA=sha256_file_r13(r13Path);
assert(strcmpi(sourceSHA,EXPECTED_SOURCE_SHA),'[R13] production source SHA mismatch.');
assert(strcmpi(runnerSHA,EXPECTED_RUNNER_SHA),'[R13] runner SHA mismatch.');
assert(strcmpi(r8SHA,EXPECTED_R8_SHA),'[R13] R8-R2 SHA mismatch.');
assert(strcmpi(r5SHA,EXPECTED_R5_SHA),'[R13] R5-R4 SHA mismatch.');
assert(strcmpi(r13SHA,EXPECTED_R13_SHA),'[R13] R13 SHA mismatch.');
runnerText=fileread(runnerPath); sem=assert_runner_semantics_r13(runnerText);
fprintf('====================================================================================================\n');
fprintf(' AF7R58ZZY-R13-R2 | OBSERVER CALL-ORDINAL / SHAPE FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] diagnosticOnly=1 | addedWarmup=0 | productionModified=0 | phaseShift=0\n');
fprintf(' source SHA = %s\n runner SHA = %s\n R13 SHA    = %s\n',sourceSHA,runnerSHA,r13SHA);
root=fileparts(srcPath); outDir=tempname(root); mkdir(outDir);
reportPath=fullfile(outDir,'AF7R58ZZY_R13_R2_REPORT.txt'); outPath=fullfile(outDir,'AF7R58ZZY_R13_R2_OUT.mat'); rawPath=fullfile(outDir,'AF7R58ZZY_R13_R2_RAW_CAPTURE.mat');
Run=run_flight_r13(root,scriptDir,runnerText); runError=Run.error; F=struct(); diagError=struct('identifier','','message','');
try
    assert(Run.closed,'[R13] flight did not close.');
    F=analyze_r13(Run.obj);
catch ME
    diagError.identifier=ME.identifier; diagError.message=ME.message;
end
diagnosticClosed=Run.closed && isempty(runError.message) && isempty(diagError.message) && isfield(F,'closed') && F.closed;
if diagnosticClosed, status='OBSERVER_CALL_ORDINAL_FORENSIC_R13_COMPLETE'; else, status='OBSERVER_CALL_ORDINAL_FORENSIC_R13_INCOMPLETE_FAIL_CLOSED'; end
RAW=struct('status',status,'forensic',F,'run_error',runError,'diag_error',diagError,'authority',struct('source_sha',sourceSHA,'runner_sha',runnerSHA,'r8_sha',r8SHA,'r5_sha',r5SHA,'r13_sha',r13SHA,'runner_semantics',sem)); save(rawPath,'RAW','-v7.3');
OUT=struct('status',status,'diagnostic_closed',diagnosticClosed,'production_modified',false,'added_warmup',0,'new_arming_work',0,'new_takeoff_work',0,'new_landing_work',0,'forensic',F,'run_error',runError,'diag_error',diagError,'raw_capture_path',rawPath,'report_path',reportPath); save(outPath,'OUT','-v7.3');
fid=fopen(reportPath,'w'); assert(fid>=0); c=onCleanup(@()safe_fclose_r13(fid));
say_r13(fid,'status=%s',status); say_r13(fid,'diagnosticClosed=%d',diagnosticClosed); say_r13(fid,'productionModified=0'); say_r13(fid,'addedWarmup=0'); say_r13(fid,'newArmingWork=0'); say_r13(fid,'newTakeoffWork=0'); say_r13(fid,'newLandingWork=0'); say_r13(fid,'sourceSHA=%s',sourceSHA); say_r13(fid,'runnerSHA=%s',runnerSHA); say_r13(fid,'r13SHA=%s',r13SHA); say_r13(fid,'runnerSemantics=%d/%d',sem.pass_count,sem.total_count); say_r13(fid,'raw=%s',rawPath); say_r13(fid,'out=%s',outPath);
if diagnosticClosed
    say_r13(fid,''); say_r13(fid,'[CALL_ORDINAL_TIMING]');
    say_r13(fid,'columns=context t updateOrdinal oneOrdinal contractOrdinal total one contract residual hasPrev h');
    for k=1:size(F.rows,1)
        say_r13(fid,'row=%s %.9g %d %d %d %.9g %.9g %.9g %.9g %d %.9g',F.context{k},F.rows(k,1),round(F.rows(k,34)),round(F.rows(k,35)),round(F.rows(k,36)),F.rows(k,2),F.rows(k,9),F.rows(k,14),F.residual(k),round(F.rows(k,27)),F.rows(k,30));
    end
    say_r13(fid,''); say_r13(fid,'[PREFLIGHT_VS_LIVE_SHAPE]');
    say_r13(fid,'preflightHasPrevEvents=%d',F.preflight_hasprev_count); say_r13(fid,'liveHasPrevEvents=%d',F.live_hasprev_count); say_r13(fid,'allInputShapeExact=%d',F.all_input_shape_exact); say_r13(fid,'allOutputShapeExact=%d',F.all_output_shape_exact); say_r13(fid,'staticCacheSame=%d',F.static_cache_same); say_r13(fid,'hSame=%d',F.h_same);
    say_r13(fid,'columns=t updateOrdinal oneOrdinal contractOrdinal inputShapeExact outputShapeExact exactReplay hotOneMs hotContractMs hotSumMs liveToHotRatio');
    for k=1:numel(F.live_compare)
        E=F.live_compare(k); say_r13(fid,'row=%.9g %d %d %d %d %d %d %.9g %.9g %.9g %.9g',E.t,E.update_ordinal,E.one_ordinal,E.contract_ordinal,E.input_shape_exact,E.output_shape_exact,E.exact_replay,E.hot_one_ms,E.hot_contract_ms,E.hot_sum_ms,E.live_to_hot_ratio);
    end
    say_r13(fid,''); say_r13(fid,'[SPIKE_ORDINALS]'); say_r13(fid,'oneStepSpikeOrdinals=%s',mat2str(F.one_spike_ordinals)); say_r13(fid,'contractSpikeOrdinals=%s',mat2str(F.contract_spike_ordinals)); say_r13(fid,'observerSpikeUpdateOrdinals=%s',mat2str(F.update_spike_ordinals)); say_r13(fid,'classification=%s',F.classification); say_r13(fid,'nextAction=%s',F.next_action);
end
if ~isempty(runError.message), say_r13(fid,'runError=%s | %s',runError.identifier,runError.message); end
if ~isempty(diagError.message), say_r13(fid,'diagError=%s | %s',diagError.identifier,diagError.message); end
fclose(fid); clear c;
fprintf('\n[D. RESULT]\n status = %s\n',status); if diagnosticClosed, fprintf(' shape exact = %d | classification = %s\n',F.all_input_shape_exact,F.classification); end
fprintf('RETURN THESE THREE FILES:\n %s\n %s\n %s\n',reportPath,outPath,rawPath);
if ~diagnosticClosed, error('[AF7R58ZZY-R13] observer forensic did not close.'); end
end

function R=run_flight_r13(root,scriptDir,runnerText)
R=struct('closed',false,'obj',[],'error',struct('identifier','','message',''));
try, evalin('base','clearvars'); catch, end
try, evalin('base','clear REPLANNING_SYSTEM_R13_DIAG REPLANNING_SYSTEM_R8R2_DIAG REPLANNING_SYSTEM_R5R4_DIAG'); catch, end
rehash;
oldCtor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
newCtor='replanner_instance = REPLANNING_SYSTEM_R13_DIAG(agent,nominal_ref,replan_opts);';
assert(count_token_r13(runnerText,oldCtor)==1,'[R13] runner constructor anchor !=1.');
txt=strrep(runnerText,oldCtor,newCtor);
pre=sprintf('addpath(''%s'',''-begin'');rehash;%c',strrep(scriptDir,'''',''''''),10); txt=[pre txt];
tmp=fullfile(root,'AF7R58ZZY_R13_R2_TEMP_RUNNER.m'); write_utf8_r13(tmp,txt); c=onCleanup(@()delete_if_exists_r13(tmp)); %#ok<NASGU>
try
    cmd=sprintf('run(''%s'')',strrep(tmp,'''','''''')); evalin('base',cmd); obj=evalin('base','replanner_instance');
    assert(isa(obj,'REPLANNING_SYSTEM_R13_DIAG')&&isa(obj,'REPLANNING_SYSTEM'),'[R13] diagnostic class contract lost.');
    R.obj=obj; R.closed=obj.af7r65_event_count>=3 && obj.af7r65_obs_count>=3;
catch ME
    R.error.identifier=ME.identifier; R.error.message=ME.message;
end
end

function F=analyze_r13(obj)
F=struct('closed',false); n=double(obj.af7r65_event_count); assert(n>=3,'[R13] too few observer events.');
events=obj.af7r65_events(1:n); ring=obj.af7r65_obs_ring(1:double(obj.af7r65_obs_count),:); assert(size(ring,2)==36,'[R13] ring width mismatch.');
ctx=cell(size(ring,1),1); residual=nan(size(ring,1),1);
for i=1:size(ring,1)
    if ring(i,33)==1, ctx{i}='PREFLIGHT'; elseif ring(i,33)==2, ctx{i}='LIVE'; else, ctx{i}='OTHER'; end
    idx=[3 4 5 6 7 8 9 14 20 21 22 23 24 25 26]; z=ring(i,idx); z=z(isfinite(z)); residual(i)=ring(i,2)-sum(z);
end
F.rows=ring; F.context=ctx; F.residual=residual;
pre=[]; live=struct([]);
for i=1:n
    E=events{i};
    if E.context_code==1 && E.has_prev, pre=E; end
    if E.context_code==2 && E.has_prev, if isempty(live), live=E; else, live(end+1)=E; end, end %#ok<AGROW>
end
assert(~isempty(pre)&&~isempty(live),'[R13] preflight/live hasPrev events missing.'); F.preflight_hasprev_count=1; F.live_hasprev_count=numel(live);
preIn=effective_input_sig_r13(pre); preOut=join_sig_r13({pre.F,pre.K});
tmpl=struct('t',NaN,'update_ordinal',0,'one_ordinal',0,'contract_ordinal',0,'input_shape_exact',false,'output_shape_exact',false,'exact_replay',false,'hot_one_ms',NaN,'hot_contract_ms',NaN,'hot_sum_ms',NaN,'live_to_hot_ratio',NaN); C=repmat(tmpl,1,numel(live));
allIn=true; allOut=true; staticSame=true; hSame=true;
for i=1:numel(live)
    E=live(i); ineq=strcmp(preIn,effective_input_sig_r13(E)); outeq=strcmp(preOut,join_sig_r13({E.F,E.K})); allIn=allIn&&ineq; allOut=allOut&&outeq; staticSame=staticSame&&(E.static_cache_valid==pre.static_cache_valid); hSame=hSame&&(abs(E.h-pre.h)<=64*eps(max([1 abs(E.h) abs(pre.h)])));
    F1=obj.observer_one_step_support(E.h,E.uPrev,E.I_before); K1=obj.observer_contract_two_events(E.C0,E.C1,F1,E.xhat); ex=isequaln(F1,E.F)&&isequaln(K1,E.K);
    nrep=30; a=zeros(1,nrep); b=zeros(1,nrep);
    for q=1:nrep
        T=tic; Fr=obj.observer_one_step_support(E.h,E.uPrev,E.I_before); a(q)=1e3*toc(T);
        T=tic; Kr=obj.observer_contract_two_events(E.C0,E.C1,Fr,E.xhat); b(q)=1e3*toc(T);
        ex=ex&&isequaln(Fr,E.F)&&isequaln(Kr,E.K);
    end
    liveRow=find(abs(ring(:,1)-E.t)<=64*eps(max(1,abs(E.t))) & ring(:,33)==2,1,'first'); liveTotal=ring(liveRow,2);
    C(i)=struct('t',E.t,'update_ordinal',E.update_ordinal,'one_ordinal',E.one_ordinal,'contract_ordinal',E.contract_ordinal,'input_shape_exact',ineq,'output_shape_exact',outeq,'exact_replay',ex,'hot_one_ms',median(a),'hot_contract_ms',median(b),'hot_sum_ms',median(a)+median(b),'live_to_hot_ratio',liveTotal/max(realmin,median(a)+median(b)));
end
F.live_compare=C; F.all_input_shape_exact=allIn; F.all_output_shape_exact=allOut; F.static_cache_same=staticSame; F.h_same=hSame;
prelive=ring(ring(:,33)==2,:); oneMask=isfinite(prelive(:,9)) & prelive(:,9)>1.0; conMask=isfinite(prelive(:,14)) & prelive(:,14)>1.0; updMask=isfinite(prelive(:,2)) & prelive(:,2)>5.0;
F.one_spike_ordinals=unique(prelive(oneMask,35)).'; F.contract_spike_ordinals=unique(prelive(conMask,36)).'; F.update_spike_ordinals=unique(prelive(updMask,34)).';
allExact=all([C.exact_replay]); hotMax=max([C.hot_sum_ms]);
if allIn&&allOut&&staticSame&&hSame&&allExact&&hotMax<0.25 && ~isempty(F.one_spike_ordinals)
    F.classification='SAME_SHAPE_EARLY_CALL_ORDINAL_JIT_OR_ALLOCATOR_STRONGLY_SUPPORTED';
    F.next_action='AOT_COMPILED_OBSERVER_NUMERIC_KERNEL_SHADOW__NO_ADDED_WARMUP';
elseif ~allIn
    F.classification='PREFLIGHT_LIVE_INPUT_SHAPE_SPECIALIZATION_MISMATCH';
    F.next_action='CLOSE_SHAPE_MISMATCH_BEFORE_ANY_OPTIMIZATION';
else
    F.classification='OBSERVER_FIRST_USE_CAUSE_REMAINS_OPEN';
    F.next_action='DO_NOT_REPAIR__DEEPER_ALLOCATOR_OR_CALLSITE_FORENSIC';
end
F.closed=allExact && all(isfinite(ring(:,2))) && numel(C)>=1;
end

function s=effective_input_sig_r13(E)
I=E.I_before; Ieff=struct('hidden',I.hidden,'q_abs_ub_current',I.q_abs_ub_current,'pT_norm_ub',I.pT_norm_ub,'true_mL_interval',I.true_mL_interval);
X=E.xhat; Xeff=struct('p',X.p,'q',X.q,'pL',X.pL,'pT',X.pT,'vL',X.vL,'wL',X.wL,'w',X.w);
s=join_sig_r13({Ieff,E.C0,E.C1,Xeff,E.uPrev});
end

function s=join_sig_r13(C)
p=cell(1,numel(C)); for i=1:numel(C), p{i}=shape_sig_r13(C{i},0); end; s=strjoin(p,'||');
end
function s=shape_sig_r13(v,d)
if d>8, s='DEPTH'; return; end
sz=size(v); szs=sprintf('%dx',sz); s=[class(v) '[' szs ']'];
if isstruct(v)
    fn=fieldnames(v); s=[s '{'];
    if isscalar(v)
        for k=1:numel(fn), s=[s fn{k} ':' shape_sig_r13(v.(fn{k}),d+1) ';']; end %#ok<AGROW>
    else
        for k=1:numel(fn), s=[s fn{k} ';']; end %#ok<AGROW>
    end
    s=[s '}'];
elseif iscell(v)
    s=[s '<']; for k=1:min(numel(v),8), s=[s shape_sig_r13(v{k},d+1) ';']; end; s=[s '>'];
end
end

function S=assert_runner_semantics_r13(txt)
checks={ ...
    'dt_25ms','dt = 0.025;'; 'terminal_50s','te = 50;'; ...
    'reference_only_commit','AVOIDANCE_COMMIT_POLICY = "REFERENCE_ONLY";'; ...
    'auto_commit_on','ENABLE_AUTO_REPLAN_COMMIT = true;'; 'plant_true_mL_014','plant_loadmass = 0.14;'; ...
    'cable_length_2m','agent.parameter.set("cableL",2.0);'; 'line_reference','{"gen_ref_line", {'; ...
    'line_speed_15','"velocity", 1.5,'; 'line_direction_pos_z','"direction", [0, 0, 1]'; ...
    'sensor_range_15m','replan_opts.trigger_dist = 15.0;'; 'payload_radius_05','replan_opts.r_load       = 0.5;'; ...
    'uav_radius_05','replan_opts.r_drone      = 0.5;'; 'cable_radius_05','replan_opts.cable        = 0.5;'; ...
    'backend_1','replan_opts.generator_backend_flag = 1;'; ...
    'observer_on','replan_opts.ENABLE_OBSERVER_HARD_SUPPORT = true;'; ...
    'observer_sim_truth_mode','replan_opts.OBSERVER_HARD_SUPPORT_MODE = "SIM_NUMERIC_TRUTH";'; ...
    'observer_true_mass_binding','replan_opts.OBSERVER_SIM_TRUE_ML = plant_loadmass;'; ...
    'takeoff_end_5s','TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5)'; 'flight_start_5s','if time.t < 5.0'; ...
    'landing_start_47s','elseif time.t < 47.0'; ...
    'flight_ref_allocation','agent.cha_allocation.f.reference = ["timevarying","sload"];'; ...
    'flight_est_allocation','agent.cha_allocation.f.estimator = ["ekf","loadstate"];'};
pass=false(size(checks,1),1); for k=1:size(checks,1), pass(k)=contains(txt,checks{k,2}); end
if ~pass(end), pass(end)=contains(txt,'agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);'); end
w0=strfind(txt,'while time.t <= time.te'); w1=strfind(txt,'time.t = time.t + dt;'); orderOK=false;
if numel(w0)==1 && ~isempty(w1)
    q=w1(find(w1>w0,1,'first')); if ~isempty(q)
        blk=txt(w0:q); tok={'do_prop_local(time,cha,cha,logger,env,agent,"sensor",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"estimator",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"reference",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"controller",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"input_transform",1,true);'};
        pos=nan(1,numel(tok)); for j=1:numel(tok), z=strfind(blk,tok{j}); if numel(z)==1,pos(j)=z;end,end %#ok<STREMP>
        orderOK=all(isfinite(pos))&&all(diff(pos)>0);
    end
end
S=struct('closed',all([pass;orderOK]),'pass_count',nnz([pass;orderOK]),'total_count',numel([pass;orderOK]));
assert(S.closed,'[R13] current runner target semantics changed.');
end
function n=count_token_r13(txt,token), n=numel(strfind(txt,token)); end %#ok<STREMP>
function h=sha256_file_r13(path)
md=java.security.MessageDigest.getInstance('SHA-256'); fis=java.io.FileInputStream(java.io.File(path)); c=onCleanup(@()fis.close()); %#ok<NASGU>
buf=zeros(1,1024*1024,'int8'); while true, n=fis.read(buf,0,numel(buf)); if n<0, break; end; md.update(buf(1:n)); end
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
function write_utf8_r13(path,txt), fid=fopen(path,'w','n','UTF-8'); assert(fid>=0); c=onCleanup(@()fclose(fid)); fprintf(fid,'%s',txt); end %#ok<NASGU>
function delete_if_exists_r13(path), try, if isfile(path), delete(path); end, catch, end, end
function say_r13(fid,fmt,varargin), fprintf(fid,'%s%c',sprintf(fmt,varargin{:}),10); end
function safe_fclose_r13(fid), try, if fid>0, fclose(fid); end, catch, end, end
