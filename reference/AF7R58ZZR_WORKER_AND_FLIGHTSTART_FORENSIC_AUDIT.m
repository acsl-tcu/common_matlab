function OUT = AF7R58ZZR_WORKER_AND_FLIGHTSTART_FORENSIC_AUDIT()
% AF7R58ZZR
% Read-only audit after AF7R58ZZQ-R2 / same-flight rerun.
%
% Goals:
%   1) classify the Flight-start runtime spike separately from replanning;
%   2) determine how much of the successful replan attempt is native callback
%      versus surrounding context/certificate/commit preparation;
%   3) decide the correct worker boundary before changing production.
%
% NO production/source/state modification.
% NO synthetic obstacle.
% H=4 s contract is only audited, never changed.

EXPECTED_SHA='bc97df3a0a7341ac5605d52f4dd94923bfd831a11be535450666ec6d10ff41ec';

OUT=struct('schema','AF7R58ZZR_WORKER_FLIGHTSTART_FORENSIC_V1', ...
    'production_modified',false,'status','OPEN','error',struct());

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZR | WORKER BOUNDARY + FLIGHT-START SPIKE FORENSIC AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sourceBound=1 | noCommit=1 | noSyntheticReplan=1\n');
fprintf('[CONTRACT] fixedHorizon4sRequired=1 | extendedPreviewForbidden=1\n');

try
    %% A. Exact source authority.
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR),'[AF7R58ZZR] REPLANNING_SYSTEM unresolved.');
    OUT.source_sha=file_hash(pR);
    OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);
    assert(OUT.source_exact, ...
        '[AF7R58ZZR] source differs from exact AF7R58ZZQ-R2 authority.');

    R=fileread(pR);
    OUT.contract=struct();
    OUT.contract.fixed_horizon_4s=contains(R,'horizon_s     = 4.0;');
    OUT.contract.no_extended_preview= ...
        ~contains(R,'prepare_early_intervention_reference_cache') && ...
        ~contains(R,'AF7R58ZZP theorem-derived EARLY intervention scheduler');
    OUT.contract.one_cycle_sync_attempt= ...
        count_exact(R,'attempt_budget_s=double(obj.prediction_dt);')==2;
    OUT.contract.finite_sensing= ...
        contains(R,'The generator receives ONLY sensor-known obstacles:') && ...
        contains(R,'''detected_only_obstacle_contract'',true') && ...
        contains(R,'Initial known set is empty.');
    OUT.contract.closed=all(structfun(@logical,OUT.contract));

    fprintf('\n[A. SOURCE CONTRACT]\n');
    fprintf(' source SHA exact                    = %d\n',OUT.source_exact);
    fprintf(' fixed H=4 s                         = %d\n',OUT.contract.fixed_horizon_4s);
    fprintf(' extended preview absent             = %d\n',OUT.contract.no_extended_preview);
    fprintf(' one-cycle sync attempt architecture = %d\n',OUT.contract.one_cycle_sync_attempt);
    fprintf(' finite-sensing contract             = %d\n',OUT.contract.finite_sensing);

    %% B. Runtime profile.
    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZR] AF7R58ZZB_RUNTIME_PROFILE absent. Rerun the same flight first.');
    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');

    [names,getv,nrow]=profile_adapter(P);
    must=["t","phase","online_before_plant_s","full_iteration_s", ...
          "reference_s","gen_request_delta","gen_deadline_delta", ...
          "gen_commit_delta","gen_last_cycle_critical_ms","gen_last_callback_ms"];
    miss=must(~arrayfun(@(x)any(names==x),must));
    assert(isempty(miss),'[AF7R58ZZR] profile missing: %s',strjoin(cellstr(miss),', '));

    t=double(getv("t")); t=t(:);
    phase=getv("phase"); phase=char(phase(:));
    online=double(getv("online_before_plant_s")); online=online(:)*1e3;
    full=double(getv("full_iteration_s")); full=full(:)*1e3;
    refms=double(getv("reference_s")); refms=refms(:)*1e3;
    req=double(getv("gen_request_delta")); req=req(:);
    fail=double(getv("gen_deadline_delta")); fail=fail(:);
    commit=double(getv("gen_commit_delta")); commit=commit(:);
    attempt=double(getv("gen_last_cycle_critical_ms")); attempt=attempt(:);
    callback=double(getv("gen_last_callback_ms")); callback=callback(:);

    assert(numel(t)==nrow && numel(phase)==nrow);

    flight=find(phase=='f' & isfinite(t));
    assert(~isempty(flight),'[AF7R58ZZR] no Flight rows in runtime profile.');
    iF0=flight(1);
    tF0=t(iF0);

    reqIdx=find(phase=='f' & req>0);
    OUT.flight_start=struct();
    OUT.flight_start.index=iF0;
    OUT.flight_start.time_s=tF0;
    OUT.flight_start.online_ms=online(iF0);
    OUT.flight_start.full_ms=full(iF0);
    OUT.flight_start.reference_ms=refms(iF0);
    OUT.flight_start.request_delta=req(iF0);

    % Window is intentionally short and descriptive only; it is not a safety gate.
    fwin=flight(t(flight)<=tF0+0.50+1e-12);
    [mxFull,jf]=safe_max_index(full(fwin));
    iFpeak=fwin(jf);
    [mxOnline,jo]=safe_max_index(online(fwin));
    iOpeak=fwin(jo);

    OUT.flight_start.window_0p5s_count=numel(fwin);
    OUT.flight_start.peak_full_index=iFpeak;
    OUT.flight_start.peak_full_time_s=t(iFpeak);
    OUT.flight_start.peak_full_ms=mxFull;
    OUT.flight_start.peak_full_online_ms=online(iFpeak);
    OUT.flight_start.peak_full_reference_ms=refms(iFpeak);
    OUT.flight_start.peak_full_request_delta=req(iFpeak);
    OUT.flight_start.peak_online_index=iOpeak;
    OUT.flight_start.peak_online_time_s=t(iOpeak);
    OUT.flight_start.peak_online_ms=mxOnline;
    OUT.flight_start.peak_online_full_ms=full(iOpeak);
    OUT.flight_start.peak_online_reference_ms=refms(iOpeak);
    OUT.flight_start.peak_online_request_delta=req(iOpeak);

    OUT.replan=struct();
    OUT.replan.request_count=numel(reqIdx);
    if ~isempty(reqIdx)
        ir=reqIdx(1);
        OUT.replan.first_index=ir;
        OUT.replan.first_time_s=t(ir);
        OUT.replan.attempt_ms=attempt(ir);
        OUT.replan.callback_ms=callback(ir);
        OUT.replan.reference_ms=refms(ir);
        OUT.replan.online_ms=online(ir);
        OUT.replan.full_ms=full(ir);
        OUT.replan.deadline_fail=fail(ir);
        OUT.replan.commit=commit(ir);
        OUT.replan.callback_fraction_of_attempt=callback(ir)/attempt(ir);
        OUT.replan.noncallback_attempt_ms=attempt(ir)-callback(ir);
    else
        OUT.replan.first_index=NaN;
        OUT.replan.first_time_s=NaN;
        OUT.replan.attempt_ms=NaN;
        OUT.replan.callback_ms=NaN;
        OUT.replan.reference_ms=NaN;
        OUT.replan.online_ms=NaN;
        OUT.replan.full_ms=NaN;
        OUT.replan.deadline_fail=NaN;
        OUT.replan.commit=NaN;
        OUT.replan.callback_fraction_of_attempt=NaN;
        OUT.replan.noncallback_attempt_ms=NaN;
    end

    OUT.flight_start.peak_precedes_first_replan = ...
        ~isempty(reqIdx) && iFpeak<reqIdx(1);
    OUT.flight_start.peak_is_replan_request = req(iFpeak)>0;

    fprintf('\n[B. FLIGHT-START vs REPLAN]\n');
    fprintf(' first Flight row        : i=%d t=%.6f s | online=%.3f ms full=%.3f ms ref=%.3f ms request=%.0f\n', ...
        iF0,tF0,online(iF0),full(iF0),refms(iF0),req(iF0));
    fprintf(' first-0.5s full peak    : i=%d t=%.6f s | online=%.3f ms full=%.3f ms ref=%.3f ms request=%.0f\n', ...
        iFpeak,t(iFpeak),online(iFpeak),full(iFpeak),refms(iFpeak),req(iFpeak));
    fprintf(' first-0.5s online peak  : i=%d t=%.6f s | online=%.3f ms full=%.3f ms ref=%.3f ms request=%.0f\n', ...
        iOpeak,t(iOpeak),online(iOpeak),full(iOpeak),refms(iOpeak),req(iOpeak));

    if ~isempty(reqIdx)
        ir=reqIdx(1);
        fprintf(' first replan request     : i=%d t=%.6f s | attempt=%.3f ms callback=%.3f ms ref=%.3f ms online=%.3f ms full=%.3f ms fail=%.0f commit=%.0f\n', ...
            ir,t(ir),attempt(ir),callback(ir),refms(ir),online(ir),full(ir),fail(ir),commit(ir));
        fprintf(' callback / attempt       : %.2f %%\n',100*OUT.replan.callback_fraction_of_attempt);
        fprintf(' non-callback attempt     : %.3f ms\n',OUT.replan.noncallback_attempt_ms);
    end

    %% C. Rank available detailed timing fields at the two key rows.
    timingNames=names(startsWith(names,"rt_") & endsWith(names,"_ms"));
    OUT.available_rt_timing_fields=cellstr(timingNames);

    fprintf('\n[C. AVAILABLE DETAILED rt_*_ms FIELDS]\n');
    fprintf(' count = %d\n',numel(timingNames));

    OUT.flight_start.top_rt_fields=rank_row_fields(timingNames,getv,iFpeak,12);
    fprintf(' top fields at Flight-start peak:\n');
    print_rank(OUT.flight_start.top_rt_fields);

    if ~isempty(reqIdx)
        OUT.replan.top_rt_fields=rank_row_fields(timingNames,getv,reqIdx(1),12);
        fprintf(' top fields at first replan request:\n');
        print_rank(OUT.replan.top_rt_fields);
    else
        OUT.replan.top_rt_fields=struct('name',{},'value_ms',{});
    end

    %% D. Source/object evidence for the Flight-start reference path.
    OUT.flight_start.reference_path_source=struct();
    OUT.flight_start.reference_path_source.nominal_table_fastpath_present= ...
        contains(R,'use_nominal_table=isequal(cha,''f'') && obj.nominal_table_valid');
    OUT.flight_start.reference_path_source.rolling_phase_match_required= ...
        contains(R,'isequal(obj.reference_cache.cha,cha)');
    OUT.flight_start.reference_path_source.full_fill_fallback_present= ...
        contains(R,'source=''full_base_ref_do_fill'';');
    OUT.flight_start.reference_path_source.full_fill_points_at_H4_dt= ...
        round(4/0.025)+1;

    OUT.object_found=false;
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        obj=evalin('base','replanner_instance');
        if isa(obj,'REPLANNING_SYSTEM')
            OUT.object_found=true;
            OUT.object=struct();
            OUT.object.nominal_table_valid=logical(obj.nominal_table_valid);
            OUT.object.nominal_table_t0=double(obj.nominal_table_t0);
            OUT.object.nominal_table_tend=double(obj.nominal_table_tend);
            OUT.object.nominal_table_dt=double(obj.nominal_table_dt);
            OUT.object.reference_cache_valid=logical(obj.reference_cache_valid);
            OUT.object.active_reference_mode=char(obj.active_reference_mode);
            OUT.object.active_reference_version=double(obj.active_reference_version);
            OUT.object.nominal_table_covers_first_flight= ...
                OUT.object.nominal_table_valid && ...
                tF0>=OUT.object.nominal_table_t0-1e-10 && ...
                (tF0+4)<=OUT.object.nominal_table_tend+1e-10;
        end
    end

    fprintf('\n[D. FLIGHT-START REFERENCE-PATH EVIDENCE]\n');
    fprintf(' nominal-table fast path exists         = %d\n', ...
        OUT.flight_start.reference_path_source.nominal_table_fastpath_present);
    fprintf(' rolling cache requires same phase cha  = %d\n', ...
        OUT.flight_start.reference_path_source.rolling_phase_match_required);
    fprintf(' 4-s full-fill fallback exists          = %d\n', ...
        OUT.flight_start.reference_path_source.full_fill_fallback_present);
    fprintf(' H=4, dt=.025 packet points             = %d\n', ...
        OUT.flight_start.reference_path_source.full_fill_points_at_H4_dt);
    if OUT.object_found
        fprintf(' postrun nominal table valid             = %d\n',OUT.object.nominal_table_valid);
        fprintf(' nominal table t0/tend                   = %.6f / %.6f s\n', ...
            OUT.object.nominal_table_t0,OUT.object.nominal_table_tend);
        fprintf(' table covers first Flight 4-s window    = %d\n', ...
            OUT.object.nominal_table_covers_first_flight);
    end

    %% E. Worker boundary audit.
    OUT.worker=struct();
    OUT.worker.parallel_toolbox_license=license('test','Distrib_Computing_Toolbox');
    OUT.worker.parfeval_available=exist('parfeval','file')==2;
    OUT.worker.background_pool_symbol_available= ...
        exist('backgroundPool','file')==2 || exist('backgroundPool','builtin')==5;
    OUT.worker.native_callback_source= ...
        contains(R,'RAW=af7r36_single_core_mex(1,pkt);');
    OUT.worker.reference_certificate_source= ...
        contains(R,'obj.evaluate_reference_only_proposal_certificate(candidate)');
    OUT.worker.commit_prepare_source=contains(R,'af7r58i_prepare_batch_model');
    OUT.worker.mex_has_process_local_compile_state= ...
        source_file_contains('af7r36_single_core_mex.cpp','G_compiled') && ...
        source_file_contains('af7r36_single_core_mex.cpp','nominal_signature');

    % Data-driven worker-boundary conclusion.
    % No arbitrary threshold is used as a safety condition; this is only an
    % architectural classification from the measured decomposition.
    if isfinite(OUT.replan.callback_fraction_of_attempt)
        if OUT.replan.callback_fraction_of_attempt < 0.5
            OUT.worker.recommended_boundary= ...
                'FULL_PLAN_CERT_PREP_WORKER__NOT_MEX_ONLY';
        else
            OUT.worker.recommended_boundary= ...
                'NATIVE_GENERATOR_FIRST__MEASURE_CERT_PREP_SEPARATELY';
        end
    else
        OUT.worker.recommended_boundary='NO_REAL_REQUEST_MEASUREMENT';
    end

    OUT.worker.thread_pool_not_promoted_reason= ...
        ['AF7R36 MEX keeps compiled nominal state in static process-local state; ' ...
         'do not assume thread-pool isolation. Process/ROS2-worker architecture ' ...
         'must be qualified with explicit worker priming and signature binding.'];

    fprintf('\n[E. WORKER READINESS / CORRECT BOUNDARY]\n');
    fprintf(' Parallel Computing Toolbox license  = %d\n',OUT.worker.parallel_toolbox_license);
    fprintf(' parfeval available                  = %d\n',OUT.worker.parfeval_available);
    fprintf(' backgroundPool symbol available     = %d\n',OUT.worker.background_pool_symbol_available);
    fprintf(' native callback path found          = %d\n',OUT.worker.native_callback_source);
    fprintf(' reference certificate path found    = %d\n',OUT.worker.reference_certificate_source);
    fprintf(' commit-prep path found               = %d\n',OUT.worker.commit_prepare_source);
    fprintf(' MEX compiled-state evidence          = %d\n',OUT.worker.mex_has_process_local_compile_state);
    fprintf(' recommended worker boundary          = %s\n',OUT.worker.recommended_boundary);

    %% F. Classification.
    OUT.classification=struct();

    if OUT.flight_start.peak_is_replan_request
        OUT.classification.flight_start_spike='REPLAN_COUPLED';
    elseif OUT.flight_start.peak_precedes_first_replan
        OUT.classification.flight_start_spike='PRE_REPLAN__SEPARATE_ROOT_CAUSE';
    else
        OUT.classification.flight_start_spike='NOT_YET_SEPARATED';
    end

    if ~isempty(reqIdx) && OUT.replan.commit>0 && OUT.replan.deadline_fail==0
        OUT.classification.replan='FIRST_REQUEST_CERTIFIED_COMMIT_CLOSED';
    else
        OUT.classification.replan='REPLAN_STILL_OPEN';
    end

    if strcmp(OUT.worker.recommended_boundary,'FULL_PLAN_CERT_PREP_WORKER__NOT_MEX_ONLY')
        OUT.classification.worker='OFFLOAD_GENERATION_PLUS_CERT_PREP__MAIN_THREAD_ATOMIC_ACTIVATION_ONLY';
    else
        OUT.classification.worker=OUT.worker.recommended_boundary;
    end

    % Source-supported hypothesis only; do not claim as measured if detailed
    % timing fields do not support it.
    OUT.flight_start.hypothesis= ...
        ['If the first-Flight spike is reference-dominant and the nominal-table ' ...
         'fast path was unavailable/not covering that row, phase-sensitive rolling ' ...
         'cache cannot shift across cha and the code falls back to a full 4-s ' ...
         'reference fill. If the table covered the row, this hypothesis is rejected.'];

    OUT.status='READONLY_FORENSIC_COMPLETE__NO_PRODUCTION_CHANGE';

    fprintf('\n[F. CLASSIFICATION]\n');
    fprintf(' Flight-start spike = %s\n',OUT.classification.flight_start_spike);
    fprintf(' replan             = %s\n',OUT.classification.replan);
    fprintf(' worker             = %s\n',OUT.classification.worker);
    fprintf(' status             = %s\n',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR]\n%s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which('REPLANNING_SYSTEM'));
if isempty(root), root=pwd; end
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZR_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZR_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'status=%s\n',OUT.status);
if isfield(OUT,'source_sha'), fprintf(fid,'sourceSHA=%s\n',OUT.source_sha); end
if isfield(OUT,'classification')
    fprintf(fid,'flightStartClassification=%s\n',OUT.classification.flight_start_spike);
    fprintf(fid,'replanClassification=%s\n',OUT.classification.replan);
    fprintf(fid,'workerClassification=%s\n',OUT.classification.worker);
end
if isfield(OUT,'flight_start')
    fprintf(fid,'flightStartT=%.17g\n',OUT.flight_start.time_s);
    fprintf(fid,'flightPeakT=%.17g\n',OUT.flight_start.peak_full_time_s);
    fprintf(fid,'flightPeakFullMs=%.9g\n',OUT.flight_start.peak_full_ms);
    fprintf(fid,'flightPeakOnlineMs=%.9g\n',OUT.flight_start.peak_full_online_ms);
    fprintf(fid,'flightPeakReferenceMs=%.9g\n',OUT.flight_start.peak_full_reference_ms);
    fprintf(fid,'flightPeakRequestDelta=%.9g\n',OUT.flight_start.peak_full_request_delta);
end
if isfield(OUT,'replan')
    fprintf(fid,'replanRequestCount=%d\n',OUT.replan.request_count);
    fprintf(fid,'replanAttemptMs=%.9g\n',OUT.replan.attempt_ms);
    fprintf(fid,'replanCallbackMs=%.9g\n',OUT.replan.callback_ms);
    fprintf(fid,'replanNoncallbackAttemptMs=%.9g\n',OUT.replan.noncallback_attempt_ms);
    fprintf(fid,'replanReferenceMs=%.9g\n',OUT.replan.reference_ms);
    fprintf(fid,'replanFullMs=%.9g\n',OUT.replan.full_ms);
end
if isfield(OUT,'worker')
    fprintf(fid,'workerBoundary=%s\n',OUT.worker.recommended_boundary);
    fprintf(fid,'parallelToolbox=%d\n',OUT.worker.parallel_toolbox_license);
    fprintf(fid,'parfevalAvailable=%d\n',OUT.worker.parfeval_available);
    fprintf(fid,'backgroundPoolAvailable=%d\n',OUT.worker.background_pool_symbol_available);
end
if isfield(OUT,'object_found') && OUT.object_found
    fprintf(fid,'nominalTableValid=%d\n',OUT.object.nominal_table_valid);
    fprintf(fid,'nominalTableCoversFirstFlight=%d\n',OUT.object.nominal_table_covers_first_flight);
end
if isfield(OUT,'error')&&isstruct(OUT.error)&&isfield(OUT.error,'message')
    fprintf(fid,'errorIdentifier=%s\n',OUT.error.identifier);
    fprintf(fid,'errorMessage=%s\n',OUT.error.message);
end
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZR_OUT',OUT);
fprintf('\nRETURN: %s\nRETURN: %s\n',report,matp);
end


function [names,getv,nrow]=profile_adapter(P)
if istable(P)
    names=string(P.Properties.VariableNames);
    getv=@(nm) P.(char(nm));
    nrow=height(P);
elseif isstruct(P)&&isscalar(P)
    names=string(fieldnames(P));
    getv=@(nm) P.(char(nm));
    nrow=numel(P.t);
else
    error('[AF7R58ZZR] runtime profile must be table or scalar struct.');
end
end


function S=rank_row_fields(names,getv,idx,nmax)
S=struct('name',{},'value_ms',{});
for k=1:numel(names)
    nm=names(k);
    try
        v=getv(nm);
        if isnumeric(v)||islogical(v)
            v=double(v);
            if numel(v)>=idx
                x=v(idx);
                if isfinite(x)
                    S(end+1)=struct('name',char(nm),'value_ms',double(x)); %#ok<AGROW>
                end
            end
        end
    catch
    end
end
if isempty(S), return; end
[~,ord]=sort([S.value_ms],'descend');
ord=ord(1:min(nmax,numel(ord)));
S=S(ord);
end


function print_rank(S)
if isempty(S)
    fprintf('   <none>\n');
    return;
end
for k=1:numel(S)
    fprintf('   %-42s %12.6f ms\n',S(k).name,S(k).value_ms);
end
end


function [m,j]=safe_max_index(x)
x=double(x(:));
good=isfinite(x);
assert(any(good),'[AF7R58ZZR] no finite values in spike window.');
ix=find(good);
[m,k]=max(x(good));
j=ix(k);
end


function tf=source_file_contains(name,token)
p=which(name);
if isempty(p)
    % which() does not always resolve C++ source files; try beside class.
    r=which('REPLANNING_SYSTEM');
    if ~isempty(r)
        q=fullfile(fileparts(r),name);
        if isfile(q), p=q; end
    end
end
if isempty(p)||~isfile(p)
    tf=false;
    return;
end
tf=contains(fileread(p),token);
end


function n=count_exact(s,pat)
n=numel(strfind(s,pat));
end


function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end
