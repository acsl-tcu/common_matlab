%% AF7R58ZZY_R12_R2_POSTRUN_RECOVERY
% Recovery-only audit for an already completed R12 run.
% No simulation, no production modification, no phase work.
% Fixes R12 report-writer bug: Forensic.observer.events is a struct array,
% not necessarily a cell array.

clearvars; clc;
rawName='AF7R58ZZY_R12_RAW_CAPTURE.mat';
rawPath=fullfile(pwd,rawName);
if ~isfile(rawPath)
    D=dir(fullfile(pwd,'**',rawName));
    assert(~isempty(D),'[R12-R2] Existing R12 RAW capture not found. Put this script beside AF7R58ZZY_R12_RAW_CAPTURE.mat.');
    [~,ii]=max([D.datenum]);
    rawPath=fullfile(D(ii).folder,D(ii).name);
end
S=load(rawPath,'RAW');
assert(isfield(S,'RAW')&&isstruct(S.RAW),'[R12-R2] RAW struct missing.');
RAW=S.RAW;
assert(isfield(RAW,'forensic')&&isstruct(RAW.forensic),'[R12-R2] forensic data missing.');
F=RAW.forensic;
req={'performance','baseline_windows','optimized_windows','baseline_pipeline','optimized_pipeline','observer','fast','reference'};
assert(all(isfield(F,req)),'[R12-R2] R12 forensic capture incomplete.');
assert(isfield(RAW,'mex')&&RAW.mex.closed&&RAW.mex.full_helper_bitwise_exact,'[R12-R2] R12 exact MEX closure missing.');

P=F.performance;
metricsClosed=isfield(P,'closed')&&logical(P.closed);
observerClosed=isfield(F.observer,'closed')&&logical(F.observer.closed);
fastClosed=isfield(F.fast,'closed')&&logical(F.fast.closed);
referenceClosed=isfield(F.reference,'closed')&&logical(F.reference.closed);
diagnosticClosed=metricsClosed&&observerClosed&&fastClosed&&referenceClosed;
control25Closed=metricsClosed&&P.target_do_peak_ms<=25.0;
historical4Closed=metricsClosed&&P.target_do_peak_ms<=4.0;
desired3Closed=metricsClosed&&P.target_do_peak_ms<=3.0;
performanceClosed=control25Closed&&logical(P.no_regression_changed_path);

reportPath=fullfile(pwd,'AF7R58ZZY_R12_R2_RECOVERED_REPORT.txt');
fid=fopen(reportPath,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
say(fid,'status=%s',ternary(performanceClosed,'FULL_PATH_FORENSIC_R12_R2_RECOVERED__PERFORMANCE_25MS_CLOSED','FULL_PATH_FORENSIC_R12_R2_RECOVERED__PERFORMANCE_OPEN'));
say(fid,'recoveredFromExistingR12Raw=1');
say(fid,'simulationRerun=0');
say(fid,'reportWriterBugFixed=STRUCT_ARRAY_PAREN_INDEXING');
say(fid,'diagnosticClosed=%d',diagnosticClosed); say(fid,'performanceClosed=%d',performanceClosed);
say(fid,'controlPeriod25msClosed=%d',control25Closed); say(fid,'historical4msClosed=%d',historical4Closed); say(fid,'desired3msClosed=%d',desired3Closed);
say(fid,'productionModified=0'); say(fid,'newArmingWork=0'); say(fid,'newTakeoffWork=0'); say(fid,'newLandingWork=0');
say(fid,'doAuthorityColumn=2'); say(fid,'column3Meaning=GET_LOAD_MASS_NOT_DO_TOTAL');
say(fid,'sourceRaw=%s',rawPath);

say(fid,''); say(fid,'[CORRECT_FULL_PATH_PERFORMANCE]');
say(fid,'columns=metric baseline_ms optimized_ms ratio');
say(fid,'flightEntryDoTotalPeak %.9g %.9g %.9g',P.flight_entry_do_baseline_ms,P.flight_entry_do_ms,P.flight_entry_do_ratio);
say(fid,'replanDoTotalPeak %.9g %.9g %.9g',P.replan_do_baseline_ms,P.replan_do_ms,P.replan_do_ratio);
say(fid,'targetDoTotalPeak %.9g %.9g %.9g',P.target_do_baseline_peak_ms,P.target_do_peak_ms,P.target_do_ratio);
say(fid,'observerPeak %.9g %.9g %.9g',P.observer_peak_baseline_ms,P.observer_peak_ms,P.observer_peak_ratio);
say(fid,'stage1Total %.9g %.9g %.9g',P.stage1_baseline_ms,P.stage1_ms,P.stage1_ratio);
say(fid,'stage2Total %.9g %.9g %.9g',P.stage2_total_baseline_ms,P.stage2_total_ms,P.stage2_total_ratio);
say(fid,'stage2Postcache %.9g %.9g %.9g',P.stage2_postcache_baseline_ms,P.stage2_postcache_ms,P.stage2_postcache_ratio);
say(fid,'stage3Total %.9g %.9g %.9g',P.stage3_baseline_ms,P.stage3_ms,P.stage3_ratio);
say(fid,'postcommitRefPacket %.9g %.9g %.9g',P.postcommit_ref_baseline_ms,P.postcommit_ref_ms,P.postcommit_ref_ratio);

say(fid,''); say(fid,'[OBSERVER_LIVE_VS_EXACT_REPLAY]');
O=F.observer; say(fid,'snapshotCount=%d',O.snapshot_count); say(fid,'preflightPrimeDone=%d',O.preflight_prime_done); say(fid,'preflightPrimeMs=%.9g',O.preflight_prime_ms);
say(fid,'columns=t liveTotal oneHot contractHot hotSum liveToHotRatio exact');
Ev=O.events;
for k=1:numel(Ev)
    if iscell(Ev), E=Ev{k}; else, E=Ev(k); end
    say(fid,'row=%.6g %.9g %.9g %.9g %.9g %.9g %d',E.t,E.live_total_ms,E.one_hot_median_ms,E.contract_hot_median_ms,E.hot_sum_ms,E.live_to_hot_ratio,E.exact);
end
say(fid,'classification=%s',O.classification);

say(fid,''); say(fid,'[FAST_HELPER_LIVE_VS_SAME_OBJECT_REPLAY]');
Q=F.fast;
say(fid,'liveStage2PostcacheMs=%.9g',Q.live_stage2_postcache_ms);
say(fid,'sameObjectFirstMs=%.9g',Q.first_ms); say(fid,'sameObjectHotMedianMs=%.9g',Q.hot_median_ms); say(fid,'sameObjectHotP95Ms=%.9g',Q.hot_p95_ms);
say(fid,'liveMinusHotMedianMs=%.9g',Q.live_minus_hot_median_ms); say(fid,'originalHelperHotMedianMs=%.9g',Q.original_hot_median_ms); say(fid,'bitwiseExact=%d',Q.bitwise_exact);
if isfield(Q,'profile')&&isfield(Q.profile,'ok')&&Q.profile.ok
    say(fid,'profileAvailable=1');
    T=Q.profile.top;
    for k=1:numel(T), say(fid,'rank=%d total_s=%.9g calls=%.0f name=%s',k,T(k).total_s,T(k).calls,T(k).name); end
else
    say(fid,'profileAvailable=0');
end

say(fid,''); say(fid,'[POSTCOMMIT_REFERENCE_PACKET_DETAIL]');
R=F.reference; say(fid,'peakTotalMs=%.9g',R.peak_total_ms); say(fid,'peakT=%.9g',R.peak_t); say(fid,'peakDominant=%s',R.peak_dominant);
say(fid,'columns=t total assert setup eligibility shiftCalc shiftAlloc tailScalar tailBatch derive validate store pathCode');
for i=1:size(R.rows,1), fprintf(fid,'row='); fprintf(fid,' %.9g',R.rows(i,:)); fprintf(fid,'\n'); end

say(fid,''); say(fid,'[ROOT_CAUSE_SUMMARY]');
rat=[Ev.live_to_hot_ratio];
say(fid,'observerExactReplayAll=%d',all([Ev.exact]));
say(fid,'observerMaxLiveToHotRatio=%.9g',max(rat));
say(fid,'observerRoot=LIVE_FIRST_USE_OR_DYNAMIC_ALLOCATION_REMAINS_DESPITE_PREFLIGHT_PRIME');
say(fid,'fastLiveMinusHotMedianMs=%.9g',Q.live_minus_hot_median_ms);
say(fid,'fastRoot=STAGE2_CALLSITE_FIRST_USE_OVERHEAD_PLUS_STEADY_HELPER_COST');
say(fid,'referenceRoot=%s',R.peak_dominant);
say(fid,'nextAction=DO_NOT_INSTALL_R11_YET__FIX_AND_VERIFY_CALLSITE_FIRST_USE_AND_POSTCOMMIT_REFERENCE_SPIKES');

OUT=struct('status','FULL_PATH_FORENSIC_R12_R2_RECOVERED__PERFORMANCE_OPEN','diagnostic_closed',diagnosticClosed,'performance_closed',performanceClosed, ...
    'control_period_25ms_closed',control25Closed,'historical_4ms_closed',historical4Closed,'desired_3ms_closed',desired3Closed, ...
    'forensic',F,'source_raw',rawPath,'report_path',reportPath,'simulation_rerun',false,'production_modified',false);
save(fullfile(pwd,'AF7R58ZZY_R12_R2_OUT.mat'),'OUT','-v7.3');
fprintf('\n[R12-R2] Existing RAW reanalysis complete.\n report = %s\n',reportPath);

function say(fid,varargin)
s=sprintf(varargin{:}); fprintf('%s\n',s); fprintf(fid,'%s\n',s);
end
function x=ternary(c,a,b), if c, x=a; else, x=b; end, end
