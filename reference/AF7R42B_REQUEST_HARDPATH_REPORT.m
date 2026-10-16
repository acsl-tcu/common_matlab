function OUT = AF7R42B_REQUEST_HARDPATH_REPORT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R42B | <=4 ms REQUEST / REFERENCE-CERT / ATOMIC-COMMIT ATTRIBUTION\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | diagnosticOnly=1 | safetyLogicUnchanged=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(isprop(replanner,'af7r42_request_trace'),'[AF7R42B] AF7R42 trace property missing.');

T=replanner.af7r42_request_trace;
assert(isstruct(T)&&~isempty(T),'[AF7R42B] no real Flight request trace captured.');

fprintf([' #  t[s] stat   pre   ctx  cbQPC cbTIC record  gate  refCert commit ' ...
         ' cPrep cModel cRebld cAssert cTail  final  slack [ms]\n']);
for k=1:numel(T)
    fprintf('%2d %6.3f %4d %5.3f %5.3f %6.3f %6.3f %6.3f %5.3f %7.3f %6.3f %6.3f %6.3f %6.3f %7.3f %6.3f %6.3f %7.3f\n', ...
        k,T(k).t,T(k).status, ...
        T(k).pre_ms,T(k).context_ms,T(k).callback_qpc_ms,T(k).callback_tic_ms, ...
        T(k).record_ms,T(k).gate_ms,T(k).refcert_ms,T(k).commit_ms, ...
        T(k).commit_prepare_ms,T(k).commit_model_ms,T(k).commit_rebuild_ms, ...
        T(k).commit_assert_ms,T(k).commit_rollback_or_tail_ms, ...
        T(k).final_ms,T(k).final_slack_ms);
end

fprintf('\n[PRE-GENERATOR PROFILING AT REQUESTS]\n');
fprintf(' #  t[s] certGuard geometry sensor refPacket trigger eventCapture [ms]\n');
for k=1:numel(T)
    fprintf('%2d %6.3f %9.4f %8.4f %6.4f %9.4f %7.4f %12.4f\n', ...
        k,T(k).t,T(k).profile_cert_guard_ms,T(k).profile_geometry_ms, ...
        T(k).profile_sensor_ms,T(k).profile_reference_packet_ms, ...
        T(k).profile_trigger_ms,T(k).trigger_event_capture_ms);
end

idx20=find([T.status]==-20);
idx30=find([T.status]==-30);
idxOK=find([T.status]==30);

summarize_group('STATUS -20 : deadline exceeded before reference certificate',T,idx20);
summarize_group('STATUS -30 : atomic commit exceeded deadline',T,idx30);
summarize_group('STATUS 30 : successful reference-only commit',T,idxOK);

% Identify dominant median increment over the failing groups.
idx=[idx20 idx30];
names={'pre','context','callback_tic','record','refcert','commit'};
if isempty(idx)
    dom='NONE';
    domMed=NaN;
else
    M=[ ...
        [T(idx).pre_ms].', ...
        [T(idx).context_ms].', ...
        [T(idx).callback_tic_ms].', ...
        [T(idx).record_ms].', ...
        [T(idx).refcert_ms].', ...
        [T(idx).commit_ms].'];
    med=zeros(1,size(M,2));
    for j=1:size(M,2)
        v=M(:,j); v=v(isfinite(v));
        if isempty(v), med(j)=NaN; else, med(j)=median(v); end
    end
    vv=med; vv(~isfinite(vv))=-Inf;
    [domMed,j]=max(vv);
    dom=names{j};
end

fprintf('\n dominant failing-request median stage      = %s (%.6g ms)\n',dom,domMed);
fprintf(' hard deadline [ms]                        = %.6g\n',1e3*double(replanner.hard_modify_deadline_s));
fprintf(' request trace count                       = %d\n',numel(T));
fprintf(' safety theorem changed                    = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R42B_REQUEST_HARDPATH_REPORT_V1', ...
    'trace',T,'status_m20_index',idx20,'status_m30_index',idx30, ...
    'status_30_index',idxOK,'dominant_stage',dom,'dominant_stage_median_ms',domMed);
assignin('base','AF7R42B_REQUEST_HARDPATH_REPORT_OUT',OUT);
end


function summarize_group(title,T,idx)
fprintf('\n[%s]\n',title);
fprintf(' count                                     = %d\n',numel(idx));
if isempty(idx), return; end
F={'pre_ms','context_ms','callback_qpc_ms','callback_tic_ms','record_ms', ...
   'gate_ms','refcert_ms','commit_ms','commit_prepare_ms','commit_model_ms', ...
   'commit_rebuild_ms','commit_assert_ms','commit_rollback_or_tail_ms', ...
   'final_ms','final_slack_ms'};
for j=1:numel(F)
    v=arrayfun(@(x)double(x.(F{j})),T(idx));
    v=v(isfinite(v));
    if isempty(v)
        fprintf(' %-38s = NaN\n',strrep(F{j},'_',' '));
    else
        fprintf(' %-38s min/med/max = %.6g / %.6g / %.6g\n', ...
            strrep(F{j},'_',' '),min(v),median(v),max(v));
    end
end
end
