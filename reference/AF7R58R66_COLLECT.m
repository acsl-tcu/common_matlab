function R66=AF7R58R66_COLLECT()
% Postflight only. Never perform disk I/O inside Flight.
root=fileparts(mfilename('fullpath'));
R66=struct('status','NO_FLIGHT_CAPTURE','live_capture',false, ...
    'hlc_unchanged',true,'detector_unchanged',true, ...
    'closed_loop_four_second_future_certified',false, ...
    'natural_return_committed',false, ...
    'robust_error_tube_installed',false, ...
    'ekf_current_now_requests',NaN,'staged_commit_abort_count',NaN, ...
    'now_event_columns',{{'t','reference_version','known_count', ...
    'reference_hit','ekf_now_unseparated','obstacle_id','component', ...
    'reference_ttc','cable_residual','ref_to_EKF_UAV', ...
    'ref_to_EKF_load','cumulative_now_hits'}}, ...
    'now_events',nan(0,12),'live_c7_authorities',struct([]));
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(D)&&isfield(D,'r64_now_request_count') && ...
            isfield(D,'r63_now_event_ring')
        R66.live_capture=true;
        R66.status='EKF_NOW_NO_LEAD_REQUEST_CAPTURED__FUTURE_AND_REJOIN_UNPROVED';
        R66.ekf_current_now_requests=double(D.r64_now_request_count);
        R66.staged_commit_abort_count=double(D.r64_abort_staged_count);
        ring=double(D.r63_now_event_ring);
        n=min(double(D.r63_now_event_count),size(ring,1));
        if n>0
            h=double(D.r63_now_event_count);
            ix=mod((h-n):(h-1),size(ring,1))+1;
            R66.now_events=ring(ix,:);
        end
        R66.live_c7_authorities=D.r63_live_authority;
    end
end
% Postflight-only bridge status (no new work in Flight).
R66.maximum_observed_reference_version=NaN;
R66.r16_stage2_status='NOT_CAPTURED';
R66.r16_stage3_status='NOT_CAPTURED';
R66.r16_stage1_status='NOT_CAPTURED';
if ~isempty(R66.live_c7_authorities)
    R66.maximum_observed_reference_version=max( ...
        arrayfun(@(z)double(z.reference_version),R66.live_c7_authorities));
end
if isappdata(0,'AF7R58R15_LAST_STATUS')
    B=getappdata(0,'AF7R58R15_LAST_STATUS');
    if isstruct(B)
        if isfield(B,'r16_stage2_status'),R66.r16_stage2_status=char(B.r16_stage2_status);end
        if isfield(B,'r16_stage3_status'),R66.r16_stage3_status=char(B.r16_stage3_status);end
        if isfield(B,'r16_stage1_status'),R66.r16_stage1_status=char(B.r16_stage1_status);end
    end
end
R66.mat_file=fullfile(root,'AF7R58R66_OUT.mat');
R66.report_file=fullfile(root,'AF7R58R66_REPORT.txt');
save(R66.mat_file,'R66','-v7');
fid=fopen(R66.report_file,'wt');
assert(fid>0,'R66:Save','Cannot create report');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R66 BASELINE NATIVE CERT RESTORED + PHASE GATE PRESERVED + HOLD MODEL FIX\n');
fprintf(fid,'status=%s capture=%d\n',R66.status,R66.live_capture);
fprintf(fid,'EKF_NOW_REQUEST_COUNT=%g STAGED_COMMIT_ABORT_COUNT=%g\n', ...
    R66.ekf_current_now_requests,R66.staged_commit_abort_count);
for ii=1:size(R66.now_events,1)
    z=R66.now_events(ii,:);
    fprintf(fid,'event t=%.6f ver=%g known=%g ref=%g now=%g obs=%g comp=%g ttc=%.6f\n',z(1:8));
end
for ii=1:numel(R66.live_c7_authorities)
    A=R66.live_c7_authorities(ii);
    fprintf(fid,'C7 t=%.6f ver=%g valid=%d error=%.9g status=%s delay=%.9g\n', ...
        A.time_s,A.reference_version,logical(A.valid), ...
        A.flat_active_jet_consistency_error,A.status,A.phase_delay_s);
end
fprintf(fid,'MAX_OBSERVED_REFERENCE_VERSION=%.0f\n',R66.maximum_observed_reference_version);
fprintf(fid,'R16_STAGE1=%s\nR16_STAGE2=%s\nR16_STAGE3=%s\n', ...
    R66.r16_stage1_status,R66.r16_stage2_status,R66.r16_stage3_status);
fprintf(fid,'Future physical HLC+plant 4s integration NOT INSTALLED. Candidate return NOT COMMITTED.\n');
fprintf(fid,'HLC UNCHANGED. Existing reference-level hard certificates still enforced. New phase candidates fail closed. External failure-stop action NOT IMPLEMENTED.\n');
fprintf(fid,'MAT=%s\nTXT=%s\n',R66.mat_file,R66.report_file);
end
