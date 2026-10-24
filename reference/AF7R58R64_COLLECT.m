function R64=AF7R58R64_COLLECT()
% Postflight only. Never perform disk I/O inside Flight.
root=fileparts(mfilename('fullpath'));
R64=struct('status','NO_FLIGHT_CAPTURE','live_capture',false, ...
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
        R64.live_capture=true;
        R64.status='EKF_NOW_NO_LEAD_REQUEST_CAPTURED__FUTURE_AND_REJOIN_UNPROVED';
        R64.ekf_current_now_requests=double(D.r64_now_request_count);
        R64.staged_commit_abort_count=double(D.r64_abort_staged_count);
        ring=double(D.r63_now_event_ring);
        n=min(double(D.r63_now_event_count),size(ring,1));
        if n>0
            h=double(D.r63_now_event_count);
            ix=mod((h-n):(h-1),size(ring,1))+1;
            R64.now_events=ring(ix,:);
        end
        R64.live_c7_authorities=D.r63_live_authority;
    end
end
R64.mat_file=fullfile(root,'AF7R58R64_OUT.mat');
R64.report_file=fullfile(root,'AF7R58R64_REPORT.txt');
save(R64.mat_file,'R64','-v7');
fid=fopen(R64.report_file,'wt');
assert(fid>0,'R64:Save','Cannot create report');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R64 SOURCE-BOUND EKF CURRENT ZERO-LEAD ACTION + VERIFIED C7 SOURCE\n');
fprintf(fid,'status=%s capture=%d\n',R64.status,R64.live_capture);
fprintf(fid,'EKF_NOW_REQUEST_COUNT=%g STAGED_COMMIT_ABORT_COUNT=%g\n', ...
    R64.ekf_current_now_requests,R64.staged_commit_abort_count);
for ii=1:size(R64.now_events,1)
    z=R64.now_events(ii,:);
    fprintf(fid,'event t=%.6f ver=%g known=%g ref=%g now=%g obs=%g comp=%g ttc=%.6f\n',z(1:8));
end
for ii=1:numel(R64.live_c7_authorities)
    A=R64.live_c7_authorities(ii);
    fprintf(fid,'C7 t=%.6f ver=%g valid=%d error=%.9g status=%s delay=%.9g\n', ...
        A.time_s,A.reference_version,logical(A.valid), ...
        A.flat_active_jet_consistency_error,A.status,A.phase_delay_s);
end
fprintf(fid,'Future physical HLC+plant 4s integration NOT INSTALLED. Candidate return NOT COMMITTED.\n');
fprintf(fid,'HLC UNCHANGED. External failure-stop action NOT IMPLEMENTED by R64.\n');
fprintf(fid,'MAT=%s\nTXT=%s\n',R64.mat_file,R64.report_file);
end
