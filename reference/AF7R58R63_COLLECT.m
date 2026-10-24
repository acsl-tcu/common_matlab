function R63=AF7R58R63_COLLECT()
% R63 actual Flight event source & EKF NOW risk. Postflight I/O only.
% The recorded jets are REAL active source, not R44 historical test data.
root=fileparts(mfilename('fullpath'));
R63=struct();
R63.status='NO_LIVE_FLIGHT_CAPTURE';
R63.live_capture_available=false;
R63.hlc_unchanged=true;
R63.reference_trigger_future_still_active_only=true;
R63.current_ekf_now_gate_installed=true;
R63.real_hlc_plant_4s_future_installed=false;
R63.natural_rejoin_committed=false;
R63.robust_error_tube_installed=false;
R63.source_class=which('REPLANNING_SYSTEM');
R63.now_event_columns={'t','reference_version','known_count', ...
    'reference_hit','ekf_now_unsafe_or_unseparated', ...
    'now_obstacle_id','now_component_code','reference_ttc', ...
    'ekf_cable_constraint_residual','ref_payload_to_EKF_UAV', ...
    'ref_payload_to_EKF_load','now_hit_cumulative'};
R63.now_events=nan(0,12);
R63.live_authorities=struct([]);
R63.now_hit_total=0;
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(D)&&isfield(D,'r63_now_event_ring') && ...
            isfield(D,'r63_now_event_count') && ...
            isfield(D,'r63_live_authority')
        R63.live_capture_available=true;
        R63.status='ACTUAL_FLIGHT_NOW_EKF_AND_C7_PHASE_CAPTURED__UNPROVED';
        ring=double(D.r63_now_event_ring);
        n=min(size(ring,1),double(D.r63_now_event_count));
        h=double(D.r63_now_event_count);
        if n>0
            rr=mod((h-n):(h-1),size(ring,1))+1;
            R63.now_events=ring(rr,:);
        end
        R63.live_authorities=D.r63_live_authority;
        R63.now_hit_total=double(D.r63_now_hit_count);
    end
end
R63.mat_file=fullfile(root,'AF7R58R63_OUT.mat');
R63.report_file=fullfile(root,'AF7R58R63_REPORT.txt');
save(R63.mat_file,'R63','-v7');
fid=fopen(R63.report_file,'wt');
assert(fid>0,'R63:Save','Report file open failed');
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R63 | LIVE EKF CURRENT RISK + LIVE C7/PHASE AUTHORITY | NO COMMIT\n');
fprintf(fid,'source=%s\n',R63.source_class);
fprintf(fid,'status=%s live_capture=%d now_hit_total=%d\n', ...
    R63.status,R63.live_capture_available,R63.now_hit_total);
for k=1:size(R63.now_events,1)
    z=R63.now_events(k,:);
    fprintf(fid,'EKF_NOW t=%.6f ver=%g known=%g ref_hit=%g unsafe=%g obs=%g comp=%g ref_ttc=%.6f residual=%.8g refToQ=%.8g refToL=%.8g hits=%g\n',z);
end
fprintf(fid,'live_authority_count=%d\n',numel(R63.live_authorities));
for k=1:numel(R63.live_authorities)
    A=R63.live_authorities(k);
    fprintf(fid,'LIVE_JETS t=%.6f ver=%g mode=%s valid=%d status=%s delay=%.9g flatError=%.9g\n', ...
        A.time_s,A.reference_version,A.reference_mode,logical(A.valid), ...
        A.status,A.phase_delay_s,A.flat_active_jet_consistency_error);
end
fprintf(fid,'HLC=UNCHANGED; EKF_now does NOT replace 4s actual HLC+plant future; natural C7 available as noncommittable live candidate only.\n');
fprintf(fid,'No extra robustness tube; current component protective radii retained.\n');
fprintf(fid,'MAT=%s\nTXT=%s\n',R63.mat_file,R63.report_file);
end
