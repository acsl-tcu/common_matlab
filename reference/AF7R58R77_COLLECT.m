function R77=AF7R58R77_COLLECT()
% Postflight only: read already captured SAME-EVENT R76 obstacle snapshots.
% No changes to Flight, HLC, EKF, plant, or reference commit pipeline.
D=fileparts(mfilename('fullpath'));
R77=struct('status','NOT_RUN','events',struct([]), ...
  'error_message','','mat_file',fullfile(D,'AF7R58R77_OUT.mat'), ...
  'report_file',fullfile(D,'AF7R58R77_REPORT.txt'));
try
 R77.source=AF7R58R77_SOURCE_CHECK();
 cobj=AF7R58R74_REGISTRY('get');
 C=cobj.r71r2_event_snapshots;
 assert(~isempty(C),'[R77] no retained R76 same-event snapshots.');
 Base=AF7R58R75_COLLECT();
 assert(Base.capture_available && numel(Base.events)==numel(C), ...
     '[R77] numerical predictions and event captures differ.');
 for j=1:numel(C)
    E=struct('time_s',double(C(j).time_s), ...
       'reference_version',double(C(j).reference_version), ...
       'status','NOT_RUN','frame',struct(),'error_message','');
    try
       p=Base.events(j).prediction;
       E.frame=AF7R58R77_FRAME_AUDIT(C(j),p);
       E.status=E.frame.status;
       E.error_message=E.frame.error_message;
    catch ME
       E.status='FAIL_CLOSED_EVENT_FRAME_COMPARISON';
       E.error_message=sprintf('%s: %s',char(ME.identifier),char(ME.message));
    end
    if isempty(R77.events),R77.events=E;
    else,R77.events(end+1)=E; %#ok<AGROW> postflight only
    end
 end
 good=arrayfun(@(e)strcmp(e.status, ...
    'THREE_WAY_SAME_EVENT_SAMPLE_COMPARISON__NOT_CERTIFIED'),R77.events);
 if all(good)
   R77.status='THREE_FRAME_COMPARISONS_AVAILABLE__TRIGGER_NOT_REPLACED';
 else
   R77.status='PARTIAL_OR_FAILED_FRAME_COMPARISONS__FAIL_CLOSED';
 end
catch ME
 R77.status='FAIL_CLOSED_COMPARISON_SETUP';
 R77.error_message=sprintf('%s: %s',char(ME.identifier),char(ME.message));
end
save(R77.mat_file,'R77','-v7');
fid=fopen(R77.report_file,'w');
assert(fid>=0,'[R77] cannot open local result text file.');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R77 | SOURCE-BOUND THREE-WAY FLIGHT FRAME / TRIGGER COMPARISON\n');
fprintf(fid,'status=%s events=%d error=%s\n',R77.status,numel(R77.events),R77.error_message);
for j=1:numel(R77.events)
 e=R77.events(j);fprintf(fid,'event=%d t=%.9f ver=%d status=%s\n', ...
     j,e.time_s,e.reference_version,e.status);
 if isfield(e.frame,'adapter_offset_m')
     a=e.frame;b=a.reference_raw;d=a.reference_hlc_frame;
     fprintf(fid,'  flight_adapter_constant_z_offset_m=%.12g\n',a.adapter_offset_m);
     fprintf(fid,'  packet_vs_flat_error_m=%.12g adapter_vs_HLC_error_m=%.12g consistent=%d\n', ...
         a.source_at_event_error_m,a.adapter_to_hlc_all_samples_error_m,a.reference_packet_consistent);
     if isfield(b,'reference_first_time_s') && isfield(d,'reference_first_time_s')
         fprintf(fid,'  RAW reference: hit=%d ttc=%.9g component=%s nativeHit=%g nativeIndex=%.9g\n', ...
           b.reference_sample_outer_hit,b.reference_first_time_s,b.reference_first_component, ...
           double(b.native_reference_nonseparation),double(b.native_reference_jhit));
         fprintf(fid,'  HLC-FRAME reference: hit=%d ttc=%.9g component=%s nativeHit=%g nativeIndex=%.9g\n', ...
           d.reference_sample_outer_hit,d.reference_first_time_s,d.reference_first_component, ...
           double(d.native_reference_nonseparation),double(d.native_reference_jhit));
         fprintf(fid,'  EKF HLC+plant physical: hit=%d ttc=%.9g component=%s nativeHit=%g nativeIndex=%.9g\n', ...
           b.physical_sample_outer_hit,b.physical_first_time_s,b.physical_first_component, ...
           double(b.native_physical_nonseparation),double(b.native_physical_jhit));
         fprintf(fid,'  initial vehicle gap=%.9g m load gap=%.9g m\n', ...
           a.initial_uav_tracking_gap_m,a.initial_payload_tracking_gap_m);
     end
 end
 fprintf(fid,'  error=%s\n',e.error_message);
end
fprintf(fid,'LIMITS: All results are point-in-time samples, not interval proof. HLC/plant predictions use EVENT latest EKF mass.\n');
fprintf(fid,'No production trigger, geometry proof, HLC, EKF, plant, reference, or runner changed.\n');
fprintf(fid,'MAT=%s\nTXT=%s\n',R77.mat_file,R77.report_file);
end
