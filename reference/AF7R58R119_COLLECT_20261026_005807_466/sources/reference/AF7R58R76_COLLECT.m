function R76=AF7R58R76_COLLECT()
% No extra Flight work. Compare with same-event conservative obstacle packet
% after the existing, verified R75 frozen HLC+plant numeric replay.
D=fileparts(mfilename('fullpath'));
R76=struct('status','NOT_RUN','events',struct([]),'error_message','', ...
 'source',struct(),'mat_file',fullfile(D,'AF7R58R76_OUT.mat'), ...
 'report_file',fullfile(D,'AF7R58R76_REPORT.txt'));
try
 R76.source=AF7R58R76_SOURCE_CHECK();
 Base=AF7R58R75_COLLECT();
 R76.preview_status=Base.status;
 assert(Base.capture_available && ~isempty(Base.events), ...
    '[R76] no registered Flight events in the existing R75 collector.');
 replanner=AF7R58R74_REGISTRY('get');
 C=replanner.r71r2_event_snapshots;
 assert(numel(C)==numel(Base.events), ...
    '[R76] event count differs between capture and physical prediction.');
 for j=1:numel(C)
   e=Base.events(j);
   E=struct('time_s',double(C(j).time_s), ...
       'reference_version',double(C(j).reference_version), ...
       'prediction_status',char(e.status), ...
       'numeric_collision',struct(),'error_message','');
   if isfield(e.prediction,'t')
       try
          E.numeric_collision=AF7R58R76_COLLISION_AUDIT(C(j),e.prediction);
       catch ME
          E.error_message=sprintf('%s: %s',char(ME.identifier),char(ME.message));
          E.numeric_collision=struct('status','FAIL_CLOSED_COLLISION_AUDIT');
       end
   else
       E.error_message=char(e.error_message);
   end
   if isempty(R76.events)
      R76.events=E;
   else
      R76.events(end+1)=E; %#ok<AGROW> postflight only
   end
 end
 good=arrayfun(@(x)isfield(x.numeric_collision,'status') && ...
   strcmp(char(x.numeric_collision.status), ...
       'SAME_EVENT_REFERENCE_VS_EKF_HLC_PLANT_SAMPLED__NO_CONTINUOUS_PROOF') && ...
   isempty(x.error_message),R76.events);
 if all(good)
    R76.status='ALL_SAME_EVENT_NUMERIC_COMPARISONS_AVAILABLE__NOT_CERTIFIED';
 else
    R76.status='INCOMPLETE_OR_LEGACY_EVENT_COMPARISON__NO_TRIGGER_REPLACEMENT';
 end
catch ME
 R76.status='FAIL_CLOSED_COMPARISON_SETUP';
 R76.error_message=sprintf('%s: %s',char(ME.identifier),char(ME.message));
end
save(R76.mat_file,'R76','-v7');
fid=fopen(R76.report_file,'w');
assert(fid>=0,'[R76] cannot write report in reference directory.');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R76 REPLANNER ONLY | SAME EVENT PHYSICAL VS REFERENCE COLLISION AUDIT\n');
fprintf(fid,'status=%s events=%d\n',R76.status,numel(R76.events));
fprintf(fid,'error=%s\n',R76.error_message);
for j=1:numel(R76.events)
 E=R76.events(j);A=E.numeric_collision;
 fprintf(fid,'event=%d t=%.9f ver=%d prediction=%s audit=%s\n', ...
    j,E.time_s,E.reference_version,E.prediction_status,char(A.status));
 if isfield(A,'physical_sample_outer_hit')
    fprintf(fid,'  known=%d samples=%d refSampleHit=%d physicalSampleHit=%d originalTrigger=%d\n', ...
      A.known_count,A.samples,A.reference_sample_outer_hit,...
      A.physical_sample_outer_hit,A.existing_trigger_hit);
    fprintf(fid,'  referenceTTC=%.9g physicalTTC=%.9g originalTTC=%.9g\n', ...
       A.reference_first_time_s,A.physical_first_time_s,A.existing_trigger_ttc_s);
    fprintf(fid,'  physicalObs=%.9g physicalComponent=%s minOuterMargin=%.9g\n', ...
       A.physical_first_obstacle_id,A.physical_first_component, ...
       A.min_outer_normalized_clearance);
    fprintf(fid,'  nativeReference=%g nativePhysical=%g refNativeEqualsOld=%d\n', ...
       double(A.native_reference_nonseparation),double(A.native_physical_nonseparation), ...
       A.native_reference_matches_original);
 end
 fprintf(fid,'  error=%s\n',E.error_message);
end
fprintf(fid,'NOTE: Numeric grid cannot prove between-sample safety, thrust/tension constraints, or closed-loop reachability.\n');
fprintf(fid,'Production trigger unchanged; a negative numeric result MUST NOT be used as a safety certificate.\n');
fprintf(fid,'MAT=%s\nTXT=%s\n',R76.mat_file,R76.report_file);
end
