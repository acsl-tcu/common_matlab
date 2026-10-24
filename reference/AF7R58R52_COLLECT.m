function R52 = AF7R58R52_COLLECT()
% Run AFTER a normal simulation; save directly next to this .m file.
% Collect already-published R20/R22 profiles, no Flight-time disk I/O.
root=fileparts(mfilename('fullpath'));
R52=struct();
R52.schema='AF7R58R52_POST_FLIGHT_PROFILE_V1';
R52.read_only=true;
R52.hlc_24s_spike_in_scope=false;
R52.production_source=which('REPLANNING_SYSTEM');
R52.capture_available=false;
R52.return_status='UNCHANGED__R51_C7_SHAPE_WITNESS_ONLY';
R52.trigger_status='UNCHANGED__CURRENT_ESTIMATE_MODEL_NOT_YET_CONNECTED';
R52.execution_error_tube_status='DEFERRED_BY_USER_REQUIREMENT';
R52.timing=struct('status','NO_R20_R22_CAPTURE');
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    C=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(C) && isfield(C,'r20_profile') && ...
            isfield(C,'r22_profile')
        R52.capture_available=true;
        R52.timing.status='SAME_PUBLISHED_CAPTURE_R20_R22';
        R52.timing.r20=C.r20_profile;
        R52.timing.r22=C.r22_profile;
        if isfield(C,'events'), R52.timing.events=C.events; end
    end
end
R52.mat_file=fullfile(root,'AF7R58R52_OUT.mat');
R52.report_file=fullfile(root,'AF7R58R52_REPORT.txt');
save(R52.mat_file,'R52','-v7');
f=fopen(R52.report_file,'w');
assert(f>0,'R52:Report','Unable to create report in %s',root);
c=onCleanup(@()fclose(f));
fprintf(f,'R52 POST-FLIGHT | SOURCE-BOUND | ERROR TUBE DEFERRED\n');
fprintf(f,'capture=%d | timing_status=%s\n',R52.capture_available,R52.timing.status);
fprintf(f,'source=%s\n',R52.production_source);
if R52.capture_available
    names={'stage1_future_ms','stage1_context_ms','stage1_native_call_ms', ...
           'native_packet_ms','native_mex_ms','native_wrapper_ms', ...
           'stage2_reference_cert_ms','stage2_bundle_ms'};
    for i=1:numel(names)
        k=names{i};
        if isfield(R52.timing.r20,k)
            fprintf(f,'r20.%s=%.10g\n',k,double(R52.timing.r20.(k)));
        end
    end
    fields={'stage2_model_ms','stage2_sensor_ms', ...
            'stage2_eval_handles_ms','stage2_cache_ms', ...
            'stage2_native_eval_packet_ms','stage2_native_eval_ms', ...
            'r52_stage2_exact_packet_reuse_count','r52_stage2_exact_packet_last_reused'};
    for i=1:numel(fields)
        k=fields{i};
        if isfield(R52.timing.r22,k)
            fprintf(f,'r22.%s=%.10g\n',k,double(R52.timing.r22.(k)));
        end
    end
end
fprintf(f,'RETURN=unchanged; no phase/whole-body rejoin certificate from R51\n');
fprintf(f,'TRIGGER=unchanged; future reference still used for prediction\n');
fprintf(f,'EXECUTION_TRACKING_TUBE=deferred\n');
disp(['R52 MAT: ' R52.mat_file]);
disp(['R52 REPORT: ' R52.report_file]);
end
