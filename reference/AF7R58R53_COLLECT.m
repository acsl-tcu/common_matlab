function R53 = AF7R58R53_COLLECT()
% AFTER the usual simulation, call AF7R58R53_COLLECT.
% Never called from Flight; no additional Flight-time disk writes.
% Saves MAT and TXT in the folder where this function was placed.
root=fileparts(mfilename('fullpath'));
R53=struct();
R53.schema='AF7R58R53_STAGE2_NATIVE_TANGENT_REUSE_V1';
R53.production_source=which('REPLANNING_SYSTEM');
R53.error_tube_deferred=true;
R53.return_not_modified=true;
R53.trigger_not_modified=true;
R53.hlc_24s_spike_out_of_scope=true;
R53.capture_available=false;
R53.timing=struct('status','NO_POST_FLIGHT_R20_R22_CAPTURE');
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    C=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(C) && isfield(C,'r20_profile') && isfield(C,'r22_profile')
        R53.capture_available=true;
        R53.timing.status='R20_R22_PUBLISHED_CAPTURE';
        R53.timing.r20=C.r20_profile;
        R53.timing.r22=C.r22_profile;
        if isfield(C,'events'),R53.timing.events=C.events;end
    end
end
R53.mat_file=fullfile(root,'AF7R58R53_OUT.mat');
R53.report_file=fullfile(root,'AF7R58R53_REPORT.txt');
save(R53.mat_file,'R53','-v7');
f=fopen(R53.report_file,'wt');
assert(f>0,'R53:File','Cannot create R53 report in %s.',root);
c=onCleanup(@() fclose(f));
fprintf(f,'R53 | COMPLETE REPLANNER SOURCE | STAGE2 NATIVE TANGENT SHARING\n');
fprintf(f,'production=%s\n',R53.production_source);
fprintf(f,'capture_available=%d\n',R53.capture_available);
fprintf(f,'tube=DEFERRED | return=UNCHANGED | trigger=UNCHANGED\n');
fprintf(f,'source_pairing=NEEDS_SAME_RUN_VERIFICATION\n');
if R53.capture_available
    keys20={'stage1_future_ms','stage1_context_ms','stage1_native_call_ms', ...
            'native_packet_ms','native_mex_ms','native_wrapper_ms', ...
            'stage2_reference_cert_ms','stage2_bundle_ms'};
    keys22={'stage2_model_ms','stage2_sensor_ms','stage2_eval_handles_ms', ...
            'stage2_cache_ms','stage2_native_eval_packet_ms','stage2_native_eval_ms', ...
            'r52_stage2_exact_packet_reuse_count', ...
            'r53_native_tangent_shared_count','r53_native_tangent_shared_last'};
    for i=1:numel(keys20)
        k=keys20{i};
        if isfield(R53.timing.r20,k)
            fprintf(f,'r20.%s=%.10g\n',k,double(R53.timing.r20.(k)));
        end
    end
    for i=1:numel(keys22)
        k=keys22{i};
        if isfield(R53.timing.r22,k)
            fprintf(f,'r22.%s=%.10g\n',k,double(R53.timing.r22.(k)));
        end
    end
end
fprintf('R53 MAT: %s\nR53 REPORT: %s\n',R53.mat_file,R53.report_file);
end
