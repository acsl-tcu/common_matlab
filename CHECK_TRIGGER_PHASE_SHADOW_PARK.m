function R=CHECK_TRIGGER_PHASE_SHADOW_PARK(replanner_instance)
% Read-only postflight verification of optional phase-shadow parking.
% This is PERFORMANCE metadata, never a collision-safety certificate.
R=struct('status','NOT_COMPLETED_SHADOW_PARK_CHECK','reason','', ...
    'parked_calls',0,'frame_calls',0,'authority_calls',0, ...
    'verified_generations',0,'safe_to_commit',false);
try
    S=replanner_instance.get_status();
    required={'phase_shadow_parked_ready','phase_shadow_parked_calls', ...
        'phase_shadow_frame_calls','phase_shadow_authority_calls','phase_shadow_geometry'};
    assert(all(isfield(S,required)),'CHECK_TRIGGER_PHASE_SHADOW_PARK:Source', ...
        'The corresponding REPLANNING_SYSTEM.m source has not been installed.');
    R.parked_calls=S.phase_shadow_parked_calls;
    R.frame_calls=S.phase_shadow_frame_calls;
    R.authority_calls=S.phase_shadow_authority_calls;
    R.verified_generations=S.phase_shadow_geometry.completed_generations;
    assert(R.authority_calls>=8 && R.verified_generations>=1 && ...
        S.phase_shadow_parked_ready && R.parked_calls>=1, ...
        'CHECK_TRIGGER_PHASE_SHADOW_PARK:NotSettled', ...
        'Shadow was not parked; inspect mass availability and observed generation.');
    R.status='PASS_SHADOW_REUSED_AFTER_PROVENANCE_STAGING__NOT_TIMING_OR_SAFETY';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_PHASE_SHADOW_PARK_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nparked_calls=%d\nframe_calls=%d\nauthority_calls=%d\nverified_generations=%d\nreason=%s\n', ...
        R.status,R.parked_calls,R.frame_calls,R.authority_calls,R.verified_generations,R.reason);
    fclose(fid);
end
save('TRIGGER_PHASE_SHADOW_PARK_RESULT.mat','R');
end
