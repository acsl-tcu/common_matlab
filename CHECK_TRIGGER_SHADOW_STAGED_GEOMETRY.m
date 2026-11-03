function R=CHECK_TRIGGER_SHADOW_STAGED_GEOMETRY(replanner_instance)
% Read-only diagnostic for one already completed Flight. No extra Flight.
R=struct('status','NOT_INSPECTED_STAGED_GEOMETRY', ...
 'shadow_calls',0,'verified_shapes',0,'completed_generations',0, ...
 'invalid_observations',0,'verified_ids',[],'outer_ids',[], ...
 'last_status','','reason','');
try
    S=replanner_instance.get_status();
    assert(isfield(S,'phase_shadow_geometry'), ...
      'CHECK_TRIGGER_SHADOW_STAGED_GEOMETRY:OldSource', ...
      'The installed REPLANNING_SYSTEM lacks staged observed geometry.');
    G=S.phase_shadow_geometry;
    R.shadow_calls=G.calls;
    R.verified_shapes=G.verified_shapes;
    R.completed_generations=G.completed_generations;
    R.invalid_observations=G.invalid_observations;
    R.verified_ids=G.verified_ids;
    R.outer_ids=G.outer_ids;
    R.last_status=G.last_status;
    if G.completed_generations>=1 && ...
            G.invalid_observations==0 && ...
            isequal(G.verified_ids,G.outer_ids)
        R.status='PASS_REAL_OBSERVED_GEOMETRY_STAGED__TIMING_UNPROVED';
    else
        R.status='STAGING_INCOMPLETE_OR_INVALID__ORIGINAL_FLIGHT_GATE_ONLY';
        R.reason=G.last_reason;
    end
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_SHADOW_STAGED_GEOMETRY_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nshadow_calls=%d\nverified_shapes=%d\ncompleted_generations=%d\ninvalid_observations=%d\nverified_ids=%s\nouter_ids=%s\nlast_status=%s\nreason=%s\n', ...
        R.status,R.shadow_calls,R.verified_shapes, ...
        R.completed_generations,R.invalid_observations, ...
        mat2str(R.verified_ids),mat2str(R.outer_ids), ...
        R.last_status,R.reason);
    fclose(fid);
end
end
