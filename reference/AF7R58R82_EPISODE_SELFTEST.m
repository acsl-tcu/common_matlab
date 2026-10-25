function T = AF7R58R82_EPISODE_SELFTEST()
% Standalone policy test: no production write and no controller invocation.
C=struct('position_c7',true,'tangent_yaw_c6',true, ...
    'body_geometry_continuous',true,'reference_kinematics',true, ...
    'whole_known_obstacle_coverage',true,'actuator_and_tension',true, ...
    'full_nominal_tail',true,'execution_dynamics',true, ...
    'inter_sample_closure',true,'source_bound_proof',true, ...
    'ieee_sound_proof',true,'robust_error_tube',true);
E=struct('obstacle_id',3,'active_version',0, ...
    'candidate_source_version',0,'known_ids',[2 3], ...
    'candidate_known_ids',[3 2], 'hazard_status','CONFIRMED', ...
    'certificate',C,'robust_inflation_on',true,'backup_on',true);
G0=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(G0.admit && G0.request_replan && ...
    strcmp(G0.hazard_episode_key,'obs_3__reference_0'));
E.active_version=1; E.candidate_source_version=1;
G1=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(G1.admit && G1.request_replan && ...
    strcmp(G1.hazard_episode_key,'obs_3__reference_1'));
% Same obstacle but a new reference version must NOT be silently ignored.
assert(~strcmp(G0.hazard_episode_key,G1.hazard_episode_key));
E.certificate.actuator_and_tension=false;
G2=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(~G2.admit && G2.reference_only_eligible && ...
    strcmp(G2.reason,'FULL_EXECUTION_OR_TAIL_PROOF_MISSING'));
E.certificate.actuator_and_tension=true;
E.certificate.robust_error_tube=false;
G3=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(~G3.admit && strcmp(G3.reason,'ROBUST_TUBE_PROOF_MISSING'));
E.robust_inflation_on=false;
G4=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(G4.admit && ~G4.production_proved);
E.robust_inflation_on=true;E.certificate.robust_error_tube=true;
E.candidate_source_version=0;
G5=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(~G5.admit && strcmp(G5.reason,'STALE_REFERENCE_VERSION'));
E.candidate_source_version=1; E.candidate_known_ids=3;
G6=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(~G6.admit && strcmp(G6.reason,'KNOWN_OBSTACLE_SET_CHANGED_OR_INCOMPLETE'));
E.candidate_known_ids=[2 3]; E.hazard_status='UNKNOWN';
G7=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(~G7.admit && G7.requires_backup);
% Actual user R81 flags: reference pass, missing input/tail -> no admission.
E.hazard_status='CONFIRMED';E.certificate.actuator_and_tension=false;
E.certificate.full_nominal_tail=false;
G8=AF7R58R82_EPISODE_COMMIT_GATE(E);
assert(~G8.admit && G8.reference_only_eligible);
T=struct('status','PASS_R82_EPISODE_VERSION_FAIL_CLOSED_POLICY', ...
    'same_obstacle_new_version_retained',true, ...
    'R81_reference_only_rejected_for_production',true, ...
    'independent_robust_switch_preserved',true, ...
    'production_modified',false,'production_safety_proved',false);
end
