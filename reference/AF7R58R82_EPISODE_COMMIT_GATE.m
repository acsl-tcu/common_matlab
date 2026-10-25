function G = AF7R58R82_EPISODE_COMMIT_GATE(E)
% AF7R58R82_EPISODE_COMMIT_GATE
% Pure, fail-closed admission policy for an externally CERTIFIED candidate.
% Does NOT calculate certificates or modify the live replanner/controller.
% A repeated obstacle ID on a NEW reference version is a NEW hazard episode.
% No numerical sampled forecast can be promoted to a continuous-time proof.
%
% REQUIRED FIELDS (no implicit defaults):
% E.obstacle_id, E.active_version, E.candidate_source_version
% E.known_ids, E.candidate_known_ids (IDs from current sensor memory)
% E.hazard_status = 'CONFIRMED'|'UNKNOWN'|'CLEAR'
% E.certificate = struct with boolean fields named below
% E.robust_inflation_on, E.backup_on (independent diagnostic switches)
%
% IMPORTANT: Certificate fields must be populated by trusted, source-bound
% external proof providers. Caller-supplied true values are NOT proof.
fields = {'obstacle_id','active_version','candidate_source_version', ...
    'known_ids','candidate_known_ids','hazard_status','certificate', ...
    'robust_inflation_on','backup_on'};
assert(isstruct(E) && isscalar(E), 'R82:InputMustBeScalarStruct');
for k = 1:numel(fields)
    assert(isfield(E,fields{k}), 'R82:MissingField:%s',fields{k});
end
G = struct('schema','R82_SOURCE_BOUND_ADMISSION_POLICY_V1', ...
    'admit',false,'request_replan',false,'requires_backup',false, ...
    'hazard_episode_key','','reason','UNRESOLVED', ...
    'reference_only_eligible',false,'production_proved',false, ...
    'live_production_modified',false);
assert(isnumeric(E.obstacle_id) && isscalar(E.obstacle_id) && ...
       isfinite(E.obstacle_id) && E.obstacle_id == fix(E.obstacle_id) && E.obstacle_id>=1, ...
       'R82:InvalidObstacleID');
assert(isnumeric(E.active_version) && isscalar(E.active_version) && ...
       isfinite(E.active_version) && E.active_version == fix(E.active_version) && E.active_version>=0, ...
       'R82:InvalidVersion');
assert(isnumeric(E.candidate_source_version) && isscalar(E.candidate_source_version) && ...
       isfinite(E.candidate_source_version) && E.candidate_source_version==fix(E.candidate_source_version) && E.candidate_source_version>=0, ...
       'R82:InvalidCandidateVersion');
assert(isnumeric(E.known_ids) && isnumeric(E.candidate_known_ids), ...
       'R82:KnownIDsMustBeNumeric');
assert(all(isfinite(E.known_ids(:))) && all(E.known_ids(:)>=1) && ...
       all(E.known_ids(:)==fix(E.known_ids(:))) && ...
       numel(unique(E.known_ids(:)))==numel(E.known_ids(:)), ...
       'R82:InvalidKnownIDs');
assert(all(isfinite(E.candidate_known_ids(:))) && all(E.candidate_known_ids(:)>=1) && ...
       all(E.candidate_known_ids(:)==fix(E.candidate_known_ids(:))) && ...
       numel(unique(E.candidate_known_ids(:)))==numel(E.candidate_known_ids(:)), ...
       'R82:InvalidCandidateKnownIDs');
assert(ischar(E.hazard_status) || (isstring(E.hazard_status)&&isscalar(E.hazard_status)), ...
       'R82:InvalidHazardStatus');
status=char(E.hazard_status);
assert(ismember(status,{'CONFIRMED','UNKNOWN','CLEAR'}), 'R82:InvalidHazardStatus');
assert(islogical(E.robust_inflation_on) && isscalar(E.robust_inflation_on), ...
       'R82:InvalidRobustSwitch');
assert(islogical(E.backup_on) && isscalar(E.backup_on), 'R82:InvalidBackupSwitch');
G.hazard_episode_key=sprintf('obs_%d__reference_%d',E.obstacle_id,E.active_version);
G.request_replan=strcmp(status,'CONFIRMED');
G.requires_backup=strcmp(status,'UNKNOWN');
if ~any(E.known_ids(:)==E.obstacle_id)
    G.reason='HAZARD_ID_NOT_IN_CURRENT_SENSOR_KNOWN_SET';
    G.requires_backup=true;
    return;
end
if strcmp(status,'UNKNOWN')
    G.reason='FUTURE_RISK_UNRESOLVED_FAIL_CLOSED';
    return;
end
if strcmp(status,'CLEAR')
    G.reason='NO_CONFIRMED_TRIGGER_FOR_THIS_EPISODE';
    return;
end
if E.candidate_source_version ~= E.active_version
    G.reason='STALE_REFERENCE_VERSION';
    G.requires_backup=true;
    return;
end
if ~isequal(sort(E.known_ids(:)),sort(E.candidate_known_ids(:)))
    G.reason='KNOWN_OBSTACLE_SET_CHANGED_OR_INCOMPLETE';
    G.requires_backup=true;
    return;
end
C=E.certificate;
assert(isstruct(C) && isscalar(C), 'R82:CertificateMustBeScalarStruct');
required={ 'position_c7','tangent_yaw_c6','body_geometry_continuous', ...
    'reference_kinematics','whole_known_obstacle_coverage', ...
    'actuator_and_tension','full_nominal_tail','execution_dynamics', ...
    'inter_sample_closure','source_bound_proof','ieee_sound_proof', ...
    'robust_error_tube'};
for k=1:numel(required)
    name=required{k};
    assert(isfield(C,name) && islogical(C.(name)) && isscalar(C.(name)), ...
        'R82:CertificateFieldMissingOrNotLogical:%s',name);
end
G.reference_only_eligible=C.position_c7 && C.tangent_yaw_c6 && ...
    C.body_geometry_continuous && C.reference_kinematics && ...
    C.whole_known_obstacle_coverage;
if ~G.reference_only_eligible
    G.reason='REFERENCE_CERTIFICATION_INCOMPLETE';
    return;
end
% The OFF switch removes ONLY the additional robust inflation. It does not
% turn off the dynamics/actuator/continuous-time guarantees.
other=C.actuator_and_tension && C.full_nominal_tail && ...
    C.execution_dynamics && C.inter_sample_closure && ...
    C.source_bound_proof && C.ieee_sound_proof;
if ~other
    G.reason='FULL_EXECUTION_OR_TAIL_PROOF_MISSING';
    return;
end
if E.robust_inflation_on && ~C.robust_error_tube
    G.reason='ROBUST_TUBE_PROOF_MISSING';
    return;
end
% Even here, this pure function is not a production committer. It is a
% conservative policy verdict, contingent on trustworthy proof authorities.
G.admit=true;
G.production_proved=false;
G.reason='POLICY_ADMITS_EXTERNAL_CERTIFIED_CANDIDATE_NOT_COMMITTED';
end
