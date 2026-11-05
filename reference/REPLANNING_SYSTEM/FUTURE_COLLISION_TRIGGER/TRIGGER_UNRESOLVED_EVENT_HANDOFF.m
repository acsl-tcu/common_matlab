function [R,S] = TRIGGER_UNRESOLVED_EVENT_HANDOFF(ids,nominal,old,horizon_s,phase_f,enabled)
%TRIGGER_UNRESOLVED_EVENT_HANDOFF  Sound, non-committing risk/event handoff.
% This is NOT a 4-second physical safety certificate.  It prevents missing
% an OBSERVED obstacle solely because the physical 4-second enclosure is
% UNKNOWN.  A numerical sample hit is evidence only for NOMINAL contact;
% an unresolved interval is POSSIBLE contact, not confirmed contact.
%
% Contract:
%   - IDs come ONLY from the cumulative, actual observed sensor memory.
%   - If physical 4s clearance is not independently certified, every
%     observed ID remains possibly at risk (even with no sampled hit).
%   - Unknown-event is emitted ONCE per (plan_revision, observed ID).
%     A later genuinely accepted C7 plan increments revision explicitly.
%   - Original nominal new_event_ids remain a SEPARATE channel; they are
%     not shadowed by an earlier unknown event on the same obstacle.
%   - No UNKNOWN/CLEAR/nominal result grants a trajectory commit.
%   - Without the actual reference/actuator/true-set/rounding proof, this
%     gateway NEVER creates or accepts a physical SAFE certificate.
R=struct('status','UNKNOWN_HANDOFF_NOT_EVALUATED','reason','', ...
    'time_horizon_s',NaN,'observed_ids',zeros(1,0), ...
    'possibly_at_risk_ids',zeros(1,0), ...
    'new_unresolved_event_ids',zeros(1,0), ...
    'new_nominal_event_ids',zeros(1,0), ...
    'nominal_escalation_ids',zeros(1,0), ...
    'new_candidate_request_ids',zeros(1,0), ...
    'request_trajectory_candidate',false, ...
    'request_due_to_unresolved_physics',false, ...
    'possible_ttc_interval_s',[NaN NaN], ...
    'nominal_first_sampled_ttc_s',NaN, ...
    'nominal_contact_is_physical_proof',false, ...
    'known_observed_only',false,'plan_revision',uint64(0), ...
    'requires_external_backup_or_validated_plan',false, ...
    'physical_4s_clear_certified',false, ...
    'safe_to_continue_nominal_by_this_gate',false, ...
    'safe_to_commit',false);
S=old;
try
    assert(isstruct(old)&&isscalar(old)&& ...
        all(isfield(old,{'plan_revision','announced_ids'})) && ...
        isa(old.plan_revision,'uint64')&&isscalar(old.plan_revision), ...
        'TRIGGER_UNRESOLVED_EVENT_HANDOFF:State', ...
        'A state with an explicit committed-plan generation is required.');
    oldids=reshape(double(old.announced_ids),1,[]);
    validate_ids(oldids);
    assert(isnumeric(ids)&&isreal(ids)&& ...
        (isempty(ids)||isvector(ids)) && ...
        all(isfinite(ids(:))), ...
        'TRIGGER_UNRESOLVED_EVENT_HANDOFF:IDs', ...
        'Use finite actual observed-memory IDs only.');
    ids=reshape(double(ids),1,[]);
    validate_ids(ids);
    assert(isnumeric(horizon_s)&&isscalar(horizon_s)&&isreal(horizon_s) && ...
        isfinite(horizon_s)&&horizon_s>0 && ...
        islogical(phase_f)&&isscalar(phase_f) && ...
        islogical(enabled)&&isscalar(enabled), ...
        'TRIGGER_UNRESOLVED_EVENT_HANDOFF:ClockOrPhase', ...
        'Positive horizon and explicit phase/enable flags required.');
    assert(isstruct(nominal)&&isscalar(nominal), ...
        'TRIGGER_UNRESOLVED_EVENT_HANDOFF:Nominal', ...
        'The current-tick numeric predictor result must be a scalar struct.');
    R.time_horizon_s=double(horizon_s);
    R.observed_ids=ids;
    R.known_observed_only=true;
    R.plan_revision=old.plan_revision;
    if ~enabled
        R.status='UNKNOWN_TRIGGER_OR_SENSOR_DISABLED';
        R.reason='Disabled sensor/trigger cannot establish absence or issue a valid geometry-specific event.';
        R.requires_external_backup_or_validated_plan=true;
        return
    end
    if ~phase_f
        R.status='UNKNOWN_NONFLIGHT_CANDIDATE_NOT_REQUESTED';
        R.reason='The existing reference/predictor is Flight-phase only; no invented takeoff/landing contact forecast.';
        R.requires_external_backup_or_validated_plan=~isempty(ids);
        return
    end
    if isempty(ids)
        R.status='NO_OBSERVED_OBSTACLES__UNSEEN_WORLD_UNKNOWN';
        R.reason='No known sensor-observed obstacle; never claim the unsensed world clear.';
        return
    end
    % No functioning physical 4s validator is connected to this API.
    % A user-supplied or nominal "safe" Boolean is NEVER accepted as proof.
    % This remains UNKNOWN even when nominal samples are all separated.
    R.possibly_at_risk_ids=ids;
    R.possible_ttc_interval_s=[0 double(horizon_s)];
    R.requires_external_backup_or_validated_plan=true;
    R.request_due_to_unresolved_physics=true;
    is_new=~ismember(ids,oldids);
    R.new_unresolved_event_ids=ids(is_new);
    S.announced_ids=[oldids R.new_unresolved_event_ids];
    new_nominal=zeros(1,0);
    if isfield(nominal,'new_event_ids') && ~isempty(nominal.new_event_ids)
        new_nominal=reshape(double(nominal.new_event_ids),1,[]);
        validate_ids(new_nominal);
        assert(all(ismember(new_nominal,ids)), ...
            'TRIGGER_UNRESOLVED_EVENT_HANDOFF:NominalIDs', ...
            'A nominal event cannot name a never-observed obstacle.');
    end
    R.new_nominal_event_ids=new_nominal;
    R.nominal_escalation_ids=new_nominal(ismember(new_nominal,oldids));
    if isfield(nominal,'first_sampled_collision_ttc_s')
        t=double(nominal.first_sampled_collision_ttc_s);
        if isscalar(t)&&isfinite(t)&&t>=0&&t<=horizon_s
            R.nominal_first_sampled_ttc_s=t;
        end
    end
    % An earlier UNKNOWN request has already notified the generator of
    % this ID in the same committed plan.  A later nominal sample hit is
    % an escalation in diagnostics, NOT a duplicate generator request.
    R.new_candidate_request_ids=R.new_unresolved_event_ids;
    R.request_trajectory_candidate=~isempty(R.new_candidate_request_ids);
    if R.request_trajectory_candidate
        R.status='REQUEST_CANDIDATE__CONTINUOUS_PHYSICS_UNKNOWN';
    else
        R.status='UNRESOLVED_RISK_ALREADY_NOTIFIED_FOR_PLAN';
    end
    R.reason=['Possible 0..H contact is NOT ruled out for the observed IDs. ' ...
        'Sampled nominal contact times are priority hints only. ' ...
        'Do not commit an unverified trajectory; continue independent backup.' ];
catch ME
    % No speculative ID, proof, or reference-commit permission on error.
    S=old;
    R.status='UNKNOWN_HANDOFF_INVALID_INPUT';
    R.reason=[ME.identifier ' | ' ME.message];
    R.requires_external_backup_or_validated_plan=true;
end
end
function validate_ids(ids)
assert(all(isfinite(ids))&&all(ids>0)&&all(ids==fix(ids)) && ...
    numel(unique(ids))==numel(ids), ...
    'TRIGGER_UNRESOLVED_EVENT_HANDOFF:IDDomain', ...
    'Observed IDs must be finite, distinct, positive integers.');
end
