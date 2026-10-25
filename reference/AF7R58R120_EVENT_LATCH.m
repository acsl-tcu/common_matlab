function [O,next]=AF7R58R120_EVENT_LATCH(A,previous)
% Pure event-level duplicate prevention, NOT a safety override.
% Known static obstacles persist across observations; repeated 25 ms
% notifications of the SAME pending/rejected event must never launch
% another identical candidate generation. A material change in the
% reference version, known-set geometry, or a newly uncertified previously
% certified active route can explicitly open a NEW event.
% UNKNOWN is protective and CANNOT be converted to safe by a latch.
need={'event_id','reference_version','known_set_revision', ...
 'physical_status','active_plan_certified','certified_plan_version', ...
 'event_resolved','new_material_hazard'};
for i=1:numel(need)
 assert(isfield(A,need{i}),'R120:EventAuthority', ...
   'Missing %s',need{i});
end
O=struct('launch_generation',false,'keep_active',false, ...
 'unresolved_protective',true,'status','UNKNOWN_FAIL_CLOSED', ...
 'safety_certified',false,'new_event',false);
next=struct('valid',true,'event_id',A.event_id, ...
 'reference_version',A.reference_version, ...
 'known_set_revision',A.known_set_revision, ...
 'launched_for_this_event',false, ...
 'last_physical_status',A.physical_status);
if nargin<2||isempty(previous)||~isstruct(previous)|| ...
 ~isfield(previous,'valid')||~logical(previous.valid)
  changed=true;
else
  keys={'event_id','reference_version','known_set_revision'};
  changed=false;
  for i=1:numel(keys)
     if ~isfield(previous,keys{i}) || ...
       ~isequal(previous.(keys{i}),A.(keys{i}))
       changed=true;
     end
  end
  if isfield(previous,'launched_for_this_event')
    next.launched_for_this_event=logical(previous.launched_for_this_event);
  end
end
% Only the independent physical trigger certificate, not the hazard
% identifier or a generator latch, may grant physical known-set clearance.
T=A.physical_status;
assert(isstruct(T)&&isfield(T,'proved_clear')&& ...
    isfield(T,'request'),'R120:PhysicalTriggerDecisionRequired');
if logical(T.proved_clear) && logical(A.active_plan_certified) && ...
    isequal(A.certified_plan_version,A.reference_version)
    O.keep_active=true;O.unresolved_protective=false;
    O.safety_certified=true;
    O.status='KEEP_SAME_CERTIFIED_PLAN';
    next.launched_for_this_event=false;
    return;
end
% A safety certificate can become invalid without a new sensed obstacle.
% Such an event may request a NEW plan even for the SAME ellipsoid.
changed=changed||logical(A.new_material_hazard);
O.new_event=changed;
if logical(A.event_resolved)
    O.status='RESOLVED_EVENT_NO_NEW_GENERATION';
    O.unresolved_protective=~logical(T.proved_clear);return;
end
if logical(T.request) && (changed || ~next.launched_for_this_event)
    O.launch_generation=true;
    O.status='NEW_UNCERTIFIED_HAZARD_EVENT_GENERATE_ONCE';
    next.launched_for_this_event=true;
elseif logical(T.request)
    O.status='PENDING_IDENTICAL_UNCERTIFIED_EVENT_NO_DUPLICATE_GENERATION';
else
    O.status='INCONCLUSIVE_EVENT_REQUIRES_PROTECTIVE_DECISION';
end
end
