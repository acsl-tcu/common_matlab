function R=TRIGGER_FIRST_UNPROVED_INTERVAL(times_s,observed_ids,separated)
% TRIGGER_FIRST_UNPROVED_INTERVAL  Chronological multi-obstacle risk handoff.
% separated(j,k)==true is a CONDITIONAL continuous-time separator witness
% for the ENTIRE [times(k), times(k+1)] from an upstream reachability gate.
% A false entry is UNKNOWN, not proof of contact. All simultaneous IDs in
% the earliest unproved interval are retained; no fixed obstacle count.
% The output NEVER authorizes changing the flight reference.
R=struct('status','UNKNOWN_INTERVAL_SEPARATION_INPUT', ...
    'conditional_replan_required',true, ...
    'obstacle_id',NaN,'earliest_possible_ids',zeros(1,0), ...
    'first_interval_index',NaN,'first_possible_ttc_left_s',NaN, ...
    'first_possible_ttc_right_s',NaN, ...
    'conditional_clear_intervals',0,'safe_to_commit',false,'reason','');
try
    t=reshape(double(times_s),1,[]);
    ids=reshape(double(observed_ids),1,[]);
    assert(isnumeric(times_s)&&isreal(times_s) && ...
        numel(t)>=2 && all(isfinite(t))&&all(diff(t)>0) && ...
        isnumeric(observed_ids)&&isreal(observed_ids) && ...
        all(isfinite(ids))&&all(ids>0)&&all(ids==fix(ids)) && ...
        numel(unique(ids))==numel(ids) && ...
        islogical(separated) && ...
        isequal(size(separated),[numel(ids) numel(t)-1]), ...
        'TRIGGER_FIRST_UNPROVED_INTERVAL:Inputs', ...
        'Only real observed IDs and complete time-interval witnesses accepted.');
    if isempty(ids)
        R.status='NO_OBSERVED_OBSTACLES__UNKNOWN_WORLD';
        R.conditional_replan_required=false;
        R.reason='No sensed obstacle was supplied; unknown world is not certified safe.';
        return
    end
    k=find(any(~separated,1),1,'first');
    if ~isempty(k)
        all_ids=ids((~separated(:,k)).');
        R.status='CONDITIONAL_EARLIEST_UNPROVED_INTERVAL__REPLAN';
        R.earliest_possible_ids=all_ids;
        R.obstacle_id=all_ids(1);
        R.first_interval_index=k;
        R.first_possible_ttc_left_s=t(k)-t(1);
        R.first_possible_ttc_right_s=t(k+1)-t(1);
        R.conditional_clear_intervals=k-1;
        R.reason='Sufficient separation is not shown; collision itself is NOT proven.';
        return
    end
    R.conditional_clear_intervals=numel(t)-1;
    R.status='CONDITIONAL_ALL_OBSERVED_INTERVALS_SEPARATED__NOT_FLIGHT_PROOF';
    R.conditional_replan_required=false;
    R.reason=['All provided witnessed intervals separated, conditional on ' ...
        'externally verified all-time reach tubes and sound geometry arithmetic.'];
catch ME
    R.status='UNKNOWN_INTERVAL_SEPARATION_INPUT';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
