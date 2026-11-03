function [new_ids, duplicate_suppressed, next_active]=TRIGGER_STABLE_EVENT_IDS(sample_hits,active_ids,known_ids)
% Stable ID-set transition for the observed-only nominal contact trigger.
% Replaces setdiff/ismember/union/intersect at the FIRST contact event.
% No geometry/prediction changes and no certification or auto-commit.
% Original MATLAB semantics:
% new=setdiff(hits,active,'stable'); duplicate=any(ismember(hits,active));
% next=intersect(union(active,new,'stable'),known,'stable').
% Handles empty/row/column/duplicate inputs without an ID-count limit.
assert(isnumeric(sample_hits)&&isreal(sample_hits) && ...
       isnumeric(active_ids)&&isreal(active_ids) && ...
       isnumeric(known_ids)&&isreal(known_ids), ...
       'TRIGGER_STABLE_EVENT_IDS:Type','ID sets must be real numeric.');
hits=reshape(double(sample_hits),1,[]);
active=reshape(double(active_ids),1,[]);
known=reshape(double(known_ids),1,[]);
assert(all(isfinite([hits,active,known])) && ...
       all([hits,active,known]>0) && ...
       all([hits,active,known]==fix([hits,active,known])), ...
       'TRIGGER_STABLE_EVENT_IDS:IDs','IDs must be positive finite integers.');
new_ids=zeros(1,0);
duplicate_suppressed=false;
for j=1:numel(hits)
    id=hits(j);
    if any(active==id)
        duplicate_suppressed=true;
    elseif ~any(new_ids==id)
        new_ids(end+1)=id; %#ok<AGROW> rare event: bounded by current observed IDs
    end
end
combined=[active,new_ids];
next_active=zeros(1,0);
for j=1:numel(combined)
    id=combined(j);
    if any(known==id) && ~any(next_active==id)
        next_active(end+1)=id; %#ok<AGROW> stable unique, event-only
    end
end
end
