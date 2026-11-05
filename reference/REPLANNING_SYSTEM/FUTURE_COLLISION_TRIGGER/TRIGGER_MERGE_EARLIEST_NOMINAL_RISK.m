function C=TRIGGER_MERGE_EARLIEST_NOMINAL_RISK(sampled,swept,grid)
%TRIGGER_MERGE_EARLIEST_NOMINAL_RISK Chronological nominal-risk decision.
% The intervals in SWEPT conservatively enclose the affine-interpolated
% UAV, payload and straight cable at EVERY time in the sampling interval.
% Interval START is an EARLIEST POSSIBLE time, not a proven contact time.
% A sampled contact is a confirmed contact with the NUMERIC SAMPLE, not a
% certified true-ODE or real-world impact.
%
% Merge both sources for ALL observed obstacles. Previous events are not
% allowed to mask a different, newly triggerable obstacle. Tied IDs are
% returned together, independently of obstacle traversal order.
assert(isstruct(sampled)&&isstruct(swept)&&isnumeric(grid)&&isrow(grid) ...
    &&numel(grid)>=2&&all(isfinite(grid))&&all(diff(grid)>0), ...
    'TRIGGER_MERGE_EARLIEST_NOMINAL_RISK:Inputs','Invalid contact evidence.');
S=make_sample(sampled.first_obstacle_ids,sampled.first_ttc_s, ...
    sampled.first_sample_index,sampled.first_components,grid);
SN=make_sample(sampled.new_obstacle_ids,sampled.new_ttc_s, ...
    sampled.new_sample_index,sampled.new_components,grid);
W=make_swept(swept.first_obstacle_ids,swept.first_ttc_s, ...
    swept.first_time_bracket_s,swept.first_parts);
WN=make_swept(swept.new_obstacle_ids,swept.new_ttc_s, ...
    swept.new_time_bracket_s,swept.new_parts);
all_first=earlier(S,W);
all_new=earlier(SN,WN);
selected=all_first;
if ~isempty(all_new.ids)
    selected=all_new;
end
C=struct('has_risk',~isempty(all_first.ids), ...
    'has_new',~isempty(all_new.ids), ...
    'first',all_first,'first_new',all_new,'selected',selected);
end

function x=empty()
x=struct('ids',zeros(1,0),'ttc_s',NaN, ...
    'bracket_s',[NaN NaN],'component','NONE', ...
    'kind','NONE','sampled_confirmed',false, ...
    'intersample_possible',false);
end

function x=make_sample(ids,t,index,components,grid)
x=empty();
if isempty(ids),return;end
assert(isscalar(t)&&isfinite(t)&&isscalar(index)&& ...
    index>=1&&index<=numel(grid)&&grid(index)==t, ...
    'TRIGGER_MERGE_EARLIEST_NOMINAL_RISK:Sample','Invalid sampled time.');
x.ids=reshape(double(ids),1,[]);x.ttc_s=t;
x.bracket_s=[t t];
x.component=components{1};x.kind='SAMPLED_CONTACT';
x.sampled_confirmed=true;
end

function x=make_swept(ids,t,bracket,components)
x=empty();
if isempty(ids),return;end
assert(isscalar(t)&&isfinite(t)&&numel(bracket)==2 && ...
    all(isfinite(bracket))&&bracket(1)==t&&bracket(2)>t, ...
    'TRIGGER_MERGE_EARLIEST_NOMINAL_RISK:Sweep','Invalid interval bracket.');
x.ids=reshape(double(ids),1,[]);x.ttc_s=t;
x.bracket_s=reshape(bracket,1,2);
x.component=components{1};x.kind='POSSIBLE_INTERSAMPLE_CONTACT';
x.intersample_possible=true;
end

function z=earlier(a,b)
if isempty(a.ids),z=b;return;end
if isempty(b.ids),z=a;return;end
if a.ttc_s<b.ttc_s,z=a;return;end
if b.ttc_s<a.ttc_s,z=b;return;end
% Exact clock-grid ties: include EVERY ID; do not throw away a candidate.
z=a;
z.ids=unique([a.ids b.ids],'sorted');
z.bracket_s=[min(a.bracket_s(1),b.bracket_s(1)) ...
             max(a.bracket_s(2),b.bracket_s(2))];
z.sampled_confirmed=a.sampled_confirmed||b.sampled_confirmed;
z.intersample_possible=a.intersample_possible||b.intersample_possible;
if z.intersample_possible
    z.kind='POSSIBLE_INTERSAMPLE_CONTACT';
    if ~z.sampled_confirmed
        z.component=b.component;
    end
end
end
