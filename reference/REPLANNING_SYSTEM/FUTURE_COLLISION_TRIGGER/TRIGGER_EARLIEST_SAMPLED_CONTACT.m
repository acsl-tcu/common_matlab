function R=TRIGGER_EARLIEST_SAMPLED_CONTACT(u,l,sample_times_s,shapes,ru,rl,rc,time_s,runtime_options,announced_ids)
% Earliest SAMPLED nominal contact oracle used by the production trigger.
% Uses the SAME source geometry evaluator and strict predicate.
% Scan each previously announced ID only up to the earliest overall hit,
% and each not-yet-announced ID only up to the earliest NEW hit.
% Tie handling includes all IDs at the corresponding same sampled time.
% Preserve BOTH earliest global contact and earliest not-yet-announced ID.
% Otherwise an older repeat collision masks a NEW obstacle farther ahead.
% Never receives world-definition data: shapes must be already observed.
assert(isnumeric(u)&&isreal(u)&&size(u,1)==3&& ...
    isequal(size(u),size(l))&&all(isfinite(u(:)))&&all(isfinite(l(:))) && ...
    isnumeric(sample_times_s)&&isrow(sample_times_s)&& ...
    numel(sample_times_s)==size(u,2)&&all(isfinite(sample_times_s)) && ...
    all(diff(sample_times_s)>0)&&isstruct(shapes) && ...
    isscalar(ru)&&isfinite(ru)&&ru>=0 && ...
    isscalar(rl)&&isfinite(rl)&&rl>=0 && ...
    isscalar(rc)&&isfinite(rc)&&rc>=0 && ...
    isscalar(time_s)&&isfinite(time_s), ...
    'TRIGGER_EARLIEST_SAMPLED_CONTACT:Inputs', ...
    'Real 3xN predictions, increasing sample times and observed shapes required.');
n=size(u,2);
if nargin<10,announced_ids=zeros(1,0);end
assert(isnumeric(announced_ids)&&isreal(announced_ids) && ...
    all(isfinite(announced_ids(:))), ...
    'TRIGGER_EARLIEST_SAMPLED_CONTACT:ActiveIDs', ...
    'Previously announced IDs must be finite numeric IDs.');
announced_ids=reshape(double(announced_ids),1,[]);
ids=reshape(double([shapes.id]),1,[]);
assert(numel(unique(ids))==numel(ids) && all(isfinite(ids)), ...
    'TRIGGER_EARLIEST_SAMPLED_CONTACT:IDs', 'IDs must be unique.');
first=n+1;
first_ids=zeros(1,0);first_parts=cell(1,0);
first_new=n+1;
first_new_ids=zeros(1,0);first_new_parts=cell(1,0);
unknown_ids=zeros(1,0);
visited_samples=0;
body_exact_queries=0; cable_exact_queries=0;
support_exclusions=0; broadphase_exclusions=0;
for j=1:numel(shapes)
    shape=shapes(j);
    if ~isfield(shape,'is_stationary')||~isequal(shape.is_stationary,true)
        unknown_ids(end+1)=ids(j); %#ok<AGROW>
        continue
    end
    is_new=~any(announced_ids==ids(j));
    % A previously announced contact cannot hide a newly observed one.
    % Query new IDs through earliest NEW time, regardless of old repeats.
    if is_new,upto=min(first_new,n);
    else,upto=min(first,n);
    end
    try
        if nargin>=9
            [uh,lh,ch,cu,qinfo]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                u(:,1:upto),l(:,1:upto),shape,ru,rl,rc, ...
                time_s,true,runtime_options);
        else
            [uh,lh,ch,cu,qinfo]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                u(:,1:upto),l(:,1:upto),shape,ru,rl,rc,time_s,true);
        end
        body_exact_queries=body_exact_queries+qinfo.body_exact_queries;
        cable_exact_queries=cable_exact_queries+qinfo.cable_exact_queries;
        support_exclusions=support_exclusions+ ...
            qinfo.body_support_excluded+qinfo.cable_support_excluded;
        broadphase_exclusions=broadphase_exclusions+qinfo.broadphase_excluded;
    catch
        unknown_ids(end+1)=ids(j); %#ok<AGROW>
        continue
    end
    visited_samples=visited_samples+upto;
    if any(cu)
        unknown_ids(end+1)=ids(j); %#ok<AGROW>
    end
    idx=find(uh|lh|ch,1,'first');
    if isempty(idx),continue;end
    if idx<first
        first=idx;
        first_ids=ids(j);
        first_parts={part_at(uh,lh,idx)};
    elseif idx==first
        first_ids(end+1)=ids(j); %#ok<AGROW>
        first_parts{end+1}=part_at(uh,lh,idx); %#ok<AGROW>
    end
    if is_new
        if idx<first_new
            first_new=idx;
            first_new_ids=ids(j);
            first_new_parts={part_at(uh,lh,idx)};
        elseif idx==first_new
            first_new_ids(end+1)=ids(j); %#ok<AGROW>
            first_new_parts{end+1}=part_at(uh,lh,idx); %#ok<AGROW>
        end
    end
end
R=struct('status','SAMPLED_ONLY__NO_CONTINUOUS_SAFETY', ...
    'replan_requested',first<=n, 'first_obstacle_ids',first_ids, ...
    'first_sample_index',NaN,'first_ttc_s',NaN, ...
    'first_components',{first_parts}, ...
    'new_obstacle_ids',first_new_ids, ...
    'new_sample_index',NaN,'new_ttc_s',NaN, ...
    'new_components',{first_new_parts}, ...
    'uncertain_obstacle_ids',unknown_ids, ...
    'geometry_evaluated_sample_slots',visited_samples, ...
    'body_exact_queries',body_exact_queries, ...
    'cable_exact_queries',cable_exact_queries, ...
    'support_exclusions',support_exclusions, ...
    'broadphase_exclusions',broadphase_exclusions);
if first<=n
    R.first_sample_index=first;
    R.first_ttc_s=sample_times_s(first);
end
if first_new<=n
    R.new_sample_index=first_new;
    R.new_ttc_s=sample_times_s(first_new);
end
% Empty first_ids never implies a mathematically safe 4-second horizon.
end
function label=part_at(uh,lh,idx)
if uh(idx),label='UAV';
elseif lh(idx),label='PAYLOAD';
else,label='CABLE_CENTERLINE';end
end
