function R=TRIGGER_EARLIEST_SAMPLED_CONTACT(u,l,sample_times_s,shapes,ru,rl,rc,time_s)
% Optional candidate: earliest SAMPLED nominal contact only, not safety.
% Uses the SAME source geometry evaluator and strict predicate. Once a
% candidate sample i is known, later obstacles are checked only through i.
% Tie handling includes every obstacle at that SAME sampled time.
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
ids=reshape(double([shapes.id]),1,[]);
assert(numel(unique(ids))==numel(ids) && all(isfinite(ids)), ...
    'TRIGGER_EARLIEST_SAMPLED_CONTACT:IDs', 'IDs must be unique.');
first=n+1;
first_ids=zeros(1,0);first_parts=cell(1,0);
unknown_ids=zeros(1,0);
visited_samples=0;
for j=1:numel(shapes)
    shape=shapes(j);
    if ~isfield(shape,'is_stationary')||~isequal(shape.is_stationary,true)
        unknown_ids(end+1)=ids(j); %#ok<AGROW>
        continue
    end
    upto=min(first,n);
    try
        [uh,lh,ch,cu]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            u(:,1:upto),l(:,1:upto),shape,ru,rl,rc,time_s,true);
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
end
R=struct('status','SAMPLED_ONLY__NO_CONTINUOUS_SAFETY', ...
    'replan_requested',first<=n, 'first_obstacle_ids',first_ids, ...
    'first_sample_index',NaN,'first_ttc_s',NaN, ...
    'first_components',{first_parts}, ...
    'uncertain_obstacle_ids',unknown_ids, ...
    'geometry_evaluated_sample_slots',visited_samples);
if first<=n
    R.first_sample_index=first;
    R.first_ttc_s=sample_times_s(first);
end
% Empty first_ids never implies a mathematically safe 4-second horizon.
end
function label=part_at(uh,lh,idx)
if uh(idx),label='UAV';
elseif lh(idx),label='PAYLOAD';
else,label='CABLE_CENTERLINE';end
end
