function R=AF7R58R44_CURRENT_RETURN_HULL(P)
% Inspect the actual accepted correction Bezier layers, no synthetic Creturn.
% Convex hull result is a sufficient sign test for individual correction layers.
% It is NOT a complete combined reference/obstacle certificate.
if nargin<1
    assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
       'Actual replanner_instance is unavailable.');
    P=evalin('base','replanner_instance');
end
assert(isa(P,'REPLANNING_SYSTEM') && ...
  strcmpi(char(P.active_reference_mode),'REPAIRED'), ...
  'An actually committed REPAIRED reference is required.');
A=P.active_repaired_reference;
assert(isfield(A,'batch_flat_model') && isstruct(A.batch_flat_model), ...
    'Active batch_flat_model is not available.');
M=A.batch_flat_model;
assert(isfield(M,'corrections'),'No current correction layers.');
C=M.corrections;
assert(~isempty(C),'No current correction layers.');
% Select a *diagnostic axis* from the first layer's largest nonzero CP,
% not a synthetic hand-written axis. Sign of combined path is NOT inferred.
first=C(1).control_points;
if iscell(first)
    CP=double(first{1});
elseif ndims(first)==3
    CP=double(first(:,:,1));
else
    CP=double(first);
end
assert(isequal(size(CP),[3 16]),'First canonical control polygon is not 3x16.');
[am,ii]=max(sqrt(sum(CP.^2,1)));
assert(isfinite(am)&&am>0,'Cannot derive diagnostic axis from zero correction.');
n=CP(:,ii)/am;
R=struct('schema','R44_ACTUAL_RETURN_CONTROL_HULL_V1', ...
    'axis',n,'active_version',double(P.active_reference_version), ...
    'layer_count',numel(C),'source','actual certified batch_flat_model', ...
    'continuous_combined_return_certified',false,'overshoot_necessary',false);
rows=[];
for k=1:numel(C)
    it=C(k);
    a=double(it.segment_t0(:));b=double(it.segment_t1(:));
    assert(numel(a)==numel(b) && all(b>a),'Invalid segment clocks.');
    K=numel(a);
    for j=1:K
        if iscell(it.control_points)
            Q=double(it.control_points{j});
        elseif ndims(it.control_points)==3
            Q=double(it.control_points(:,:,j));
        else
            assert(K==1,'Ambiguous packed polygon.');
            Q=double(it.control_points);
        end
        assert(isequal(size(Q),[3 16])&&all(isfinite(Q),'all'), ...
            'Invalid canonical control polygon.');
        d=n.'*Q;
        rows(end+1,:)=[k,j,a(j),b(j),min(d),max(d)]; %#ok<AGROW>
    end
end
R.segments=array2table(rows,'VariableNames', ...
    {'layer','segment','t0_s','t1_s','signed_cp_min_m','signed_cp_max_m'});
R.every_layer_nonnegative_cp=all(rows(:,5)>=0);
R.any_negative_cp=any(rows(:,5)<0);
R.phase_layers_present=isfield(M,'phase_layers') && ~isempty(M.phase_layers);
R.theoretical_one_sided_sufficient_if_exact_control_points= ...
    R.every_layer_nonnegative_cp && ~R.phase_layers_present;
R.status='READ_ONLY_LIVE_BEZIER_HULL__NO_COLLISION_OR_JET_CERTIFICATE';
end
