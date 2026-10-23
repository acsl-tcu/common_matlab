function R=AF7R58R44R2_RETURN_HULL(src)
% Read-only convex-hull diagnostic of the *actual saved* correction layers.
% Missing file, unavailable CP or unsupported layout returns a status, not a
% fake certificate. No sampled collision clearance claim.
R=struct('schema','AF7R58R44R2_RETURN_HULL_V1', ...
    'status','NOT_EVALUATED','actual_polygons_available',false, ...
    'min_signed_cp_m',NaN,'max_signed_cp_m',NaN, ...
    'phase_layers_present',false, ...
    'theoretical_one_sided_sufficient_if_exact_control_points',false, ...
    'continuous_combined_return_certified',false, ...
    'wholebody_collision_certified',false, ...
    'overshoot_necessary',false, ...
    'automatic_commit',false);
R.segments=table();
if ischar(src) || (isstring(src) && isscalar(src))
    fn=char(src);
    if ~isfile(fn)
        R.status='SNAPSHOT_FILE_MISSING__USE_RUN_AND_CAPTURE';
        fprintf('[R44R2] Snapshot not found: %s\n',fn);
        return;
    end
    D=load(fn,'Snapshot');
    if ~isfield(D,'Snapshot')
        R.status='MAT_FILE_WITHOUT_SNAPSHOT';
        return;
    end
    S=D.Snapshot;
else
    S=src;
end
if ~isstruct(S) || ~isfield(S,'schema') || ...
        ~any(strcmp(char(S.schema),{ ...
        'AF7R58R44R2_ACTIVE_SNAPSHOT_V1', ...
        'AF7R58R44R1_ACTIVE_SNAPSHOT_V1'}))
    R.status='UNSUPPORTED_SNAPSHOT_SCHEMA';
    return;
end
if ~isfield(S,'contains_committed_c7_control_points') || ...
        ~logical(S.contains_committed_c7_control_points) || ...
        ~isfield(S,'corrections') || isempty(S.corrections)
    R.status='NO_COMMITTED_CONTROL_POLYGONS_IN_SNAPSHOT';
    return;
end
C=S.corrections;
if ~isstruct(C)
    R.status='CORRECTION_LAYOUT_NOT_STRUCT';
    return;
end
rows=zeros(0,6);
axis=[];
for k=1:numel(C)
    if ~all(isfield(C(k),{'segment_t0','segment_t1','control_points'}))
        R.status='MISSING_CANONICAL_CONTROL_POINT_FIELDS';
        return;
    end
    a=double(C(k).segment_t0(:));
    b=double(C(k).segment_t1(:));
    if numel(a)~=numel(b) || isempty(a) || ...
            ~all(isfinite([a;b])) || ~all(b>a)
        R.status='INVALID_SEGMENT_CLOCKS';
        return;
    end
    Qs=C(k).control_points;
    for j=1:numel(a)
        if iscell(Qs)
            if numel(Qs)<j
                R.status='MISSING_POLYGON_CELL';
                return;
            end
            Q=double(Qs{j});
        elseif isnumeric(Qs) && ndims(Qs)==3
            Q=double(Qs(:,:,j));
        elseif isnumeric(Qs) && numel(a)==1
            Q=double(Qs);
        else
            R.status='AMBIGUOUS_CONTROL_POLYGON_LAYOUT';
            return;
        end
        if ~isequal(size(Q),[3 16]) || ~all(isfinite(Q(:)))
            R.status='INVALID_BEZIER_D15_CONTROL_POLYGON';
            return;
        end
        if isempty(axis)
            [mag,ix]=max(sqrt(sum(Q.^2,1)));
            if ~(isfinite(mag) && mag>0)
                R.status='DIAGNOSTIC_DIRECTION_UNDEFINED';
                return;
            end
            axis=Q(:,ix)/mag;
        end
        d=axis.'*Q;
        rows(end+1,:)=[k,j,a(j),b(j),min(d),max(d)]; %#ok<AGROW>
    end
end
R.axis=axis;
R.segments=array2table(rows,'VariableNames', ...
    {'layer','segment','t0_s','t1_s','signed_cp_min_m','signed_cp_max_m'});
R.min_signed_cp_m=min(rows(:,5));
R.max_signed_cp_m=max(rows(:,6));
R.phase_layers_present=isfield(S,'phase_layers') && ~isempty(S.phase_layers);
R.actual_polygons_available=true;
R.theoretical_one_sided_sufficient_if_exact_control_points= ...
    all(rows(:,5)>=0) && ~R.phase_layers_present;
R.status='ACTUAL_CONTROL_POLYGONS_INSPECTED__NOT_FULL_REFERENCE_PROOF';
end
