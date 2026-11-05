function R=TRIGGER_NOMINAL_SWEPT_INTERVALS(U,L,t,shapes,ru,rl,rc,announced_ids,sampled_ids)
%TRIGGER_NOMINAL_SWEPT_INTERVALS Conservative, CONTINUOUS geometric check of
% the piecewise-LINEAR interpolant of numeric HLC/plant samples.
%
% For tau in [t(k),t(k+1)], affine interpolated UAV/payload centres lie
% on their respective endpoint segments, and the entire interpolated
% straight cable lies in conv{U_k,U_{k+1},L_k,L_{k+1}}. The latter is
% enclosed by its axis-aligned bounding box; the physical radii form a
% Minkowski sum. Obstacle outer balls / exact convex-hull support can ONLY
% exclude impossible overlap; a remaining hit is POSSIBLE, not confirmed.
%
% IMPORTANT: This theorem is for the INTERPOLATED NUMERICAL REFERENCE,
% NOT for the true continuous solution of the 24-state ODE or true plant.
% No bound on integration error, deviation between nominal interpolant and
% true trajectories, or EKF uncertainty is available from the current
% predictor. Never publish physical clearance based on this check.
%
% ALL observed shapes are swept, INCLUDING shapes with sampled contact.
% A sampled hit at t(k+1) does not rule out an EARLIER swept hit within
% [t(k),t(k+1)]. Skipping sampled IDs would invalidate earliest-risk time.
% sampled_ids is kept as a backward-compatible unused argument.
assert(isnumeric(U)&&isreal(U)&&size(U,1)==3 && ...
    isequal(size(U),size(L))&&all(isfinite(U(:)))&&all(isfinite(L(:))) && ...
    isnumeric(t)&&isrow(t)&&numel(t)==size(U,2) && ...
    numel(t)>=2&&all(isfinite(t))&&all(diff(t)>0) && ...
    isstruct(shapes)&&all(isfinite([ru rl rc]))&&all([ru rl rc]>=0), ...
    'TRIGGER_NOMINAL_SWEPT_INTERVALS:Inputs', ...
    'Finite 3xN endpoints, increasing time, observed shapes and radii required.');
if nargin<8,announced_ids=zeros(1,0);end
if nargin<9,sampled_ids=zeros(1,0);end
ids=reshape(double([shapes.id]),1,[]);
assert(numel(unique(ids))==numel(ids) && ...
    all(isfinite(ids)) && all(isfinite(announced_ids(:))) && ...
    all(isfinite(sampled_ids(:))), ...
    'TRIGGER_NOMINAL_SWEPT_INTERVALS:IDs','All observed IDs must be unique.');
N=numel(t)-1;
% Compute the axis-aligned box of the four endpoint vertices ONCE.
% Every point of the interpolated cable sweep lies in this box.
box_lo=min(min(U(:,1:end-1),U(:,2:end)), ...
           min(L(:,1:end-1),L(:,2:end)));
box_hi=max(max(U(:,1:end-1),U(:,2:end)), ...
           max(L(:,1:end-1),L(:,2:end)));
first=N+1; first_ids=zeros(1,0); first_parts=cell(1,0);
first_new=N+1; new_ids=zeros(1,0); new_parts=cell(1,0);
possible_ids=zeros(1,0);
intervals_considered=0;
for j=1:numel(shapes)
    sh=shapes(j); id=ids(j);
    % DO NOT skip sampled_ids: an earlier intersample hit may occur.
    assert(isfield(sh,'is_stationary')&&isequal(sh.is_stationary,true)&& ...
        isfield(sh,'center0')&&isfield(sh,'bounding_radius_m'), ...
        'TRIGGER_NOMINAL_SWEPT_INTERVALS:Shape', ...
        'A stationary, prepared, observed obstacle is required.');
    c=double(sh.center0(:)); b=double(sh.bounding_radius_m);
    assert(isequal(size(c),[3 1])&&all(isfinite(c)) && ...
        isscalar(b)&&isfinite(b)&&b>=0, ...
        'TRIGGER_NOMINAL_SWEPT_INTERVALS:OuterBall', ...
        'An observed obstacle must have a valid outer-ball enclosure.');
    stop=min(first,N);
    if ~ismember(id,announced_ids),stop=min(first_new,N);end
    % Vectorized conservative outer-ball filter. No per-sample MATLAB
    % function calls for obstacles separated from the complete 4-point
    % interpolated body/cable hull. Exact candidate tests run only nearby.
    magnitude=max([1;abs(c);abs(box_lo(:));abs(box_hi(:));b;ru;rl;rc]);
    guard=512*eps(magnitude);
    near_all=min(max(c,box_lo(:,1:stop)),box_hi(:,1:stop));
    offset=near_all-c;
    outer_limit=b+max([ru rl rc])+guard;
    candidates=find(sum(offset.^2,1)<=(outer_limit^2));
    for k=candidates
        intervals_considered=intervals_considered+1;
        a=U(:,k); aa=U(:,k+1);
        d=L(:,k); dd=L(:,k+1);
        mag=max([1;abs(c);abs(a);abs(aa);abs(d);abs(dd);b;ru;rl;rc]);
        % Conservative engineering floating-point guard; NOT a certified
        % directed-rounding proof for source geometry or plant integration.
        slack=512*eps(mag);
        hit='';
        if candidate_segment(c,a,aa,b+ru+slack) && ...
                ~separated_support(c,sh,[a aa],ru,slack,b)
            hit='UAV';
        elseif candidate_segment(c,d,dd,b+rl+slack) && ...
                ~separated_support(c,sh,[d dd],rl,slack,b)
            hit='PAYLOAD';
        else
            verts=[a aa d dd];
            lo=min(verts,[],2); hi=max(verts,[],2);
            near=min(max(c,lo),hi);
            if norm(near-c)<=b+rc+slack && ...
                    ~separated_support(c,sh,verts,rc,slack,b)
                hit='CABLE';
            end
        end
        if isempty(hit),continue;end
        possible_ids(end+1)=id; %#ok<AGROW>
        if k<first
            first=k;first_ids=id;first_parts={hit};
        elseif k==first
            first_ids(end+1)=id;first_parts{end+1}=hit; %#ok<AGROW>
        end
        if ~ismember(id,announced_ids)
            if k<first_new
                first_new=k;new_ids=id;new_parts={hit};
            elseif k==first_new
                new_ids(end+1)=id;new_parts{end+1}=hit; %#ok<AGROW>
            end
        end
        break
    end
end
R=struct('status','CONDITIONAL_PIECEWISE_LINEAR_SWEEP_ONLY', ...
    'possible_obstacle_ids',possible_ids, ...
    'all_sampled_ids_swept',true, ...
    'first_obstacle_ids',first_ids,'first_interval_index',NaN, ...
    'first_ttc_s',NaN,'first_time_bracket_s',[NaN NaN], ...
    'first_parts',{first_parts}, ...
    'new_obstacle_ids',new_ids,'new_interval_index',NaN, ...
    'new_ttc_s',NaN,'new_time_bracket_s',[NaN NaN], ...
    'new_parts',{new_parts},'evaluated_intervals',intervals_considered, ...
    'continuous_linear_interpolant_may_intersect',first<=N, ...
    'real_plant_continuous_safety_proven',false);
if first<=N
    R.first_interval_index=first;
    R.first_ttc_s=t(first);
    R.first_time_bracket_s=t(first:first+1);
end
if first_new<=N
    R.new_interval_index=first_new;
    R.new_ttc_s=t(first_new);
    R.new_time_bracket_s=t(first_new:first_new+1);
end
end

function yes=candidate_segment(c,a,b,limit)
v=b-a;den=dot(v,v);
if den==0
    nearest=a;
else
    s=min(1,max(0,dot(c-a,v)/den));
    nearest=a+s*v;
end
yes=norm(nearest-c)<=limit;
end

function yes=separated_support(c,sh,vertices,r,slack,b)
% Single valid separating plane suffices to EXCLUDE overlap; an invalid
% support dispatch never creates clearance and falls back to outer ball.
lo=min(vertices,[],2);hi=max(vertices,[],2);
nearest=min(max(c,lo),hi);
n=nearest-c;
norm_n=norm(n);
yes=false;
if norm_n<=0,return;end
n=n/norm_n;
h=shape_support(sh,n,b);
if ~(isfinite(h)&&h>=0),return;end
projection=n.'*(vertices-c);
yes=min(projection)>h+r+slack;
end

function h=shape_support(sh,n,b)
% Conservatively bound the observed solid's support in the selected
% direction. The unsupported-shape fallback is the enclosing ball, NOT a
% guessed zero-width shape. For nonconvex solids the convex hull is used.
h=b;
try
    if ~isfield(sh,'rotation')||~isfield(sh,'dimensions') || ...
            ~isfield(sh,'type'),return;end
    R=double(sh.rotation);
    if ~isequal(size(R),[3 3])||any(~isfinite(R(:))),return;end
    q=R.'*n; d=double(sh.dimensions(:));
    switch char(sh.type)
        case 'sphere'
            x=d(1);
        case 'ellipsoid'
            x=norm(d(1:3).*q);
        case 'box'
            x=sum(abs(q).*d(1:3)/2);
        case 'cylinder'
            x=d(1)*hypot(q(1),q(2))+d(2)/2*abs(q(3));
        case 'capsule'
            x=d(1)*norm(q)+d(2)/2*abs(q(3));
        otherwise
            return
    end
    % Never shrink the source-derived outer ball using a possibly
    % non-orthogonal (or inadequately validated) stored rotation.
    if any(~isfinite(d))||any(d<0),return;end
    orth=norm(R.'*R-eye(3),'fro');
    if orth>1e-10,return;end
    h=min(b,max(0,x+128*eps(max(1,abs(x)))));
catch
    h=b;
end
end
