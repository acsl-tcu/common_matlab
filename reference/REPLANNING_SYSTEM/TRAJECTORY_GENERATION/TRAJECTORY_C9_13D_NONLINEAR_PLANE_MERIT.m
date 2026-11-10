function [alpha,D]=TRAJECTORY_C9_13D_NONLINEAR_PLANE_MERIT(Q,req,current,step,Tmax)
% Search-only nonlinear globalization for the SAME 13D single C9 polynomial.
% Every original shape/ground half-space is re-evaluated at the ACTUAL
% position of its payload, cable, or UAV probe, not at its QP tangent.
% No point evaluation is ever used as a full-time safety certificate.
% The fixed support planes are computed by TRAJECTORY_C9_13D_NATURAL_QP.
% For the sampled-plane violation merit Phi=max(0,max_i(h_i-n_i'*x_i)):
% accept the largest dyadic step reducing Phi (strictly, if Phi>0),
% or preserving Phi=0. The dyadic search ends at machine resolution;
% no empirical speed, geometry or duration thresholds are introduced.
clock=tic;
D=struct('status','NOT_RUN','accepted',false,'alpha',NaN, ...
    'backtracks',0,'merit_before_m',NaN,'merit_after_m',NaN, ...
    'elapsed_ms',NaN,'trials',0,'baseline_model_parity_m',NaN, ...
    'interval_hull_enabled',false,'interval_before_m',NaN, ...
    'interval_after_m',NaN,'interval_evaluations',0, ...
    'interval_rejected_trials',0,'interval_status','NOT_REQUESTED');
alpha=NaN;
assert(isstruct(Q)&&isstruct(req)&&isstruct(current) && ...
    isnumeric(step)&&isequal(size(step),[13 1])&&all(isfinite(step)), ...
    'C9_13D_LINESEARCH:Input','Finite 13D direction required.');
if ~isfield(Q,'geometry_plane_normals_3xm') || ...
        ~isfield(Q,'geometry_plane_thresholds_m') || ...
        ~isfield(Q,'geometry_plane_phases') || ...
        ~isfield(Q,'geometry_plane_lambdas')
    D.status='UNKNOWN_MISSING_PHYSICAL_PLANE_DATA';
    D.elapsed_ms=1000*toc(clock);return
end
normals=Q.geometry_plane_normals_3xm;
thresholds=Q.geometry_plane_thresholds_m(:);
fractions=Q.geometry_plane_lambdas(:);
u=Q.geometry_plane_phases(:);
M=numel(u);
if ~(isequal(size(normals),[3 M])&&numel(thresholds)==M && ...
        numel(fractions)==M && all(isfinite(normals(:))) && ...
        all(isfinite(thresholds))&&all(isfinite(u)) && ...
        all(u>=0 & u<=1)&&all(fractions>=0 & fractions<=1))
    D.status='UNKNOWN_INVALID_PHYSICAL_PLANE_DATA';
    D.elapsed_ms=1000*toc(clock);return
end
T=current.duration_s;
F=current.control_points_m(:,11:14);
if M==0
    if isfinite(T+step(13)) && T+step(13)>0 && T+step(13)<=Tmax
        alpha=1;D.accepted=true;D.status='NO_GEOMETRY_PLANES_FULL_STEP';
        D.merit_before_m=0;D.merit_after_m=0;
    else
        D.status='UNKNOWN_NO_VALID_DURATION_STEP';
    end
    D.elapsed_ms=1000*toc(clock);return
end
% The physical probe locations (normalized phase) never change during
% this line search. Reuse the degree-23/21 Bernstein basis matrices across
% ALL trial durations/free-control candidates, without a shape support call.
[unique_phases,~,location]=unique(u);
B0=c9_basis(23,unique_phases);
B2=c9_basis(21,unique_phases);
% Read the exact nonlinear current candidate. Every shape has already
% been support-evaluated in Q: the score does NOT repeat support calls.
[m0,ok]=point_merit(current,req,location,fractions,normals,thresholds,B0,B2);
if ~ok
    D.status='UNKNOWN_DEGENERATE_CURRENT_TAUT_DIRECTION';
    D.elapsed_ms=1000*toc(clock);return
end
D.merit_before_m=m0;
% Optional stronger globalization: a conservative bound over complete
% Voronoi time CELLS, rather than the finite point samples alone. This
% avoids declaring progress based only on zeros at the QP probe times.
% It is still a SEARCH score, not a shape/force or execution certificate.
use_hull=false;
if isfield(req.search_options,'enable_13d_interval_hull_line_search')
    flag=req.search_options.enable_13d_interval_hull_line_search;
    assert((isnumeric(flag)||islogical(flag))&&isscalar(flag)&& ...
        isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
        'C9_13D:HullSwitch','Interval hull line search flag must be 0/1.');
    use_hull=logical(flag);
end
D.interval_hull_enabled=use_hull;
% Evaluate dozens of trial candidates against the SAME Bernstein cells.
% Precompute each cell's EXACT linear restriction operator once, not on
% every backtracking step.  This changes SEARCH cost, never the oracle.
use_cache=false;
if isfield(req.search_options,'use_cached_13d_hull_cells')
    v=req.search_options.use_cached_13d_hull_cells;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&&isreal(v) ...
        &&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D:HullCacheFlag','Hull cache flag must be 0/1.');
    use_cache=use_hull && logical(v);
end
hullcache=struct('enabled',use_cache);
if use_hull
    [E0,hullcache]=TRAJECTORY_C9_13D_INTERVAL_HULL_SCORE( ...
        current,Q,req,hullcache);
    D.interval_evaluations=D.interval_evaluations+1;
    D.interval_status=E0.status;
    if ~E0.valid
        D.status='UNKNOWN_BASELINE_CELL_HULL_NOT_ENCLOSED';
        D.elapsed_ms=1000*toc(clock);return
    end
    D.interval_before_m=E0.bound_m;
end
% Q's linearization and direct actual positions must agree at the anchor.
% This is a numerical consistency check, NOT a physical safety margin.
if isfield(Q,'geometry_plane_linearized_rhs_m')
    model=max([0;Q.geometry_plane_linearized_rhs_m(:)]);
    D.baseline_model_parity_m=abs(model-m0);
    scale=max([1;abs(model);abs(m0);abs(thresholds(:))]);
    if D.baseline_model_parity_m>256*eps(scale)
        D.status='UNKNOWN_GEOMETRY_LINEARIZATION_ANCHOR_MISMATCH';
        D.elapsed_ms=1000*toc(clock);return
    end
end
alpha_trial=1;
% The existing QP feasibility tolerance is a NUMERICAL stopping scale,
% not an additional physical clearance or empirical avoidance radius.
free_resolution=max(double(req.search_options.projection_tolerance_m), ...
    eps(max([1;abs(F(:))])));
duration_resolution=eps(max(1,abs(T)));
while alpha_trial>0
    % An optional caller-provided service budget stops futile dyadic
    % retries at the next checkpoint; it cannot interrupt a MATLAB call.
    if isfield(req.search_options,'internal_13d_start_clock') && ...
            isfield(req.search_options,'internal_13d_budget_ms') && ...
            1000*toc(req.search_options.internal_13d_start_clock)>= ...
                req.search_options.internal_13d_budget_ms
        D.status='UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED';
        D.elapsed_ms=1000*toc(clock);
        return
    end
    if norm(alpha_trial*step(1:12))<=free_resolution && ...
            abs(alpha_trial*step(13))<=duration_resolution
        break
    end
    Tnext=T+alpha_trial*step(13);
    Fnext=F+alpha_trial*reshape(step(1:12),3,4);
    % A valid duration and a representably different candidate are required.
    if ~(isfinite(Tnext)&&Tnext>0&&Tnext<=Tmax&& ...
            all(isfinite(Fnext(:))))
        improve=false;
    elseif isequal(Fnext,F)&&Tnext==T
        break
    else
        trial=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
            req.t0_s,Tnext,req.start_jets,req.end_jets,Fnext);
        [mnew,valid]=point_merit(trial,req,location,fractions,normals,thresholds,B0,B2);
        roundTol=256*eps(max([1;abs(m0);abs(thresholds(:))]));
        if m0>roundTol
            improve=valid && mnew<m0-roundTol;
        else
            improve=valid && mnew<=roundTol;
        end
    end
    D.trials=D.trials+1;
    if improve && use_hull
        [E,hullcache]=TRAJECTORY_C9_13D_INTERVAL_HULL_SCORE( ...
            trial,Q,req,hullcache);
        D.interval_evaluations=D.interval_evaluations+1;
        D.interval_status=E.status;
        if ~E.valid
            improve=false;
        else
            hull_tol=256*eps(max([1;abs(D.interval_before_m); ...
                abs(E.bound_m);abs(thresholds(:))]));
            if D.interval_before_m>hull_tol
                % Require actual descent of the conservative cell bound.
                improve=E.bound_m<D.interval_before_m-hull_tol;
            else
                improve=E.bound_m<=hull_tol;
            end
            if improve
                D.interval_after_m=E.bound_m;
            end
        end
        if ~improve
            D.interval_rejected_trials=D.interval_rejected_trials+1;
        end
    end
    if improve
        alpha=alpha_trial;
        D.alpha=alpha;
        D.accepted=true;
        D.merit_after_m=mnew;
        D.status='SAMPLED_NONLINEAR_MERIT_DESCENT__FULL_TIME_PROOF_STILL_REQUIRED';
        D.elapsed_ms=1000*toc(clock);return
    end
    alpha_trial=alpha_trial/2;
    D.backtracks=D.backtracks+1;
end
D.status='UNKNOWN_NO_REPRESENTABLE_NONLINEAR_MERIT_PROGRESS';
D.elapsed_ms=1000*toc(clock);
end

function [merit,valid]=point_merit(piece,req,location,lam,n,h,B0,B2)
% Exact nominal flat-model body positions at finitely many search probes.
% Flight safety is NEVER inferred from the finite set.
valid=false;merit=Inf;
P=double(piece.control_points_m);
T=double(piece.duration_s);
if ~(isequal(size(P),[3 24])&&isfinite(T)&&T>0)
    return
end
p=P*B0;
a=(23*22/T^2)*diff(P,2,2)*B2;
g=double(req.authority.gravity_world_mps2(:));
w=bsxfun(@minus,g,a);
s=sqrt(sum(w.^2,1));
if any(~isfinite(s)|s<=0),return;end
q=bsxfun(@rdivide,w,s);
L=double(req.authority.cable_length_m);
centers=p(:,location)-L*bsxfun(@times,q(:,location),lam.');
res=h(:)-sum(n.*centers,1).';
if any(~isfinite(res)),return;end
merit=max([0;res]);
valid=true;
end

function B=c9_basis(n,u)
% Stable de Casteljau-compatible basis recurrence; endpoints exact.
u=double(u(:)).';
B=zeros(n+1,numel(u));
for j=1:numel(u)
    t=u(j);
    if t==0,B(1,j)=1;continue;end
    if t==1,B(end,j)=1;continue;end
    if t<=0.5
        B(1,j)=(1-t)^n;
        for k=1:n
            B(k+1,j)=B(k,j)*((n-k+1)/k)*(t/(1-t));
        end
    else
        B(end,j)=t^n;
        for k=n:-1:1
            B(k,j)=B(k+1,j)*(k/(n-k+1))*((1-t)/t);
        end
    end
end
end
