classdef TRIGGER_CONTINUOUS_INTERVAL_GATE
    % TRIGGER_CONTINUOUS_INTERVAL_GATE
    % Sufficient, shape-independent temporal-exclusion theorem.
    %
    % INPUT ASSUMPTION (NOT generated or proved by this class):
    % for ALL tau in [grid(k),grid(k+1)],
    %     norm(pU(tau)-U(:,k)) <= eU(k),
    %     norm(pL(tau)-L(:,k)) <= eL(k),
    % INCLUDING current EKF uncertainty, model/integration error, HLC
    % tracking and all modeled disturbances. No value is guessed here.
    % The cable is the finite STRAIGHT segment between pU and pL, only
    % when tautness has been independently justified.
    %
    % Mathematical implication: each time-dependent cable segment stays
    % within the Minkowski sum of the LEFT segment and a ball of radius
    % max(eU,eL). A separation plane for the obstacle's true convex hull
    % (or its conservative enclosure) therefore proves a whole interval
    % clear; this works for any obstacle shape/pose, convex or nonconvex.
    %
    % This routine is NOT a reachability/tautness/rounding-error prover.
    % It NEVER claims real closed-loop safety. All 'clear' outputs are
    % CONDITIONAL ON the externally supplied bounds being rigorous.
    methods (Static)
        function R=evaluate(U,L,grid,shapes,ru,rl,rc,eU,eL)
            assert(isnumeric(U)&&isreal(U)&&size(U,1)==3 && ...
                isequal(size(U),size(L))&&all(isfinite(U(:)))&&...
                all(isfinite(L(:)))&&isnumeric(grid)&&isrow(grid)&&...
                numel(grid)==size(U,2)&&numel(grid)>=2 && ...
                all(isfinite(grid))&&all(diff(grid)>0), ...
                'TRIGGER_CONTINUOUS_INTERVAL_GATE:Trajectory', ...
                'Finite 3xN endpoints with increasing exact time grid required.');
            assert(isstruct(shapes)&&isvector(shapes) && ...
                all(isfinite([ru rl rc]))&&all([ru rl rc]>=0), ...
                'TRIGGER_CONTINUOUS_INTERVAL_GATE:Geometry', ...
                'Prepared geometry and nonnegative physical radii required.');
            nk=size(U,2)-1;
            assert(isnumeric(eU)&&isnumeric(eL)&&isreal(eU)&&isreal(eL) && ...
                isrow(eU)&&isrow(eL)&&numel(eU)==nk&&numel(eL)==nk && ...
                all(isfinite(eU))&&all(isfinite(eL)) && ...
                all(eU>=0)&&all(eL>=0), ...
                'TRIGGER_CONTINUOUS_INTERVAL_GATE:UncertifiedBound', ...
                'Nonnegative externally proven interval excursions are required.');
            nobs=numel(shapes);
            separated=false(nobs,nk);
            for j=1:nobs
                sh=shapes(j);
                assert(isfield(sh,'is_stationary')&&isfield(sh,'center0')&&...
                    isfield(sh,'bounding_radius_m')&&isfield(sh,'id'),...
                    'TRIGGER_CONTINUOUS_INTERVAL_GATE:Shape',...
                    'Prepared sensor geometry is required.');
                if ~sh.is_stationary
                    continue  % moving obstacle: this static theorem is inapplicable
                end
                c=double(sh.center0(:)); b=double(sh.bounding_radius_m);
                assert(numel(c)==3&&all(isfinite(c))&&isfinite(b)&&b>=0,...
                    'TRIGGER_CONTINUOUS_INTERVAL_GATE:Enclosure', ...
                    'Finite shape enclosure required.');
                for k=1:nk
                    u=U(:,k); l=L(:,k);
                    rU=double(ru)+double(eU(k));
                    rL=double(rl)+double(eL(k));
                    rC=double(rc)+max(double(eU(k)),double(eL(k)));
                    % Floating-point engineering guard: NOT an outward
                    % rounded interval-arithmetic implementation.
                    slack=256*eps(max([1;abs(c);abs(u);abs(l);b;rU;rL;rC]));
                    du=u-c;dl=l-c;
                    a=l-u;aa=dot(a,a);
                    if aa==0
                        nearest=u;
                    else
                        t=max(0,min(1,dot(c-u,a)/aa));
                        nearest=u+t*a;
                    end
                    % Shape-independent enclosure fast path; only proves
                    % exclusion. If near, use the true convex-hull support
                    % bound, NEVER assert contact from the outer sphere.
                    pu=norm(du)>b+rU+slack;
                    pl=norm(dl)>b+rL+slack;
                    pc=norm(nearest-c)>b+rC+slack;
                    if ~pu
                        pu=TRIGGER_GEOMETRY_CERTIFICATE.separated_point(...
                            u,rU+slack,sh,c);
                    end
                    if ~pl
                        pl=TRIGGER_GEOMETRY_CERTIFICATE.separated_point(...
                            l,rL+slack,sh,c);
                    end
                    if ~pc
                        pc=TRIGGER_GEOMETRY_CERTIFICATE.separated_cable(...
                            u,l,rC+slack,sh,c);
                    end
                    separated(j,k)=pu&&pl&&pc;
                    if ~separated(j,k)
                        % Support planes can be conservative for NONCONVEX
                        % shapes (e.g. the hole of a torus). The real-shape
                        % distance oracle supplies a 1-Lipschitz witness for
                        % the entire spatial cable and the whole TIME
                        % interval, IF the caller's excursions are proved.
                        % On budget exhaustion/error this is UNKNOWN.
                        D=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE( ...
                            u,l,sh,ru,rl,rc,eU(k),eL(k),grid(k));
                        separated(j,k)=D.conditionally_separated;
                    end
                end
            end
            unresolved=~separated;
            ids=reshape(double([shapes.id]),1,[]);
            affected=ids(any(unresolved,2).');
            [row,col]=find(unresolved);
            if isempty(col)
                first=NaN;
            else
                first=grid(min(col));
            end
            R=struct('status','CONDITIONAL_INTERVAL_GEOMETRY_ONLY',...
                'conditionally_clear',~any(unresolved(:)),...
                'separated_intervals',separated,...
                'unresolved_intervals',unresolved,...
                'possibly_at_risk_ids',affected,...
                'earliest_unresolved_left_s',first,...
                'continuous_closed_loop_safe_certified',false,...
                'requires_verified_excursion_and_tautness',true,...
                'requires_outward_rounding_audit',true,...
                'interval_count',nk,'observed_shape_count',nobs);
        end
    end
end
