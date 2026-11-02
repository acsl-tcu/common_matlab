classdef TRIGGER_BERNSTEIN_BODY_SWEEP
    % Continuous spatial separation of a Bernstein-polynomial suspended body.
    %
    % INPUT: 3x(n+1) Bernstein control matrices for UAV and payload
    % positions over the SAME time interval. Any degree n>=0 is supported.
    % They describe a NOMINAL polynomial trajectory, not a verified
    % EKF/HLC/plant closed-loop reachability tube. The straight, taut cable
    % centerline at parameter s is (1-s)*pU(t) + s*pL(t), 0<=s<=1.
    %
    % Convex-hull property: all possible cable positions over the entire
    % interval belong to conv([U_controls,L_controls]). A separating plane
    % for that hull and a conservative supporting plane for the TRUE
    % obstacle solid proves no nominal full-time cable contact. The UAV
    % and load each use separate control hulls and their actual sphere radii.
    % Adding eU/eL expands the radii; these are ONLY CONDITIONAL inputs,
    % not inferred or certified tracking bounds. No unknown obstacle is
    % introduced; shapes must have come from actual observation/memory.
    %
    % The support oracle is shared with the existing sensor/trigger. An
    % enclosing sphere is used ONLY to exclude; never to assert contact.
    % This double-precision implementation does not implement directed
    % outward rounding. Hence it returns a geometric sufficient test in
    % floating arithmetic, NOT a formal numerical/runtime safety certificate.
    methods (Static)
        function R=evaluate(U,L,shapes,ru,rl,rc,eU,eL)
            if nargin<7, eU=0;end
            if nargin<8, eL=0;end
            assert(isnumeric(U)&&isreal(U)&&size(U,1)==3 && ...
                isequal(size(U),size(L))&&size(U,2)>=1 && ...
                all(isfinite(U(:))) && all(isfinite(L(:))), ...
                'TRIGGER_BERNSTEIN_BODY_SWEEP:Controls', ...
                'Both endpoints need finite equal-degree 3x(n+1) Bernstein controls.');
            assert(isstruct(shapes) && ...
                all(isfinite([ru rl rc eU eL])) && ...
                all([ru rl rc eU eL]>=0), ...
                'TRIGGER_BERNSTEIN_BODY_SWEEP:Radii', ...
                'Only nonnegative externally supplied radii/tubes are allowed.');
            u=double(U);l=double(L);
            n=numel(shapes);
            clear_u=false(1,n);clear_l=false(1,n);clear_c=false(1,n);
            errors=cell(1,n);
            for j=1:n
                sh=shapes(j);
                try
                    assert(isfield(sh,'id')&&isfield(sh,'center0')&& ...
                        isfield(sh,'bounding_radius_m')&& ...
                        isfield(sh,'is_stationary') && sh.is_stationary, ...
                        'TRIGGER_BERNSTEIN_BODY_SWEEP:Shape', ...
                        'A valid observed static solid is required.');
                    c=double(sh.center0(:));b=double(sh.bounding_radius_m);
                    assert(numel(c)==3&&all(isfinite(c))&& ...
                        isscalar(b)&&isfinite(b)&&b>=0, ...
                        'TRIGGER_BERNSTEIN_BODY_SWEEP:Enclosure', ...
                        'Original solid needs a finite known enclosing radius.');
                    clear_u(j)=TRIGGER_BERNSTEIN_BODY_SWEEP. ...
                        control_hull_separated(u,c,sh,b,ru+eU);
                    clear_l(j)=TRIGGER_BERNSTEIN_BODY_SWEEP. ...
                        control_hull_separated(l,c,sh,b,rl+eL);
                    % Straight cable at EVERY time and EVERY point s.
                    % Error interpolation is bounded by max(eU,eL).
                    clear_c(j)=TRIGGER_BERNSTEIN_BODY_SWEEP. ...
                        control_hull_separated([u l],c,sh,b,rc+max(eU,eL));
                catch ME
                    errors{j}=sprintf('%s: %s',ME.identifier,ME.message);
                    % Fail closed; this never implies a collision.
                end
            end
            sep=clear_u & clear_l & clear_c;
            ids=reshape(double([shapes.id]),1,[]);
            R=struct('status','REFERENCE_POLYNOMIAL_GEOMETRY_ONLY', ...
                'nominal_separated',all(sep), ...
                'conditionally_separated_with_supplied_tubes',all(sep), ...
                'per_shape_nominal_exclusion',sep, ...
                'unresolved_ids',ids(~sep), ...
                'uav_hull_excluded',clear_u, ...
                'payload_hull_excluded',clear_l, ...
                'cable_hull_excluded',clear_c, ...
                'query_errors',{errors}, ...
                'interval_is_continuous_in_time',true, ...
                'includes_all_cable_parameters',true, ...
                'tracks_original_obstacle_shapes',true, ...
                'degree',size(u,2)-1, ...
                'provided_uav_error_m',eU, ...
                'provided_payload_error_m',eL, ...
                'rigorous_closed_loop_reachability_verified',false, ...
                'directed_rounding_verified',false, ...
                'continuous_closed_loop_safe_certified',false);
        end
    end
    methods (Static,Access=private)
        function yes=control_hull_separated(B,c,sh,b,r)
            yes=false;
            D=B-c;
            % The AABB of all control points encloses their entire convex
            % hull. This broad phase is valid for arbitrary closed solids.
            low=min(D,[],2);high=max(D,[],2);
            delta=max(max(low,-high),0);
            scale=max([1;abs(B(:));abs(c);b;r]);
            guard=512*eps(scale);
            if norm(delta)>b+r+guard
                yes=true;return
            end
            % For any unit normal n, if min_{control i} n.(Bi-c)
            % exceeds h_O(n)+r, then EVERY convex combination is outside
            % the true obstacle. It works also for concave/mesh solids when
            % the support oracle is an upper support of the occupied set.
            center_normal=mean(D,2);
            axes=[eye(3),-eye(3)];
            normals=[center_normal,axes];
            lengths=sqrt(sum(normals.^2,1));
            valid=lengths>0;
            normals=normals(:,valid)./lengths(valid);
            if isempty(normals),return;end
            if exist('TRIGGER_GEOMETRY_CERTIFICATE','class') == 8
                h=TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many(sh,normals);
            else
                h=b*ones(1,size(normals,2));
            end
            assert(isequal(size(h),[1 size(normals,2)])&&all(isfinite(h)), ...
                'TRIGGER_BERNSTEIN_BODY_SWEEP:Support', ...
                'The original solid has no valid conservative support.');
            minimum=min(normals.'*D,[],2).';
            yes=any(minimum>h+r+guard);
        end
    end
end
