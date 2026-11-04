classdef TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY
    % Shape-agnostic contact screening for the OBSERVED obstacle snapshot.
    %
    % UAV/payload: spheres centered at the predicted EKF-model positions.
    % Cable: finite straight segment between those positions; rc is solely
    % the physical cable cross-section, never an invented swing margin.
    %
    % Two optional *exclusion* stages precede the sensor's exact solid
    % distance oracle:
    % (1) a guaranteed enclosing sphere, and
    % (2) a support-plane half-space of the true occupied obstacle's convex
    %     hull. Neither enclosure is used to DECLARE a contact.
    % If a new sensor geometry does not have specialized support code,
    % its enclosing sphere is still a valid, shape-agnostic support bound.
    % The shared sensor geometry distance oracle is always the narrow phase.
    % An unresolved near/tangent cable query is UNKNOWN, not free space.
    %
    % All geometry here is at sampled times. This class NEVER makes any
    % continuous-time or tracking-error safety claim.
    methods (Static)
        function [u_hit,l_hit,c_hit,c_unknown,info] = evaluate(u,l,shape,ru,rl,rc,t,first_hit_only,runtime_options)
            if nargin<8,first_hit_only=false;end
            assert(isnumeric(u)&&isreal(u)&&size(u,1)==3 && ...
                isequal(size(u),size(l)) && all(isfinite(u(:))) && ...
                all(isfinite(l(:))) && isstruct(shape) && isscalar(shape) && ...
                isfield(shape,'bounding_radius_m') && ...
                isfield(shape,'is_stationary') && shape.is_stationary && ...
                isscalar(ru)&&isnumeric(ru)&&isfinite(ru)&&ru>=0 && ...
                isscalar(rl)&&isnumeric(rl)&&isfinite(rl)&&rl>=0 && ...
                isscalar(rc)&&isnumeric(rc)&&isfinite(rc)&&rc>=0 && ...
                isscalar(t)&&isnumeric(t)&&isfinite(t) && ...
                isscalar(first_hit_only) && (islogical(first_hit_only) || ...
                (isnumeric(first_hit_only) && any(first_hit_only==[0 1]))), ...
                'TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY:Contract', ...
                'Finite 3xN samples, prepared stationary obstacle and nonnegative physical radii required.');
            u=double(u);l=double(l);
            n=size(u,2);
            u_hit=false(1,n);l_hit=false(1,n);
            c_hit=false(1,n);c_unknown=false(1,n);
            info=struct('body_exact_queries',0,'cable_exact_queries',0, ...
                'body_support_excluded',0,'cable_support_excluded',0, ...
                'broadphase_excluded',0,'sampled_axis_plane_excluded',false, ...
                'sampled_block_support_excluded',0, ...
                'sampled_block_support_probes',0, ...
                'sampled_block_ball_excluded',0, ...
                'first_hit_only',logical(first_hit_only), ...
                'sampled_only',true);
            % Real Flight supplies the two IMMUTABLE per-Flight diagnostic
            % switches from its already constructed trigger instance.  The
            % old 8-argument interface remains unchanged for standalone
            % tests and synthetic callers that deliberately switch getenv.
            if nargin>=9
                assert(isstruct(runtime_options)&&isscalar(runtime_options) && ...
                    isfield(runtime_options,'diagnostic_timing_enabled') && ...
                    isfield(runtime_options,'sampled_block_experiment_enabled') && ...
                    islogical(runtime_options.diagnostic_timing_enabled) && ...
                    isscalar(runtime_options.diagnostic_timing_enabled) && ...
                    islogical(runtime_options.sampled_block_experiment_enabled) && ...
                    isscalar(runtime_options.sampled_block_experiment_enabled), ...
                    'TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY:RuntimeOptions', ...
                    'Flight options must be scalar logical snapshot fields.');
                diagnose=runtime_options.diagnostic_timing_enabled;
                block_experiment=runtime_options.sampled_block_experiment_enabled;
            else
                diagnose=strcmp(getenv('REPLANNING_STAGE_TIMING'),'1');
                block_experiment=strcmp(getenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT'),'1');
            end
            if diagnose,diagnostic_geom_clock=tic;end
            c=double(shape.center0(:));
            radius=double(shape.bounding_radius_m);
            assert(isfinite(radius)&&radius>=0&&numel(c)==3&&all(isfinite(c)), ...
                'TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY:ShapeBound', ...
                'A valid enclosing sphere must be supplied by the sensor geometry oracle.');
            % Rounding slack is for engineering fail-closed screening; this
            % double-precision implementation is NOT a formally outward-
            % rounded interval arithmetic certificate.
            slack=128*eps(max([1;radius;abs(c)]));
            % ONE global broad-phase for arbitrarily many path samples.
            % The axis-aligned box contains all sampled ball centers and
            % all straight cables between them, but does NOT enclose the
            % unknown true motion between timestamps.
            lower=min([u,l],[],2); upper=max([u,l],[],2);
            out=max(max(lower-c,c-upper),0);
            if dot(out,out)>(radius+max([ru,rl,rc])+slack)^2
                info.broadphase_excluded=3*n;
                if diagnose
                    info.diagnostic_stage_ms=[1000*toc(diagnostic_geom_clock),0,0];
                end
                return
            end
            % An elongated or arbitrarily rotated solid can have a large
            % bounding ball even when a SINGLE coordinate supporting plane
            % separates its true shape from every sampled UAV/cable/load
            % point. This static-shape, real-observation-only exclusion
            % avoids many repeated support/distance queries WITHOUT ever
            % creating a sampled contact or continuous-time SAFE outcome.
            % Optional sample-only support-plane prefilter. When the
            % prepared obstacle's reference center lies inside the
            % sampled endpoint AABB, probing six support normals often
            % costs more than it saves (notably for trajectories through
            % a nonconvex hole). Skip this OPTIONAL EXCLUSION attempt;
            % the original broadphase/fused support/exact oracle still
            % checks ALL candidates. This condition is purely geometric,
            % shape-type independent, and never declares clearance.
            % For shapes with an offset reference center this may miss a
            % useful optimization, but cannot create a false exclusion.
            if any(c<lower | c>upper)
                [axis_excluded,~]=TRIGGER_SAMPLED_AXIS_SUPPORT_GATE( ...
                    shape,lower,upper,ru,rl,rc);
                if axis_excluded
                    info.broadphase_excluded=3*n;
                    info.sampled_axis_plane_excluded=true;
                    if diagnose
                        info.diagnostic_stage_ms=[1000*toc(diagnostic_geom_clock),0,0];
                    end
                    return
                end
            end
            du=u-c;dl=l-c;
            cu=sum(du.^2,1)<=(radius+ru+slack)^2;
            cl=sum(dl.^2,1)<=(radius+rl+slack)^2;
            segment=l-u;
            seg2=sum(segment.^2,1);
            a=zeros(1,n);
            active=seg2>0;
            a(active)=max(0,min(1,sum((-du(:,active)).*segment(:,active),1)./seg2(active)));
            nearest=du+segment.*a;
            cc=sum(nearest.^2,1)<=(radius+rc+slack)^2;
            info.broadphase_excluded=3*n-nnz(cu)-nnz(cl)-nnz(cc);
            % Shape-independent, per-time-block separating witnesses.
            % Never promotes nominal sampled exclusion to continuous SAFE.
            % The paired MATLAB A/B audit of the former unconditionally
            % enabled 16-point block witness measured a REGRESSION
            % (baseline 0.66755 ms, block 0.80240 ms for its test path).
            % Do not impose that extra allocation/support stage on every
            % realtime tick. Preserve exact-time/full-geometry semantics.
            % Explicit offline experiments can still exercise the old
            % block witness; default Flight uses the established fused
            % original-shape support filter and original distance oracle.
            if block_experiment
                [cu,cl,cc,block]=TRIGGER_SAMPLED_BLOCK_SUPPORT_GATE( ...
                    u,l,shape,ru,rl,rc,cu,cl,cc);
                info.sampled_block_support_excluded= ...
                    block.body_excluded+block.cable_excluded;
                info.sampled_block_ball_excluded=block.blocks_ball_excluded;
                info.sampled_block_support_probes=block.support_probes;
            end

            % Evaluate both body-center candidates against the SAME
            % obstacle support oracle in one batch. This preserves each
            % original direction, radius, and strict exclusion inequality.
            % Only calculation grouping changes, not contact semantics.
            body_flags=[cu,cl];
            body_rel=[du,dl];
            body_radii=[repmat(ru,1,n),repmat(rl,1,n)];
            if diagnose,diagnostic_broad_ms=1000*toc(diagnostic_geom_clock);end
            % ONE common support dispatch for both body-center normals
            % and the full finite cable's nearest-direction normals.
            % Uses the SAME original radial support and strict inequality.
            % The sensor's true-shape distance oracle remains untouched.
            [body_flags,cc,pruned_body,pruned_c]= ...
                TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.support_gates_fused( ...
                body_rel,body_flags,body_radii,du,dl,nearest,cc, ...
                shape,rc,slack);
            cu=body_flags(1:n);
            cl=body_flags(n+1:end);
            info.body_support_excluded=pruned_body;
            info.cable_support_excluded=pruned_c;

            if diagnose,diagnostic_support_end_ms=1000*toc(diagnostic_geom_clock);end
            d_u=nan(1,n);d_l=nan(1,n);
            first=n+1;
            % Query each component chronologically (ascending samples).
            % If first_hit_only is true, later samples of the SAME obstacle
            % do not affect earliest sampled contact. All samples before it
            % are still checked for all three components.
            ui=find(cu);
            for z=1:numel(ui)
                k=ui(z);
                if first_hit_only&&k>=first,break;end
                info.body_exact_queries=info.body_exact_queries+1;
                d_u(k)=OBSTACLE_GEOMETRY_QUERY.distance(u(:,k),shape,t,c);
                if d_u(k)<=ru
                    u_hit(k)=true;
                    first=min(first,k);
                    if ~first_hit_only,continue;end
                    break
                end
            end
            li=find(cl);
            for z=1:numel(li)
                k=li(z);
                if first_hit_only&&k>=first,break;end
                info.body_exact_queries=info.body_exact_queries+1;
                d_l(k)=OBSTACLE_GEOMETRY_QUERY.distance(l(:,k),shape,t,c);
                if d_l(k)<=rl
                    l_hit(k)=true;
                    first=min(first,k);
                    if first_hit_only,break;end
                end
            end
            ci=find(cc);
            for z=1:numel(ci)
                k=ci(z);
                if first_hit_only&&k>=first,break;end
                info.cable_exact_queries=info.cable_exact_queries+1;
                [hit,unknown]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.segment_query( ...
                    u(:,k),l(:,k),shape,t,c,rc,d_u(k),d_l(k));
                c_hit(k)=hit;c_unknown(k)=unknown;
                if hit
                    first=min(first,k);
                    if first_hit_only,break;end
                end
            end
            if diagnose
                diagnostic_exact_end_ms=1000*toc(diagnostic_geom_clock);
                info.diagnostic_stage_ms=[diagnostic_broad_ms, ...
                    diagnostic_support_end_ms-diagnostic_broad_ms, ...
                    diagnostic_exact_end_ms-diagnostic_support_end_ms];
            end
        end
    end
    methods (Static,Access=private)
        function [body,cable,excluded_body,excluded_cable]= ...
                support_gates_fused(body_rel,body,body_radii, ...
                du,dl,nearest,cable,shape,rc,slack)
            % Fuses the two source-support calls, not the geometry logic.
            % Body and cable use EXACTLY their previous directions and
            % strict inequalities. No heuristic inflation or changed radii.
            ib=find(body);
            ic=find(cable);
            excluded_body=0;excluded_cable=0;
            nb=zeros(3,0);nc=zeros(3,0);
            if ~isempty(ib)
                xb=body_rel(:,ib);
                rb=sqrt(sum(xb.^2,1));
                nonzero_b=rb>0;
                ib=ib(nonzero_b);
                if ~isempty(ib)
                    nb=xb(:,nonzero_b)./rb(nonzero_b);
                    rb=rb(nonzero_b);
                end
            end
            if ~isempty(ic)
                xc=nearest(:,ic);
                rcandidate=sqrt(sum(xc.^2,1));
                nonzero_c=rcandidate>0;
                ic=ic(nonzero_c);
                if ~isempty(ic)
                    nc=xc(:,nonzero_c)./rcandidate(nonzero_c);
                end
            end
            count_b=numel(ib);
            count_c=numel(ic);
            if count_b+count_c==0,return;end
            h=TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many( ...
                shape,[nb nc]);
            if count_b>0
                sep=rb>h(1:count_b)+body_radii(ib)+slack;
                body(ib(sep))=false;
                excluded_body=nnz(sep);
            end
            if count_c>0
                hc=h(count_b+1:end);
                a=sum(nc.*du(:,ic),1);
                b=sum(nc.*dl(:,ic),1);
                sep=min(a,b)>hc+rc+slack;
                cable(ic(sep))=false;
                excluded_cable=nnz(sep);
            end
        end
        function [candidates,excluded]=point_gate(d,candidates,shape,r,slack)
            idx=find(candidates);
            excluded=0;
            if isempty(idx),return;end
            x=d(:,idx);
            magnitude=sqrt(sum(x.^2,1));
            good=magnitude>0;
            if ~any(good),return;end
            n=x(:,good)./magnitude(good);
            h=TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many(shape,n);
            % Separating plane: n.p > h_O(n)+r is a sufficient guarantee
            % of no nominal contact. A false result means NOT PROVED.
            % Here r is the per-candidate material radius, so this is
            % elementwise the SAME inequality as the legacy UAV/payload
            % calls using scalar ru and rl respectively.
            safe=magnitude(good)>h+r(idx(good))+slack;
            removed=idx(good);
            removed=removed(safe);
            candidates(removed)=false;
            excluded=numel(removed);
        end
        function [candidates,excluded]=cable_gate(du,dl,nearest,candidates,shape,r,slack)
            idx=find(candidates);
            excluded=0;
            if isempty(idx),return;end
            x=nearest(:,idx);
            magnitude=sqrt(sum(x.^2,1));
            good=magnitude>0;
            if ~any(good),return;end
            n=x(:,good)./magnitude(good);
            selected=idx(good);
            h=TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many(shape,n);
            a=sum(n.*du(:,selected),1);
            b=sum(n.*dl(:,selected),1);
            safe=min(a,b)>h+r+slack;
            candidates(selected(safe))=false;
            excluded=nnz(safe);
        end
        function [hit,unknown]=segment_query(a,b,shape,t,center,r,d0,d1)
            % For any occupied closed set O, d(p,O) is 1-Lipschitz.
            % Along a subsegment of length L with midpoint m:
            % min_{segment} d(p,O) >= d(m,O)-L/2.
            % Positive lower bound > r proves clearance for the ENTIRE
            % spatial cable segment, with no cable-sphere approximation.
            hit=false;unknown=false;
            delta=b-a;length_total=norm(delta);
            if ~isfinite(d0)
                d0=OBSTACLE_GEOMETRY_QUERY.distance(a,shape,t,center);
            end
            if d0<=r,hit=true;return;end
            if length_total==0,return;end
            if ~isfinite(d1)
                d1=OBSTACLE_GEOMETRY_QUERY.distance(b,shape,t,center);
            end
            if d1<=r,hit=true;return;end
            dm=OBSTACLE_GEOMETRY_QUERY.distance((a+b)*0.5,shape,t,center);
            if dm<=r,hit=true;return;end
            % Max subdivision depth is limited by double representation,
            % not a computational budget. No forced clear/false on timeout.
            lo=zeros(1,64);hi=zeros(1,64);
            dlo=zeros(1,64);dhi=zeros(1,64);dmid=zeros(1,64);
            lo(1)=0;hi(1)=1;dlo(1)=d0;dhi(1)=d1;dmid(1)=dm;
            top=1;
            while top>0
                x0=lo(top);x1=hi(top);
                f0=dlo(top);f1=dhi(top);fm=dmid(top);
                top=top-1;
                L=(x1-x0)*length_total;
                lower=max(fm-L/2,(f0+f1-L)/2);
                if lower>r,continue;end
                xm=(x0+x1)*0.5;
                if xm==x0||xm==x1
                    unknown=true;return
                end
                xl=(x0+xm)*0.5;xr=(xm+x1)*0.5;
                fl=OBSTACLE_GEOMETRY_QUERY.distance(a+xl*delta,shape,t,center);
                if fl<=r,hit=true;return;end
                fr=OBSTACLE_GEOMETRY_QUERY.distance(a+xr*delta,shape,t,center);
                if fr<=r,hit=true;return;end
                top=top+1;
                lo(top)=xm;hi(top)=x1;dlo(top)=fm;dhi(top)=f1;dmid(top)=fr;
                top=top+1;
                lo(top)=x0;hi(top)=xm;dlo(top)=f0;dhi(top)=fm;dmid(top)=fl;
            end
        end
    end
end
