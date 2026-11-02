classdef TRIGGER_DIRECTIONAL_INTERVAL_GATE
    % A directional (non-spherical) certificate conditional on externally
    % VERIFIED continuous-time position boxes. It never declares actual
    % closed-loop safety; numerical outward rounding and dynamic reach
    % evidence remain separate required proof obligations.
    %
    % For every time in interval k: U(t) in [Ulo(:,k),Uhi(:,k)],
    % L(t) in [Llo(:,k),Lhi(:,k)] componentwise. For taut straight cable:
    % C(t,s)=(1-s)U(t)+sL(t), 0<=s<=1.
    % Let m(B,n)=min_{b in box B} n.b and h(O,n)=max_{o in O} n.o.
    % If min(m(U,n),m(L,n))>h(O,n)+rCable, that plane separates the
    % ENTIRE straight cable for the ENTIRE interval. Analogous tests
    % with rU/rL apply to the two spherical bodies.
    %
    % Unknown and nonconvex solid geometry is allowed: support of the
    % obstacle's convex hull is conservative for exclusion. No bounding
    % sphere is used to replace cable geometry.
    methods (Static)
        function R=evaluate(Ulo,Uhi,Llo,Lhi,shapes,rU,rL,rCable)
            A={Ulo,Uhi,Llo,Lhi};
            assert(all(cellfun(@(v)isnumeric(v)&&isreal(v)&& ...
                ndims(v)==2&&size(v,1)==3&&all(isfinite(v(:))),A)), ...
                'TRIGGER_DIRECTIONAL_INTERVAL_GATE:Box', ...
                'Finite real 3xK enclosure arrays are required.');
            K=size(Ulo,2);
            assert(K>=1 && all(cellfun(@(v)isequal(size(v),[3 K]),A)) && ...
                all(Ulo(:)<=Uhi(:)) && all(Llo(:)<=Lhi(:)), ...
                'TRIGGER_DIRECTIONAL_INTERVAL_GATE:Order', ...
                'Enclosures must be ordered and share interval columns.');
            assert(isstruct(shapes)&&isvector(shapes)&& ...
                all(isfinite([rU rL rCable]))&&all([rU rL rCable]>=0), ...
                'TRIGGER_DIRECTIONAL_INTERVAL_GATE:Input', ...
                'Prepared shape list and physical nonnegative radii required.');
            N=numel(shapes);
            sepU=false(N,K);sepL=false(N,K);sepC=false(N,K);
            basis=[eye(3),-eye(3)];
            for j=1:N
                sh=shapes(j);
                assert(isfield(sh,'id')&&isfield(sh,'center0')&& ...
                    isfield(sh,'is_stationary'), ...
                    'TRIGGER_DIRECTIONAL_INTERVAL_GATE:Shape', ...
                    'Prepared obstacle shape with ID and motion flag is required.');
                if ~sh.is_stationary
                    continue   % Motion not certified by static support.
                end
                center=double(sh.center0(:));
                assert(isequal(size(center),[3 1])&&all(isfinite(center)), ...
                    'TRIGGER_DIRECTIONAL_INTERVAL_GATE:ShapeCenter', ...
                    'A finite 3D shape center is required.');
                for k=1:K
                    % These axes are candidate separating planes, NOT an
                    % approximation of the cable by its axis-aligned box.
                    % The extra direction uses actual enclosure centers.
                    mid=(Ulo(:,k)+Uhi(:,k)+Llo(:,k)+Lhi(:,k))/4;
                    radial=mid-center;
                    if norm(radial)>0
                        directions=[basis, radial/norm(radial)];
                    else
                        directions=basis;
                    end
                    for h=1:size(directions,2)
                        n=directions(:,h);
                        upperO=dot(n,center)+ ...
                            TRIGGER_GEOMETRY_CERTIFICATE.support_radial(sh,n);
                        minU=sum(n.*( (n>=0).*Ulo(:,k)+(n<0).*Uhi(:,k) ));
                        minL=sum(n.*( (n>=0).*Llo(:,k)+(n<0).*Lhi(:,k) ));
                        % Engineering tolerance ONLY. Without directed
                        % rounding, the result remains a CONDITIONAL claim.
                        scale=max([1,abs(upperO),abs(minU),abs(minL), ...
                            abs(rU),abs(rL),abs(rCable)]);
                        slack=256*eps(scale);
                        if minU>upperO+rU+slack,sepU(j,k)=true;end
                        if minL>upperO+rL+slack,sepL(j,k)=true;end
                        if min(minU,minL)>upperO+rCable+slack
                            sepC(j,k)=true;
                        end
                        if sepU(j,k)&&sepL(j,k)&&sepC(j,k),break;end
                    end
                end
            end
            allsep=sepU&sepL&sepC;
            ids=reshape(double([shapes.id]),1,[]);
            R=struct('status','CONDITIONAL_DIRECTIONAL_BOX_GEOMETRY_ONLY', ...
                'conditionally_clear',all(allsep(:)), ...
                'separated_uav',sepU,'separated_payload',sepL, ...
                'separated_cable',sepC,'separated_intervals',allsep, ...
                'unresolved_obstacle_ids',ids(any(~allsep,2).'), ...
                'interval_count',K,'observed_shape_count',N, ...
                'actual_closed_loop_safe_certified',false, ...
                'requires_dynamics_verified_boxes',true, ...
                'requires_taut_cable_proof',true, ...
                'requires_outward_rounding_audit',true);
        end
    end
end
