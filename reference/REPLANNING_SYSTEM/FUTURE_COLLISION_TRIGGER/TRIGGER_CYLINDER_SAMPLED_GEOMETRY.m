classdef TRIGGER_CYLINDER_SAMPLED_GEOMETRY
    % Exact-at-samples finite-cylinder geometry for the nominal taut line.
    % Caution: NOT a continuous-time swept volume or closed-loop proof.
    % All coordinates use the true SO(3) obstacle rotation; the cable is a
    % FINITE SEGMENT, not an obstacle-enclosing sphere or a sphere sweep.
    % The UAV/payload are spheres, evaluated against the finite cylinder.
    % Cable material radius must be ZERO for this exact centerline branch.
    methods (Static)
        function [uav_hit, payload_hit, cable_hit] = evaluate(q,l,shape,r_uav,r_payload)
            assert(isnumeric(q)&&isreal(q)&&size(q,1)==3 && ...
                isequal(size(q),size(l)) && all(isfinite(q(:))) && ...
                all(isfinite(l(:))) && strcmp(char(shape.type),'cylinder') && ...
                shape.is_stationary && isscalar(r_uav) && isscalar(r_payload) && ...
                isfinite(r_uav) && r_uav>=0 && ...
                isfinite(r_payload) && r_payload>=0,...
                'TRIGGER_CYLINDER_SAMPLED_GEOMETRY:Contract', ...
                'Finite static cylinder and finite 3xN segment endpoints required.');
            R=double(shape.rotation);
            c=double(shape.center0(:));
            dims=double(shape.dimensions(:));
            assert(isequal(size(R),[3 3]) && isequal(size(c),[3 1]) && ...
                numel(dims)==2 && all(isfinite(dims)) && all(dims>0), ...
                'TRIGGER_CYLINDER_SAMPLED_GEOMETRY:Cylinder','Invalid cylinder geometry.');
            % Unit-convention inherited from OBSTACLE_GEOMETRY_QUERY:
            % dimensions = [physical radius, full axial height].
            a=R'*(double(q)-c);
            b=R'*(double(l)-c);
            rad=dims(1); half_height=dims(2)*0.5;
            % Exact Euclidean distance to a solid, finite circular cylinder.
            dist_u=hypot(max(hypot(a(1,:),a(2,:))-rad,0), ...
                         max(abs(a(3,:))-half_height,0));
            dist_l=hypot(max(hypot(b(1,:),b(2,:))-rad,0), ...
                         max(abs(b(3,:))-half_height,0));
            uav_hit=dist_u<=r_uav;
            payload_hit=dist_l<=r_payload;
            % Intersect [a,b] with the z-slab of the finite cylinder, then
            % minimize radial distance to its axis over the clipped segment.
            % This detects side, top/bottom cap and segment-inside contact.
            d=b-a;
            n=size(a,2);
            s_lo=zeros(1,n);
            s_hi=ones(1,n);
            moving_z=d(3,:)~=0;
            if any(moving_z)
                za=(-half_height-a(3,moving_z))./d(3,moving_z);
                zb=( half_height-a(3,moving_z))./d(3,moving_z);
                s_lo(moving_z)=max(0,min(za,zb));
                s_hi(moving_z)=min(1,max(za,zb));
            end
            clipped_outside=(s_lo>s_hi) | ...
                (~moving_z & abs(a(3,:))>half_height);
            den=d(1,:).^2+d(2,:).^2;
            s=s_lo;
            moving_xy=den>0;
            if any(moving_xy)
                t=-(a(1,moving_xy).*d(1,moving_xy)+ ...
                    a(2,moving_xy).*d(2,moving_xy))./den(moving_xy);
                s(moving_xy)=max(s_lo(moving_xy), ...
                                 min(s_hi(moving_xy),t));
            end
            x=a(1,:)+s.*d(1,:);
            y=a(2,:)+s.*d(2,:);
            cable_hit=~clipped_outside & (x.^2+y.^2<=rad^2);
            % Explicitly 1xN, preserving chronological sample order.
            cable_hit=reshape(cable_hit,1,[]);
            uav_hit=reshape(uav_hit,1,[]);
            payload_hit=reshape(payload_hit,1,[]);
        end
    end
end
