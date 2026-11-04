classdef TRIGGER_GEOMETRY_CERTIFICATE
    % Geometry-only conservative separation for the entire suspended body.
    % Works on observed obstacle geometry; no future sensing.
    % Cable is a finite segment from UAV to payload, not a spherical envelope.
    % separated_cable(q,l,0,...) uses the ideal thin line; nonzero radius
    % optionally represents the actual material cross-section (capsule),
    % NOT unmeasured oscillation or an arbitrary robustness margin.
    % A false separation result means UNKNOWN/possibly intersecting, not
    % necessarily an actual collision. It is a conservative support test.
    % The support function is the exact support of the convex hull of the
    % original obstacle. Nonconvex cases are conservative (may be UNKNOWN).
    methods (Static)
        function yes=separated_point(point,radius,shape,center)
            direction=double(point(:))-center;
            scale=norm(direction);
            yes=false;
            if scale==0,return;end
            n=direction/scale;
            radial=TRIGGER_GEOMETRY_CERTIFICATE.support_radial(shape,n);
            yes=scale>radial+radius;
        end
        function yes=separated_cable(q,l,radius,shape,center)
            % Endpoints q/l move with the predicted suspended-load state.
            % For positive radius this tests separation from the capsule;
            % for zero radius it tests separation from the full finite line.
            v=l-q;vv=dot(v,v);
            if vv==0
                yes=TRIGGER_GEOMETRY_CERTIFICATE.separated_point(q,radius,shape,center);
                return
            end
            alpha=max(0,min(1,dot(center-q,v)/vv));
            nearest=q+alpha*v;
            n=nearest-center;
            scale=norm(n);yes=false;
            if scale==0,return;end
            n=n/scale;
            radial=TRIGGER_GEOMETRY_CERTIFICATE.support_radial(shape,n);
            yes=min(dot(n,q-center),dot(n,l-center))>radial+radius;
        end
        function r=support_radial_many(shape,n)
            % Vectorized exact support of the convex hull of an occupied
            % solid. For a NEW sensor shape lacking this optimization, use
            % its verified bounding radius instead (no false clearance).
            % This is a separation-only accelerator; not collision geometry.
            assert(isnumeric(n) && size(n,1)==3 && isreal(n) && ...
                all(isfinite(n(:))), ...
                'TRIGGER_GEOMETRY_CERTIFICATE:Normals','Finite 3xN normals required.');
            k=size(n,2);
            if k==0,r=zeros(1,0);return;end
            % Optional native interval-upper support.  It NEVER declares
            % contact and NEVER rounds an exclusion radius down.  A failure
            % reverts to the identical original MATLAB implementation.
            % The switch is frozen for the MATLAB class session; Flight
            % must not change modes within its three repetitions.
            persistent native_policy
            if isempty(native_policy)
                native_policy=strcmp(getenv('TRIGGER_NATIVE_SUPPORT_BATCH'),'1') && ...
                    exist('TRIGGER_SUPPORT_UPPER_NATIVE','file')==3;
            end
            if native_policy
                try
                    r=TRIGGER_SUPPORT_UPPER_NATIVE(shape,double(n));
                    assert(isequal(size(r),[1 k]) && all(isfinite(r)));
                    return
                catch
                    % Never use incomplete native results. Original exact
                    % shape-support dispatch below is the safe fallback.
                end
            end
            q=double(shape.rotation)'*double(n);
            d=shape.dimensions;
            switch char(shape.type)
                case 'sphere'
                    r=double(d(1))*sqrt(sum(q.^2,1));
                case 'ellipsoid'
                    r=sqrt(sum((double(d(:)).*q).^2,1));
                case 'box'
                    r=sum(abs(q).*double(d(:))/2,1);
                case 'cylinder'
                    r=double(d(1))*hypot(q(1,:),q(2,:))+ ...
                        double(d(2))/2*abs(q(3,:));
                case 'capsule'
                    r=double(d(1))*sqrt(sum(q.^2,1))+ ...
                        double(d(2))/2*abs(q(3,:));
                case 'cone'
                    h=double(d(2));rad=double(d(1))*hypot(q(1,:),q(2,:));
                    r=max(rad-h/2*q(3,:),h/2*q(3,:));
                case 'frustum'
                    h=double(d(3));rho=hypot(q(1,:),q(2,:));
                    r=max(double(d(1))*rho-h/2*q(3,:), ...
                          double(d(2))*rho+h/2*q(3,:));
                case 'pyramid'
                    r=max(double(shape.vertices)*q,[],1);
                case 'regular_prism'
                    r=max(double(shape.outline)*q(1:2,:),[],1)+ ...
                        double(d(3))/2*abs(q(3,:));
                case 'polygon_prism'
                    r=max(double(shape.outline)*q(1:2,:),[],1)+ ...
                        double(d.height)/2*abs(q(3,:));
                case 'torus'
                    r=double(d(1))*hypot(q(1,:),q(2,:))+ ...
                        double(d(2))*sqrt(sum(q.^2,1));
                case 'polyhedron'
                    r=max(double(shape.vertices)*q,[],1);
                otherwise
                    % No shape-specific behavior is required in the trigger
                    % for a newly added sensor geometry type. Bounding
                    % sphere support is conservative for any closed solid.
                    r=double(shape.bounding_radius_m)*sqrt(sum(q.^2,1));
            end
            assert(isequal(size(r),[1 k])&&all(isfinite(r)), ...
                'TRIGGER_GEOMETRY_CERTIFICATE:Support', ...
                'Invalid enclosing support bound.');
        end
        function r=support_radial(shape,n)
            % Exact support of the convex hull in direction n. For nonconvex
            % shapes it may be conservative but NEVER under-approximates.
            b=shape.rotation'*n;
            d=shape.dimensions;
            switch char(shape.type)
                case 'sphere'
                    r=double(d(1));
                case 'ellipsoid'
                    r=norm(double(d(:)).*b);
                case 'box'
                    r=sum(abs(b).*double(d(:))/2);
                case 'cylinder'
                    r=double(d(1))*hypot(b(1),b(2)) + double(d(2))/2*abs(b(3));
                case 'capsule'
                    r=double(d(1))+double(d(2))/2*abs(b(3));
                case 'cone'
                    h=double(d(2));rr=double(d(1))*hypot(b(1),b(2));
                    r=max(rr-h/2*b(3),h/2*b(3));
                case 'frustum'
                    h=double(d(3));rho=hypot(b(1),b(2));
                    r=max(double(d(1))*rho-h/2*b(3),double(d(2))*rho+h/2*b(3));
                case 'pyramid'
                    r=max(double(shape.vertices)*b);
                case 'regular_prism'
                    r=max(double(shape.outline)*b(1:2))+double(d(3))/2*abs(b(3));
                case 'polygon_prism'
                    r=max(double(shape.outline)*b(1:2))+double(d.height)/2*abs(b(3));
                case 'torus'
                    r=double(d(1))*hypot(b(1),b(2))+double(d(2));
                case 'polyhedron'
                    r=max(double(shape.vertices)*b);
                otherwise
                    % Future sensor-defined raw shapes can still be excluded
                    % conservatively by the sensor's enclosing radius.
                    % It never invents a new contact geometry.
                    r=double(shape.bounding_radius_m)*norm(n);
            end
        end
        function yes=cable_centerline_hit(q,l,shape)
            % Exact centerline intersection for a sphere/rotated ellipsoid.
            % Uses all s in [0,1], not endpoint samples and not a cable sphere.
            % Other shapes return false ('not established'), NEVER clearance.
            % This test does not account for positive material cable radius.
            yes=false;
            type=char(shape.type);
            if ~(strcmp(type,'sphere')||strcmp(type,'ellipsoid')),return;end
            if strcmp(type,'sphere')
                a=double(shape.dimensions(1))*ones(3,1);
            else
                a=double(shape.dimensions(:));
            end
            v=shape.rotation'*(q-shape.center0)./a;
            w=shape.rotation'*(l-q)./a;
            den=dot(w,w);
            if den==0
                yes=dot(v,v)<=1;return
            end
            s=max(0,min(1,-dot(v,w)/den));
            z=v+s*w;
            yes=dot(z,z)<=1;
        end
    end
end
