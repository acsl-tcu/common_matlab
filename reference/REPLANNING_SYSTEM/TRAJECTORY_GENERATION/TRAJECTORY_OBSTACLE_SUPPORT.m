classdef TRAJECTORY_OBSTACLE_SUPPORT
    % World-frame support of observed STATIC 3-D occupancy convex hulls.
    % Uses canonical 12-shape validator; nonconvex shapes use hull support.
    % h_O(n)=max_{p in O} n'*p. n may be unnormalised.
    % All support values are homogeneous, including spheres/capsules.
    methods (Static)
        function S=prepare(observed)
            assert(isstruct(observed), ...
                'TRAJECTORY_OBSTACLE_SUPPORT:Input', ...
                'Only observed static obstacle structures are accepted.');
            S=repmat(struct('id',0,'kind','','dim',[], ...
                'center',zeros(3,1),'R',eye(3), ...
                'local_vertices',zeros(0,3)),1,numel(observed));
            ids=zeros(1,numel(observed));
            assert(exist('OBSTACLE_SHAPES','class')==8, ...
                'TRAJECTORY_OBSTACLE_SUPPORT:Validator', ...
                'Canonical shape validator must be on MATLAB path.');
            for j=1:numel(observed)
                o=observed(j);
                assert(isfield(o,'id')&&isnumeric(o.id)&&isscalar(o.id)&& ...
                    isfinite(o.id)&&o.id>=1&&o.id==fix(o.id), ...
                    'TRAJECTORY_OBSTACLE_SUPPORT:ID','Invalid observed ID.');
                if isfield(o,'geometry')
                    assert(isstruct(o.geometry)&& ...
                        all(isfield(o.geometry,{'type','dimensions'}))&& ...
                        isfield(o,'position')&&isfield(o,'motion')&& ...
                        strcmp(char(o.motion),'static'), ...
                        'TRAJECTORY_OBSTACLE_SUPPORT:Static', ...
                        'Complete static observed obstacle required.');
                    kind=char(o.geometry.type);
                    dim=o.geometry.dimensions;
                    center=o.position;
                else
                    assert(all(isfield(o, ...
                        {'type','dimensions','center0','is_stationary'})) && ...
                        isequal(o.is_stationary,true), ...
                        'TRAJECTORY_OBSTACLE_SUPPORT:Static', ...
                        'Incomplete or moving observed obstacle.');
                    kind=char(o.type);dim=o.dimensions;center=o.center0;
                end
                assert(isfield(o,'rotation'), ...
                    'TRAJECTORY_OBSTACLE_SUPPORT:Rotation','Rotation required.');
                Q=double(o.rotation);center=double(center(:));
                assert(isequal(size(Q),[3,3])&&all(isfinite(Q(:)))&& ...
                    norm(Q.'*Q-eye(3),'fro')<1e-8&&abs(det(Q)-1)<1e-8&& ...
                    isequal(size(center),[3,1])&&all(isfinite(center)), ...
                    'TRAJECTORY_OBSTACLE_SUPPORT:Pose','Invalid SO(3) pose.');
                OBSTACLE_SHAPES.validate( ...
                    struct('type',kind,'dimensions',dim));
                ids(j)=double(o.id);
                % Convert vertex-defined convex hulls to their immutable
                % local vertices ONCE per observed geometry snapshot. There
                % is no obstacle-ID-specific native code or hidden data.
                V=zeros(0,3);
                switch kind
                    case 'pyramid'
                        a=double(dim(1))/2; zz=double(dim(2))/2;
                        V=[a a -zz;a -a -zz;-a a -zz;-a -a -zz;0 0 zz];
                    case 'regular_prism'
                        ang=2*pi*(0:double(dim(1))-1)/double(dim(1));
                        ring=double(dim(2))*[cos(ang(:)),sin(ang(:))];
                        zz=double(dim(3))/2;
                        V=[ring,-zz*ones(size(ring,1),1); ...
                            ring,zz*ones(size(ring,1),1)];
                    case 'polygon_prism'
                        ring=double(dim.outline);
                        zz=double(dim.height)/2;
                        V=[ring,-zz*ones(size(ring,1),1); ...
                            ring,zz*ones(size(ring,1),1)];
                    case 'polyhedron'
                        V=double(dim.vertices);
                end
                S(j)=struct('id',ids(j),'kind',kind,'dim',dim, ...
                    'center',center,'R',Q,'local_vertices',V);
            end
            assert(numel(unique(ids))==numel(ids), ...
                'TRAJECTORY_OBSTACLE_SUPPORT:ID','Duplicate observed IDs.');
        end
        function h=value(S,n)
            assert(isstruct(S)&&isscalar(S)&&isnumeric(n)&& ...
                isreal(n)&&isequal(size(n),[3 1])&&all(isfinite(n)), ...
                'TRAJECTORY_OBSTACLE_SUPPORT:Direction','Invalid direction.');
            w=S.R.'*n;d=S.dim;
            rho=hypot(w(1),w(2));l=norm(w);
            if ~isempty(S.local_vertices)
                h=max(S.local_vertices*w);
                h=double(S.center.'*n+h);
                assert(isfinite(h)&&isreal(h), ...
                    'TRAJECTORY_OBSTACLE_SUPPORT:Support','Invalid support.');
                return
            end
            switch S.kind
                case 'sphere'
                    h=double(d(1))*l;
                case 'ellipsoid'
                    h=norm(double(d(:)).*w);
                case 'box'
                    h=sum(double(d(:)).*abs(w))/2;
                case 'cylinder'
                    h=double(d(1))*rho+double(d(2))*abs(w(3))/2;
                case 'capsule'
                    h=double(d(1))*l+double(d(2))*abs(w(3))/2;
                case 'cone'
                    h=max(double(d(1))*rho-double(d(2))*w(3)/2, ...
                        double(d(2))*w(3)/2);
                case 'frustum'
                    h=max(double(d(1))*rho-double(d(3))*w(3)/2, ...
                        double(d(2))*rho+double(d(3))*w(3)/2);
                case 'torus'
                    h=double(d(1))*rho+double(d(2))*l;
                case 'pyramid'
                    a=double(d(1))/2;z=double(d(2))/2;
                    V=[a a -z;a -a -z;-a a -z;-a -a -z;0 0 z];
                    h=max(V*w);
                case 'regular_prism'
                    k=0:double(d(1))-1;theta=2*pi*k/double(d(1));
                    h=max(double(d(2))*(cos(theta)*w(1)+ ...
                        sin(theta)*w(2)))+double(d(3))*abs(w(3))/2;
                case 'polygon_prism'
                    h=max(double(d.outline)*w(1:2))+ ...
                        double(d.height)*abs(w(3))/2;
                case 'polyhedron'
                    h=max(double(d.vertices)*w);
                otherwise
                    error('TRAJECTORY_OBSTACLE_SUPPORT:Shape', ...
                        'Unknown shape (fail closed).');
            end
            assert(isfinite(h)&&isreal(h), ...
                'TRAJECTORY_OBSTACLE_SUPPORT:Support','Invalid support.');
            h=double(S.center.'*n+h);
        end
        function [h,p]=value_and_point(S,n)
            % Supporting WORLD point p satisfies h=n'*p for each of the
            % canonical 12 static shapes; valid subgradient of h(n).
            % For nonconvex obstacles this is the support of their hull.
            assert(isstruct(S)&&isscalar(S)&&isnumeric(n)&&isreal(n)&& ...
                isequal(size(n),[3 1])&&all(isfinite(n)), ...
                'TRAJECTORY_OBSTACLE_SUPPORT:Direction','Invalid direction.');
            w=S.R.'*n; d=S.dim;
            rho=hypot(w(1),w(2));mag=norm(w);
            radial=[0;0];
            if rho>0,radial=w(1:2)/rho;end
            local=zeros(3,1);
            if ~isempty(S.local_vertices)
                [~,ii]=max(S.local_vertices*w);
                local=S.local_vertices(ii,:).';
                p=double(S.center)+double(S.R)*local;
                h=double(n.'*p);
                assert(isfinite(h)&&all(isfinite(p)), ...
                    'TRAJECTORY_OBSTACLE_SUPPORT:Support','Invalid support point.');
                return
            end
            switch S.kind
                case 'sphere'
                    if mag>0,local=double(d(1))*w/mag;end
                case 'ellipsoid'
                    a=double(d(:)); q=norm(a.*w);
                    if q>0,local=(a.^2).*w/q;end
                case 'box'
                    local=0.5*double(d(:)).*sign(w);
                case 'cylinder'
                    local=[double(d(1))*radial;double(d(2))/2*sign(w(3))];
                case 'capsule'
                    if mag>0,local=double(d(1))*w/mag;end
                    local(3)=local(3)+double(d(2))/2*sign(w(3));
                case 'cone'
                    h2=double(d(2))/2;
                    if double(d(1))*rho-h2*w(3) >= h2*w(3)
                        local=[double(d(1))*radial;-h2];
                    else
                        local=[0;0;h2];
                    end
                case 'frustum'
                    h2=double(d(3))/2;
                    if double(d(1))*rho-h2*w(3) >= ...
                            double(d(2))*rho+h2*w(3)
                        local=[double(d(1))*radial;-h2];
                    else
                        local=[double(d(2))*radial;h2];
                    end
                case 'torus'
                    if mag>0,local=double(d(2))*w/mag;end
                    local(1:2)=local(1:2)+double(d(1))*radial;
                case 'pyramid'
                    a=double(d(1))/2; q=double(d(2))/2;
                    V=[a a -q;a -a -q;-a a -q;-a -a -q;0 0 q];
                    [~,id]=max(V*w);local=V(id,:).';
                case 'regular_prism'
                    ang=2*pi*(0:double(d(1))-1)/double(d(1));
                    [~,id]=max(cos(ang)*w(1)+sin(ang)*w(2));
                    local=[double(d(2))*cos(ang(id)); ...
                        double(d(2))*sin(ang(id));double(d(3))/2*sign(w(3))];
                case 'polygon_prism'
                    V=double(d.outline);
                    [~,id]=max(V*w(1:2));
                    local=[V(id,:).';double(d.height)/2*sign(w(3))];
                case 'polyhedron'
                    V=double(d.vertices);
                    [~,id]=max(V*w);local=V(id,:).';
                otherwise
                    error('TRAJECTORY_OBSTACLE_SUPPORT:Shape', ...
                        'Unknown obstacle shape (fail closed).');
            end
            p=double(S.center)+double(S.R)*local;
            h=double(n.'*p);
            assert(isfinite(h)&&all(isfinite(p)), ...
                'TRAJECTORY_OBSTACLE_SUPPORT:Support','Invalid support point.');
        end
    end
end
