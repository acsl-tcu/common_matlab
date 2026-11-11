function h=TRAJECTORY_C9_SUPPORT_BATCH(S,normal_world)
% Vectorized EXACT SAME real-arithmetic support of the canonical observed
% static occupancy hull as TRAJECTORY_OBSTACLE_SUPPORT.value, for 3xM normals.
% Arbitrary SO(3), every canonical 12-shape type, no case-specific detours.
% This function is a QP construction aid, NEVER a collision certificate.
n=double(normal_world);
assert(isstruct(S)&&isscalar(S)&&isequal(size(n,1),3)&& ...
    isreal(n)&&all(isfinite(n(:)))&&all(isfield(S, ...
    {'center','R','kind','dim','local_vertices'})), ...
    'C9_SUPPORT_BATCH:InvalidInput','Expected canonical prepared shape and 3xM normals.');
W=double(S.R).'*n;d=S.dim;
rho=hypot(W(1,:),W(2,:));mag=hypot(W(1,:),hypot(W(2,:),W(3,:)));
if ~isempty(S.local_vertices)
    local=max(double(S.local_vertices)*W,[],1);
else
    switch S.kind
        case 'sphere'
            local=double(d(1))*mag;
        case 'ellipsoid'
            v=double(d(:)).*W;
            local=hypot(v(1,:),hypot(v(2,:),v(3,:)));
        case 'box'
            local=sum(double(d(:)).*abs(W),1)/2;
        case 'cylinder'
            local=double(d(1))*rho+double(d(2))*abs(W(3,:))/2;
        case 'capsule'
            local=double(d(1))*mag+double(d(2))*abs(W(3,:))/2;
        case 'cone'
            local=max(double(d(1))*rho-double(d(2))*W(3,:)/2, ...
                double(d(2))*W(3,:)/2);
        case 'frustum'
            local=max(double(d(1))*rho-double(d(3))*W(3,:)/2, ...
                double(d(2))*rho+double(d(3))*W(3,:)/2);
        case 'torus'
            local=double(d(1))*rho+double(d(2))*mag;
        otherwise
            error('C9_SUPPORT_BATCH:UnknownKind', ...
                'Unsupported canonical kind without precomputed vertices: %s',S.kind);
    end
end
h=double(S.center(:)).'*n+local;
assert(all(isfinite(h))&&isequal(size(h),[1 size(n,2)]), ...
    'C9_SUPPORT_BATCH:InvalidSupport','Nonfinite support; do not use for QP.');
end
