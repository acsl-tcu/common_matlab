function xd = wp_pack_reference(P, sd, yaw)
%WP_PACK_REFERENCE  Compose P(s(t)) via Bell polynomials through order 6.
% P: 3x7 spatial derivatives wrt s, sd: 7x1 temporal derivatives of s.
% xd = [x;y;z;yaw; dx;dy;dz;dyaw; ... ; d6x;d6y;d6z;d6yaw]
if ~isequal(size(P),[3,7]) || ~isequal(size(sd),[7,1])
    error('WaypointRef:DerivativeDimensions','Expected P=3x7 and sd=7x1.');
end
out = zeros(3,7);
out(:,1) = P(:,1);
B = zeros(7,7);
B(1,1) = 1;
for n = 1:6
    for k = 1:n
        for j = 1:(n-k+1)
            B(n+1,k+1) = B(n+1,k+1) + ...
                nchoosek(n-1,j-1)*sd(j+1)*B(n-j+1,k);
        end
        out(:,n+1) = out(:,n+1) + P(:,k+1)*B(n+1,k+1);
    end
end
xd = zeros(28,1);
for n = 0:6
    xd(4*n+(1:3)) = out(:,n+1);
end
xd(4) = yaw;
end
