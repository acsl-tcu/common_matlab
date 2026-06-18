function z = build_phi(p, eul, v, om, Re3)
% BUILD_PHI  26-dim residual dictionary  phi(x)  (thesis eq. 3.17)
% Inputs (column vectors):
%   p   3x1 position
%   eul 3x1 Euler angles [phi(roll); theta(pitch); psi(yaw)]
%   v   3x1 velocity
%   om  3x1 body angular rate [w1;w2;w3]
%   Re3 3x1 third column of R  (thrust direction in inertial frame)
% Output: z 26x1
%
% NOTE: order MUST match the thesis so A_r,B_r stay consistent:
%   [ x(12); Re3(3); 1; w1w2; w2w3; w3w1;
%     w2 cosφ; w3 sinφ; w1 cosθ/cosφ; w2 sinφ/cosφ; w3 cosφ/cosθ;
%     w2 sinφ sinθ/cosφ; w3 cosφ sinθ/cosθ ]
    phi = eul(1); th = eul(2);                 % psi (eul(3)) not used in couplings
    o1 = om(1); o2 = om(2); o3 = om(3);
    x  = [p(:); eul(:); v(:); om(:)];          % 12x1 Euclidean state

    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(th);  sth  = sin(th);
    % small guards against the cosφ/cosθ singularities (normal flight only)
    cphi = sign_nz(cphi)*max(abs(cphi),1e-3);
    cth  = sign_nz(cth) *max(abs(cth), 1e-3);

    z = [ x;
          Re3(:);
          1;
          o1*o2; o2*o3; o3*o1;
          o2*cphi;
          o3*sphi;
          o1*cth/cphi;
          o2*sphi/cphi;
          o3*cphi/cth;
          o2*sphi*sth/cphi;
          o3*cphi*sth/cth ];
end

function s = sign_nz(x)
    s = sign(x); if s==0, s = 1; end
end