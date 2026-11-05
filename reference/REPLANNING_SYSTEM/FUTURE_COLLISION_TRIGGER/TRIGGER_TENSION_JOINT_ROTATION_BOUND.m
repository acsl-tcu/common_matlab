function J=TRIGGER_TENSION_JOINT_ROTATION_BOUND(S,Z)
% All-time source-derived thrust-axis / taut-cable dot-product bound.
%
% The ORIGINAL plant has pTdot = omegaL x pT and body attitude dynamics
% Rdot = R*[omega]_x (in original Euler chart). With the original exact
% taut manifold |pT|=1 and |R*e3|=1, the body thrust axis b=R*e3 obeys
% |b_dot| <= |omega| and |pT_dot| <= |omegaL|. Consequently
%   |pT(t).b(t) - pT(0).b(0)| <= h (OmegaQ + OmegaL).
% OmegaQ/OmegaL are computed from the VERIFIED all-time 24D angular tube.
% S is the actual starting interval box (NOT an EKF point reset).
% No sampled-time test, future ground truth, or unproved speed cap is used.
% The output is an additional sound SUFFICIENT intersection bound for the
% source's taut ODE only; hybrid-mode correctness is a separate obligation.
J=struct('valid',false,'reason','', ...
    'dot_initial_lower',NaN,'dot_initial_upper',NaN, ...
    'dot_lower',NaN,'dot_upper',NaN, ...
    'omega_body_upper_rad_s',NaN,'omega_cable_upper_rad_s',NaN, ...
    'maximum_dot_drift',NaN, ...
    'safe_to_commit',false);
try
    assert(isstruct(S)&&isscalar(S)&& ...
        all(isfield(S,{'x0_lower','x0_upper','h_upper_s'})) && ...
        isstruct(Z)&&isscalar(Z)&&all(isfield(Z,{'lower','upper'})), ...
        'TRIGGER_TENSION_JOINT_ROTATION_BOUND:Structure');
    lo=double(S.x0_lower(:)); hi=double(S.x0_upper(:));
    zl=double(Z.lower(:)); zh=double(Z.upper(:));
    h=double(S.h_upper_s);
    assert(isequal(size(lo),[24 1])&&isequal(size(hi),[24 1]) && ...
        isequal(size(zl),[24 1])&&isequal(size(zh),[24 1]) && ...
        all(isfinite([lo;hi;zl;zh])) && all(lo<=hi) && ...
        all(zl<=zh) && all(zl<=lo) && all(zh>=hi) && ...
        isscalar(h)&&isfinite(h)&&h>0, ...
        'TRIGGER_TENSION_JOINT_ROTATION_BOUND:Tube', ...
        'A source-verified all-time tube containing the starting box and a positive time upper bound are required.');
    c=@(a,b) TRIGGER_OUTWARD_INTERVAL(a,b);
    roll=c(lo(4),hi(4));pitch=c(lo(5),hi(5));yaw=c(lo(6),hi(6));
    d1=c(lo(19),hi(19));d2=c(lo(20),hi(20));d3=c(lo(21),hi(21));
    sr=sin(roll);cr=cos(roll);sp=sin(pitch);cp=cos(pitch);
    sy=sin(yaw);cy=cos(yaw);
    % Exactly matches the Euler-source D = pT1*t67-pT2*t68+pT3*t77.
    b1=sp.*cr.*cy+sr.*sy;
    b2=sp.*cr.*sy-sr.*cy;
    b3=cp.*cr;
    D0=d1.*b1+d2.*b2+d3.*b3;
    omegaQ=angular_speed_upper(zl(10:12),zh(10:12));
    omegaL=angular_speed_upper(zl(22:24),zh(22:24));
    drift=c(0,h).*(c(omegaQ,omegaQ)+c(omegaL,omegaL));
    D=D0+c(-drift.hi,drift.hi);
    low=max(-1,D.lo);high=min(1,D.hi);
    assert(low<=high&&all(isfinite([low high])) && ...
        all(isfinite([omegaQ omegaL drift.hi])), ...
        'TRIGGER_TENSION_JOINT_ROTATION_BOUND:Incompatible', ...
        'Enclosure conflicts with the unit thrust/cable direction invariant.');
    J.dot_initial_lower=D0.lo;J.dot_initial_upper=D0.hi;
    J.dot_lower=low;J.dot_upper=high;
    J.omega_body_upper_rad_s=omegaQ;
    J.omega_cable_upper_rad_s=omegaL;
    J.maximum_dot_drift=drift.hi;
    J.valid=true;
    J.reason=['Source exact pT and body-axis kinematics, taut-unit constraints, ' ...
        'starting 24D box and verified all-time angular-rate box.'];
catch ME
    J.valid=false;J.reason=[ME.identifier ' | ' ME.message];
end
end
function upper=angular_speed_upper(lo,hi)
% Sum-of-squares is nonnegative exactly; each square uses outward
% interval multiplication rather than a rounding-assumed norm().
I=TRIGGER_OUTWARD_INTERVAL(0,0);
for k=1:3
    mag=max(abs(lo(k)),abs(hi(k)));
    b=TRIGGER_OUTWARD_INTERVAL(-mag,mag);
    I=I+b.^2;
end
% The IEEE binary64 sqrt is correctly rounded on supported MATLAB CPUs;
% outward one-ULP ensures the exact upper endpoint is included.
assert(isfinite(I.hi)&&I.hi>=0, ...
    'TRIGGER_TENSION_JOINT_ROTATION_BOUND:OmegaOverflow');
upper=TRIGGER_FLOAT_OUTWARD_ONE_ULP(sqrt(I.hi),+1);
assert(isfinite(upper)&&upper>=0, ...
    'TRIGGER_TENSION_JOINT_ROTATION_BOUND:OmegaNonfinite');
end
