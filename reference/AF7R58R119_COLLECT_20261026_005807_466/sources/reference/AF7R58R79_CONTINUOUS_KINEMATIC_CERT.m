function K=AF7R58R79_CONTINUOUS_KINEMATIC_CERT(C,gravity_m_s2,cable_length_m,vq_finite_sensing_limit_m_s)
% Conservative native-style Bernstein upper bound for taut-cable kinematics.
% NOT a thrust/torque/tension certificate. All values are live external authority.
assert(isstruct(C)&&isfield(C,'payload_cp')&& ...
    isequal(size(C.payload_cp),[3 16]),'[R79] invalid Bezier path.');
X=[gravity_m_s2,cable_length_m,vq_finite_sensing_limit_m_s];
assert(all(isfinite(X))&&all(X>0), ...
    '[R79] gravity, cable length and finite-sensing bound required.');
P=double(C.payload_cp);T=C.duration_s;
assert(isfinite(T)&&T>0,'[R79] positive duration required.');
U=zeros(1,3);
for r=1:3
    P=(size(P,2)-1)/T*diff(P,1,2);
    U(r)=max(vecnorm(P,2,1));
end
sigma=gravity_m_s2-U(2);
if sigma>0
    qdotBound=U(3)/sigma;
    uavBound=U(1)+cable_length_m*qdotBound;
else
    qdotBound=Inf;uavBound=Inf;
end
pass=isfinite(uavBound)&&uavBound<vq_finite_sensing_limit_m_s;
if pass
    status='CERTIFIED_CONSERVATIVE_REFERENCE_KINEMATICS';
else
    status='NOT_CERTIFIED_REFERENCE_KINEMATICS';
end
K=struct('certified',pass,'status',status, ...
    'velocity_upper_m_s',U(1),'accel_upper_m_s2',U(2), ...
    'jerk_upper_m_s3',U(3),'specific_force_lower_m_s2',sigma, ...
    'cable_qdot_upper_rad_s',qdotBound, ...
    'uav_speed_upper_m_s',uavBound, ...
    'finite_sensing_limit_m_s',vq_finite_sensing_limit_m_s, ...
    'continuous_time',pass, ...
    'limitations','NO actuator thrust/torque, tension, estimator tube or controller feasibility proof');
end
