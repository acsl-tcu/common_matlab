function R=TRIGGER_TAUT_SWEEP_SPEED_CERT(D)
% Mathematically justified swept-speed and static-obstacle clearance bound.
% For the TAUT original 24-state dynamics, p_c(s)=p_Q+s*L*d (0<=s<=1),
% dot(d)=omega_L cross d and ||d||=1 imply
%   ||dot(p_c(s))|| <= ||v_Q|| + L ||omega_L||.
% This also bounds the payload centre and hence the Hausdorff speed of
% translated UAV sphere, cable capsule and payload sphere, for CONSTANT
% radii and cable length within each hold. It is NOT a substitute for
% real observer-error, held HLC input, tension or interval roundoff proofs.
%
% Supply verified continuous-time Picard tubes (24 x K), actual arbitrary
% positive hold durations, physical cable length upper and certified lower
% distances from the COMPLETE body/cable/payload solid to a STATIC obstacle
% at each interval start. No invented speed/accel bound or fixed phase time.
R=struct('status','UNKNOWN_TAUT_SWEEP_SPEED', ...
    'conditional_clearance_pass',false,'conditional_swept_bound',false, ...
    'speed_uav_upper_mps',[],'speed_swept_upper_mps',[], ...
    'distance_loss_upper_m',[],'clearance_lower_end_m',[], ...
    'safe_to_commit',false,'reason','');
try
    needed={'tube_lower','tube_upper','hold_duration_s', ...
        'cable_length_upper_m','start_clearance_lower_m'};
    assert(isstruct(D)&&isscalar(D)&&all(isfield(D,needed)), ...
        'TRIGGER_TAUT_SWEEP_SPEED_CERT:Fields', ...
        'All-time original state tube, source-true length and start clearance needed.');
    lo=double(D.tube_lower);hi=double(D.tube_upper);
    h=double(D.hold_duration_s(:).');
    K=numel(h);
    assert(K>=1 && isequal(size(lo),[24 K]) && ...
        isequal(size(hi),[24 K]) && all(isfinite(lo(:))) && ...
        all(isfinite(hi(:))) && all(lo(:)<=hi(:)) && ...
        all(isfinite(h)) && all(h>0), ...
        'TRIGGER_TAUT_SWEEP_SPEED_CERT:Tube', ...
        'Need a valid all-time original Picard box in each hold.');
    L=double(D.cable_length_upper_m);
    d0=double(D.start_clearance_lower_m);
    assert(isnumeric(L)&&isscalar(L)&&isreal(L)&&isfinite(L)&&L>0, ...
        'TRIGGER_TAUT_SWEEP_SPEED_CERT:CableLength', ...
        'A certified positive cable length upper bound is mandatory.');
    assert(isnumeric(d0)&&isreal(d0)&&isequal(size(d0),[1 K]) && ...
        all(isfinite(d0)), ...
        'TRIGGER_TAUT_SWEEP_SPEED_CERT:Clearance', ...
        'The certified start clearance to the static solid must be 1xK.');
    % Zero-order interval upper bound on Euclidean norms, using the exact
    % indices of UAV translational velocity and taut-cable angular rate.
    velmax=max(abs(lo(7:9,:)),abs(hi(7:9,:)));
    angmax=max(abs(lo(22:24,:)),abs(hi(22:24,:)));
    vq=sqrt(sum(velmax.^2,1));
    omega=sqrt(sum(angmax.^2,1));
    % Round UP conservatively. MATLAB floating-point directed-rounding
    % correctness is NOT independently audited: nextafter provides only a
    % local numerical overapproximation, not a certified software proof.
    if exist('nextafter','builtin') || exist('nextafter','file')
        vq=nextafter(vq,inf);
        omega=nextafter(omega,inf);
    end
    vs=vq+L.*omega;
    loss=h.*vs;
    if exist('nextafter','builtin') || exist('nextafter','file')
        vs=nextafter(vs,inf);
        loss=nextafter(h.*vs,inf);
    end
    assert(all(isfinite(loss)), ...
        'TRIGGER_TAUT_SWEEP_SPEED_CERT:Overflow', ...
        'Nonfinite numerical bound cannot be accepted.');
    R.speed_uav_upper_mps=vq;
    R.speed_swept_upper_mps=vs;
    R.distance_loss_upper_m=loss;
    R.clearance_lower_end_m=d0-loss;
    R.conditional_swept_bound=true;
    R.conditional_clearance_pass=all(d0>loss);
    if R.conditional_clearance_pass
        R.status='CONDITIONAL_TAUT_SWEEP_CLEARANCE_PASS__NOT_FLIGHT_PROOF';
    else
        R.status='UNKNOWN_SWEPT_CLEARANCE_NOT_ESTABLISHED';
    end
    R.reason=['Requires independently certified all-time state tubes, ' ...
        'taut-unit direction and unchanged cable length, EXACT static ' ...
        'obstacle start clearance including body radii and rounding. ' ...
        'It cannot grant Flight safety.'];
catch ME
    R.status='UNKNOWN_SWEEP_CERT_INPUT_OR_NUMERICS';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
