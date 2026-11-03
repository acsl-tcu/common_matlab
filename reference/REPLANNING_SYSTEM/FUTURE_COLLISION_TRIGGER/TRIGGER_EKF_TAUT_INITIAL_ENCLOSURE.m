function R=TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE(C)
% Construct an interval INITIAL 24-state plant set from one EKF snapshot,
% explicit (externally proved) signed estimator-error bounds and taut
% manifold identities. No assumed tracking tube, payload mass, Flight
% epoch, unseen obstacles or guessed physical speed/acceleration limits.
%
% Physical state order: [pQ, Euler, vQ, wQ, pL, vL, d, wL].
% Here d points from UAV to load and L>0 is the true taut cable length.
% For each real taut state: pL=pQ+L*d and vL=vQ+L*(wL x d).
% Intersecting these derived enclosures with the explicitly supplied EKF
% error bounds is a SAFE contraction IF BOTH INPUT ENCLOSURES are valid.
%
% IMPORTANT: This routine DOES NOT prove the supplied EKF error bounds,
% the unit-sphere constraint, the actual taut mode or directed rounding
% on this MATLAB installation. Consequently it NEVER grants Flight safety.
R=struct('status','UNKNOWN_EKF_TAUT_INITIAL_SET', ...
    'reason','','conditional_enclosure',false,'safe_to_commit',false, ...
    'initial_lower',zeros(24,0),'initial_upper',zeros(24,0), ...
    'reduced_payload_lower',zeros(3,0), ...
    'reduced_payload_upper',zeros(3,0), ...
    'reduced_velocity_lower',zeros(3,0), ...
    'reduced_velocity_upper',zeros(3,0), ...
    'initial_box_contracted',false, ...
    'unit_direction_feasible',false, ...
    'observer_error_certified_by_source',false, ...
    'exact_taut_mode_certified',false, ...
    'rounding_environment_certified',false);
try
    assert(isstruct(C)&&isscalar(C) && ...
        all(isfield(C,{'ekf_now','error_lower','error_upper','cable_length_lower_m','cable_length_upper_m'})), ...
        'TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE:Required', ...
        'EKF, explicit signed errors and externally supplied cable length required.');
    e=C.ekf_now;
    names={'p','q','v','w','pL','vL','pT','wL'};
    assert(isstruct(e)&&isscalar(e)&&all(isfield(e,names)), ...
        'TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE:EKF', ...
        '24-state EKF fields are required in original model order.');
    hat=zeros(24,1);
    for j=1:8
        v=e.(names{j});
        assert(isnumeric(v)&&isreal(v)&&isequal(size(v),[3 1]) && ...
            all(isfinite(v)), ...
            'TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE:EKF', ...
            'Each EKF field must be a finite real 3x1 vector.');
        hat(3*j-2:3*j)=double(v);
    end
    elo=double(C.error_lower);ehi=double(C.error_upper);
    assert(isequal(size(elo),[24 1])&&isequal(size(ehi),[24 1]) && ...
        all(isfinite([elo;ehi]))&&all(elo<=ehi), ...
        'TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE:Errors', ...
        'Require signed bounds TRUE minus EKF estimate in original 24-state coordinates.');
    La=double(C.cable_length_lower_m);
    Lb=double(C.cable_length_upper_m);
    assert(isscalar(La)&&isscalar(Lb)&&isfinite(La)&&isfinite(Lb)&& ...
        0<La&&La<=Lb, ...
        'TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE:Cable', ...
        'Require a positive externally justified taut cable length interval.');
    % All arithmetic routes through the existing outward-interval wrapper.
    % A separate verified arithmetic-backend proof remains mandatory.
    mk=@(a,b)TRIGGER_OUTWARD_INTERVAL(double(a),double(b));
    lo=zeros(24,1);hi=zeros(24,1);
    for k=1:24
        z=mk(hat(k),hat(k))+mk(elo(k),ehi(k));
        lo(k)=z.lo;hi(k)=z.hi;
    end
    % Exact unit-vector geometry gives the componentwise bound [-1,1].
    % This does not itself certify that the unknown true state is taut.
    lo(19:21)=max(lo(19:21),-1);
    hi(19:21)=min(hi(19:21),1);
    if any(lo>hi)
        R.status='UNKNOWN_EKF_DIRECTION_NOT_UNIT_COMPATIBLE';
        R.reason='No unit cable direction is consistent with the reported EKF error.';
        return
    end
    % The original plant uses 1/cos(pitch), and is not defined on a box
    % containing a pitch chart singularity. This is a necessary chart gate.
    lowpitch=lo(5);highpitch=hi(5);
    if ceil((lowpitch-pi/2)/pi)<=floor((highpitch-pi/2)/pi)
        R.status='UNKNOWN_EKF_PITCH_CHART_SINGULAR';
        R.reason='Initial uncertainty reaches an original Euler pitch singularity.';
        return
    end
    di_lo=lo(19:21);di_hi=hi(19:21);
    nearest=max(0,max(di_lo,-di_hi));
    furthest=max(abs(di_lo),abs(di_hi));
    feasible=sum(nearest.^2)<=1 && sum(furthest.^2)>=1;
    R.unit_direction_feasible=feasible;
    if ~feasible
        R.status='UNKNOWN_EKF_DIRECTION_BOX_HAS_NO_UNIT_VECTOR';
        R.reason='Estimated direction bounds are inconsistent with ||d||=1.';
        return
    end
    L=mk(La,Lb);
    derived_p=zeros(3,2);derived_v=zeros(3,2);
    for j=1:3
        jp=mod(j,3)+1;jq=mod(j+1,3)+1;
        pj=mk(lo(j),hi(j));
        vj=mk(lo(6+j),hi(6+j));
        dj=mk(lo(18+j),hi(18+j));
        wk=mk(lo(21+jp),hi(21+jp));
        wh=mk(lo(21+jq),hi(21+jq));
        dh=mk(lo(18+jq),hi(18+jq));
        dk=mk(lo(18+jp),hi(18+jp));
        pos=pj+L.*dj;
        vel=vj+L.*(wk.*dh-wh.*dk);
        derived_p(j,:)=[pos.lo pos.hi];
        derived_v(j,:)=[vel.lo vel.hi];
    end
    R.reduced_payload_lower=derived_p(:,1);
    R.reduced_payload_upper=derived_p(:,2);
    R.reduced_velocity_lower=derived_v(:,1);
    R.reduced_velocity_upper=derived_v(:,2);
    % Correct 24-state index for pL is 13:15; for vL it is 16:18.
    oldlo=lo;oldhi=hi;
    lo(13:15)=max(lo(13:15),derived_p(:,1));
    hi(13:15)=min(hi(13:15),derived_p(:,2));
    lo(16:18)=max(lo(16:18),derived_v(:,1));
    hi(16:18)=min(hi(16:18),derived_v(:,2));
    if any(lo>hi)
        R.status='UNKNOWN_EKF_TAUT_MANIFOLD_INCONSISTENT';
        R.reason='No state can satisfy both reported EKF error bounds and the taut cable identities.';
        return
    end
    R.initial_box_contracted=any(lo>oldlo)|any(hi<oldhi);
    R.initial_lower=lo;
    R.initial_upper=hi;
    R.conditional_enclosure=true;
    R.status='CONDITIONAL_EKF_TAUT_INITIAL_BOX__NOT_EKF_ERROR_PROOF';
    R.reason=['The intersection is an enclosure only if the supplied ' ...
        'signed EKF error bounds and taut-mode identities are independently ' ...
        'valid. Future mass bounds, sampled HLC inputs, hybrid modes and ' ...
        'verified interval rounding are still required.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
