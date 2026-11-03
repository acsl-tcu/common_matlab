function R=TRIGGER_ORIGINAL_PLANT_PICARD_STEP(D)
% Candidate one sample-held interval reach step using the original physical
% 24-state RHS interval extension, NOT a complete closed-loop theorem.
%
% D: x0_lower/upper (24x1), tube_lower/upper (24x1),
%    u_lower/upper (4x1 held during h), param_lower/upper (21x1),
%    h_s (positive). All sets must be sourced and justified externally.
%
% For x'=f(x,u,param), the strict inclusion
% X0 + [0,h] F(Tube,U,Param) subset int(Tube)
% implies an all-time Tube under standard Lipschitz hypotheses.
% xnext is propagated as X0 + h*F(Tube,U,Param).
%
% Unknown estimator-to-plant error, real HLC held-input image, tension,
% hybrid events, mass variation bounds, original source identity audit,
% and compiler floating-point semantics keep safe_to_commit FALSE.
R=struct('status','UNKNOWN_ORIGINAL_PLANT_PICARD', ...
    'conditional_strict_inclusion',false,'margin_lower',nan(24,1), ...
    'margin_upper',nan(24,1), ...
    'rhs',struct(),'endpoint_lower',nan(24,1), ...
    'endpoint_upper',nan(24,1), ...
    'actual_initial_set_verified',false, ...
    'actual_hlc_held_input_set_verified',false, ...
    'ekf_and_hybrid_verified',false, ...
    'mass_tension_actuation_verified',false, ...
    'original_source_equivalence_verified',false, ...
    'safe_to_commit',false,'reason','');
try
    need={'x0_lower','x0_upper','tube_lower','tube_upper', ...
        'u_lower','u_upper','param_lower','param_upper','h_s'};
    assert(isstruct(D)&&isscalar(D)&&all(isfield(D,need)), ...
        'TRIGGER_ORIGINAL_PLANT_PICARD_STEP:Fields', ...
        'Complete conditional input boxes required.');
    h=double(D.h_s);
    assert(isnumeric(h)&&isscalar(h)&&isreal(h)&&isfinite(h)&&h>0,...
        'TRIGGER_ORIGINAL_PLANT_PICARD_STEP:Step', ...
        'Positive held-input duration required.');
    X=struct('lower',D.x0_lower,'upper',D.x0_upper);
    Z=struct('lower',D.tube_lower,'upper',D.tube_upper);
    U=struct('lower',D.u_lower,'upper',D.u_upper);
    P=struct('lower',D.param_lower,'upper',D.param_upper);
    rhs=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS(Z,U,P);
    R.rhs=rhs;
    if ~rhs.outward_elementary_operations
        R.reason=rhs.reason;
        return
    end
    for j=1:24
        xlo=X.lower(j);xhi=X.upper(j);
        assert(isfinite(xlo)&&isfinite(xhi)&&xlo<=xhi && ...
            isfinite(Z.lower(j))&&isfinite(Z.upper(j))&& ...
            Z.lower(j)<Z.upper(j));
        ix=TRIGGER_OUTWARD_INTERVAL(xlo,xhi);
        derivative=TRIGGER_OUTWARD_INTERVAL(rhs.rhs_lower(j),rhs.rhs_upper(j));
        flow=ix+TRIGGER_OUTWARD_INTERVAL(0,h).*derivative;
        endpoint=ix+TRIGGER_OUTWARD_INTERVAL(h,h).*derivative;
        R.margin_lower(j)=flow.lo-Z.lower(j);
        R.margin_upper(j)=Z.upper(j)-flow.hi;
        R.endpoint_lower(j)=endpoint.lo;
        R.endpoint_upper(j)=endpoint.hi;
    end
    R.conditional_strict_inclusion=all(R.margin_lower>0) && ...
        all(R.margin_upper>0);
    if R.conditional_strict_inclusion
        R.status='CONDITIONAL_ORIGINAL_RHS_PICARD_STEP__NOT_CLOSED_LOOP_SAFE';
    else
        R.status='UNKNOWN_ORIGINAL_RHS_PICARD_INCLUSION_NOT_SHOWN';
    end
    R.reason=['Verified provenance for real EKF initial error, original HLC ' ...
        'sample-held input image, time-varying mass, tension, hybrid ' ...
        'events, compiler rounding and source audit is still required.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
