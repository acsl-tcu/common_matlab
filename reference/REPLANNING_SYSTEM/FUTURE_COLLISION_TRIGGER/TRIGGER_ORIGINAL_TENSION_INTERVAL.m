function T=TRIGGER_ORIGINAL_TENSION_INTERVAL(X,U,P)
% Conditional tension formula FROM THE ORIGINAL 24-state plant RHS.
% pT points UAV -> load; for an ideal taut cable and |pT|=1:
%   acceleration_load = -g*e3 - (tension/mL)*pT
% Therefore tension = -mL*pT'*(acceleration_load + g*e3).
% The original RHS kinematics have pTdot = wL x pT, hence
% d(|pT|^2)/dt=2*pT'*(wL x pT)=0 in exact mathematics.
% The unit initial constraint, geometric cable constraints, continuous
% tension positivity, model correctness and directed rounding remain
% separately required. This is NEVER a Flight certification.
T=struct('status','UNKNOWN_TENSION', ...
    'conditional_tension_positive',false, ...
    'tension_lower_N',NaN,'tension_upper_N',NaN, ...
    'original_pT_unit_invariant_identity',true, ...
    'initial_unit_constraint_verified',false, ...
    'taut_cable_validated',false,'safe_to_commit',false,'reason','');
try
    assert(isstruct(X)&&isstruct(U)&&isstruct(P) && ...
        isfield(X,'lower')&&isfield(X,'upper') && ...
        isfield(P,'lower')&&isfield(P,'upper') && ...
        numel(X.lower)==24 && numel(X.upper)==24 && ...
        numel(P.lower)==21 && numel(P.upper)==21, ...
        'TRIGGER_ORIGINAL_TENSION_INTERVAL:Input','24-state and 21-param boxes required.');
    F=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS(X,U,P);
    if ~F.outward_elementary_operations
        T.status='UNKNOWN_TENSION_INTERVAL_RHS';
        T.reason=F.reason;
        return
    end
    p=cell(3,1);acc=cell(3,1);
    for j=1:3
        p{j}=TRIGGER_OUTWARD_INTERVAL(X.lower(18+j),X.upper(18+j));
        acc{j}=TRIGGER_OUTWARD_INTERVAL( ...
            F.rhs_lower(15+j),F.rhs_upper(15+j));
    end
    g=TRIGGER_OUTWARD_INTERVAL(P.lower(9),P.upper(9));
    m=TRIGGER_OUTWARD_INTERVAL(P.lower(20),P.upper(20));
    assert(m.lo>0,'TRIGGER_ORIGINAL_TENSION_INTERVAL:Mass', ...
        'Payload mass range must remain strictly positive.');
    projected=p{1}.*acc{1}+p{2}.*acc{2}+p{3}.*(acc{3}+g);
    tension=-m.*projected;
    T.tension_lower_N=tension.lo;
    T.tension_upper_N=tension.hi;
    T.conditional_tension_positive=tension.lo>0;
    if T.conditional_tension_positive
        T.status='CONDITIONAL_POSITIVE_TENSION__NOT_FULL_TAUT_CERTIFICATE';
        T.reason='Projected original RHS has positive tension assuming the exact unit-direction taut-cable model; remaining obligations unverified.';
    else
        T.status='UNKNOWN_POSITIVE_TENSION_NOT_ESTABLISHED';
        T.reason='Nonpositive/uncertain interval tension; cannot rely on taut-cable dynamics.';
    end
catch ME
    T.status='UNKNOWN_TENSION_INTERVAL_EXCEPTION';
    T.reason=[ME.identifier ' | ' ME.message];
end
end
