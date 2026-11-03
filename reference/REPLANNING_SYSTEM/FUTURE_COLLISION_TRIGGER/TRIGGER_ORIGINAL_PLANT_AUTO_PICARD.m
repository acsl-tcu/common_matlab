function A=TRIGGER_ORIGINAL_PLANT_AUTO_PICARD(D)
% Automatically seek a strict, CONTINUOUS-TIME one-step Picard enclosure
% of the ORIGINAL 24-state plant with interval state, HELD input and
% interval parameters (including loadmass variation on this interval).
%
% This routine does not know the real EKF error, actual HLC held-output
% image, applied-input wiring, tension mode or uncertain mass evolution.
% These are EXTERNAL proof obligations. Fail closed if no strict tube exists.
% Iteration bound only limits computation; it is not a physical bound.
% The local numerical inflation is part of the COMPUTED ENCLOSURE, not a
% fabricated motion limit. Uses TRIGGER_OUTWARD_INTERVAL arithmetic.
A=struct('status','UNKNOWN_AUTO_PICARD_NOT_EXECUTED', ...
    'conditional_strict_inclusion',false,'iterations',0, ...
    'tube_lower',nan(24,1),'tube_upper',nan(24,1), ...
    'endpoint_lower',nan(24,1),'endpoint_upper',nan(24,1), ...
    'conditional_min_margin',NaN,'rhs',struct(), ...
    'original_plant_source_pinned',false, ...
    'source_and_rounding_audited',false, ...
    'actual_ekf_initial_set_certified',false, ...
    'actual_hlc_held_input_certified',false, ...
    'taut_cable_tension_certified',false, ...
    'safe_to_commit',false,'reason','');
try
    required={'x0_lower','x0_upper','u_lower','u_upper', ...
        'param_lower','param_upper','h_s'};
    assert(isstruct(D)&&isscalar(D)&&all(isfield(D,required)), ...
        'TRIGGER_ORIGINAL_PLANT_AUTO_PICARD:Fields', ...
        '24-state initial box, held-input box, parameter box and h are required.');
    xlo=colbox(D.x0_lower,D.x0_upper,24,'X0');
    xhi=double(D.x0_upper(:));
    u=colbox(D.u_lower,D.u_upper,4,'U'); %#ok<NASGU>
    p=colbox(D.param_lower,D.param_upper,21,'P'); %#ok<NASGU>
    h=double(D.h_s);
    assert(isnumeric(D.h_s)&&isscalar(D.h_s)&&isreal(h)&&isfinite(h)&&h>0, ...
        'TRIGGER_ORIGINAL_PLANT_AUTO_PICARD:Time','Positive hold time required.');
    % No assumptions on actual HLC output: this box must be proven by the
    % calling closed-loop verifier for all admissible EKF inputs at tick k.
    U=struct('lower',double(D.u_lower(:)), 'upper',double(D.u_upper(:)));
    P=struct('lower',double(D.param_lower(:)), 'upper',double(D.param_upper(:)));
    zero_rhs=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS( ...
        struct('lower',xlo,'upper',xhi),U,P);
    if ~zero_rhs.outward_elementary_operations
        A.status='UNKNOWN_INITIAL_INTERVAL_RHS';A.reason=zero_rhs.reason;return
    end
    zlo=zeros(24,1);zhi=zeros(24,1);
    for j=1:24
        x=TRIGGER_OUTWARD_INTERVAL(xlo(j),xhi(j));
        f=TRIGGER_OUTWARD_INTERVAL( ...
            zero_rhs.rhs_lower(j),zero_rhs.rhs_upper(j));
        e=x+TRIGGER_OUTWARD_INTERVAL(0,h).*f;
        zlo(j)=min(xlo(j),e.lo);
        zhi(j)=max(xhi(j),e.hi);
    end
    % Strict inclusion requires an actual nonempty interior even where the
    % initial state and derivative are points/zero. Expand only the search
    % box; the proof below still uses the full original interval RHS.
    [zlo,zhi]=inflate(zlo,zhi);
    max_iter=16;
    for k=1:max_iter
        A.iterations=k;
        B=D;
        B.tube_lower=zlo;
        B.tube_upper=zhi;
        C=TRIGGER_ORIGINAL_PLANT_PICARD_STEP(B);
        A.rhs=C.rhs;
        if C.conditional_strict_inclusion
            A.conditional_strict_inclusion=true;
            A.tube_lower=zlo;
            A.tube_upper=zhi;
            A.endpoint_lower=C.endpoint_lower;
            A.endpoint_upper=C.endpoint_upper;
            A.conditional_min_margin=min([C.margin_lower;C.margin_upper]);
            A.status='CONDITIONAL_AUTO_PICARD_INTERVAL__NOT_CLOSED_LOOP_SAFETY';
            A.reason=['A strict all-time tube was constructed for supplied X0,U,P,h. ' ...
                'Real EKF error, actual held HLC input enclosure, mode and numerical ' ...
                'source certification are still missing.'];
            return
        end
        if ~isfield(C,'rhs') || ~isstruct(C.rhs) || ...
                ~isfield(C.rhs,'outward_elementary_operations') || ...
                ~C.rhs.outward_elementary_operations || ...
                any(~isfinite(C.endpoint_lower)) || ...
                any(~isfinite(C.endpoint_upper))
            A.status='UNKNOWN_PICARD_RHS_OR_CHART';A.reason=C.reason;return
        end
        % Enclose the CURRENT trial Z and the newly calculated flow image
        % using interval-safe endpoint bounds, then widen the proposed Z.
        flowlo=zeros(24,1);flowhi=zeros(24,1);
        for j=1:24
            x=TRIGGER_OUTWARD_INTERVAL(xlo(j),xhi(j));
            f=TRIGGER_OUTWARD_INTERVAL( ...
                C.rhs.rhs_lower(j),C.rhs.rhs_upper(j));
            b=x+TRIGGER_OUTWARD_INTERVAL(0,h).*f;
            flowlo(j)=b.lo;flowhi(j)=b.hi;
        end
        [zlo,zhi]=inflate(min(zlo,flowlo),max(zhi,flowhi));
    end
    A.status='UNKNOWN_PICARD_NO_STRICT_TUBE_WITHIN_ITERATION_BUDGET';
    A.reason='No strict fixed-point tube found; unsafe to infer non-collision.';
catch ME
    A.status='UNKNOWN_PICARD_INPUT_OR_NUMERIC_EXCEPTION';
    A.reason=[ME.identifier ' | ' ME.message];
end
end

function lo=colbox(L,H,n,label)
assert(isnumeric(L)&&isnumeric(H)&&isreal(L)&&isreal(H) && ...
    isequal(size(L),[n 1])&&isequal(size(H),[n 1]) && ...
    all(isfinite(L))&&all(isfinite(H))&&all(L<=H), ...
    'TRIGGER_ORIGINAL_PLANT_AUTO_PICARD:Box', ...
    '%s must be an ordered finite %dx1 double box.',label,n);
lo=double(L(:));
end

function [lo,hi]=inflate(lo,hi)
assert(all(isfinite(lo))&&all(isfinite(hi))&&all(lo<=hi), ...
    'TRIGGER_ORIGINAL_PLANT_AUTO_PICARD:Inflate','Invalid trial tube.');
width=hi-lo;
assert(all(isfinite(width)), ...
    'TRIGGER_ORIGINAL_PLANT_AUTO_PICARD:Inflate','Tube overflow.');
% The expansion factor is a numerical fixed-point search policy, not a
% bound on acceleration, speed or physical motion.
scale=max(max(abs(lo),abs(hi)),1);
pad=max(0.125*width,16*eps(scale));
for j=1:numel(lo)
    xl=TRIGGER_OUTWARD_INTERVAL(lo(j),lo(j))- ...
        TRIGGER_OUTWARD_INTERVAL(pad(j),pad(j));
    xu=TRIGGER_OUTWARD_INTERVAL(hi(j),hi(j))+ ...
        TRIGGER_OUTWARD_INTERVAL(pad(j),pad(j));
    lo(j)=xl.lo;hi(j)=xu.hi;
end
assert(all(isfinite(lo))&&all(isfinite(hi))&&all(lo<hi), ...
    'TRIGGER_ORIGINAL_PLANT_AUTO_PICARD:Inflate','Finite interior not found.');
end
