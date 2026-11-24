function out = CBF_REAL_D2T_THRUST_ROWS(T,dT,dt,limits)
% Exact ONE-HOLD thrust-box halfspaces for constant ddT over [0,dt].
% T(s)=T+dT*s+(ddT/2)*s^2.  The continuous box constraint
%   Tmin <= T(s) <= Tmax  for EVERY s in [0,dt]
% is EQUIVALENT to ddTLower <= ddT <= ddTUpper when the initial
% (T,dT) is admissible.  Bounds depend on state and dt, not on a
% fabricated fixed acceleration limit.  Physical actuator feasibility
% is separate, as is collision safety between samples.
validateattributes(T,{'double'},{'scalar','real','finite'});
validateattributes(dT,{'double'},{'scalar','real','finite'});
validateattributes(dt,{'double'},{'scalar','positive','finite'});
validateattributes(limits,{'double'},{'vector','numel',2,'real','finite'});
limits=limits(:);
assert(limits(1)>=0 && limits(2)>limits(1),'CBFD2T:ThrustBounds');
low=limits(1);high=limits(2);
% Endpoint gives bounds on ddT with nonzero positive time coefficient.
aLo=2*(low-T-dT*dt)/(dt^2);
aHi=2*(high-T-dT*dt)/(dt^2);
status='EXACT_THRUST_HOLD_BOUNDS';
if T<low || T>high
    status='INITIAL_THRUST_OUT_OF_BOUNDS';
elseif T==low && dT<0
    % At the lower boundary, negative right derivative exits the set
    % immediately, irrespective of any finite acceleration.
    status='OUTWARD_THRUST_RATE_AT_LOWER_BOUNDARY';
elseif T==high && dT>0
    status='OUTWARD_THRUST_RATE_AT_UPPER_BOUNDARY';
else
    % The sharpest intermediate lower-box constraint occurs where
    % sStar = 2*(T-low)/(-dT), if dT<0 and sStar is inside the hold.
    if dT<0 && T>low
        sStar=2*(T-low)/(-dT);
        if sStar<dt
            aLo=max(aLo,dT^2/(2*(T-low)));
        end
    end
    % Analogous sharpest upper-box constraint when dT>0.
    if dT>0 && T<high
        sStar=2*(high-T)/dT;
        if sStar<dt
            aHi=min(aHi,-dT^2/(2*(high-T)));
        end
    end
    if aLo>aHi
        status='NO_CONSTANT_DDT_HOLDS_THRUST_IN_BOX';
    end
end
% Columns correspond to controls [ddT,tauX,tauY], in A*v>=b.
% Retain the rows for diagnosis even on invalid initial state; caller
% MUST gate on out.feasibleInitialHold before solving.
out=struct('A',[1 0 0;-1 0 0], 'b',[aLo;-aHi], ...
    'names',{{'thrust_exact_hold_min','thrust_exact_hold_max'}}, ...
    'ddTInterval',[aLo;aHi], 'initialThrust',T,'initialThrustRate',dT, ...
    'thrustBounds',limits,'holdSeconds',dt, ...
    'feasibleInitialHold',strcmp(status,'EXACT_THRUST_HOLD_BOUNDS'), ...
    'status',status,'exactForConstantDdT',true, ...
    'actuatorDynamicsCertified',false);
end
