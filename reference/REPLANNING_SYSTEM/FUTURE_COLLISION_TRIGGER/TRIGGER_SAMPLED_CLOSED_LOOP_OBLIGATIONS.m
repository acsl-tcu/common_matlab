function R=TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS(P)
% TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS
% A *conditional algebraic* four-second sampled/held-input reach chain.
%
% Physical 24-state order: [p;q;v;w;pL;vL;pT;wL] (3 each).
% A real closed-loop theorem needs, FOR EACH CONTROL INTERVAL k:
% (A) X(k) encloses all possible actual states, including EKF error;
% (B) U(k) encloses the actual original HLC output held over that interval
%     for every possible estimator state, disturbances and parameter;
% (C) F^box(Z(k),U(k),Theta(k),W(k)) encloses the ORIGINAL plant RHS
%     for all x in Z, all times and all admissible masses/branches;
% (D) tension>0, actuation bounds, quaternion/chart and discrete EKF/HLC
%     transitions have rigorously proved validity and state inclusion;
% (E) outward rounding is verified at every arithmetic boundary.
%
% Under those hypotheses, strict Picard inclusion X+[0,h]F(Z) in int(Z)
% implies an all-time enclosure on each interval (standard existence /
% uniqueness hypotheses assumed). X_next is computed from X+hF(Z).
% Its inclusion in the next interval start is automatic: this function
% never accepts a separately invented x0(k+1). The two projected boxes
% for p and pL can then be checked with TRIGGER_DIRECTIONAL_REACH_CERT
% for full UAV, payload and taut straight cable geometry.
%
% This function checks algebraic structure only in ordinary floating point.
% It does NOT establish A--E or certify any double arithmetic as outward.
% It cannot authorize Flight and ALWAYS returns safe_to_commit=false.
R=struct('status','UNKNOWN_CLOSED_LOOP_INTERVAL_PROOF_INCOMPLETE', ...
    'chain_algebra_pass',false,'conditional_geometry_pass',false, ...
    'conditional_min_inclusion_margin',NaN, ...
    'conditional_endpoint_lower',[],'conditional_endpoint_upper',[], ...
    'interval_pass',false(1,0),'conditional_reach',struct(), ...
    'directional_result',struct(), ...
    'outward_rhs_verified',false,'ekf_error_verified',false, ...
    'sampled_hlc_and_hybrid_verified',false, ...
    'mass_tension_actuator_verified',false, ...
    'safe_to_commit',false, ...
    'continuous_closed_loop_safe_certified',false,'reason','');
try
    assert(isstruct(P)&&isscalar(P)&& ...
        all(isfield(P,{'time_grid_s','initial_lower', ...
            'initial_upper','tube_lower','tube_upper', ...
            'rhs_lower','rhs_upper'})), ...
        'TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS:Fields', ...
        'Required 24-state interval data is missing.');
    grid=double(P.time_grid_s(:).');
    assert(numel(grid)>=2 && all(isfinite(grid)) && ...
        all(diff(grid)>0), ...
        'TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS:Time', ...
        'Monotonic real candidate time grid required.');
    n=numel(grid)-1;
    xlo=double(P.initial_lower(:));xhi=double(P.initial_upper(:));
    assert(isequal(size(xlo),[24 1]) && isequal(size(xhi),[24 1]) && ...
        all(isfinite([xlo;xhi])) && all(xlo<=xhi), ...
        'TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS:Initial', ...
        'Initial physical 24-state uncertainty box required.');
    names={'tube_lower','tube_upper','rhs_lower','rhs_upper'};
    for j=1:4
        a=P.(names{j});
        assert(isnumeric(a)&&isreal(a)&&isequal(size(a),[24 n])&& ...
            all(isfinite(a(:))), ...
            'TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS:Dimensions', ...
            'Each interval must supply 24-by-N finite bounds.');
    end
    zl=double(P.tube_lower); zu=double(P.tube_upper);
    fl=double(P.rhs_lower); fu=double(P.rhs_upper);
    assert(all(zl(:)<zu(:))&&all(fl(:)<=fu(:)), ...
        'TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS:Ordered', ...
        'Nonempty ordered interval boxes required.');
    qlo=zeros(3,n);qhi=zeros(3,n);llo=zeros(3,n);lhi=zeros(3,n);
    endpoints_lo=zeros(24,n+1);endpoints_hi=zeros(24,n+1);
    endpoints_lo(:,1)=xlo;endpoints_hi(:,1)=xhi;
    passes=false(1,n);minmargin=Inf;
    for k=1:n
        % Endpoint uncertainty is inherited from the preceding interval.
        % Never substitute the nominal trajectory at the next sample.
        B=struct('h_s',grid(k+1)-grid(k), ...
            'x0_lower',xlo,'x0_upper',xhi, ...
            'tube_lower',zl(:,k),'tube_upper',zu(:,k), ...
            'rhs_lower',fl(:,k),'rhs_upper',fu(:,k));
        C=TRIGGER_PICARD_INTERVAL_OBLIGATIONS(B);
        passes(k)=C.conditional_strict_inclusion;
        minmargin=min(minmargin,C.conditional_min_margin);
        if ~passes(k)
            R.status='UNKNOWN_PICARD_CHAIN_INTERVAL_NOT_INCLUDED';
            R.reason=sprintf('Interval %d lacks strict inclusion; stop.',k);
            R.interval_pass=passes;
            return
        end
        h=grid(k+1)-grid(k);
        xlo=xlo+h*fl(:,k);
        xhi=xhi+h*fu(:,k);
        endpoints_lo(:,k+1)=xlo;
        endpoints_hi(:,k+1)=xhi;
        qlo(:,k)=zl(1:3,k);qhi(:,k)=zu(1:3,k);
        llo(:,k)=zl(13:15,k);lhi(:,k)=zu(13:15,k);
    end
    R.interval_pass=passes;
    R.chain_algebra_pass=all(passes);
    R.conditional_min_inclusion_margin=minmargin;
    R.conditional_endpoint_lower=endpoints_lo;
    R.conditional_endpoint_upper=endpoints_hi;
    R.conditional_reach=struct('time_grid_s',grid, ...
        'uav_lower_m',qlo,'uav_upper_m',qhi, ...
        'payload_lower_m',llo,'payload_upper_m',lhi);
    if isfield(P,'observed_prepared_shapes') && isfield(P,'body')
        g=TRIGGER_DIRECTIONAL_REACH_CERT(R.conditional_reach, ...
            P.observed_prepared_shapes,P.body);
        R.directional_result=g;
        R.conditional_geometry_pass=g.conditional_geometry_pass && ...
            ~isempty(P.observed_prepared_shapes);
    end
    if R.conditional_geometry_pass
        R.status='CONDITIONAL_PICARD_AND_GEOMETRY_CHAIN__NOT_FLIGHT_PROOF';
    else
        R.status='CONDITIONAL_PICARD_CHAIN__GEOMETRY_UNPROVEN';
    end
    R.reason=['Algebraic enclosure only. Missing verified original 24-state ' ...
        'interval RHS, EKF/held-HLC inclusion, time-varying mass, tension, ' ...
        'saturation, discrete updates and outward rounding.'];
catch ME
    R.status='UNKNOWN_CLOSED_LOOP_CHAIN_INPUT_OR_NUMERIC_ERROR';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
