function T=TRIGGER_ORIGINAL_PLANT_TRIANGULAR_PICARD(D)
% Independent mathematically sufficient alternative to 24D box Picard.
% States 10:12 form a verified autonomous-input angular subsystem.
% Only a SOURCE-PINNED strict 3D full-time omega tube is used. Remaining
% 21 original RHS coordinates each undergo their own strict Picard test.
% Return UNKNOWN unless BOTH halves are proven, never treat a sampled
% angular path or nominal state as a true-set bound. No safety grant.
% For x'=[g(x_other,w),f(w)], w'=f(w) independent of x_other:
%   1) certify w(t) in Omega for every t on [0,h];
%   2) certify X_other + [0,h] g(Z_other,Omega) inside int Z_other.
% Both certificates together imply a full continuous-time enclosure.
% A failed proof is UNKNOWN; never assert the 21D check implies 3D.
T=struct('success',false,'reason','angular subsystem not certified', ...
    'angular_subslabs',0,'angular_margin',NaN, ...
    'angular_method','', 'angular_energy_norm_upper',NaN, ...
    'residual_margin',NaN,'iterations',0,'safe_to_commit',false, ...
    'tube_lower',nan(24,1),'tube_upper',nan(24,1), ...
    'endpoint_lower',nan(24,1),'endpoint_upper',nan(24,1),'rhs',struct());
try
    required={'x0_lower','x0_upper','u_lower','u_upper', ...
        'param_lower','param_upper','h_s'};
    assert(isstruct(D)&&isscalar(D)&&all(isfield(D,required)), ...
        'TRIGGER_ORIGINAL_PLANT_TRIANGULAR_PICARD:Fields');
    xlo=double(D.x0_lower(:));xhi=double(D.x0_upper(:));
    U=struct('lower',double(D.u_lower(:)), ...
        'upper',double(D.u_upper(:)));
    P=struct('lower',double(D.param_lower(:)), ...
        'upper',double(D.param_upper(:)));
    h=double(D.h_s);
    assert(isequal(size(xlo),[24 1])&&isequal(size(xhi),[24 1]) && ...
        isequal(size(U.lower),[4 1])&&isequal(size(U.upper),[4 1]) && ...
        isequal(size(P.lower),[21 1])&&isequal(size(P.upper),[21 1]) && ...
        isscalar(h)&&isfinite(h)&&h>0 && ...
        all(isfinite([xlo;xhi;U.lower;U.upper;P.lower;P.upper])) && ...
        all(xlo<=xhi)&&all(U.lower<=U.upper)&&all(P.lower<=P.upper), ...
        'TRIGGER_ORIGINAL_PLANT_TRIANGULAR_PICARD:Boxes');
    % First keep the existing fast, sharp strict angular Picard proof.
    % If that specific fixed-point search fails, use the EXACT Euler
    % energy cancellation to certify the whole angular trajectory instead.
    Cang=TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE(D);
    angular_method='STRICT_3D_PICARD';
    if ~Cang.conditional_strict_inclusion
        prior=[Cang.status ' | ' Cang.reason];
        Cang=TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE(D);
        angular_method='CONSTANT_INERTIA_ENERGY';
        if ~Cang.energy_all_time_proven
            T.reason=['Strict 3D: ' prior ' | Energy: ' ...
                Cang.status ' | ' Cang.reason];
            return
        end
    end
    angular=10:12;
    rest=[1:9 13:24];
    Zlo=xlo;Zhi=xhi;
    Zlo(angular)=Cang.tube_lower;
    Zhi(angular)=Cang.tube_upper;
    F0=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS( ...
        struct('lower',Zlo,'upper',Zhi),U,P);
    if ~F0.outward_elementary_operations
        T.reason=['21D initial RHS unavailable: ' F0.reason];return
    end
    for j=rest
        x=TRIGGER_OUTWARD_INTERVAL(xlo(j),xhi(j));
        f=TRIGGER_OUTWARD_INTERVAL(F0.rhs_lower(j),F0.rhs_upper(j));
        image=x+TRIGGER_OUTWARD_INTERVAL(0,h).*f;
        Zlo(j)=min(xlo(j),image.lo);
        Zhi(j)=max(xhi(j),image.hi);
    end
    [Zlo,Zhi]=inflate21(Zlo,Zhi);
    % EXACT angular enclosure, NOT its inflated copy, is the source for
    % all 21-coordinate RHS evaluations on this full time interval.
    Zlo(angular)=Cang.tube_lower;
    Zhi(angular)=Cang.tube_upper;
    for it=1:16
        T.iterations=it;
        B=D;B.tube_lower=Zlo;B.tube_upper=Zhi;
        C=TRIGGER_ORIGINAL_PLANT_PICARD_STEP(B);
        if ~isfield(C,'rhs') || ~isstruct(C.rhs) || ...
                ~isfield(C.rhs,'outward_elementary_operations') || ...
                ~C.rhs.outward_elementary_operations || ...
                any(~isfinite(C.endpoint_lower)) || ...
                any(~isfinite(C.endpoint_upper))
            T.reason=['21D RHS unavailable: ' C.reason];return
        end
        if all(C.margin_lower(rest)>0) && all(C.margin_upper(rest)>0)
            T.success=true;
            T.angular_subslabs=Cang.subslabs;
            T.angular_method=angular_method;
            if strcmp(angular_method,'CONSTANT_INERTIA_ENERGY')
                % Lyapunov energy proof has no strict Picard margin.
                T.angular_margin=NaN;
                T.angular_energy_norm_upper=Cang.energy_norm_upper;
            else
                T.angular_margin=Cang.minimum_margin;
            end
            T.residual_margin=min([C.margin_lower(rest);C.margin_upper(rest)]);
            T.tube_lower=Zlo;T.tube_upper=Zhi;
            T.endpoint_lower=C.endpoint_lower;
            T.endpoint_upper=C.endpoint_upper;
            T.endpoint_lower(angular)=Cang.endpoint_lower;
            T.endpoint_upper(angular)=Cang.endpoint_upper;
            T.rhs=C.rhs;
            T.reason=['Certified all-time 3D angular by ' angular_method ...
                ' and independently certified remaining 21D strict inclusion.'];
            return
        end
        for j=rest
            x=TRIGGER_OUTWARD_INTERVAL(xlo(j),xhi(j));
            f=TRIGGER_OUTWARD_INTERVAL(C.rhs.rhs_lower(j),C.rhs.rhs_upper(j));
            flow=x+TRIGGER_OUTWARD_INTERVAL(0,h).*f;
            Zlo(j)=min(Zlo(j),flow.lo);
            Zhi(j)=max(Zhi(j),flow.hi);
        end
        [Zlo,Zhi]=inflate21(Zlo,Zhi);
        Zlo(angular)=Cang.tube_lower;
        Zhi(angular)=Cang.tube_upper;
    end
    T.reason='Remaining 21 states did not meet strict inclusion in 16 search trials.';
catch ME
    T.reason=[ME.identifier ' | ' ME.message];
end
end

function [lo,hi]=inflate21(lo,hi)
% The trial box is expanded for the search only. The strict inclusion test
% is always repeated on the exact resulting box before accepting it.
width=hi-lo;
assert(all(isfinite([lo;hi;width])) && all(width>=0), ...
    'TRIGGER_ORIGINAL_PLANT_TRIANGULAR_PICARD:Overflow');
pad=max(0.125*width,16*eps(max(max(abs(lo),abs(hi)),1)));
for j=1:numel(lo)
    low=TRIGGER_OUTWARD_INTERVAL(lo(j),lo(j))- ...
        TRIGGER_OUTWARD_INTERVAL(pad(j),pad(j));
    high=TRIGGER_OUTWARD_INTERVAL(hi(j),hi(j))+ ...
        TRIGGER_OUTWARD_INTERVAL(pad(j),pad(j));
    lo(j)=low.lo;hi(j)=high.hi;
end
assert(all(isfinite([lo;hi]))&&all(lo<hi), ...
    'TRIGGER_ORIGINAL_PLANT_TRIANGULAR_PICARD:NoInterior');
end
