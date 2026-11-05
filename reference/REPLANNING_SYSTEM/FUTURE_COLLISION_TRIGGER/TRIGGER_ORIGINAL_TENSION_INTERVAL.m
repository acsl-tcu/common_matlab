function T=TRIGGER_ORIGINAL_TENSION_INTERVAL(X,U,P,initial_slab)
% Source-derived DIRECT tension interval, conditional on exact taut mode.
% Original with_load_model_euler_for_HL.m:
%   aL + g*e3 = -pT * [L*m*|omegaL x pT|^2 - u1*D]/(m+mL),
%   D = pT1*t67 - pT2*t68 + pT3*t77.
% Exact taut-manifold identity pT'*pT=1 therefore yields
%   N = (mL/(m+mL))*(L*m*|omegaL x pT|^2-u1*D).
% This removes the interval dependency from computing 24-state aL and
% projecting it back onto pT. Positive N lower on the whole flowpipe
% proves positive model tension on that slab (given taut source/model).
% No dynamic plant truth, EKF, source-world obstacle or nominal collision
% is used. This function never grants safe_to_commit.
T=struct('status','UNKNOWN_TENSION', ...
    'conditional_tension_positive',false, ...
    'tension_lower_N',NaN,'tension_upper_N',NaN, ...
    'original_pT_unit_invariant_identity',true, ...
    'initial_unit_constraint_verified',false, ...
    'taut_cable_validated',false, ...
    'direct_original_source_formula',false, ...
    'orientation_interval_method','EXACT_EULER_PLUS_LOCAL_LIPSCHITZ', ...
    'thrust_axis_dot_lower',NaN,'thrust_axis_dot_upper',NaN, ...
    'lipschitz_tightening_used',false, ...
    'joint_drift_tightening_used',false, ...
    'joint_drift_bound',struct(), ...
    'safe_to_commit',false,'reason','');
try
    must={'lower','upper'};
    assert(isstruct(X)&&isscalar(X)&&all(isfield(X,must)) && ...
        isstruct(U)&&isscalar(U)&&all(isfield(U,must)) && ...
        isstruct(P)&&isscalar(P)&&all(isfield(P,must)) && ...
        isequal(size(X.lower),[24 1])&&isequal(size(X.upper),[24 1]) && ...
        isequal(size(U.lower),[4 1])&&isequal(size(U.upper),[4 1]) && ...
        isequal(size(P.lower),[21 1])&&isequal(size(P.upper),[21 1]), ...
        'TRIGGER_ORIGINAL_TENSION_INTERVAL:Inputs', ...
        'Original 24D state, 4D input, 21D parameter boxes required.');
    assert(all(isfinite([X.lower;X.upper;U.lower;U.upper;P.lower;P.upper])) && ...
        all(X.lower<=X.upper)&&all(U.lower<=U.upper)&& ...
        all(P.lower<=P.upper)&&P.lower(1)>0&&P.lower(20)>0&& ...
        P.lower(21)>0, ...
        'TRIGGER_ORIGINAL_TENSION_INTERVAL:Physics', ...
        'Finite, ordered boxes and positive UAV mass, load mass, cable length required.');

    c=@(a,b) TRIGGER_OUTWARD_INTERVAL(a,b);
    w=cell(3,1);d=cell(3,1);
    for j=1:3
        w{j}=c(X.lower(21+j),X.upper(21+j));
        d{j}=c(X.lower(18+j),X.upper(18+j));
    end
    % EXACT RHS t33,t34,t35 = omegaL cross pT, except component ordering.
    a=w{1}.*d{2}-w{2}.*d{1};
    b=w{1}.*d{3}-w{3}.*d{1};
    cc=w{2}.*d{3}-w{3}.*d{2};
    cross2=a.^2+b.^2+cc.^2;
    cross2=c(max(0,cross2.lo),cross2.hi); % square-sum identity

    % Source algebra, verified by exact symbolic expansion of the original
    % quaternion t52..t55/t67/t68/t77 intermediates:
    %   t67 = sin(pitch)*cos(roll)*cos(yaw)+sin(roll)*sin(yaw)
    %   t68 = sin(roll)*cos(yaw)-sin(pitch)*cos(roll)*sin(yaw)
    %   t77 = cos(pitch)*cos(roll).
    % Therefore the exact source projection D is directly expressible in
    % the THREE physical Euler angles, without the dependency inflation
    % from quaternion half-angle reconstruction.
    roll=c(X.lower(4),X.upper(4));
    pitch=c(X.lower(5),X.upper(5));
    yaw=c(X.lower(6),X.upper(6));
    sr=sin(roll); cr=cos(roll);
    sp=sin(pitch); cp=cos(pitch);
    sy=sin(yaw); cy=cos(yaw);
    horizontal=d{1}.*cy+d{2}.*sy;
    lateral=d{1}.*sy-d{2}.*cy;
    D=horizontal.*sp.*cr+lateral.*sr+d{3}.*cp.*cr;

    % Independently, b(theta) is a UNIT vector obtained from a rotation
    % R(theta)e3.  Rotation of a unit vector by angle delta changes its
    % Euclidean position by at most |delta|.  Three successive Euler
    % rotations give the GLOBAL (not small-angle) Lipschitz bound
    %  |d dot b(theta)-d dot b(theta0)|
    %      <= |dr|+|dp|+|dy|, whenever ||d||=1.
    % This uses the exact taut-manifold unit-pT identity used in the
    % source-projected tension expression.  Importantly, it avoids the
    % independent-interval multiplication of different Euler-angle terms.
    % Evaluate the extra witness ONLY when the direct interval has not
    % already established D<0, to avoid gratuitous hotpath computation.
    if D.hi>=0
        anglesLo=double(X.lower(4:6));
        anglesHi=double(X.upper(4:6));
        theta0=0.5*anglesLo+0.5*anglesHi;
        if all(isfinite(theta0))
            % Endpoint subtraction and sums are outward.  The tiny extra
            % widening also protects exact-center rounding at subnormals.
            radius=c(0,0);
            for angleI=1:3
                diff=c(anglesLo(angleI),anglesHi(angleI))- ...
                     c(theta0(angleI),theta0(angleI));
                maxdist=max(abs(diff.lo),abs(diff.hi));
                % Widen the computed radius outward (addition is outward).
                radius=radius+c(0,maxdist);
            end
            croll=cos(c(theta0(1),theta0(1)));
            sroll=sin(c(theta0(1),theta0(1)));
            cpitch=cos(c(theta0(2),theta0(2)));
            spitch=sin(c(theta0(2),theta0(2)));
            cyaw=cos(c(theta0(3),theta0(3)));
            syaw=sin(c(theta0(3),theta0(3)));
            b1=spitch.*croll.*cyaw+sroll.*syaw;
            b2=spitch.*croll.*syaw-sroll.*cyaw;
            b3=cpitch.*croll;
            anchor=d{1}.*b1+d{2}.*b2+d{3}.*b3;
            DL=anchor-radius;
            DU=anchor+radius;
            % Intersection is valid only because BOTH intervals contain
            % the exact physical D for every admissible taut state.
            low=max(D.lo,DL.lo);
            high=min(D.hi,DU.hi);
            if low>high
                error('TRIGGER_ORIGINAL_TENSION_INTERVAL:Correlation', ...
                    'Independent exact Euler and rotation-Lipschitz enclosures disagree.');
            end
            T.lipschitz_tightening_used=(low>D.lo || high<D.hi);
            D=c(low,high);
        end
    end
    % Both pT and body thrust axis are unit vectors on the exact source
    % taut manifold.  Their dot product necessarily lies in [-1,1].
    % This is NOT a generic clipping heuristic: it follows from Cauchy.
    dlow=max(-1,D.lo);
    dhigh=min(1,D.hi);
    if dlow>dhigh
        error('TRIGGER_ORIGINAL_TENSION_INTERVAL:UnitAxis', ...
            'Source unit-vector geometry incompatible with interval D.');
    end
    D=c(dlow,dhigh);
    T.thrust_axis_dot_lower=D.lo;
    T.thrust_axis_dot_upper=D.hi;

    m=c(P.lower(1),P.upper(1));
    mL=c(P.lower(20),P.upper(20));
    L=c(P.lower(21),P.upper(21));
    thrust=c(U.lower(1),U.upper(1));
    numerator=L.*m.*cross2-thrust.*D;
    tension=mL./(m+mL).*numerator;
    % The joint-axis proof has a cost. Avoid recomputing it on the many
    % subslabs that ALREADY prove strictly positive tension using the
    % existing full-time interval. It is mathematically only an optional
    % intersection needed on the stalled slabs.
    if tension.lo<=0 && nargin>=4 && ~isempty(initial_slab)
        J=TRIGGER_TENSION_JOINT_ROTATION_BOUND(initial_slab,X);
        if ~J.valid
            error('TRIGGER_ORIGINAL_TENSION_INTERVAL:JointDrift', ...
                'Supplied source drift certificate is invalid: %s',J.reason);
        end
        dl=max(D.lo,J.dot_lower);
        dh=min(D.hi,J.dot_upper);
        if dl>dh
            error('TRIGGER_ORIGINAL_TENSION_INTERVAL:JointDisjoint', ...
                'Two source-proven enclosures of taut dot product are disjoint.');
        end
        T.joint_drift_tightening_used=(dl>D.lo || dh<D.hi);
        T.joint_drift_bound=J;
        D=c(dl,dh);
        T.thrust_axis_dot_lower=D.lo;
        T.thrust_axis_dot_upper=D.hi;
        numerator=L.*m.*cross2-thrust.*D;
        tension=mL./(m+mL).*numerator;
    end
    T.direct_original_source_formula=true;
    T.tension_lower_N=tension.lo;
    T.tension_upper_N=tension.hi;
    T.conditional_tension_positive=(tension.lo>0);
    if T.conditional_tension_positive
        T.status='CONDITIONAL_POSITIVE_TENSION_SOURCE_FACTORED';
        T.reason='Exact source-projected taut tension has a positive all-time lower interval bound; source/hybrid physical validity remain separate.';
    else
        T.status='UNKNOWN_POSITIVE_TENSION_NOT_ESTABLISHED';
        T.reason='The original source direct tension interval straddles zero or negative values; no taut-mode certificate.';
    end
catch ME
    T.status='UNKNOWN_TENSION_DIRECT_SOURCE_EVALUATION';
    T.reason=[ME.identifier ' | ' ME.message];
end
end
