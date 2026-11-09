% AUDIT-ONLY frozen scalar product reference from the previous accepted ZIP.
% Delete with other CHECK_* code after validation. NOT production.
function R = CHECK_C9_TORQUE_SCALAR_REFERENCE(piece,authority,options)
% TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL
% Continuous-time REAL-ARITHMETIC sufficient certificate for NOMINAL
% rigid-body torques implied by a taut-load C9 flat-output trajectory.
%
% Unlike sampled inverse-dynamics checks, every derivative of the payload
% polynomial is enclosed by its Bernstein convex hull on each time leaf.
% Interval differentiation encloses q=w/||w|| through derivative 4,
% F=-(mU+mL)*w - mU*L*q'' through derivative 2, and heading-exact R through
% derivative 2. M = J*omega' + cross(omega,J*omega) is then enclosed.
%
% Body yaw is EXACTLY the planar path heading: b1 has horizontal projection
% in the direction of pU'_xy provided F_z>0, rather than merely aligning a
% projected intermediate yaw vector with b3.
%
% The optional [mass_lo,mass_hi] range is over a CONSTANT physical mass
% chosen anywhere in that externally certified interval; it does NOT model
% an actual time-varying physical mass, derivatives of the EKF estimate,
% actuator dynamics, closed-loop tracking, or uncertain HLC/plant behavior.
% For actual mL(t), bounds on dmL/dt and d2mL/dt2 are additionally necessary.
% Hence no outcome from this function ever authorizes HLC COMMIT.
%
% Real-arithmetic Bernstein/interval certificate only. MATLAB floating-point
% interval arithmetic below DOES NOT use directed rounding; a separate
% numerical enclosure error budget must be certified before safety-critical
% use. Failure or exhausted budgets result in UNKNOWN, not collision.
clock=tic;
R=struct('status','UNKNOWN_NOMINAL_TORQUE_NOT_PROVEN', ...
    'conditional_nominal_all_time_torque_proven',false, ...
    'safe_to_commit',false,'actual_HLC_tracking_proven',false, ...
    'numerically_directed_rounding_proven',false, ...
    'constant_physical_mass_assumed',true, ...
    'mass_scope','NO_VALID_AUTHORITY_YET', ...
    'payload_mass_snapshot_kg',NaN, ...
    'checked_nodes',0,'accepted_leaves',0, ...
    'fast_norm_checked_nodes',0,'fast_norm_accepted_leaves',0, ...
    'fast_norm_exception_count',0,'fast_norm_enabled',true, ...
    'precomputed_binomial_enabled',true, ...
    'factored_diagonal_gyro_enabled',true, ...
    'infeasible_leaf_early_exit_enabled',true, ...
    'nominal_torque_limit_witness_nodes',0, ...
    'detailed_interval_checked_nodes',0, ...
    'unresolved_interval_s',[NaN NaN], ...
    'last_leaf_reason','NOT_EVALUATED', ...
    'last_exception_identifier','','last_exception_message','', ...
    'torque_bounds_Nm',NaN(3,2), ...
    'minimum_torque_headroom_Nm',NaN, ...
    'elapsed_ms',NaN);
try
    assert(isstruct(piece)&&isscalar(piece)&&all(isfield(piece, ...
        {'t0_s','duration_s','derivative_controls'}))&& ...
        isfinite(piece.t0_s)&&isfinite(piece.duration_s)&&piece.duration_s>0, ...
        'C9_TORQUE:Piece','Invalid C9 piece');
    assert(isstruct(authority)&&isscalar(authority)&&all(isfield(authority, ...
        {'uav_mass_kg','payload_mass_kg','cable_length_m', ...
        'gravity_world_mps2','uav_inertia_kgm2','torque_limits_Nm'})), ...
        'C9_TORQUE:Authority','Need current HLC/plant physical authority');
    assert(isstruct(options)&&isscalar(options)&&all(isfield(options, ...
        {'max_subdivision_depth','max_node_count','minimum_force_z_N', ...
        'minimum_planar_speed_mps','torque_guard_Nm'})), ...
        'C9_TORQUE:Options','No invented physical or numerical guards');
    mu=double(authority.uav_mass_kg);
    m0=double(authority.payload_mass_kg);
    L=double(authority.cable_length_m);
    g=double(authority.gravity_world_mps2(:));
    J=double(authority.uav_inertia_kgm2(:));
    limits=double(authority.torque_limits_Nm);
    guard=double(options.torque_guard_Nm(:));
    minFz=double(options.minimum_force_z_N);
    minV=double(options.minimum_planar_speed_mps);
    dep=double(options.max_subdivision_depth);
    budget=double(options.max_node_count);
    enable_fast_norm=true;
    if isfield(options,'enable_fast_norm_sufficient_bound')
        user_switch=options.enable_fast_norm_sufficient_bound;
        assert((islogical(user_switch)||isnumeric(user_switch)) && ...
            isscalar(user_switch)&&isreal(user_switch)&& ...
            isfinite(double(user_switch))&& ...
            any(double(user_switch)==[0 1]), ...
            'C9_TORQUE:FastNormSwitch','Fast norm switch must be logical 0/1');
        enable_fast_norm=logical(user_switch);
    end
    R.fast_norm_enabled=enable_fast_norm;
    % Binomial orders are intrinsic to the fourth-order taut-load jet,
    % not tuneable robot or obstacle parameters. This saves repeated calls
    % to MATLAB nchoosek inside each interval subdivision.  The OFF
    % switch exists only to compare against the original arithmetic.
    fast_binomial=true;
    if isfield(options,'use_precomputed_binomial')
        flag=options.use_precomputed_binomial;
        assert((islogical(flag)||isnumeric(flag))&&isscalar(flag)&& ...
            isreal(flag)&&isfinite(double(flag))&& ...
            (double(flag)==0||double(flag)==1), ...
            'C9_TORQUE:BinomialSwitch', ...
            'use_precomputed_binomial must be 0 or 1.');
        fast_binomial=logical(flag);
    end
    R.precomputed_binomial_enabled=fast_binomial;
    % Algebraically exact rigid-body Euler identity for diagonal inertia:
    %   [Jx ax+(Jz-Jy)*wy*wz;
    %    Jy ay+(Jx-Jz)*wz*wx;
    %    Jz az+(Jy-Jx)*wx*wy].
    % Unlike a generic interval cross product, it preserves the shared
    % angular-velocity factor and cancels equal inertia terms BEFORE
    % enclosure.  The user switch permits direct A/B comparison.
    factor_gyro=true;
    if isfield(options,'use_factored_diagonal_gyro')
        v=options.use_factored_diagonal_gyro;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_TORQUE:GyroSwitch', ...
            'use_factored_diagonal_gyro must be logical 0/1.');
        factor_gyro=logical(v);
    end
    R.factored_diagonal_gyro_enabled=factor_gyro;
    % A rigorously disjoint torque interval at any nonempty time leaf
    % already rules out this NOMINAL C9 candidate. No need to refine the
    % leaf; report UNKNOWN as before, never a safety certificate.
    witness_exit=true;
    if isfield(options,'enable_infeasible_leaf_early_exit')
        v=options.enable_infeasible_leaf_early_exit;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_TORQUE:WitnessSwitch', ...
            'enable_infeasible_leaf_early_exit must be logical 0/1.');
        witness_exit=logical(v);
    end
    R.infeasible_leaf_early_exit_enabled=witness_exit;
    % Pascal rows n=0..4, coefficients k=0..n.  Exactly integer-valued
    % in binary64; the same numbers returned by nchoosek for these orders.
    B=[1 0 0 0 0; ...
       1 1 0 0 0; ...
       1 2 1 0 0; ...
       1 3 3 1 0; ...
       1 4 6 4 1];
    assert(all(isfinite([mu,m0,L,g(:).',J(:).',limits(:).',guard(:).',minFz,minV,dep,budget]))&& ...
        all([mu,m0,L]>0)&&numel(g)==3&&numel(J)==3&&all(J>0)&& ...
        isequal(size(limits),[3 2])&&all(limits(:,2)>limits(:,1))&& ...
        numel(guard)==3&&all(guard>=0)&&minFz>=0&&minV>=0&& ...
        dep>=0&&dep==fix(dep)&&budget>=1&&budget==fix(budget), ...
        'C9_TORQUE:Authority','Malformed external physical limits or guard');
    % Point mass is sourced from this candidate's current live authority.
    % No uncertainty width is synthesized from the estimator.  If an
    % externally certified constant-physical-mass interval is supplied,
    % evaluate its entire range; it is not an EKF prediction.
    R.payload_mass_snapshot_kg=m0;
    R.mass_scope='ONE_CURRENT_MASS_SNAPSHOT__NOT_FUTURE_MASS_PROOF';
    ml=[m0 m0];
    if isfield(options,'certified_constant_physical_mass_interval_kg')
        R.mass_scope='EXTERNALLY_PROVIDED_CONSTANT_MASS_INTERVAL';
        ml=double(options.certified_constant_physical_mass_interval_kg(:).');
        assert(numel(ml)==2&&all(isfinite(ml))&&ml(1)>0&& ...
            ml(2)>=ml(1)&&ml(1)<=m0&&m0<=ml(2), ...
            'C9_TORQUE:Mass','Externally certified constant-mass interval required');
    end
    % Bind the certified derivative intervals to the ACTUAL polynomial.
    % Checking only caller-supplied derivative_controls would allow the
    % certificate to prove a different curve from control_points_m.
    assert(isfield(piece,'control_points_m')&& ...
        isnumeric(piece.control_points_m)&& ...
        isreal(piece.control_points_m)&& ...
        isequal(size(piece.control_points_m),[3 24])&& ...
        all(isfinite(piece.control_points_m(:))), ...
        'C9_TORQUE:PolynomialBinding','Require real degree-23 control points');
    D=piece.derivative_controls;
    assert(iscell(D)&&numel(D)==10, ...
        'C9_TORQUE:Jets','Need every C9 derivative polygon');
    expected=double(piece.control_points_m);
    hull=cell(1,7);
    for k=1:10
        assert(isnumeric(D{k})&&isreal(D{k})&& ...
            isequal(size(D{k}),[3 25-k])&& ...
            all(isfinite(D{k}(:)))&& ...
            isequal(double(D{k}),expected), ...
            'C9_TORQUE:JetsBinding', ...
            'Derivative polygon does not match the certified C9 piece');
        if k<=7
            hull{k}=expected;
        end
        if k<10
            expected=(24-k)/piece.duration_s*diff(expected,1,2);
        end
    end
    [ok,reason]=node(hull,0,1,0);
    if ok
        R.conditional_nominal_all_time_torque_proven=true;
        R.status='CONDITIONAL_NOMINAL_RIGID_BODY_TORQUE_ALL_TIME__HLC_TRACKING_PENDING';
        R.minimum_torque_headroom_Nm=min([ ...
            R.torque_bounds_Nm(:,1)-limits(:,1)-guard, ...
            limits(:,2)-guard-R.torque_bounds_Nm(:,2)],[],'all');
    else
        R.status=reason;
    end
catch ME
    R.status=['UNKNOWN_TORQUE_CERTIFICATE_INPUT_' ME.identifier];
    R.last_exception_identifier=ME.identifier;
    R.last_exception_message=ME.message;
end
R.elapsed_ms=1000*toc(clock);

    function [good,reason]=node(polys,lo,hi,depth)
        R.checked_nodes=R.checked_nodes+1;
        good=false;reason='UNKNOWN_TORQUE_INTERVAL_BOUNDS';
        if R.checked_nodes>budget
            R.unresolved_interval_s=piece.t0_s+piece.duration_s*[lo hi];
            reason='UNKNOWN_TORQUE_NODE_BUDGET';return
        end
        try
            % Cheap scalar-norm sufficient certificate first.  This bound
            % covers the SAME exact C9 polynomial and all t in this leaf;
            % failure to prove merely falls back to the detailed interval
            % model.  It is not a second candidate-generator/stage.
            if enable_fast_norm
                R.fast_norm_checked_nodes=R.fast_norm_checked_nodes+1;
                quick_valid=false;
                try
                    [Mquick,quick_valid]=quick_norm_box(polys);
                catch
                    % A fast-path implementation exception never bypasses
                    % the pre-existing detailed interval certificate.
                    R.fast_norm_exception_count=R.fast_norm_exception_count+1;
                end
                if quick_valid && all(Mquick.lo>=limits(:,1)+guard) && ...
                        all(Mquick.hi<=limits(:,2)-guard)
                    good=true;reason='PROVEN_FAST_NORM_LEAF';
                    R.accepted_leaves=R.accepted_leaves+1;
                    R.fast_norm_accepted_leaves=R.fast_norm_accepted_leaves+1;
                    if any(isnan(R.torque_bounds_Nm(:)))
                        R.torque_bounds_Nm=[Mquick.lo Mquick.hi];
                    else
                        R.torque_bounds_Nm(:,1)=min(R.torque_bounds_Nm(:,1),Mquick.lo);
                        R.torque_bounds_Nm(:,2)=max(R.torque_bounds_Nm(:,2),Mquick.hi);
                    end
                    return
                end
            end
            R.detailed_interval_checked_nodes=R.detailed_interval_checked_nodes+1;
            [M,nondegenerate]=torque_box(polys);
            if ~nondegenerate
                R.last_leaf_reason='SINGULAR_FORCE_Z_OR_HORIZONTAL_HEADING';
            else
                R.last_leaf_reason='NOMINAL_TORQUE_INTERVAL_NOT_WITHIN_LIMITS';
            end
            if nondegenerate && all(M.lo>=limits(:,1)+guard) && ...
                    all(M.hi<=limits(:,2)-guard)
                good=true;reason='PROVEN_LEAF';
                R.accepted_leaves=R.accepted_leaves+1;
                if any(isnan(R.torque_bounds_Nm(:)))
                    R.torque_bounds_Nm=[M.lo M.hi];
                else
                    R.torque_bounds_Nm(:,1)=min(R.torque_bounds_Nm(:,1),M.lo);
                    R.torque_bounds_Nm(:,2)=max(R.torque_bounds_Nm(:,2),M.hi);
                end
                return
            end
            % If the entire enclosure for any torque component is
            % outside the certified HLC nominal limit, even one instant
            % is unsafe for this candidate.  This leaf is nonempty, so
            % continuing subdivision cannot make this C9 candidate safe.
            % Keep UNKNOWN (not COMMIT); don't call it physical collision.
            if witness_exit && nondegenerate && ...
                    (any(M.lo>limits(:,2)-guard)|| ...
                     any(M.hi<limits(:,1)+guard))
                R.nominal_torque_limit_witness_nodes= ...
                    R.nominal_torque_limit_witness_nodes+1;
                R.last_leaf_reason='PROVEN_NOMINAL_TORQUE_LIMIT_EXCLUSION';
                R.unresolved_interval_s=piece.t0_s+ ...
                    piece.duration_s*[lo hi];
                reason='UNKNOWN_TORQUE_PROVEN_NOMINAL_LIMIT_EXCLUSION';
                return
            end
        catch ME
            R.last_leaf_reason=['INTERVAL_ARITHMETIC_UNKNOWN_' ME.identifier];
            R.last_exception_identifier=ME.identifier;
            R.last_exception_message=ME.message;
            % Degenerate/nonfinite interval and unsupported singularity:
            % subdivide rather than treating a failed bound as safe.
        end
        if depth>=dep
            R.unresolved_interval_s=piece.t0_s+piece.duration_s*[lo hi];
            reason='UNKNOWN_TORQUE_LEAF_UNRESOLVED';return
        end
        left=cell(size(polys));right=cell(size(polys));
        for j=1:numel(polys)
            [left{j},right{j}]=bisect(polys{j});
        end
        mid=(lo+hi)/2;
        [g1,r1]=node(left,lo,mid,depth+1);
        if ~g1,reason=r1;return;end
        [g2,r2]=node(right,mid,hi,depth+1);
        good=g2;reason=r2;
    end
    function c=choose(n,k)
        if fast_binomial
            c=B(n+1,k+1);
        else
            c=nchoosek(n,k);
        end
    end
    function [M,valid]=quick_norm_box(P)
        % Real-arithmetic sufficient bounds; no sampling and no heuristic
        % clearance.  The existing floating-point directed-rounding caveat
        % applies to this fast proof just as to the detailed proof.
        M=I(NaN(3,1)); valid=false;
        % w = g - pL'', q = w/||w||, F = -(mu+m)w-mu L q''.
        wbox=sub(I(g),Iof(P{3}));
        aa=max(abs(wbox.lo),abs(wbox.hi));
        bb=max([wbox.lo,-wbox.hi,zeros(3,1)],[],2);
        wmin=norm(bb,2);
        if ~(isfinite(wmin)&&wmin>0), return; end
        W=zeros(1,5); W(1)=norm(aa,2);
        for kk=1:4
            % Bernstein convex-hull upper bounds of ||w^(kk)||_2.
            % w^(kk) = -pL^(kk+2) for kk >= 1.
            W(kk+1)=max(sqrt(sum(P{kk+3}.^2,1)));
        end
        % n=||w||; n^2=w.w; n*q=w. Differentiate both identities
        % recursively. All arrays below enclose absolute derivative norms.
        N=zeros(1,5);Q=zeros(1,5);Q(1)=1;
        for k=1:4
            acc=0;
            for j=0:k
                acc=acc+choose(k,j)*W(j+1)*W(k-j+1);
            end
            for j=1:k-1
                acc=acc+choose(k,j)*N(j+1)*N(k-j+1);
            end
            N(k+1)=acc/(2*wmin);
            acc=W(k+1);
            for j=1:k
                acc=acc+choose(k,j)*N(j+1)*Q(k-j+1);
            end
            Q(k+1)=acc/wmin;
        end
        mplus=mu+ml(2);
        F0=mplus*W(1)+mu*L*Q(3);
        F1=mplus*W(2)+mu*L*Q(4);
        F2=mplus*W(3)+mu*L*Q(5);
        % Exact interval product extremum: +(mu+m)*w_z.  The q''
        % component is enclosed in [-Q2,Q2]. No assumed gravity sign.
        wz=[wbox.lo(3),wbox.hi(3)];
        corners=[(mu+ml(1))*wz, (mu+ml(2))*wz];
        fzlo=-max(corners)-mu*L*Q(3);
        if ~(isfinite(fzlo)&&fzlo>minFz && F0>0),return;end
        % Horizontal UAV velocity hraw = (pL' - L q')_xy. The
        % distance from the origin to its complete interval box is a
        % continuous-time lower bound on ||hraw||.
        vb=Iof(P{2});
        vlo=vb.lo(1:2)-L*Q(2);
        vhi=vb.hi(1:2)+L*Q(2);
        vmin=norm(max([vlo,-vhi,zeros(2,1)],[],2),2);
        if ~(isfinite(vmin)&&vmin>minV && vmin>0),return;end
        V1=max(sqrt(sum(P{3}(1:2,:).^2,1)))+L*Q(3);
        V2=max(sqrt(sum(P{4}(1:2,:).^2,1)))+L*Q(4);
        % For u=x/||x||: ||u'||<=||x'||/x_min,
        % ||u''||<=||x''||/x_min+3||x'||^2/x_min^2.
        H1=V1/vmin;
        H2=V2/vmin+3*H1^2;
        Z1=F1/fzlo;
        Z2=F2/fzlo+3*Z1^2;
        % For a = horizontal-normal(hraw) x b3, ||a|| >= b3_z.
        % b3_z = F_z/||F|| >= fzlo/F0 > 0.
        amin=fzlo/F0;
        A1=H1+Z1;
        A2=H2+2*H1*Z1+Z2;
        B11=A1/amin;
        B12=A2/amin+3*B11^2;
        B21=Z1+B11;
        B22=Z2+2*Z1*B11+B12;
        % The orthonormal columns [b1,b2,b3] imply
        % ||omega|| <= ||R'||_F, ||dot(omega)|| <= ||R''||_F.
        omega_bound=hypot(hypot(B11,B21),Z1);
        alpha_bound=hypot(hypot(B12,B22),Z2);
        % Component-wise Euler rigid-body equation, diag(J):
        % Old |M_i|<=J_i*||dot(omega)||+max(J)*||omega||^2.
        % New factored: use each diagonal inertia difference and /2.
        if factor_gyro
            % |wi*wj| <= (wi^2+wj^2)/2 <= ||omega||^2/2.
            % This is a certified stronger enclosure than max(J)*||w||^2
            % for strictly positive diagonal inertia components.
            deltaJ=abs([J(3)-J(2);J(1)-J(3);J(2)-J(1)]);
            tau=J*alpha_bound+0.5*deltaJ*omega_bound^2;
        else
            tau=J*alpha_bound+max(J)*omega_bound^2;
        end
        if any(~isfinite([W,N,Q,F0,F1,F2,fzlo,vmin,tau(:).'])),return;end
        M=I(-tau,tau);valid=true;
    end

    function [M,valid]=torque_box(P)
        valid=false;
        w=cell(1,5);
        w{1}=sub(I(g),Iof(P{3}));
        for n=1:4
            w{n+1}=neg(Iof(P{n+3}));
        end
        q=unit_jet(w,4,B,fast_binomial);
        f=cell(1,3);v=cell(1,3);
        % [m_min,m_max] describes ONE uncertain SCALAR mass.  I(ml)
        % incorrectly creates a 1x2 point-vector and makes the
        % subsequent scalar-times-3D-force interval multiplication fail.
        mass=I(ml(1),ml(2));
        for n=0:2
            f{n+1}=sub(neg(mul(add(I(mu),mass),w{n+1})), ...
                mul(I(mu*L),q{n+3}));
            v{n+1}=sub(Iof(P{n+2}),mul(I(L),q{n+2}));
        end
        if ~(f{1}.lo(3)>minFz)
            M=I(zeros(3,1));return
        end
        h=cell(1,3);
        for n=1:3
            h{n}=I([v{n}.lo(1:2);0],[v{n}.hi(1:2);0]);
        end
        if planar_lower(h{1})<=minV
            M=I(zeros(3,1));return
        end
        h=unit_jet(h,2,B,fast_binomial);
        lateral=cell(1,3);
        for n=1:3
            lateral{n}=I([-h{n}.hi(2);h{n}.lo(1);0], ...
                           [-h{n}.lo(2);h{n}.hi(1);0]);
        end
        b3=unit_jet(f,2,B,fast_binomial);
        raw=jet_cross(lateral,b3,2,B,fast_binomial);
        b1=unit_jet(raw,2,B,fast_binomial);
        b2=jet_cross(b3,b1,2,B,fast_binomial);
        % All-time heading-exact Euler yaw requires positive F_z.
        % If the attitude construction has a singular denominator, the
        % interval normalizations above throw and this leaf is UNKNOWN.
        omega=I(zeros(3,1));alpha=I(zeros(3,1));
        omega=join_components( ...
            dotI(b3{1},b2{2}), ...
            dotI(b1{1},b3{2}), ...
            dotI(b2{1},b1{2}));
        alpha=join_components( ...
            add(dotI(b3{2},b2{2}),dotI(b3{1},b2{3})), ...
            add(dotI(b1{2},b3{2}),dotI(b1{1},b3{3})), ...
            add(dotI(b2{2},b1{2}),dotI(b2{1},b1{3})));
        if factor_gyro
            % Exact identity in real arithmetic for positive diagonal J.
            % Enclose each common angular-velocity product only once.
            % When two inertias agree the relevant gyro component vanishes
            % EXACTLY, instead of adding two unnecessarily wide intervals.
            gy=diagonal_gyro_interval(omega,J);
        else
            gy=crossI(omega,mul(I(J),omega));
        end
        M=add(mul(I(J),alpha),gy);
        valid=all(isfinite([M.lo;M.hi]));
    end
end

function Z=I(a,b)
if nargin<2,b=a;end
Z=struct('lo',double(a),'hi',double(b));
end
function Z=Iof(A)
Z=I(min(A,[],2),max(A,[],2));
end
function Z=add(A,B)
Z=I(A.lo+B.lo,A.hi+B.hi);
end
function Z=neg(A)
Z=I(-A.hi,-A.lo);
end
function Z=sub(A,B)
Z=add(A,neg(B));
end
function Z=mul(A,B)
[a,b]=compatible(A,B);
x=[a.lo.*b.lo,a.lo.*b.hi,a.hi.*b.lo,a.hi.*b.hi];
Z=I(min(x,[],2),max(x,[],2));
end
function [A,B]=compatible(A,B)
if isscalar(A.lo)&&~isscalar(B.lo)
    A=I(repmat(A.lo,size(B.lo)),repmat(A.hi,size(B.hi)));
elseif isscalar(B.lo)&&~isscalar(A.lo)
    B=I(repmat(B.lo,size(A.lo)),repmat(B.hi,size(A.hi)));
end
assert(isequal(size(A.lo),size(B.lo)),'C9_TORQUE:IntervalShape','Interval size mismatch');
end
function Z=div(A,B)
assert(all(B.lo(:)>0)||all(B.hi(:)<0), ...
    'C9_TORQUE:Singular','Interval division through zero');
Z=mul(A,I(1./B.hi,1./B.lo));
end
function Z=dotI(A,B)
[A,B]=compatible(A,B);
Z=I(0);
for i=1:numel(A.lo)
 Z=add(Z,mul(I(A.lo(i),A.hi(i)),I(B.lo(i),B.hi(i))));
end
end
function Z=crossI(A,B)
Z=I(zeros(3,1));
ix=[2 3;3 1;1 2];
for k=1:3
    a=ix(k,1);b=ix(k,2);
    p=mul(I(A.lo(a),A.hi(a)),I(B.lo(b),B.hi(b)));
    q=mul(I(A.lo(b),A.hi(b)),I(B.lo(a),B.hi(a)));
    d=sub(p,q);Z.lo(k)=d.lo;Z.hi(k)=d.hi;
end
end
function gy=diagonal_gyro_interval(omega,J)
% Interval extension of the exact Euler rigid-body gyroscopic products.
% Each scalar multiply encloses all values of its two angular intervals.
% It does not assume signs/correlations, and introduces no approximation.
wy=I(omega.lo(2),omega.hi(2));
wz=I(omega.lo(3),omega.hi(3));
wx=I(omega.lo(1),omega.hi(1));
g1=mul(I(J(3)-J(2)),mul(wy,wz));
g2=mul(I(J(1)-J(3)),mul(wz,wx));
g3=mul(I(J(2)-J(1)),mul(wx,wy));
gy=join_components(g1,g2,g3);
end
function Z=squareI(X)
a=X.lo;b=X.hi;
Z=I(min(a.^2,b.^2),max(a.^2,b.^2));
Z.lo(a<=0&b>=0)=0;
end
function Z=normsqI(X)
Z=I(0);
for i=1:numel(X.lo)
    Z=add(Z,squareI(I(X.lo(i),X.hi(i))));
end
end
function Z=sqrtI(A)
assert(all(A.lo(:)>0),'C9_TORQUE:ZeroNorm','No lower norm bound');
Z=I(sqrt(A.lo),sqrt(A.hi));
end
function U=unit_jet(X,nmax,B,fast_binomial)
assert(numel(X)>nmax,'C9_TORQUE:JetOrder','Missing derivative jet');
s=cell(1,nmax+1);U=cell(1,nmax+1);
s{1}=sqrtI(normsqI(X{1}));
U{1}=div(X{1},s{1});
for n=1:nmax
    rhs=I(0);
    for k=0:n
        if fast_binomial,c=B(n+1,k+1);else,c=nchoosek(n,k);end
        rhs=add(rhs,mul(I(c),dotI(X{k+1},X{n-k+1})));
    end
    for k=1:n-1
        if fast_binomial,c=B(n+1,k+1);else,c=nchoosek(n,k);end
        rhs=sub(rhs,mul(I(c),mul(s{k+1},s{n-k+1})));
    end
    s{n+1}=div(rhs,mul(I(2),s{1}));
    num=X{n+1};
    for k=1:n
        if fast_binomial,c=B(n+1,k+1);else,c=nchoosek(n,k);end
        num=sub(num,mul(I(c),mul(s{k+1},U{n-k+1})));
    end
    U{n+1}=div(num,s{1});
end
end
function V=jet_cross(A,B,nmax,C,fast_binomial)
V=cell(1,nmax+1);
for n=0:nmax
    v=I(zeros(3,1));
    for k=0:n
        if fast_binomial,c=C(n+1,k+1);else,c=nchoosek(n,k);end
        v=add(v,mul(I(c),crossI(A{k+1},B{n-k+1})));
    end
    V{n+1}=v;
end
end
function V=join_components(A,B,C)
V=I([A.lo;B.lo;C.lo],[A.hi;B.hi;C.hi]);
end
function n=planar_lower(V)
q=normsqI(I([V.lo(1:2);0],[V.hi(1:2);0]));
n=sqrt(q.lo);
end
function [L,R]=bisect(P)
% Do not repeat de Casteljau MATLAB loops for every torque proof leaf.
[L,R]=TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);
end
