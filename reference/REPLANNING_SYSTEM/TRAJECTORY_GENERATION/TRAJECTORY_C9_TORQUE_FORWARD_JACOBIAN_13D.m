function R=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN_13D(piece,authority,options,phases,precomputed_duration)
% Analytic 13D forward differentiation of nominal torques at FIXED
% normalized phases.  Parameter order: [free(:);physical duration_s].
% Source is the existing exact 12D dual-Jet formulas. The 13th column
% propagates EXACT physical-duration derivatives of the Bernstein 0..6
% derivative polygons. It is not a numerical time step or an HLC proof.
% No minimum positive yaw-speed heuristic is introduced.
clock=tic;
uvals=double(phases(:).');
ts=piece.t0_s+piece.duration_s*uvals;
R=struct('status','UNKNOWN_FORWARD_TORQUE_NOT_EVALUATED', ...
    'valid',false(1,numel(ts)), ...
    'torque_Nm',NaN(3,numel(ts)), ...
    'gradient_Nm_per_physical',NaN(3,13,numel(ts)), ...
    'heading_speed_mps',NaN(1,numel(ts)), ...
    'duration_cache_reused',false,'jet_batch_enabled',false, ...
    'shared_jet_enabled',false,'native_torque_batch_requested',false, ...
    'native_torque_batch_executed',false,'native_torque_batch_ms',0, ...
    'elapsed_ms',NaN,'safe_to_commit',false, ...
    'scope','NOMINAL_POINT_LOCAL_JACOBIAN_ONLY__NOT_ALL_TIME_OR_ACTUAL_HLC');
try
    assert(isstruct(piece)&&isscalar(piece)&& ...
        all(isfield(piece,{'control_points_m','duration_s','t0_s'}))&& ...
        isequal(size(piece.control_points_m),[3 24])&& ...
        isnumeric(piece.control_points_m)&&isreal(piece.control_points_m)&& ...
        all(isfinite(piece.control_points_m(:)))&& ...
        isscalar(piece.t0_s)&&isreal(piece.t0_s)&&isfinite(piece.t0_s)&& ...
        isscalar(piece.duration_s)&&isreal(piece.duration_s)&& ...
        isfinite(piece.duration_s)&&piece.duration_s>0, ...
        'C9_FORWARD_JAC:Piece','Need real finite degree-23 C9 controls');
    assert(isnumeric(phases)&&isreal(phases)&&isvector(phases)&& ...
        all(isfinite(phases(:)))&&all(phases(:)>=0)&&all(phases(:)<=1), ...
        'C9_FORWARD_JAC_13D:Phases','Finite phases in [0,1] required.');
    assert(isstruct(authority)&&isscalar(authority)&& ...
        all(isfield(authority,{'uav_mass_kg','payload_mass_kg', ...
        'cable_length_m','uav_inertia_kgm2','gravity_world_mps2'})), ...
        'C9_FORWARD_JAC:Authority','Live authority required');
    assert(isstruct(options)&&isscalar(options)&& ...
        all(isfield(options,{'minimum_force_z_N', ...
        'minimum_planar_speed_mps'})), ...
        'C9_FORWARD_JAC:Options','Current point guards required');
    mu=double(authority.uav_mass_kg);
    ml=double(authority.payload_mass_kg);
    L=double(authority.cable_length_m);
    g=double(authority.gravity_world_mps2(:));
    J=double(authority.uav_inertia_kgm2(:));
    fzmin=double(options.minimum_force_z_N);
    hmin=double(options.minimum_planar_speed_mps);
    assert(isscalar(mu)&&isscalar(ml)&&isscalar(L)&& ...
        numel(g)==3&&numel(J)==3&&all(J>0)&& ...
        all(isfinite([mu;ml;L;g;J;fzmin;hmin]))&& ...
        mu>0&&ml>=0&&L>0&&fzmin>=0&&hmin>=0&& ...
        isreal(ts)&&all(isfinite(ts))&& ...
        all(ts>=piece.t0_s)&&all(ts<=piece.t0_s+piece.duration_s), ...
        'C9_FORWARD_JAC:Domain','Invalid physical values or time');
    % Default enabled after numerical parity + real MATLAB timing A/B.
    % Explicit 0 still selects the original scalar evaluator.
    batch=true;
    if isfield(options,'use_batched_13d_torque_jets')
        v=options.use_batched_13d_torque_jets;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_FORWARD_JAC:BatchFlag','Batch flag must be 0/1.');
        batch=logical(v);
    end
    R.jet_batch_enabled=batch;
    C=[1 0 0 0 0;1 1 0 0 0;1 2 1 0 0; ...
       1 3 3 1 0;1 4 6 4 1];
    % 42x24: physical values, 36 raw free-control sensitivities,
    % then THREE exact physical-duration sensitivities.
    if nargin==5
        assert(TRAJECTORY_C9_DURATION_CACHE_MATCH(precomputed_duration, ...
            piece.t0_s,piece.duration_s,piece.start_jets,piece.end_jets, ...
            piece.control_points_m(:,11:14),uvals), ...
            'C9_FORWARD_JAC:StaleDurationCache', ...
            'Precomputed derivative data belong to a different C9 candidate.');
        assert(isequal(precomputed_duration.piece.control_points_m, ...
            piece.control_points_m), ...
            'C9_FORWARD_JAC:StaleDurationCache', ...
            'Source polynomial differs from validated cache.');
        S=precomputed_duration;
        R.duration_cache_reused=true;
    else
        S=TRAJECTORY_C9_DURATION_SENSITIVITY( ...
            piece.t0_s,piece.duration_s,piece.start_jets, ...
            piece.end_jets,piece.control_points_m(:,11:14));
    end
    use_shared=false;
    if isfield(options,'use_13d_shared_jet_kernel')
        flag=options.use_13d_shared_jet_kernel;
        assert((isnumeric(flag)||islogical(flag))&&isscalar(flag)&& ...
            isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
            'C9_FORWARD_JAC:SharedJetFlag','Shared-jet mode must be 0/1.');
        use_shared=logical(flag);
    end
    if use_shared
        assert(nargin==5&&R.duration_cache_reused, ...
            'C9_FORWARD_JAC:MissingSharedCache', ...
            'Shared jets require the validated SAME-QP duration snapshot.');
        jet_values=TRAJECTORY_C9_13D_SHARED_JET_KERNEL(S, ...
            piece.duration_s,uvals);
        batch=true;
        R.jet_batch_enabled=true;
        R.shared_jet_enabled=true;
    else
    Z=zeros(42,24);
    Z(1:3,:)=double(piece.control_points_m);
    E=eye(12);
    for j=1:12
        D=reshape(E(:,j),3,4);
        Z((3*j+1):(3*j+3),11:14)=D;
    end
    % Duration column includes BOTH dP/dT and the derivative of
    % (23)_k/T^k in every physical time derivative polygon.
    Dpoly=cell(1,7);
    for k=0:6
        Z(40:42,:)=S.dphysical_derivative_controls_dT{k+1};
        Dpoly{k+1}=Z;
        if k<6
            Z=(23-k)/piece.duration_s*diff(Z,1,2);
        end
    end
    if batch
        % One Bernstein matrix per derivative order and ALL query phases.
        % Exactly the same polynomial, expressed in the Bernstein basis;
        % this is NOT a time-sample safety or torque certificate.
        jet_values=cell(1,7);
        for k=0:6
            W=local_bernstein_basis_matrix(23-k,uvals);
            jet_values{k+1}=Dpoly{k+1}*W;
        end
    end
    end % legacy 42x24 polynomial dual jet path
    % Optional single native 13D dual-Jet evaluation for ALL search phases.
    % Local QP search gradients ONLY; never use it to certify all-time
    % torque, tracking or actuator safety. No pre-Flight build in this call.
    native_on=false;
    if isfield(options,'use_native_13d_torque_batch_kernel')
        flag=options.use_native_13d_torque_batch_kernel;
        assert((isnumeric(flag)||islogical(flag))&&isscalar(flag)&& ...
            isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
            'C9_FORWARD_JAC_13D:NativeFlag','Native batch switch must be 0/1.');
        native_on=logical(flag);
    end
    R.native_torque_batch_requested=native_on;
    if native_on
        % The native kernel requires the same seven 42xN jet blocks as
        % the existing batched MATLAB evaluator. No local time samples
        % can be promoted to continuous-time proof by this switch.
        if ~batch
            R.status='UNKNOWN_NATIVE_13D_REQUIRES_BATCHED_JETS';
            R.elapsed_ms=1000*toc(clock);return
        end
        folder=fileparts(mfilename('fullpath'));
        expected=fullfile(folder, ...
            ['TRAJECTORY_C9_13D_TORQUE_BATCH_NATIVE.' mexext]);
        actual=which('TRAJECTORY_C9_13D_TORQUE_BATCH_NATIVE');
        if exist(expected,'file')~=3 || ~strcmpi(actual,expected) || ...
                ~strcmp(TRAJECTORY_C9_13D_TORQUE_BATCH_NATIVE(), ...
                'C9_13D_TORQUE_BATCH_NATIVE_20261110_V1')
            R.status='UNKNOWN_NATIVE_13D_NOT_COMPILED_OR_PINNED';
            R.elapsed_ms=1000*toc(clock);return
        end
        native_clock=tic;
        [tmat,jmat,vmask,hsp]=TRAJECTORY_C9_13D_TORQUE_BATCH_NATIVE( ...
            jet_values,g,[mu;ml;L],J,[fzmin;hmin]);
        R.native_torque_batch_ms=1000*toc(native_clock);
        assert(isequal(size(tmat),[3,numel(ts)])&& ...
            isequal(size(jmat),[3,13,numel(ts)])&& ...
            islogical(vmask)&&numel(vmask)==numel(ts)&& ...
            numel(hsp)==numel(ts), ...
            'C9_FORWARD_JAC_13D:NativeOutput','Native batch shape mismatch.');
        R.torque_Nm=tmat;
        R.gradient_Nm_per_physical=jmat;
        R.valid=reshape(vmask,1,[]);
        R.heading_speed_mps=reshape(hsp,1,[]);
        R.native_torque_batch_executed=true;
        if all(R.valid)
            R.status='FORWARD_NOMINAL_TORQUE_13D_JACOBIAN__NOT_A_CERTIFICATE';
        else
            R.status='UNKNOWN_FORWARD_SINGULAR_OR_NONFINITE';
        end
        R.elapsed_ms=1000*toc(clock);return
    end
    for it=1:numel(ts)
        u=uvals(it);
        d=cell(1,7);
        for k=0:6
            if batch
                d{k+1}=reshape(jet_values{k+1}(:,it),3,14);
            else
                Q=Dpoly{k+1};
                for col=size(Q,2):-1:2
                    Q=(1-u)*Q(:,1:col-1)+u*Q(:,2:col);
                end
                d{k+1}=reshape(Q(:,1),3,14);
            end
        end
        w=cell(1,5);
        w{1}=-d{3};w{1}(:,1)=g-d{3}(:,1);
        for k=1:4
            w{k+1}=-d{k+3};
        end
        if ~(sum(w{1}(:,1).^2)>0),continue;end
        q=unitjet_dual(w,4,C);
        F=cell(1,3);v=cell(1,3);
        for k=0:2
            F{k+1}=-(mu+ml)*w{k+1}-mu*L*q{k+3};
            v{k+1}=d{k+2}-L*q{k+2};
        end
        speed=sqrt(sum(v{1}(1:2,1).^2));
        R.heading_speed_mps(it)=speed;
        if ~(F{1}(3,1)>fzmin&&speed>hmin&&speed>0&& ...
             sum(F{1}(:,1).^2)>0),continue;end
        lateral=cell(1,3);
        for k=1:3
            lateral{k}=[-v{k}(2,:);v{k}(1,:);zeros(1,14)];
        end
        raw=jetcross_dual(lateral,F,2,C);
        if ~(sum(raw{1}(:,1).^2)>0),continue;end
        b1=unitjet_dual(raw,2,C);
        b3=unitjet_dual(F,2,C);
        b2=jetcross_dual(b3,b1,2,C);
        omega=zeros(3,14);alpha=zeros(3,14);
        omega(1,:)=ddot(b3{1},b2{2});
        omega(2,:)=ddot(b1{1},b3{2});
        omega(3,:)=ddot(b2{1},b1{2});
        alpha(1,:)=ddot(b3{2},b2{2})+ddot(b3{1},b2{3});
        alpha(2,:)=ddot(b1{2},b3{2})+ddot(b1{1},b3{3});
        alpha(3,:)=ddot(b2{2},b1{2})+ddot(b2{1},b1{3});
        gyro=[(J(3)-J(2))*dmul(omega(2,:),omega(3,:)); ...
              (J(1)-J(3))*dmul(omega(3,:),omega(1,:)); ...
              (J(2)-J(1))*dmul(omega(1,:),omega(2,:))];
        tau=J.*alpha+gyro;
        if ~all(isfinite(tau(:))),continue;end
        R.torque_Nm(:,it)=tau(:,1);
        R.gradient_Nm_per_physical(:,:,it)=tau(:,2:end);
        R.valid(it)=true;
    end
    if all(R.valid)
        R.status='FORWARD_NOMINAL_TORQUE_13D_JACOBIAN__NOT_A_CERTIFICATE';
    else
        R.status='UNKNOWN_FORWARD_SINGULAR_OR_NONFINITE';
    end
catch ME
    R.status=['UNKNOWN_FORWARD_JACOBIAN_' ME.identifier];
end
R.elapsed_ms=1000*toc(clock);
end

function y=dmul(a,b)
y=[a(1)*b(1),a(2:end)*b(1)+a(1)*b(2:end)];
end
function y=ddot(A,B)
y=[A(:,1).'*B(:,1), A(:,1).'*B(:,2:end)+B(:,1).'*A(:,2:end)];
end
function Y=dscale(s,X)
Y=[s(1)*X(:,1),s(1)*X(:,2:end)+X(:,1)*s(2:end)];
end
function Y=ddiv(X,s)
Y=zeros(size(X));Y(:,1)=X(:,1)/s(1);
Y(:,2:end)=(X(:,2:end)-Y(:,1)*s(2:end))/s(1);
end
function Y=dcross(A,B)
Y=zeros(size(A));Y(:,1)=cross(A(:,1),B(:,1));
Y(1,2:end)=A(2,2:end)*B(3,1)+A(2,1)*B(3,2:end) ...
          -A(3,2:end)*B(2,1)-A(3,1)*B(2,2:end);
Y(2,2:end)=A(3,2:end)*B(1,1)+A(3,1)*B(1,2:end) ...
          -A(1,2:end)*B(3,1)-A(1,1)*B(3,2:end);
Y(3,2:end)=A(1,2:end)*B(2,1)+A(1,1)*B(2,2:end) ...
          -A(2,2:end)*B(1,1)-A(2,1)*B(1,2:end);
end
function U=unitjet_dual(X,nmax,C)
s=cell(1,nmax+1);U=cell(1,nmax+1);
s{1}=zeros(1,14);
s{1}(1)=sqrt(sum(X{1}(:,1).^2));
s{1}(2:end)=X{1}(:,1).'*X{1}(:,2:end)/s{1}(1);
U{1}=ddiv(X{1},s{1});
for n=1:nmax
    rhs=zeros(1,14);
    for k=0:n
        rhs=rhs+C(n+1,k+1)*ddot(X{k+1},X{n-k+1});
    end
    for k=1:n-1
        rhs=rhs-C(n+1,k+1)*dmul(s{k+1},s{n-k+1});
    end
    s{n+1}=zeros(1,14);
    s{n+1}(1)=rhs(1)/(2*s{1}(1));
    s{n+1}(2:end)=(rhs(2:end)-2*s{1}(2:end)*s{n+1}(1)) ...
        /(2*s{1}(1));
    num=X{n+1};
    for k=1:n
        num=num-C(n+1,k+1)*dscale(s{k+1},U{n-k+1});
    end
    U{n+1}=ddiv(num,s{1});
end
end
function V=jetcross_dual(A,B,nmax,C)
V=cell(1,nmax+1);
for n=0:nmax
    acc=zeros(3,14);
    for k=0:n
        acc=acc+C(n+1,k+1)*dcross(A{k+1},B{n-k+1});
    end
    V{n+1}=acc;
end
end

function B=local_bernstein_basis_matrix(n,u)
% Stable small-degree (n<=23) recurrence; handles u=0 and u=1 exactly.
% B is (n+1)xN, so ALL 42 dual channels use the same basis.
N=numel(u);B=zeros(n+1,N);
for it=1:N
    x=u(it);
    if x==0
        B(1,it)=1;
    elseif x==1
        B(n+1,it)=1;
    elseif x<0 || x>1 || ~isfinite(x)
        error('C9_FORWARD_JAC:BatchPhase','Phase must belong to [0,1].');
    else
        % Start from the larger of x and (1-x), to keep the recurrence
        % ratios bounded near the endpoint most susceptible to underflow.
        if x<=0.5
            B(1,it)=(1-x)^n;
            for j=1:n
                B(j+1,it)=B(j,it)*(n-j+1)/j*x/(1-x);
            end
        else
            B(n+1,it)=x^n;
            for j=n:-1:1
                B(j,it)=B(j+1,it)*j/(n-j+1)*(1-x)/x;
            end
        end
    end
end
end
