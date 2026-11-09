function R=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN(piece,authority,options,times_s,U)
% Exact REAL forward-mode sensitivities of the SAME nominal point-torque map.
% Each vector contains a value (column 1) and its 12 whitened-control
% derivatives (columns 2:13). Both spatial and time jets are differentiated.
% This computes ALL 12 derivatives together, sharing Bernstein reductions.
% NO numeric torque certification, closed-loop HLC guarantee or COMMIT.
clock=tic;
ts=double(times_s(:).');
R=struct('status','UNKNOWN_FORWARD_TORQUE_NOT_EVALUATED', ...
    'valid',false(1,numel(ts)), ...
    'torque_Nm',NaN(3,numel(ts)), ...
    'gradient_Nm_per_whitened',NaN(3,12,numel(ts)), ...
    'heading_speed_mps',NaN(1,numel(ts)), ...
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
    assert(isnumeric(U)&&isreal(U)&&isequal(size(U),[4 4])&& ...
        all(isfinite(U(:)))&&rcond(U)>0, ...
        'C9_FORWARD_JAC:Whitening','Invalid live QP whitening');
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
    C=[1 0 0 0 0;1 1 0 0 0;1 2 1 0 0; ...
       1 3 3 1 0;1 4 6 4 1];
    % 39x24: first 3 rows are values, next 36 rows are the derivatives
    % for all 12 local QP variables. EXACTLY the old complex-step columns.
    Z=zeros(39,24);
    Z(1:3,:)=double(piece.control_points_m);
    E=eye(12);
    for j=1:12
        D=reshape(E(:,j),3,4)/U.';
        Z((3*j+1):(3*j+3),11:14)=D;
    end
    % Construct all 0..6 time derivative control polygons ONCE.
    Dpoly=cell(1,7);
    for k=0:6
        Dpoly{k+1}=Z;
        if k<6
            Z=(23-k)/piece.duration_s*diff(Z,1,2);
        end
    end
    for it=1:numel(ts)
        u=(ts(it)-piece.t0_s)/piece.duration_s;
        d=cell(1,7);
        for k=0:6
            Q=Dpoly{k+1};
            for col=size(Q,2):-1:2
                Q=(1-u)*Q(:,1:col-1)+u*Q(:,2:col);
            end
            d{k+1}=reshape(Q(:,1),3,13);
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
            lateral{k}=[-v{k}(2,:);v{k}(1,:);zeros(1,13)];
        end
        raw=jetcross_dual(lateral,F,2,C);
        if ~(sum(raw{1}(:,1).^2)>0),continue;end
        b1=unitjet_dual(raw,2,C);
        b3=unitjet_dual(F,2,C);
        b2=jetcross_dual(b3,b1,2,C);
        omega=zeros(3,13);alpha=zeros(3,13);
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
        R.gradient_Nm_per_whitened(:,:,it)=tau(:,2:end);
        R.valid(it)=true;
    end
    if all(R.valid)
        R.status='FORWARD_NOMINAL_TORQUE_JACOBIAN__NOT_A_CERTIFICATE';
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
s{1}=zeros(1,13);
s{1}(1)=sqrt(sum(X{1}(:,1).^2));
s{1}(2:end)=X{1}(:,1).'*X{1}(:,2:end)/s{1}(1);
U{1}=ddiv(X{1},s{1});
for n=1:nmax
    rhs=zeros(1,13);
    for k=0:n
        rhs=rhs+C(n+1,k+1)*ddot(X{k+1},X{n-k+1});
    end
    for k=1:n-1
        rhs=rhs-C(n+1,k+1)*dmul(s{k+1},s{n-k+1});
    end
    s{n+1}=zeros(1,13);
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
    acc=zeros(3,13);
    for k=0:n
        acc=acc+C(n+1,k+1)*dcross(A{k+1},B{n-k+1});
    end
    V{n+1}=acc;
end
end
