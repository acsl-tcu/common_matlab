function R=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN_25D(piece,authority,options,phase)
% Exact 25-variable forward dual-Jet nominal torque at fixed phases.
% One degree-27 C9 piece: 24 free interior coefficients and duration T.
% Uses the existing nominal rigid-body formula. This is a SEARCH tangent;
% all-time torque, actual HLC and physical safety are NOT proved here.
clock=tic;
uvals=double(phase(:).');N=numel(uvals);
R=struct('status','UNKNOWN_C9_25D_TORQUE_LOCAL', ...
    'safe_to_commit',false,'valid',false(1,N), ...
    'torque_Nm',NaN(3,N),'gradient_Nm_per_physical',NaN(3,25,N), ...
    'heading_speed_mps',NaN(1,N),'elapsed_ms',NaN, ...
    'native_requested',false,'native_executed',false, ...
    'native_kernel_ms',0);
try
    assert(isstruct(piece)&&isscalar(piece)&& ...
        isfield(piece,'control_points_m')&& ...
        isequal(size(piece.control_points_m),[3 28])&& ...
        all(isfinite(piece.control_points_m(:)))&& ...
        isfield(piece,'start_jets')&&isfield(piece,'end_jets')&& ...
        isfinite(piece.duration_s)&&piece.duration_s>0&& ...
        all(isfinite(uvals))&&all(uvals>=0)&&all(uvals<=1), ...
        'C9_25D_TORQUE:Piece','Need finite degree27 C9 piece and phases.');
    assert(isstruct(authority)&&isscalar(authority)&& ...
        all(isfield(authority,{'uav_mass_kg','payload_mass_kg', ...
        'cable_length_m','uav_inertia_kgm2','gravity_world_mps2'}))&& ...
        isstruct(options)&&isscalar(options)&& ...
        all(isfield(options,{'minimum_force_z_N','minimum_planar_speed_mps'})), ...
        'C9_25D_TORQUE:Authority','Live physical authority and guards required.');
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
        mu>0&&ml>=0&&L>0&&fzmin>=0&&hmin>=0, ...
        'C9_25D_TORQUE:Physics','Invalid external physical snapshot.');
    [P,dP]=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS( ...
        piece.start_jets,piece.end_jets, ...
        piece.control_points_m(:,11:18),piece.duration_s);
    assert(isequal(P,piece.control_points_m), ...
        'C9_25D_TORQUE:Identity','Derivative must bind to actual curve.');
    % 3 nominal + 24*3 free + 3 duration = 78 dual channels.
    Z=zeros(78,28);Z(1:3,:)=P;
    for j=1:8
        for axis=1:3
            channel=3*(j-1)+axis; % MATLAB vec(X) column ordering
            rows=3+3*(channel-1)+(1:3);
            e=zeros(3,1);e(axis)=1;
            Z(rows,10+j)=e;
        end
    end
    Z(76:78,:)=dP;
    T=piece.duration_s;
    jet_values=cell(1,7);
    for k=0:6
        weights=local_basis_matrix(27-k,uvals);
        jet_values{k+1}=Z*weights;
        if k<6
            Z=(27-k)/T*diff(Z,1,2);
            Z(76:78,:)=Z(76:78,:)-Z(1:3,:)/T;
        end
    end
    C=[1 0 0 0 0;1 1 0 0 0;1 2 1 0 0; ...
       1 3 3 1 0;1 4 6 4 1];
    % One explicit preflight-pinned native call for ALL 25D torque local
    % sensitivities. The seven polynomial jets are identical to the
    % MATLAB path; neither path is an all-time torque certificate.
    native_on=false;
    if isfield(options,'use_native_25d_torque_batch_kernel')
        flag=options.use_native_25d_torque_batch_kernel;
        assert((islogical(flag)||isnumeric(flag))&&isscalar(flag)&& ...
            isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
            'C9_25D_TORQUE:NativeFlag','Expected explicit 0/1 switch.');
        native_on=logical(flag);
    end
    R.native_requested=native_on;
    if native_on
        dirhere=fileparts(mfilename('fullpath'));
        name='TRAJECTORY_C9_25D_TORQUE_BATCH_NATIVE';
        expected=fullfile(dirhere,[name '.' mexext]);
        if exist(expected,'file')~=3 || ~strcmpi(which(name),expected) || ...
                ~strcmp(TRAJECTORY_C9_25D_TORQUE_BATCH_NATIVE(), ...
                  'C9_25D_TORQUE_BATCH_NATIVE_20261111_V1')
            R.status='UNKNOWN_NATIVE_25D_NOT_COMPILED_OR_PINNED';
            R.elapsed_ms=1000*toc(clock);return
        end
        tnative=tic;
        [tn,jn,validn,hn]=TRAJECTORY_C9_25D_TORQUE_BATCH_NATIVE( ...
            jet_values,g,[mu;ml;L],J,[fzmin;hmin]);
        R.native_kernel_ms=1000*toc(tnative);
        assert(size(tn,1)==3&&size(tn,2)==N&& ...
            size(jn,1)==3&&size(jn,2)==25&&size(jn,3)==N&& ...
            islogical(validn)&&numel(validn)==N&&numel(hn)==N, ...
            'C9_25D_TORQUE:NativeOutput','MEX output dimensions invalid.');
        R.torque_Nm=tn;
        R.gradient_Nm_per_physical=jn;
        R.valid=reshape(validn,1,N);
        R.heading_speed_mps=reshape(hn,1,N);
        R.native_executed=true;
        if all(R.valid)
            R.status='VALID_LOCAL_DEGREE27_C9_25D_NATIVE_TORQUE_TANGENT__NO_CERTIFICATE';
        else
            R.status='UNKNOWN_C9_25D_NATIVE_TORQUE_SINGULAR_POINT';
        end
        R.elapsed_ms=1000*toc(clock);return
    end
    for it=1:N
        d=cell(1,7);
        for k=0:6
            d{k+1}=reshape(jet_values{k+1}(:,it),3,26);
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
            lateral{k}=[-v{k}(2,:);v{k}(1,:);zeros(1,26)];
        end
        raw=jetcross_dual(lateral,F,2,C);
        if ~(sum(raw{1}(:,1).^2)>0),continue;end
        b1=unitjet_dual(raw,2,C);
        b3=unitjet_dual(F,2,C);
        b2=jetcross_dual(b3,b1,2,C);
        omega=zeros(3,26);alpha=zeros(3,26);
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
        R.status='VALID_LOCAL_DEGREE27_C9_25D_TORQUE_TANGENT__NO_CERTIFICATE';
    else
        R.status='UNKNOWN_C9_25D_TORQUE_SINGULAR_POINT';
    end
catch ME
    R.status=['UNKNOWN_C9_25D_TORQUE_JACOBIAN_' ME.identifier];
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
s{1}=zeros(1,26);
s{1}(1)=sqrt(sum(X{1}(:,1).^2));
s{1}(2:end)=X{1}(:,1).'*X{1}(:,2:end)/s{1}(1);
U{1}=ddiv(X{1},s{1});
for n=1:nmax
    rhs=zeros(1,26);
    for k=0:n
        rhs=rhs+C(n+1,k+1)*ddot(X{k+1},X{n-k+1});
    end
    for k=1:n-1
        rhs=rhs-C(n+1,k+1)*dmul(s{k+1},s{n-k+1});
    end
    s{n+1}=zeros(1,26);
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
    acc=zeros(3,26);
    for k=0:n
        acc=acc+C(n+1,k+1)*dcross(A{k+1},B{n-k+1});
    end
    V{n+1}=acc;
end
end

function B=local_basis_matrix(n,u)
N=numel(u);B=zeros(n+1,N);
for it=1:N
    x=u(it);
    if x==0,B(1,it)=1;
    elseif x==1,B(n+1,it)=1;
    elseif x<=0.5
        B(1,it)=(1-x)^n;
        for k=1:n,B(k+1,it)=B(k,it)*(n-k+1)*x/(k*(1-x));end
    else
        B(n+1,it)=x^n;
        for k=n:-1:1,B(k,it)=B(k+1,it)*k*(1-x)/((n-k+1)*x);end
    end
end
end
