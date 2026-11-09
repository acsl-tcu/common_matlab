function S=TRAJECTORY_C9_TORQUE_POINT_EVALUATE(piece,authority,options,times_s)
% Pointwise nominal *search* values for the exact degree-23 C9 load polynomial.
% NOT a continuous-time certificate, a numerical-exclusion proof, an HLC
% command, or authorization to COMMIT. In particular this model uses ONE
% snapshot of the estimator's payload mass, not an unknown future mL(t).
%
% Internally the evaluator permits complex Bernstein free controls so that
% the joint QP can obtain real-analytic sensitivities by complex-step.
% All dot products below are BILINEAR (no conjugate), and norm=sqrt(x.'*x).
% No spatial direction, obstacle geometry, body radius, or future time
% is fixed in this point evaluator. The caller supplies its search times.
clock=tic;
S=struct('status','UNKNOWN_POINT_TORQUE_NOT_EVALUATED', ...
 'valid',false(1,numel(times_s)),'torque_Nm',NaN(3,numel(times_s)), ...
 'heading_speed_mps',NaN(1,numel(times_s)), ...
 'force_z_N',NaN(1,numel(times_s)), ...
 'angular_velocity_body_radps',NaN(3,numel(times_s)), ...
 'angular_accel_body_radps2',NaN(3,numel(times_s)), ...
 'inertial_torque_Nm',NaN(3,numel(times_s)), ...
 'gyroscopic_torque_Nm',NaN(3,numel(times_s)), ...
 'time_s',double(times_s(:).'), 'outside_limits',false(1,numel(times_s)), ...
 'maximum_point_violation_Nm',NaN,'elapsed_ms',NaN, ...
 'safe_to_commit',false, ...
 'scope','POINTWISE_NOMINAL_SNAPSHOT_SEARCH_ONLY__NOT_ALL_TIME_OR_HLC');
try
 assert(isstruct(piece)&&isscalar(piece)&& ...
   all(isfield(piece,{'control_points_m','duration_s','t0_s'}))&& ...
   isequal(size(piece.control_points_m),[3 24])&& ...
   isnumeric(piece.control_points_m)&& ...
   all(isfinite(piece.control_points_m(:)))&& ...
   isreal(piece.duration_s)&&isfinite(piece.duration_s)&& ...
   piece.duration_s>0&&isfinite(piece.t0_s+piece.duration_s)&& ...
   isscalar(piece.t0_s)&&isfinite(piece.t0_s), ...
   'C9_POINT_TORQUE:Polynomial','Need the actual finite C9 control polygon');
 assert(isstruct(authority)&&isscalar(authority)&& ...
   all(isfield(authority,{'uav_mass_kg','payload_mass_kg', ...
   'cable_length_m','uav_inertia_kgm2','gravity_world_mps2', ...
   'torque_limits_Nm'})), 'C9_POINT_TORQUE:Authority', ...
   'Need live input and physical authority');
 assert(isstruct(options)&&isscalar(options)&& ...
   all(isfield(options,{'minimum_force_z_N', ...
   'minimum_planar_speed_mps','torque_guard_Nm'})), ...
   'C9_POINT_TORQUE:Options','Require caller-owned guards');
 mu=double(authority.uav_mass_kg);ml=double(authority.payload_mass_kg);
 L=double(authority.cable_length_m);
 g=double(authority.gravity_world_mps2(:));
 J=double(authority.uav_inertia_kgm2(:));
 limits=double(authority.torque_limits_Nm);
 guard=double(options.torque_guard_Nm(:));
 fz_min=double(options.minimum_force_z_N);
 h_min=double(options.minimum_planar_speed_mps);
 ts=double(times_s(:).');
 assert(all(isfinite([mu;ml;L;g;J;limits(:);guard;fz_min;h_min])) && ...
   mu>0&&ml>=0&&L>0&&numel(g)==3&&numel(J)==3&&all(J>0)&& ...
   isequal(size(limits),[3 2])&&all(limits(:,2)>limits(:,1))&& ...
   numel(guard)==3&&all(guard>=0)&&fz_min>=0&&h_min>=0&& ...
   isreal(ts)&&all(isfinite(ts))&& ...
   all(ts>=piece.t0_s)&&all(ts<=piece.t0_s+piece.duration_s), ...
   'C9_POINT_TORQUE:Domain','Invalid authority, limits, or probe times');
 P=double(piece.control_points_m);
 % PERFORMANCE-ONLY OPTION.  The degree-23 derivative control polygons
 % depend on (piece,T), NOT on a probe time.  Computing them once for all
 % probes preserves the original Bernstein/de Casteljau evaluation and its
 % physical equations. Keep an A/B switch until real MATLAB measurements.
 precompute_derivatives=false;
 if isfield(options,'use_precomputed_point_derivative_polygons')
     flag=options.use_precomputed_point_derivative_polygons;
     assert((islogical(flag)||isnumeric(flag))&&isscalar(flag)&& ...
         isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
         'C9_POINT_TORQUE:PrecomputeFlag','The performance switch must be 0/1');
     precompute_derivatives=logical(flag);
 end
 % Independent algorithmic alternative to repeated de Casteljau.
 % Uses exact Bernstein coefficients (n<=23) and evaluates every requested
 % search time in one matrix multiplication per derivative.  This is only
 % a NUMERICAL POINT PROBE, never a continuous-time interval certificate.
 % Default ON for multi-time probes after paired MATLAB A/B parity and timing.
 % Explicit caller 0/1 switch always overrides this search-only default.
 batch_bernstein=(numel(ts)>1); % proven point-probe acceleration; single probe unchanged
 if isfield(options,'use_vectorized_bernstein_point_jets')
     flag=options.use_vectorized_bernstein_point_jets;
     assert((islogical(flag)||isnumeric(flag))&&isscalar(flag)&& ...
         isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
         'C9_POINT_TORQUE:BatchFlag', 'Batch point option must be 0/1');
     batch_bernstein=logical(flag);
 end
 polys=cell(1,7);
 if precompute_derivatives || batch_bernstein
     polys{1}=P;
     for kk=1:6
         polys{kk+1}=(24-kk)/piece.duration_s*diff(polys{kk},1,2);
     end
 end
 bulk=cell(1,7);
 if batch_bernstein
     x=(ts(:)-piece.t0_s)/piece.duration_s;
     % Exact integer Pascal coefficients (all <= 23 choose 11 < 2^53).
     pascal=cell(1,24);pascal{1}=1;
     for degree=1:23
         prev=pascal{degree};
         pascal{degree+1}=[1,prev(1:end-1)+prev(2:end),1];
     end
     for kk=0:6
         degree=23-kk;
         jj=0:degree;
         weights=(x.^jj).*((1-x).^(degree-jj));
         weights=weights.*pascal{degree+1};
         bulk{kk+1}=polys{kk+1}*weights.';
     end
 end
 C=[1 0 0 0 0;1 1 0 0 0;1 2 1 0 0; ...
    1 3 3 1 0;1 4 6 4 1];
 for it=1:numel(ts)
   u=(ts(it)-piece.t0_s)/piece.duration_s;
   % Exactly the same 0..6 derivative control polygons and de Casteljau
   % recursion in both modes. No point sample is ever a safety certificate.
   d=cell(1,7);
   if batch_bernstein
       for k=0:6,d{k+1}=bulk{k+1}(:,it);end
   else
       poly=P;
       for k=0:6
         if precompute_derivatives
             Q=polys{k+1};
         else
             Q=poly;
         end
         for col=size(Q,2):-1:2
           Q=(1-u)*Q(:,1:col-1)+u*Q(:,2:col);
         end
         d{k+1}=Q(:,1);
         if ~precompute_derivatives && k<6
             poly=(23-k)/piece.duration_s*diff(poly,1,2);
         end
       end
   end
   w=cell(1,5);w{1}=g-d{3};
   for k=1:4,w{k+1}=-d{k+3};end
   if ~(real(sum(w{1}.^2))>0),continue;end
   q=unitjet(w,4,C);
   F=cell(1,3);v=cell(1,3);
   for k=0:2
     F{k+1}=-(mu+ml)*w{k+1}-mu*L*q{k+3};
     v{k+1}=d{k+2}-L*q{k+2};
   end
   speed=sqrt(sum(v{1}(1:2).^2));
   S.heading_speed_mps(it)=real(speed);
   S.force_z_N(it)=real(F{1}(3));
   if ~(real(F{1}(3))>fz_min&&real(speed)>h_min&& ...
           real(speed)>0&&real(sum(F{1}.^2))>0)
       continue
   end
   lateral=cell(1,3);
   for k=1:3
      lateral{k}=[-v{k}(2);v{k}(1);0];
   end
   raw=jetcross(lateral,F,2,C);
   if ~(real(sum(raw{1}.^2))>0),continue;end
   b1=unitjet(raw,2,C);b3=unitjet(F,2,C);
   b2=jetcross(b3,b1,2,C);
   % R'*R' and R'*R'' are expressed as ordinary bilinear products;
   % conjugating a complex step would destroy the analytic Jacobian.
   omega=[sum(b3{1}.*b2{2}); ...
          sum(b1{1}.*b3{2}); ...
          sum(b2{1}.*b1{2})];
   alpha=[sum(b3{2}.*b2{2})+sum(b3{1}.*b2{3}); ...
          sum(b1{2}.*b3{2})+sum(b1{1}.*b3{3}); ...
          sum(b2{2}.*b1{2})+sum(b2{1}.*b1{3})];
   gyro=[(J(3)-J(2))*omega(2)*omega(3); ...
         (J(1)-J(3))*omega(3)*omega(1); ...
         (J(2)-J(1))*omega(1)*omega(2)];
   inertial=J.*alpha;
   tau=inertial+gyro;
   if all(isfinite(tau))
       S.valid(it)=true;
       S.torque_Nm(:,it)=tau;
       S.angular_velocity_body_radps(:,it)=omega;
       S.angular_accel_body_radps2(:,it)=alpha;
       S.inertial_torque_Nm(:,it)=inertial;
       S.gyroscopic_torque_Nm(:,it)=gyro;
       S.outside_limits(it)=any(real(tau)>limits(:,2)-guard)|| ...
                            any(real(tau)<limits(:,1)+guard);
   end
 end
 if all(S.valid)
     S.status='VALID_NUMERIC_POINT_PROBES__NOT_A_CERTIFICATE';
 else
     S.status='UNKNOWN_SOME_POINT_PROBES_SINGULAR_OR_NONFINITE';
 end
 if any(S.valid)
     t=real(S.torque_Nm(:,S.valid));
     excess=max([t-(limits(:,2)-guard), ...
          (limits(:,1)+guard)-t],[],'all');
     S.maximum_point_violation_Nm=max(0,excess);
 end
catch ME
 S.status=['UNKNOWN_POINT_EVALUATOR_' ME.identifier];
end
S.elapsed_ms=1000*toc(clock);
end

function U=unitjet(X,nmax,C)
% Differentiation of s^2=X.'*X and s*U=X, exact up to nmax=4.
s=cell(1,nmax+1);U=cell(1,nmax+1);
s{1}=sqrt(sum(X{1}.^2));U{1}=X{1}/s{1};
for n=1:nmax
    rhs=0;
    for k=0:n
        rhs=rhs+C(n+1,k+1)*sum(X{k+1}.*X{n-k+1});
    end
    for k=1:n-1
        rhs=rhs-C(n+1,k+1)*s{k+1}*s{n-k+1};
    end
    s{n+1}=rhs/(2*s{1});
    num=X{n+1};
    for k=1:n
        num=num-C(n+1,k+1)*s{k+1}*U{n-k+1};
    end
    U{n+1}=num/s{1};
end
end

function V=jetcross(A,B,nmax,C)
V=cell(1,nmax+1);
for n=0:nmax
    acc=zeros(3,1);
    for k=0:n
        acc=acc+C(n+1,k+1)*cross(A{k+1},B{n-k+1});
    end
    V{n+1}=acc;
end
end
