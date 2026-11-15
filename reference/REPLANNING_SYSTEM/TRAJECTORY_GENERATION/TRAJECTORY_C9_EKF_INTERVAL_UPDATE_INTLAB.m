function R=TRAJECTORY_C9_EKF_INTERVAL_UPDATE_INTLAB(agent,prior_x_25x2,prior_P_25x25x2,measurement_12x2,prior_held_u_4x2,dt,latest_mass_1x2)
% Source-matched *one EKF tick* interval enclosure, no guessed disturbance.
% Exact logical order of EKF.do (Flight):
%    xh=ODE(model,xold,uold); A=I+JacobianF(xold,p)*dt;
%    Pp=A*P*A'+B*Q*B'; C=JacobianH(xold,p);
%    G=Pp*C'/(C*Pp*C'+R);
%    Ph=(I-G*C)*Pp; xh=xh+G*(y-C*xh);
%    xh(25)=max(0,xh(25)).
% EKF.do uses min(toc(timer),dt) for covariance Jacobian A, rather
% than a fixed dt. Its Flight runtime value is enclosed by [0,dt].
% The EKF model's continuous plant prediction still integrates full dt.
% The right matrix division is enclosed by a verified preconditioned
% interval solve with ||I-R0*S'||_inf < 1, NOT by a double inverse.
%
% REQUIRED EXTERNAL FACT: measurement_12x2 encloses ACTUAL MOTIVE reading,
% including whatever sensor/round-trip mapping applies. A plant endpoint
% box alone does not prove that fact; this function never silently treats
% an EKF covariance as a deterministic error set.
%
% The return is a directed source-algebraic mapping. Floating-point error
% of the ACTUAL non-interval EKF implementation is NOT bounded here.
R=struct('status','UNKNOWN_EKF_SOURCE_INTERVAL_NOT_STARTED', ...
 'valid',false,'directed',false,'input_measurement_proven',false, ...
 'actual_numeric_EKF_roundoff_proven',false,'future_160_ticks_proven',false, ...
 'safe_to_commit',false,'xhat_next_25x2',NaN(25,2), ...
 'cov_next_25x25x2',NaN(25,25,2),'predictor_status','NOT_RUN', ...
 'gain_contraction_upper',NaN,'gain_verified',false, ...
 'dynamic_mass_jacobian_authority_used',false, ...
 'elapsed_ms',NaN);
clk=tic;
if ~(valid_box(prior_x_25x2,25) && valid_box(measurement_12x2,12) && ...
        valid_box(prior_held_u_4x2,4) && ...
        isnumeric(prior_P_25x25x2)&&isreal(prior_P_25x25x2)&& ...
        isequal(size(prior_P_25x25x2),[25 25 2])&& ...
        all(isfinite(prior_P_25x25x2(:)))&& ...
        all(prior_P_25x25x2(:,:,1)<=prior_P_25x25x2(:,:,2),'all') && ...
        isnumeric(dt)&&isscalar(dt)&&isfinite(dt)&&dt>0)
    R.elapsed_ms=1000*toc(clk);return
end
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_EKF_INTLAB_NOT_INSTALLED'; R.elapsed_ms=1000*toc(clk);return
end
try
    ekf=agent.estimator.ekf;
    if ~(isa(ekf,'EKF') && isa(ekf.model,'MODEL_CLASS') && ...
            strcmp(char(ekf.model.name),'Load_mL_HL') && ...
            strcmp(func2str(ekf.model.method),'with_load_model_mL_euler_for_HL') && ...
            isequal(double(ekf.dt),double(dt)) && ...
            isequal(size(ekf.B),[25 13]) && ...
            isequal(size(ekf.Q),[13 13]) && ...
            isequal(size(ekf.R),[12 12]))
        R.status='UNKNOWN_EKF_SOURCE_LAYOUT_NOT_MATCHED';
        R.elapsed_ms=1000*toc(clk);return
    end
    % Use the *actual* estimator parameter layout, without substituting
    % the physical plant's source-array order or the Sim true payload mass.
    modelP=double(ekf.model.param(:).');
    if ~(numel(modelP)==21&&all(isfinite(modelP))&& ...
         modelP(1)>0&&all(modelP(6:8)>0)&&modelP(21)>0)
        R.status='UNKNOWN_EKF_SOURCE_PARAMETER_LAYOUT';
        R.elapsed_ms=1000*toc(clk);return
    end
    V=TRAJECTORY_C9_EKF_PREDICTION_REACH_INTLAB( ...
        ekf.model.method,modelP,prior_x_25x2,prior_held_u_4x2,dt);
    R.predictor_status=V.status;
    if ~V.validated
        R.status=['UNKNOWN_EKF_PREDICTION_' V.status];
        R.elapsed_ms=1000*toc(clk);return
    end
    Xold=infsup(prior_x_25x2(:,1),prior_x_25x2(:,2));
    Xp=infsup(V.end_box_25x2(:,1),V.end_box_25x2(:,2));
    Pold=infsup(prior_P_25x25x2(:,:,1),prior_P_25x25x2(:,:,2));
    y=infsup(measurement_12x2(:,1),measurement_12x2(:,2));
    % EKF.do uses agent.parameter.get() as p for JacobianF/H, rather
    % than the model's 21 parameter row. Preserve that exact distinction.
    srcp=agent.parameter.get();
    if ~isnumeric(srcp)||numel(srcp)~=21||any(~isfinite(srcp(:)))
        R.status='UNKNOWN_EKF_LIVE_JACOBIAN_AUTHORITY';
        R.elapsed_ms=1000*toc(clk);return
    end
    % The Flight state manager writes the newest EKF mL back into
    % agent.parameter(20). Over multiple FUTURE ticks the handle's
    % present-day loadmass must NOT be frozen. Use the actual source
    % parameter order of DRONE_PARAM_SUSPENDED_LOAD, and enclose the
    % current prior estimator mass rather than assuming it stays fixed.
    if nargin>=7 && ~isempty(latest_mass_1x2)
        mm=double(latest_mass_1x2(:)).';
        if numel(mm)~=2 || any(~isfinite(mm)) || mm(1)<0 || mm(1)>mm(2)
            R.status='UNKNOWN_EKF_DYNAMIC_MASS_ENCLOSURE';
            R.elapsed_ms=1000*toc(clk);return
        end
        if ~(mm(1)<=prior_x_25x2(25,1) && ...
                mm(2)>=prior_x_25x2(25,2))
            R.status='UNKNOWN_EKF_LATEST_MASS_NOT_ENCLOSING_ESTIMATOR';
            R.elapsed_ms=1000*toc(clk);return
        end
        srcp=intval(srcp);
        srcp(20)=infsup(mm(1),mm(2));
        R.dynamic_mass_jacobian_authority_used=true;
    end
    J=ekf.JacobianF(Xold,srcp);
    C=ekf.JacobianH(Xold,srcp);
    % A numeric Jacobian evaluated only at a midpoint cannot enclose
    % a nonlinear Jacobian on a non-singleton EKF state box.
    if ~isa(J,'intval') && any(prior_x_25x2(:,1)~=prior_x_25x2(:,2))
        R.status='UNKNOWN_EKF_NONINTERVAL_NONLINEAR_JACOBIAN';
        R.elapsed_ms=1000*toc(clk);return
    end
    if ~(isequal(size(J),[25 25]) && isequal(size(C),[12 25]))
        R.status='UNKNOWN_EKF_SOURCE_JACOBIAN_SHAPE';
        R.elapsed_ms=1000*toc(clk);return
    end
    if ~isa(J,'intval'),J=intval(J);end
    if ~isa(C,'intval'),C=intval(C);end
    I=intval(eye(25));
    % In the ACTUAL EKF.do implementation, dt_cov=min(wall_timer,dt).
    % Its nonnegative valid interval is [0,dt], not a fictitious fixed dt.
    dt_cov=infsup(0,dt);
    A=I+J*dt_cov;
    PP=A*Pold*A.'+intval(ekf.B)*intval(ekf.Q)*intval(ekf.B).';
    S=C*PP*C.'+intval(ekf.R);
    W=PP*C.';
    [GT,contract,good]=verified_left_solve(S.',W.');
    R.gain_contraction_upper=contract;
    if ~good
        R.status='UNKNOWN_EKF_KALMAN_GAIN_INTERVAL_SINGULAR_OR_TOO_WIDE';
        R.elapsed_ms=1000*toc(clk);return
    end
    G=GT.';
    Xnext=Xp+G*(y-C*Xp);
    Pnext=(I-G*C)*PP;
    % Flight's STATE_CLASS/Model projection: EKF loadmass>=0.
    Xnext(25)=max(intval(0),Xnext(25));
    xlo=double(inf(Xnext)); xhi=double(sup(Xnext));
    plo=double(inf(Pnext)); phi=double(sup(Pnext));
    if ~(all(isfinite([xlo;xhi;plo(:);phi(:)])) && ...
            all(xlo<=xhi)&&all(plo(:)<=phi(:)))
        R.status='UNKNOWN_EKF_UPDATE_INTERVAL_NOT_FINITE';
        R.elapsed_ms=1000*toc(clk);return
    end
    R.xhat_next_25x2=[xlo,xhi];
    R.cov_next_25x25x2=cat(3,plo,phi);
    R.gain_verified=true;
    R.valid=true;
    R.directed=true;
    R.status='CONDITIONAL_SOURCE_EKF_ONE_TICK_INTERVAL_MAP__MEASUREMENT_AND_FLOAT_EKF_ROUNDOFF_PENDING';
catch ME
    R.status=['UNKNOWN_EKF_SOURCE_INTERVAL_EXCEPTION_' ME.identifier];
end
R.elapsed_ms=1000*toc(clk);
end

function [X,rho,valid]=verified_left_solve(S,B)
% Interval 12x12 linear solve with a bound on the entire inverse family.
% For E=I-R0*S and ||E||_inf<=rho<1, the fixed-point enclosure
% x=x0 + (I-E)^-1 R0*(B-S*x0) is sound for EVERY S,B in the box.
valid=false;rho=NaN;X=[];
n=size(S,1);m=size(B,2);
try
    sc=(double(inf(S))+double(sup(S)))/2;
    rc=sc\eye(n);
    if any(~isfinite(rc(:))),return;end
    R0=intval(rc);
    E=intval(eye(n))-R0*S;
    % Sum inside INTLAB; do not round down a double sum of suprema.
    upperRows=double(sup(sum(abs(E),2)));
    rho=max(upperRows);
    if ~(isfinite(rho)&&rho>=0&&rho<1),return;end
    X=infsup(zeros(n,m),zeros(n,m));
    for j=1:m
        bc=B(:,j);
        x0=rc*mid(bc);
        residual=R0*(bc-S*x0);
        % Interval division bounds 1/(1-rho) outward.
        bound_int=intval(max(double(sup(abs(residual)))))/(1-intval(rho));
        bound=double(sup(bound_int));
        if ~(isfinite(bound)&&bound>=0),return;end
        X(:,j)=infsup(x0,x0)+infsup(-bound,bound);
    end
    valid=true;
catch
    valid=false;
end
end
function ok=valid_box(X,n)
ok=isnumeric(X)&&isreal(X)&&isequal(size(X),[n 2]) && ...
    all(isfinite(X(:)))&&all(X(:,1)<=X(:,2));
end
