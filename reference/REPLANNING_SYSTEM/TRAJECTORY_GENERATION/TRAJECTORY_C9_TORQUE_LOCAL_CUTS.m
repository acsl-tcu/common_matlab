function K=TRAJECTORY_C9_TORQUE_LOCAL_CUTS(piece,authority,options,times_s,U)
% LOCAL SQP LINEARIZATION of the real-analytic 3-axis NOMINAL torque.
% Produces A'*dx>=b in the *same 12 whitened C9 free coordinates* as the
% simultaneous obstacle QP. Each axis contributes at most two active
% inequality rows (the strongest violated upper/lower sampled limits).
% Complex-step differentiates the exact pointwise model, avoiding noisy
% finite differences. This is NOT a convex global input certificate:
% the nonlinear final candidate still requires independent ALL-TIME checks.
clock=tic;
K=struct('status','UNKNOWN_TORQUE_CUTS_NOT_EVALUATED', ...
    'A',zeros(12,0),'b',zeros(0,1), ...
    'cut_count',0,'witness_count',0,'witness_times_s',zeros(1,0), ...
    'cut_axes',zeros(1,0),'cut_sides',zeros(1,0), ...
    'cut_times_s',zeros(1,0),'cut_point_values_Nm',zeros(1,0), ...
    'cut_gradients_Nm_per_qp',zeros(12,0), ...
    'largest_violation_Nm',NaN,'jacobian_backend','COMPLEX_STEP', ...
    'point_evaluations',0,'jacobian_evaluations',0, ...
    'elapsed_ms',NaN,'safe_to_commit',false, ...
    'scope','LOCAL_LINEARIZATION_ONLY__NOT_NOMINAL_ALL_TIME_OR_HLC');
try
    assert(isnumeric(U)&&isreal(U)&&isequal(size(U),[4 4])&& ...
        all(isfinite(U(:)))&&rcond(U)>0, ...
        'C9_TORQUE_CUTS:Whitening','Need actual current C9 QP whitening');
    P=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
        piece,authority,options,times_s);
    K.point_evaluations=numel(times_s);
    K.largest_violation_Nm=P.maximum_point_violation_Nm;
    K.witness_count=sum(P.outside_limits);
    K.witness_times_s=P.time_s(P.outside_limits);
    if ~all(P.valid)
        K.status='UNKNOWN_TORQUE_POINT_SINGULARITY';
        K.elapsed_ms=1000*toc(clock);return
    end
    if K.witness_count==0
        K.status='NO_VIOLATED_TORQUE_SEARCH_PROBES';
        K.elapsed_ms=1000*toc(clock);return
    end
    lo=double(authority.torque_limits_Nm(:,1));
    hi=double(authority.torque_limits_Nm(:,2));
    guard=double(options.torque_guard_Nm(:));
    lo=lo+guard;hi=hi-guard;
    M=real(P.torque_Nm);
    % One most-violated point for each of the 3 upper and 3 lower bounds.
    % The number six comes from the six physical input inequalities,
    % not an obstacle-specific tuning parameter.
    active_axes=zeros(1,0);active_sides=zeros(1,0);
    active_points=zeros(1,0);active_margin=zeros(1,0);
    for axis=1:3
        [over,k]=max(M(axis,:)-hi(axis));
        if over>0
            active_axes(end+1)=axis; %#ok<AGROW>
            active_sides(end+1)=1; %#ok<AGROW>
            active_points(end+1)=k; %#ok<AGROW>
            active_margin(end+1)=over; %#ok<AGROW>
        end
        [under,k]=max(lo(axis)-M(axis,:));
        if under>0
            active_axes(end+1)=axis; %#ok<AGROW>
            active_sides(end+1)=-1; %#ok<AGROW>
            active_points(end+1)=k; %#ok<AGROW>
            active_margin(end+1)=under; %#ok<AGROW>
        end
    end
    ix=unique(active_points);
    selected_times=P.time_s(ix);
    gradients=NaN(3,12,numel(ix));
    % The 12-coordinate forward differentiation is algebraically equivalent
    % to 12 independent complex steps. OFF until measured MATLAB A/B.
    forward=false;
    if isfield(options,'use_forward_torque_jacobian')
        z=options.use_forward_torque_jacobian;
        assert((isnumeric(z)||islogical(z))&&isscalar(z)&& ...
            isfinite(double(z))&&any(double(z)==[0 1]), ...
            'C9_TORQUE_CUTS:ForwardFlag','Forward flag must be 0/1');
        forward=logical(z);
    end
    if forward
        K.jacobian_backend='FORWARD_12_COORDINATE';
        Z=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN( ...
            piece,authority,options,selected_times,U);
        if ~all(Z.valid)
            K.status=Z.status;
            K.elapsed_ms=1000*toc(clock);return
        end
        gradients=Z.gradient_Nm_per_whitened;
        K.jacobian_evaluations=numel(ix);
    else
    % Analytic (holomorphic) point formulas: complex-step has truncation
    % O(h^2) with NO subtractive finite-difference cancellation. The
    % algorithmic step is dimensionless in whitened QP coordinates.
    h=1e-20;
    E=eye(12);
    for j=1:12
        step=reshape(E(:,j),3,4)/U.';
        perturb=piece;
        perturb.control_points_m(:,11:14)= ...
           double(piece.control_points_m(:,11:14))+1i*h*step;
        Z=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
           perturb,authority,options,selected_times);
        K.jacobian_evaluations=K.jacobian_evaluations+numel(ix);
        if ~all(Z.valid)
            K.status='UNKNOWN_COMPLEX_STEP_SINGULARITY';
            K.elapsed_ms=1000*toc(clock);return
        end
        gradients(:,j,:)=reshape(imag(Z.torque_Nm)/h,3,1,[]);
    end
    end
    A=zeros(12,numel(active_axes));b=zeros(numel(active_axes),1);
    for r=1:numel(active_axes)
        axis=active_axes(r);
        jj=find(ix==active_points(r),1);
        grad=reshape(gradients(axis,:,jj),12,1);
        mag=norm(grad);
        if ~(isfinite(mag)&&mag>0)
            K.status='UNKNOWN_NONCORRECTABLE_TORQUE_LINEARIZATION';
            K.elapsed_ms=1000*toc(clock);return
        end
        % Upper: -grad'*x >= M-hi. Lower: +grad'*x >= lo-M.
        A(:,r)=-active_sides(r)*grad/mag;
        b(r)=active_margin(r)/mag;
    end
    K.A=A;K.b=b;K.cut_count=size(A,2);
    K.cut_axes=active_axes;K.cut_sides=active_sides;
    K.cut_times_s=P.time_s(active_points);
    K.cut_point_values_Nm=M(sub2ind(size(M), ...
       active_axes(:).',active_points(:).'));
    K.cut_gradients_Nm_per_qp=A.*(-active_sides);
    % The gradients above are normalized to unit length for QP scaling.
    % The original magnitudes are reconstructed from active_margin/b.
    K.cut_gradients_Nm_per_qp= ...
       K.cut_gradients_Nm_per_qp.*(active_margin./b.');
    K.status='TORQUE_POINT_LOCAL_CUTS__FULL_CERTIFICATE_REQUIRED';
catch ME
    K.status=['UNKNOWN_TORQUE_LOCAL_CUTS_' ME.identifier];
end
K.elapsed_ms=1000*toc(clock);
end
