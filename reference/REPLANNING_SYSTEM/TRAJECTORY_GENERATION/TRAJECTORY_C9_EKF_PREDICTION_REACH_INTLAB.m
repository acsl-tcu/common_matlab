function R=TRAJECTORY_C9_EKF_PREDICTION_REACH_INTLAB(method,P21,x0_box,u_box,dt)
% Validated source ODE prediction of the ORIGINAL 25-state EKF model.
% Same ZOH as MODEL_CLASS.do; no linearized surrogate or sampled proxy.
% P21 is the exact live 21-parameter row for the estimator, and mL is
% state element 25 (NOT a frozen parameter). The endpoint is projected
% by the original max(0,mL) operation by the caller.
% Intersample inclusion: X0 + [0,dt]*f(Y,U,P) subset interior(Y).
R=struct('status','UNKNOWN_EKF_PREDICTION_INPUT', ...
    'validated',false,'directed',false,'iterations',0, ...
    'all_box_25x2',NaN(25,2),'end_box_25x2',NaN(25,2), ...
    'safe_to_commit',false);
if ~(isa(method,'function_handle') && ...
        strcmp(func2str(method),'with_load_model_mL_euler_for_HL') && ...
        valid_box(x0_box,25)&&valid_box(u_box,4)&& ...
        isnumeric(P21)&&isreal(P21)&&isequal(size(P21),[1 21])&& ...
        all(isfinite(P21(:)))&&P21(1)>0&&all(P21(6:8)>0)&& ...
        P21(21)>0&&isnumeric(dt)&&isscalar(dt)&&isfinite(dt)&&dt>0)
    return
end
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_EKF_INTERVAL_ENGINE_MISSING';return
end
try
    X0=infsup(x0_box(:,1),x0_box(:,2));
    U=infsup(u_box(:,1),u_box(:,2));
    P=infsup(P21,P21);
    tau=infsup(0,dt);
    F0=method(X0,U,P);
    if ~finite_interval(F0,25)
        R.status='UNKNOWN_EKF_PREDICTION_RHS_INITIAL';return
    end
    Y=X0+tau.*F0;
    for j=1:15
        R.iterations=j;
        if ~finite_interval(Y,25) || ...
                double(inf(X0(25)))<0 || ...
                double(inf(Y(25)))< -P21(1) || ...
                contains_zero(cos(Y(5)))
            R.status='UNKNOWN_EKF_PREDICTION_SINGULAR_DOMAIN';return
        end
        FY=method(Y,U,P);
        if ~finite_interval(FY,25)
            R.status='UNKNOWN_EKF_PREDICTION_RHS_RANGE';return
        end
        TUBE=X0+tau.*FY;
        a=double(inf(Y));b=double(sup(Y));
        c=double(inf(TUBE));d=double(sup(TUBE));
        % Strict inclusion guards the existence theorem, not just a
        % floating-point equality of the hull endpoints.
        if all(c>a & d<b)
            END=X0+dt.*FY;
            if ~finite_interval(END,25)
                R.status='UNKNOWN_EKF_PREDICTION_END_RANGE';return
            end
            R.validated=true;R.directed=true;
            R.all_box_25x2=[a,b];
            R.end_box_25x2=[double(inf(END)),double(sup(END))];
            R.status='CONDITIONAL_SOURCE_EKF_25D_ODE_REACH__MEASUREMENT_UPDATE_PENDING';
            return
        end
        if ~all(isfinite([a;b;c;d]))
            R.status='UNKNOWN_EKF_PREDICTION_UNBOUNDED';return
        end
        lo=min(a,c); hi=max(b,d);
        % This is a CONSTRUCTION of a wider candidate Y, not a
        % physical-error heuristic; validity is only the inclusion test.
        spread=max(hi-lo,eps(max(1,max(abs(lo),abs(hi)))));
        Y=infsup(lo-spread,hi+spread);
    end
    R.status='UNKNOWN_EKF_PREDICTION_PICARD_INCLUSION_FAILED';
catch ME
    R.status=['UNKNOWN_EKF_PREDICTION_EXCEPTION_' ME.identifier];
end
end
function ok=valid_box(X,n)
ok=isnumeric(X)&&isreal(X)&&isequal(size(X),[n 2]) && ...
    all(isfinite(X(:)))&&all(X(:,1)<=X(:,2));
end
function ok=finite_interval(V,n)
try
    ok=isa(V,'intval')&&isequal(size(V),[n 1])&& ...
       all(isfinite([double(inf(V));double(sup(V))]));
catch
    ok=false;
end
end
function yes=contains_zero(z)
yes=any(double(inf(z))<=0 & double(sup(z))>=0);
end
