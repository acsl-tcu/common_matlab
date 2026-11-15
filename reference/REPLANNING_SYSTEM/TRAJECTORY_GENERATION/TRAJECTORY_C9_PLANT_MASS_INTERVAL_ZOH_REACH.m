function R=TRAJECTORY_C9_PLANT_MASS_INTERVAL_ZOH_REACH(method,P21_box,x0_box,u_pre_box,dt)
% Source-exact nonlinear ideal plant 24-state tick with CURRENT estimated
% payload mass as a parameter INTERVAL; mass is held on this ZOH tick.
% No physical Sim truth mismatch is included in this idealized scope.
% Acceptance is a STRICT directed Picard inclusion. All other cases UNKNOWN.
R=struct('status','UNKNOWN_PLANT_MASS_INTERVAL_INPUT', ...
    'valid',false,'directed',false,'all_24x2',NaN(24,2), ...
    'end_24x2',NaN(24,2),'iterations',0,'safe_to_commit',false);
if ~(isa(method,'function_handle')&& ...
        strcmp(func2str(method),'with_load_model_euler_for_HL')&& ...
        valid_box(P21_box,21)&&valid_box(x0_box,24)&& ...
        valid_box(u_pre_box,4)&&isnumeric(dt)&&isscalar(dt)&& ...
        isfinite(dt)&&dt>0)
    return
end
if exist('infsup','file')~=2||exist('intval','file')~=2
    R.status='UNKNOWN_PLANT_INTERVAL_ENGINE_MISSING';return
end
if ~(P21_box(1,1)>0 && all(P21_box(6:8,1)>0) && ...
        P21_box(21,1)>0 && P21_box(20,1)>=0 && ...
        P21_box(1,1)+P21_box(20,1)>0)
    R.status='UNKNOWN_PLANT_MODEL_MASS_SINGULAR';return
end
try
    X0=infsup(x0_box(:,1),x0_box(:,2));
    U=infsup(u_pre_box(:,1),u_pre_box(:,2));
    P=infsup(P21_box(:,1).',P21_box(:,2).'); % SOURCE expects 1x21 row
    tau=infsup(0,dt);
    F0=method(X0,U,P);
    if ~finite_int(F0,24)
        R.status='UNKNOWN_PLANT_INTERVAL_INITIAL_RHS';return
    end
    Y=X0+tau.*F0;
    for j=1:15
        R.iterations=j;
        if ~finite_int(Y,24)||contains_zero(cos(Y(5)))
            R.status='UNKNOWN_PLANT_INTERVAL_EULER_DOMAIN';return
        end
        F=method(Y,U,P);
        if ~finite_int(F,24)
            R.status='UNKNOWN_PLANT_INTERVAL_RHS';return
        end
        Q=X0+tau.*F;
        a=double(inf(Y));b=double(sup(Y));
        c=double(inf(Q));d=double(sup(Q));
        if all(c>a & d<b)
            E=X0+dt.*F;
            if ~finite_int(E,24)
                R.status='UNKNOWN_PLANT_INTERVAL_END';return
            end
            R.all_24x2=[a,b];
            R.end_24x2=[double(inf(E)),double(sup(E))];
            R.valid=true;R.directed=true;
            R.status='CONDITIONAL_IDEAL_SOURCE_PLANT_FULL_TICK_REACH__SENSOR_NEXT_PENDING';
            return
        end
        if ~all(isfinite([a;b;c;d]))
            R.status='UNKNOWN_PLANT_INTERVAL_UNBOUNDED';return
        end
        lo=min(a,c);hi=max(b,d);
        widen=max(hi-lo,eps(max(1,max(abs(lo),abs(hi)))));
        Y=infsup(lo-widen,hi+widen);
    end
    R.status='UNKNOWN_PLANT_INTERVAL_PICARD_INCLUSION';
catch ME
    R.status=['UNKNOWN_PLANT_INTERVAL_EXCEPTION_' ME.identifier];
end
end
function ok=valid_box(X,n)
ok=isnumeric(X)&&isreal(X)&&isequal(size(X),[n 2]) && ...
  all(isfinite(X(:)))&&all(X(:,1)<=X(:,2));
end
function ok=finite_int(x,n)
try
  ok=isa(x,'intval')&&isequal(size(x),[n 1])&& ...
     all(isfinite([double(inf(x));double(sup(x))]));
catch
  ok=false;
end
end
function yes=contains_zero(z)
yes=any(double(inf(z))<=0 & double(sup(z))>=0);
end
