function R=TRAJECTORY_C9_HLC_PLANT_TICK_REACH_INTLAB(plant_method,plant_parameter_21,initial_state_24x2,pre_input_4x2,input_limits_4x2,dt_s)
%TRAJECTORY_C9_HLC_PLANT_TICK_REACH_INTLAB  Verified one-tick SOURCE ODE tube.
%
% Bounded ZOH ODE: x'=with_load_model_euler_for_HL(x,u,P) for 0<=s<=dt.
% The caller must bind plant_method/parameters to the ACTIVE Sim model and
% pre_input_4x2 to a sound interval evaluation of the ACTIVE HLC at this
% tick. The 4 pre-clamp inputs are NEVER silently replaced by a nominal
% flatness formula. Post-clamp u is enclosed by the exact monotone clamp.
%
% The validated Picard containment condition is
%     X0 + [0,h] * f(Y,U,P) subset Y.
% For a locally Lipschitz RHS on convex Y (all denominators bounded away
% from zero), this ensures the unique solution exists for the entire tick,
% stays in Y, and ends inside X0+h*f(Y,U,P). No interpolation samples,
% guessed error radius, CFL assumption or clipping of certificate failures.
% INTLAB infsup is required for OUTWARD-rounded interval arithmetic.
%
% IMPORTANT: this is a REAL SOURCE plant tick-REACHABILITY COMPONENT, not
% an automatic future EKF/HLC tube B proof. In particular, the caller
% still needs a certified future EKF state box for the HLC input, a
% reference box and yaw-branch proof on EACH tick, plus a proof of ZOH
% authority binding. Caller must decide whether P(20) is the idealized
% latest EKF mass (user's model) or the Sim plant mass; the function does
% not substitute either secretly. Any missing link remains UNKNOWN.
R=struct('status','UNKNOWN_PLANT_TICK_INVALID_INPUT', ...
 'source_rhs_bound',false,'directed_rounding_used',false, ...
 'intersample_reach_proven',false,'actual_HLC_EKF_closed_loop_proven',false, ...
 'pre_input_within_limits',false,'safe_to_commit',false, ...
 'box_all_24x2',NaN(24,2),'box_end_24x2',NaN(24,2), ...
 'u_held_4x2',NaN(4,2),'picard_iterations',0, ...
 'elapsed_ms',NaN,'scope','CONDITIONAL_RIGOROUS_ONE_TICK_PLANT_SOURCE_ONLY');
clock=tic;
if ~(isa(plant_method,'function_handle') && ...
     strcmp(func2str(plant_method),'with_load_model_euler_for_HL') && ...
     valid_box(initial_state_24x2,24) && ...
     valid_box(pre_input_4x2,4) && ...
     valid_box(input_limits_4x2,4) && ...
     isdouble(plant_parameter_21)&&isreal(plant_parameter_21)&& ...
     isequal(size(plant_parameter_21),[1 21])&& ...
     all(isfinite(plant_parameter_21(:)))&& ...
     isdouble(dt_s)&&isscalar(dt_s)&&isfinite(dt_s)&&dt_s>0)
    R.elapsed_ms=1000*toc(clock);return
end
P=plant_parameter_21;
if ~(P(1)>0&&P(6)>0&&P(7)>0&&P(8)>0&&P(21)>0&& ...
        P(20)>=0&&P(1)+P(20)>0)
    R.status='UNKNOWN_PLANT_SOURCE_SINGULAR_PHYSICAL_PARAMETERS';
    R.elapsed_ms=1000*toc(clock);return
end
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_PLANT_DIRECTED_INTERVAL_ENGINE_MISSING';
    R.elapsed_ms=1000*toc(clock);return
end
% Source HLC output is the PRE-clamp input. Clamp is monotone in each
% component; this output is the interval of actually held inputs.
loLim=input_limits_4x2(:,1);hiLim=input_limits_4x2(:,2);
lo=max(loLim,min(hiLim,pre_input_4x2(:,1)));
hi=max(loLim,min(hiLim,pre_input_4x2(:,2)));
R.u_held_4x2=[lo,hi];
R.pre_input_within_limits=all(pre_input_4x2(:,1)>=loLim & ...
                                pre_input_4x2(:,2)<=hiLim);
try
    X0=infsup(initial_state_24x2(:,1),initial_state_24x2(:,2));
    U=infsup(lo,hi);
    PI=infsup(P,P);
    tau=infsup(0,dt_s);
    f0=plant_method(X0,U,PI);
    if ~valid_interval_vector(f0,24)
        R.status='UNKNOWN_PLANT_SOURCE_NONFINITE_AT_INITIAL_SET';
        R.elapsed_ms=1000*toc(clock);return
    end
    Y=X0+tau.*f0;
    % Inclusion rather than a preselected number of subdivisions is the
    % sole ACCEPTANCE criterion. At most 14 enlargements are attempted;
    % no certificate is returned if that construction does not succeed.
    for it=1:15
        R.picard_iterations=it;
        if ~denominator_domain_ok(Y,P)
            R.status='UNKNOWN_PLANT_SOURCE_POLE_IN_REACH_BOX';
            R.elapsed_ms=1000*toc(clock);return
        end
        Fy=plant_method(Y,U,PI);
        if ~valid_interval_vector(Fy,24)
            R.status='UNKNOWN_PLANT_SOURCE_NONFINITE_RHS';
            R.elapsed_ms=1000*toc(clock);return
        end
        Q=X0+tau.*Fy;
        yl=double(inf(Y));yh=double(sup(Y));
        ql=double(inf(Q));qh=double(sup(Q));
        if all(isfinite([yl;yh;ql;qh])) && ...
                all(yl<=ql & qh<=yh)
            endBox=X0+dt_s.*Fy;
            if ~valid_interval_vector(endBox,24)
                R.status='UNKNOWN_PLANT_SOURCE_NONFINITE_END';
                R.elapsed_ms=1000*toc(clock);return
            end
            R.box_all_24x2=[yl,yh];
            R.box_end_24x2=[double(inf(endBox)),double(sup(endBox))];
            R.source_rhs_bound=true;
            R.directed_rounding_used=true;
            R.intersample_reach_proven=true;
            R.status='CONDITIONAL_SOURCE_PLANT_ZOH_TICK_REACH_PROVEN__EKF_HLC_BINDING_PENDING';
            R.elapsed_ms=1000*toc(clock);return
        end
        if ~all(isfinite([yl;yh;ql;qh]))
            R.status='UNKNOWN_PLANT_SOURCE_INTERVAL_UNBOUNDED';
            R.elapsed_ms=1000*toc(clock);return
        end
        lower=min(yl,ql);upper=max(yh,qh);
        span=upper-lower;
        enlargement=max(span,eps(max(1,max(abs(lower),abs(upper)))));
        % Doubling is only an enclosure-CONSTRUCTION strategy; no
        % acceptance depends on an empirical margin. Only the rigorous
        % Picard inclusion above can label the tick reachable.
        Y=infsup(lower-enlargement,upper+enlargement);
    end
    R.status='UNKNOWN_PLANT_PICARD_INCLUSION_NOT_ESTABLISHED';
catch ME
    R.status=['UNKNOWN_PLANT_SOURCE_INTERVAL_EXCEPTION_' ME.identifier];
end
R.elapsed_ms=1000*toc(clock);
end

function ok=denominator_domain_ok(Y,P)
% Active generated plant RHS contains 1/cos(pitch), 1/m, 1/(m+mL),
% 1/jx, 1/jy, 1/jz and 1/cableL.
try
    cp=cos(Y(5));
    ok=~(double(inf(cp))<=0 && double(sup(cp))>=0) && ...
        all(isfinite([double(inf(cp)),double(sup(cp))])) && ...
        P(1)>0 && P(6)>0 && P(7)>0 && P(8)>0 && ...
        P(21)>0 && P(1)+P(20)>0;
catch
    ok=false;
end
end
function ok=valid_interval_vector(x,n)
try
 ok=isequal(size(x),[n 1]) && ...
    all(isfinite([double(inf(x));double(sup(x))]));
catch
 ok=false;
end
end
function ok=valid_box(b,n)
ok=isnumeric(b)&&isreal(b)&&isequal(size(b),[n 2])&& ...
    all(isfinite(b(:)))&&all(b(:,1)<=b(:,2));
end
