function R=TRAJECTORY_C9_DIRECTED_BERNSTEIN_FINAL_GATE(piece,observed, ...
    radii,authority,geo)
%TRAJECTORY_C9_DIRECTED_BERNSTEIN_FINAL_GATE No double-as-proof promotion.
% Independent outward INTLAB sufficient full-time NOMINAL envelope for
% exactly one 23/27-degree C9 payload Bernstein segment. All UAV/cable/
% payload/ground geometry is checked via source shape support inequalities.
% Safe to use only for mathematical model with binary64 input parameters
% interpreted as exact point values and validated prepared SO(3) shapes.
% This function does NOT cover HLC tracking, actual double code rounding,
% yaw/torque, or unknown unsensed shapes; no Flight authorization.
R=struct('status','UNKNOWN_DIRECTED_BERNSTEIN_NOT_RUN', ...
 'directed_interval_operators_used',false, ...
 'nominal_fulltime_tension_proven',false, ...
 'nominal_fulltime_thrust_proven',false, ...
 'nominal_fulltime_allbody_proven',false, ...
 'conditional_nominal_fulltime_proven',false, ...
 'actual_floating_execution_proven',false, ...
 'input_shape_source_rounding_proven',false, ...
 'HLC_B_proven',false,'safe_to_commit',false, ...
 'minimum_w_norm_lower',NaN, ...
 'tension_lower_N',NaN,'thrust_interval_N',[NaN NaN], ...
 'body_status','NOT_RUN','elapsed_ms',NaN);
clock=tic;
try
 if exist('intval','file')~=2 || exist('infsup','file')~=2
     R.status='UNKNOWN_DIRECTED_INTLAB_NOT_AVAILABLE';return
 end
 if ~(isstruct(piece)&&isfield(piece,'control_points_m') && ...
         isfield(piece,'duration_s') && ...
         isequal(size(piece.control_points_m,1),3) && ...
         ismember(size(piece.control_points_m,2),[24 28]) && ...
         all(isfinite(piece.control_points_m(:))) && ...
         isfinite(piece.duration_s)&&piece.duration_s>0 && ...
         isstruct(observed)&&isstruct(radii)&&isstruct(authority)&& ...
         isstruct(geo)&&isfield(geo,'trajectory_phase')&& ...
         strcmp(char(geo.trajectory_phase),'f'))
     R.status='UNKNOWN_DIRECTED_AUTHORITY_OR_FLIGHT';return
 end
 P0=intval(double(piece.control_points_m));
 T=intval(double(piece.duration_s));
 D2=bern_derivative(bern_derivative(P0,T),T);
 D3=bern_derivative(D2,T);D4=bern_derivative(D3,T);
 W=intval(double(authority.gravity_world_mps2(:)))-D2;
 WL=double(min(inf(W),[],2));WU=double(max(sup(W),[],2));
 Wi=infsup(WL,WU);
 normW=sqrt(sum(Wi.^2));
 lowerW=double(inf(normW));upperW=double(sup(normW));
 R.minimum_w_norm_lower=lowerW;
 if ~(isfinite(lowerW)&&isfinite(upperW)&&lowerW>0)
     R.status='UNKNOWN_DIRECTED_TAUT_DIRECTION_SINGULAR';return
 end
 mU=intval(authority.uav_mass_kg);
 mL=intval(authority.payload_mass_kg);
 L=intval(authority.cable_length_m);
 if ~(double(inf(mU))>0&&double(inf(mL))>=0&&double(inf(L))>0)
     R.status='UNKNOWN_DIRECTED_MASS_LENGTH_AUTHORITY';return
 end
 tension=mL*normW;
 R.tension_lower_N=double(inf(tension));
 if ~(R.tension_lower_N>double(authority.minimum_tension_N))
     R.status='UNKNOWN_DIRECTED_TENSION_NOT_PROVED';return
 end
 R.nominal_fulltime_tension_proven=true;
 D1=norm_upper(D3);D2upper=norm_upper(D4);
 qdd=D2upper/normW+3*(D1/normW)^2;
 delta=mU*L*qdd;
 force=(mU+mL)*normW;
 flo=double(inf(force-delta));
 fhi=double(sup(force+delta));
 R.thrust_interval_N=[flo,fhi];
 lo=double(authority.thrust_limits_N(1));
 hi=double(authority.thrust_limits_N(2));
 guard=0;
 if isfield(geo,'force_guard_N'),guard=double(geo.force_guard_N);end
 if ~(isfinite(flo)&&isfinite(fhi) && ...
         flo>=lo+guard && fhi<=hi-guard)
     R.status='UNKNOWN_DIRECTED_THRUST_NOT_PROVED';return
 end
 R.nominal_fulltime_thrust_proven=true;
 % Bernstein convex hull encloses the load at every t; W hull encloses
 % every acceleration. Division by normW here is an outward INTLAB
 % extension, so U = pL-L W/||W|| encloses the whole UAV trajectory.
 PL=infsup(min(inf(P0),[],2),max(sup(P0),[],2));
 q=Wi/normW;
 U=PL-L*q;
 X=zeros(24,2);
 X(1:3,:)=[double(inf(U)),double(sup(U))];
 X(13:15,:)=[double(inf(PL)),double(sup(PL))];
 rr=radii;
 if isfield(geo,'robust_tube_enabled') && geo.robust_tube_enabled
     if ~isfield(geo,'uncertainty_bounds_m')
         R.status='UNKNOWN_DIRECTED_TUBE_INFLATION_AUTHORITY';return
     end
     u=geo.uncertainty_bounds_m;
     rr.uav_m=rr.uav_m+u.uav_m;
     rr.cable_m=rr.cable_m+u.cable_m;
     rr.payload_m=rr.payload_m+u.payload_m;
 end
 if isfield(geo,'numerical_guard_m')
     v=geo.numerical_guard_m;
     assert(isscalar(v)&&isfinite(v)&&v>=0);
     rr.uav_m=rr.uav_m+v;
     rr.cable_m=rr.cable_m+v;
     rr.payload_m=rr.payload_m+v;
 end
 S=TRAJECTORY_C9_HLC_TICK_BODY_SEPARATION_INTLAB(X,observed,rr);
 R.body_status=S.status;
 R.nominal_fulltime_allbody_proven=S.conditional_alltime_body_separation;
 R.directed_interval_operators_used=true;
 if ~R.nominal_fulltime_allbody_proven
     R.status=['UNKNOWN_DIRECTED_BODY_' S.status];return
 end
 R.conditional_nominal_fulltime_proven=true;
 R.status='CONDITIONAL_DIRECTED_NOMINAL_ALLTIME_BERNSTEIN_TENSION_THRUST_BODY__HLC_TORQUE_NUMERICAL_SOURCE_UNPROVEN';
catch ME
 R.status=['UNKNOWN_DIRECTED_BERNSTEIN_EXCEPTION_' ME.identifier];
 R.error_message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
function D=bern_derivative(P,T)
 n=size(P,2)-1;
 D=(intval(n)/T)*(P(:,2:end)-P(:,1:end-1));
end
function z=norm_upper(P)
 a=max(abs(double(inf(P))),abs(double(sup(P))));
 % A point INTLAB vector with nonnegative component maxima bounds each
 % derivative Bernstein coefficient; interval norm rounds outward.
 a=max(a,[],2);
 z=sqrt(sum(intval(a).^2));
end
