function [J,D] = AF7R58R105_C7_TAIL_EVAL(route, source, t)
% Source-bounded read-only payload C7 piecewise connector + time-shifted
% nominal continuation.  Does NOT assert tail obstacle/input/closed-loop safety.
% source: nominal_derivative_fns (raw), flight_reference_time_origin,
% nominal_table_tend, cable_length_m. All supplied from a live source snapshot.
assert(isstruct(route)&&isscalar(route)&&isfield(route,'segments')&& ...
    isfield(route,'phase_delay_s')&&isfield(route,'knot_jets')&& ...
    isfield(route,'times_s')&&isfield(route,'num_segments'), ...
    'R105:RouteSchema','A verified C7 connector route is required.');
needed={'nominal_derivative_fns','flight_reference_time_origin', ...
    'nominal_table_tend','cable_length_m'};
assert(isstruct(source)&&isscalar(source)&&all(isfield(source,needed)), ...
    'R105:TailAuthority','Live nominal domain and cable length required.');
assert(isscalar(t)&&isnumeric(t)&&isfinite(t), ...
    'R105:FiniteEvalTime','Evaluation time must be finite.');
S=double(route.num_segments);ts=double(route.times_s(:).');
assert(numel(ts)==S+1&&all(diff(ts)>0)&&numel(route.segments)==S&& ...
    isequal(size(route.knot_jets),[3 8 S+1]), ...
    'R105:ConnectorSegments','Invalid physical knot or shared-jet layout.');
lag=double(route.phase_delay_s);L=double(source.cable_length_m);
tEnd=double(source.nominal_table_tend);tOrigin=double(source.flight_reference_time_origin);
assert(isfinite(lag)&&lag>=0&&isfinite(L)&&L>0&& ...
    isfinite(tEnd)&&isfinite(tOrigin)&&tEnd>=tOrigin, ...
    'R105:LiveNominalDomain','Nominal-domain authority is invalid.');
D=struct('status','OUTSIDE_CERTIFIED_SOURCE_DOMAIN', ...
    'inside_finite_source_domain',false, ...
    'part','NONE','segment_index',NaN, ...
    'yaw_rad',NaN,'horizontal_speed_m_s',NaN, ...
    'heading_nondegenerate_numeric',false, ...
    'tail_closed_loop_certified',false,'production_commit_authorized',false);
J=nan(3,8);
if t<ts(1) || t>tEnd+lag || ts(end)-lag>tEnd || ts(end)-lag<tOrigin
    return;
end
if t<=ts(end)
    i=find(t>=ts(1:end-1)&t<=ts(2:end),1,'last');
    if isempty(i),return;end
    C=route.segments(i);
    if ~isequal(size(C.payload_cp),[3 16])|| ...
            ~isfinite(C.duration_s)||C.duration_s<=0|| ...
            abs(double(C.duration_s)-(ts(i+1)-ts(i)))> ...
            64*eps(max([1,ts(i+1)-ts(i)]))
        D.status='INVALID_CONNECTOR_CONTROL_POINTS';return;
    end
    u=(double(t)-ts(i))/double(C.duration_s);
    if u<0||u>1,return;end
    cp=double(C.payload_cp);
    for d=0:7
        J(:,d+1)=cp*local_bernstein(size(cp,2)-1,u);
        if d<7
            cp=((size(cp,2)-1)/double(C.duration_s))*diff(cp,1,2);
        end
    end
    D.part='CONNECTOR';D.segment_index=i;
else
    tau=double(t)-lag;
    if tau<tOrigin||tau>tEnd,return;end
    J=AF7R58R88_NOMINAL_JETS(source.nominal_derivative_fns,tOrigin,tau);
    J(3,1)=J(3,1)-L;
    D.part='DELAYED_NOMINAL';
end
if ~all(isfinite(J(:)))
    D.status='NONFINITE_C7_JETS';return;
end
D.status='FINITE_SOURCE_BOUNDED_C7_EVALUATION__NOT_FULL_SAFETY';
D.inside_finite_source_domain=true;
D.horizontal_speed_m_s=hypot(J(1,2),J(2,2));
if D.horizontal_speed_m_s>0
    D.heading_nondegenerate_numeric=true;
    D.yaw_rad=atan2(J(2,2),J(1,2));
end
end

function B=local_bernstein(n,u)
k=0:n;
B=(arrayfun(@(j)nchoosek(n,j),k).').*(u.^k.').*((1-u).^(n-k).');
end
