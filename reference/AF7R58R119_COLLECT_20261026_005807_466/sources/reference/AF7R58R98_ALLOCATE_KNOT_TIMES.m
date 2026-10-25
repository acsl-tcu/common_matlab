function A=AF7R58R98_ALLOCATE_KNOT_TIMES(W,t0,t1,L,vmax)
% REFERENCE ONLY: exact necessary knot reachability time allocation.
% With a taut cable |pQ-pL|=L, any two knots satisfy
%   |pQ(i+1)-pQ(i)| >= max(0,|pL(i+1)-pL(i)|-2L).
% No generated knot interval may be shorter than that / external vQmax.
% Remaining time is distributed with segment lengths (no fixed time cap).
assert(isnumeric(W)&&size(W,1)==3&&size(W,2)>=2&&all(isfinite(W(:))) ...
    &&isnumeric(t0)&&isscalar(t0)&&isfinite(t0) ...
    &&isnumeric(t1)&&isscalar(t1)&&isfinite(t1)&&t1>t0 ...
    &&isnumeric(L)&&isscalar(L)&&isfinite(L)&&L>0 ...
    &&isnumeric(vmax)&&isscalar(vmax)&&isfinite(vmax)&&vmax>0, ...
    'R98:MissingLiveKnotTimeInputs', ...
    'Finite physical times, cable length and speed limit are required.');
T=t1-t0;d=vecnorm(diff(W,1,2),2,1);
lo=max(0,d-2*L)/vmax;necessary=sum(lo);
A=struct('status','UNRESOLVED','feasible_necessary_time',false, ...
    'necessary_total_time_s',necessary,'available_total_time_s',T, ...
    'minimum_per_knot_time_s',lo,'times_s',[], ...
    'production_commit_authorized',false, ...
    'whole_body_certified',false);
if any(~isfinite(d))||any(d<=0)||~isfinite(necessary)
    A.status='INCONCLUSIVE_DEGENERATE_OR_UNREPRESENTABLE_KNOTS';return;
end
if necessary>T
    A.status='NECESSARY_TOTAL_UAV_TRAVEL_TIME_EXCEEDS_DURATION';return;
end
slack=T-necessary;
if slack<=0
    A.status='INCONCLUSIVE_NO_STRICTLY_POSITIVE_KNOT_SLACK';return;
end
share=d/sum(d);
dt=lo+slack*share;
if any(~isfinite(dt))||any(dt<=0)
    A.status='INCONCLUSIVE_NONPOSITIVE_KNOT_INTERVAL';return;
end
% Preserve exact physical endpoints, and reject if floating point cannot
% represent strictly increasing knot times.
times=[t0,t0+cumsum(dt)];times(end)=t1;
if any(diff(times)<=0)
    A.status='INCONCLUSIVE_MACHINE_KNOT_TIME_RESOLUTION';return;
end
v=(max(0,d-2*L))./diff(times);
if any(v>vmax+32*eps(max([1,vmax,max(v)])))
    A.status='INCONCLUSIVE_ROUNDED_KNOT_TIME_LOWER_BOUND';return;
end
A.status='NECESSARY_KNOT_REACHABILITY_TIME_ALLOCATED__NOT_SUFFICIENT';
A.feasible_necessary_time=true;
A.times_s=times;
end
