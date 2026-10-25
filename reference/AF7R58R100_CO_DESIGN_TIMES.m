function A=AF7R58R100_CO_DESIGN_TIMES(W,t0,t1,startJets,endJets,delay,L,g,vmax,opts)
% Generator-stage necessary actual UAV-knot travel-time co-design.
% Retimes internal knot intervals based on reconstructed taut-cable Q knots,
% not a pessimistic 2L payload displacement lower bound alone.
% All retimings preserve t0,t1 and their EXACT requested C7 boundary jets.
% Finite work budget != fixed maneuver duration. Never a safety proof.
assert(isstruct(opts)&&isscalar(opts), 'R100:TimingOptionsRequired', ...
    'Generator numeric work options must be a struct.');
A=AF7R58R98_ALLOCATE_KNOT_TIMES(W,t0,t1,L,vmax);
A.retime_trials=0;
A.estimated_peak_before_m_s=NaN;
A.estimated_peak_after_m_s=NaN;
A.retime_status='NOT_RUN';
A.production_commit_authorized=false;
if ~A.feasible_necessary_time || size(W,2)<=2,return;end
if isfield(opts,'max_retime_trials')
    N=double(opts.max_retime_trials);
else
    N=2; % computational effort only
end
assert(isfinite(N)&&N>=0&&N==fix(N), 'R100:RetimingWorkBudget', ...
    'max_retime_trials must be a nonnegative integer.');
T=t1-t0;times=A.times_s;
bestTimes=times;bestScore=Inf;
for iter=0:N
    C=AF7R58R100_C7_INTERIOR_SPEED_AWARE( ...
        times,W,startJets,endJets,delay,L,g,vmax,opts);
    if ~C.reference_candidate_found || ~C.knot_speed_necessary_pass
        A.retime_status='INCONCLUSIVE_C7_ROUTE_FOR_RETIMER';break;
    end
    Q=zeros(3,numel(times));
    for k=1:numel(times)
        f=C.knot_jets(:,3,k)+[0;0;g];nf=norm(f);
        if ~(isfinite(nf)&&nf>128*eps(max([1,g,norm(f)])))
            A.retime_status='INCONCLUSIVE_KNOT_CABLE_DIRECTION';return;
        end
        Q(:,k)=C.knot_jets(:,1,k)+L*f/nf;
    end
    dis=vecnorm(diff(Q,1,2),2,1);
    avg=dis./diff(times);
    score=max([C.interior_peak_sample_m_s,max(avg)]);
    if iter==0,A.estimated_peak_before_m_s=score;end
    if score<bestScore
        bestScore=score;bestTimes=times;
    end
    if iter==N,break;end
    lo=dis/vmax;
    if ~all(isfinite(lo)) || sum(lo)>=T
        A.retime_status='INCONCLUSIVE_NO_ACTUAL_KNOT_TIME_SLACK';break;
    end
    % Exact actual knot distance is the necessary UAV travel distance for
    % the current jets. Distribute the slack by distance (no clock cap).
    if sum(dis)>0
        weights=dis/sum(dis);
    else
        weights=ones(size(dis))/numel(dis);
    end
    dNew=lo+(T-sum(lo))*weights;
    if any(~isfinite(dNew))||any(dNew<=0),break;end
    candidate=[t0,t0+cumsum(dNew)];candidate(end)=t1;
    if any(diff(candidate)<=0),break;end
    if max(abs(candidate-times))<=32*eps(max([1,abs(times)]))
        A.retime_status='RETIME_STATIONARY';break;
    end
    times=candidate;A.retime_trials=A.retime_trials+1;
end
A.times_s=bestTimes;
A.estimated_peak_after_m_s=bestScore;
A.status='R100_KNOT_TIME_OBJECTIVE_SEARCH__NECESSARY_ONLY';
if isempty(A.retime_status)||strcmp(A.retime_status,'NOT_RUN')
    A.retime_status='R100_NUMERIC_KNOT_RETIMING_DONE';
end
% Preserve R98 physical-time necessary conditions for the selected timing.
R=AF7R58R96_KNOT_REACHABILITY_PREFLIGHT(W,bestTimes,L,vmax);
if ~R.passes_necessary_condition
    A.feasible_necessary_time=false;
    A.status='INCONCLUSIVE_RETIME_BROKE_NECESSARY_REACHABILITY';
end
end
