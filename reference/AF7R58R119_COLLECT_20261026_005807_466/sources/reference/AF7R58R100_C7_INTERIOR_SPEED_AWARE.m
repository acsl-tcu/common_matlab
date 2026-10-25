function R=AF7R58R100_C7_INTERIOR_SPEED_AWARE(times,waypoints,startJets,endJets,phaseDelay,L,g,vMax,opts)
% GENERATION OBJECTIVE: reduce sampled peak of true taut-model UAV speed
% by searching interior shared C7 knot jets BEFORE independent R91/R80/R90.
% Numerical samples RANK candidate shapes; NEVER certify whole interval.
% External physical limits (vMax,L,g) are required; work budgets are solver
% effort only. This module NEVER authorizes production Flight commit.
assert(isstruct(opts)&&isscalar(opts), 'R100:OptionsRequired', ...
    'Supply scalar generator-work options.');
R=AF7R58R96_C7_KNOT_SPEED_ROUTE(times,waypoints, ...
    startJets,endJets,phaseDelay,L,g,vMax);
R.internal_speed_trials=0;
R.interior_peak_baseline_sample_m_s=NaN;
R.interior_peak_sample_m_s=NaN;
R.interior_peak_samples_are_safety_proof=false;
R.production_commit_authorized=false;
if ~R.reference_candidate_found || ~R.knot_speed_necessary_pass
    return;
end
if isfield(opts,'max_internal_velocity_trials')
    n=double(opts.max_internal_velocity_trials);
else
    n=9;  % module-internal numeric search WORK budget, not trajectory bound
end
assert(isfinite(n)&&n>=2&&n==fix(n), 'R100:VelocityTrialBudget', ...
    'max_internal_velocity_trials must be integer >=2.');
if isfield(opts,'velocity_objective_samples_per_segment')
    nSamples=double(opts.velocity_objective_samples_per_segment);
else
    nSamples=13;  % ranking mesh ONLY, not continuous-time safety sampling
end
assert(isfinite(nSamples)&&nSamples>=3&&nSamples==fix(nSamples), ...
    'R100:NumericRankingGrid', 'Sample count must be integer >=3.');
baseline=sample_peak(R,g,L,nSamples);
R.interior_peak_baseline_sample_m_s=baseline;
R.interior_peak_sample_m_s=baseline;
K=numel(times);
if K<=2 || ~isfinite(baseline),return;end
% The globally minimum-8th-derivative jets are the endpoint of a 1D
% finite-dimensional C7-feasible interpolation family, not a general optimum.
B=AF7R58R92_C7_MINIMUM8_ROUTE(times,waypoints, ...
    startJets,endJets,phaseDelay,L);
if ~B.reference_candidate_found,return;end
Jmin=B.knot_jets;
Janchor=Jmin;
for k=2:K-1
    vl=(waypoints(:,k)-waypoints(:,k-1))/(times(k)-times(k-1));
    vr=(waypoints(:,k+1)-waypoints(:,k))/(times(k+1)-times(k));
    vv=(vl+vr)/2;
    nv=norm(vv);
    if nv>=vMax
        vv=vv*(vMax*(1-1024*eps(1))/nv);
    end
    Janchor(:,2,k)=vv;
    Janchor(:,3:8,k)=0;
end
alphaCap=R.knot_jet_blend_alpha;
if ~(isscalar(alphaCap)&&isfinite(alphaCap)&&alphaCap>=0&&alphaCap<=1)
    return;
end
Qbest=R;best=baseline;
for i=1:n
    alpha=alphaCap*(i-1)/(n-1);
    if abs(alpha-alphaCap)<16*eps(max(1,alphaCap))
        continue; % R96 baseline is already a member, no duplicate work
    end
    J=Janchor;
    J(:,:,2:end-1)=Janchor(:,:,2:end-1)+alpha*( ...
        Jmin(:,:,2:end-1)-Janchor(:,:,2:end-1));
    if ~knot_vq_ok(J,g,L,vMax),continue;end
    Q=R;
    Q.knot_jets=J;
    Q.knot_jet_blend_alpha=alpha;
    for s=1:Q.num_segments
        Q.segments(s).payload_cp=jet_to_bezier( ...
            J(:,:,s),J(:,:,s+1),times(s+1)-times(s));
    end
    vpeak=sample_peak(Q,g,L,nSamples);
    R.internal_speed_trials=R.internal_speed_trials+1;
    % Sample peak is only an objective for the generation heuristic.
    if isfinite(vpeak) && vpeak<best
        best=vpeak;Qbest=Q;
    end
end
if best<baseline
    R.knot_jets=Qbest.knot_jets;
    R.knot_jet_blend_alpha=Qbest.knot_jet_blend_alpha;
    for s=1:R.num_segments
        R.segments(s).payload_cp=Qbest.segments(s).payload_cp;
    end
    R.status='R100_INTERIOR_VELOCITY_OBJECTIVE_IMPROVED__NOT_SAFETY_CERT';
    R.speed_aware_knot_adjusted=true;
    R.adjusted_jet_solution_is_a_feasibility_heuristic=true;
    R.original_min8_is_the_only_globally_energy_optimal_route=false;
    R.max_jet_residual=endpoint_residual(R);
    R.minimum_8th_derivative_quadratic=energy8(R);
end
R.interior_peak_sample_m_s=best;
R.interior_peak_samples_are_safety_proof=false;
R.production_commit_authorized=false;
end

function ok=knot_vq_ok(J,g,L,vmax)
ok=true;
for k=1:size(J,3)
    v=J(:,2,k);a=J(:,3,k);j=J(:,4,k);
    f=a+[0;0;g];sigma=norm(f);
    if ~(isfinite(sigma)&&sigma>128*eps(max([1,g,norm(a)])))
        ok=false;return;
    end
    q=f/sigma;qd=(j-q*(q'*j))/sigma;
    vv=norm(v+L*qd);
    if ~(isfinite(vv)&&vv<vmax-128*eps(max([1,vmax,vv])))
        ok=false;return;
    end
end
end

function P=jet_to_bezier(A,B,T)
P=zeros(3,16);
for j=0:7
    x=zeros(3,1);y=zeros(3,1);
    for r=0:j
        falling=prod(15:-1:(16-r));
        c=nchoosek(j,r)*T^r/falling;
        x=x+c*A(:,r+1);
        y=y+(-1)^r*c*B(:,r+1);
    end
    P(:,j+1)=x;
    P(:,16-j)=y;
end
end

function v=sample_peak(R,g,L,nSamples)
if nargin<4,nSamples=13;end
u=linspace(0,1,nSamples);
B14=bern_basis(14,u);B13=bern_basis(13,u);B12=bern_basis(12,u);
v=0;
for s=1:R.num_segments
    P=double(R.segments(s).payload_cp);
    T=double(R.segments(s).duration_s);
    if ~(isequal(size(P),[3,16])&&isfinite(T)&&T>0&& ...
            all(isfinite(P(:))))
        v=Inf;return;
    end
    V=15/T*diff(P,1,2);
    A=14/T*diff(V,1,2);
    J=13/T*diff(A,1,2);
    vv=V*B14;aa=A*B13;jj=J*B12;
    for k=1:nSamples
        f=aa(:,k)+[0;0;g];sig=norm(f);
        if ~(isfinite(sig)&&sig>128*eps(max([1,g,norm(f)])))
            v=Inf;return;
        end
        q=f/sig;qd=(jj(:,k)-q*(q'*jj(:,k)))/sig;
        vi=norm(vv(:,k)+L*qd);
        if ~isfinite(vi),v=Inf;return;end
        v=max(v,vi);
    end
end
end

function B=bern_basis(n,u)
B=zeros(n+1,numel(u));
for j=0:n
    B(j+1,:)=nchoosek(n,j)*(u.^j).*((1-u).^(n-j));
end
end

function w=endpoint_residual(R)
w=0;J=R.knot_jets;
for s=1:R.num_segments
    D=R.segments(s).payload_cp;
    T=R.segments(s).duration_s;
    for k=0:7
        w=max(w,max(abs([D(:,1)-J(:,k+1,s); ...
                       D(:,end)-J(:,k+1,s+1)]),[],'all'));
        if k<7,D=(size(D,2)-1)/T*diff(D,1,2);end
    end
end
end

function value=energy8(R)
% Diagnostic physical-time squared eighth derivative, NOT a safety score.
G=zeros(8);n=7;
for i=0:n
    for j=0:n
        G(i+1,j+1)=nchoosek(n,i)*nchoosek(n,j)/ ...
            ((2*n+1)*nchoosek(2*n,i+j));
    end
end
value=0;
for s=1:R.num_segments
    Z=R.segments(s).payload_cp;
    T=R.segments(s).duration_s;
    for d=1:8
        Z=(size(Z,2)-1)/T*diff(Z,1,2);
    end
    value=value+T*sum(sum((Z*G).*Z));
end
end
