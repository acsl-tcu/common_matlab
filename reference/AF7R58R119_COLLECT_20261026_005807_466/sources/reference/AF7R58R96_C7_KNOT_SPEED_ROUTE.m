function R=AF7R58R96_C7_KNOT_SPEED_ROUTE(times,waypoints,startJets,endJets,phaseDelay,L,g,vMax,refineSteps)
% R95: speed-aware internal C7 knot jets, preserving EXACT external jets.
% Reference candidate ONLY. Per-knot necessary speed guards do NOT imply
% continuous speed, collision, actuator, IEEE or closed-loop certification.
% No geometric/duration cutoffs. All physical parameters are caller-supplied.
if nargin<9,refineSteps=20;end % purely numerical solver-work budget
assert(isnumeric(g)&&isscalar(g)&&isfinite(g)&&g>0&& ...
    isnumeric(vMax)&&isscalar(vMax)&&isfinite(vMax)&&vMax>0&& ...
    isnumeric(L)&&isscalar(L)&&isfinite(L)&&L>0&& ...
    isnumeric(refineSteps)&&isscalar(refineSteps)&& ...
    isfinite(refineSteps)&&refineSteps>=0&&refineSteps==fix(refineSteps), ...
    'R95:MissingExternalKinematicsOrBadWorkBudget');
base=AF7R58R92_C7_MINIMUM8_ROUTE(times,waypoints,startJets,endJets,phaseDelay,L);
R=base;
R.knot_speed_limit_m_s=vMax;
R.speed_aware_knot_adjusted=false;
R.knot_jet_blend_alpha=NaN;
R.knot_speed_necessary_pass=false;
R.original_min8_is_the_only_globally_energy_optimal_route=true;
R.adjusted_jet_solution_is_a_feasibility_heuristic=false;
R.production_commit_authorized=false;
R.nominal_trajectory_only=true;
R.knot_max_uav_speed_m_s=NaN;
R.base_min8_knot_max_uav_speed_m_s=NaN;
R.original_min8_energy=NaN;
if ~base.reference_candidate_found,return;end
K=numel(times);
R.original_min8_energy=base.minimum_8th_derivative_quadratic;
[pass0,mx0]=knot_speeds(base.knot_jets,g,L,vMax);
R.base_min8_knot_max_uav_speed_m_s=mx0;
if pass0
 R.knot_speed_necessary_pass=true;
 R.knot_max_uav_speed_m_s=mx0;
 R.knot_jet_blend_alpha=1;
 R.status='R95_ORIGINAL_MIN8_KNOTS_PASS__FULL_INTERVAL_UNPROVED';
 return;
end
if K==2
 R.status='R95_EXTERNAL_ENDPOINT_SPEED_UNCERTIFIED_NO_INTERNAL_KNOT';return;
end
% Boundary jets originate from live active + delayed nominal references.
% These are never altered to manufacture a successful candidate.
[pBoundary,sBoundary]=knot_speeds(base.knot_jets(:,:,[1 K]),g,L,vMax);
if ~pBoundary
 R.status='R95_EXTERNAL_BOUNDARY_SPEED_VIOLATION__NO_REFERENCE_COMMIT';
 R.knot_max_uav_speed_m_s=sBoundary;return;
end
J0=base.knot_jets;
Janchor=J0;
for k=2:K-1
    left=(waypoints(:,k)-waypoints(:,k-1))/(times(k)-times(k-1));
    right=(waypoints(:,k+1)-waypoints(:,k))/(times(k+1)-times(k));
    v=(left+right)/2; % physical-time secant tangent, not a forced stop
    nv=norm(v);
    if nv>=vMax
        % This is a numerical strictness margin relative to the EXTERNAL
        % velocity limit, not a new planner velocity/time threshold.
        v=v*(vMax*(1-1024*eps(1))/nv);
    end
    Janchor(:,2,k)=v;
    Janchor(:,3:8,k)=0; % feasible anchor; final jets selected by continuation
end
[anchorPass,anchorMax]=knot_speeds(Janchor,g,L,vMax);
if ~anchorPass
 R.status='R95_NO_FEASIBLE_KNOT_ANCHOR';
 R.knot_max_uav_speed_m_s=anchorMax;return;
end
% J(alpha) interpolates between the unconstrained global min-p8 solution
% and a speed-feasible tangent anchor; ALL knot jets remain shared between
% neighboring segments, hence C7 for every alpha. This is a 1D feasible
% continuation, not a global constrained optimality theorem.
lo=0;hi=1;Jbest=Janchor;bestSpeed=anchorMax;
for n=1:refineSteps
    mid=(lo+hi)/2;
    if ~(mid>lo && mid<hi),break;end
    Jtry=mix_jets(Janchor,J0,mid);
    [ok,mx]=knot_speeds(Jtry,g,L,vMax);
    if ok
        lo=mid;Jbest=Jtry;bestSpeed=mx;
    else
        hi=mid;
    end
end
R.knot_jet_blend_alpha=lo;
R.knot_max_uav_speed_m_s=bestSpeed;
R.speed_aware_knot_adjusted=true;
R.adjusted_jet_solution_is_a_feasibility_heuristic=true;
R.original_min8_is_the_only_globally_energy_optimal_route=false;
R.knot_speed_necessary_pass=true;
R.knot_jets=Jbest;
S=K-1;
for s=1:S
    T=times(s+1)-times(s);
    P=from_jets(Jbest(:,:,s),Jbest(:,:,s+1),T);
    if any(~isfinite(P(:)))
        R.status='R95_NONFINITE_CONSTRAINED_CONTROL_POLYGON';
        R.reference_candidate_found=false;return;
    end
    R.segments(s).payload_cp=P;
end
worst=0;energy=0;
for s=1:S
    T=times(s+1)-times(s);D=R.segments(s).payload_cp;
    for d=0:7
        worst=max(worst,max(abs([D(:,1)-Jbest(:,d+1,s); ...
            D(:,end)-Jbest(:,d+1,s+1)]),[],'all'));
        if d<7,D=(size(D,2)-1)/T*diff(D,1,2);end
    end
    Z=R.segments(s).payload_cp;
    for d=1:8,Z=(size(Z,2)-1)/T*diff(Z,1,2);end
    % Integral of squared degree-7 Bernstein polynomial, exact in real arithmetic.
    n=7;G=zeros(8);
    for i=0:n
        for j=0:n
            G(i+1,j+1)=nchoosek(n,i)*nchoosek(n,j)/ ...
                ((2*n+1)*nchoosek(2*n,i+j));
        end
    end
    energy=energy+T*sum(sum((Z*G).*Z));
end
R.max_jet_residual=worst;
R.minimum_8th_derivative_quadratic=energy;
R.status='R95_C7_KNOT_SPEED_NECESSARY_PASS__FULL_INTERVAL_UNPROVED';
end

function J=mix_jets(A,B,alpha)
J=A;
if size(A,3)>2
    J(:,:,2:end-1)=A(:,:,2:end-1)+ ...
        alpha*(B(:,:,2:end-1)-A(:,:,2:end-1));
end
end

function [ok,mx]=knot_speeds(J,g,L,vMax)
mx=0;ok=true;
for k=1:size(J,3)
    v=J(:,2,k);a=J(:,3,k);j=J(:,4,k);
    f=a+[0;0;g];sigma=norm(f);
    if ~(isfinite(sigma)&&sigma>128*eps(max([1,g,norm(a)])))
        ok=false;mx=Inf;return;
    end
    q=f/sigma;qdot=(j-q*(q'*j))/sigma;
    vv=norm(v+L*qdot);mx=max(mx,vv);
    tol=128*eps(max([1,vMax,vv]));
    if ~(isfinite(vv)&&vv<vMax-tol)
        ok=false;
    end
end
end

function P=from_jets(A,B,T)
D=size(A,1);P=zeros(D,16);
for j=0:7
    x=zeros(D,1);y=zeros(D,1);
    for r=0:j
        falling=prod(15:-1:(16-r));
        fac=nchoosek(j,r)*T^r/falling;
        x=x+fac*A(:,r+1);
        y=y+(-1)^r*fac*B(:,r+1);
    end
    P(:,j+1)=x;P(:,16-j)=y;
end
end
