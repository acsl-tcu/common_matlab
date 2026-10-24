function OUT = AF7R58R60_C7_FREE_REJOIN(IN)
% Source-independent, NON-COMMITTABLE candidate synthesizer.
% Required inputs (no runtime/system-parameter defaults):
%   t0_s, initial_duration_s, phase_delay_s, phase_slowdown_beta
%   phase_start_jets_0_to_7 (1x8 current PHASE physical-time jets)
%   active_jets_0_to_7 (3x8 physical-time derivative columns)
%   nominal_jets_fn (function handle tau -> 3x8 derivative columns)
%   nominal_position_fn (function handle tau -> 3x1)
%   max_trials (computational resource allowance, NOT a flight-time cap)
% Optional representation-only options: growth_factor (>1), enforce_scalar_monotone.
%
% Output: candidate geometry and diagnostics only; NEVER a safety certificate.
% p_new(t) = p_nom(s(t)) + d(t), with s'(t) in [0,1], s(t)<=t,
% phase derivatives 0..7 match the CURRENT active phase at t0; after t1
% s(t)=t-DeltaFinal. A Bernstein-hull certificate enforces 0<=s'<=1.
% d(t) is a degree-15 C7 endpoint-Hermite correction.  Starting physical
% jets 0..7 are EXACTLY joined in mathematical real arithmetic, regardless
% of the choice of candidate duration.  Endpoint correction jets 0..7=0.
% The phase slowdown is a degree-17 polynomial, NOT a sampled time warp.
% It maintains s'<=1; no same-clock catch-up is ever attempted.
%
% Important: This does not certify q singularity, heading regularity,
% actuator/tension feasibility, whole-body obstacle clearance, or real HLC.
% A candidate must pass the independent existing hard certificate BEFORE commit.

required={'t0_s','initial_duration_s','phase_delay_s','phase_slowdown_beta', ...
    'phase_start_jets_0_to_7','active_jets_0_to_7','nominal_jets_fn','nominal_position_fn','max_trials'};
for k=1:numel(required)
    assert(isstruct(IN)&&isfield(IN,required{k}), ...
        'R60:MissingInput','Mandatory input missing: %s',required{k});
end
assert(isscalar(IN.t0_s)&&isfinite(IN.t0_s),'R60:Time','Bad start time');
assert(isscalar(IN.initial_duration_s)&&isfinite(IN.initial_duration_s) ...
    && IN.initial_duration_s>0,'R60:Duration','Bad input duration');
assert(isscalar(IN.phase_delay_s)&&isfinite(IN.phase_delay_s) ...
    && IN.phase_delay_s>=0,'R60:Delay','Bad initial delay');
assert(isscalar(IN.phase_slowdown_beta)&&isfinite(IN.phase_slowdown_beta) ...
    && IN.phase_slowdown_beta>=0 && IN.phase_slowdown_beta<=1, ...
    'R60:Rate','Beta must be in [0,1] for phase rate in [0,1]');
assert(isscalar(IN.max_trials)&&isfinite(IN.max_trials) ...
    && IN.max_trials==fix(IN.max_trials)&&IN.max_trials>=1, ...
    'R60:Budget','max_trials must be a positive integer');
S0=double(IN.phase_start_jets_0_to_7(:).');
assert(isequal(size(S0),[1 8])&&all(isfinite(S0)) && ...
    abs(S0(1)-(double(IN.t0_s)-double(IN.phase_delay_s)))<= ...
    64*eps(max([1,abs(S0(1)),abs(double(IN.t0_s))])), ...
    'R60:Phase','Active phase start value conflicts with delay/time');
J=double(IN.active_jets_0_to_7);
assert(isequal(size(J),[3 8])&&all(isfinite(J),'all'), ...
    'R60:Jets','Active reference must supply physical-time jets 0..7');
assert(isa(IN.nominal_jets_fn,'function_handle') && ...
    isa(IN.nominal_position_fn,'function_handle'), ...
    'R60:Nominal','Nominal derivative and position callbacks are mandatory');
GF=2; % Representation-search growth only; no accepted maneuver-time ceiling.
if isfield(IN,'growth_factor')
    GF=double(IN.growth_factor);
end
assert(isscalar(GF)&&isfinite(GF)&&GF>1,'R60:Growth','Growth factor must exceed 1');
needProjected=false;
if isfield(IN,'enforce_projected_no_overshoot')
    needProjected=logical(IN.enforce_projected_no_overshoot);
end
needMonotone=false;
if isfield(IN,'enforce_scalar_monotone')
    needMonotone=logical(IN.enforce_scalar_monotone);
end
OUT=struct('status','UNRESOLVED_RESOURCE_BUDGET', ...
    'commit_allowed',false,'wholebody_safety_certified',false, ...
    'hlc_execution_certified',false,'tube_certified',false, ...
    'numerical_candidate_found',false,'candidate',struct(), ...
    'attempts',repmat(struct('T_s',NaN,'phase_delay_end_s',NaN, ...
    'start_jet_abs_error',NaN,'end_jet_abs_error',NaN, ...
    'monotone_control_polygon',false,'finite',false),1,0));
T=double(IN.initial_duration_s); t0=double(IN.t0_s);
delta0=double(IN.phase_delay_s); beta=double(IN.phase_slowdown_beta);
for it=1:double(IN.max_trials)
    try
        tau0=S0(1);
        F0=double(IN.nominal_jets_fn(tau0));
        assert(isequal(size(F0),[3 8]) && all(isfinite(F0),'all'), ...
            'R60:NominalJet','Invalid nominal jets at active phase');
        J0=compose_nominal_and_phase_jets(F0,S0);
        dj=J-J0;
        % Degree-15 Bernstein control points from start physical-time jets.
        cp=zeros(3,16);
        dif=zeros(3,8);
        for k=0:7
            dif(:,k+1)=dj(:,k+1)*T^k/falling(15,k);
        end
        for i=0:7
            for k=0:i
                cp(:,i+1)=cp(:,i+1)+nchoosek(i,k)*dif(:,k+1);
            end
        end
        % The last eight Bernstein points are exactly zero. Hence d^(0..7)(T)=0.
        % Bernstein control points of s'(t), degree 16. The first seven
        % enforce arbitrary inherited phase derivatives 1..7 (thus no phase
        % discontinuity on replanning during a previous phase transition).
        % The last seven force unit terminal rate and its derivatives 1..6=0.
        % The three free middle controls implement a slowdown without ever
        % asking s'(t)>1. If ANY control exceeds [0,1], reject this trial:
        % no sampled or heuristic rate claim is allowed.
        vcp=ones(1,17);
        phaseDiff=zeros(1,7);
        for k=0:6
            phaseDiff(k+1)=S0(k+2)*T^k/falling(16,k);
        end
        for i=0:6
            vcp(i+1)=0;
            for k=0:i
                vcp(i+1)=vcp(i+1)+nchoosek(i,k)*phaseDiff(k+1);
            end
        end
        vcp(8:10)=1-beta;
        phaseRateValid=all(isfinite(vcp)) && ...
            all(vcp>=0) && all(vcp<=1);
        scp=zeros(1,18);scp(1)=tau0;
        for i=1:17
            scp(i+1)=scp(i)+T*vcp(i)/17;
        end
        deltaEnd=t0+T-scp(end);
        Jend=double(IN.nominal_jets_fn(t0+T-deltaEnd));
        assert(isequal(size(Jend),[3 8])&&all(isfinite(Jend),'all'), ...
            'R60:NominalJet','Invalid nominal jets at delayed endpoint');
        jd0=bernstein_endpoint_jets(cp,T,'start');
        jd1=bernstein_endpoint_jets(cp,T,'end');
        err0=max(abs(J0(:)+jd0(:)-J(:)));
        err1=max(abs(jd1(:)));
        % For nonzero net displacement, check along-direction monotone envelope
        % and transverse CP overshoot.  This is a quality screen, NOT safety.
        a=cp(:,1); nd=norm(a);
        monotone=false;
        if nd>0
            lambda=(a.'*cp)/(nd^2);
            perp=cp-a*lambda;
            tol=1e-10*max(1,norm(cp,'fro'));
            monotone=all(diff(lambda)<=tol) && all(lambda>=-tol) && ...
                all(lambda<=1+tol) && all(vecnorm(perp)<=tol);
        else
            monotone=(norm(cp,'fro')<=1e-12);
        end
        % The Bernstein convex-hull property gives a WHOLE-INTERVAL
        % monotone signed return in the initial correction direction:
        % b_i = <d_0,P_i>/||d_0||^2 must be nonincreasing in [0,1].
        % A separate control-point norm screen prevents the correction's
        % spatial envelope expanding beyond its starting magnitude.
        % This screens the OFFSET, not the full world-path derivative, and
        % is NOT a substitute for obstacle/yaw/tension certification.
        projectNoOvershoot=false;
        if nd>0
            lambda=(a.'*cp)/(nd^2);
            normcp=vecnorm(cp,2,1);
            roundTol=512*eps(max(1,nd));
            projectNoOvershoot=all(diff(lambda)<=roundTol) && ...
                all(lambda>=-roundTol) && all(lambda<=1+roundTol) && ...
                all(normcp<=nd+roundTol);
        else
            projectNoOvershoot=all(vecnorm(cp,2,1)<=512*eps(1));
        end
        finite=phaseRateValid && all(isfinite(cp),'all') && all(isfinite(scp)) && ...
            isfinite(deltaEnd)&&deltaEnd>=delta0-1e-10;
        OUT.attempts(end+1)=struct('T_s',T, ...
            'phase_delay_end_s',deltaEnd, ...
            'start_jet_abs_error',err0,'end_jet_abs_error',err1, ...
            'monotone_control_polygon',monotone,'finite',finite);
        % Search T for a monotone geometry if requested. The search count is
        % explicitly a CPU-budget stop, not a physical maneuver horizon.
        if finite && (~needMonotone || monotone) && ...
                (~needProjected || projectNoOvershoot)
            C=struct('t0_s',t0,'t1_s',t0+T,'T_s',T, ...
                'phase_bezier17',scp,'phase_rate_bezier16',vcp, ...
                'offset_bezier15',cp, ...
                'phase_delay_initial_s',delta0, ...
                'phase_delay_final_s',deltaEnd, ...
                'phase_slowdown_beta',beta, ...
                'no_same_clock_catchup',true, ...
                'phase_rate_nonnegative',true, ...
                'monotone_offset_control_polygon',monotone, ...
                'projected_no_overshoot_control_polygon', ...
                    logical(projectNoOvershoot), ...
                'active_start_jets',J,'active_phase_start_jets',S0, ...
                'nominal_start_jets',J0, ...
                'nominal_end_jets',Jend, ...
                'start_jet_error',err0,'end_jet_error',err1, ...
                'numeric_only',true);
            OUT.candidate=C;OUT.numerical_candidate_found=true;
            OUT.status='C7_GEOMETRIC_CANDIDATE_UNCERTIFIED';
            return;
        end
    catch ME
        if strcmp(ME.identifier,'R60:NominalJet')
            rethrow(ME);
        end
        OUT.status=['UNRESOLVED_CANDIDATE_ERROR__' ME.identifier];
    end
    T=T*GF;
    if ~isfinite(T)
        OUT.status='UNRESOLVED_FLOAT_DURATION_OVERFLOW';
        return;
    end
end
end

function x=falling(n,k)
x=1;
for i=0:k-1
    x=x*(n-i);
end
end

function J=bernstein_endpoint_jets(P,T,side)
n=size(P,2)-1;
J=zeros(3,8);D=P;
for k=0:7
    if strcmp(side,'start')
        J(:,k+1)=D(:,1)*falling(n,k)/T^k;
    else
        J(:,k+1)=D(:,end)*falling(n,k)/T^k;
    end
    D=diff(D,1,2);
end
end

function J=compose_nominal_and_phase_jets(F,S)
% Truncated univariate Taylor composition to order 7 (Faà di Bruno without
% cancellation-prone symbolic expansion). F(:,m+1)=d^m p_nom/ds^m,
% S(m+1)=d^m s/dt^m. Returned jets are physical-time derivatives.
u=zeros(1,8);
for k=1:7
    u(k+1)=S(k+1)/factorial(k);
end
Jcoef=zeros(3,8);
power=zeros(1,8); power(1)=1;
for m=0:7
    Jcoef=Jcoef+(F(:,m+1)/factorial(m))*power;
    new=zeros(1,8);
    for i=0:7
        for j=0:7-i
            new(i+j+1)=new(i+j+1)+power(i+1)*u(j+1);
        end
    end
    power=new;
end
J=Jcoef;
for k=0:7
    J(:,k+1)=Jcoef(:,k+1)*factorial(k);
end
end
