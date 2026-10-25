function O = AF7R58R102_C7_CONTINUOUS_JET_OPT(R,g,L,vMax,opts)
% R102: optimize ALL shared interior C7 jets, rather than a 9-point alpha line.
% Generator objective uses sampled taut-model actual UAV speed; samples
% NEVER prove safety. Source and destination jets (orders 0..7) are fixed.
% Final separate R91/R80/R90 validation is mandatory before any usage.
% Uses base MATLAB only (no Optimization Toolbox, workers, or IO).
assert(isstruct(R)&&isscalar(R)&&R.reference_candidate_found, ...
    'R102:ValidExistingC7RouteRequired', 'Supply a valid multi-segment C7 route.');
assert(isstruct(opts)&&isscalar(opts), 'R102:OptionsRequired', 'Supply an options struct.');
assert(all(isfinite([g,L,vMax]))&&g>0&&L>0&&vMax>0, ...
    'R102:LiveParametersRequired', 'Live g, L, vMax must be positive finite numbers.');
if isfield(opts,'max_continuous_jet_steps'),N=double(opts.max_continuous_jet_steps);
else,N=60;end  % numerical work budget only; NOT a maneuver duration limit
if isfield(opts,'jet_objective_samples_per_segment')
    nSamples=double(opts.jet_objective_samples_per_segment);
else,nSamples=19;end  % optimization mesh, NOT a safety certificate
assert(isscalar(N)&&isfinite(N)&&N>=1&&N==fix(N)&& ...
       isscalar(nSamples)&&isfinite(nSamples)&&nSamples>=5&& ...
       nSamples==fix(nSamples), 'R102:InvalidNumericWorkBudget', ...
    'Numeric steps and samples must be positive finite integer budgets.');
O=struct('route',R,'status','NOT_OPTIMIZED', ...
    'iterations',0,'accepted_steps',0, ...
    'sampled_speed_before_m_s',NaN,'sampled_speed_after_m_s',NaN, ...
    'sampled_speed_certified',false,'production_commit_authorized',false, ...
    'external_jets_unchanged',false,'c7_shared_internal_jets',false);
K=numel(R.times_s);S=K-1;
assert(R.num_segments==S&&size(R.knot_jets,3)==K&& ...
    size(R.knot_jets,2)==8&&size(R.knot_jets,1)==3, ...
    'R102:KnotJetSchema', 'Route has inconsistent section/jet schema.');
if K<=2
    O.status='NO_INTERNAL_JETS_TO_OPTIMIZE';return;
end
U=7*(K-2); ns=S*nSamples;
BV=zeros(ns,U);BA=zeros(ns,U);BJ=zeros(ns,U);
V0=zeros(ns,3);A0=zeros(ns,3);J0=zeros(ns,3);
point=1;
for s=1:S
    h=double(R.segments(s).duration_s);
    assert(h>0&&isfinite(h)&&isequal(size(R.segments(s).payload_cp),[3,16]), ...
        'R102:SectionSchema', 'Each section must be a degree-15 Bezier.');
    bb14=local_bernstein(14,linspace(0,1,nSamples));
    bb13=local_bernstein(13,linspace(0,1,nSamples));
    bb12=local_bernstein(12,linspace(0,1,nSamples));
    p=double(R.segments(s).payload_cp);
    v=(15/h)*diff(p,1,2);
    a=(14/h)*diff(v,1,2);
    j=(13/h)*diff(a,1,2);
    idx=point:point+nSamples-1;
    V0(idx,:)=(v*bb14).';A0(idx,:)=(a*bb13).';
    J0(idx,:)=(j*bb12).';
    for k=max(2,s):min(K-1,s+1)
        isStart=(k==s);
        for d=1:7
            col=(k-2)*7+d;
            pBasis=local_jet_basis(d,h,isStart);
            vBasis=(15/h)*diff(pBasis);
            aBasis=(14/h)*diff(vBasis);
            jBasis=(13/h)*diff(aBasis);
            BV(idx,col)=(vBasis*bb14).';
            BA(idx,col)=(aBasis*bb13).';
            BJ(idx,col)=(jBasis*bb12).';
        end
    end
    point=point+nSamples;
end
% Precondition by measured influence of EACH interior jet on sampled speed.
% These are variable coordinates, not physical thresholds or fallback values.
sensitivity=max(abs(BV),[],1)+(L/g)*max(abs(BJ),[],1);
initialJerk=max(sqrt(sum(J0.^2,2)));
sensitivity=sensitivity+(L*initialJerk/g^2)*max(abs(BA),[],1);
if any(~isfinite(sensitivity))||any(sensitivity<=0)
    O.status='INCONCLUSIVE_JET_BASIS_CONDITIONING';return;
end
scale=vMax./sensitivity;
if any(~isfinite(scale))||any(scale<=0)
    O.status='INCONCLUSIVE_NONFINITE_SCALING';return;
end
BV=BV.*scale;BA=BA.*scale;BJ=BJ.*scale;
% Objective is smooth maximum of ACTUAL taut-model UAV speed over a
% numerical mesh. For each speed sample, exact differential is used for
% the physical map q=f/||f|| and qdot=(I-qq')j/||f||.
tau=vMax/sqrt(nSamples*S);  % dimensionally consistent numeric smoothing
Y=zeros(U,3);
[f,G,peak]=objective(Y,V0,A0,J0,BV,BA,BJ,g,L,tau);
O.sampled_speed_before_m_s=peak;
O.sampled_speed_after_m_s=peak;
if ~isfinite(f)
    O.status='INCONCLUSIVE_BASELINE_TAUT_MAP';return;
end
bestF=f;bestY=Y;bestPeak=peak;
for iter=1:N
    O.iterations=iter;
    gn=sqrt(sum(G(:).^2));
    if ~isfinite(gn)||gn<=64*eps(max(1,abs(f)))
        O.status='NUMERIC_JET_GRADIENT_STATIONARY';break;
    end
    D=-G/gn;step=1;accepted=false;
    for ls=1:22
        trial=Y+step*D;
        [f1,G1,peak1]=objective(trial,V0,A0,J0,BV,BA,BJ,g,L,tau);
        if isfinite(f1) && f1<f-1e-4*step*gn
            Y=trial;f=f1;G=G1;peak=peak1;accepted=true;
            O.accepted_steps=O.accepted_steps+1;
            if peak<bestPeak
                bestPeak=peak;bestY=Y;bestF=f;
            end
            break;
        end
        step=step/2;
    end
    if ~accepted
        O.status='NUMERIC_JET_LINE_SEARCH_STATIONARY';break;
    end
    if bestPeak<vMax-128*eps(max(1,vMax))
        O.status='SAMPLED_SPEED_BELOW_LIMIT__NOT_CERTIFIED';break;
    end
end
if strcmp(O.status,'NOT_OPTIMIZED')
    O.status='NUMERIC_JET_WORK_BUDGET_REACHED';
end
O.sampled_speed_after_m_s=bestPeak;
if bestPeak+64*eps(max(1,bestPeak))>=O.sampled_speed_before_m_s
    O.external_jets_unchanged=true;
    O.c7_shared_internal_jets=true;
    O.status='NO_NUMERIC_SPEED_IMPROVEMENT';
    return;
end
Z=R;
J=Z.knot_jets;
for k=2:K-1
    for d=1:7
        c=(k-2)*7+d;
        J(:,d+1,k)=J(:,d+1,k)+(scale(c)*bestY(c,:)).';
    end
end
for s=1:S
    Z.segments(s).payload_cp=local_cp(J(:,:,s),J(:,:,s+1), ...
        Z.segments(s).duration_s);
end
Z.knot_jets=J;
Z.status='R102_JOINT_INTERIOR_JETS_NUMERIC_GENERATION__FULL_CERT_REQUIRED';
Z.continuity_order=7;
Z.interior_peak_baseline_sample_m_s=O.sampled_speed_before_m_s;
Z.interior_peak_sample_m_s=O.sampled_speed_after_m_s;
Z.production_commit_authorized=false;
O.route=Z;
O.external_jets_unchanged=isequaln(J(:,:,1),R.knot_jets(:,:,1))&& ...
    isequaln(J(:,:,end),R.knot_jets(:,:,end));
O.c7_shared_internal_jets=true;
end

function [obj,gradient,peak]=objective(Y,V0,A0,J0,BV,BA,BJ,g,L,tau)
ns=size(V0,1);m=size(Y,1);
vel=V0+BV*Y;acc=A0+BA*Y;jerk=J0+BJ*Y;
speed=zeros(ns,1);dV=zeros(ns,3);dA=zeros(ns,3);dJ=zeros(ns,3);
obj=Inf;gradient=zeros(m,3);peak=Inf;
for z=1:ns
    f=acc(z,:).'+[0;0;g];sig=norm(f);
    if ~(isfinite(sig)&&sig>128*eps(max([1,g,norm(f)])))
        return;
    end
    q=f/sig;P=eye(3)-q*q.';j=jerk(z,:).';
    vQ=vel(z,:).'+L*(P*j)/sig;
    a=norm(vQ);
    if ~isfinite(a),return;end
    speed(z)=a;
    unit=vQ/max(a,realmin('double'));
    Da=-L/sig^2*((q.'*j)*P+q*(j.'*P)+(P*j)*q.');
    Dj=L*P/sig;
    dV(z,:)=unit.';
    dA(z,:)=(Da.'*unit).';
    dJ(z,:)=(Dj.'*unit).';
end
peak=max(speed);
ref=max(speed);
e=exp((speed-ref)/tau);
sumExp=sum(e);
if ~(isfinite(sumExp)&&sumExp>0),return;end
w=e/sumExp;
obj=ref+tau*log(sumExp/ns);
gradient=BV.'*(dV.*w)+BA.'*(dA.*w)+BJ.'*(dJ.*w);
end

function p=local_jet_basis(d,T,fromStart)
p=zeros(1,16);
for j=d:7
    f=nchoosek(j,d)*T^d/prod(15:-1:(16-d));
    if fromStart,p(j+1)=f;else,p(16-j)=(-1)^d*f;end
end
end
function P=local_cp(A,B,T)
P=zeros(3,16);
for j=0:7
    aa=zeros(3,1);bb=zeros(3,1);
    for d=0:j
        f=nchoosek(j,d)*T^d/prod(15:-1:(16-d));
        aa=aa+f*A(:,d+1);
        bb=bb+(-1)^d*f*B(:,d+1);
    end
    P(:,j+1)=aa;P(:,16-j)=bb;
end
end
function B=local_bernstein(n,u)
B=zeros(n+1,numel(u));
for k=0:n
    B(k+1,:)=nchoosek(n,k)*(u.^k).*((1-u).^(n-k));
end
end
