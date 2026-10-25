function R = AF7R58R90_INPUT_ENVELOPE(C,P,maxNodes)
% R90: Input constraints for a TAUT straight-cable, flatness nominal path.
% Does not certify HLC actual commands, body torque, model uncertainty,
% cable deformation, obstacles, or post-connector nominal continuation.
%
% All physical constants and input limits MUST be supplied from the
% SAME runtime authority as the trajectory under test. No defaults.
% C.payload_cp : 3x16 degree-15 payload Bezier in world coordinates.
% P.gravity_m_s2, P.cable_length_m, P.vehicle_mass_kg,
% P.latest_EKF_mL_kg, P.thrust_bounds_N (1x2),
% P.torque_bounds_Nm (3x2) are required.  Optional P.tension_max_N.
% maxNodes is a COMPUTATIONAL budget, not a maneuver duration cap.
req={'gravity_m_s2','cable_length_m','vehicle_mass_kg', ...
    'latest_EKF_mL_kg','thrust_bounds_N','torque_bounds_Nm'};
assert(isstruct(P)&&isscalar(P),'R90:RuntimeContractRequired');
for i=1:numel(req)
    assert(isfield(P,req{i}), '[R90] missing live authority: %s',req{i});
end
assert(isstruct(C)&&isfield(C,'payload_cp')&& ...
    isequal(size(C.payload_cp),[3,16])&&all(isfinite(C.payload_cp(:)))&& ...
    isfield(C,'duration_s')&&isscalar(C.duration_s)&& ...
    isfinite(C.duration_s)&&C.duration_s>0&& ...
    isfield(C,'t0')&&isfield(C,'t1')&& ...
    isfinite(C.t0)&&isfinite(C.t1)&&C.t1>C.t0, ...
    'R90:InvalidC7Bezier');
assert(abs((C.t1-C.t0)-C.duration_s) <= ...
    64*eps(max([1,abs(C.t0),abs(C.t1),C.duration_s])), ...
    'R90:InconsistentTimeAuthority');
assert(all(isfinite([P.gravity_m_s2,P.cable_length_m, ...
    P.vehicle_mass_kg,P.latest_EKF_mL_kg])) && ...
    all([P.gravity_m_s2,P.cable_length_m, ...
    P.vehicle_mass_kg,P.latest_EKF_mL_kg]>0), ...
    'R90:InvalidPhysicalAuthority');
assert(isnumeric(P.thrust_bounds_N)&& ...
    isequal(size(P.thrust_bounds_N),[1 2])&& ...
    all(isfinite(P.thrust_bounds_N))&& ...
    P.thrust_bounds_N(1)>=0&& ...
    P.thrust_bounds_N(2)>P.thrust_bounds_N(1), ...
    'R90:InvalidThrustBounds');
assert(isnumeric(P.torque_bounds_Nm)&& ...
    isequal(size(P.torque_bounds_Nm),[3 2])&& ...
    all(isfinite(P.torque_bounds_Nm(:)))&& ...
    all(P.torque_bounds_Nm(:,1)<P.torque_bounds_Nm(:,2)), ...
    'R90:InvalidTorqueBounds');
assert(isnumeric(maxNodes)&&isscalar(maxNodes)&& ...
    isfinite(maxNodes)&&maxNodes==fix(maxNodes)&&maxNodes>=1, ...
    'R90:InvalidWorkBudget');
capKnown=isfield(P,'tension_max_N');
if capKnown
    assert(isnumeric(P.tension_max_N)&&isscalar(P.tension_max_N)&& ...
        isfinite(P.tension_max_N)&&P.tension_max_N>0, ...
        'R90:InvalidTensionStrength');
end
R=struct('status','INCONCLUSIVE_NOT_STARTED', ...
    'taut_tension_positive_certified',false, ...
    'cable_strength_certified',false, ...
    'nominal_thrust_certified',false, ...
    'nominal_torque_certified',false, ...
    'closed_loop_input_certified',false, ...
    'production_commit_authorized',false, ...
    'nodes',0,'safe_leaves',0,'witness_interval_s',[NaN NaN], ...
    'tension_lower_N',NaN,'tension_upper_N',NaN, ...
    'nominal_thrust_lower_N',NaN,'nominal_thrust_upper_N',NaN, ...
    'start_thrust_N',NaN,'end_thrust_N',NaN, ...
    'start_tension_N',NaN,'end_tension_N',NaN, ...
    'torque_status','NOT_CERTIFIED_NEEDS_SOURCE_BOUND_HLC_INTERVAL_OPERATOR', ...
    'limitations',['Reference/taut-cable only; numerical Bezier bounds ', ...
      'without directed-rounding proof; NOT real HLC closed-loop safety']);
g=P.gravity_m_s2; L=P.cable_length_m;
mQ=P.vehicle_mass_kg; mL=P.latest_EKF_mL_kg;
CP=double(C.payload_cp); h=double(C.duration_s);
[R.start_thrust_N,R.start_tension_N,ok0]=at_endpoint(CP,h,g,L,mQ,mL,true);
[R.end_thrust_N,R.end_tension_N,ok1]=at_endpoint(CP,h,g,L,mQ,mL,false);
if ~ok0||~ok1
    R.status='INCONCLUSIVE_ENDPOINT_SPECIFIC_FORCE';return;
end
scale=max([1,P.thrust_bounds_N,R.start_thrust_N,R.end_thrust_N]);
slack=128*eps(scale);
if R.start_thrust_N > P.thrust_bounds_N(2)+slack || ...
   R.end_thrust_N > P.thrust_bounds_N(2)+slack || ...
   R.start_thrust_N < P.thrust_bounds_N(1)-slack || ...
   R.end_thrust_N < P.thrust_bounds_N(1)-slack
    R.status='NECESSARY_ENDPOINT_THRUST_LIMIT_VIOLATED';
    return;
end
if capKnown && (R.start_tension_N>P.tension_max_N+slack || ...
                R.end_tension_N>P.tension_max_N+slack)
    R.status='NECESSARY_ENDPOINT_CABLE_STRENGTH_LIMIT_VIOLATED';
    return;
end
stack=struct('cp',CP,'a',double(C.t0),'b',double(C.t1));
fLo=Inf;fHi=0;TL=Inf;TU=0;
while ~isempty(stack)
    Z=stack(end);stack(end)=[];
    if R.nodes>=maxNodes
        R.status='INCONCLUSIVE_WORK_BUDGET';
        R.witness_interval_s=[Z.a Z.b];return;
    end
    R.nodes=R.nodes+1;
    [okay,lo,hi,tlo,thi]=bound_leaf(Z.cp,Z.b-Z.a,g,L,mQ,mL, ...
        P.thrust_bounds_N,capKnown,P);
    if okay
        R.safe_leaves=R.safe_leaves+1;
        fLo=min(fLo,lo);fHi=max(fHi,hi);
        TL=min(TL,tlo);TU=max(TU,thi);
        continue;
    end
    mid=Z.a+(Z.b-Z.a)/2;
    if ~(isfinite(mid)&&mid>Z.a&&mid<Z.b)
        R.status='INCONCLUSIVE_FLOAT_TIME_RESOLUTION';
        R.witness_interval_s=[Z.a Z.b];return;
    end
    [A,B]=split_half(Z.cp);
    stack(end+1)=struct('cp',B,'a',mid,'b',Z.b); %#ok<AGROW>
    stack(end+1)=struct('cp',A,'a',Z.a,'b',mid); %#ok<AGROW>
end
R.tension_lower_N=TL;R.tension_upper_N=TU;
R.nominal_thrust_lower_N=fLo;R.nominal_thrust_upper_N=fHi;
R.taut_tension_positive_certified=true;
R.cable_strength_certified=capKnown;
R.nominal_thrust_certified=true;
R.status='CONSERVATIVE_NUMERIC_NOMINAL_TENSION_AND_THRUST_ONLY';
end

function [pass,forceLo,forceHi,tensionLo,tensionHi]=bound_leaf(X,h,g,L,mQ,mL,lim,capKnown,P)
pass=false;forceLo=NaN;forceHi=NaN;tensionLo=NaN;tensionHi=NaN;
if ~(isfinite(h)&&h>0),return;end
Ac=deriv_cp(X,h,2);Jc=deriv_cp(X,h,3);Sc=deriv_cp(X,h,4);
if any(~isfinite([Ac(:);Jc(:);Sc(:)])),return;end
F=Ac+[zeros(2,size(Ac,2));g*ones(1,size(Ac,2))];
sigmaHi=max(vecnorm(F,2,1));
a=mean(F,2);an=norm(a);if ~(isfinite(an)&&an>0),return;end
a=a/an;
sigmaLo=min(a'*F);
tol=128*eps(max([1,g,sigmaHi]));
if ~(isfinite(sigmaLo)&&sigmaLo>tol),return;end
J=max(vecnorm(Jc,2,1));S=max(vecnorm(Sc,2,1));
% q=f/||f||, qddot=P(q)*snap/||f|| -2*qdot*(q'jerk)/||f|| -q*||qdot||^2
% ||qddot|| <= S/sigmaLo + 3*(J/sigmaLo)^2.
qddHi=S/sigmaLo+3*(J/sigmaLo)^2;
forceHi=(mQ+mL)*sigmaHi+mQ*L*qddHi;
forceLo=max(0,(mQ+mL)*sigmaLo-mQ*L*qddHi);
tensionLo=mL*sigmaLo;tensionHi=mL*sigmaHi;
if ~(all(isfinite([forceLo,forceHi,tensionLo,tensionHi]))&& ...
    tensionLo>0),return;end
okForce=(forceHi < lim(2)-tol) && ...
    (lim(1)==0 || forceLo > lim(1)+tol);
okStrength=~capKnown || tensionHi < P.tension_max_N-tol;
pass=okForce&&okStrength;
end

function [thrust,tension,ok]=at_endpoint(P,h,g,L,mQ,mL,start)
a=deriv_cp(P,h,2);j=deriv_cp(P,h,3);s=deriv_cp(P,h,4);
if start
    a=a(:,1);j=j(:,1);s=s(:,1);
else
    a=a(:,end);j=j(:,end);s=s(:,end);
end
f=a+[0;0;g];sig=norm(f);thrust=NaN;tension=NaN;ok=false;
if ~isfinite(sig)||sig<=128*eps(max([1,g,norm(a)])),return;end
q=f/sig;qd=(j-q*(q'*j))/sig;
qdd=(s-q*(q'*s))/sig-2*qd*(q'*j)/sig-q*(qd'*qd);
F=(mQ+mL)*f+mQ*L*qdd;
thrust=norm(F);tension=mL*sig;
ok=isfinite(thrust)&&isfinite(tension)&&tension>0;
end

function D=deriv_cp(P,h,order)
D=P;
for r=1:order
    D=(size(D,2)-1)/h*diff(D,1,2);
end
end
function [A,B]=split_half(P)
N=size(P,2);A=zeros(size(P));B=zeros(size(P));
A(:,1)=P(:,1);B(:,end)=P(:,end);
for k=2:N
    P=(P(:,1:end-1)+P(:,2:end))/2;
    A(:,k)=P(:,1);B(:,N-k+1)=P(:,end);
end
end
