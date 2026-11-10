function R=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT(live_lower,live_upper, ...
    beta2_interval_3x3x2,rhs_interval_3x2,thrust_interval_1x2,solver_mode)
%TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT  3x3 HLC backslash interval lemma.
% ACTUAL HLC_SUSPENDED_LOAD.do algebra: us=beta2\rhs before saturation,
% tmp=[Uf(1);us].  For ALL t of the model/corridor time horizon, require
% externally PROVEN pointwise enclosures of the real-model beta2, rhs, Uf.
% Given interval matrix B and interval rhs r, Cramer's identity says
% us_i = det(B with column i replaced by r) / det(B).
% Natural interval extensions of all determinants and division enclose
% every real-valued solution if 0 is EXCLUDED from det(B)'s enclosure.
% No perturbation norm, nominal trajectory-specific axis approximation,
% sampling-only input PASS, arbitrary velocity floor or tuning weight.
%
% This is a REAL-ARITHMETIC CONDITIONAL proof. Ordinary MATLAB doubles
% are not directed-rounded interval numbers. Incoming enclosures need a
% genuine mathematical all-time justification; merely filling fields
% called full_time_* does not prove them. NEVER grants safe_to_commit.
R=struct('status','UNKNOWN_DIRECT_INTERVAL_INPUT_REQUIRED', ...
    'safe_to_commit',false,'physical_input_certified',false, ...
    'conditional_headroom_all_positive',false, ...
    'conditional_raw_input_enclosure',NaN(4,2), ...
    'minimum_conditional_headroom',NaN, ...
    'determinant_interval',NaN(1,2), ...
    'numerator_intervals',NaN(3,2), ...
    'interval_invertibility_certified',false, ...
    'neumann_regular_certified',false, ...
    'neumann_contraction_inf',NaN, ...
    'neumann_input_interval',NaN(3,2), ...
    'method','CRAMER_3X3_INTERVAL_REAL_ARITHMETIC', ...
    'scope','CONDITIONAL_ALL_TIME_HLC_BETA2_RHS_UF_INTERVALS_ONLY');
if nargin<6
    solver_mode='CRAMER'; % exact legacy behaviour unless explicitly enabled
end
if isstring(solver_mode)&&isscalar(solver_mode),solver_mode=char(solver_mode);end
if ~(ischar(solver_mode)&&isrow(solver_mode)&& ...
        any(strcmp(solver_mode,{'CRAMER','NEUMANN','COMBINED'})))
    R.status='UNKNOWN_DIRECT_INTERVAL_SOLVER_MODE_INVALID';return
end
if ~(isnumeric(live_lower)&&isreal(live_lower)&& ...
        isequal(size(live_lower),[4,1])&& ...
        isnumeric(live_upper)&&isreal(live_upper)&& ...
        isequal(size(live_upper),[4,1])&& ...
        isnumeric(beta2_interval_3x3x2)&& ...
        isreal(beta2_interval_3x3x2)&& ...
        isequal(size(beta2_interval_3x3x2),[3,3,2])&& ...
        isnumeric(rhs_interval_3x2)&&isreal(rhs_interval_3x2)&& ...
        isequal(size(rhs_interval_3x2),[3,2])&& ...
        isnumeric(thrust_interval_1x2)&&isreal(thrust_interval_1x2)&& ...
        isequal(size(thrust_interval_1x2),[1,2]))
    return
end
lo=double(live_lower);hi=double(live_upper);
B=double(beta2_interval_3x3x2);
r=double(rhs_interval_3x2);
f=double(thrust_interval_1x2);
if any(~isfinite([lo;hi;B(:);r(:);f(:)])) || ...
        any(lo>=hi)||lo(1)<0|| ...
        any(B(:,:,1)>B(:,:,2),'all')||any(r(:,1)>r(:,2))||f(1)>f(2)
    R.status='UNKNOWN_DIRECT_INTERVAL_INVALID_BOUNDS';return
end
% Optional preconditioned interval linear-solve sufficient certificate.
% With C=inv(mid(B)), E=I-C*B and rho(|E|)<1, the exact solution obeys
%   |u-u0| <= (I-|E|)^(-1) * |C*(r-B*u0)|,
% u0=C*mid(r).  The comparison-matrix inverse is nonnegative.
% This bounds EVERY solution of B*u=r, even when the natural interval
% determinant is too wide to exclude zero. No guessed numerical radius.
raw=zeros(4,2);raw(1,:)=f;
cramer_ok=false;neumann_ok=false;
if any(strcmp(solver_mode,{'CRAMER','COMBINED'}))
    d=det3(B);
    R.determinant_interval=d;
    if ~(d(1)<=0&&d(2)>=0)
        cr=zeros(3,2);
        invdet=[1/d(2),1/d(1)];
        for i=1:3
            Bi=B;
            Bi(:,i,1)=r(:,1);Bi(:,i,2)=r(:,2);
            nu=det3(Bi);
            R.numerator_intervals(i,:)=nu;
            cr(i,:)=mul(nu,invdet);
        end
        cramer_ok=all(isfinite(cr(:)))&&all(cr(:,1)<=cr(:,2));
    end
end
if any(strcmp(solver_mode,{'NEUMANN','COMBINED'}))
    [neumann_ok,nbox,contract]=preconditioned_interval_solve(B,r);
    R.neumann_contraction_inf=contract;
    if neumann_ok
        R.neumann_regular_certified=true;
        R.neumann_input_interval=nbox;
    end
end
if cramer_ok&&neumann_ok
    % Both are enclosures of the SAME all-time 3-vector input. Intersection
    % is sound in real arithmetic and never widens either bound.
    tight=[max(cr(:,1),nbox(:,1)),min(cr(:,2),nbox(:,2))];
    if any(tight(:,1)>tight(:,2))
        R.status='UNKNOWN_DISJOINT_HLC_INTERVAL_EXTENSIONS';return
    end
    raw(2:4,:)=tight;
    R.method='INTERSECTED_CRAMER_AND_PRECONDITIONED_REAL_ARITHMETIC';
elseif cramer_ok
    raw(2:4,:)=cr;
    R.method='CRAMER_3X3_INTERVAL_REAL_ARITHMETIC';
elseif neumann_ok
    raw(2:4,:)=nbox;
    R.method='PRECONDITIONED_INTERVAL_REAL_ARITHMETIC';
else
    R.status='UNKNOWN_HLC_BETA2_INTERVAL_MAY_BE_SINGULAR';return
end
R.interval_invertibility_certified=true;
if any(~isfinite(raw(:))) || any(raw(:,1)>raw(:,2))
    R.status='UNKNOWN_HLC_DIRECT_INTERVAL_NONFINITE';return
end
R.conditional_raw_input_enclosure=raw;
R.minimum_conditional_headroom=min([raw(:,1)-lo;hi-raw(:,2)]);
R.conditional_headroom_all_positive=all(raw(:,1)>=lo & raw(:,2)<=hi);
if R.conditional_headroom_all_positive
    R.status='CONDITIONAL_HLC_DIRECT_INTERVAL_PRECLAMP_FOUR_INPUT__NOT_EXECUTION_PROVEN';
else
    R.status='UNKNOWN_HLC_DIRECT_INTERVAL_FOUR_INPUT_HEADROOM';
end
end

function [ok,box,eta]=preconditioned_interval_solve(B,r)
% Exact-real-arithmetic interval fixed-point enclosure. Regularity
% sufficient condition || |I-CB| ||_inf < 1 (Banach/Neumann). It is
% checked from the whole 3x3 interval; no sample may grant a PASS.
ok=false;box=NaN(3,2);eta=NaN;
midB=(B(:,:,1)+B(:,:,2))/2;
midr=(r(:,1)+r(:,2))/2;
if ~all(isfinite([midB(:);midr(:)])),return;end
% The midpoint belongs to the interval box. If it is singular, this
% input box CONTAINS a singular matrix: no full-box invertibility is
% possible. Avoid MATLAB inv/backslash on a singular matrix (and its
% warnings). A 3x3 cofactor preconditioner is purely algebraic and adds
% no empirical rcond tolerance or fabricated regularity assumption.
% C need not be an exact inverse: the verified contraction ||I-CB||<1
% is itself the real-arithmetic sufficient condition for regularity.
[C,midpoint_regular]=adjugate_preconditioner_3x3(midB);
if ~midpoint_regular,return;end
Elo=eye(3);Ehi=eye(3);
for i=1:3
    for j=1:3
        for k=1:3
            t=mul([C(i,k),C(i,k)],reshape(B(k,j,:),1,2));
            Elo(i,j)=Elo(i,j)-t(2);
            Ehi(i,j)=Ehi(i,j)-t(1);
        end
    end
end
absE=max(abs(Elo),abs(Ehi));
eta=max(sum(absE,2));
if ~(isfinite(eta)&&eta<1),return;end
u0=C*midr;
% residual r-B*u0, evaluated as an interval expression
res=r;
for i=1:3
    for j=1:3
        t=mul(reshape(B(i,j,:),1,2),[u0(j),u0(j)]);
        res(i,:)=sub(res(i,:),t);
    end
end
v=zeros(3,2);
for i=1:3
    for j=1:3
        t=mul([C(i,j),C(i,j)],res(j,:));
        v(i,:)=add(v(i,:),t);
    end
end
b=max(abs(v),[],2);
try
    radius=(eye(3)-absE)\b;
catch
    return
end
if any(~isfinite(radius))||any(radius<0),return;end
box=[u0-radius,u0+radius];
ok=all(isfinite(box(:)))&&all(box(:,1)<=box(:,2));
end

function [C,regular]=adjugate_preconditioner_3x3(M)
% Warning-free exact-formula 3x3 cofactor preconditioner, double arithmetic.
% No rcond-based threshold: a midpoint whose computed determinant is zero
% cannot furnish a verified inverse, so fail closed. All inputs still
% require the independent interval contraction test in the caller.
C=NaN(3);regular=false;
if ~isequal(size(M),[3 3]) || any(~isfinite(M(:))),return;end
K=zeros(3);
K(1,1)=M(2,2)*M(3,3)-M(2,3)*M(3,2);
K(1,2)=M(2,3)*M(3,1)-M(2,1)*M(3,3);
K(1,3)=M(2,1)*M(3,2)-M(2,2)*M(3,1);
K(2,1)=M(1,3)*M(3,2)-M(1,2)*M(3,3);
K(2,2)=M(1,1)*M(3,3)-M(1,3)*M(3,1);
K(2,3)=M(1,2)*M(3,1)-M(1,1)*M(3,2);
K(3,1)=M(1,2)*M(2,3)-M(1,3)*M(2,2);
K(3,2)=M(1,3)*M(2,1)-M(1,1)*M(2,3);
K(3,3)=M(1,1)*M(2,2)-M(1,2)*M(2,1);
d=sum(M(1,:).*K(1,:));
if ~isfinite(d)||d==0||any(~isfinite(K(:))),return;end
C=K.'/d;
regular=all(isfinite(C(:)));
end

function d=det3(B)
% Fully interval-expanded permutation formula for a 3x3 determinant.
% Positive even permutations and negative odd permutations.
plus=[1,2,3;2,3,1;3,1,2];
minus=[1,3,2;3,2,1;2,1,3];
d=[0,0];
for k=1:3
    d=add(d,term(B,plus(k,:)));
    d=sub(d,term(B,minus(k,:)));
end
end
function v=term(B,p)
v=[1,1];
for i=1:3
    v=mul(v,reshape(B(i,p(i),:),1,2));
end
end
function z=add(x,y)
z=[x(1)+y(1),x(2)+y(2)];
end
function z=sub(x,y)
z=[x(1)-y(2),x(2)-y(1)];
end
function z=mul(x,y)
p=[x(1)*y(1),x(1)*y(2),x(2)*y(1),x(2)*y(2)];
z=[min(p),max(p)];
end
