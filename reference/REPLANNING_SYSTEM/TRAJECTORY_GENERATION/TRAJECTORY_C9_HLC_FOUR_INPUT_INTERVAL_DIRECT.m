function R=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT(live_lower,live_upper, ...
    beta2_interval_3x3x2,rhs_interval_3x2,thrust_interval_1x2)
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
    'method','CRAMER_3X3_INTERVAL_REAL_ARITHMETIC', ...
    'scope','CONDITIONAL_ALL_TIME_HLC_BETA2_RHS_UF_INTERVALS_ONLY');
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
d=det3(B);
R.determinant_interval=d;
if d(1)<=0&&d(2)>=0
    R.status='UNKNOWN_HLC_BETA2_INTERVAL_MAY_BE_SINGULAR';return
end
R.interval_invertibility_certified=true;
raw=zeros(4,2);
raw(1,:)=f;
for i=1:3
    Bi=B;
    Bi(:,i,1)=r(:,1);
    Bi(:,i,2)=r(:,2);
    nu=det3(Bi);
    R.numerator_intervals(i,:)=nu;
    % Division is sound only because det(B) does not cross zero.
    invdet=[1/d(2),1/d(1)];
    raw(i+1,:)=mul(nu,invdet);
end
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
