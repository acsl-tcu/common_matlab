function S=TRAJECTORY_C9_DURATION_SENSITIVITY(t0_s,T_s,left_jets,right_jets,free,phase_samples)
% TRAJECTORY_C9_DURATION_SENSITIVITY
% Analytic derivative with respect to *physical duration* of the degree-23
% Bernstein payload polynomial, keeping its prescribed physical 0..9
% endpoint jets and its 12 free coordinates FIXED.  This is the correct
% differential primitive for adding duration to a SINGLE C9 search; naive
% rescaling of the time coordinate DOES NOT preserve C7/C9 joins.
%
% For k=0..9, P_k(T)=sum_{r=0}^k nchoosek(k,r)*T^r/(23)_r * left(:,r+1).
% For k=0..9, P_(23-k)(T)=sum_{r=0}^k (-1)^r*nchoosek(k,r)
%                         *T^r/(23)_r * right(:,r+1).
% The four interior Bernstein controls remain fixed here; the full
% nonlinear optimization must eventually vary them AND duration together.
%
% rth PHYSICAL derivative polygon: D_r=(23)_r/T^r * diff^r(P).
% Its duration derivative satisfies:
%   dD_r/dT = (24-r)/T*diff(dD_(r-1)/dT) - D_r/T.
% These are exact algebraic derivatives, not finite differences.
% Caller still needs obstacle, tautness, input, execution and ALL-TIME
% certificates before accepting any candidate. No heuristic time cap,
% minimum speed threshold or arbitrary objective weight is introduced.
if nargin<6,phase_samples=zeros(1,0);end
assert(isnumeric(phase_samples)&&isreal(phase_samples)&& ...
       (isempty(phase_samples)||isvector(phase_samples))&&all(isfinite(phase_samples(:)))&& ...
       all(phase_samples(:)>=0)&&all(phase_samples(:)<=1), ...
       'C9_DURATION_SENSITIVITY:Phase','Phases must be finite in [0,1].');
assert(isnumeric(t0_s)&&isreal(t0_s)&&isscalar(t0_s)&&isfinite(t0_s)&& ...
       isnumeric(T_s)&&isreal(T_s)&&isscalar(T_s)&&isfinite(T_s)&&T_s>0 && ...
       isfinite(t0_s+T_s)&& ...
       isnumeric(left_jets)&&isreal(left_jets)&&isequal(size(left_jets),[3 10])&& ...
       all(isfinite(left_jets(:)))&& ...
       isnumeric(right_jets)&&isreal(right_jets)&&isequal(size(right_jets),[3 10])&& ...
       all(isfinite(right_jets(:)))&& ...
       isnumeric(free)&&isreal(free)&&isequal(size(free),[3 4])&& ...
       all(isfinite(free(:))), ...
       'C9_DURATION_SENSITIVITY:Input', 'Finite C9 jets, controls and positive T required.');
P=TRAJECTORY_C9_FLAT_BERNSTEIN.build(t0_s,T_s,left_jets,right_jets,free);
dP=zeros(3,24);
fall=1;
for r=1:9
    fall=fall*(24-r);
    % Coefficient of the rth jet in d/dT of the endpoint control polygon.
    scale=(r*T_s^(r-1))/fall;
    for k=r:9
        c=nchoosek(k,r)*scale;
        dP(:,k+1)=dP(:,k+1)+c*left_jets(:,r+1);
        dP(:,24-k)=dP(:,24-k)+(-1)^r*c*right_jets(:,r+1);
    end
end
D=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(P);
dD=cell(1,10);dD{1}=dP;
for r=1:9
    dD{r+1}=(24-r)/T_s*diff(dD{r},1,2)-D{r+1}/T_s;
end
assert(all(isfinite(dP(:)))&&all(cellfun(@(x)all(isfinite(x(:))),dD)), ...
       'C9_DURATION_SENSITIVITY:Nonfinite', ...
       'Duration derivative produced nonfinite coefficients; UNKNOWN.');
S=struct('status','ANALYTIC_DURATION_DERIVATIVE__NOT_A_FEASIBILITY_PROOF', ...
         'duration_s',T_s, ...
         'piece',P, ...
         'dcontrol_points_dT_mps',dP, ...
         'physical_derivative_controls',{D}, ...
         'dphysical_derivative_controls_dT',{dD}, ...
         'safe_to_commit',false, ...
         'scope','C9_FIXED_PHYSICAL_ENDPOINT_JETS__ONE_POLYNOMIAL__NO_TIME_OPTIMUM_OR_HLC_PROOF');
% Optional time-derivative jets needed for a SINGLE joint (free,T) solve.
% At a fixed normalized phase u: d/dT(p^(r)(t0+uT)) is dD_r/dT evaluated
% at u.  At a fixed PHYSICAL t, du/dT=-u/T, hence
%   (d/dT)_t p^(r)(t) = (d/dT)_u p^(r)(t0+uT) - u*p^(r+1)(t).
% Returned fixed-time derivatives are r=0..8 (order 9 is available only
% if one also evaluates the physical 10th derivative).
u=double(phase_samples(:).');N=numel(u);
S.phase_samples=u;
S.physical_jets=zeros(3,10,N);
S.dphysical_jets_dT_fixed_phase=zeros(3,10,N);
S.dphysical_jets_dT_fixed_time=zeros(3,9,N);
for r=0:9
    deg=23-r; j=0:deg;
    if N==0,continue;end
    weights=(u(:).^j).*((1-u(:)).^(deg-j));
    c=ones(1,deg+1);
    for jj=1:deg
        c(jj+1)=c(jj)*(deg-jj+1)/jj;
    end
    weights=weights.*c;
    S.physical_jets(:,r+1,:)=reshape(D{r+1}*weights.',3,1,N);
    S.dphysical_jets_dT_fixed_phase(:,r+1,:)= ...
        reshape(dD{r+1}*weights.',3,1,N);
end
for r=0:8
    S.dphysical_jets_dT_fixed_time(:,r+1,:)= ...
        S.dphysical_jets_dT_fixed_phase(:,r+1,:)- ...
        reshape(u,1,1,N).*S.physical_jets(:,r+2,:);
end
end
