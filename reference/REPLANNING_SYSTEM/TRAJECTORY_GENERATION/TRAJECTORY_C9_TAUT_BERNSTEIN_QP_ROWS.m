function [A,b,S]=TRAJECTORY_C9_TAUT_BERNSTEIN_QP_ROWS(piece,authority,U)
%TRAJECTORY_C9_TAUT_BERNSTEIN_QP_ROWS Embed a whole-time taut CONE in C9 QP.
% This generates SEARCH CONSTRAINTS, NOT a replacement safety certificate.
% With w(t)=g-pL''(t), any fixed unit n satisfying
%      n'*w_i(new) >= minimum_tension_N/mL + acceleration_guard_mps2
% for all 22 Bernstein acceleration controls implies this SAME halfspace
% bound for every real t (Bernstein convex hull). It is affine in the 3x4
% free degree-23 controls. A candidate still REQUIRES the independent
% nonlinear whole-time geometry/tension/thrust/HLC certification.
%
% To avoid constraining a search to a needlessly narrow cone, emit rows
% ONLY when the CURRENT polynomial already satisfies the strict common
% cone. Therefore, normal n is induced by the candidate's actual w_i,
% never a prescribed up/side direction; this works under arbitrary SO(3).
% If current w_i fail this sufficient condition, return no constraints;
% this is NOT evidence of a collision or physical impossibility.
%
% QP variables X=DeltaF*U', x=X(:), DeltaF=reshape(x,3,4)/U'.
% Each row is A(:,j)'*x >= b(j) (same convention as original QP).
A=zeros(12,0);b=zeros(0,1);
S=struct('status','NOT_EVALUATED','rows',0, ...
    'current_common_cone_valid',false, ...
    'current_cone_lower_bound_mps2',NaN, ...
    'required_acceleration_lower_bound_mps2',NaN, ...
    'normal',[NaN;NaN;NaN], ...
    'safe_to_commit',false);
assert(isstruct(piece)&&isscalar(piece)&& ...
    all(isfield(piece,{'control_points_m','duration_s'}))&& ...
    isequal(size(piece.control_points_m),[3 24])&& ...
    all(isfinite(piece.control_points_m(:)))&& ...
    isfinite(piece.duration_s)&&piece.duration_s>0, ...
    'C9_TAUT_QP:Piece','Require real finite degree-23 control points.');
assert(isstruct(authority)&&isscalar(authority)&& ...
    all(isfield(authority,{'gravity_world_mps2','payload_mass_kg', ...
        'minimum_tension_N','acceleration_guard_mps2'}))&& ...
    isequal(size(authority.gravity_world_mps2),[3,1])&& ...
    all(isfinite(authority.gravity_world_mps2(:)))&& ...
    isfinite(authority.payload_mass_kg)&&authority.payload_mass_kg>0&& ...
    isfinite(authority.minimum_tension_N)&&authority.minimum_tension_N>=0&& ...
    isfinite(authority.acceleration_guard_mps2)&& ...
    authority.acceleration_guard_mps2>=0, ...
    'C9_TAUT_QP:Authority','Live force/tension bounds required.');
assert(isnumeric(U)&&isreal(U)&&isequal(size(U),[4,4])&& ...
    all(isfinite(U(:)))&&rcond(U)>0, ...
    'C9_TAUT_QP:Coordinates','Require nonsingular QP whitening.');
P=double(piece.control_points_m);
T=double(piece.duration_s);
k=23*22/T^2;
w=repmat(double(authority.gravity_world_mps2),1,22)-k*diff(P,2,2);
c=mean(w,2);
normc=norm(c);
if ~(isfinite(normc)&&normc>0)
    S.status='SKIP_NO_GEOMETRY_DERIVED_COMMON_NORMAL';return
end
n=c/normc;
threshold=double(authority.minimum_tension_N)/ ...
    double(authority.payload_mass_kg)+ ...
    double(authority.acceleration_guard_mps2);
S.required_acceleration_lower_bound_mps2=threshold;
S.current_cone_lower_bound_mps2=min(n.'*w);
S.normal=n;
if ~(all(isfinite(w(:)))&&S.current_cone_lower_bound_mps2>threshold)
    % The common-halfspace test is sufficient, not necessary. In
    % particular, we MUST NOT rule out physically safe twisted paths.
    S.status='SKIP_CURRENT_COMMON_CONE_NOT_PROVEN';return
end
S.current_common_cone_valid=true;
% Exactly SIX of the 22 second-derivative controls depend on the middle
% four free degree-23 points. The other controls are fixed at endpoints;
% their strict inequalities are already checked above.
A=zeros(12,6);b=zeros(6,1);out=0;
for j=1:22
    coeff=zeros(1,4);
    for f=1:4
        pindex=10+f; % free control points are P(:,11:14)
        coeff(f)=double(pindex==j)-2*double(pindex==j+1)+ ...
            double(pindex==j+2);
    end
    if ~any(coeff),continue;end
    % w_i(new)=w_i(old)-k*sum_f coeff(f)*DeltaF(:,f).
    % n'*w_i(new)>=threshold <=>
    %   <-k*n*coeff,DeltaF> >= threshold-n'*w_i(old).
    grad=-k*(n*coeff);
    out=out+1;
    A(:,out)=reshape(grad/U,12,1);
    b(out)=threshold-n.'*w(:,j);
end
A=A(:,1:out);b=b(1:out);
S.rows=out;
S.status='EMBEDDED_STRICT_CURRENT_TAUT_CONE_AS_AFFINE_QP_ROWS';
% Merely adding these rows does not prove that the QP will obey them,
% nor that the actual nonlinear plant and HLC will follow the piece.
end
