function R=TRAJECTORY_KINEMATIC_CERTIFICATE(pieces,limit,options)
%TRAJECTORY_KINEMATIC_CERTIFICATE  Continuous-time upper derivative norms.
% Bound every t in each Bernstein piece using derivative control hulls:
% ||p^(k)(t)|| <= max_i ||derivative_controls{k+1}(:,i)||.
% Optional midpoint de Casteljau subdivision of the ORIGINAL controls
% tightens hull bounds without time sampling. All speed/accel/jerk limits
% come from the caller, NOT from this module or arbitrary defaults.
% Required limits for each order k=1..6: limit.derivative_norm_max(k).
% Output means only nominal reference kinematics (NOT dynamic feasibility).
assert(isstruct(pieces)&&~isempty(pieces)&&isstruct(limit)&& ...
    isscalar(limit)&&isfield(limit,'derivative_norm_max')&& ...
    isnumeric(limit.derivative_norm_max)&&numel(limit.derivative_norm_max)==6 && ...
    isreal(limit.derivative_norm_max)&&all(isfinite(limit.derivative_norm_max))&& ...
    all(limit.derivative_norm_max>=0)&&isstruct(options)&& ...
    isfield(options,'max_subdivision_depth')&& ...
    isnumeric(options.max_subdivision_depth)&&isscalar(options.max_subdivision_depth)&& ...
    isfinite(options.max_subdivision_depth)&& ...
    options.max_subdivision_depth>=0&& ...
    fix(options.max_subdivision_depth)==options.max_subdivision_depth, ...
    'TRAJECTORY_KINEMATIC:Authority','External derivative bounds and proof budget required.');
accepted=true;bad=zeros(1,0);lower=zeros(1,6);upper=zeros(1,6);
for j=1:numel(pieces)
    p=pieces(j);
    assert(all(isfield(p,{'control_points_m','duration_s'})), ...
        'TRAJECTORY_KINEMATIC:Piece','Incomplete Bernstein piece.');
    [ok,hi]=check(p.control_points_m,p.duration_s, ...
        double(limit.derivative_norm_max(:).'),options.max_subdivision_depth);
    upper=max(upper,hi);
    if ~ok,accepted=false;bad(end+1)=j;end %#ok<AGROW>
end
R=struct('status','UNKNOWN_BOUND_NOT_CERTIFIED','certified',accepted, ...
    'safe_to_commit',false,'upper_bounds',upper, ...
    'violating_or_unresolved_segments',bad, ...
    'meaning','Polynomial derivative-norm bounds 1..6 only; not force/torque/tension');
if accepted,R.status='CERTIFIED_CONTINUOUS_NOMINAL_KINEMATICS';end
end
function [ok,bnd]=check(P,T,limits,depth)
n=size(P,2)-1;
assert(size(P,1)==3&&n>=6&&all(isfinite(P(:)))&&isscalar(T)&&T>0&&isfinite(T), ...
    'TRAJECTORY_KINEMATIC:Piece','Invalid polynomial or duration.');
D=P;bnd=zeros(1,6);ok=true;
for k=1:6
    D=(n-k+1)/T*diff(D,1,2);
    bnd(k)=max(sqrt(sum(D.^2,1)));
    if bnd(k)>limits(k),ok=false;end
end
if ok||depth==0,return;end
% A subdivision can tighten the convex hull without changing the polynomial.
[A,B]=split_mid(P);
[ok0,h0]=check(A,T/2,limits,depth-1);
[ok1,h1]=check(B,T/2,limits,depth-1);
ok=ok0&&ok1;bnd=max(h0,h1);
end
function [L,R]=split_mid(P)
n=size(P,2);A=P;L=zeros(3,n);R=zeros(3,n);
L(:,1)=A(:,1);R(:,end)=A(:,end);
for k=2:n
    A=0.5*(A(:,1:end-1)+A(:,2:end));
    L(:,k)=A(:,1);R(:,n-k+1)=A(:,end);
end
end
