function R=AF7R58R92_SUPPORT_SEED_ROUTES(pStart,pEnd,knownObs,inflation,maxAssignments)
% Geometry-only homotopy seeds for arbitrary oriented static ellipsoids.
% These are WAYPOINT INITIAL GUESSES, never safe paths or certificates.
% Different left/right/up/down support choices are enumerated; number of
% attempted assignments is an EXTERNAL CPU-work budget, not a trajectory or
% maneuver-duration limit. Every eventual candidate must pass R80/R91/R90.
assert(isnumeric(pStart)&&isequal(size(pStart),[3 1])&& ...
       all(isfinite(pStart))&&isnumeric(pEnd)&& ...
       isequal(size(pEnd),[3 1])&&all(isfinite(pEnd)), ...
       'R92:Finite3DStartGoalRequired');
assert(isstruct(knownObs),'R92:SensorKnownEllipsoidsRequired');
assert(isnumeric(inflation)&&isscalar(inflation)&&isfinite(inflation)&& ...
    inflation>=0,'R92:ExternalSeedRadiusRequired');
assert(isnumeric(maxAssignments)&&isscalar(maxAssignments)&& ...
    isfinite(maxAssignments)&&maxAssignments>=1&& ...
    fix(maxAssignments)==maxAssignments,'R92:FiniteCPUWorkBudgetRequired');
v=pEnd-pStart;d=norm(v);
assert(isfinite(d)&&d>0,'R92:ZeroChordRequiresDifferentLocalChart');
u=v/d;I=eye(3);
[~,j]=min(abs(u));n1=cross(u,I(:,j));n1=n1/norm(n1);
n2=cross(u,n1);n2=n2/norm(n2);
normals=[n1,-n1,n2,-n2];
near=false(1,numel(knownObs));prog=zeros(1,numel(knownObs));
for k=1:numel(knownObs)
 O=knownObs(k);
 assert(isfield(O,'center')&&isfield(O,'R')&&isfield(O,'axes') && ...
   isequal(size(O.center),[3 1])&&isequal(size(O.axes),[3 1])&& ...
   isequal(size(O.R),[3 3])&&all(isfinite(O.center))&& ...
   all(isfinite(O.axes))&&all(O.axes>0)&& ...
   all(isfinite(O.R(:)))&& ...
   norm(O.R'*O.R-eye(3),'fro')<1e-9&& ...
   abs(det(O.R)-1)<1e-9,'R92:InvalidKnownEllipsoid');
 c=double(O.center);tau=max(0,min(1,dot(c-pStart,u)/d));
 foot=pStart+tau*v;
 % Bounding sphere only filters SEED-RELEVANT ellipsoids. Final R80
 % always certifies ALL sensor-known obstacles, including filtered ones.
 near(k)=(norm(c-foot)<=max(O.axes)+inflation && tau>0 && tau<1);
 prog(k)=tau;
end
idx=find(near);[~,order]=sort(prog(idx));idx=idx(order);
M=numel(idx);
if M==0
    R=struct('status','DIRECT_SEED__CERTIFICATE_STILL_REQUIRED', ...
      'routes',{{zeros(3,0)}},'route_length_m',d, ...
      'seed_relevant_known_indices',idx, ...
      'known_obstacle_count',numel(knownObs), ...
      'assignments_considered',1,'all_assignments_exhausted',true, ...
      'reference_certified',false);return;
end
routes=cell(1,0);lengths=[];
digits=zeros(1,M);exhausted=false;
for itr=1:maxAssignments
    P=zeros(3,M);
    for k=1:M
        o=knownObs(idx(k));n=normals(:,digits(k)+1);
        % Analytic SO(3)-equivariant support point, exact in real arithmetic.
        S=o.R*diag(double(o.axes).^2)*o.R';
        support=S*n/sqrt(n'*S*n);
        P(:,k)=double(o.center)+support+inflation*n;
    end
    routes{end+1}=P; %#ok<AGROW>
    path=[pStart,P,pEnd];
    lengths(end+1)=sum(vecnorm(diff(path,1,2),2,1)); %#ok<AGROW>
    carry=true;
    for k=M:-1:1
        if carry
            digits(k)=digits(k)+1;
            if digits(k)>=4,digits(k)=0;
            else,carry=false;end
        end
    end
    if carry,exhausted=true;break;end
end
[lengths,ord]=sort(lengths);
routes=routes(ord);
R=struct('status','SUPPORT_SEEDS_ONLY__NOT_CERTIFIED', ...
    'routes',{routes},'route_length_m',lengths, ...
    'seed_relevant_known_indices',idx, ...
    'known_obstacle_count',numel(knownObs), ...
    'assignments_considered',numel(routes), ...
    'all_assignments_exhausted',exhausted, ...
    'reference_certified',false);
end
