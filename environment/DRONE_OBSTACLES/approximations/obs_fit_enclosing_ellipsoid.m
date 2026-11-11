function [center,Rlocal,axes,info]=obs_fit_enclosing_ellipsoid(vertices,options)
% Approximate minimum-volume ENclosing ellipsoid for convex hull of points.
%
% Khachiyan / Frank-Wolfe dual optimization; NO Optimization Toolbox.
% IMPORTANT: A posteriori inflation is computed from EVERY vertex of the
% OUTER-COVER polytope. Since an ellipsoid is convex and contains these
% vertices, it contains their entire convex hull, and thus the source.
%
% Coordinate frame: vertices and returned center/Rlocal are obstacle-local.
% DO NOT use a random surface point cloud in lieu of outer-cover vertices.

if nargin<2,options=struct();end
if isfield(options,'mveeTolerance'),tol=options.mveeTolerance;else,tol=2e-3;end
if isfield(options,'mveeMaxIterations'),maxIter=options.mveeMaxIterations;else,maxIter=2500;end
if isfield(options,'mveeSafetyFactor'),guard=options.mveeSafetyFactor;else,guard=1+1e-8;end
assert(isfinite(tol)&&tol>0&&tol<=0.1, 'mveeTolerance must be in (0,0.1]');
assert(isfinite(maxIter)&&maxIter>=1&&mod(maxIter,1)==0, ...
    'mveeMaxIterations must be positive integer');
assert(isfinite(guard)&&guard>=1, 'mveeSafetyFactor must be >= 1');
V=unique(double(vertices),'rows');
assert(size(V,2)==3 && size(V,1)>=4 && all(isfinite(V(:))), ...
    'Invalid points supplied for MVEE');
assert(rank(V-V(1,:))==3,'MVEE requires nondegenerate 3D points');

% Axis-by-axis normalization is conditioning only, not a geometric cover.
lo=min(V,[],1);hi=max(V,[],1);shift=(lo+hi)/2;
scale=max((hi-lo)/2,1e-10);
X=(V-shift)./scale; N=size(X,1); d=3;
P=[X';ones(1,N)];
w=ones(N,1)/N;
converged=false;
for iter=1:maxIter
    Q=(P.*w')*P';
    Q=(Q+Q')/2;
    if rcond(Q)<1e-13
        error('obs_fit_enclosing_ellipsoid:Singular', ...
            'Numerically degenerate MVEE matrix; inspect obstacle geometry');
    end
    B=Q\P;
    M=sum(P.*B,1);
    [maximum,idx]=max(M);
    if maximum <= (d+1)*(1+tol)
        converged=true;break;
    end
    alpha=max(0,min(1,(maximum-(d+1))/((d+1)*(maximum-1))));
    w=(1-alpha)*w;
    w(idx)=w(idx)+alpha;
end

% Standard MVEE primal reconstruction: A=inv(Covariance)/d.
cN=(w'*X);D=X-cN;
covN=(D.*w)'*D;
covN=(covN+covN')/2;
[U,L]=eig(covN);
e=real(diag(L));
assert(all(isfinite(e)) && all(e>1e-14), ...
    'MVEE covariance is degenerate');
axesN=sqrt(d*e(:)');

% Exact finite vertex certificate for the OUTER cover. Even if dual
% iteration stopped early, inflation contains every cover vertex.
localD=D*U;
q=sum((localD./axesN).^2,2);
maxRadius=sqrt(max(q));
fac=max(1,maxRadius)*guard;
axesN=axesN*fac;

% Undo normalization, then diagonalize the physical ellipsoid matrix.
B=diag(scale)*U*diag(axesN);
G=B*B';G=(G+G')/2;
[Rp,E]=eig(G);
a=sqrt(real(diag(E)))';
assert(all(isfinite(a)) && all(a>0), 'Invalid fitted ellipsoid axes');
center=shift+cN.*scale;
if det(Rp)<0,Rp(:,3)=-Rp(:,3);end
Rlocal=Rp; axes=a;

% A second certificate after eigendecomposition and floating-point rounding.
D2=(V-center)*Rlocal;
q2=sum((D2./axes).^2,2);
fac2=max(1,sqrt(max(q2)))*(1+1e-10);
axes=axes*fac2;
q3=sum((D2./axes).^2,2);
assert(all(q3<=1+1e-9), ...
    'MVEE output failed to enclose certified source cover vertices');
info=struct('numVertices',N,'iterations',iter,'converged',converged, ...
    'maxNormBeforeInflation',maxRadius, ...
    'inflationFactor',fac*fac2,'volume',4*pi/3*prod(axes));
end
