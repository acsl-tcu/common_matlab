function [n,META] = af7r36_canonical_separator_normal(R,a,progress)
% Deterministic canonical maximizer of the lateral ellipsoid metric.
%
% Nonzero progress:
%   max n'Qn  s.t. ||n||=1, n'v=0
% is reduced to a deterministic 2x2 symmetric problem in a canonical basis
% of v^\perp.  A numerically degenerate top eigenspace uses b1 as a
% deterministic tie-break.  The ambiguity guard is binary64 implementation
% bookkeeping only; it is NOT a physical safety margin.
%
% Zero progress:
%   Q's eigenspaces are known from R and a.  The top eigenspace projector is
% built directly and a deterministic world-axis projection is selected.

R=double(R); a=double(a(:)); progress=double(progress(:));
assert(isequal(size(R),[3 3])&&numel(a)==3&&all(a>0)&&all(isfinite([R(:);a;progress])));

Q=R*diag(1./(a.^2))*R.';
Q=0.5*(Q+Q.');

np=norm(progress);
META=struct('schema','AF7R36_CANONICAL_SEPARATOR_META_V1', ...
    'mode','','degenerate',false,'guard',NaN,'radial_gap',NaN);

if np>128*eps(max(1,np))
    v=progress/np;
    av=abs(v);
    [~,k]=min(av); % first minimum = deterministic tie break
    e=zeros(3,1); e(k)=1;
    b1=e-v*(v.'*e);
    b1=b1/norm(b1);
    b2=cross(v,b1);
    b2=b2/norm(b2);

    a11=b1.'*Q*b1;
    a12=b1.'*Q*b2;
    a22=b2.'*Q*b2;
    delta=0.5*(a11-a22);
    rad=hypot(delta,a12);
    scale=max([1,abs(a11),abs(a12),abs(a22),norm(Q,'fro')]);
    guard=4096*eps(scale);

    if rad<=guard
        n=b1;
        deg=true;
    else
        theta=0.5*atan2(a12,delta);
        n=cos(theta)*b1+sin(theta)*b2;
        n=n/norm(n);
        deg=false;
    end
    META.mode='NONZERO_PROGRESS_ANALYTIC_2D';
    META.degenerate=deg;
    META.guard=guard;
    META.radial_gap=rad;
else
    lam=1./(a.^2);
    lmax=max(lam);
    scale=max([1;abs(lam)]);
    guard=4096*eps(scale);
    E=zeros(3);
    for j=1:3
        if abs(lam(j)-lmax)<=guard
            r=R(:,j);
            E=E+r*r.';
        end
    end
    n=[];
    for k=1:3
        e=zeros(3,1); e(k)=1;
        q=E*e;
        if norm(q)>128*eps
            n=q/norm(q);
            break;
        end
    end
    assert(~isempty(n),'[AF7R36] zero-progress top eigenspace selection failed.');
    META.mode='ZERO_PROGRESS_TOP_EIGENSPACE_PROJECTOR';
    META.degenerate=nnz(abs(lam-lmax)<=guard)>1;
    META.guard=guard;
    META.radial_gap=NaN;
end

% Canonical sign. +/- is still enumerated, so this affects only labels and
% cross-platform reproducibility, not the disjunctive feasible union.
g=128*eps(max(1,max(abs(n))));
for k=1:3
    if abs(n(k))>g
        if n(k)<0, n=-n; end
        break;
    end
end
n=n/norm(n);
end
