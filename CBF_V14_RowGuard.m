function G=CBF_V14_RowGuard(A,b,opts)
%CBF_V14_ROWGUARD Test-only diagnostic of nearly inactive but violated rows.
% A*nu>=b, with A,b from the original hard constraints.  This does not
% delete, soften, normalize or replace ANY constraint. An unresolved
% positive-RHS row must NOT be declared feasible from a gigantic QP result.
% Thresholds are numerical DIAGNOSTICS, NOT mathematical proofs of zero.
if nargin<3,opts=struct();end
if ~isfield(opts,'zeroActionAbsTol'),opts.zeroActionAbsTol=1e-10;end
if ~isfield(opts,'positiveRhsTol'),opts.positiveRhsTol=1e-7;end
if ~isfield(opts,'extremeDerivativeNorm'),opts.extremeDerivativeNorm=1e8;end
if ~isnumeric(A)||~ismatrix(A)||~isnumeric(b)||numel(b)~=size(A,1)
    error('CBF:V14GuardSize','A must be numeric n-by-m; b must be n-by-1.');
end
b=b(:);
if any(~isfinite([A(:);b])) || opts.zeroActionAbsTol<0 || ...
        opts.positiveRhsTol<0 || opts.extremeDerivativeNorm<=0
    error('CBF:V14GuardInvalid','Nonfinite coefficients or invalid thresholds.');
end
rowNorm2=sqrt(sum(A.^2,2));
rowNormInf=max(abs(A),[],2);
nearZero=rowNormInf<=opts.zeroActionAbsTol;
positive=b>opts.positiveRhsTol;
unresolved=nearZero&positive;
% Lower bound by Cauchy-Schwarz: if A*nu>=b>0 then ||nu||_2>=b/||A||_2.
minRequiredNorm2=zeros(size(b));
sel=b>0 & rowNorm2>0;
minRequiredNorm2(sel)=b(sel)./rowNorm2(sel);
minRequiredNorm2(b>0 & rowNorm2==0)=Inf;
extreme=b>opts.positiveRhsTol & ...
    minRequiredNorm2>=opts.extremeDerivativeNorm;
G=struct('rowNorm2',rowNorm2,'rowNormInf',rowNormInf,...
 'nearZeroAction',nearZero,'positiveRhs',positive,...
 'unresolvedPositiveRhs',unresolved,'minRequiredDerivativeNorm2',minRequiredNorm2,...
 'extremeDerivativeRequirement',extreme,...
 'hasUnresolved',any(unresolved),'hasExtreme',any(extreme),...
 'zeroActionAbsTol',opts.zeroActionAbsTol,...
 'positiveRhsTol',opts.positiveRhsTol,...
 'extremeDerivativeNorm',opts.extremeDerivativeNorm);
end
