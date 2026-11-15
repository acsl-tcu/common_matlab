function result=CBF_Audit_LP(qp,subset)
% CBF_AUDIT_LP 既存の A*nu>=b の可行性だけを別のLPで確認する。
% QP本体・制約・ゲインを変更しない。診断用のスラックは制御に使用しない。
if nargin<2 || isempty(subset),subset=true(size(qp.b));end
A=qp.A(subset,:);b=qp.b(subset);
result=struct('feasible',true,'exitflag',1,'minResidual',NaN,'nu',zeros(4,1));
if isempty(b),return;end
if any(~isfinite(A(:))) || any(~isfinite(b))
    result.feasible=false;result.exitflag=-99;return;
end
opts=optimoptions('linprog','Display','off');
try
    [u,~,flag]=linprog(zeros(4,1),-A,-b,[],[],[],[],opts);
    result.exitflag=flag;
    if isempty(u)
        result.feasible=false;result.minResidual=NaN;result.nu=nan(4,1);
    else
        result.nu=u;result.minResidual=min(A*u-b);
        tol=1e-7*(1+norm(b,Inf));
        result.feasible=flag>0 && result.minResidual>=-tol;
    end
catch
    result.feasible=false;result.exitflag=-98;
end
end
