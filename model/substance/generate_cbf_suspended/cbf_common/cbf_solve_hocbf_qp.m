function out=cbf_solve_hocbf_qp(qp,nuNom,W)
%CBF_SOLVE_HOCBF_QP  ハードCBFを緩めない最小修正QP。
% min (nu-nuNom)' W (nu-nuNom) s.t. qp.A*nu>=qp.b。
% 実行不可の場合、勝手なスラック/入力クリップは行わない。
% 仮想入力の場合は実入力制約がまだ含まれないことに注意。
if exist('quadprog','file')~=2
    error('CBF:QuadprogRequired','Optimization Toolboxのquadprogが必要です。');
end
nuNom=nuNom(:);
if numel(nuNom)~=4,error('CBF:NominalInputSize','基準入力は4要素です。');end
if nargin<3 || isempty(W)
    error('CBF:MissingWeight','QP重み行列Wは呼び出し元から指定してください。');
end
if ~isequal(size(W),[4 4]) || any(eig((W+W.')/2)<=0)
    error('CBF:Weight','Wは正定値の4x4行列にしてください。');
end
W=(W+W.')/2;
% optimoptionsの生成を毎周期繰り返さない（最適化問題は変更しない）。
persistent options;
if isempty(options)
    options=optimoptions('quadprog','Display','off');
end
[nu,~,exitflag,output]=quadprog(2*W,-2*W*nuNom,-qp.A,-qp.b,[],[],[],[],[],options);
out=struct('nu',nu,'exitflag',exitflag,'output',output, ...
           'initialHOCBFSetSatisfied',all(qp.psiInitiallyNonnegative), ...
           'inputBoundsIncluded',qp.inputLimitIncluded);
if exitflag>0
    out.minResidual=min(qp.A*nu-qp.b);
else
    out.minResidual=NaN;
end
end
