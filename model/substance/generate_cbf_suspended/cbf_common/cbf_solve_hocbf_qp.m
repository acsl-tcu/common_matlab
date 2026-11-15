function [out, timing]=cbf_solve_hocbf_qp(qp,nuNom,W,decisionBounds)
%CBF_SOLVE_HOCBF_QP  ハードCBFを緩めない最小修正QP。
% min (nu-nuNom)' W (nu-nuNom) s.t. qp.A*nu>=qp.b。
% 実行不可の場合、勝手なスラック/入力クリップは行わない。
% 仮想入力の場合は実入力制約がまだ含まれないことに注意。
% 第4引数 decisionBounds.lower / .upper (各4x1) を指定すると
% 決定入力への明示的なハード上下限をquadprogに渡す。
% 物理的な総推力・トルクの制限ではないため実HLCの保証と混同しない。
% 境界を与えない場合は従来の無制限QPと完全互換。
profileOn=(nargout>=2);
if profileOn, tAll=tic; timing=struct(); else, timing=[]; end
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
% 有限な決定入力集合の診断用（値は外部からのみ受け取る）。
if nargin<4 || isempty(decisionBounds)
    lb=[];ub=[];
else
    if ~isstruct(decisionBounds) || ~isfield(decisionBounds,'lower') || ...
       ~isfield(decisionBounds,'upper')
        error('CBF:DecisionLimits','lower / upper のある構造体を指定してください。');
    end
    lb=decisionBounds.lower(:);ub=decisionBounds.upper(:);
    if numel(lb)~=4 || numel(ub)~=4 || ...
       any(~isfinite(lb)) || any(~isfinite(ub)) || any(lb>=ub)
        error('CBF:DecisionLimits','決定入力上下限は各4次元の有限値、lower<upperが必要です。');
    end
end
if profileOn, timing.validationMs=1000*toc(tAll); tStep=tic; end
options=optimoptions('quadprog','Display','off');
if profileOn, timing.optionsMs=1000*toc(tStep); tStep=tic; end
[nu,~,exitflag,output]=quadprog(2*W,-2*W*nuNom,-qp.A,-qp.b,[],[],lb,ub,[],options);
if profileOn, timing.quadprogMs=1000*toc(tStep); tStep=tic; end
out=struct('nu',nu,'exitflag',exitflag,'output',output, ...
           'initialHOCBFSetSatisfied',all(qp.psiInitiallyNonnegative), ...
           'inputBoundsIncluded',qp.inputLimitIncluded);
if exitflag>0
    out.minResidual=min(qp.A*nu-qp.b);
    if ~isempty(lb)
        out.minResidual=min([out.minResidual;nu-lb;ub-nu]);
    end
else
    out.minResidual=NaN;
end
out.decisionBoundsUsed=~isempty(lb);
if profileOn
    timing.postprocessMs=1000*toc(tStep);
    timing.totalMs=1000*toc(tAll);
end
end
