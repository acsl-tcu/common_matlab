function qp = cbf_hocbf_scene(mode,shape,x,settings,obstacles)
%CBF_HOCBF_SCENE  複数障害物の高次QP行を結合する。
% obstacles = {struct('type','sphere',...),struct('type','ellipsoid',...),...};
% 地面と実入力上下限は重複させず1度だけ追加する。
% 戻り値: qp.A * nu >= qp.b。スラックは導入しない。
if ~iscell(obstacles) || isempty(obstacles)
    error('CBF:SceneInput','障害物は非空のセル配列で渡してください。');
end
qp=[];
% 力学のTaylor係数は障害物の個数・回転姿勢に依存しない。
% 1周期につき5本だけ計算し、全障害物に使い回す。
jetCache=cbf_precompute_jets(mode,x,settings);
for i=1:numel(obstacles)
    obs=obstacles{i};
    if ~isfield(obs,'type')
        error('CBF:SceneType','障害物にtypeが必要です。');
    end
    [h,info]=cbf_evaluate_case(mode,shape,obs.type,x,settings,obs);
    if i==1
        one=cbf_hocbf_evaluate(mode,shape,obs.type,x,settings,obs,h,info,'all',jetCache);
        qp=one;
        qp.labels=cellfun(@(name)sprintf('obs%d_%s',i,name), ...
            qp.labels,'UniformOutput',false);
    else
        % 2個目以降は幾何障害物のTaylorだけを生成。
        % 地面・入力上下限の高階係数は再計算しない（制約自体は保持）。
        only=cbf_hocbf_evaluate(mode,shape,obs.type,x,settings,obs,h,info,'obstacles',jetCache);
        qp.A=[qp.A;only.A];
        qp.b=[qp.b;only.b];
        qp.labels=[qp.labels;cellfun(@(name)sprintf('obs%d_%s',i,name), ...
            only.labels,'UniformOutput',false)];
        qp.order=[qp.order;only.order];
        qp.psi=[qp.psi;only.psi];
        qp.psiInitiallyNonnegative=[qp.psiInitiallyNonnegative;only.psiInitiallyNonnegative];
        qp.effectNorm=[qp.effectNorm;only.effectNorm];
        qp.degenerate=[qp.degenerate;only.degenerate];
        qp.uncontrollableViolation=[qp.uncontrollableViolation;only.uncontrollableViolation];
        qp.relativeDegreeAtState=[qp.relativeDegreeAtState;only.relativeDegreeAtState];
        qp.earlyInputResidual=[qp.earlyInputResidual;only.earlyInputResidual];
    end
end
% 既知の数学的包含関係に基づく冗長な地面CBFの省略。
qp=cbf_prune_proven_ground(qp,mode,shape,settings);
end
