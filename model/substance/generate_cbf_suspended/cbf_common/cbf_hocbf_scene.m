function [qp, timing] = cbf_hocbf_scene(mode,shape,x,settings,obstacles)
%CBF_HOCBF_SCENE  複数障害物の高次QP行を結合する。
% obstacles = {struct('type','sphere',...),struct('type','ellipsoid',...),...};
% 地面と実入力上下限は重複させず1度だけ追加する。
% 戻り値: qp.A * nu >= qp.b。スラックは導入しない。
profileOn=(nargout>=2);
if profileOn
    tAll=tic;
    timing=struct('jetMs',0,'geometryMs',0,'highOrderMs',0, ...
        'mergeMs',0,'pruneMs',0,'totalMs',0, ...
        'perObstacle',[]);
    perObstacle=repmat(struct('geometryMs',0,'highOrderMs',0, ...
        'parameterMs',0,'zeroTaylorMs',0,'fourBasisTaylorMs',0, ...
        'hocbfAssemblyMs',0,'realInputBoundsMs',0, ...
        'diagnosticsMs',0,'totalMs',0),numel(obstacles),1);
else
    timing=[];
end
if ~iscell(obstacles) || isempty(obstacles)
    error('CBF:SceneInput','障害物は非空のセル配列で渡してください。');
end
qp=[];
% 力学のTaylor係数は障害物の個数・回転姿勢に依存しない。
% 1周期につき5本だけ計算し、全障害物に使い回す。
if profileOn, tStep=tic; end
jetCache=cbf_precompute_jets(mode,x,settings);
if profileOn, timing.jetMs=1000*toc(tStep); end
for i=1:numel(obstacles)
    obs=obstacles{i};
    if ~isfield(obs,'type')
        error('CBF:SceneType','障害物にtypeが必要です。');
    end
    if profileOn, tStep=tic; end
    [h,info]=cbf_evaluate_case(mode,shape,obs.type,x,settings,obs);
    if profileOn
        perObstacle(i).geometryMs=1000*toc(tStep);
        timing.geometryMs=timing.geometryMs+perObstacle(i).geometryMs;
        tStep=tic;
    end
    if i==1
        if profileOn
            [one,detail]=cbf_hocbf_evaluate(mode,shape,obs.type,x,settings,obs,h,info,'all',jetCache);
        else
            one=cbf_hocbf_evaluate(mode,shape,obs.type,x,settings,obs,h,info,'all',jetCache);
        end
        if profileOn
            perObstacle(i).highOrderMs=1000*toc(tStep);
            tMerge=tic;
        end
        qp=one;
        qp.labels=cellfun(@(name)sprintf('obs%d_%s',i,name), ...
            qp.labels,'UniformOutput',false);
    else
        % 2個目以降は幾何障害物のTaylorだけを生成。
        % 地面・入力上下限の高階係数は再計算しない（制約自体は保持）。
        if profileOn
            [only,detail]=cbf_hocbf_evaluate(mode,shape,obs.type,x,settings,obs,h,info,'obstacles',jetCache);
        else
            only=cbf_hocbf_evaluate(mode,shape,obs.type,x,settings,obs,h,info,'obstacles',jetCache);
        end
        if profileOn
            perObstacle(i).highOrderMs=1000*toc(tStep);
            tMerge=tic;
        end
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
    if profileOn
        timing.mergeMs=timing.mergeMs+1000*toc(tMerge);
        timing.highOrderMs=timing.highOrderMs+perObstacle(i).highOrderMs;
        perObstacle(i).parameterMs=detail.parametersMs;
        perObstacle(i).zeroTaylorMs=detail.zeroTaylorMs;
        perObstacle(i).fourBasisTaylorMs=detail.fourBasisTaylorMs;
        perObstacle(i).hocbfAssemblyMs=detail.hocbfAssemblyMs;
        perObstacle(i).realInputBoundsMs=detail.realInputBoundsMs;
        perObstacle(i).diagnosticsMs=detail.diagnosticsMs;
        perObstacle(i).totalMs=detail.totalMs;
    end
end
% 既知の数学的包含関係に基づく冗長な地面CBFの省略。
if profileOn, tStep=tic; end
qp=cbf_prune_proven_ground(qp,mode,shape,settings);
if profileOn
    timing.pruneMs=1000*toc(tStep);
    timing.totalMs=1000*toc(tAll);
    % mergeMs は制約の結合実測時間。他の計測オーバーヘッドは含まない。
    timing.overheadMs=max(0,timing.totalMs-timing.jetMs-timing.geometryMs- ...
        timing.highOrderMs-timing.mergeMs-timing.pruneMs);
    timing.perObstacle=perObstacle;
    timing.zeroTaylorMs=sum([perObstacle.zeroTaylorMs]);
    timing.fourBasisTaylorMs=sum([perObstacle.fourBasisTaylorMs]);
    timing.hocbfAssemblyMs=sum([perObstacle.hocbfAssemblyMs]);
    timing.parameterMs=sum([perObstacle.parameterMs]);
    timing.realInputBoundsMs=sum([perObstacle.realInputBoundsMs]);
    timing.diagnosticsMs=sum([perObstacle.diagnosticsMs]);
end
end
