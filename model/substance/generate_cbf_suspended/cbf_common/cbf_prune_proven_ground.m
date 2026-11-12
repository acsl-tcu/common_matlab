function qp=cbf_prune_proven_ground(qp,mode,shape,settings)
%CBF_PRUNE_PROVEN_GROUND 幾何学的・HOCBF的に冗長と証明できる地面行のみ除去。
% 次の2ケース以外は一切除去しない。
% B: 機体/吊り荷の半径がケーブル半径以上のとき、各端点の地面制約は重複。
% C(real): 両端球の半径が被覆球半径以上で全行が6次、同じゲインのとき
%          中間点の地面HOCBFは両端の凸結合から従う。
% C(virtual): 吊り荷6次と中間球4次が異なるので除去しない。
% 出力はQP行のみ削除。元の幾何hは保持する。
qp.prunedGroundLabels={};
if isfield(settings.cbf,'pruneRedundantGround') && ...
       ~settings.cbf.pruneRedundantGround
    return;
end
labels=qp.labels;
keep=true(numel(labels),1);
if strcmpi(shape,'B')
    if settings.shape.droneRadius >= settings.shape.cableRadius
        keep=keep & cellfun(@(v)isempty(strfind(v,'ground_cable_drone_end')),labels); %#ok<STREMP>
    end
    if settings.shape.loadRadius >= settings.shape.cableRadius
        keep=keep & cellfun(@(v)isempty(strfind(v,'ground_cable_load_end')),labels); %#ok<STREMP>
    end
elseif strcmpi(shape,'C') && strcmp(mode,'real')
    plan=cbf_choose_cable_spheres(settings);
    if settings.shape.droneRadius>=plan.sphereRadius && ...
       settings.shape.loadRadius>=plan.sphereRadius
        keep=keep & cellfun(@(v)isempty(strfind(v,'ground_cable_sphere_')),labels); %#ok<STREMP>
    end
end
qp.prunedGroundLabels=labels(~keep);
qp.A=qp.A(keep,:);
qp.b=qp.b(keep);
qp.labels=qp.labels(keep);
qp.order=qp.order(keep);
qp.psi=qp.psi(keep);
qp.psiInitiallyNonnegative=qp.psiInitiallyNonnegative(keep);
qp.effectNorm=qp.effectNorm(keep);
qp.degenerate=qp.degenerate(keep);
qp.uncontrollableViolation=qp.uncontrollableViolation(keep);
qp.relativeDegreeAtState=qp.relativeDegreeAtState(keep);
% earlyInputResidual は幾何行のみ。入力制限行には対応しない。
isInput=~cellfun('isempty',strfind(labels,'_thrust_')) | ... %#ok<STREMP>
        ~cellfun('isempty',strfind(labels,'_tau_')); %#ok<STREMP>
% シーンの行順は [障害物1＋地面＋入力, 障害物2, ...] であり、
% earlyInputResidualには入力の8行が含まれない点に注意する。
assert(numel(qp.earlyInputResidual)==sum(~isInput), ...
       'CBF:EarlyRowSize','低次入力残差と制約行の対応が壊れています。');
qp.earlyInputResidual=qp.earlyInputResidual(keep(~isInput));
end
