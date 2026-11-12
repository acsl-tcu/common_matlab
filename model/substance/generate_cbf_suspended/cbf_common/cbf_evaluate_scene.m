function [h,labels] = cbf_evaluate_scene(mode,shape,x,settings,obstacles)
%CBF_EVALUATE_SCENE 複数障害物（球と楕円体の混在も可）の安全関数をまとめる。
% obstacles はセル配列: {obsSphere,obsEllipsoid,...}
% 各障害物に type='sphere' or 'ellipsoid' が必要。
% 地面制約は最初の障害物から1回だけ取り込み重複させない。
if ~iscell(obstacles) || isempty(obstacles)
    error('CBF:SceneInput','obstacles は1個以上の構造体を含むセル配列です。');
end
h=[]; labels=cell(0,1);
for i=1:numel(obstacles)
    obs=obstacles{i};
    if ~isfield(obs,'type'),error('CBF:SceneType','各障害物にtypeが必要です。');end
    [hi,meta]=cbf_evaluate_case(mode,shape,obs.type,x,settings,obs);
    for j=1:numel(hi)
        label=meta.labels{j};
        isGround=strncmp(label,'ground_',7);
        if i==1 || ~isGround
            h=[h;hi(j)];
            labels{end+1,1}=sprintf('obs%d_%s',i,label);
        end
    end
end
end
