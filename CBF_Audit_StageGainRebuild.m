function [modified,check] = CBF_Audit_StageGainRebuild(qp,shape,x,settings,obstacle,gains)
%CBF_AUDIT_STAGEGAINREBUILD 診断専用。地面CBFの段別ゲインのみ再構成。
% 既存CBF/QPの関数・ファイルは書き換えない。
% 6次の線形段別ゲイン (D+k6)...(D+k1)h の多項式を使用。
% 非地面CBF、実入力限界CBF、QPの目的関数は変更しない。
% NOTE: 現時点で適用できるのは実入力系のみ。
if numel(gains)~=6 || any(~isfinite(gains)) || any(gains<=0)
    error('CBF:AuditStageGains','正の6段ゲインが必要です。');
end
gains=gains(:).';
if ~strcmp(qp.mode,'real')
    error('CBF:AuditStageMode','実入力系だけが対象です。');
end
kind=obstacle.type;
[h,info]=cbf_evaluate_case('real',shape,kind,x,settings,obstacle); %#ok<ASGLU>
jet=cbf_precompute_jets('real',x,settings);
J=jet.jets{1};
P=cbf_parameter_vector(settings);
O=cbf_obstacle_vector(obstacle,kind);
plan=cbf_choose_cable_spheres(settings);
H0=cbf_geometry_taylor(shape,kind,J.pLoad,J.qCable,J.pDrone,P,O,plan,'all');
if size(H0,1)~=numel(info.labels)
    error('CBF:AuditStageShape','幾何行とTaylor行が一致しません。');
end
k0=settings.cbf.hocbfGain;
if isfield(settings.cbf,'groundGain') && ~isempty(settings.cbf.groundGain)
    k0=settings.cbf.groundGain;
end
oldPoly=1;newPoly=1;
for j=1:6
    oldPoly=conv(oldPoly,[1 k0]);
    newPoly=conv(newPoly,[1 gains(j)]);
end
modified=qp; nChanged=0;
for i=1:numel(qp.labels)
    label=qp.labels{i};
    if ~contains(label,'ground_'),continue;end
    loc=strrep(label,'obs1_','');
    j=find(strcmp(info.labels,loc),1);
    if isempty(j)
        error('CBF:AuditStageLabel','対象地面行が元の幾何計算にありません: %s',label);
    end
    r=qp.order(i);
    if r~=6,error('CBF:AuditStageOrder','実入力地面制約の設計次数が6以外です。');end
    raw=H0(j,1:7).*factorial(0:6);
    oldLower=0;newLower=0;
    for m=0:5
        oldLower=oldLower+oldPoly(7-m)*raw(m+1);
        newLower=newLower+newPoly(7-m)*raw(m+1);
    end
    % -h6 -(newLower) = original b + oldLower - newLower
    modified.b(i)=qp.b(i)+oldLower-newLower;
    pp=zeros(6,1);
    poly=1;
    for stage=0:5
        if stage>0,poly=conv(poly,[1 gains(stage)]);end
        for m=0:stage
            pp(stage+1)=pp(stage+1)+poly(stage-m+1)*raw(m+1);
        end
    end
    modified.psi{i}=pp;
    modified.psiInitiallyNonnegative(i)=all(pp>=0);
    nChanged=nChanged+1;
end
check=struct('groundRowsModified',nChanged,'coefficientError',norm(qp.A-modified.A,Inf),...
    'maxBChange',norm(qp.b-modified.b,Inf));
end
