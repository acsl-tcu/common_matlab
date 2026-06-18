%% ====================================================================
%  EDMD-MEC 实机对比脚本  compare_pert_comp.m
%  比较"不补偿(Pert.)"和"补偿(Comp.)"两次飞行的位置 RMSE 与轨迹。
%  改 CONFIG 两个文件名，按 F5 运行。
%% ====================================================================
clear; clc; close all;

%% ===================== 【CONFIG：只改这里】 =====================
FILE_PERT = 'exphlmpcstarnomec_Log(18-Jun-2026_17_02_48).mat';   % 不补偿那次(之前的)
FILE_COMP = 'exphlmpcstarmec2_Log(18-Jun-2026_17_05_11).mat';               % 开补偿那次(刚收集的)
PHASE_VAL = 102;
TITLE     = 'HL: lemniscate';
%% ===============================================================

P = load_run(FILE_PERT, PHASE_VAL);   % .t .p(实测Nx3) .pref(参考Nx3)
C = load_run(FILE_COMP, PHASE_VAL);

rmseP = rmse_pos(P);
rmseC = rmse_pos(C);
impr  = (rmseP - rmseC)/rmseP*100;

fprintf('\n==== %s ====\n', TITLE);
fprintf('Pert.(无补偿) 位置RMSE = %.4f m\n', rmseP);
fprintf('Comp.(补偿)   位置RMSE = %.4f m\n', rmseC);
fprintf('改善 Δ                 = %.2f %%   %s\n', impr, ternary(impr>0,'(变好)','(变差)'));

% ---- 叠图：参考 vs Pert vs Comp ----
figure('Name',TITLE,'Color','w');
plot3(P.pref(:,1),P.pref(:,2),P.pref(:,3),'k--','LineWidth',1.2); hold on; grid on;
plot3(P.p(:,1),   P.p(:,2),   P.p(:,3),   'r-','LineWidth',1.0);
plot3(C.p(:,1),   C.p(:,2),   C.p(:,3),   'g-','LineWidth',1.2);
axis equal; xlabel('p_1'); ylabel('p_2'); zlabel('p_3');
legend('reference','Pert. (no MEC)','Comp. (EDMD-MEC)','Location','best');
title(sprintf('%s   RMSE %.3f→%.3f m  (%.1f%%)', TITLE, rmseP, rmseC, impr));
view(2);   % 顶视图看8字；想看3D注释掉这行

% ---- 误差随时间 ----
figure('Name',[TITLE ' error(t)'],'Color','w');
eP=vecnorm(P.p-P.pref,2,2); eC=vecnorm(C.p-C.pref,2,2);
plot(P.t-P.t(1),eP,'r'); hold on; plot(C.t-C.t(1),eC,'g'); grid on;
xlabel('t [s]'); ylabel('|p-p_{ref}| [m]'); legend('Pert.','Comp.');
title([TITLE ' position error']);


%% ===================== 局部函数 =====================
function r = load_run(fname, phaseVal)
    log = LOGGER(fname); D = log.Data;
    ph = D.phase(:); idx = find(ph==phaseVal);
    if numel(idx)<2, error('%s: phase==%g 少于2个', fname, phaseVal); end
    sel = idx(2):idx(end);
    r.t = D.t(sel); r.t = r.t(:);
    res = D.agent.estimator.result;          % 实测
    ref = D.agent.reference.result;          % 参考 ★若路径不同改这里
    n=numel(sel); r.p=zeros(n,3); r.pref=zeros(n,3);
    for ii=1:n
        k=sel(ii);
        r.p(ii,:)    = res{1,k}.state.p(:).';
        r.pref(ii,:) = ref{1,k}.state.p(:).';   % ★参考位置;若字段不同改这里
    end
end

function e = rmse_pos(r)
    d = vecnorm(r.p - r.pref, 2, 2);
    e = sqrt(mean(d.^2));
end

function o = ternary(c,a,b), if c, o=a; else, o=b; end, end