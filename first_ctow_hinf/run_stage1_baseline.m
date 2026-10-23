%RUN_STAGE1_BASELINE  段階1：基準（G0/G1/G2）の再現と評価表
P = ctow_params();
pass = ctow_selftest();
fprintf('\n--- 評価表 ---\n');
mG0 = ctow_eval(P.K.G0, 'G0 (current)', P, 0, false);
mG1 = ctow_eval(P.K.G1, 'G1 (LQR R=1e-2)', P);
mG2 = ctow_eval(P.K.G2, 'G2 (optimized)', P);
save('stage1_baseline.mat', 'mG0', 'mG1', 'mG2');
if pass
    fprintf('\n判定: OK（参照値を再現）\n次のアクション: 段階3（run_stage3_designA）へ進む。\n');
else
    fprintf('\n判定: NG（参照値を再現しない）\n次のアクション: ctow_hyb_rho の離散化（expm）・遅れ・FC遅れの入れ方を手順書3章と照合し、再現するまで設計に進まない。\n');
end
