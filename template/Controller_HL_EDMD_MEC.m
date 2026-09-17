function Controller = Controller_HL_EDMD_MEC(dt, model_file, state_source, apply_compensation)
% Controller_HL_EDMD_MEC
% HLC_EDMD_MEC 用パラメータ生成（ノイズ環境で滑らかに補償する設定）。
%
% 変更方針：
%   - mode 3 -> 2 : 質量偏差は B_err 経由でなく標称予測との差分で補うため、
%                   A_nom/B_nom の予測を使う Mode2 が質量・慣性変化に効く。
%   - beta 小さめ : 補償信号を一次ローパスで平滑化（抖動抑制の本丸）。
%   - alpha 控えめ: ゲインを上げすぎず滑らかさ優先。

    if nargin < 2 || isempty(model_file)
        model_file = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\edmd_residual_model_hl_plant.mat';
    end
    if nargin < 3 || isempty(state_source)
        state_source = 'plant';
    end
    if nargin < 4 || isempty(apply_compensation)
        apply_compensation = 1;
    end

    Controller = Controller_HL(dt);
    Controller.explore.enable = 0;          % 補償飛行では激励OFF
    Controller.edmd_disturbance_enable = 0; % 外乱注入もOFF

    % ===== 残差補償設定（ノイズ下で滑らかに）=====
    Controller.residual.mode = 2;           % ★ 3->2: 質量/慣性偏差に効くモード
    Controller.residual.model_file = model_file;
    Controller.residual.state_source = state_source;
    Controller.residual.apply_compensation = apply_compensation;

    % --- 平滑化の主要ノブ ---
    Controller.residual.alpha = 0.40;       % 補償ゲイン (0.25->0.40) 控えめに上げる
    Controller.residual.beta  = 0.15;       % ★ ローパス強め (0.20->0.15) 抖動抑制
    Controller.residual.du_max = [0.30; 0.012; 0.012; 0.000]; % 推力補償枠を拡大

    % --- チャンネル選択：推力 + roll + pitch を有効、yaw無効 ---
    Controller.residual.active_channels = [1; 1; 1; 0];

    % --- 数値安定 ---
    Controller.residual.pinv_damping = 1e-1; % 5e-2 -> 1e-1 補償方向を滑らかに
    Controller.residual.q_scale = 50.0;
    Controller.residual.r_scale = 0.1;
    Controller.residual.use_reference = 1;

    % --- 補償を適用する誤差エンベロープ（大きく外れたら補償停止）---
    Controller.residual.max_pos_err = 1.5;
    Controller.residual.max_ang_err = 0.35;
    Controller.residual.max_vel_err = 2.5;
    Controller.residual.max_w_err = 3.0;

end
