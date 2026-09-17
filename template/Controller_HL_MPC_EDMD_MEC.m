function Controller = Controller_HL_MPC_EDMD_MEC(dt, agent, model_file, state_source, apply_compensation, mpc_profile)
% Controller_HL_MPC_EDMD_MEC
% HL_MPC_EDMD_MEC 用パラメータ生成。
% Controller_HL_MPC（二層QP-MPC設定）を継承し、EDMD残差補償の設定を追加。
% 補償パラメータは HLC_EDMD_MEC で検証済みの「Mode2 + 平滑」設定を踏襲。

    if nargin < 3 || isempty(model_file)
        model_file = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\edmd_residual_model_hlmpc_plant.mat';
    end
    if nargin < 4 || isempty(state_source)
        state_source = 'plant';
    end
    if nargin < 5 || isempty(apply_compensation)
        apply_compensation = 1;
    end
    if nargin < 6 || isempty(mpc_profile)
        mpc_profile = 'mismatch';
    end
    mpc_profile = lower(char(string(mpc_profile)));

    % 二層QP-MPCの基本設定をそのまま継承
    % （H=55, 大Q, Rd/Rd0差分項など、実機検証済みの重みをそのまま使う）
    Controller = Controller_HL_MPC(dt, agent);

    % --- 重みバージョン確認：実機版(Rd/Rd0含む)を継承できているか ---
    if ~isfield(Controller.mpc, 'Rd') || ~isfield(Controller.mpc, 'Rd0')
        warning('Controller_HL_MPC_EDMD_MEC:oldWeights', ...
            ['継承した Controller_HL_MPC に Rd/Rd0 がありません。', ...
             '実機版に更新してください（pull忘れに注意）。']);
    end

    % ===== 仿真専用: 予測域 H を短縮して外乱・劣化が見えるようにする =====
    % 実機の H=55 は予測域が長すぎ、MPCが外乱を予見して補償してしまい、
    % degraded と nominal の xy がほぼ重なる（=劣化が見えない）。
    % 論文の LPV-MPC は H=12 で 0.06→0.12 の慣性誤差を明確に出している。
    % HL-MPC も H=12 に合わせ、慣性失配がxyに見えるようにする。
    % ★重要: この短縮は3条件すべてに同一適用される（公平）。
    %   実機の Controller_HL_MPC は変更しない（保護）。
    sim_horizon = 12;   % 論文LPV-MPCと一致（実機55→仿真12）
    Controller_original = Controller;
    restore_original_profile = false;
    switch mpc_profile
        case {'tracking', 'original', 'hlmpc'}
            restore_original_profile = true;
            sim_horizon = Controller.H;
            qxy = diag([500, 200, 60, 6]);
        case {'balanced', 'saddle'}
            sim_horizon = 25;
            qxy = diag([2500, 800, 180, 6]);
        otherwise
            sim_horizon = 12;
            qxy = diag([500, 200, 60, 6]);
    end
    Controller.H = sim_horizon;
    Controller.mpc.N = sim_horizon;
    Controller.mpc.Nvf = sim_horizon;

    % ===== 仿真専用: 位置重みを論文LPV-MPC水準(500)に下げる =====
    % 実機の x/y 位置重み(6000/5200)が強すぎて、慣性失配があっても
    % MPCが軌道を追従しきってしまう（=偏差が見えない）。
    % 論文LPV-MPCの位置重み diag([500,500,600]) 水準に下げ、
    % 慣性失配の影響が軌道全体の偏差として現れるようにする。
    % z層(s=1)とyaw層(s=4)は元のまま。x/y層(s=2,3)の位置重みのみ下げる。
    Controller.mpc.Q{2} = diag([500, 200, 60, 6]);   % x（位置6000→500）
    Controller.mpc.Q{3} = diag([500, 200, 60, 6]);   % y（位置5200→500）

    % H変更＋重み変更に伴い、全4サブシステムのMPC行列を再構築
    % （Controller_HL_MPC と同一の構築式）
    Controller.mpc.Q{2} = qxy;
    Controller.mpc.Q{3} = qxy;

    for s = 1:4
        A = Controller.mpc.A{s};
        B = Controller.mpc.B{s};
        N = Controller.mpc.N;   % = sim_horizon（NvfもHと同じ）
        Q = Controller.mpc.Q{s};
        R = Controller.mpc.R{s};
        nx = size(A, 1);
        nu = size(B, 2);
        Sx = zeros(nx*N, nx);
        Su = zeros(nx*N, nu*N);
        for i = 1:N
            Sx((i-1)*nx+1:i*nx, :) = A^i;
            for j = 1:i
                Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A^(i-j)*B;
            end
        end
        Qb = kron(eye(N), Q);
        Rb = kron(eye(N), R);
        Rd = Controller.mpc.Rd{s};
        D  = diff(eye(N));
        Controller.mpc.Sx{s} = Sx;
        Controller.mpc.Su{s} = Su;
        Controller.mpc.Qb{s} = Qb;
        Hq = 2 * (Su' * Qb * Su + Rb + Rd * (D' * D));
        Controller.mpc.Hq{s} = (Hq + Hq') / 2;
        Controller.mpc.lbq{s} = repmat(Controller.mpc.lb{s}, N, 1);
        Controller.mpc.ubq{s} = repmat(Controller.mpc.ub{s}, N, 1);
    end
    fprintf('[sim_setup] H=%d, xy位置重み→500 (論文LPV-MPC水準)。3条件共通。\n', ...
        sim_horizon);

    % HL_MPC には explore 機構が無いので、補償クラス側のデフォルト（OFF）を使う。
    % 劣化飛行で激励を入れたい場合は Sim 側で Controller.explore.enable=1 を設定。
    if restore_original_profile
        Controller = Controller_original;
    end
    fprintf('[sim_setup] profile=%s, H=%d\n', mpc_profile, Controller.H);

    Controller.edmd_disturbance_enable = 0;

    % ===== 残差補償設定（HLC_EDMD_MEC 検証済みの平滑設定）=====
    Controller.residual.mode = 2;            % Mode2: 質量/慣性偏差に効く
    Controller.residual.model_file = model_file;
    Controller.residual.state_source = state_source;
    Controller.residual.apply_compensation = apply_compensation;

    % --- 平滑化ノブ ---
    Controller.residual.alpha = 0.40;        % 補償ゲイン
    Controller.residual.beta  = 0.15;        % ローパス（小さいほど滑らか）
    Controller.residual.du_max = [0.30; 0.012; 0.012; 0.000];

    % --- チャンネル：推力 + roll + pitch 有効、yaw無効 ---
    Controller.residual.active_channels = [1; 1; 1; 0];

    % --- 数値安定 ---
    Controller.residual.pinv_damping = 1e-1;
    Controller.residual.q_scale = 50.0;
    Controller.residual.r_scale = 0.1;
    Controller.residual.use_reference = 1;

    % Residual cancellation. Keep this less aggressive than the old
    % reference-chasing mode, but large enough to be visible in closed loop.
    Controller.residual.mode = 3;
    Controller.residual.alpha = 0.35;
    Controller.residual.beta = 0.15;
    Controller.residual.du_max = [0.16; 0.008; 0.008; 0.000];
    Controller.residual.pinv_damping = 0.3;

    if evalin('base', 'exist(''hlmpc_residual_override'', ''var'')')
        residual_override = evalin('base', 'hlmpc_residual_override');
        if isstruct(residual_override)
            fns = fieldnames(residual_override);
            for k = 1:numel(fns)
                Controller.residual.(fns{k}) = residual_override.(fns{k});
            end
            fprintf('[sim_setup] residual override: mode=%d alpha=%.3f beta=%.3f damping=%.3f du_max=[%.3g %.3g %.3g %.3g]\n', ...
                Controller.residual.mode, Controller.residual.alpha, Controller.residual.beta, ...
                Controller.residual.pinv_damping, Controller.residual.du_max(1), ...
                Controller.residual.du_max(2), Controller.residual.du_max(3), Controller.residual.du_max(4));
        end
    end

    % --- 補償適用エンベロープ ---
    Controller.residual.max_pos_err = 1.5;
    Controller.residual.max_ang_err = 0.35;
    Controller.residual.max_vel_err = 2.5;
    Controller.residual.max_w_err = 3.0;

end
