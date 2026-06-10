classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% クアッドコプター用階層型線形化を使った入力算出
properties
    self
    result
    param
end

methods

    function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
        obj.self = self;
        obj.param = param;

    end

    function result = do(obj, varargin)
        Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
        model = obj.self.estimator.result; % 推定した状態
        ref = obj.self.reference.result; % 目標値

        % 目標値を取得
        if isprop(ref.state, 'xd')
            xd = ref.state.xd; % 20次元の目標値に対応する用
        else
            xd = ref.state.get();
        end

        pL = model.state.pL;
        if isprop(model.state, "pT")
            pT = model.state.pT;
        else
            delta = pL - model.state.p;
            if norm(delta) > 1e-9
                pT = delta / norm(delta);
            else
                pT = [0; 0; -1];
            end
        end
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];

        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
        % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
        % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
        % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
        yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
        yawd = xd(4); % 目標yaw角
        yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
        yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
        xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
        %目標値の格納
        xd = [xd; zeros(28 - size(xd, 1), 1)];

        % 階層型線形化による入力計算
        % 仮想入力のゲイン
        F1 = Param.F1; % z方向サブシステムのゲイン
        F2 = Param.F2; % x方向サブシステムのゲイン
        F3 = Param.F3; % y方向サブシステムのゲイン
        F4 = Param.F4; % yaw方向サブシステムのゲイン

        % -----------------------------------------------------------------
        % 名目（Nominal）仮想操作量の計算
        % -----------------------------------------------------------------
        vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
        vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算

        v_nominal = [vf_nominal(1); vs_nominal(1); vs_nominal(2)]; % z,x,yの仮想入力の加速度の抜き出し

        % -----------------------------------------------------------------
        % 障害物定義・保護球定義
        % -----------------------------------------------------------------
        obs_env = ENVIRONMENT_OBSTACLE(); % 障害物配列の一括取得
        num_obstacles = length(obs_env);  % 障害物の数を自動カウント

        protection_config = [
            % 位置割合 (λ_new),  その位置の球の半径 (rq)
            0.00,               0.30;   % 1つ目：吊り荷本体（起点）
            0.25,               0.25;   % 2つ目：ワイヤー下部
            0.50,               0.25;   % 3つ目：ワイヤー中間
            0.75,               0.25;   % 4つ目：ワイヤー上部
            1.00,               0.40;   % 5つ目：ドローン本体（終点）
        ];

        v_safe = v_nominal; % 安全入力に公称入力を

        %ここにいろいろなものを入れる

        % =================================================================
        % 3. 安全化された仮想操作量を、各階層の変数へ再分配
        % =================================================================
        vf_safe = vf_nominal;
        vs_safe = vs_nominal;
        
        vf_safe(1) = v_safe(1); % 安全化されたZ軸の仮想入力
        vs_safe(1) = v_safe(2); % 安全化されたX軸の仮想入力
        vs_safe(2) = v_safe(3); % 安全化されたY軸の仮想入力

        % =================================================================
        % 4. 下流レイヤーへの非線形相殺・実物理入力への変換
        % =================================================================
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P); % 第一層の仮想入力の実入力(推力)への変換
        % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
        %tic
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P); % 第二層のbetaの逆行列
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P); % 第二層のvs - alpha
        us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
        %obj.result.aa=toc;
        tmp = [uf(1); us]; % 実入力へ変換
        obj.result.tmp = tmp; % 入力に制限を付けてない値を格納

        % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
        %disp("time: "+ num2str(t,2)+" z position of drone:
        %"+num2str(model.state.p(3),3)+" estimated load mass:
        %"+num2str(P(6),4)+" dst:(x,y) "+num2str(P(end-1:end),4))
        obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; %+[normrnd(0,0.01,1);normrnd(0,0.001,[3,1])]*1; %入力にノイズを付与可能
        % obj.result.input = [0.0,0,0,(0.5236+0.04)*9.81]';%
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;

    end

    function show(obj)
        obj.result
    end
end

end