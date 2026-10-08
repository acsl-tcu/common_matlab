classdef HLC_HINF_SUSPENDED_LOAD < handle
    % クアッドコプター用階層型線形化(HL)＋H∞設計による仮想入力ゲイン F1〜F4
    %
    % F1〜F4 は次のどちらかで決まる:
    %   (a) Param.F1〜F4 を直接与える（手動ゲイン）
    %   (b) 階層型線形化の各サブシステムをH∞で設計して自動生成する
    %       （Param.useHinf=true、または Param.F1 が無いとき）
    %
    % H∞設計は Controller_HL_Hinf_Suspended_Load と同じ構成:
    %   外乱 Wdst はプラント出力(位置)に加算、測定ノイズ Wnoise はその後に加算
    %   Wpos は外乱込みの位置、Wact は Act の出力を評価
    %   hinfsyn(公称モデル) → 閉ループ極 → place で静的ゲイン F を作成
    % 設計は最初の1回だけ行い、設定が変わるまでキャッシュする。
    %
    % 使い方:
    %   param = struct('dt', dt, 'useHinf', true);
    %   agent.controller.set_function_class("hlc_suspended", HLC_HINF_SUSPENDED_LOAD(agent, param));
    %
    % 設計の設定（すべて省略可）:
    %   param.hinf.mode    = 'sim' | 'exp'    % 重みの組。省略時は agent.plant の種類で自動選択
    %   param.hinf.verbose = true             % γ と閉ループ極を表示
    %   param.hinf.weights.wd_dst       = 10;                          % 外乱の帯域 [rad/s]
    %   param.hinf.weights.noise        = 0.01;                        % 測定ノイズの大きさ
    %   param.hinf.weights.Wact.x       = 0.8*tf([1 0.5],[1 5]);       % z,x,y,yaw 個別に上書き可
    %   param.hinf.weights.PosTarget.x  = tf(1,[1/0.5 1]);
    % 必要: Control System Toolbox / Robust Control Toolbox (hinfsyn)
    properties
        self
        result
        param
        gainCache     % 設計済みゲイン（再計算を避ける）
        gainCacheKey  % 設計時の設定（変わったら再設計）
    end

    methods

        function obj = HLC_HINF_SUSPENDED_LOAD(self, param)
            obj.self = self;
            obj.param = param;
        end

        function result = do(obj, varargin)
            Param = obj.param; % param (optional) : 構造体：dt、ゲインF1-F4 または H∞設計の設定
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
            P(7)=1.2 ;
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
            xd = [xd; zeros(28 - size(xd, 1), 1)]; % 足りない分は0で埋める．
            obj.result.xdresult=xd;

            % 階層型線形化による入力計算
            % 仮想入力のゲイン（手動指定 or H∞設計）
            [F1, F2, F3, F4, gainInfo] = obj.getGains(Param);
            obj.result.gain.F1=F1; % z方向サブシステムのゲイン
            obj.result.gain.F2=F2; % x方向サブシステムのゲイン
            obj.result.gain.F3=F3; % y方向サブシステムのゲイン
            obj.result.gain.F4=F4; % yaw方向サブシステムのゲイン
            obj.result.gain.info=gainInfo; % 設計方法・γなど

            vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
            vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算

            uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
            % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
            %tic
            beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
            vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
            us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
            %obj.result.aa=toc;
            %
            tmp = [uf(1); us]; % 実入力へ変換
            obj.result.tmp = tmp; % 入力に制限を付けてない値を格納

            % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
            obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; %+[normrnd(0,0.01,1);normrnd(0,0.001,[3,1])]*1; %入力にノイズを付与可能
            obj.result.xd = xd;
            obj.result.x = x;
            result = obj.result;
        end

        function show(obj)
            obj.result
        end

        function [F1, F2, F3, F4, info] = getGains(obj, Param)
            % ゲインの取得。H∞設計は設定が変わらない限り1回だけ計算してキャッシュする。
            has = @HLC_HINF_SUSPENDED_LOAD.hasField; % struct でもオブジェクトでも判定できる
            useHinf = (has(Param, 'useHinf') && Param.useHinf) || ~has(Param, 'F1');
            if ~useHinf
                F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4;
                info = struct('method', 'manual');
                return
            end
            if has(Param, 'hinf')
                opt = Param.hinf;
            else
                opt = struct();
            end
            % 重みの組（sim / exp）：指定が無ければ plant の種類で決める
            mode = 'sim';
            try
                if string(class(obj.self.plant)) == "DRONE_EXP_MODEL", mode = 'exp'; end
            catch
            end
            if HLC_HINF_SUSPENDED_LOAD.hasField(opt, 'mode'), mode = opt.mode; end

            key = {opt, mode};
            if isempty(obj.gainCache) || ~isequal(obj.gainCacheKey, key)
                obj.gainCache = HLC_HINF_SUSPENDED_LOAD.designHinfGains(opt, mode);
                obj.gainCacheKey = key;
            end
            F1 = obj.gainCache.F1; F2 = obj.gainCache.F2;
            F3 = obj.gainCache.F3; F4 = obj.gainCache.F4;
            info = obj.gainCache.info;
        end
    end

    methods (Static)

        function G = designHinfGains(opt, mode)
            % 4つのサブシステムそれぞれでH∞設計し、静的ゲイン F1〜F4 を返す。
            %   z: 2次, x: 6次, y: 6次, yaw: 2次（Brunovsky標準形の積分器列）
            if nargin < 1, opt = struct(); end
            if nargin < 2, mode = 'sim'; end
            pick = @HLC_HINF_SUSPENDED_LOAD.pick;
            verbose = pick(opt, 'verbose', true);

            W = HLC_HINF_SUSPENDED_LOAD.defaultWeights(mode);
            if HLC_HINF_SUSPENDED_LOAD.hasField(opt, 'weights')
                W = HLC_HINF_SUSPENDED_LOAD.mergeStruct(W, opt.weights);
            end

            axisNames = {'z', 'x', 'y', 'yaw'};
            Fs = cell(1, 4);
            gam = zeros(1, 4);
            poles = cell(1, 4);
            for k = 1:4
                ax = axisNames{k};
                [IC, A, B, n] = HLC_HINF_SUSPENDED_LOAD.buildPlant(ax, W);
                [K, ~, gam(k)] = hinfsyn(IC, n, 1);   % 観測 n 個、制御入力 1 個
                [Fs{k}, poles{k}] = HLC_HINF_SUSPENDED_LOAD.staticGain(K, A, B, n);
                if verbose
                    fprintf('%-3s gamma = %.4f\n', ax, gam(k));
                    fprintf('%-3s 閉ループ極: ', ax); disp(poles{k}.');
                    fprintf('%-3s 固有値(A-BF): ', ax); disp(eig(A - B * Fs{k}).');
                end
            end
            G.F1 = Fs{1}; G.F2 = Fs{2}; G.F3 = Fs{3}; G.F4 = Fs{4};
            G.info = struct('method', 'hinfsyn', 'mode', mode, 'gamma', gam, ...
                'axis', {axisNames}, 'poles', {poles});
        end

        function [IC, A, B, n] = buildPlant(ax, W)
            % 軸 ax の一般化プラント（外乱は位置出力に加算、その後に測定ノイズを加算）
            %   入力: [d(外乱); n(測定ノイズ); u(制御入力)]
            %   出力: [e1(Wact); e2(Wpos); y1..yn(測定)]
            switch ax
                case {'z', 'yaw'}
                    n = 2; ipos = 1;
                    A = [0 1; 0 0]; B = [0; 1];
                    out = {['p' ax '0']; ['v' ax]};
                case {'x', 'y'}
                    n = 6; ipos = 3;
                    A = diag(ones(5, 1), 1); B = [0; 0; 0; 0; 0; 1];
                    out = {['pL' ax]; ['vL' ax]; ['p' ax '0']; ['v' ax]; ['acc' ax]; ['jerk' ax]};
                otherwise
                    error('HLC_HINF_SUSPENDED_LOAD:axis', '不明な軸: %s', ax);
            end
            posName = ['p' ax];            % 外乱を足した後の位置
            pos0    = out{ipos};           % 外乱を足す前の位置

            Pl = ss(A, B, eye(n), 0);      % uAX → プラント → out
            Pl.InputName  = {['u' ax]};
            Pl.OutputName = out;

            % アクチュエータ（評価用の枝）。公称モデルはゲイン1（不確かさは hinfsyn では使わない）
            Act = ss(1);
            Act.InputName  = ['u' ax];
            Act.OutputName = ['act' ax];

            Wd = tf(1, [1 / W.wd_dst, 1]);               % 低周波外乱
            Wd.InputName = ['d' ax]; Wd.OutputName = ['r' ax];

            Wa = W.Wact.(ax);
            Wa.InputName = ['act' ax]; Wa.OutputName = ['e1' ax];

            Wp = 1 / W.PosTarget.(ax);
            Wp.InputName = posName; Wp.OutputName = ['e2' ax];

            Wn = ss(W.noise);
            Wn.InputName = ['n' ax]; Wn.OutputName = ['Wn' ax];

            dst = sumblk(sprintf('%s = %s + r%s', posName, pos0, ax));   % 外乱を位置に加算

            meas = cell(1, n);                                           % y_k = 出力_k + ノイズ
            ynames = cell(n, 1);
            for k = 1:n
                src = out{k};
                if k == ipos, src = posName; end
                ynames{k} = sprintf('y%d%s', k, ax);
                meas{k} = sumblk(sprintf('%s = %s + Wn%s', ynames{k}, src, ax));
            end

            IC = connect(Pl, Act, Wd, dst, Wp, Wa, Wn, meas{:}, ...
                {['d' ax]; ['n' ax]; ['u' ax]}, ...
                [{['e1' ax]; ['e2' ax]}; ynames]);
        end

        function [F, poles] = staticGain(K, A, B, n)
            % hinfsyn の動的補償器 K の閉ループ極を求め、その極を place で状態フィードバックゲインに写す
            C = eye(n);
            [AK, BK, CK, DK] = ssdata(ss(K));
            Acl = [A + B * DK * C, B * CK; BK * C, AK];
            poles = sort(eig(Acl), 'ComparisonMethod', 'real');
            try
                F = place(A, B, HLC_HINF_SUSPENDED_LOAD.pickpoles(poles, n));
            catch err
                error('HLC_HINF_SUSPENDED_LOAD:place', 'place失敗: 閉ループ極を確認してください (%s)', err.message);
            end
        end

        function p = pickpoles(P, n)
            % 実部が小さい（左側）順に、共役ペアを崩さず n 個の極を選ぶ
            P = P(:);
            [~, idx] = sort(real(P));
            P = P(idx);
            p = [];
            used = false(numel(P), 1);
            for k = 1:numel(P)
                if used(k), continue; end
                used(k) = true;
                if abs(imag(P(k))) < 1e-9 * max(1, abs(P(k)))
                    if numel(p) + 1 <= n
                        p(end+1, 1) = real(P(k)); %#ok<AGROW>
                    end
                else
                    d = abs(P - conj(P(k)));
                    d(used) = inf;
                    [dm, j] = min(d);
                    if ~isempty(j) && dm < 1e-6 * max(1, abs(P(k)))
                        used(j) = true;
                        if numel(p) + 2 <= n
                            p = [p; P(k); conj(P(k))]; %#ok<AGROW>
                        end
                    end
                end
                if numel(p) == n, break; end
            end
            if numel(p) < n
                error('pickpoles: 共役ペアを崩さずに %d 個選べませんでした', n);
            end
        end

        function W = defaultWeights(mode)
            % 重み関数の既定値（sim用 / exp用(保守的)）
            W.wd_dst = 10;     % 外乱の帯域。ss(1.0) だと Wpos に直接入って非プロパーになり hinfsyn が失敗する
            W.noise  = 0.01;
            if strcmp(mode, 'exp')
                W.Wact.z   = 0.8 * tf([1 1],   [1 10]);
                W.Wact.x   = 0.8 * tf([1 0.3], [1 3]);
                W.Wact.y   = 0.8 * tf([1 0.3], [1 3]);
                W.Wact.yaw = 0.8 * tf([1 0.5], [1 5]);
                W.PosTarget.z   = tf(1, [1 / 1.0, 1]);
                W.PosTarget.x   = tf(1, [1 / 0.3, 1]);
                W.PosTarget.y   = tf(1, [1 / 0.3, 1]);
                W.PosTarget.yaw = tf(1, [1 / 0.5, 1]);
            else
                W.Wact.z   = 0.8 * tf([1 2],   [1 20]);
                W.Wact.x   = 0.8 * tf([1 0.5], [1 5]);
                W.Wact.y   = 0.8 * tf([1 0.5], [1 5]);
                W.Wact.yaw = 0.8 * tf([1 1],   [1 10]);
                W.PosTarget.z   = tf(1, [1 / 2.0, 1]);
                W.PosTarget.x   = tf(1, [1 / 0.5, 1]);
                W.PosTarget.y   = tf(1, [1 / 0.5, 1]);
                W.PosTarget.yaw = tf(1, [1 / 1.0, 1]);
            end
        end

        function a = mergeStruct(a, b)
            % b の内容で a を上書き（struct は再帰的に統合）
            f = fieldnames(b);
            for i = 1:numel(f)
                if isstruct(b.(f{i})) && isfield(a, f{i}) && isstruct(a.(f{i}))
                    a.(f{i}) = HLC_HINF_SUSPENDED_LOAD.mergeStruct(a.(f{i}), b.(f{i}));
                else
                    a.(f{i}) = b.(f{i});
                end
            end
        end

        function r = hasField(s, name)
            r = (isstruct(s) && isfield(s, name)) || (isobject(s) && isprop(s, name));
        end

        function v = pick(s, name, default)
            if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                v = s.(name);
            else
                v = default;
            end
        end
    end
end