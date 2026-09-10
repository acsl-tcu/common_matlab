classdef HLC_SUSPENDED_LOAD < handle
    % クアッドコプター用階層型線形化を使った入力算出
    properties
        self
        result
        param
        cha='s';
        noize=0;
        t=[];
        notchx
        notchy
    end

    methods

        function obj = HLC_SUSPENDED_LOAD(self, param)
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
<<<<<<< Updated upstream
            obj.result.xdresult=xd;
=======
            % tt=obj.notchFilter([0;xd(1:3)], Param.dt);
            % xd(1)=tt(2);
>>>>>>> Stashed changes

            % 階層型線形化による入力計算
            % 仮想入力のゲイン
            F1 = Param.F1; % z方向サブシステムのゲイン
            F2 = Param.F2; % x方向サブシステムのゲイン
            F3 = Param.F3; % y方向サブシステムのゲイン
            F4 = Param.F4; % yaw方向サブシステムのゲイン

            vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
            vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
            % v= obj.notchFilter([0;vs'], Param.dt);
            % vs(1)=v(2);
            uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
            % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
            %tic
            beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
            vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
            us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
            %obj.result.aa=toc;
<<<<<<< Updated upstream
            % obj.cha = varargin{2};
            % 
=======
            obj.cha = varargin{2};

>>>>>>> Stashed changes
            % if obj.cha=="f"
            %     % chirp ノイズ
            %     %基本設定
            %     noize=1;
            %     disp(noize)
            % 
            %     if isempty(obj.t)    %flightからreferenceの時間を開始
            %         obj.t=varargin{1}.t; % 目標重心位置（絶対座標）
            %     end
            %     t_now = varargin{1}.t-obj.t;       %flight開始時の時刻から開始
            %     obj.result.ftime=t_now;
            %     t_end=varargin{1}.te;
            % 
            %     %pitch,roll 統一
<<<<<<< Updated upstream
            %     u=1e-1*chirp(t_now, 0, t_end, 0.25, [], -90 );
=======
            %     u=0.02*chirp(t_now, 0.05, t_end, 3.0 , "logarithmic" );
            %     %%f_lo = 0.05;  f_hi = 3.0;             % [Hz]
            %     %%tau_amp = 0.02;                       % 励振トルク振幅 [N*m]（小さめ・線形域）
            %     %%probe = tau_amp * chirp(tvec, f_lo, te, f_hi, "logarithmic");
>>>>>>> Stashed changes
            %     % u=1e-1 * sin(2*pi*0.2*t_now);
            %     % u=5e-3 * (sin(2*pi*0.25*t_now)+t_now)+-5e-2;
            %     obj.result.chirp=u;
            %     us(1)=us(1)+u;
            % 
            % 
            %     %pitch,roll 別々
            %     % u_pitch=1e-1*chirp(t_now, 0, t_end*2, 5, [], -90 );
            %     % u_roll=1e-1*chirp(t_now, 0, t_end*2, 5);
            %     % obj.result.chirp_pitch=u_pitch;
            %     % obj.result.chirp_roll=u_roll;
            %     % us(1) = us(1)+u_pitch;
            %     % us(2) = us(2)+u_roll;
            % end
<<<<<<< Updated upstream

            tmp = [uf(1); us]; % 実入力へ変換

            % tmp = obj.notchFilter(tmp, Param.dt);   % ノッチフィルタ適用

=======
            % 
            tmp = [uf(1); us]; % 実入力へ変換
            % tmp(3)
            tmp = obj.notchFilter(tmp, Param.dt);   % ノッチフィルタ適用
            % tmp(3)
>>>>>>> Stashed changes
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

        function tmp_f = notchFilter(obj, tmp, dt)
            if isempty(obj.notchx)
                g = obj.self.parameter.get("gravity");
                L = obj.self.parameter.get("cableL");   % ケーブル長を直接取得
                % wn = 1.57; %中心周波数を設計
                % wn = sqrt(g/3.0); %中心周波数を設計
                wn=-1;
                % fprintf('L = %.4f, wn = %.4f rad/s\n', L, wn);
                zz = [0.001, 0.01]; %ノッチの深さを設定 [roll, pitch]
                zp = [0.5, 1]; %ノッチの幅を設定 [roll, pitch]

                % roll用フィルタ設計
                dx = c2d(tf([1 2*zz(1)*wn wn^2], [1 2*zp(1)*wn wn^2]), dt, 'tustin');%tfを使って伝達関数を作成
                %c2d(..., dt, 'tustin')連続系の伝達関数をdtをもとに離散系に変換
                [bx, ax] = tfdata(dx, 'v'); %伝達関数の係数を数値ベクトルで保管保管
                zx0 = zeros(max(length(ax), length(bx)) - 1, 1); %メモリの初期化
                obj.notchx = struct('b', bx, 'a', ax, 'z', zx0); %名前づけで楽に

                % pitch用フィルタ設計
                dy = c2d(tf([1 2*zz(2)*wn wn^2], [1 2*zp(2)*wn wn^2]), dt, 'tustin');
                [by, ay] = tfdata(dy, 'v');
                zy0 = zeros(max(length(ay), length(by)) - 1, 1);
                obj.notchy = struct('b', by, 'a', ay, 'z', zy0);
            end

            tmp_f = tmp;
            % [tmp_f(2), obj.notchx.z] = filter(obj.notchx.b, obj.notchx.a, tmp(2), obj.notchx.z);   % roll　 入力信号 x をフィルタ処理した結果が得られる
            %入力：フィルタの分子係数、フィルタの分母係数、フィルタにかける入力信号、前回のフィルタ処理後に保存しておいた遅延要素の値
            [tmp_f(3), obj.notchy.z] = filter(obj.notchy.b, obj.notchy.a, tmp(3), obj.notchy.z); % pitch
            % obj.notchy.z
        end
    end
end


