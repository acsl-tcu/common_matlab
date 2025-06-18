classdef HLC_SUSPENDED_LOAD < handle
    % クアッドコプター用階層型線形化を使った入力算出
    properties
        self
        result
        param
        P % physical parameter
        Q
        IT
        u_opt0
        fmc_options
        vdro_pre
        vL_pre
        aidrns
        ais
        ms
        estimate_load_mass
        flag_anti_spike=0
        elf
        isGround = 1
        % FOR_LOADの代わり
        pL0 % initial pL
        tt0 = [] % transition time for pL modification
        td = 5 % transition-duration
        ratet % transition rate
        mLL % mL at the beginning of landing phase
        cableLL % cable length at the beginning of landing phase
        baseP % base position with respect to load position (taken by arming position)
    end

    methods
        function obj = HLC_SUSPENDED_LOAD(self,param)
            obj.self = self;
            obj.param = param;
            obj.Q = STATE_CLASS(struct('state_list',"q",'num_list',4));
            % obj.u_opt0 = [(self.parameter.mass + self.parameter.loadmass*0)*self.parameter.gravity;0;0;0];
            % obj.fmc_options = optimoptions(@fmincon,'Display','off');
            % obj.vdro_pre = 0;
            % obj.vL_pre = 0;
            % obj.aidrns=zeros(3,20);
            % obj.ais =zeros(3,20);
            % obj.ms = ones(1,10)*0.4;
            % obj.estimate_load_mass = ESTIMATE_LOAD_MASS(self);
            obj.ratet = 1/obj.td^2; % 二次関数で0-1の間で変化する
            %物理パラメータ
            obj.P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]),0,0];
        end

        function result=do(obj,varargin)
            t = varargin{1}.t;
            Param  = obj.param;  % param (optional) : 構造体：物理パラメータP，ゲインF1-F4
            model  = obj.self.estimator.result;  % 推定した状態
            ref = obj.self.reference.result;  % 目標値
            cha = varargin{2};
            if isempty(obj.baseP)
                obj.baseP = model.state.p - model.state.pL;
            end

            % 目標値を取得
            if isprop(ref.state,'xd')
                xd = ref.state.xd; % 20次元の目標値に対応する用
            else
                xd = ref.state.get();
            end

            [pL,pT,P,xd]= obj.calc_pL(t,model,cha,xd);

            x = [model.state.getq('compact');model.state.w;pL;model.state.vL;pT;model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え

            % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
            % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
            yaw = wrapToPi(model.state.q(3));  % 機体yaw角[-pi,pi]にする特にyaw
            yawd  = xd(4);  % 目標yaw角
            yawUnit = [cos(yaw);sin(yaw);0];  % yawの方向ベクトル
            yawdUnit  = [cos(yawd);sin(yawd);0]; % yawdの方向ベクトル
            deltaYaw  = sign(cross(yawdUnit,yawUnit))*acos(yawdUnit'*yawUnit);% 目標角度からみた機体角度との誤差
            xd(4)  = -deltaYaw(3) + yaw;  % yaw打ち消しと誤差をyawの目標角に入れる．
            %目標値の格納
            xd =[xd;zeros(28-size(xd,1),1)];  % 足りない分は0で埋める．

            % 階層型線形化による入力計算
            % 仮想入力のゲイン
            F1 = Param.F1; % z方向サブシステムのゲイン
            F2 = Param.F2; % x方向サブシステムのゲイン
            F3 = Param.F3; % y方向サブシステムのゲイン
            F4 = Param.F4; % yaw方向サブシステムのゲイン

            vf = obj.Vfd_SuspendedLoadxyDst(Param.dt,x,xd',F1); % 実験で刻み時間が変わったときに対応
            vs = obj.Vs_SuspendedLoadxyDst(x,xd',vf,P,F2,F3,F4);  % 第二層x,y,yawサブシステムの仮想入力の計算
            uf = obj.Uf_SuspendedLoadxyDst(x,xd',vf,P); % 第一層の仮想入力の実入力(推力)への変換
            % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
            %tic
            beta2  = obj.Beta2_SuspendedLoadxyDst(x,xd',vf,P); % 第二層のbetaの逆行列
            vs_alpha2  = obj.V2_alpha2_SuspendedLoadxyDst(x,xd',vf,vs',P); % 第二層のvs - alpha
            us = beta2\vs_alpha2;  % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha)%h234*invbeta2*a2;
            %obj.result.aa=toc;
            tmp = [uf(1);us]; % 実入力へ変換
            obj.result.tmp = tmp; % 入力に制限を付けてない値を格納

            % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
            %disp("time: "+ num2str(t,2)+" z position of drone: "+num2str(model.state.p(3),3)+" estimated load mass: "+num2str(P(6),4)+" dst:(x,y) "+num2str(P(end-1:end),4))
            obj.result.input = [max(0,min(20,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];%+[normrnd(0,0.01,1);normrnd(0,0.001,[3,1])]*1;%入力にノイズを付与可能
            obj.result.xd = xd;
            obj.result.x = x;
            result = obj.result;
        end
        function show(obj)
            obj.result
        end


        function [pL,pT,P,xd] = calc_pL(obj,t,model,cha,xd)
            P = obj.P;
            p = model.state.p;
            pL = model.state.pL;
            L = obj.self.parameter.get("cableL");
            if isprop(model.state,"mL")
                mL = max(0,model.state.mL); % load mass
            else
                mL = P(6);
            end
            nxy = xd(1:3); %
            if isempty(obj.pL0)
                obj.pL0 = pL; % 初期の牽引物位置
            end
            if strcmp(cha,'t') % take off
                % % 質量推定が進んだら or
                % 牽引物の初期高さ+機体の全高より高くなったら
                % センサ値を使い始める
                if mL > 0.1 || p(3)> obj.pL0(3) + L*0.9
                    if isempty(obj.tt0)
                        obj.isGround = 0;
                        obj.tt0 = t;
                    end
                    tt = min((t - obj.tt0),obj.td);  %takeoffの経過時間がセンサ値使用率100%になる時間を越えないようにする
                    k = obj.ratet*tt^2; %センサ値反映割合
                    pL  = p + k*(pL - p); %牽引物位置と機体位置の差に反映割合をかけてセンサ値を反映
                    % z 方向は最後に更新する
                else %閾値を越えなかったら機体の真下に牽引物がいることにする
                    pL = p;
                end
                nxy = pL;
            elseif strcmp(cha,'l') % landing
                if isempty(obj.cableLL) || isempty(obj.mLL)
                    obj.cableLL = norm(p - pL);  % landing開始時の機体と牽引物の距離
                    obj.mLL  = mL;  % landing開始時の質量
                    obj.tt0 = t;
                end
                %地面についたかの判定
                if p(3) - pL(3) < obj.cableLL*0.9 || obj.isGround == 1
                    obj.isGround  = 1;  % この分岐に一回でも入ったら入り続けるようにフラグ立てる
                    tt  = min(t - obj.tt0, obj.td);  % landingの経過時間がセンサ値使用率0%になる時間を越えないようにする
                    k  = obj.ratet*tt^2;  % センサ値反映割合
                    pL  = p + (1 - k)*(pL - p); % 牽引物位置と機体位置の差に反映割合をかけてセンサ値を反映
                    nxy = nxy + k*obj.baseP;
                    obj.mLL = (1 - k)*obj.mLL;
                    mL = min(mL, obj.mLL);  % 傾いて着陸した時に推定が吹っ飛ばないように制限
                end

            end
            %l = sqrt(L^2 - sum((p(1:2)-pL(1:2)).^2));
            P(6) = mL;
            if strcmp(cha,'f') %
                P = obj.P;
                pL = model.state.pL;
                pT = model.state.pT;
            else
                pL(3) = p(3) - L;
                ttt = pL - p;
                pT = ttt/norm(ttt);
                if isprop(model.state,"dst") && strcmp(cha,'f')
                    P(end-1:end)  = model.state.dst';
                end               
            end
            nxy(3) = xd(3) - L;
            xd(1:3) = nxy;
        end
    end
end