classdef TIME_VARYING_REFERENCE_SPLIT < handle
    % 時間関数としてのリファレンスを生成するクラス
    % obj = TIME_VARYING_REFERENCE()
    % NOTE: takeoff/landing are handled by dedicated reference classes.
    properties
        self                    % agent(i)を格納
        agent1                  % agent(1)を格納
        func                    % 時間関数のハンドル
        funcRotms               % rhoiを目標角度に合わせる回転行列
        com                     % 使用制御モデル->"Cooperative"or"Split"
        result                  % do method の返り値を格納
        N                       % 機体数
        base_time_flight = []   % 目標軌道に追従し始めたときの時刻
        constPrep = 0           % 前時刻の制約の位置
        constPrev = 0           % 前時刻の制約の速度

    end

    methods
        function obj = TIME_VARYING_REFERENCE_SPLIT(self, args, agent1)
            % make refernce of payload and split payload
            % payload       : args{3} = Cooperative
            % split payload : args{3} = split
            % phase         : handled by caller
            arguments
                self   
                args
                agent1
            end     
            obj.agent1              = agent1;               % agent(1)の格納
            obj.self                = self;                 % agent(i)の格納
            obj.N                   = args{4};              % 機体数Nの格納
            obj.com                 = args{3};              % 対象になるシステムの格納
            gen_func_name           = str2func(args{1});    % referenceの関数の格納
            param_for_gen_func      = args{2};              % referenceの関数のに代入する値の格納
            
            if length(args) > 2
                if strcmp(args{3}, "Cooperative")% 牽引物の目標軌道
                    [obj.func,obj.funcRotms]    = gen_ref_for_HL_Cooperative_Load(gen_func_name(param_for_gen_func{:}));% 目標値の関数を格納
                    obj.result.state            = STATE_CLASS(struct('state_list', ["xd", "p"], 'num_list', [27, 3]));% stateクラスを格納（保存したい変数を設定）
                    obj.result.state.set_state("xd",obj.func(0));                           % 目標値の初期値を設定
                    obj.result.state.set_state("p",obj.result.state.xd(1:3));               % 目標位置の初期値を設定（特に使わない）

                elseif strcmp(args{3}, "Split")% 分割後の牽引物の目標軌道
                    obj.result.state    = STATE_CLASS(struct('state_list', ["xd", "p", "minDroneDistance", "constp","constTargetp"], 'num_list', [28, 3, 1, 1, 1]));  % stateクラスを格納（保存したい変数を設定）
                    obj.result.state.set_state("xd",zeros(28,1));                           % 目標値の初期値を設定
                    obj.result.state.set_state("p",obj.result.state.xd(1:3));               % 目標位置の初期値を設定（特に使わない）
                    obj.result.state.set_state("minDroneDistance",0);                       % 機体間距離の最小値
                    obj.result.state.set_state("constp",0);                                 % 制約の初期値
                    obj.result.state.set_state("constTargetp",0);                           % 制約の目標値
                end
            else %上記以外のreference関数を複数機牽引用に修正
                temp.pYaw = gen_func_name(param_for_gen_func{:}); %位置をyaw角の目標値
                    % 回転の時間関数を設定
                    if ~isfield(temp,"q")
                        syms t
                        roll   = 0;
                        pitch  = 0;
                        yaw    = 0;
                        temp.q = [roll;pitch;yaw];%roll,pitch,yaw
                        clear t
                    end
                [obj.func,obj.funcRotms] = gen_ref_for_HL_Cooperative_Load(temp);% 目標値の関数を格納
                obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));% 目標値の初期値を設定
            end
        end

        function result = do(obj, varargin)
           dt       = varargin{1}.dt;   % 刻み時間

           if isempty(obj.base_time_flight)
                obj.base_time_flight  =  varargin{1}.t;
           end
           t = varargin{1}.t - obj.base_time_flight;

           %refernceの計算
           if strcmp(obj.com, "Split")
               %================================================
               % ~0は分割前牽引物，~iは分割後の牽引物を表す
               %================================================
               id           = obj.self.id - 1;
               rli          = sqrt(2) * obj.self.parameter.get("lx"); % 機体のロータまでの長さ
               rhoi         = obj.agent1.parameter.rho(:, id);        % 牽引物の中心位置からリンクまでの距離
               rhoiUnit12   = [rhoi(1:2) / norm(rhoi(1:2)); 0];        % rhoiをx-y平面に射影したベクトルの単位ベクトル

               epDronei     = obj.self.estimator.result.state.p;      % 機体位置
               spDrones     = obj.agent1.reference.timevarying.result.spDrones;       % 全ての機体位置
               ref0         = obj.agent1.reference.timevarying.result.state.xd(1:24); % 分割前の牽引物目標軌道
               rotms        = obj.agent1.reference.timevarying.result.rotms;          % rhoiを目標位置に向ける回転行列

               droneDistance        = vecnorm(spDrones - epDronei);    % 自身と相手との距離
               sortedDroneDistance  = sort(droneDistance);             % 小さい順に並べ替え
               minDroneDistance     = sortedDroneDistance(2) - 2 * rli;

               constTargetp         = 0.1 / (minDroneDistance - 0.4)^2;
               constp               = obj.constPrep + obj.constPrev * dt;
               obj.constPrep        = constp;
               kv                   = 0.05;
               obj.constPrev        = -kv * (constp - constTargetp);

               rhoi                 = rhoi + constp * rhoiUnit12;     % バリア関数で機体どうしの衝突を回避
               refi                 = ref0 + sum(rotms .* repmat(rhoi', 24, 1), 2);

               obj.result.state.xd                  = refi;
               obj.result.state.minDroneDistance    = minDroneDistance;
               obj.result.state.constTargetp        = constTargetp;
               obj.result.state.constp              = constp;
               obj.result.state.p                   = refi(1:3);

           else
               %牽引物の目標軌道
                   xd                       = obj.func(t);                          % 牽引物の目標軌道
                   [q,rotms]                = obj.funcRotms(t);                     % 回転行列と高次微分
               %全ての機体の位置を取得
                   % センサクラスがMOTIVEのとき(exp)
                   if isa( obj.self.sensor,"MOTIVE")
                       rigid                 = obj.agent1.sensor.result.rigid;      % 剛体情報を全て取得
                       spDrones              = zeros(3,(length(rigid)-1)/2);
                       for i = 1:(length(rigid)-1)/2
                            spDrones(:,i)    = rigid(2*i).p;                        % 機体の位置を格納
                       end
                   % sim
                   else
                       sensor1 = obj.self.sensor.result.state;                      % 複数機モデルから機体と接続点の位置を計測
                       %分割前牽引物
                           sp                 = sensor1.p;                          % 牽引物位置
                           sR                 = RodriguesQuaternion(sensor1.Q);     % 牽引物の回転行列
                       %分割後牽引物
                           rho                = obj.self.parameter.rho;
                           spDrones = zeros(size(rho));
                           for i = 1:size(rho,2)
                               spL            = sp + sR * rho(:,i);                 % 分割後の質量重心位置
                               spT            = sensor1.qi(3*i-2:3*i,1);            % 分割後の紐の方向ベクトル
                               spDrones(:,i)  = spL - obj.self.parameter.li(i)*spT; % 機体位置
                           end
                   end
               %log
                   obj.result.state.xd  = [xd;q];    %角度を最後に追加
                   obj.result.rotms     = rotms;     %目標値の回転列
                   obj.result.spDrones  = spDrones; %機体
                   obj.result.state.p   = xd(1:3);
           end
           result = obj.result;% 戻り値
        end

        function show(obj, logger)
            rp = logger.data(1,"p","r");
            plot3(rp(:,1), rp(:,2), rp(:,3));                     % xy平面の軌道を描く
            daspect([1 1 1]);
            hold on
            ep = logger.data(1,"p","e");
            plot3(ep(:,1), ep(:,2), ep(:,3));       % xy平面の軌道を描く
            legend(["reference", "estimate"]);
            title('reference and estimated trajectories');
            xlabel("x [m]");
            ylabel("y [m]");
            hold off
        end
        
    end
end
