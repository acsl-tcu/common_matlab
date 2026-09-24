classdef HLC < handle
  % Hierarchical linearization based controller for a quadcopter
  properties
    self
    result
    param
    H=12;
    mec          % EDMD-MEC 
    du_prev = zeros(4,1);
    use_mec = 1;   %  0=使わない　１＝使う
    parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    excite_counter = 0;
  end

  methods
    function obj = HLC(self,param)
      obj.self = self;
      obj.param = param;
      obj.param.P = self.parameter.get(obj.parameter_name);
      obj.result.input = zeros(self.estimator.model.dim(2),1);
      if obj.use_mec
         % L = load('mec_model_20260916_HL_only_and_reiki.mat'); obj.mec = L.model;%EDMD-MEC model
         L = load('mec_model_HL20260916_usepayload.mat'); obj.mec = L.model;%EDMD_include

      end
    end

    function result = do(obj,varargin)
        
        % time = varargin{1};
         phase = varargin{2};
        % obj.param.t = time.t;
        % if phase == 'f'
        %     ref = obj.generate_reference();
        % else
             
        % end
      model = obj.self.estimator.result;
      ref = obj.self.reference.result;
      xd = ref.state.xd;
      if isfield(obj.param, 'enable_show') && obj.param.enable_show
          fprintf('controller: HLC,  phase: %s \n', phase);
          disp(ref.state.p);
      end
      xd0 =xd;
      P = obj.param.P;
      F1 = obj.param.F1;
      F2 = obj.param.F2;
      F3 = obj.param.F3;
      F4 = obj.param.F4;
      xd=[xd;zeros(20-size(xd,1),1)];% 足りない分は０で埋める．

      % yaw 角についてボディ座標に合わせることで目標姿勢と現在姿勢の間の2pi問題を緩和
      % TODO : 本質的にはx-xdを受け付ける関数にして，x-xdの状態で2pi問題を解決すれば良い．
      Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
      x = [R2q(Rb0'*model.state.getq("rotmat"));Rb0'*model.state.p;Rb0'*model.state.v;model.state.w]; % [q, p, v, w]に並べ替え
      xd(1:3)=Rb0'*xd(1:3);
      xd(4) = 0;
      xd(5:7)=Rb0'*xd(5:7);
      xd(9:11)=Rb0'*xd(9:11);
      xd(13:15)=Rb0'*xd(13:15);
      xd(17:19)=Rb0'*xd(17:19);
      %if isfield(obj.param,'dt')
      if isfield(varargin{1},'dt') && varargin{1}.dt <= obj.param.dt
        dt = varargin{1}.dt;
         vf = Vfd(dt,x,xd',P,F1);
        vs = Vsd(dt,x,xd',vf,P,F2,F3,F4);
      else
        vf = Vf(x,xd',P,F1);
        vs = Vs(x,xd',vf,P,F2,F3,F4);
      end
      %disp([xd(1:3)',x(5:7)',xd(1:3)'-xd0(1:3)']);
      tmp = Uf(x,xd',vf,P) + Us(x,xd',vf,vs',P);
     
if obj.use_mec
    est  = obj.self.estimator.result.state;     % 
    refs = obj.self.reference.result.state;      % 

    g = 9.81;
    Rk = est.getq("rotmat"); Re3_k = Rk(:,3);
    zk = build_phi(est.p(:), est.q(:), est.v(:), est.w(:), Re3_k);

    % 
    pref = refs.p(:);
    vref = refs.v(:);
    xd   = refs.xd(:);
    aref = xd(9:11);                 % 加速度 ref(9:11)

    % --- ---
    yawr = atan2(vref(2), vref(1));  % 参考yaw=速度方向(你的figure8里ref(4)=0,但用速度方向更稳)
    acmd = aref + [0;0;g];
    if norm(acmd)<1e-6, zb=[0;0;1]; else, zb=acmd/norm(acmd); end   % 名义机体z轴=Re3
    % 由 zb 和 yaw 反算 roll/pitch
    xc = [cos(yawr); sin(yawr); 0];
    yb = cross(zb, xc); yb = yb/max(norm(yb),1e-6);
    xb = cross(yb, zb);
    Rr = [xb, yb, zb];               % 名义旋转矩阵
    Re3r = zb;
    eulr = obj.R2eul_local(Rr);          % [roll;pitch;yaw]

    zref = build_phi(pref, eulr, vref, est.w(:), Re3r);  % w仍用实测占位

    rk     = zref - obj.mec.Ar*zk - obj.mec.Br*tmp;
    du_raw = obj.mec.M * rk;
    du     = (1-obj.mec.beta)*obj.du_prev + obj.mec.beta * ...
             max(min(du_raw, obj.mec.dumax), -obj.mec.dumax);
    obj.du_prev = du;
    tmp = tmp + du;
end
% =============================
      % max,min are applied for the safty
      
      
      obj.result.u_nom = [max(0,min(10,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
      obj.result.deltau = obj.generate_identification_excitation(obj.result.u_nom, phase, varargin{1});
      obj.result.deltau = 0;
      obj.result.input=obj.result.u_nom+ obj.result.deltau;
      obj.result.input = [max(0,min(10,obj.result.input(1))); ...
                          max(-1,min(1,obj.result.input(2))); ...
                          max(-1,min(1,obj.result.input(3))); ...
                          max(-1,min(1,obj.result.input(4)))];
      obj.result.hlc = obj.result.input;
      obj.result.pre_u = obj.result.input;
      obj.result.u_raw = obj.result.input;
      obj.result.hlc_x = x;
      obj.result.hlc_xd = xd;
      obj.result.u_explore = obj.result.deltau;
      obj.result.explore_active = obj.get_explore_active_flag(phase);
      obj.result.explore_mode = obj.get_explore_config().mode;
      result = obj.result;
      % if isfield(obj.param, 'enable_show') && obj.param.enable_show
          obj.show();
      % end
      % est_print = obj.self.estimator.result.state;
      % fprintf("==================================================================\n")
      % fprintf("==================================================================\n")
      % fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
      %     est_print.p(1), est_print.p(2), est_print.p(3),...
      %     est_print.v(1), est_print.v(2), est_print.v(3),...
      %     est_print.q(1), est_print.q(2), est_print.q(3)); % s:state 現在状態
      % fprintf("pr: %f %f %f \t vr: %f %f %f \t qr: %f %f %f \n", ...
      %     ref.state.p(1), ref.state.p(2), ref.state.p(3),...
      %     ref.state.v(1), ref.state.v(2),ref.state.v(3),...
          % ref.state.q(1),ref.state.q(2),ref.state.q(3));          % r:reference 目標状態
      % fprintf("t: %f \t input: %f %f %f %f \t J: %f \t sigma: %f", ...
      %   obj.param.t, obj.result.input(1), obj.result.input(2), obj.result.input(3), obj.result.input(4), obj.result.bestcost(1),obj.input.sigma(1));
      % fprintf("\n");
              
    end

      function du = generate_identification_excitation(obj, u_nom, phase, time_info)
    % システム同定用の励振入力 du を生成する関数
    % du(1): 推力入力に対する励振
    % du(2): roll方向入力に対する励振
    % du(3): pitch方向入力に対する励振
    % du(4): yaw方向入力に対する励振

    cfg = obj.get_explore_config();
    du = zeros(4, 1);

    if ~cfg.enable
        return;
    end
    % 励振機能が無効の場合は励振入力を加えない

    if cfg.flight_only && ~obj.get_explore_active_flag(phase)
        return;
    end
    % flight_only が有効な場合，飛行フェーズ以外では励振を行わない

    t = obj.get_time_value(time_info);
    tau = max(0, t - cfg.start_time);
    ramp = min(1.0, tau / max(cfg.ramp_time, 1e-6));
    % start_time 以降に励振を開始し，ramp_time の間で徐々に振幅を大きくする

    switch lower(char(string(cfg.mode)))
        case 'legacy_relative_random'
            du = cfg.legacy_scale .* abs(u_nom) .* (2 * rand(4, 1) - 1);
            % 旧方式のランダム励振
            % 公称入力 u_nom の大きさに比例したランダム入力を各チャンネルに加える
            % legacy_scale を大きくすると励振が強くなり，小さくすると弱くなる

        otherwise
            du(1) = sum(cfg.u1_amp(:) .* sin(2 * pi * cfg.u1_freq(:) * tau + cfg.u1_phase(:)));
            % 推力チャンネルのマルチサイン励振
            % u1_amp で推力励振の大きさ，u1_freq で周波数，u1_phase で位相を調整する

            du(2) = sum(cfg.u2_amp(:) .* sin(2 * pi * cfg.u2_freq(:) * tau + cfg.u2_phase(:)));
            % rollチャンネルのマルチサイン励振
            % u2_amp を大きくすると roll 方向の同定信号が強くなる

            du(3) = sum(cfg.u3_amp(:) .* sin(2 * pi * cfg.u3_freq(:) * tau + cfg.u3_phase(:)));
            % pitchチャンネルのマルチサイン励振
            % u3_amp を大きくすると pitch 方向の応答をより強く励起できる

            du(4) = sum(cfg.u4_amp(:) .* sin(2 * pi * cfg.u4_freq(:) * tau + cfg.u4_phase(:)));
            % yawチャンネルのマルチサイン励振
            % yaw方向は影響が大きくなりやすいため，比較的小さい振幅に設定する

            du = ramp * du;
            % 励振開始直後の急激な入力変化を避けるため，ランプ係数を掛ける
    end

    du = max(min(du, cfg.du_cap(:)), -cfg.du_cap(:));
    % 安全のため，各チャンネルの励振入力を du_cap の範囲内に制限する
end


function cfg = get_explore_config(obj)
    % 励振信号の設計パラメータを設定する関数
    % 振幅 amp，周波数 freq，位相 phase，入力上限 du_cap をここで調整する

    cfg = struct();

    cfg.enable = 1;
    % 1: 励振を有効化，0: 励振を無効化

    cfg.flight_only = 1;
    % 1: 飛行中のみ励振を加える，0: すべてのフェーズで励振を加える

    cfg.mode = 'structured_multisine';
    % structured_multisine: 複数周波数の正弦波を用いた設計励振
    % legacy_relative_random: 公称入力に比例したランダム励振

    cfg.start_time = 0.0;
    % 励振を開始する時刻

    cfg.ramp_time = 1.0;
    % 励振振幅を徐々に立ち上げる時間
    % 大きくすると入力変化が滑らかになり，小さくすると早く最大振幅に達する

    cfg.legacy_scale = 0.2 * ones(4, 1);
    % ランダム励振モードで使用する入力倍率

    cfg.u1_amp = [0.35; 0.15];
    cfg.u1_freq = [0.35; 0.90];
    cfg.u1_phase = [0.20; 1.10];
    % u1: 推力入力の励振設計
    % amp を大きくすると上下方向の応答が強く出る
    % freq を複数設定することで，異なる周波数帯の動特性を同定しやすくする

    cfg.u2_amp = [0.010; 0.005];
    cfg.u2_freq = [0.55; 1.25];
    cfg.u2_phase = [0.40; 1.30];
    % u2: roll方向入力の励振設計
    % roll方向の姿勢応答を同定するための小振幅マルチサイン信号

    cfg.u3_amp = [0.010; 0.005];
    cfg.u3_freq = [0.70; 1.55];
    cfg.u3_phase = [0.90; 0.30];
    % u3: pitch方向入力の励振設計
    % pitch方向の姿勢応答を同定するための小振幅マルチサイン信号

    cfg.u4_amp = [0.0010; 0.0005];
    cfg.u4_freq = [0.45; 1.05];
    cfg.u4_phase = [0.50; 1.40];
    % u4: yaw方向入力の励振設計
    % yaw方向は安定性への影響を考慮し，他の姿勢入力より小さい振幅にする

    cfg.du_cap = [0.55; 0.018; 0.018; 0.003];
    % 各励振入力の最大値を設定する
    % du_cap を大きくすると同定信号は強くなるが，機体の挙動が乱れやすくなる
    % du_cap を小さくすると安全性は高くなるが，同定精度が低下する可能性がある

    if isfield(obj.param, 'explore')
        fns = fieldnames(obj.param.explore);
        for i = 1:numel(fns)
            cfg.(fns{i}) = obj.param.explore.(fns{i});
        end
    end
    % obj.param.explore に設定がある場合，デフォルト値を上書きする
    % 実験条件に応じて amp, freq, phase, du_cap などを外部から変更できる
end

      function t = get_time_value(obj, time_info)
            if nargin >= 2 && isstruct(time_info) && isfield(time_info, 't')
                t = double(time_info.t);
                return;
            end

            obj.excite_counter = obj.excite_counter + 1;
            if isfield(obj.param, 'dt') && ~isempty(obj.param.dt)
                t = (obj.excite_counter - 1) * obj.param.dt;
            else
                t = obj.excite_counter - 1;
            end
      end

      function active = get_explore_active_flag(~, phase)
            if isstring(phase) || ischar(phase)
                phase_str = lower(char(phase));
                active = ~isempty(phase_str) && phase_str(1) == 'f';
            else
                active = true;
            end
      end
    
   
            % clc;
            % est_print = obj.self.estimator.result.state;
            function e = R2eul_local(~, R)       % ZYX -> [roll;pitch;yaw]
                e = [ atan2(R(3,2),R(3,3)); asin(max(min(-R(3,1),1),-1)); atan2(R(2,1),R(1,1)) ];
            end
      function show(obj)
            % clc;
            % est_print = obj.self.estimator.result.state;
            est_print = obj.self.estimator.result.state;
            ref_print =obj.self.reference.result.state;
            fprintf("==================================================================\n")
            fprintf("==================================================================\n")
            fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
                est_print.p(1), est_print.p(2), est_print.p(3),...
                est_print.v(1), est_print.v(2), est_print.v(3),...
                est_print.q(1), est_print.q(2), est_print.q(3)); % s:state 現在状態
            fprintf("pr: %f %f %f \t vr: %f %f %f \t qr: %f %f %f \n", ...
             ref_print.p(1), ref_print.p(2), ref_print.p(3),...
                ref_print.v(1), ref_print.v(2), ref_print.v(3),...
                ref_print.xd(4), ref_print.xd(5), ref_print.xd(6)); % r:reference 目標状態
            
        end  
  end
end

