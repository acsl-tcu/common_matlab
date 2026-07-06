classdef HLC < handle
  % Hierarchical linearization based controller for a quadcopter
  properties
    self
    result
    param
    H=12;
    mec          % EDMD-MEC 模型
    du_prev = zeros(4,1);
    use_mec = 0;   % 想关掉补偿就设 false
    parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
  end

  methods
    function obj = HLC(self,param)
      obj.self = self;
      obj.param = param;
      obj.param.P = self.parameter.get(obj.parameter_name);
      obj.result.input = zeros(self.estimator.model.dim(2),1);
      L = load('mec_model_HL.mat'); obj.mec = L.model;
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
      fprintf('controller: HLC,  phase: %s \n',phase);
      
      disp(ref.state.p);
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
      % ===== EDMD-MEC 残差补偿 =====
      % if obj.use_mec
      %     est  = obj.self.estimator.result.state;        % 实测(有 q,v,w,R)
      %     refs = obj.self.reference.result.state;          % 参考(有 p,v)
      % 
      %     Rk    = est.getq("rotmat");  Re3_k = Rk(:,3);
      %     zk    = build_phi(est.p(:), est.q(:), est.v(:), est.w(:), Re3_k);
      % 
      %     % 参考侧：位置/速度用真值；roll,pitch 用实测占位(相减抵消)；yaw 由参考速度算
      %     pref = refs.p(:);
      %     vref = refs.v(:);
      %     qref = est.q(:);
      %     qref(3) = atan2(vref(2), vref(1));               % 参考yaw=速度方向
      %     zref = build_phi(pref, qref, vref, est.w(:), Re3_k);
      % 
      %     rk     = zref - obj.mec.Ar*zk - obj.mec.Br*tmp;
      %     du_raw = obj.mec.M * rk;
      %     du     = (1-obj.mec.beta)*obj.du_prev + obj.mec.beta * ...
      %         max(min(du_raw, obj.mec.dumax), -obj.mec.dumax);
      %     obj.du_prev = du;
      %     tmp = tmp + du;
      % end
      % =============================
      % ===== EDMD-MEC 残差补偿 =====
if obj.use_mec
    est  = obj.self.estimator.result.state;     % 实测
    refs = obj.self.reference.result.state;      % 参考(p,v,xd含acc)

    g = 9.81;
    Rk = est.getq("rotmat"); Re3_k = Rk(:,3);
    zk = build_phi(est.p(:), est.q(:), est.v(:), est.w(:), Re3_k);

    % --- 参考位置/速度 ---
    pref = refs.p(:);
    vref = refs.v(:);
    xd   = refs.xd(:);
    aref = xd(9:11);                 % 参考加速度 ref(9:11)

    % --- 微分平坦反算名义姿态 ---
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
      obj.result.input = [max(0,min(10,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
      obj.result.hlc = obj.result.input;
      obj.result.pre_u = obj.result.input;
      result = obj.result;
      obj.show();
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

