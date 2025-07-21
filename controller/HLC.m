classdef HLC < handle
  % Hierarchical linearization based controller for a quadcopter
  properties
    self
    result
    param
    H=12;
    parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
  end

  methods
    function obj = HLC(self,param)
      obj.self = self;
      obj.param = param;
      obj.param.P = self.parameter.get(obj.parameter_name);
      obj.result.input = zeros(self.estimator.model.dim(2),1);
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
      disp('controller: HLC,  phase: ');
      disp(phase);
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
      % max,min are applied for the safty
      obj.result.input = [max(0,min(10,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
      obj.result.hlc = obj.result.input;
      result = obj.result;
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
    function [xr] = generate_reference(obj)
            total_size=16;
            xr = zeros(total_size, obj.H);    % initialize
            % 時間関数の取得→時間を代入してリファレンス生成

            RefTime = obj.self.reference.bezier.ref_generator;    % 時間関数の取得
            for h = 0:obj.H-1
                t = obj.param.t + obj.param.dt * h; % reference生成の時刻をずらす
                ref = RefTime(t);
                xr(1:3, h+1) = ref(1:3);
                xr(7:9, h+1) = ref(5:7);
                xr(4:6, h+1) =   [0;0;ref(4)]; % 姿勢角
                xr(10:12, h+1) = [0;0;0];
                xr(13:16, h+1) = [0;0;0;0]; % MC -> 0.6597,   HL -> 0
            end
    end
   
            % clc;
            % est_print = obj.self.estimator.result.state;
           
        
  end
end

