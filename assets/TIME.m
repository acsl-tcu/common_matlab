classdef TIME < handle
% 現在時刻:t，刻み時間:dtの管理と計算時間:calc_timeの保存を行うクラス
    % [Input arguments]
    % ts: Start time
    % dt: Sampling time
    % te: Terminal time
    % N: The number of the agents.
properties
    t
    ts
    te
    dt
    set_dt % 制御周期として設定した刻み時間
    k = 1;
    calc_time
    target4loop = ["total", "drawnow", "motive_getData", "do_calculation"];
    target4do   = ["sensor",    "estimator",        "reference",...
                   "controller","input_transform",  "plant",...
                   "logging"];
end
methods
  function obj = TIME(ts,dt,te,N)
      obj.t = ts;
      obj.ts = ts;
      obj.dt = dt;
      obj.set_dt = dt;
      obj.te = te;

      len = size(ts:dt:te, 2);
      for tag = obj.target4loop
          obj.calc_time.(tag) = zeros(len,1);
      end

      for tag = obj.target4do
          if tag == "logging"
              obj.calc_time.(tag) = zeros(len,1);
          else
              obj.calc_time.(tag) = zeros(len,N); % 各agent用に(len x N)行列
          end
      end
  end


  function store_calc_time4loop(obj, dt)
      % dt(配列の要素順)はtarget4loopの順番に合わせる
      for i = 1:length(obj.target4loop)
          obj.calc_time.(obj.target4loop(i))(obj.k) = dt(i);
      end
  end


  function store_calc_time4do_calculation(obj, do_dt, log_dt)
      % do_dt(配列の要素順)はtarget4doの順番に合わせる
      for i = 1:length(obj.target4loop)
          tag = obj.target4do(i);
          if tag == "logging"
              obj.calc_time.(tag)(obj.k) = log_dt(i);
          else
              obj.calc_time.(tag)(obj.k,:) = do_dt(i,:);
          end
      end
  end
end

end
