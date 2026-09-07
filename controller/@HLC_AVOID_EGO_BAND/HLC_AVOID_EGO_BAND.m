classdef HLC_AVOID_EGO_BAND < HLC_SUSPENDED_LOAD
    properties
        p_off; v_off; a_off; j_off; s_off; d5_off;
        Q_pts; % 制御点(3x3行列)
    end
    methods
        function obj = HLC_AVOID_EGO_BAND(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off=zeros(3,1); obj.v_off=zeros(3,1); obj.a_off=zeros(3,1);
            obj.j_off=zeros(3,1); obj.s_off=zeros(3,1); obj.d5_off=zeros(3,1);
            obj.Q_pts = [];
        end
        function result = do(obj, time, varargin)
            agent_obj = varargin{4}; idx = varargin{5};
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            p_mid = (agent_obj(idx).estimator.result.state.p + agent_obj(idx).estimator.result.state.pL)/2.0;
            V0 = xd_nom(5:7);
            
            % 制御点の初期化と進行
            dt_lookahead = 1.0;
            if isempty(obj.Q_pts)
                obj.Q_pts = [p_mid + V0*dt_lookahead, p_mid + V0*dt_lookahead*2, p_mid + V0*dt_lookahead*3];
            else
                % 目標軌道に合わせて制御点をシフト
                obj.Q_pts = [p_mid + V0*dt_lookahead, p_mid + V0*dt_lookahead*2, p_mid + V0*dt_lookahead*3];
            end
            
            % Ego-Planner方式: コスト関数の勾配降下法(Elastic Band)
            obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
            alpha = 0.05; w_smooth = 1.0; w_obs = 10.0; safe_dist = 2.5;
            for iter=1:20
                for i=1:3
                    % 滑らかさの勾配 (ゴムひものように真っ直ぐになろうとする力)
                    if i==1; q_prev = p_mid; else; q_prev = obj.Q_pts(:,i-1); end
                    if i==3; q_next = p_mid + V0*dt_lookahead*4; else; q_next = obj.Q_pts(:,i+1); end
                    grad_smooth = 2*obj.Q_pts(:,i) - q_prev - q_next;
                    
                    % 衝突回避の勾配 (障害物から押し出される力)
                    grad_obs = zeros(3,1);
                    for k=1:length(obs_list)
                        diff = obj.Q_pts(:,i) - obs_list(k).p_obs;
                        d = norm(diff);
                        if d < safe_dist
                            grad_obs = grad_obs - 2*(safe_dist - d) * (diff/d);
                        end
                    end
                    % 制御点の更新
                    obj.Q_pts(:,i) = obj.Q_pts(:,i) - alpha * (w_smooth*grad_smooth + w_obs*grad_obs);
                end
            end
            
            % 最初の制御点へ向かう仮想速度
            V_ref = (obj.Q_pts(:,1) - p_mid) / dt_lookahead;
            F_virt = 2.0 * (V_ref - V0);
            
            dt = 0.025; if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
            lambda = 4.0; c0=lambda^6; c1=6*lambda^5; c2=15*lambda^4; c3=20*lambda^3; c4=15*lambda^2; c5=6*lambda;
            obj.d5_off = obj.d5_off + (c0 * F_virt - (c5*obj.d5_off + c4*obj.s_off + c3*obj.j_off + c2*obj.a_off + c1*obj.v_off + c0*obj.p_off)) * dt;
            obj.s_off = obj.s_off + obj.d5_off * dt; obj.j_off = obj.j_off + obj.s_off * dt;
            obj.a_off = obj.a_off + obj.j_off * dt;  obj.v_off = obj.v_off + obj.a_off * dt;
            obj.p_off = obj.p_off + obj.v_off * dt;
            
            xd_mod = xd_nom;
            xd_mod(1:3) = xd_nom(1:3) + obj.p_off; xd_mod(5:7) = xd_nom(5:7) + obj.v_off; xd_mod(9:11) = xd_nom(9:11) + obj.a_off;
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
        end
    end
end
