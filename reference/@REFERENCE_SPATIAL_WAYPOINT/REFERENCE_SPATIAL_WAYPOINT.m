classdef REFERENCE_SPATIAL_WAYPOINT < handle
    % REFERENCE_SPATIAL_WAYPOINT
    % 時間(t)に縛られず、目標位置(p_target)へ向かう空間ベースの軌道生成器。
    % 【重要】懸垂荷物用コントローラが要求する「6階微分までなめらか」な要件を満たすため、
    % 6次ローパスフィルター(Canonical Linear Filter)を内部に持ち、
    % p, v, a, j, s, c をC5連続で出力します。

    properties
        self
        result
        p_target
        
        % フィルタの内部状態 (3次元 x 6階 = 18次元)
        x_p, x_v, x_a, x_j, x_s, x_c
        
        % フィルタゲイン (極配置)
        k0, k1, k2, k3, k4, k5
        
        % 初期化フラグ
        is_initialized = false;
    end

    methods
        function obj = REFERENCE_SPATIAL_WAYPOINT(self, varargin)
            obj.self = self;
            obj.p_target = [10; 0; 3]; 
            
            for i = 1:2:length(varargin)
                if strcmp(varargin{i}, 'pd')
                    obj.p_target = varargin{i+1};
                end
            end
            
            obj.x_p = [0; 0; 0];
            obj.x_v = [0; 0; 0];
            obj.x_a = [0; 0; 0];
            obj.x_j = [0; 0; 0];
            obj.x_s = [0; 0; 0];
            obj.x_c = [0; 0; 0];
            
            w = 1.5; 
            obj.k0 = w^6;
            obj.k1 = 6 * w^5;
            obj.k2 = 15 * w^4;
            obj.k3 = 20 * w^3;
            obj.k4 = 15 * w^2;
            obj.k5 = 6 * w;
            
            obj.result.state = STATE_CLASS(struct('state_list', ["p", "v", "a", "j", "s", "c", "xd"], 'num_list', [3, 3, 3, 3, 3, 3, 18]));
        end

        function result = do(obj, varargin)
            time = varargin{1};
            dt = time.dt;
            if isempty(dt) || dt <= 0, dt = 0.025; end
            
            % 初回呼び出し時に現在位置でフィルタを初期化
            if ~obj.is_initialized
                obj.x_p = obj.self.estimator.result.state.p;
                obj.is_initialized = true;
            end
            
            dx_c = -obj.k0 * (obj.x_p - obj.p_target) ...
                   -obj.k1 * obj.x_v ...
                   -obj.k2 * obj.x_a ...
                   -obj.k3 * obj.x_j ...
                   -obj.k4 * obj.x_s ...
                   -obj.k5 * obj.x_c;
                   
            obj.x_c = obj.x_c + dx_c * dt;
            obj.x_s = obj.x_s + obj.x_c * dt;
            obj.x_j = obj.x_j + obj.x_s * dt;
            obj.x_a = obj.x_a + obj.x_j * dt;
            obj.x_v = obj.x_v + obj.x_a * dt;
            obj.x_p = obj.x_p + obj.x_v * dt;
            
            obj.result.state.p = obj.x_p;
            obj.result.state.v = obj.x_v;
            obj.result.state.a = obj.x_a;
            obj.result.state.j = obj.x_j;
            obj.result.state.s = obj.x_s;
            obj.result.state.c = obj.x_c;
            
            obj.result.state.xd = [obj.x_p; obj.x_v; obj.x_a; obj.x_j; obj.x_s; obj.x_c];
            
            result = obj.result;
        end
    end
end
