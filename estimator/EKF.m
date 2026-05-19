classdef EKF < handle
    % Extended Kalman filter
    % obj = EKF(model,param)
    %   model : EKFを実装する制御対象の制御モデル
    %   param : required field : Q,R,B,JacobianH
    %  JacobianH(x,p) : 出力方程式の拡張線形化した関数のhandle
    properties
        result
        % state : estimated state
        JacobianF
        JacobianH
        Q
        R
        dt
        B
        n
        sensor_name = "y";
        output_func % function of state
        output_param
        self
        model
        timer= [];
    end

    methods
        function obj = EKF(self,param)
            obj.self= self;
            obj.model = param.model;
            ELfile=strcat("Jacobian_",obj.model.name);
            if ~exist(ELfile,"file")
                obj.JacobianF=ExtendedLinearization(ELfile,obj.model);
            else
                obj.JacobianF=str2func(ELfile);
            end
            obj.result.state= state_copy(obj.model.state);
            obj.sensor_name = param.sensor_name;
            obj.output_func = param.output_func;
            obj.output_param = param.output_param;

            obj.JacobianH = param.JacobianH;
            obj.n = length(obj.model.state.get());
            obj.Q = param.Q;% 分散
            obj.R = param.R;% 分散
            obj.dt = obj.model.dt; % 刻み
            obj.B = param.B;
            obj.result.P = param.P;
            obj.result.G = zeros(obj.n,size(obj.R,2));
            if ~isfield(obj.self.estimator,"result")
                obj.self.estimator.result = obj.result;
            end
        end

        function [result]=do(obj,varargin)
            if ~isempty(obj.timer)
                dt = toc(obj.timer);
                if dt > obj.dt
                    dt = obj.dt;
                end
            else
                dt = obj.dt;
            end
            if varargin{1}.t ~= 0
                y = obj.self.sensor.result.(obj.sensor_name); % sensor output
                if isstruct(y) && isfield(y, "y")
                    y = y.y;
                end
                x = obj.result.state.get(); % estimated state at previous step
                obj.model.do(varargin{:}); % update state
                xh_pre = obj.model.state.get(); % Pre-estimation
                yh = obj.output_func(xh_pre,obj.output_param); % output estimation
                % disp("EKF")
                % [y,yh,y-yh] % for debug
                p = obj.self.parameter.get();
                A = eye(obj.n)+obj.JacobianF(x,p)*dt; % Euler approximation
                C = obj.JacobianH(x,p);
                P_pre  = A*obj.result.P*A' + obj.B*obj.Q*obj.B';       % Predicted covariance
                % if abs(det(C*P_pre*C'+obj.R)) > 1e-10
                G = (P_pre*C')/(C*P_pre*C'+obj.R); % Kalman gain
                % end
                P = (eye(obj.n)-G*C)*P_pre;	% Update covariance
                % (y-yh)' % for debug
                tmpvalue = xh_pre + G*(y-yh);	% Update state estimate
                tmpvalue = obj.model.projection(tmpvalue);
                obj.result.state.set_state(tmpvalue);
                obj.model.state.set_state(tmpvalue);
                obj.result.G = G;
                obj.result.P = P;
            end
            result=obj.result;
            obj.timer = tic;
        end
    end
end

