classdef LANDING_REFERENCE < handle
    properties
        param
        self
        vd = 0.5;
        dt
        result
        base_state
        base_time = 0;
        te = 3 % 着陸するまでの時間
        th_offset
        th_offset0 = 200;
        initialz % 初期時刻高度（takeoffする前の高度）
        fInit = 0;
    end

    methods
        function obj = LANDING_REFERENCE(self,varargin)
            % generate landing reference w.r.t. position
            obj.self = self;
            obj.result.state = STATE_CLASS(struct('state_list',["xd","p","v"],'num_list',[20,3,3]));
            obj.dt = varargin{1};
            obj.vd = varargin{2};
        end
        function  result= do(obj,varargin)
            % [Input] time,cha,logger,env


            % if isempty(obj.result.state.xd)  % first take
            %   fInit = true;
            %   obj.initialz = obj.self.estimator.result.state.p(3);
            % else
            %   obj.result.state.xd = obj.gen_ref_for_landing(varargin{1}.t-obj.base_time);
            %   fInit = (obj.self.reference.result.state.p(3) - obj.result.state.xd(3)) > 0.5;
            % end
            if obj.fInit < 10 || isempty( obj.base_state ) % 飛行中にlanding modeに入った時点の情報を保存
              % 空回しの高度を保存
                    if obj.fInit ==0
                        obj.initialz = obj.self.estimator.result.state.p(3);
                    end
                    obj.base_time=varargin{1}.t;
                    obj.base_state = [obj.self.estimator.result.state.p(1:2);obj.self.reference.result.state.p(3)]; % x,y : current position, z : reference using at flight phase
                    obj.result.state.xd = [obj.base_state;zeros(17,1)];
                    if isprop(obj.self.input_transform,"param")
                        obj.th_offset = obj.self.input_transform.param.th_offset;
                    else
                        obj.th_offset = obj.th_offset0;
                    end
                    obj.fInit = obj.fInit + 1;
                    disp(obj.fInit)
            end

            obj.result.state.xd = obj.gen_ref_for_landing(varargin{1}.t-obj.base_time);
            obj.result.state.p = obj.result.state.xd(1:3,1);
            obj.result.state.v = obj.result.state.xd(5:7,1);
            if obj.fInit >= 2 % 地面効果対策で obj.te の時間で obj.th_offset -> obj.th_offset0 に変化させる。
                obj.self.input_transform.param.th_offset = obj.th_offset - (obj.th_offset-obj.th_offset0)*min(obj.te,varargin{1}.t-obj.base_time)/obj.te;
            end
            result = obj.result;
        end
        function Xd = gen_ref_for_landing(obj,t)
            %% Setting
            % calc reference position and its higher time derivatives
            % reference designed as a 9-degree polynomial function of time
            % [Inputs]
            % t : current time
            %
            % [Output]
            % Xd : reference [[p;yd], [p^(1);0], [p^(2);0], [p^(3);0], [p^(4);0]] as column vector
            %    : Xd in R^20
            %    : yd is a yaw angle reference

            %% Variable set
            Xd  = zeros( 20, 1);
            %% Set Xd
            if t<=obj.te
                Zd = curve_interpolation_9order(t,obj.te,obj.base_state(3),0,obj.initialz,0);
            elseif t> obj.te
                Zd = zeros(1,5);
                Zd(1) = obj.initialz;
            end
            Xd(1:3,1) = obj.base_state(1:3);
            Xd(3,1) = Zd(1);
            Xd(7,1) = Zd(2);
            Xd(11,1) = Zd(3);
            Xd(15,1) = Zd(4);
            Xd(19,1) = Zd(5);
        end
    end
end

