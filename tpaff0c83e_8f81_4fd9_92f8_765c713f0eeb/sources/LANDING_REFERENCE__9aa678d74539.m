classdef LANDING_REFERENCE < handle
    properties
        param
        self
        vd
        dt
        result
        base_state
        base_time = 0;
        te % 着陸するまでの時間
        initialz % 初期時刻高度（takeoffする前の高度）
        fInit = 0;
    end

    methods
        function obj = LANDING_REFERENCE(self,opts)
            arguments
                self
                opts.dt
                opts.zd = [];
                opts.te = 3;
                opts.vd = 0.5;
            end
            % generate landing reference w.r.t. position
            obj.self = self;
            obj.result.state = STATE_CLASS(struct('state_list',["xd","p","v"],'num_list',[20,3,3]));
            obj.dt = opts.dt;
            obj.initialz = opts.zd;
            obj.vd = opts.vd;
            obj.te = opts.te;
        end
        function  result= do(obj,varargin)
            % [Input] time,cha,logger,env
            if isempty(obj.initialz)
                obj.initialz = obj.self.estimator.result.state.p(3);
            end
            if obj.fInit < 10 || isempty( obj.base_state ) % 飛行中にlanding modeに入った時点の情報を保存
                if obj.fInit == 0              % 空回しの高度（=接地時高度）を保存
                    obj.base_state = [obj.self.estimator.result.state.p(1:2);obj.self.reference.result.state.p(3)]; % x,y : current position, z : reference using at flight phase                    
                end
                if obj.fInit == 1            % 直前の高度を保存
                    obj.base_state = [obj.self.estimator.result.state.p(1:2);obj.self.reference.result.state.p(3)]; % x,y : current position, z : reference using at flight phase
                end
                obj.base_time=varargin{1}.t;            
                obj.result.state.xd = [obj.base_state;zeros(17,1)];
                obj.fInit = obj.fInit + 1; % 降下し始めるまでのループカウント
                % disp(obj.fInit)
            end

            obj.result.state.xd = obj.gen_ref_for_landing(varargin{1}.t-obj.base_time);
            obj.result.state.p = obj.result.state.xd(1:3,1);
            obj.result.state.v = obj.result.state.xd(5:7,1);
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

%% derivation and verification of curve_interpolation_9order
% clear
% n = 20;
% syms t z0 v0 te ve ze real
% syms a [1,n] real
% syms A [1,n] real
% A
% %%
% tra = 0;
% for i = 1:n
% tra = tra + a(i)*t^(i-1);
% end
% t1 = diff(tra,t,1);
% t2 = diff(tra,t,2);
% t3 = diff(tra,t,3);
% t4 = diff(tra,t,4);
% t0 = tra;
% [A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20]= solve([subs(t0,t,0) == z0;subs(t1,t,0) == v0;subs(t2,t,0)==0;subs(t3,t,0)==0;subs(t4,t,0)==0;...
% subs(t0,t,te) == ze;subs(t1,t,te) == ve;subs(t2,t,te)==0;subs(t3,t,te)==0;subs(t4,t,te)==0],a);
% B = [A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20];
% tra2 = subs(tra,a,B);
% %%
% dtra = diff(tra2,t,1);
% ddtra = diff(tra2,t,2);
% dddtra = diff(tra2,t,3);
% ddddtra = diff(tra2,t,4);
%
% comm = ["  curve interpolation function with both edge constraints wrt position and veloctiy",
%   "  9-th order polynomial of time",
%   "  [input] t,te,z0,v0,ze,ve",
%   "     t : current time",
%   "     te : interpolating length",
%   "     z0 : initial position",
%   "     v0 : initial velocity",
%   "     ze : terminal position",
%   "     ve : terminal velocity",
%   "  [output] [z,dz,ddz,dddz,ddddz] : position at time t and its higher time derivative"];
% matlabFunction([tra2,dtra,ddtra,dddtra,ddddtra],"File","curve_interpolation_9order.m","vars",{t,te,z0,v0,ze,ve},"Comments",comm)
%
% clear Z
% Te = 3;
% T = 0:0.1:Te;
% for i = 1:length(T)
% Z(i,:) = curve_interpolation_9order(T(i),Te,1,0,0,0);
% end
% plot(T,Z)

