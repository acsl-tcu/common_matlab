classdef PATH_REFERENCE_BSPLINE < handle
    % PATH_REFERENCE_BSPLINE  Independent degree-7 B-spline spatial-path reference.
    % Use exactly the existing external generator/settings/order-6 syntax:
    %   PATH_REFERENCE_BSPLINE(agent, {"gen_ref_c6_bspline", {"controlPoints",P,"maxSpeed",1.5}, 6})
    % Outputs the same 28-entry time-derivative jet as the existing HLC.
    % No TIME_VARYING_REFERENCE_C6 or gen_ref_c6_path_core dependency.
    % NOTE: auto-duration is chosen ONCE from path shape and desired speed.
    % This is NOT real-time state-dependent path following.
    properties
        param
        func
        ref
        self
        t = []
        cha = 's'
        dfunc
        result
    end
    methods
        function obj = PATH_REFERENCE_BSPLINE(self,args)
            arguments
                self
                args
            end
            if numel(args)~=3 || ~isequal(args{3},6) || ...
                    ~strcmp(char(args{1}),'gen_ref_c6_bspline')
                error('PATH_REFERENCE_BSPLINE:Arguments', ...
                    'Expected gen_ref_c6_bspline, settings, and order 6.');
            end
            obj.self=self;
            params=args{2};
            obj.ref=gen_ref_c6_bspline(params{:});
            if ~isstruct(obj.ref) || ~isfield(obj.ref,'pYaw') || ...
                    ~isfield(obj.ref,'order') || obj.ref.order~=6
                error('PATH_REFERENCE_BSPLINE:Generator','Expected analytic C6 reference.');
            end
            obj.func=obj.ref.pYaw;
            x0=obj.func(0.0);
            if ~isnumeric(x0) || ~isequal(size(x0),[28 1]) || ...
                    ~isreal(x0) || any(~isfinite(x0))
                error('PATH_REFERENCE_BSPLINE:Initial','Reference must be finite real 28x1.');
            end
            obj.result.state=STATE_CLASS(struct( ...
                'state_list',["xd","p","q","v"], ...
                'num_list',[28,3,3,3]));
            obj.result.state.set_state('xd',x0);
            if isprop(self.estimator.result.state,'p')
                obj.result.state.set_state('p',self.estimator.result.state.get('p'));
            end
            if isprop(self.estimator.result.state,'q')
                obj.result.state.set_state('q',self.estimator.result.state.get('q'));
            end
            if isprop(self.estimator.result.state,'v')
                obj.result.state.set_state('v',self.estimator.result.state.get('v'));
            end
        end
        function result=do(obj,varargin)
            obj.cha=varargin{2};
            if obj.cha=='f'
                if isempty(obj.t)
                    obj.t=varargin{1}.t;
                end
                elapsed=varargin{1}.t-obj.t;
            else
                elapsed=0;
            end
            xd=obj.func(elapsed);
            if ~isequal(size(xd),[28 1]) || any(~isfinite(xd))
                error('PATH_REFERENCE_BSPLINE:Jet','Reference is not finite 28x1.');
            end
            obj.result.state.xd=xd;
            obj.result.state.p=xd(1:3);
            obj.result.state.v=xd(5:7);
            if isfield(obj.ref,'q')
                qd=obj.ref.q(elapsed);
                obj.result.state.q(:,1)=qd(1:3);
            else
                obj.result.state.q(3,1)=xd(4);
            end
            result=obj.result;
        end
        function show(obj,logger)
            rp=logger.data(1,'p','r');
            ep=logger.data(1,'p','e');
            plot3(rp(:,1),rp(:,2),rp(:,3));
            daspect([1,1,1]); hold on;
            plot3(ep(:,1),ep(:,2),ep(:,3));
            legend({'reference','estimate'});
            title('bspline spatial path and estimated trajectory');
            xlabel('x [m]'); ylabel('y [m]'); hold off;
        end
    end
end
