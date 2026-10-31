classdef SIMULATED_OBSTACLE_SENSOR < handle
    % Omnidirectional surface-range gate + explicitly defined center-ray LOS.
    % LOS is a simulation convention, NOT a visibility theorem for every
    % point of an extended obstacle (and not a physical sensor model).
    properties (SetAccess = private)
        range_m
        LOG_ENABLED = false
        last_result = struct()
        timing = struct('count',0,'total_ms',0,'max_ms',0,'last_ms',NaN)
        timing_ring = nan(4096,2)
    end
    methods
        function obj=SIMULATED_OBSTACLE_SENSOR(range_m,log_enabled)
            assert(isnumeric(range_m)&&isscalar(range_m)&&isfinite(range_m)&&range_m>0, ...
                'SIMULATED_OBSTACLE_SENSOR:MissingRange', ...
                'Sensor range must come from the live replanner options.');
            obj.range_m=double(range_m);
            obj.LOG_ENABLED=logical(log_enabled);
        end
        function set_log(obj,enabled)
            obj.LOG_ENABLED=logical(enabled);
        end
        function S=observe(obj,t,pQ,raw)
            clk=[];if obj.LOG_ENABLED, clk=tic;end
            assert(isnumeric(pQ)&&numel(pQ)==3&&all(isfinite(pQ(:))), ...
                'SIMULATED_OBSTACLE_SENSOR:EKF','Missing valid current EKF UAV position.');
            pQ=double(pQ(:));
            M=numel(raw); dist=inf(1,M); in_range=false(1,M);
            visible=false(1,M); occluded=false(1,M); ray_miss=false(1,M);
            for i=1:M
                dist(i)=OBSTACLE_INFORMATION.surface_distance(pQ,raw(i));
                in_range(i)=dist(i)<=obj.range_m;
            end
            % This center-ray convention makes the occlusion rule explicit;
            % a line blocked by another obstacle cannot produce detection.
            for i=find(in_range)
                c=double(raw(i).p_center(:)); v=c-pQ; Nv=norm(v);
                if dist(i)==0, visible(i)=true;continue;end
                if Nv<=0, error('SIMULATED_OBSTACLE_SENSOR:Ray','Invalid center-ray geometry.');end
                % Every supported primitive is centered on p_center.
                % For custom convex vertices the source must also provide an
                % interior p_center; otherwise the center-ray is undefined.
                if OBSTACLE_INFORMATION.surface_distance(c,raw(i))>1e-9
                    error('SIMULATED_OBSTACLE_SENSOR:CenterOutside', ...
                        'Obstacle %g has p_center outside its shape.',double(raw(i).id));
                end
                left=0;right=1;
                for k=1:45
                    mid=(left+right)/2;
                    if OBSTACLE_INFORMATION.surface_distance(pQ+mid*v,raw(i))<=1e-10
                        right=mid;
                    else
                        left=mid;
                    end
                end
                stop=max(0,right-1e-7/max(1,Nv));
                blocked=false;
                for j=1:M
                    if i==j,continue;end
                    % Check other obstacles even if outside nominal sensor
                    % range: they may occlude the ray to the target.
                    [~,dj]=fminbnd(@(u) OBSTACLE_INFORMATION.surface_distance( ...
                        pQ+u*stop*v,raw(j)),0,1);
                    if dj<=1e-8, blocked=true;break;end
                end
                occluded(i)=blocked;
                visible(i)=~blocked;
            end
            visible_ids=[];if ~isempty(raw),visible_ids=[raw(visible).id];end
            S=struct('time_s',double(t),'range_m',obj.range_m,'distances_m',dist, ...
                'in_range_mask',in_range,'visible_mask',visible, ...
                'occluded_mask',occluded,'ray_miss_mask',ray_miss, ...
                'visible_ids',visible_ids, ...
                'model','RANGE_SURFACE_PLUS_SINGLE_CENTER_RAY_LOS', ...
                'full_surface_visibility_certified',false);
            obj.last_result=S;
            if obj.LOG_ENABLED
                ms=1e3*toc(clk);obj.timing.count=obj.timing.count+1;
                obj.timing.total_ms=obj.timing.total_ms+ms;
                obj.timing.max_ms=max(obj.timing.max_ms,ms);
                obj.timing.last_ms=ms;
                k=mod(obj.timing.count-1,size(obj.timing_ring,1))+1;
                obj.timing_ring(k,:)=[double(t),ms];
            end
        end
    end
end
