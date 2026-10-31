classdef FUTURE_COLLISION_TRIGGER < handle
    % Current-EKF linear-motion geometric PREVIEW ONLY.
    % Not an HLC/plant reachable-set certificate; no candidate is authorized.
    properties (SetAccess = private)
        horizon_s
        dt
        r_drone
        r_load
        r_cable
        LOG_ENABLED = false
        last_result = struct()
        timing = struct('count',0,'total_ms',0,'max_ms',0,'last_ms',NaN)
        timing_ring = nan(4096,2)
        fired_ids = zeros(1,0)
        fired_reference_version = 0
        duplicate_count = 0
    end
    methods
        function obj=FUTURE_COLLISION_TRIGGER(H,dt,rQ,rL,rC,enabled)
            vals=[H,dt,rQ,rL,rC];
            assert(isnumeric(vals)&&numel(vals)==5&&all(isfinite(vals))&&all(vals>0), ...
                'FUTURE_COLLISION_TRIGGER:MissingAuthority', ...
                'Horizon, step and physical radii must be externally authorized.');
            assert(abs(H/dt-round(H/dt))<1e-8*max(1,H/dt), ...
                'FUTURE_COLLISION_TRIGGER:Grid','Horizon/dt must be integral.');
            obj.horizon_s=double(H);obj.dt=double(dt);
            obj.r_drone=double(rQ);obj.r_load=double(rL);obj.r_cable=double(rC);
            obj.LOG_ENABLED=logical(enabled);
        end
        function set_log(obj,enabled)
            obj.LOG_ENABLED=logical(enabled);
        end
        function S=evaluate(obj,t,ekf,known,reference_version,mL)
            clk=[];if obj.LOG_ENABLED,clk=tic;end
            req={'p','pL','pT','v','vL'};
            for j=1:numel(req)
                assert(isfield(ekf,req{j}), ...
                    'FUTURE_COLLISION_TRIGGER:MissingEKF','Missing EKF field %s.',req{j});
                value=ekf.(req{j});
                assert(isnumeric(value) && numel(value)==3 && all(isfinite(value(:))), ...
                    'FUTURE_COLLISION_TRIGGER:MissingEKF', ...
                    'Current EKF field %s must be finite 3x1.',req{j});
            end
            assert(isnumeric(mL)&&isscalar(mL)&&isfinite(mL)&&mL>0, ...
                'FUTURE_COLLISION_TRIGGER:MissingMass','Current estimator mL is required.');
            assert(isnumeric(reference_version)&&isscalar(reference_version)&& ...
                isfinite(reference_version),'FUTURE_COLLISION_TRIGGER:Version','Missing reference version.');
            if reference_version~=obj.fired_reference_version
                obj.fired_ids=zeros(1,0);
                obj.fired_reference_version=reference_version;
            end
            S=struct('time_s',double(t),'status','UNKNOWN_NO_SAMPLED_HIT__HLC_UNVERIFIED', ...
                'numeric_request',false,'new_event',false, ...
                'earliest_ttc_s',NaN,'obstacle_id',NaN, ...
                'component','NONE','reference_version',reference_version, ...
                'current_mass_kg',double(mL),'closed_loop_certified',false, ...
                'continuous_time_certified',false,'safe_to_commit',false, ...
                'sample_count',round(obj.horizon_s/obj.dt)+1, ...
                'duplicate_suppressed',false,'all_risk_ids',zeros(1,0), ...
                'new_event_ids',zeros(1,0),'new_event_obstacle_id',NaN);
            pQ=double(ekf.p(:));pL=double(ekf.pL(:));
            vQ=double(ekf.v(:));vL=double(ekf.vL(:));
            % This sample sweep makes no claim about dynamics, swing or
            % tracking between the samples; no-hits must be UNKNOWN.
            hit_ids=zeros(1,0);hit_times=zeros(1,0);hit_parts=cell(1,0);
            for k=0:round(obj.horizon_s/obj.dt)
                tau=k*obj.dt;
                qk=pQ+tau*vQ;lk=pL+tau*vL;
                for j=1:numel(known)
                    o=known(j);
                    if any(hit_ids==double(o.id)),continue;end
                    % Cheap circumscribed-sphere broadphase, for REJECTION
                    % only. Exact raw shape remains used for sampled hits.
                    seg=lk-qk;den=dot(seg,seg);
                    if den==0
                        s0=0;
                    else
                        s0=max(0,min(1,dot(double(o.p_center(:))-qk,seg)/den));
                    end
                    bound=max(OBSTACLE_INFORMATION.display_axes(o));
                    if norm(qk+s0*seg-double(o.p_center(:)))> ...
                            bound+max([obj.r_drone,obj.r_load,obj.r_cable])
                        continue;
                    end
                    dq=OBSTACLE_INFORMATION.surface_distance(qk,o)-obj.r_drone;
                    dl=OBSTACLE_INFORMATION.surface_distance(lk,o)-obj.r_load;
                    if dq<=0
                        part='UAV';hit=true;
                    elseif dl<=0
                        part='PAYLOAD';hit=true;
                    else
                        % Straight cable over its FULL length at each sampled
                        % instant, not a sphere around its endpoints.
                        f=@(s) OBSTACLE_INFORMATION.surface_distance(qk+s*(lk-qk),o);
                        [~,dc]=fminbnd(f,0,1);
                        hit=dc<=obj.r_cable;
                        part='CABLE';
                    end
                    if hit
                        hit_ids(end+1)=double(o.id); %#ok<AGROW>
                        hit_times(end+1)=tau; %#ok<AGROW>
                        hit_parts{end+1}=part; %#ok<AGROW>
                    end
                end
            end
            S.all_risk_ids=hit_ids;
            if ~isempty(hit_ids)
                [~,j]=min(hit_times);
                S.earliest_ttc_s=hit_times(j);
                S.obstacle_id=hit_ids(j);
                S.component=hit_parts{j};
                S.status='PREVIEW_SAMPLED_POSSIBLE_HIT__HLC_UNVERIFIED';
                new_ids=setdiff(hit_ids,obj.fired_ids,'stable');
                S.new_event_ids=new_ids;
                S.new_event=~isempty(new_ids);
                S.numeric_request=S.new_event;
                S.duplicate_suppressed=any(ismember(hit_ids,obj.fired_ids));
                obj.duplicate_count=obj.duplicate_count+sum(ismember(hit_ids,obj.fired_ids));
                if S.new_event
                    ix=find(ismember(hit_ids,new_ids));
                    [~,jnew]=min(hit_times(ix));
                    S.new_event_obstacle_id=hit_ids(ix(jnew));
                    obj.fired_ids=union(obj.fired_ids,new_ids,'stable');
                end
            end
            obj.last_result=S;
            if obj.LOG_ENABLED
                ms=1e3*toc(clk);obj.timing.count=obj.timing.count+1;
                obj.timing.total_ms=obj.timing.total_ms+ms;
                obj.timing.last_ms=ms;
                obj.timing.max_ms=max(obj.timing.max_ms,ms);
                i=mod(obj.timing.count-1,size(obj.timing_ring,1))+1;
                obj.timing_ring(i,:)=[double(t),ms];
            end
        end
    end
end
