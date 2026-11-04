classdef TRIGGER_GENERIC_INTERVAL_SEPARATION
    % Observed-only, shape-agnostic, whole-interval, CONDITIONAL geometry.
    % Inputs T are already validated WHOLE-interval enclosures, not samples.
    % This class does NOT establish EKF errors, plant reachability, tautness,
    % outward rounding, or real-flight safety. Never auto-commit.
    methods (Static)
        function R=evaluate(T,O)
            R=struct('status','UNKNOWN_INPUT','reason','', ...
                'conditional_observed_geometry_clear',false, ...
                'component_clear',false(3,0,0), ...
                'first_unknown_interval',NaN,'first_unknown_obstacle_id',NaN, ...
                'intervals_checked',0,'obstacles_checked',0, ...
                'normals_checked',0,'safe_to_commit',false);
            try
                ts={'q_lo','q_hi','axis','half_angle_rad','L_lo','L_hi', ...
                    'r_uav','r_cable','r_payload','interval_bounds_s','horizon_s'};
                os={'shapes','observed_ids','verified_observation'};
                assert(isstruct(T)&&isscalar(T)&&all(isfield(T,ts)), 'TubeSchema');
                assert(isstruct(O)&&isscalar(O)&&all(isfield(O,os)), 'ObservationSchema');
                assert(isequal(O.verified_observation,true),'UnverifiedObservation');
                assert(isstruct(O.shapes),'ShapeSchema');
                ids=reshape(double(O.observed_ids),1,[]);
                assert(isnumeric(O.observed_ids)&&isreal(O.observed_ids)&& ...
                    all(isfinite(ids))&&numel(unique(ids))==numel(ids)&& ...
                    numel(ids)==numel(O.shapes),'InvalidObservedIds');
                if ~isempty(ids)
                    assert(all(isfield(O.shapes,{'id','center0','rotation', ...
                        'type','dimensions','is_stationary','bounding_radius_m'})) && ...
                        isequal(reshape(double([O.shapes.id]),1,[]),ids), ...
                        'PreparedShapeIDsDoNotMatchObservedIds');
                end
                assert(isnumeric(T.q_lo)&&isreal(T.q_lo)&& ...
                    isnumeric(T.q_hi)&&isreal(T.q_hi)&& ...
                    size(T.q_lo,1)==3&&isequal(size(T.q_lo),size(T.q_hi)), ...
                    'InvalidPositionBox');
                K=size(T.q_lo,2);
                assert(K>=1&&isequal(size(T.axis),[3 K])&& ...
                    isequal(size(T.interval_bounds_s),[2 K]),'InvalidIntervalDimensions');
                names={'half_angle_rad','L_lo','L_hi'};
                for i=1:numel(names)
                    assert(isequal(size(T.(names{i})),[1 K]),'InvalidRangeDimensions');
                end
                v=[T.q_lo(:);T.q_hi(:);T.axis(:);T.interval_bounds_s(:); ...
                   T.half_angle_rad(:);T.L_lo(:);T.L_hi(:); ...
                   T.r_uav;T.r_cable;T.r_payload;T.horizon_s];
                assert(isnumeric(v)&&isreal(v)&&all(isfinite(v))&& ...
                    all(T.q_lo(:)<=T.q_hi(:))&&all(T.L_lo>0)&& ...
                    all(T.L_lo<=T.L_hi)&&all(T.half_angle_rad>=0)&& ...
                    all(T.half_angle_rad<=pi)&& ...
                    all([T.r_uav T.r_cable T.r_payload]>=0)&& ...
                    isscalar(T.r_uav)&&isscalar(T.r_cable)&& ...
                    isscalar(T.r_payload)&&isscalar(T.horizon_s)&& ...
                    T.horizon_s>0&& ...
                    T.interval_bounds_s(1,1)==0&& ...
                    T.interval_bounds_s(2,end)==T.horizon_s&& ...
                    all(T.interval_bounds_s(1,:)<T.interval_bounds_s(2,:))&& ...
                    all(T.interval_bounds_s(1,2:end)== ...
                        T.interval_bounds_s(2,1:end-1)), ...
                    'IncompleteOrInvalidWholeTimeTube');
                assert(all(abs(sqrt(sum(T.axis.^2,1))-1)<1e-12), ...
                    'ConeAxesMustBeUnit');
                for i=1:numel(O.shapes)
                    s=O.shapes(i);
                    assert(isequal(s.is_stationary,true)&& ...
                        isequal(size(s.rotation),[3 3])&&numel(s.center0)==3&& ...
                        all(isfinite(double(s.center0(:))))&& ...
                        all(isfinite(double(s.rotation(:))))&& ...
                        norm(double(s.rotation)'*double(s.rotation)-eye(3),'fro')<1e-9&& ...
                        det(double(s.rotation))>0&& ...
                        isscalar(s.bounding_radius_m)&& ...
                        isfinite(s.bounding_radius_m)&&s.bounding_radius_m>=0, ...
                        'InvalidStationaryPreparedShape');
                end
            catch ME
                R.reason=[ME.identifier ' | ' ME.message];
                return
            end
            N=numel(O.shapes);R.intervals_checked=K;
            R.obstacles_checked=N;R.component_clear=false(3,N,K);
            if N==0
                R.status='NO_OBSERVED_OBSTACLES__WORLD_UNKNOWN';
                return
            end
            try
            for k=1:K
                qmid=0.5*(T.q_lo(:,k)+T.q_hi(:,k));
                pLmid=qmid+0.5*(T.L_lo(k)+T.L_hi(k))*T.axis(:,k);
                for j=1:N
                    s=O.shapes(j);c=double(s.center0(:));
                    v=[qmid-c,pLmid-c,0.5*(qmid+pLmid)-c,eye(3)];
                    nv=sqrt(sum(v.^2,1));v=v(:,nv>0)./nv(nv>0);
                    v=[v,-v];
                    for ni=1:size(v,2)
                        n=v(:,ni);R.normals_checked=R.normals_checked+1;
                        obs_hi=dot(n,c)+ ...
                            TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many(s,n);
                        for part=1:3
                            if R.component_clear(part,j,k),continue;end
                            body_lo=-TRIGGER_GENERIC_INTERVAL_SEPARATION.body_support_upper( ...
                                T,k,part,-n);
                            % Engineering guard only; NOT directed rounding.
                            guard=128*eps(max([1;abs(body_lo);abs(obs_hi)]));
                            if body_lo>obs_hi+guard
                                R.component_clear(part,j,k)=true;
                            end
                        end
                        if all(R.component_clear(:,j,k)),break;end
                    end
                    if ~all(R.component_clear(:,j,k))&&isnan(R.first_unknown_interval)
                        R.first_unknown_interval=k;
                        R.first_unknown_obstacle_id=ids(j);
                    end
                end
            end
            catch ME
                R.component_clear=false(3,N,K);
                R.status='UNKNOWN_GEOMETRY_EVALUATION_FAILED';
                R.reason=[ME.identifier ' | ' ME.message];
                return
            end
            R.conditional_observed_geometry_clear=all(R.component_clear(:));
            if R.conditional_observed_geometry_clear
                R.status='PASS_CONDITIONAL_OBSERVED_WHOLE_INTERVAL__NOT_FLIGHT_SAFETY';
                R.reason=['Reachability, EKF, tautness, sensor provenance, ' ...
                    'external parameter validity, and directed rounding are separate proofs.'];
            else
                R.status='UNKNOWN_NO_SEPARATING_WITNESS__NOT_COLLISION';
                R.reason='An available normal did not separate every component on every interval.';
            end
        end
    end
    methods (Static, Access=private)
        function h=body_support_upper(T,k,part,n)
            qhi=sum(max(n.*T.q_lo(:,k),n.*T.q_hi(:,k)));
            nr=norm(n);a=T.axis(:,k);theta=T.half_angle_rad(k);
            if theta==0
                hd=dot(n,a);
            elseif theta==pi
                hd=nr;
            else
                hd=nr*cos(max(0,acos(max(-1,min(1,dot(n,a)/nr)))-theta));
            end
            switch part
                case 1
                    h=qhi+nr*T.r_uav;
                case 2
                    % Whole straight cable, all s in [0,1].
                    h=qhi+max(0,T.L_hi(k)*hd)+nr*T.r_cable;
                case 3
                    h=qhi+max(T.L_lo(k)*hd,T.L_hi(k)*hd)+nr*T.r_payload;
            end
        end
    end
end
