classdef TRIGGER_OBSERVED_BROADPHASE
    % Shape-independent, SAMPLED-TIME exclusion only.
    % Each observed stationary solid O_j must be contained in its sensor-
    % certified ball B(center0,bounding_radius_m). This enclosing ball is
    % NEVER used to report a contact. For a given sample t_k, all of
    %  - UAV sphere,  - payload sphere,  - straight finite cable capsule
    % are outside B if their respective center/segment distances exceed
    % the sum of radii. If true for EVERY sample, expensive original-shape
    % narrow-phase queries for that obstacle are unnecessary.
    % This does NOT bound motion between samples, estimator error, cable
    % slackness, or perturbations. No continuous-time SAFE can result here.
    methods (Static)
        function G=evaluate(U,L,shapes,ru,rl,rc)
            assert(isnumeric(U)&&isreal(U)&&size(U,1)==3 && ...
                isequal(size(U),size(L))&&all(isfinite(U(:)))&& ...
                all(isfinite(L(:)))&&isstruct(shapes)&& ...
                all(isfinite([ru rl rc]))&&all([ru rl rc]>=0), ...
                'TRIGGER_OBSERVED_BROADPHASE:Input', ...
                'Finite 3xN endpoints, shapes and nonnegative physical radii required.');
            n=numel(shapes); K=size(U,2);
            seg=L-U;
            ss=sum(seg.^2,1);
            sample_scale=max([1;abs(U(:));abs(L(:));ru;rl;rc]);
            clear_samples=false(1,n);
            checked=zeros(1,n);
            ids=zeros(1,n);
            for j=1:n
                sh=shapes(j);
                if isfield(sh,'id')&&isnumeric(sh.id)&&isscalar(sh.id)&&isfinite(sh.id)
                    ids(j)=double(sh.id);
                else
                    ids(j)=NaN;
                end
                if ~isfinite(ids(j)) || ...
                        ~all(isfield(sh,{'center0','bounding_radius_m','is_stationary'})) || ...
                        ~isequal(sh.is_stationary,true)
                    continue
                end
                c=double(sh.center0(:)); b=double(sh.bounding_radius_m);
                if numel(c)~=3 || ~all(isfinite(c)) || ...
                        ~isscalar(b)||~isfinite(b)||b<0
                    continue
                end
                scale=max([sample_scale;abs(c);b]);
                guard=1024*eps(scale); % engineering guard; NOT directed rounding
                du=bsxfun(@minus,U,c); dl=bsxfun(@minus,L,c);
                % The nearest point of the FINITE segment, not a spherical
                % cable envelope, is tested at every sampled instant.
                alpha=zeros(1,K); good=isfinite(ss)&(ss>0);
                numerator=-sum(du(:,good).*seg(:,good),1);
                alpha(good)=max(0,min(1,numerator./ss(good)));
                near=du+bsxfun(@times,seg,alpha);
                dU=sqrt(sum(du.^2,1));
                dL=sqrt(sum(dl.^2,1));
                dC=sqrt(sum(near.^2,1));
                threshold=[b+ru+guard, b+rl+guard, b+rc+guard];
                if all(isfinite([dU dL dC threshold]))
                    checked(j)=K;
                    clear_samples(j)=all(dU>threshold(1)) && ...
                        all(dL>threshold(2)) && all(dC>threshold(3));
                end
            end
            G=struct('sampled_excluded',clear_samples, ...
                'needs_original_shape_query',~clear_samples, ...
                'sampled_excluded_ids',ids(clear_samples), ...
                'checked_samples',checked,'sample_count',K, ...
                'time_continuous_certified',false, ...
                'source','OBSERVED_SENSOR_ENCLOSING_BALL__EXCLUSION_ONLY');
        end
    end
end
