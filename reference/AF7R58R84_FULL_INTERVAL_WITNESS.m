function R=AF7R58R84_FULL_INTERVAL_WITNESS(S,referenceFn,tStart,tEnd,gravity,kernel)
% R84 read-only exhaustive-in-time-grid NEGATIVE-witness search.
% Caller supplies source-derived candidate validity [tStart,tEnd].
% Covers [tStart,tEnd], NOT just H=4. Samples are never safety proofs.
% No Flight-loop, no MEX write, no HLC/EKF/plant mutation.
if nargin<6 || isempty(kernel)
    assert(exist('g26_future_trigger_kernel_mex','file')==3, ...
        '[R84] G26 native MEX is missing.');
    kernel=@g26_future_trigger_kernel_mex;
end
assert(isa(referenceFn,'function_handle'),'R84:ActualReferenceEvaluatorRequired');
assert(isstruct(S)&&isscalar(S),'R84:CapturedEventRequired');
assert(isfinite(tStart)&&isfinite(tEnd)&&tEnd>=tStart && ...
    isfinite(double(S.time_s)),'R84:InvalidActualValidityInterval');
assert(isscalar(gravity)&&isfinite(gravity)&&gravity>0,'R84:LiveGravityRequired');
P=S.r76_packet;dt=double(S.prediction_dt);L=double(S.cable_length_m);
assert(isfinite(dt)&&dt>0&&isfinite(L)&&L>0,'R84:LiveParametersRequired');
assert(double(P.reference_version)==double(S.reference_version), ...
    'R84:PacketVersionChanged');
ids=double(P.ids(:));mask=logical(S.r76_known_mask(:).');
assert(numel(ids)==numel(mask)&&any(mask),'R84:UnknownCoverage');
r=double(S.r76_radii_m(:));dm=double(S.r76_d_margin_m);
D=[r(1)+dm+double(P.delta_payload_m);r(2)+dm+double(P.delta_uav_m); ...
    r(3)+dm+double(P.delta_cable_m)];
assert(all(isfinite(D))&&all(D>=0),'R84:InvalidMargins');
R=struct('status','R84_NO_SAMPLED_NEGATIVE_WITNESS__NOT_CERTIFIED', ...
    'version',double(S.reference_version),'known_ids',ids(mask).', ...
    'scan_start_s',tStart,'scan_end_s',tEnd,'scan_step_s',dt, ...
    'samples_evaluated',0,'first_hit_time_s',NaN, ...
    'first_hit_obstacle_id',NaN,'first_hit_component',NaN, ...
    'found_conservative_negative_witness',false, ...
    'sample_grid_spans_supplied_interval',true, ...
    'validity_end_authoritative_to_caller',true, ...
    'continuous_time_proved',false,'real_HLC_plant_proved',false);
% Small fixed MEMORY batches (not a maneuver-duration cap). Each batch
% continues into the next with one shared endpoint for no temporal gap.
t=tStart;
while true
    rem=tEnd-t;
    count=min(160,floor(rem/dt));
    if count<1
        times=tEnd;
    else
        times=t+(0:count)*dt;
        if tEnd-times(end)<=16*eps(max(1,abs(tEnd)))
            times(end)=tEnd;
        end
    end
    N=numel(times);PL=zeros(3,N);Q=zeros(3,N);
    for j=1:N
        X=double(referenceFn(double(times(j))));X=X(:);
        assert(numel(X)>=11 && all(isfinite(X(1:11))), ...
            'R84:UnverifiedActiveReferenceEvaluator');
        sf=X(9:11)+[0;0;gravity];sfN=norm(sf);
        assert(isfinite(sfN)&&sfN>0,'R84:FlatnessSingularity');
        PL(:,j)=X(1:3);PL(3,j)=PL(3,j)-L;
        Q(:,j)=-sf/sfN;
    end
    % Last point may not lie on dt grid; always check it as a point
    % rather than passing a nonuniform spacing to the native kernel.
    if N>1
        H=double(times(end)-times(1));
        assert(abs(H-(N-1)*dt)<=64*eps(max(1,abs(H))), ...
            'R84:NonuniformTimeGrid');
    else
        H=0;
    end
    [hit,jHit,obs,comp]=kernel(H,dt,L,PL,Q, ...
        double(P.C),double(P.R),double(P.Araw),D,ids,double(mask));
    R.samples_evaluated=R.samples_evaluated+N;
    if logical(hit)
        assert(isfinite(jHit)&&jHit>=1&&jHit<=N,'R84:BadHitIndex');
        R.first_hit_time_s=times(jHit);
        R.first_hit_obstacle_id=double(obs);
        R.first_hit_component=double(comp);
        R.found_conservative_negative_witness=true;
        R.status='R84_HLC_FRAME_SAMPLED_WITNESS__REJECT_THIS_REFERENCE';
        return;
    end
    if N==1 || times(end)>=tEnd-16*eps(max(1,abs(tEnd)))
        break;
    end
    t=times(end);
    if tEnd-t<dt
        % Verify off-grid terminal validity endpoint separately.
        times=tEnd;
        X=double(referenceFn(times));X=X(:);
        assert(numel(X)>=11&&all(isfinite(X(1:11))), ...
            'R84:FinalEndpointEvalInvalid');
        sf=X(9:11)+[0;0;gravity];ns=norm(sf);
        assert(ns>0&&isfinite(ns),'R84:FinalEndpointSingular');
        PL=double(X(1:3));PL(3)=PL(3)-L;Q=-sf/ns;
        [hit,~,obs,comp]=kernel(0,dt,L,PL,Q, ...
            double(P.C),double(P.R),double(P.Araw),D,ids,double(mask));
        R.samples_evaluated=R.samples_evaluated+1;
        if logical(hit)
            R.first_hit_time_s=tEnd;
            R.first_hit_obstacle_id=double(obs);
            R.first_hit_component=double(comp);
            R.found_conservative_negative_witness=true;
            R.status='R84_HLC_FRAME_SAMPLED_WITNESS__REJECT_THIS_REFERENCE';
        end
        break;
    end
end
end
