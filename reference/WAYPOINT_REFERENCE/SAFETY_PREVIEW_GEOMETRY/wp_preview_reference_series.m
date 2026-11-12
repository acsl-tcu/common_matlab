function [X,info]=wp_preview_reference_series(source,times,opts)
% Read C6 reference at FUTURE times WITHOUT changing the live reference.
% Function handle: source(t) returns 28x1 desired [p;yaw;v;...].
% WAYPOINT_REFERENCE object: copied virtual-clock governor state + held
% command (future estimation is UNKNOWN). This is only a scenario assumption.
if nargin<3,opts=struct();end
if ~isfield(opts,'flightStartTime'),opts.flightStartTime=5.0;end
times=times(:)';X=zeros(28,numel(times));
info=struct('provider','','adaptationAssumption','none','isLiveStateUntouched',true);
if isa(source,'function_handle')
    for k=1:numel(times)
        X(:,k)=source(times(k));
    end
    info.provider='function_handle';
elseif isobject(source) && isprop(source,'generator')
    generator=source.generator;
    if isprop(source,'adaptiveProgress') && source.adaptiveProgress && ...
            isprop(source,'lastFlightTime') && ~isempty(source.lastFlightTime)
        tau=source.virtualTime;
        states=source.rateState;
        rate=source.heldRateCommand;
        tprev=source.lastFlightTime;
        for k=1:numel(times)
            h=times(k)-tprev;
            if h<-1e-9
                error('WPPreview:ReferenceHistory','Preview times predate governor state');
            end
            [states,tau,tauDer]=wp_governor_step(states,tau,rate,max(0,h),source.filterTime);
            X(:,k)=generator.evaluateWarped(tau,tauDer);
            tprev=times(k);
        end
        info.provider='WAYPOINT_REFERENCE_V3';
        info.adaptationAssumption='held_current_progress_command';
    else
        if isprop(source,'t') && ~isempty(source.t)
            tStart=source.t;
        else
            tStart=opts.flightStartTime;
        end
        for k=1:numel(times)
            X(:,k)=generator.evaluate(max(0,times(k)-tStart));
        end
        info.provider='WAYPOINT_REFERENCE_nominal';
        info.adaptationAssumption='nominal_time_law';
    end
else
    error('WPPreview:ReferenceSource', ...
        'Supply @(t) 28x1 reference or an initialized WAYPOINT_REFERENCE object');
end
if size(X,1)~=28 || size(X,2)~=numel(times) || any(~isfinite(X(:)))
    error('WPPreview:ReferenceOutput','Reference must return finite 28x1 at each time');
end
end
