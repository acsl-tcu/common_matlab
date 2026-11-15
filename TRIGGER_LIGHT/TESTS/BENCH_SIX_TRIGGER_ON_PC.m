function out=BENCH_SIX_TRIGGER_ON_PC(repetitions)
%BENCH_SIX_TRIGGER_ON_PC 1 call = 1 mode; report ALL SIX independently.
%  1-2 ms target is PER-SELECTED-MODE, NOT sum of 6 experimental runs.
%  Timings include geometry checking, but NOT forecast generation or EKF,
%  controller, CBF, trajectory generation. Measured on user's actual PC.
if nargin<1,repetitions=100;end
assert(repetitions>=10 && repetitions==floor(repetitions));
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
scenarios={'empty','distant_36','cable_contact','near_noncontact_36'};
out=struct();
for si=1:numel(scenarios)
    switch scenarios{si}
        case 'empty'
            centers=zeros(3,0);axesLengths=zeros(3,0);
        case 'distant_36'
            centers=[40+(1:36);15*ones(1,36);2*ones(1,36)];
            axesLengths=repmat([.20;.12;.09],1,36);
        case 'cable_contact'
            centers=[.8;0;2];axesLengths=[.18;.18;.18];
        case 'near_noncontact_36'
            centers=[.3+.09*(0:35);.38*ones(1,36);2*ones(1,36)];
            axesLengths=repmat([.20;.12;.09],1,36);
    end
    [sphereWorld,ellipsoidWorld]=six_test_fixture(centers,axesLengths);
    stats=repmat(struct('systemMode','','cover','','medianMs',NaN, ...
        'p95Ms',NaN,'maxMs',NaN,'allUnder2Ms',false, ...
        'status','','narrowphasePairs',0, ...
        'syntheticProviderTickMedianMs',NaN, ...
        'syntheticProviderTickP95Ms',NaN, ...
        'syntheticProviderTickMaxMs',NaN),3,2);
    for mi=1:3
        for ci=1:2
            mode=modes{mi};cover=covers{ci};
            [~,cfg,F]=selected_test_context(mode,cover);
            if ci==1,world=sphereWorld;else,world=ellipsoidWorld;end
            known=wp_six_trigger_prepare(world,cover);
            for warmup=1:5,wp_six_trigger_evaluate(known,cfg,F);end
            times=zeros(1,repetitions);
            for repeat=1:repetitions
                clock=tic;
                R=wp_six_trigger_evaluate(known,cfg,F);
                times(repeat)=1000*toc(clock);
                assert(strcmp(R.systemMode,mode));
            end
            sorted=sort(times);
            s=stats(mi,ci);
            s.systemMode=mode;s.cover=cover;
            s.medianMs=median(times);
            s.p95Ms=sorted(ceil(.95*repetitions));
            s.maxMs=max(times);
            s.allUnder2Ms=all(times<=2);
            s.status=R.status;
            s.narrowphasePairs=R.narrowphasePairs;
            % Also measure the complete staging entry point, including
            % parameter read, cache validation and a SYNTHETIC provider.
            % This is NOT a real HLC future-predictor benchmark.
            agent.parameter=struct('mass',0.762,'cableL',2,'gravity',9.81, ...
                'jx',0.02237568,'jy',0.02985236,'jz',0.0480374,'loadmass',999);
            agent.estimator.result.state=struct('mL',0.14);
            external=wp_system_geometry_config(mode);
            callback=@(model) syntheticProvider(model,F);
            [~,cache]=wp_six_trigger_run_selected(agent,world,external, ...
                mode,cover,callback,1,[]);
            tickTimes=zeros(1,repetitions);
            for j=1:repetitions
                [tick,cache]=wp_six_trigger_run_selected(agent,world,external, ...
                    mode,cover,callback,1,cache);
                tickTimes(j)=tick.selectedTriggerElapsedMs;
            end
            tickSorted=sort(tickTimes);
            s.syntheticProviderTickMedianMs=median(tickTimes);
            s.syntheticProviderTickP95Ms=tickSorted(ceil(.95*repetitions));
            s.syntheticProviderTickMaxMs=max(tickTimes);
            stats(mi,ci)=s;
            fprintf('%-18s  %s %-19s  median %8.4f / P95 %8.4f / MAX %8.4f ms [%s]\n', ...
                scenarios{si},mode,cover,s.medianMs,s.p95Ms,s.maxMs,s.status);
        end
    end
    out.(scenarios{si})=stats;
end
% One selected approximation is (re)prepared ONLY when observation changes.
preparation=zeros(3,2);
for ci=1:2
    cover=covers{ci};
    if ci==1,world=sphereWorld;else,world=ellipsoidWorld;end
    dt=zeros(1,repetitions);
    for j=1:repetitions
        clock=tic;
        wp_six_trigger_prepare(world,cover);
        dt(j)=1000*toc(clock);
    end
    preparation(:,ci)=[median(dt); prctileLocal(dt,95); max(dt)];
end
out.preparationMedianP95MaxMs=preparation;
out.targetPerSelectedModeMs=2;
out.allSixSimultaneouslyInFlight=false;
out.includesActualForecastHlcEkfCbfReplanning=false;
out.full25msLoopMeasured=false;
end
function q=prctileLocal(x,p)
s=sort(x);q=s(ceil(numel(s)*p/100));
end

function out=syntheticProvider(model,fixtureForecast)
% Timing/test API only; NO claim to reconstruct the original HLC.
out=fixtureForecast;
out.modelLoadMass=model.loadMass;
end
