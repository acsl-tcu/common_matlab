function out=BENCH_TRIGGER_V2_TICK_STAGES(repeats)
%BENCH_TRIGGER_V2_TICK_STAGES Staging entry timings using SYNTHETIC forecast.
% Not equivalent to an actual HLC/EKF/CBF/Flight 25ms measurement.
if nargin<1,repeats=30;end
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
c=[.3+.09*(0:35);.38*ones(1,36);2*ones(1,36)];
a=repmat([.20;.12;.09],1,36);
[S,E]=six_test_fixture(c,a);
agent.parameter=struct('mass',.762,'cableL',2,'gravity',9.81, ...
    'jx',.02237568,'jy',.02985236,'jz',.0480374,'loadmass',999);
agent.estimator.result.state=struct('mL',.14);
out=repmat(struct('mode','','cover','','totalMedianMs',NaN, ...
    'totalP95Ms',NaN,'stageMedianMs',struct()),3,2);
stageKeys={'modelSnapshot','configuration','obstacleCache', ...
    'forecastProvider','geometryEvaluation','instrumentationAndOther'};
for mi=1:3
    for ci=1:2
        mode=modes{mi};cover=covers{ci};
        if ci==1,world=S;else,world=E;end
        external=wp_system_geometry_config(mode);
        [~,~,F]=selected_test_context(mode,cover);
        provider=@(model)synthetic(model,F);
        [~,cache]=wp_six_trigger_run_selected(agent,world,external,mode,cover,provider,1,[]);
        data=zeros(repeats,numel(stageKeys));total=zeros(1,repeats);
        for i=1:repeats
            [r,cache]=wp_six_trigger_run_selected(agent,world,external, ...
                mode,cover,provider,1,cache,true);
            total(i)=r.selectedTriggerElapsedMs;
            for j=1:numel(stageKeys)
                data(i,j)=r.tickStageMs.(stageKeys{j});
            end
        end
        x=out(mi,ci);
        x.mode=mode;x.cover=cover;
        x.totalMedianMs=median(total);
        total=sort(total);x.totalP95Ms=total(ceil(.95*repeats));
        x.stageMedianMs=cell2struct(num2cell(median(data,1)),stageKeys,2);
        out(mi,ci)=x;
        fprintf('SYNTHETIC tick %-1s/%-19s median %.4f ms (geom %.4f, forecast callback %.4f)\n', ...
            mode,cover,x.totalMedianMs,x.stageMedianMs.geometryEvaluation, ...
            x.stageMedianMs.forecastProvider);
    end
end
end
function F=synthetic(model,original)
F=original;F.modelLoadMass=model.loadMass;
end
