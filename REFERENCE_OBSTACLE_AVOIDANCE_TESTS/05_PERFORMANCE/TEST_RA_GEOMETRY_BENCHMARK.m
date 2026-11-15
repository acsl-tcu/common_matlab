function R=TEST_RA_GEOMETRY_BENCHMARK(repeats)
% Informational: geometry-only benchmark. NOT forecast + geometry tick.
if nargin<1,repeats=25;end
methods={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
agent=RA_TEST_AGENT();records=struct('mode',{},'cover',{}, ...
    'medianMs',{},'p95Ms',{},'maxMs',{},'status',{});
for m=1:3
    for c=1:2
        mode=methods{m};cover=covers{c};
        P=reference_avoidance_prepare_observed(agent,mode,cover, ...
            RA_TEST_WORLD(cover,'near'));
        F=RA_TEST_FORECAST(P.cfg);
        a=zeros(1,repeats);
        wp_six_trigger_evaluate_fast_v8(P.known,P.cfg,F); % warmup
        for k=1:repeats
            watch=tic;
            G=wp_six_trigger_evaluate_fast_v8(P.known,P.cfg,F);
            a(k)=1000*toc(watch);
        end
        s=sort(a);
        z=struct('mode',mode,'cover',cover, ...
            'medianMs',median(a), ...
            'p95Ms',s(max(1,ceil(.95*numel(s)))), ...
            'maxMs',max(a),'status',G.status);
        records(end+1)=z; %#ok<AGROW>
    end
end
R=struct('status','INFORMATIONAL_GEOMETRY_ONLY', ...
    'records',records,'notFullFlight25ms',true,'notPhysicalSafetyTest',true);
end
