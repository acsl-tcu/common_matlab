function T=TEST_RA_FOUR_SENSOR_BENCHMARK(repeats)
% Sensor-only overhead; includes neither HLC forecast nor C/B/D geometry.
if nargin<1,repeats=25;end
methods={'single_sphere','single_ellipsoid','multiple_spheres','multiple_ellipsoids'};
rows=struct('method',{},'partsInWorld',{},'medianMs',{},'p95Ms',{},'maxMs',{});
for j=1:numel(methods)
    method=methods{j};
    n=36;
    parts=repmat(struct('kind','sphere','center',[0;0;0],'R',eye(3), ...
      'params',struct('radius',.1,'axes',[.1;.1;.1]), ...
      'obstacleName',''),1,n);
    for k=1:n
        parts(k).center=[.2*k;0;2];
        if startsWith(method,'single_')
            parts(k).obstacleName=sprintf('ob_%02d',k);
        else
            parts(k).obstacleName=sprintf('ob_%02d',ceil(k/3));
        end
        if contains(method,'ellipsoid')
            parts(k).kind='ellipsoid';parts(k).params.axes=[.12;.07;.04];
        end
    end
    w=struct('parts',parts,'approximation',method,'ground',[]);
    cfg=ra_sim_sensor_config(20);
    ra_sim_sensor_detect(w,[0;0;2],cfg); % warmup
    times=zeros(1,repeats);
    for k=1:repeats
        ticStart=tic;
        ra_sim_sensor_detect(w,[0;0;2],cfg);
        times(k)=1000*toc(ticStart);
    end
    ss=sort(times);
    rows(end+1)=struct('method',method,'partsInWorld',n, ...
        'medianMs',median(times),'p95Ms',ss(ceil(.95*repeats)), ...
        'maxMs',max(times)); %#ok<AGROW>
end
T=struct('status','MEASURED_SENSOR_ONLY_NOT_FULL_TICK', ...
    'records',rows,'testedParts',36,'full25msNotTested',true);
end
