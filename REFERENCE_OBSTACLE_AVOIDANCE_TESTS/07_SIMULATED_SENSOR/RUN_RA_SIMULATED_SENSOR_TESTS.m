function R=RUN_RA_SIMULATED_SENSOR_TESTS(runPerformance)
%RUN_RA_SIMULATED_SENSOR_TESTS Independent sensor tests, no ZIP extraction.
% Called after SETUP_REFERENCE_OBSTACLE_AVOIDANCE has added the sensor path.
if nargin<1,runPerformance=true;end
root=fileparts(mfilename('fullpath'));
addpath(root,'-begin');
R=struct('status','NOT_RUN','basic',struct(),'generator',struct(), ...
    'performance',struct(),'resultsZip','','failures',{{}});
try
    R.basic=TEST_RA_SIMULATED_SENSOR();
catch ME
    R.basic=struct('status','FAIL','errorText', ...
        getReport(ME,'extended','hyperlinks','off'));
    R.failures{end+1}='basic';
end
try
    R.generator=TEST_RA_SIMULATED_SENSOR_GENERATOR();
catch ME
    R.generator=struct('status','FAIL','errorText', ...
        getReport(ME,'extended','hyperlinks','off'));
    R.failures{end+1}='generator';
end
if runPerformance
    try
        R.performance=sensorBenchmark();
    catch ME
        R.performance=struct('status','FAIL','errorText', ...
            getReport(ME,'extended','hyperlinks','off'));
        R.failures{end+1}='performance';
    end
end
if isempty(R.failures)
    R.status='PASS_SIM_SENSOR_CORE_CURRENT_ONLY';
else
    R.status='FAILED_SENSOR_TESTS';
end
outdir=fullfile(fileparts(root),'RESULTS');
if ~isfolder(outdir),mkdir(outdir);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
matPath=fullfile(outdir,['RA_SENSOR_' stamp '.mat']);
zipPath=fullfile(outdir,['RA_SENSOR_' stamp '.zip']);
R.resultsZip=zipPath;
save(matPath,'R');
zip(zipPath,matPath);
fprintf('[Sensor] %s  results=%s\n',R.status,R.resultsZip);
end
function B=sensorBenchmark()
covers={'multiple_spheres','multiple_ellipsoids'};
records=struct('approximation',{},'parts',{},'medianMs',{}, ...
    'p95Ms',{},'maxMs',{},'detectedObjects',{});
for c=1:2
    cover=covers{c};
    P=repmat(struct('kind','sphere','center',[0 0 0], ...
        'R',eye(3),'params',struct('radius',.5,'axes',[.5 .5 .5]), ...
        'obstacleName',''),1,36);
    for j=1:36
        P(j).center=[-25+50*(j-1)/35,mod(j,7)-3,1];
        P(j).obstacleName=sprintf('obj%02d',j);
        if c==2
            P(j).kind='ellipsoid';P(j).params.axes=[2 .3 .2];
            angle=j*pi/17;
            P(j).R=[cos(angle) -sin(angle) 0; ...
                    sin(angle) cos(angle) 0;0 0 1];
        end
    end
    W=struct('parts',P,'approximation',cover,'ground',[]);
    cfg=ra_sim_sensor_config(cover,20);
    for k=1:10,ra_sim_sensor_detect(W,[0;0;0],cfg);end
    elapsed=zeros(1,100);num=0;
    for k=1:100
        ticId=tic;
        [~,info]=ra_sim_sensor_detect(W,[0;0;0],cfg);
        elapsed(k)=1000*toc(ticId);
        num=info.detectedObjectCount;
    end
    s=sort(elapsed);
    records(end+1)=struct('approximation',cover,'parts',36, ...
        'medianMs',median(elapsed),'p95Ms',s(ceil(.95*numel(s))), ...
        'maxMs',max(elapsed),'detectedObjects',num); %#ok<AGROW>
end
B=struct('status','PASS_BENCHMARK_MEASURED','records',records, ...
    'includesFlight',false);
end
