function R = RUN_PURE_SIX_TRIGGER_TESTS(numRandomCases,benchmarkRepeats)
%RUN_PURE_SIX_TRIGGER_TESTS 6 INDEPENDENT one-selected-mode regression tests.
%
% Run from common_matlab/TRIGGER_LIGHT after extracting the ZIP:
%   R = RUN_PURE_SIX_TRIGGER_TESTS(30,100);
%   disp(R.status); disp(R.benchmark.near_noncontact_36)
%
% This is STAGING only. It does NOT modify the original Flight run/HLC/CBF.
if nargin<1,numRandomCases=30;end
if nargin<2,benchmarkRepeats=100;end
root=fileparts(mfilename('fullpath'));
oldPath=path;
restore=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(root,fullfile(root,'SIX_WAY'),fullfile(root,'TESTS'));
addpath(genpath(fullfile(root,'TESTS','REFERENCE_GEOMETRY')));
% Search for original DRONE_OBSTACLES beside the TRIGGER_LIGHT directory.
projectRoot=fileparts(root);
obstacleDir=fullfile(projectRoot,'DRONE_OBSTACLES');
if exist(fullfile(obstacleDir,'DRONE_OBSTACLES.m'),'file')==2
    addpath(genpath(obstacleDir));
end

fprintf('\n=== PURE PORT: SIX SEPARATE SELECTED-MODE TESTS ===\n');
R=struct();
R.basic=TEST_SIX_TRIGGER_BASIC();
R.inclusion=TEST_SIX_TRIGGER_SWEEP_INCLUSION();
R.obstacleGenerator=TEST_SIX_TRIGGER_DRONE_OBSTACLES();
R.tickInterface=TEST_SIX_TRIGGER_TICK_INTERFACE();
R.realHlcTrace=TEST_SIX_TRIGGER_REJOIN_TRACE();
R.random=TEST_SIX_TRIGGER_RANDOM(numRandomCases);
R.benchmark=BENCH_SIX_TRIGGER_ON_PC(benchmarkRepeats);
R.status='PASS_INDEPENDENT_SELECTED_GEOMETRY__HLC_AND_REALTIME_PENDING';
R.testedCombinations={'C/multiple_spheres','C/multiple_ellipsoids', ...
    'B/multiple_spheres','B/multiple_ellipsoids', ...
    'D/multiple_spheres','D/multiple_ellipsoids'};
R.oneSelectedModePerCall=true;
R.physicalSafetyCertified=false;
R.actualHlcForecastIntegrated=false;
R.hlcErrorBoundsDerived=false;
R.full25msLoopMeasured=false;
R.twoMsGuaranteed=false;

% Save one compact ZIP for feedback and reproducible per-mode measurements.
outDir=fullfile(root,'TESTS','RESULTS');
if ~exist(outDir,'dir'),mkdir(outDir);end
stem='PURE_SELECTED_TRIGGER_RESULTS_FOR_CHATGPT';
save(fullfile(outDir,[stem '.mat']),'R');
filename=fullfile(outDir,[stem '.txt']);
fid=fopen(filename,'w');
if fid<0,error('LightTrigger:Results','Cannot write results');end
cleanupFile=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'PURE PORT: SIX SEPARATE SELECTED-MODE GEOMETRY TESTS\n');
fprintf(fid,'Status: %s\n',R.status);
fprintf(fid,'Original obstacle generator: %s\n',R.obstacleGenerator.status);
fprintf(fid,'Actual HLC Flight trace: %s\n',R.realHlcTrace.status);
fprintf(fid,'Random collision witnesses by mode rows C/B/D cols spheres/ellipsoids:\n');
modeNames={'C','B','D'};
for mi=1:3
    fprintf(fid,'  mode %s: witnesses %d,%d; false-clears %d,%d\n', ...
       modeNames{mi},R.random.witnessesByMode(mi,1), ...
       R.random.witnessesByMode(mi,2), ...
       R.random.falseClearByMode(mi,1),R.random.falseClearByMode(mi,2));
end
for scenario={'empty','distant_36','cable_contact','near_noncontact_36'}
    key=scenario{1};
    fprintf(fid,'\nSCENARIO %s (each row is a separate trigger call)\n',key);
    stats=R.benchmark.(key);
    for mi=1:3
        for ci=1:2
            item=stats(mi,ci);
            fprintf(fid,'%s/%s median %.6f P95 %.6f max %.6f ms; status=%s; max<=2ms=%d\n', ...
                item.systemMode,item.cover,item.medianMs,item.p95Ms, ...
                item.maxMs,item.status,item.allUnder2Ms);
            fprintf(fid,'   synthetic-provider selected tick: median %.6f P95 %.6f MAX %.6f ms\n', ...
                item.syntheticProviderTickMedianMs,item.syntheticProviderTickP95Ms, ...
                item.syntheticProviderTickMaxMs);
        end
    end
end
fprintf(fid,'\nKnown-obstacle cache rebuild, sphere/ellipsoid, rows median P95 max: \n');
for i=1:3
    fprintf(fid,'%.6f %.6f\n',R.benchmark.preparationMedianP95MaxMs(i,:));
end
fprintf(fid,'\nPROVISIONAL: input HLC state bounds absent; no Flight/CBF/planner time in tests.\n');
clear cleanupFile;
zipPath=fullfile(outDir,[stem '.zip']);
zip(zipPath,{[stem '.mat'],[stem '.txt']},outDir);
fprintf('\nSaved: %s\n',zipPath);
fprintf('%s\n',R.status);
end
