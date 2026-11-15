function result=RUN_PURE_TRIGGER_TESTS(repeats,randomCases)
%RUN_PURE_TRIGGER_TESTS One-command regression + speed report.
%   R=RUN_PURE_TRIGGER_TESTS();
%   R=RUN_PURE_TRIGGER_TESTS(1000,500);
% Outputs common_matlab/PURE_TRIGGER_TEST_RESULTS_FOR_CHATGPT.{mat,txt,zip}
% Please send back the result ZIP. Does not modify the actual run/HLC/EKF.
if nargin<1,repeats=200;end
if nargin<2,randomCases=200;end
root=fileparts(mfilename('fullpath'));
outputTxt=fullfile(root,'PURE_TRIGGER_TEST_RESULTS_FOR_CHATGPT.txt');
outputMat=fullfile(root,'PURE_TRIGGER_TEST_RESULTS_FOR_CHATGPT.mat');
outputZip=fullfile(root,'PURE_TRIGGER_TEST_RESULTS_FOR_CHATGPT.zip');
tTotal=tic;
result=struct('status','RUNNING','matlabRelease',version, ...
    'startTime',datestr(now,31),'test',[],'random',[], ...
    'benchmark',[],'error','');
try
    result.test=TEST_PURE_TRIGGER_ALL();
    result.random=TEST_PURE_TRIGGER_RANDOM(randomCases);
    result.benchmark=BENCH_PURE_TRIGGER_1_2MS(repeats);
    result.status='TESTS_COMPLETE_TIME_GOAL_NOT_GUARANTEED';
catch ME
    result.status='FAILED';
    result.error=getReport(ME,'extended','hyperlinks','off');
    fprintf(2,'FAILED: %s\n',result.error);
end
result.totalElapsedSeconds=toc(tTotal);
try
    save(outputMat,'result');
    f=fopen(outputTxt,'w');
    if f<0,error('Cannot write output log');end
    cleaner=onCleanup(@()fclose(f));
    fprintf(f,'PURE PORT - TRIGGER TEST RESULTS\n');
    fprintf(f,'Release: %s\nTime: %s\nStatus: %s\n', ...
        result.matlabRelease,result.startTime,result.status);
    fprintf(f,'Bound-source caveat: no HLC/EKF certified bounds; geometry conditional.\n');
    fprintf(f,'Run duration %.6f s\n',result.totalElapsedSeconds);
    if ~isempty(result.error),fprintf(f,'ERROR:\n%s\n',result.error);end
    if ~isempty(result.benchmark)
        names=fieldnames(result.benchmark);
        for i=1:numel(names)
            v=result.benchmark.(names{i});
            if ~isstruct(v),continue;end
            fprintf(f,'%s: median %.6f ms, p95 %.6f ms, max %.6f ms, status %s, intervals %d, allCalls %d, <=2ms-all %d\n', ...
                names{i},v.medianMs,v.p95Ms,v.maxMs,v.resultStatus, ...
                v.checkedIntervals,v.numMeasuredCalls,v.targetTwoMsMetAllCalls);
        end
    end
    clear cleaner; % close before ZIP
    zip(outputZip,{outputMat,outputTxt});
    fprintf('Saved %s\n',outputZip);
catch EX
    fprintf(2,'Results writing failed: %s\n',EX.message);
end
end
