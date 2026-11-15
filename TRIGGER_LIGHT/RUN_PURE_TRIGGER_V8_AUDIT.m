function R=RUN_PURE_TRIGGER_V8_AUDIT(repeats,runMex)
%RUN_PURE_TRIGGER_V8_AUDIT V8 full V7 parity, current-EKF simulated tick,
% comparative timing, and optional MATLAB Coder C-chain MEX test.
if nargin<1,repeats=30;end
if nargin<2,runMex=false;end
R=TEST_TRIGGER_V8_FASTPATH_AND_STREAM(repeats);
if runMex
    R.mex=BUILD_AND_BENCH_V8_MEX();
else
    R.mex=struct('status','NOT_REQUESTED', ...
        'reason','Run separately after MATLAB version passes');
end
outdir=fullfile(fileparts(mfilename('fullpath')),'TESTS','RESULTS');
if ~isfolder(outdir),mkdir(outdir);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
base=['PURE_TRIGGER_V8_SELECTED_AUDIT_' stamp];
matfile=fullfile(outdir,[base '.mat']);
txtfile=fullfile(outdir,[base '.txt']);
zipfile=fullfile(outdir,[base '.zip']);
save(matfile,'R','-v7');
fid=fopen(txtfile,'wt');
if fid<0,error('V8:File','Unable to write result TXT');end
fprintf(fid,'V8 status %s\n',R.status);
fprintf(fid,'96 separate nominal matches %d/%d; proven-geometry refinements %d\n', ...
    nnz([R.records.parity]),numel(R.records),R.validClearRefinements);
fprintf(fid,'Median<=2ms %.4f; P95<=2ms %.4f; max %.6g ms\n', ...
    R.median2msPassFraction,R.p95_2msPassFraction,R.maxTotalMs);
fprintf(fid,'EKF stream: %s, mL %g kg, ground-truth parameter %g (untouched)\n', ...
    R.stream.status,R.stream.latestMassAtTick,R.stream.parameterLoadMassAfter);
for i=1:numel(R.records)
    r=R.records(i);
    fprintf(fid,'%s t0=%.1f %s x %s %s match=%d ', ...
        r.caseName,r.startTime_s,r.mode,r.cover,r.scenario,r.parity);
    fprintf(fid,'V7 %.6fms V8 %.6fms P95 %.6fms max %.6fms ', ...
        r.medianV7Ms,r.medianV8Ms,r.P95V8Ms,r.maxV8Ms);
    fprintf(fid,'forecast %.6fms geometry %.6fms extraCclear %d\n', ...
        r.medianForecastV8Ms,r.medianGeometryV8Ms,r.v8CPrecleared);
end
fprintf(fid,'MEX status %s\n',R.mex.status);
fprintf(fid,'All results nominal only. Zero bound in polyline geometry is NOT physical guarantee.\n');
fprintf(fid,'Actual flight EKF and 25ms full Flight loop not tested.\n');
fclose(fid);
zip(zipfile,{matfile,txtfile});
R.resultsZip=zipfile;
fprintf('V8 saved: %s\n',zipfile);
end
