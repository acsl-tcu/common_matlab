function R=RUN_PURE_TRIGGER_V7_SELECTED_AUDIT()
%RUN_PURE_TRIGGER_V7_SELECTED_AUDIT V7 candidate audit and save unique ZIP.
% Run from TRIGGER_LIGHT after addpath(genpath(common_matlab)).
audit=TEST_TRIGGER_V7_1_AUDIT_CONTRACT();
R=TEST_TRIGGER_V7_PV_AND_SELECTED_GEOMETRY();
R.baselineProjectionContract=audit;
outdir=fullfile(fileparts(mfilename('fullpath')),'TESTS','RESULTS');
if ~isfolder(outdir),mkdir(outdir);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
base=['PURE_TRIGGER_V7_SELECTED_AUDIT_' stamp];
matfile=fullfile(outdir,[base '.mat']);
txtfile=fullfile(outdir,[base '.txt']);
zipfile=fullfile(outdir,[base '.zip']);
save(matfile,'R','-v7');
fid=fopen(txtfile,'wt');
if fid<0,error('V7:Save','Cannot write audit TXT');end
fprintf(fid,'V7.1 status %s\n',R.status);
fprintf(fid,'Baseline UNKNOWN count %d; nominal mismatches %d\n', ...
    R.nominalBaselineUnknownCount,R.nominalClassificationDisagreements);
fprintf(fid,'Measured times only: 40 repeats per scenario. 2ms median fraction %.4f, P95 fraction %.4f\n', ...
    R.total2msMeasuredMedPassFraction,R.total2msMeasuredP95PassFraction);
fprintf(fid,'Maximum test-only source projection %.12g m (NOT physical uncertainty)\n', ...
    R.maxBaselineProjection_m);
fprintf(fid,'4s forecast comparison, each chosen geometry executed SEPARATELY\n');
for i=1:numel(R.forecastRecords)
 r=R.forecastRecords(i);
 fprintf(fid,'%s t0 %.1f LoadError_m %.7g DroneError_m %.7g PVmedianMs %.6g\n', ...
     r.caseName,r.startTime_s,r.maxLoadError_m,r.maxDroneError_m,r.forecastMedianMs);
end
for i=1:numel(R.geometryRecords)
 r=R.geometryRecords(i);
 fprintf(fid,'%s t0 %.1f %s x %s %s pred %s baseline %s agree %d medianMs %.6g P95ms %.6g maxMs %.6g\n', ...
    r.caseName,r.startTime_s,r.mode,r.cover,r.scenario, ...
    r.nominalPredictorStatus,r.originalBaselineStatus,r.classificationAgrees, ...
    r.medianTotalMs,r.p95TotalMs,r.maxTotalMs);
 fprintf(fid,'  forecastMedian %.6g geometryMedian %.6g baselineReason %s projectionMax_m %.12g N=%d\n', ...
     r.medianForecastMs,r.medianGeometryMs,r.baselineReason, ...
     r.baselineProjectionMax_m,r.timingSamples);
end
fprintf(fid,'All geometric classifications refer to NOMINAL piecewise-linear knot paths only.\n');
fprintf(fid,'Physical prediction errors, EKF covariance, input saturation and real Flight not certified.\n');
fprintf(fid,'Zero rho/acc is used ONLY for nominal polyline comparisons, NEVER certified real-safety clear.\n');
fclose(fid);
zip(zipfile,{matfile,txtfile});
R.resultsZip=zipfile;
fprintf('V7.1 saved: %s\n',zipfile);
end
