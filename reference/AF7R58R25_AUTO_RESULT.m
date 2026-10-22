function R = AF7R58R25_AUTO_RESULT()
% Analysis AFTER run_flight_headless finished, not Flight online.
folder=fileparts(which('AF7R58R25_AUTO_RESULT'));
matPath=fullfile(folder,'AF7R58R23_SPIKE_REPORT.mat');
assert(isfile(matPath), '[R25] R23/R25 Flight report not found. Run simulation.');
V=load(matPath,'T','P');
assert(isfield(V,'T') && isfield(V,'P'), '[R25] R23 report schema missing.');
T=V.T; P=V.P;
assert(isfield(T,'labels') && isfield(T,'values_ms'), ...
    '[R25] Incompatible report schema.');
ri=find(strcmp(T.labels,'reference_replanner'));
assert(isscalar(ri),'[R25] Reference timing column absent.');
ref=T.values_ms(:,ri);
[sorted,order]=sort(ref,'descend');
N=min(15,numel(order));
R=struct();
R.schema='AF7R58R25_BUFFERED_LOGGER_DIAG';
R.reference_peak_ms=sorted(1);
R.reference_peak_time_s=T.time_s(order(1));
R.reference_top_time_s=T.time_s(order(1:N));
R.reference_top_ms=sorted(1:N);
R.stage_events=T.stage_events;
R.exact_cycle_count=numel(ref);
R.logger_timings_are_in_controller_result=true;
R.changed_safety_gates=false;
R.commit_attempted=false;
R.saved_r23_profile=P;
out=fullfile(folder,'AF7R58R25_RESULT.mat');
save(out,'R','-v7');
fid=fopen(fullfile(folder,'AF7R58R25_RESULT.txt'),'w','n','UTF-8');
assert(fid>=0,'[R25] Cannot create report TXT');
c=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R25 | buffered R17 events + LOGGER timing | NO COMMIT\n');
fprintf(fid,'Top reference period: %.9f s  %.6f ms\n', ...
    R.reference_peak_time_s,R.reference_peak_ms);
fprintf(fid,'time_s reference_ms\n');
for j=1:N
    fprintf(fid,'%.9f %.6f\n',R.reference_top_time_s(j),R.reference_top_ms(j));
end
fprintf(fid,'R25 buffered events published once after Flight.\n');
fprintf(fid,'Caveat: logger self-time stays in outer runtime_profile (post-call).\n');
fprintf('AF7R58R25 RESULT: %s\n',out);
end
