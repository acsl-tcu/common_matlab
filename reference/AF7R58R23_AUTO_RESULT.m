function T = AF7R58R23_AUTO_RESULT()
% Workspace-independent result retrieval after simulation.
folder=fileparts(which('AF7R58R23_AUTO_RESULT'));
p=fullfile(folder,'AF7R58R23_SPIKE_REPORT.mat');
assert(isfile(p),['[R23] Spike report absent. Run ' ...
    'AF7R58R23_INSTALL_SPIKE_PROBE once and rerun simulation.']);
S=load(p,'T');
assert(isfield(S,'T') && isfield(S.T,'schema') && ...
    strcmp(S.T.schema,'AF7R58R23_FULL_LOOP_DIAGNOSTIC_V1'), ...
    '[R23] Profile schema mismatch.');
T=S.T;
T.top_spike_time_s=T.time_s(T.top_indices(1));
T.top_spike_values_ms=T.values_ms(T.top_indices(1),:);
fprintf('========== R23 COMPLETE SPIKE BREAKDOWN ==========\n');
fprintf('Largest full-loop spike at t=%.6f s\n',T.top_spike_time_s);
for i=1:numel(T.labels)
    fprintf('%-24s %.6f ms\n',T.labels{i},T.top_spike_values_ms(i));
end
fprintf('Full cycle trace: %s\n',p);
end
