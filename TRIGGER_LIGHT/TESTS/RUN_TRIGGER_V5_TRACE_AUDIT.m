function X = RUN_TRIGGER_V5_TRACE_AUDIT(matPath)
% Fully standalone audit. Does not run original HLC again; no MATLAB toolbox.
% Does not overwrite an existing results ZIP.
if nargin<1 || ~(ischar(matPath)||isstring(matPath))
    error('V5:Path','Specify exact saved HLC_STRESS_*.mat pathname');
end
raw=load(char(matPath),'R');
X=TEST_TRIGGER_V5_TRACE_FACTS(raw.R);
root=fileparts(mfilename('fullpath'));
outdir=fullfile(root,'RESULTS');
if ~isfolder(outdir),mkdir(outdir);end
stamp=datestr(now,'yyyymmdd_HHMMSS');
name=['V5_TRACE_FACTS_' stamp];
n=1;
while exist(fullfile(outdir,[name '.zip']),'file') || ...
    exist(fullfile(outdir,[name '.csv']),'file')
    n=n+1; name=['V5_TRACE_FACTS_' stamp '_' num2str(n)];
end
csvPath=fullfile(outdir,[name '.csv']);
matOut=fullfile(outdir,[name '.mat']);
zipPath=fullfile(outdir,[name '.zip']);
writetable(struct2table(X),csvPath);
save(matOut,'X');
zip(zipPath,{csvPath,matOut});
fprintf('V5 FACTS ZIP: %s\n',zipPath);
end
