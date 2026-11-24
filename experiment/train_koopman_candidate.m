function output = train_koopman_candidate(candidateName, observableName)
% Train one Koopman predictor using the fixed datasets and preprocessing.
arguments
    candidateName (1,1) string
    observableName (1,1) string
end
root = fileparts(fileparts(mfilename('fullpath')));
cd(root); addpath(genpath(root));
files = dir(fullfile('datasets','*.mat'));
assert(~isempty(files),'No training files found in datasets.');
F = str2func(observableName);
Data = struct('X',[],'Y',[],'U',[]);
samplePeriods = [];
for fileIndex = 1:numel(files)
    d = ImportFromExpData_tutorial(fullfile(files(fileIndex).folder,files(fileIndex).name),0,3,0,0,0,2);
    assert(isequal(size(d.X),size(d.Y)) && size(d.X,2)==size(d.U,2));
    Data.X = [Data.X d.X]; Data.Y = [Data.Y d.Y]; Data.U = [Data.U d.U];
    samplePeriods = [samplePeriods diff(d.T)];
end
est = KL(Data.X,Data.U,Data.Y,F,struct('weight',0));
est.observable = F;
est.observable_name = observableName;
est.normalized = false;
est.dt = median(samplePeriods);
assert(abs(est.dt-0.025)<1e-9,'Expected 0.025 s sample period.');
est.training_files = string({files.name});
est.position_mode = 2;
est.sample_count = size(Data.X,2);
outDir = fullfile(root,'Koopman_Linearization','Koopman_Linear_by_Data','refult_of_KL_Miya');
if ~exist(outDir,'dir'), mkdir(outDir); end
output = fullfile(outDir, candidateName + ".mat");
save(output,'est');
fprintf('TRAINED %s: observables=%d samples=%d file=%s\n',candidateName,size(est.A,1),est.sample_count,output);
end
