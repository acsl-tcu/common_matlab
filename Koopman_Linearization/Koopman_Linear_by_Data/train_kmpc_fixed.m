% Rebuild the 26-observable model in physical coordinates.
cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));
files = dir(fullfile('datasets','*.mat'));
assert(~isempty(files),'No training files found in datasets.');
Data = struct('X',[],'Y',[],'U',[]);
samplePeriods = [];

for fileIndex = 1:numel(files)
    d = ImportFromExpData_tutorial(fullfile(files(fileIndex).folder,files(fileIndex).name),0,3,0,0,0,2);
    assert(isequal(size(d.X),size(d.Y)));
    Data.X = [Data.X d.X];
    Data.Y = [Data.Y d.Y];
    Data.U = [Data.U d.U];
    samplePeriods = [samplePeriods diff(d.T)];
end

est = KL(Data.X,Data.U,Data.Y,@quaternions_all,struct('weight',0));%ここの@quaternions_allを右クリックして，観測量を変更できる

est.observable = @quaternions_all;
est.normalized = false;
est.dt = median(samplePeriods);
assert(abs(est.dt-0.025)<1e-9,'The simulation requires 0.025 s training data.');
est.training_files = string({files.name});
est.position_mode = 2;
est.sample_count = size(Data.X,2);
output = fullfile('Koopman_Linearization','Koopman_Linear_by_Data','refult_of_KL_Miya','KL_26_MPC_fixed.mat');
save(output,'est');
fprintf('TRAINED samples=%d dt=%g file=%s\n',est.sample_count,est.dt,output);
