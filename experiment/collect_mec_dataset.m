function [files,app] = collect_mec_dataset(seeds,option)
% 例：files = collect_mec_dataset(1:10);
% 振幅の順番は [推力; rollトルク; pitchトルク; yawトルク]。
arguments
    seeds (1,:) double {mustBeInteger,mustBeNonnegative,mustBeFinite} = 1:10
    option.amplitude (4,1) double {mustBeNonnegative,mustBeFinite} = [0.5;0.008;0.008;0.006]
    option.flight_time (1,1) double {mustBePositive,mustBeFinite} = 40
    option.use_compensation (1,1) logical = false
    option.output_dir (1,1) string = ""
end
assert(~isempty(seeds) && all(seeds<=2^32-1),'MEC:InvalidSeeds','有効なシードを指定してください。');
repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repo_root));
dataset = mec_dataset_settings();
dataset.enabled = true;
dataset.seeds = seeds;
dataset.amplitude = option.amplitude;
dataset.flight_time = option.flight_time;
dataset.ramp_time = min(dataset.ramp_time,dataset.flight_time/2);
dataset.use_compensation = option.use_compensation;
if option.output_dir~=""
    dataset.output_dir = char(option.output_dir);
end
Setting = struct('fExp',0,'fDebug',1,'PInterval',0.6,'mode',"SimMEC_K", ...
    'trajectory_seed',seeds(1),'trajectory_seeds',seeds,'mec_dataset',dataset);
app = SimExp(Setting);
app.run_auto_trials(numel(seeds),"MEC_dataset");
files = app.dataset_files;
fprintf('保存完了：%d / %d 試行\n',numel(files),numel(seeds));
if nargout<2
    delete(app);
end
end
