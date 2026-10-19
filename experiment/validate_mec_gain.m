function report = validate_mec_gain(gain_file)
% 非線形シミュレーションで補償OFFと比較する。元のモードファイルは書き換えない。
% 円軌道、および学習に使わないシード11/12のランダム軌道で確認する。
repo_root = fileparts(fileparts(mfilename('fullpath')));
original_dir = pwd;
original_path = path;
work_dir = tempname;
mkdir(work_dir); mkdir(fullfile(work_dir,'Data','Sim_data'));
cleanup = onCleanup(@() restore_environment(original_dir,original_path,work_dir)); %#ok<NASGU>
% run_auto_trialsは毎回モードを再構築するため、検証条件を適用する短いモードを用意する。
mode_file = fullfile(work_dir,'SimMEC_gain_validation.m');
fid = fopen(mode_file,'w');
assert(fid>=0,'MEC:ValidationFile','検証用モードを作成できません。');
fprintf(fid,'%s\n', ...
    'run(fullfile(Setting.validation_repo,''mode'',''MEC_Koopman'',''SimMEC_K.m''));', ...
    'logger.display_on = false;', ...
    'if Setting.validation_mismatch', ...
    '    agent.plant.param(6:7) = 0.185;', ...
    '    agent.plant.external_force_world = @(t,x) [0;0;0.2*sin(2*pi*0.7*t)*double(t>=7 && t<47)];', ...
    'else', ...
    '    agent.plant.param = agent.controller.mec.param.P;', ...
    '    agent.plant.external_force_world = [];', ...
    'end');
fclose(fid);
addpath(work_dir); cd(work_dir);
names = {'circle_nominal','circle_mismatch_force','spline11_mismatch_force','spline12_nominal'};
random = [false,false,true,true];
mismatch = [false,true,true,false];
seeds = [1,1,11,12];
report = struct([]);
for case_index = 1:numel(names)
    for feedback = [false,true]
        cfg = mec_dataset_settings();
        cfg.enabled = random(case_index);
        cfg.use_compensation = feedback;
        cfg.output_dir = fullfile(work_dir,'datasets');
        st = struct('fExp',0,'fDebug',1,'PInterval',0.6,'mode',"SimMEC_gain_validation", ...
            'mec_dataset',cfg,'trajectory_seeds',seeds(case_index), ...
            'mec_use_compensation',feedback,'mec_gain_file',gain_file, ...
            'validation_repo',repo_root,'validation_mismatch',mismatch(case_index));
        app = SimExp(st);
        app_cleanup = onCleanup(@() close_app(app));
        app.run_auto_trials(1,string(sprintf('validation_%s_%d',names{case_index},feedback)));
        log = app.logger;
        results = log.Data.agent(1).controller.result;
        indices = find(log.Data.phase(1:log.k)=='f');
        E = cell2mat(cellfun(@(r)r.e(:),results(indices),'UniformOutput',false));
        U = cell2mat(cellfun(@(r)r.input(:),results(indices),'UniformOutput',false));
        C = cell2mat(cellfun(@(r)r.delta_u_mec(:),results(indices),'UniformOutput',false));
        X = cell2mat(cellfun(@(r)r.x_estimated(:),results(indices),'UniformOutput',false));
        references = log.Data.agent(1).reference.result(indices);
        desired = cell2mat(cellfun(@(r)r.state.xd(1:3),references,'UniformOutput',false));
        position_error = X(1:3,:)-desired;
        metrics = struct('end_time',app.time.t,'input_peak',max(abs(U),[],2), ...
            'compensation_peak',max(abs(C),[],2),'position_rmse',sqrt(mean(sum(position_error.^2,1))), ...
            'error_rms',sqrt(mean(E.^2,2)));
        metrics.completed = app.time.t>=58.9 && all(isfinite([X(:);E(:);U(:)]));
        report(case_index).name = names{case_index};
        if feedback
            report(case_index).feedback = metrics;
        else
            report(case_index).baseline = metrics;
        end
        fprintf('VALIDATION %s feedback=%d t=%.3f positionRMSE=%.6g\n', ...
            names{case_index},feedback,app.time.t,metrics.position_rmse);
        clear app_cleanup;
    end
    report(case_index).passed = report(case_index).feedback.completed && ...
        report(case_index).baseline.completed && report(case_index).feedback.position_rmse ...
        <=report(case_index).baseline.position_rmse ...
        +max(1e-4,0.25*report(case_index).baseline.position_rmse);
end
assert(all([report.passed]),'MEC:NonlinearValidation', ...
    '完走できない、または位置RMSEが基準より25%%以上悪化する条件があります。');
fprintf('MEC_NONLINEAR_VALIDATION_PASSED\n');
end

function close_app(app)
if isvalid(app), delete(app); end
end

function restore_environment(original_dir,original_path,work_dir)
cd(original_dir); path(original_path);
if isfolder(work_dir), rmdir(work_dir,'s'); end
end
