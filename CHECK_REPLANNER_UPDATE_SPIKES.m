function report=CHECK_REPLANNER_UPDATE_SPIKES(replanner_instance)
% CHECK_REPLANNER_UPDATE_SPIKES  一時診断：初回20回のupdate内部工程を分離。
% 事前にZIP同梱の診断用REPLANNING_SYSTEM.mに一時的に差し替え、
% clear classes; run_flight_headless を実行する。
% Flight終了後に CHECK_REPLANNER_UPDATE_SPIKES(replanner_instance) を実行。
% この診断に使ったREPLANNING_SYSTEM.mは検証後に元の本番版へ戻す。
if nargin~=1 || ~isa(replanner_instance,'REPLANNING_SYSTEM')
    error('CHECK_REPLANNER_UPDATE_SPIKES:Input', ...
        'Flight後のreplanner_instanceを渡してください。');
end
if ~ismethod(replanner_instance,'get_diagnostic_update_timing')
    error('CHECK_REPLANNER_UPDATE_SPIKES:Source', ...
        '診断用REPLANNING_SYSTEM.mが使用されていません。');
end
report=replanner_instance.get_diagnostic_update_timing();
n=size(report.stage_ms,1);
if n<5
    error('CHECK_REPLANNER_UPDATE_SPIKES:Records', ...
        '診断記録が少なすぎます（%d件）。',n);
end
report.stage_sum_ms=sum(report.stage_ms,2);
report.major_stage_index=zeros(n,1);
report.major_stage_name=cell(n,1);
report.major_stage_ms=zeros(n,1);
for i=1:n
    [report.major_stage_ms(i),idx]=max(report.stage_ms(i,:));
    report.major_stage_index(i)=idx;
    report.major_stage_name{i}=report.stage_names{idx};
end
report.status='UPDATE_INTERNAL_STAGE_DIAGNOSTIC_NOT_PROOF';
report.note=['tic/tocを追加した暫定ソースによる分解なので元実装と' ...
    '完全に同じ時間とは限らない。stage_sumとinclusiveは一致しない場合がある。'];

fprintf('\n[REPLANNER UPDATE INTERNAL TIMING] (first %d calls)\n',n);
fprintf('row   t[s]   logged[ms] inclusive[ms]  maxstage[ms]  name\n');
for i=1:n
    fprintf('%3d %7.3f   %9.4f    %10.4f   %10.4f  %s\n', ...
        i,report.time_s(i),report.recorded_update_ms(i), ...
        report.inclusive_update_ms(i),report.major_stage_ms(i), ...
        report.major_stage_name{i});
end
fprintf('\nDetailed timing by stage [ms]:\n');
for i=1:numel(report.stage_names)
    fprintf('  %-22s',report.stage_names{i});
    fprintf(' %8.4f',report.stage_ms(1:min(8,n),i));
    fprintf('\n');
end

base=fileparts(mfilename('fullpath'));
save(fullfile(base,'REPLANNER_UPDATE_SPIKE_AUDIT.mat'),'report');
fid=fopen(fullfile(base,'REPLANNER_UPDATE_SPIKE_AUDIT.txt'),'wt');
if fid<0,error('CHECK_REPLANNER_UPDATE_SPIKES:Write','TXTを保存できません。');end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',report.status);
fprintf(fid,'%s\n',report.note);
fprintf(fid,'first_count=%d\n',n);
fprintf(fid,'row time_s original_logged_ms inclusive_ms max_stage_ms max_stage\n');
for i=1:n
    fprintf(fid,'%d %.8f %.8f %.8f %.8f %s\n',i,report.time_s(i), ...
        report.recorded_update_ms(i),report.inclusive_update_ms(i), ...
        report.major_stage_ms(i),report.major_stage_name{i});
end
fprintf(fid,'STAGES\n');
for i=1:n
    fprintf(fid,'%d %.8f',i,report.time_s(i));
    fprintf(fid,' %.8f',report.stage_ms(i,:));
    fprintf(fid,'\n');
end
fprintf('\n[SAVED] REPLANNER_UPDATE_SPIKE_AUDIT.txt / .mat\n');
end
