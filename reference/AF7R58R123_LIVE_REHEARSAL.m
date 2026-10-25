function R=AF7R58R123_LIVE_REHEARSAL(opts)
% Public post-Flight entrypoint. Always attempt to save, including blockers.
if nargin<1,opts=struct();end
R=AF7R58R123_REPLAY_CORE(opts);
D=fileparts(mfilename('fullpath'));
path=fullfile(D,['AF7R58R123_EVENT_' datestr(now,'yyyymmdd_HHMMSS_FFF') '.mat']);
try
    save(path,'R','-v7');
    fprintf('[R123] %s\n',char(R.status));
    fprintf('[R123] t=%.9f version=%.0f | source-match=%d | physical-interval=%d | connector=%d | tail=%d | native-commit=%d\n', ...
        R.event_time_s,R.reference_version,R.source_version_matched, ...
        R.physical_interval_certificate_closed, ...
        R.actual_same_event_route_found,R.return_tail_certified, ...
        R.production_commit_authorized);
    fprintf('[R123] report=%s\n',path);
    if ~isempty(R.error_identifier)
        fprintf('[R123] exception=%s: %s\n',R.error_identifier,R.error_message);
    end
catch ME
    warning('R123:ReportSaveFailure','%s',ME.message);
end
end
