function OUT = AF7R58R47_RUN_READONLY(terminal_source,towing_log_source,r23_timing_report)
% Three independent and read-only checks; no implicit physical defaults.
% Use [] when a required source is unavailable. Such a track remains OPEN,
% NEVER a PASS. Pass exact data already logged by the live user simulation.
OUT=struct();
OUT.schema='AF7R58R47_THREE_TRACKS_READONLY_V1';
OUT.production_modified=false;
OUT.return=struct('status','NOT_RUN__SOURCE_NOT_SUPPLIED');
OUT.towing=struct('status','NOT_RUN__SOURCE_NOT_SUPPLIED');
OUT.timing=struct('status','NOT_RUN__SOURCE_NOT_SUPPLIED');
if ~isempty(terminal_source)
    OUT.return=AF7R58R47_RETURN_AUDIT(terminal_source);
end
if ~isempty(towing_log_source)
    OUT.towing=AF7R58R47_TOWING_LOG_AUDIT(towing_log_source);
end
if ~isempty(r23_timing_report)
    OUT.timing=AF7R58R47_TIMING_SOURCE_SPLIT(r23_timing_report);
end
OUT.production_reference_certified=false;
OUT.closed_loop_continuous_obstacle_certified=false;
OUT.control_period_25ms_certified=false;
fprintf('\n[R47] Three-track READ-ONLY evidence generated; no production PASS/COMMIT.\n');
end
