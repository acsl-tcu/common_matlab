function Q = AF7R58R32_LOGGER_PLOTS(logger, runtime_profile)
% R32: plot actual, logged, number-free scalar fields.
% Read-only. No changes to reference, safety gates, or controller.
assert(isa(logger,'LOGGER'),'First input must be the live LOGGER instance.');
figure(30); clf;
logger.plot({1,"reference.result.state.rtime_e_to_e_ms",""},"phase","f");
title('Replanner do() time (inside reference)'); ylabel('Time [ms]'); grid on;
figure(31); clf;
logger.plot({{1,"reference.result.state.rtime_stage_start_ms",""}, ...
             {1,"reference.result.state.rtime_stage_advance_ms",""}}, ...
            "phase","f");
title('Stage start and advance'); ylabel('Time [ms]'); grid on;
figure(32); clf;
logger.plot({1,"reference.result.state.rtime_correction_norm_m",""},"phase","f");
title('Active minus nominal correction norm'); ylabel('Position [m]'); grid on;
Q=struct('schema','AF7R58R32_LOGGER_PLOTS_V1', ...
         'plotted_from_logger',true,'flight_certified',false);
if nargin>=2 && ~isempty(runtime_profile)
    P=runtime_profile;
    assert(isstruct(P) && isfield(P,'r23_reference_s') && ...
        isfield(P,'phase') && isfield(P,'t'), ...
        'R23 runtime_profile must include phase, time, and r23_reference_s.');
    n=numel(P.t); mask=(P.phase(:)=='f');
    assert(numel(mask)==n && numel(P.r23_reference_s)==n, ...
        'R23 runtime_profile array size mismatch.');
    figure(33); clf;
    plot(P.t(mask),1000*P.r23_reference_s(mask),'LineWidth',1.3);
    grid on; xlabel('Simulation time [s]'); ylabel('Time [ms]');
    title('Full reference including wrapper / adjacent adjustments');
    Q.outer_reference_peak_ms=max(1000*P.r23_reference_s(mask));
end
end
