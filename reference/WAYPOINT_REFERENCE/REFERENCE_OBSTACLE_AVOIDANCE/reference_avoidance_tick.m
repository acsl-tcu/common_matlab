function [result,history]=reference_avoidance_tick( ...
    agent,history,modalCache,routeCache,known,cfg,flightTime)
%REFERENCE_AVOIDANCE_TICK Official selected one-mode 25ms entry.
% Call AFTER actual EKF and original reference updated on this cycle.
% Exactly ONE (C/B/D x sphere/ellipsoid) combination is used.
% MEX auto-rebuild on missing binary is exceptional: it can block for
% seconds! Run prepare_observed before arming. For strict no-build ticks,
% set cfg.autoRebuildMexDuringTick=false (MATLAB fallback remains active).
start=tic;
M=struct('status','NOT_APPLICABLE','available',false, ...
    'rebuilt',false,'buildMs',0,'errorText','');
if strcmp(cfg.systemMode,'C') && isfield(cfg,'useMexCChainPreclear') && ...
        cfg.useMexCChainPreclear
    % Fast path: a native file already compiled/verified in preparation.
    if exist('wp_six_trigger_C_chain_numeric_mex_v8','file')~=3
        allow=true;
        if isfield(cfg,'autoRebuildMexDuringTick')
            allow=logical(cfg.autoRebuildMexDuringTick);
        end
        if allow
            M=reference_avoidance_mex_ensure('runtime');
        else
            M.status='MISSING_BINARY_RUNTIME_BUILD_DISABLED';
        end
        cfg.useMexCChainPreclear=M.available;
    else
        M.status='PRESENT_FROM_PREPARE';M.available=true;
    end
end
[result,history]=wp_six_trigger_tick_v8( ...
    agent,history,modalCache,routeCache,known,cfg,flightTime);
result.mexRuntime=M;
% The inner core measured only forecast+geometry, not any cold MEX build.
result.totalMs=1000*toc(start);
result.deadline25msMet=(result.totalMs<=25);
result.selectedOnly=true;
result.mexRebuildWasInTick=logical(M.rebuilt);
end
