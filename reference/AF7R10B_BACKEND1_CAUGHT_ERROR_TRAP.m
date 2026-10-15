function AF7R10B_BACKEND1_CAUGHT_ERROR_TRAP()
% AF7R10B_BACKEND1_CAUGHT_ERROR_TRAP
% =========================================================================
% Diagnostic-only launcher. It DOES NOT edit REPLANNING_SYSTEM.m.
%
% Purpose:
%   REPLANNING_SYSTEM currently wraps the backend callback in try/catch and
%   records only status=-1, discarding the original MException. MATLAB's
%   "dbstop if caught error" stops at the ORIGINAL throw site before that
%   catch suppresses it.
%
% Usage:
%   1) Put this file on the MATLAB path.
%   2) Run:
%        AF7R10B_BACKEND1_CAUGHT_ERROR_TRAP
%   3) MATLAB should stop at the exact backend-1 error on the first useful
%      request. Copy:
%        - the red error message
%        - dbstack output
%        - the indicated source line
%      and send them back.
%
% When finished debugging:
%      dbclear if caught error
% =========================================================================

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R10B | BACKEND-1 CAUGHT-ERROR TRAP\n');
fprintf('====================================================================================================\n');
fprintf('[DIAGNOSTIC ONLY] production source modification = 0\n');
fprintf('MATLAB will stop at the original caught exception.\n\n');

assert(exist('run_flight_headless','file')==2, ...
    '[AF7R10B] run_flight_headless.m not found on path.');
assert(exist('REPLANNING_SYSTEM','class')==8 || exist('REPLANNING_SYSTEM','file')==2, ...
    '[AF7R10B] REPLANNING_SYSTEM not found on path.');

dbclear if caught error
dbstop if caught error

fprintf(['Debugger armed: dbstop if caught error\n' ...
         'Starting the unchanged run_flight_headless now.\n' ...
         'When MATLAB stops, run "dbstack" and copy the original error message.\n\n']);

run_flight_headless
end
