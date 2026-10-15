function AF7R16R2_REAL_FLIGHT_REASON20_TRAP()
% AF7R16R2_REAL_FLIGHT_REASON20_TRAP
% Diagnostic only. Does NOT modify REPLANNING_SYSTEM.m.
%
% This version excludes ALL known preflight prime paths by requiring every
% prime-done authority used by REPLANNING_SYSTEM's Flight precondition.
%
% At K>>, run:
%   AF7R16R2_REAL_FLIGHT_REASON20_LOCALS

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R16R2] REPLANNING_SYSTEM.m not found.');
txt=fileread(src);
needle='OUT.reason_code=20;';
p=strfind(txt,needle);
assert(numel(p)==1,'[AF7R16R2] expected exactly one reason-20 source anchor.');
lineNo=1+sum(txt(1:p(1))==newline);

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R16R2 | TRUE FLIGHT REASON-20 CORRIDOR-QP TRAP\n');
fprintf('====================================================================================================\n');
fprintf('[DIAGNOSTIC ONLY] production modification = 0\n');
fprintf(' source                                  = %s\n',src);
fprintf(' reason-20 line                         = %d\n',lineNo);
fprintf(' requires online_prime_done             = 1\n');
fprintf(' requires setup_exact_do_prime_done     = 1\n');
fprintf(' requires generator_integration_done    = 1\n');
fprintf(' requires backend_kernel_prime_done     = 1\n');
fprintf(' requires trigger_event_prime_done      = 1\n');
fprintf(' requires real trigger_event_count       >= 1\n');
fprintf('====================================================================================================\n\n');

dbclear all
cond=[ ...
    'obj.online_prime_done&&' ...
    'obj.setup_exact_do_prime_done&&' ...
    'obj.generator_integration_prime_done&&' ...
    'obj.generator_backend_kernel_prime_done&&' ...
    'obj.trigger_event_capture_prime_done&&' ...
    'obj.trigger_event_count>=1&&' ...
    '~obj.setup_exact_do_prime_active&&' ...
    '~obj.generator_integration_prime_active'];

eval(sprintf('dbstop in REPLANNING_SYSTEM at %d if %s',lineNo,cond));

fprintf(['Breakpoint armed. Starting unchanged run_flight_headless.\n' ...
         'Prime/backend warmup reason-20 calls are excluded.\n' ...
         'At K>>, verify that normal Flight logs have already started, then run:\n\n' ...
         '    AF7R16R2_REAL_FLIGHT_REASON20_LOCALS\n\n']);

run_flight_headless
end
