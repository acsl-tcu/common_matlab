function AF7R11_BACKEND1_REASON18_TRAP()
% AF7R11_BACKEND1_REASON18_TRAP
% =========================================================================
% Diagnostic only. Does NOT modify REPLANNING_SYSTEM.m.
%
% Dynamically locates the line
%   OUT.reason_code=18; % no continuously certified rejoin
% in the CURRENT REPLANNING_SYSTEM.m, arms a breakpoint there, and starts
% the unchanged headless run.
%
% When MATLAB stops at K>>, run:
%   AF7R11_REASON18_LOCALS
%
% Then send the printed output.
% =========================================================================

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R11] REPLANNING_SYSTEM.m not found.');

txt=fileread(src);
needle='OUT.reason_code=18; % no continuously certified rejoin';
p=strfind(txt,needle);
assert(numel(p)==1,'[AF7R11] expected exactly one reason-18 source anchor.');

lineNo=1+sum(txt(1:p(1))==newline);

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R11 | BACKEND-1 REASON-18 BREAKPOINT TRAP\n');
fprintf('====================================================================================================\n');
fprintf('[DIAGNOSTIC ONLY] production modification = 0\n');
fprintf(' source                                  = %s\n',src);
fprintf(' reason-18 source line                   = %d\n',lineNo);
fprintf(' breakpoint target                       = NO_CONTINUOUSLY_CERTIFIED_REJOIN\n');
fprintf('====================================================================================================\n\n');

dbclear all
eval(sprintf('dbstop in REPLANNING_SYSTEM at %d',lineNo));

fprintf(['Breakpoint armed. Starting unchanged run_flight_headless.\n' ...
         'When K>> appears at reason 18, run:\n\n' ...
         '    AF7R11_REASON18_LOCALS\n\n']);

run_flight_headless
end
