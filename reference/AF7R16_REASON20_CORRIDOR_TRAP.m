function AF7R16_REASON20_CORRIDOR_TRAP()
% AF7R16_REASON20_CORRIDOR_TRAP
% Diagnostic only. Does NOT modify REPLANNING_SYSTEM.m.
%
% Stops exactly where backend-1 emits proposal reason 20.
% At K>>, run:
%   AF7R16_REASON20_LOCALS

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R16] REPLANNING_SYSTEM.m not found.');
txt=fileread(src);
needle='OUT.reason_code=20;';
p=strfind(txt,needle);
assert(numel(p)==1,'[AF7R16] expected exactly one reason-20 source anchor.');
lineNo=1+sum(txt(1:p(1))==newline);

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R16 | REASON-20 CORRIDOR-QP TRAP\n');
fprintf('====================================================================================================\n');
fprintf('[DIAGNOSTIC ONLY] production modification = 0\n');
fprintf(' source                                  = %s\n',src);
fprintf(' reason-20 line                         = %d\n',lineNo);
fprintf('====================================================================================================\n\n');

dbclear all
eval(sprintf('dbstop in REPLANNING_SYSTEM at %d',lineNo));
fprintf(['Breakpoint armed. Starting unchanged run_flight_headless.\n' ...
         'At K>>, run:\n\n' ...
         '    AF7R16_REASON20_LOCALS\n\n']);
run_flight_headless
end
