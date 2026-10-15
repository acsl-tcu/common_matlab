function AF7R14_REASON111_GEOMETRY_TRAP()
% AF7R14_REASON111_GEOMETRY_TRAP
% Diagnostic only. Does NOT modify REPLANNING_SYSTEM.m.
%
% Stops at reason 111 (static separation open) in the CURRENT source.
% At K>>, run:
%   AF7R14_REASON111_LOCALS

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R14] REPLANNING_SYSTEM.m not found.');
txt=fileread(src);
needle='G.reason_code=111;';
p=strfind(txt,needle);
assert(numel(p)==1,'[AF7R14] expected exactly one reason-111 source anchor.');
lineNo=1+sum(txt(1:p(1))==newline);

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R14 | REASON-111 STATIC-GEOMETRY TRAP\n');
fprintf('====================================================================================================\n');
fprintf('[DIAGNOSTIC ONLY] production modification = 0\n');
fprintf(' source                                  = %s\n',src);
fprintf(' reason-111 line                         = %d\n',lineNo);
fprintf('====================================================================================================\n\n');

dbclear all
eval(sprintf('dbstop in REPLANNING_SYSTEM at %d',lineNo));

fprintf(['Breakpoint armed. Starting unchanged run_flight_headless.\n' ...
         'At K>>, run:\n\n' ...
         '    AF7R14_REASON111_LOCALS\n\n']);

run_flight_headless
end
