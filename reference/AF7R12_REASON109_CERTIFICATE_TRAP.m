function AF7R12_REASON109_CERTIFICATE_TRAP()
% AF7R12_REASON109_CERTIFICATE_TRAP
% Diagnostic only. Does NOT modify REPLANNING_SYSTEM.m.
%
% Stops at the first resource-exhaustion rejection (reason 109) inside the
% continuous whole-body certificate. Then run:
%
%   AF7R12_REASON109_LOCALS
%
% This is intended to distinguish:
%   (a) actual non-separation,
%   (b) temporal-Lipschitz cover too coarse,
%   (c) nominal-sweep vs repaired-sweep self-consistency gap.

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R12] REPLANNING_SYSTEM.m not found.');
txt=fileread(src);
needle='G.reason_code=109;';
p=strfind(txt,needle);
assert(numel(p)==1,'[AF7R12] expected exactly one reason-109 anchor.');
lineNo=1+sum(txt(1:p(1))==newline);

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R12 | REASON-109 CONTINUOUS-CERTIFICATE TRAP\n');
fprintf('====================================================================================================\n');
fprintf('[DIAGNOSTIC ONLY] production modification = 0\n');
fprintf(' source                                  = %s\n',src);
fprintf(' reason-109 line                         = %d\n',lineNo);
fprintf('====================================================================================================\n\n');

dbclear all
eval(sprintf('dbstop in REPLANNING_SYSTEM at %d',lineNo));
fprintf(['Breakpoint armed. Starting unchanged run_flight_headless.\n' ...
         'At K>>, run:\n\n' ...
         '    AF7R12_REASON109_LOCALS\n\n']);
run_flight_headless
end
