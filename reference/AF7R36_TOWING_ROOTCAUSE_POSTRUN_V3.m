function S = AF7R36_TOWING_ROOTCAUSE_POSTRUN_V3(varargin)
% AF7R36_TOWING_ROOTCAUSE_POSTRUN_V3
% Read the scalar diagnostic capture left by V3 after the simulation.
%
% Preferred:
%   S = AF7R36_TOWING_ROOTCAUSE_POSTRUN_V3;
%
% Explicit fallback:
%   S = AF7R36_TOWING_ROOTCAUSE_POSTRUN_V3(agent);
%   S = AF7R36_TOWING_ROOTCAUSE_POSTRUN_V3(replanner);

replanner=[];
if nargin>=1 && ~isempty(varargin{1})
    replanner=resolve_replanner(varargin{1});
end
if isempty(replanner)
    names={'replanner','agent','self'};
    for k=1:numel(names)
        try
            if evalin('base',sprintf('exist(''%s'',''var'')',names{k}))
                v=evalin('base',names{k});
                replanner=resolve_replanner(v);
                if ~isempty(replanner), break; end
            end
        catch
        end
    end
end

assert(~isempty(replanner), ...
    ['[TOWDIAG-V3] Could not resolve the REPLANNING_SYSTEM instance. ' ...
     'Call S=AF7R36_TOWING_ROOTCAUSE_POSTRUN_V3(agent) or pass the replanner directly.']);

D=replanner.generator_last_status;
assert(isstruct(D) && isscalar(D) && isfield(D,'capture_revision'), ...
    ['[TOWDIAG-V3] No real runtime diagnostic capture is present. ' ...
     'Confirm V3 install, clear classes, and rerun the simulation.']);
assert(strcmp(char(D.capture_revision),'AF7_TOWING_DIAG_CAPTURE_V3'), ...
    '[TOWDIAG-V3] Unexpected capture revision: %s',char(D.capture_revision));

S=AF7R36_TOWING_ROOTCAUSE_PRINT(D);
S.capture_revision=char(D.capture_revision);
end

function R=resolve_replanner(v)
R=[];
try
    if isa(v,'REPLANNING_SYSTEM')
        R=v; return;
    end
catch
end

% Common agent.reference.timevarying binding used by the current flight code.
try
    q=v.reference.timevarying;
    if isa(q,'REPLANNING_SYSTEM'), R=q; return; end
catch
end
try
    q=v.reference.replanning;
    if isa(q,'REPLANNING_SYSTEM'), R=q; return; end
catch
end
try
    q=v.replanner;
    if isa(q,'REPLANNING_SYSTEM'), R=q; return; end
catch
end
end
