function S = AF7R36_TOWING_ROOTCAUSE_POSTRUN(varargin)
% AF7R36_TOWING_ROOTCAUSE_POSTRUN
% -------------------------------------------------------------------------
% Retrieve the V2-persisted AF7R36 diagnostic result after simulation.
%
% Preferred usage after run_flight_headless:
%   S = AF7R36_TOWING_ROOTCAUSE_POSTRUN;
%
% Also accepted:
%   S = AF7R36_TOWING_ROOTCAUSE_POSTRUN(agent);
%   S = AF7R36_TOWING_ROOTCAUSE_POSTRUN(replanner);

replanner=[];

if nargin>=1 && ~isempty(varargin{1})
    replanner=resolve_replanner(varargin{1});
else
    % Canonical current layout first.
    try
        if evalin('base','exist(''agent'',''var'')')
            a=evalin('base','agent');
            replanner=resolve_replanner(a);
        end
    catch
        replanner=[];
    end

    % Direct replanner variables as fallback.
    if isempty(replanner)
        W=evalin('base','whos');
        for k=1:numel(W)
            try
                v=evalin('base',W(k).name);
                if isa(v,'REPLANNING_SYSTEM')
                    replanner=v;
                    break;
                end
            catch
            end
        end
    end
end

assert(~isempty(replanner) && isa(replanner,'REPLANNING_SYSTEM'), ...
    ['[TOWDIAG-V2] Could not resolve the live REPLANNING_SYSTEM object. ' ...
     'Call AF7R36_TOWING_ROOTCAUSE_POSTRUN(agent) or pass the replanner handle directly.']);

assert(isprop(replanner,'generator_last_status'), ...
    '[TOWDIAG-V2] generator_last_status property unavailable.');
X=replanner.generator_last_status;
assert(isstruct(X) && isscalar(X) && isfield(X,'towing_diag_revision'), ...
    ['[TOWDIAG-V2] No runtime diagnostic capture found. Install V2, run ' ...
     '`clear classes; clear mex; rehash`, then rerun the same simulation.']);

S=AF7R36_TOWING_ROOTCAUSE_PRINT(X);
S.replanner_status_code=double(replanner.generator_last_status_code);
S.generator_request_count=double(replanner.generator_request_count);
S.generator_candidate_count=double(replanner.generator_candidate_count);

fprintf('[POSTRUN]\n');
fprintf(' generator request count                = %d\n',round(S.generator_request_count));
fprintf(' generator candidate count              = %d\n',round(S.generator_candidate_count));
fprintf(' final generator status code            = %.0f\n',S.replanner_status_code);
end

function r=resolve_replanner(x)
r=[];
if isa(x,'REPLANNING_SYSTEM')
    r=x;
    return;
end
try
    if isobject(x) || isstruct(x)
        % Current canonical run_flight_headless registration.
        y=x.reference.timevarying;
        if isa(y,'REPLANNING_SYSTEM')
            r=y;
            return;
        end
    end
catch
end
try
    y=x.reference.flight;
    if isa(y,'REPLANNING_SYSTEM')
        r=y;
    end
catch
end
end
