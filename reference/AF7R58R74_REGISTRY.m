function out=AF7R58R74_REGISTRY(action,varargin)
% R73: process-local persistent live Replanner handle for POSTFLIGHT use.
% Registered only on construction and trigger version changes (NOT per cycle).
% mlock protects state against runner `clear all`; no Flight disk I/O.
% `release` explicitly discards the retained live instance after inspection.
persistent liveHandle registrationCount captureCount sourcePath
mlock;
if isempty(registrationCount), registrationCount=0;end
if isempty(captureCount), captureCount=0;end
switch lower(char(string(action)))
    case 'register'
        assert(nargin==2 && isa(varargin{1},'REPLANNING_SYSTEM') && ...
            isvalid(varargin{1}), '[R72] invalid replanner registration');
        liveHandle=varargin{1};
        sourcePath=which('REPLANNING_SYSTEM');
        registrationCount=registrationCount+1;
        captureCount=0;
        out=AF7R58R74_REGISTRY_STATUS_LOCAL(liveHandle,registrationCount,captureCount,sourcePath);
    case 'capture'
        assert(nargin==2 && isa(varargin{1},'REPLANNING_SYSTEM') && ...
            isvalid(varargin{1}), '[R72] invalid capture registration');
        if isempty(liveHandle) || ~isvalid(liveHandle) || ...
                ~isequal(liveHandle,varargin{1})
            liveHandle=varargin{1};
            sourcePath=which('REPLANNING_SYSTEM');
            registrationCount=registrationCount+1;
            captureCount=0;
        end
        captureCount=captureCount+1;
        out=AF7R58R74_REGISTRY_STATUS_LOCAL(liveHandle,registrationCount,captureCount,sourcePath);
    case 'get'
        assert(~isempty(liveHandle) && isa(liveHandle,'REPLANNING_SYSTEM') && ...
            isvalid(liveHandle), '[R72] persistent Replanner registry is empty');
        out=liveHandle;
    case {'status','peek'}
        out=AF7R58R74_REGISTRY_STATUS_LOCAL(liveHandle,registrationCount,captureCount,sourcePath);
    case 'release'
        liveHandle=[];
        out=AF7R58R74_REGISTRY_STATUS_LOCAL(liveHandle,registrationCount,captureCount,sourcePath);
    otherwise
        error('[R72] unsupported registry action');
end
end
function S=AF7R58R74_REGISTRY_STATUS_LOCAL(h,n,c,src)
valid=false;eventCount=0;version=NaN;
if ~isempty(h)
    try
        valid=isa(h,'REPLANNING_SYSTEM')&&isvalid(h);
        if valid
            eventCount=numel(h.r71r2_event_snapshots);
            version=double(h.active_reference_version);
        end
    catch
        valid=false;
    end
end
S=struct('registered',logical(valid),'registration_count',double(n), ...
    'capture_count',double(c),'snapshot_count',double(eventCount), ...
    'last_reference_version',version,'source',char(string(src)));
end
