function R=TRIGGER_NATIVE_SOURCE_PIN_CACHE(command)
% Flight-independent static source/binary validation cache.
% 'prepare': verifies source and compiled MEX against embedded audit pin;
%            preflight calls it ONCE per Flight, no observations or state.
% 'get':     cheap constant-time hot-path lookup, never file IO.
% No external audit .mat file is needed. Unknown binaries fail closed.
persistent pin
if nargin~=1 || ~(ischar(command)||isstring(command)) || ...
        ~isscalar(string(command))
    error('TRIGGER_NATIVE_SOURCE_PIN_CACHE:Command', ...
        'Expected prepare, get, or reset command.');
end
switch char(command)
    case 'prepare'
        % Always refresh at a Flight boundary. A changed source or MEX
        % invalidates the embedded binary/source pin by SHA-256 check.
        pin=[];
        pin=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(true);
        R=pin;
    case 'get'
        if isempty(pin)
            R=struct('passed',false, ...
                'status','UNKNOWN_NATIVE_SOURCE_PIN_PREFLIGHT_NOT_EXECUTED');
        else
            R=pin;
        end
    case 'reset'
        pin=[];
        R=struct('passed',false,'status','RESET');
    otherwise
        error('TRIGGER_NATIVE_SOURCE_PIN_CACHE:Command', ...
            'Unknown command: %s',char(command));
end
end
