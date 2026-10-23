function R=AF7R58R44R1_POSTRUN_CAPTURE(varargin)
% Compatibility repair of R44R1: no unconditional P.result.state.time access.
% Writes ACTUAL snapshot into the expected R44R1 file location.
if nargin>=1 && ~isempty(varargin{1})
    P=varargin{1};
else
    P=[];
end
if nargin>=2 && ~isempty(varargin{2})
    fn=varargin{2};
else
    fn=fullfile(fileparts(mfilename('fullpath')), ...
        'AF7R58R44R1_ACTIVE_SNAPSHOT.mat');
end
R=AF7R58R44R2_CAPTURE(P,fn);
end
