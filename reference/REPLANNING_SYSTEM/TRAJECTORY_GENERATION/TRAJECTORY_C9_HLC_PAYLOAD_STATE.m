function [pL,vL,valid] = TRAJECTORY_C9_HLC_PAYLOAD_STATE(x)
%TRAJECTORY_C9_HLC_PAYLOAD_STATE  Decode exactly the current HLC result.x.
% HLC_SUSPENDED_LOAD.do constructs:
% x=[state.getq('compact')(4); w(3); pL(3); vL(3); pT(3); wL(3)].
% Thus x is 19x1, pL=x(8:10), vL=x(11:13).
% This is a source-layout contract, not a kinematic or tracking proof.
% On a changed HLC state layout, FAIL CLOSED: never guess an offset.
pL=[];vL=[];valid=false;
if ~isnumeric(x) || ~isreal(x) || numel(x)~=19 || ...
        ~all(isfinite(x(:)))
    return
end
x=double(x(:));
pL=x(8:10);
vL=x(11:13);
valid=true;
end
