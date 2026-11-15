function motion = obs_motion_waypoints(times,positionsXYZ,orientationsRPYDeg,loop)
% 独立したキーフレーム軌道。位置/姿勢は pchip 補間 (C1)。
% 高相対次数CBF用の高階導関数は、将来別途定義が必要。
if nargin<3 || isempty(orientationsRPYDeg),orientationsRPYDeg=zeros(size(positionsXYZ));end
if nargin<4,loop=false;end
times=times(:)';
assert(numel(times)>=2 && all(diff(times)>0) && times(1)==0, ...
    'times must be strictly increasing and start at 0');
assert(isequal(size(positionsXYZ),[numel(times),3]),'positions must be Nx3');
assert(isequal(size(orientationsRPYDeg),[numel(times),3]),'RPY must be Nx3');
assert(all(isfinite([times(:);positionsXYZ(:);orientationsRPYDeg(:)])), ...
    'motion keys must be finite');
if loop
 assert(max(abs(positionsXYZ(1,:)-positionsXYZ(end,:)))<1e-8, ...
     'loop requires identical first/last positions');
 assert(max(abs(orientationsRPYDeg(1,:)-orientationsRPYDeg(end,:)))<1e-8, ...
     'loop requires identical first/last RPY (unwrapped)');
end
motion=struct('type','waypoints','times',times, ...
    'positions',positionsXYZ,'rpy',orientationsRPYDeg,'loop',logical(loop));
end
