function objects=scene_empty()
% No obstacles; ground still loaded separately by DRONE_OBSTACLES.
objects=struct('name',{},'type',{},'position',{},'rpyDeg',{},'R',{},'params',{},'approxMethod',{},'isDynamic',{},'motion',{});
end
