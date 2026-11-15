function g=ground_z0()
% z=0 is ground boundary; valid side z>=0. Ground is not a finite obstacle.
g=struct('name','ground_z0','kind','halfspace','point',[0 0 0], ...
 'normal',[0 0 1],'forbiddenSide','negative', 'safetyMargin',0);
end
