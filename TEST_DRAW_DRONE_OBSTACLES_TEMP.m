% TEST_DRAW_DRONE_OBSTACLES_TEMP.m -- place in commonmatlab root, delete after PASS.
% This tests drawing only. The previous 132 geometry tests are not repeated.
clearvars;close all;clc;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'environment','DRONE_OBSTACLES')));
addpath(genpath(fullfile(root,'plot')));

opts=struct('subdivisions',[1 1 1],'superSubdivisions',[1 1 1]);
W=DRONE_OBSTACLES('scene_showcase_13','multiple_ellipsoids','ground_z0',opts);
f=figure('Visible','off');a=axes('Parent',f);
d=DRAW_DRONE_OBSTACLES(W,'ax',a,'fitToObstacles',true);
assert(numel(d.originalGroups)==13 && numel(d.approxGroups)==13);
assert(all(isgraphics(d.originalGroups)) && all(isgraphics(d.approxGroups)));
assert(isgraphics(d.groundSurface));
assert(~isempty(findall(a,'Type','surface')) || ...
       ~isempty(findall(a,'Type','patch')),'No 3D surfaces were generated');
close(f);
f=plot_obstacles_4views(W,'time',0);
assert(numel(findall(f,'Type','axes'))==4,'Expected 3D, XY, XZ and YZ');
close(f);
fprintf('PASS static: 13 original shapes + approximations + ground + four views\n');

WD=DRONE_OBSTACLES('scene_dynamic_independent','multiple_ellipsoids','ground_z0',opts);
f=figure('Visible','off');a=axes('Parent',f);
d=DRAW_DRONE_OBSTACLES(WD,'ax',a,'motionRange',[0 10]);
d.update(0); h0=d.originalGroups(1).Matrix;
d.update(1.7); h1=d.originalGroups(1).Matrix;
assert(norm(h0-h1,'fro')>1e-5,'Dynamic object did not move');
assert(all(isgraphics(d.approxGroups)),'Approximation transforms missing');
assert(norm(d.originalGroups(1).Matrix-d.approxGroups(1).Matrix,'fro')<1e-11, ...
       'Original and approximation no longer share the same rigid transform');
close(f);
fprintf('PASS dynamic: independent motions and synchronized approximation transform\n');
fprintf('ALL DRAW TESTS PASS. Temporary file may be deleted.\n');
