% TEST_DRONE_OBSTACLES_DYNAMIC_DRAW_TEMP.m
% commonmatlab直下に配置。MATLAB実行後、合格したら削除してよい。
clearvars; close all; clc;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'environment','DRONE_OBSTACLES')));
addpath(fullfile(root,'plot'));
methods={'single_sphere','single_ellipsoid','multiple_spheres', ...
 'multiple_ellipsoids','superellipsoid','convex_polyhedra'};

%% (1) 修正した三角柱メッシュと13形状の生成確認
objects=scene_showcase_13();
assert(numel(objects)==13,'Expected 13 obstacle types');
assert(strcmp(objects(13).type,'triangular_prism_mesh'));
C=obs_components(objects(13),struct());
assert(numel(C)==2,'Expected two triangular prism cells');
fprintf('PASS: triangular prism mesh parser/definition\n');

%% (2) 動的障害物の独立軌道
W=DRONE_OBSTACLES('scene_dynamic_independent', ...
 'multiple_ellipsoids','ground_z0',struct('subdivisions',[1 1 1]));
assert(numel(W.objects)==3);
assert(all([W.objects.isDynamic]));
A=obs_world_at_time(W,0);
B=obs_world_at_time(W,1.7);
assert(norm(A.objects(1).position-B.objects(1).position)>0.1);
assert(norm(A.objects(2).position-B.objects(2).position)>0.1);
assert(norm(A.objects(3).position-B.objects(3).position)>0.1);
for i=1:3
 xyz=B.objects(i).position;
 assert(obs_original_contains(B.objects(i),xyz,W.options));
 assert(any(obs_world_contains(B.parts,xyz)), ...
  'Dynamic part does not enclose center of %s',B.objects(i).name);
 assert(norm(B.objects(i).R'*B.objects(i).R-eye(3),'fro')<1e-10);
end
fprintf('PASS: three independent dynamic obstacle trajectories\n');

%% (3) 6近似方式の時間更新後の中心包含（簡易検査）
for k=1:numel(methods)
 S=DRONE_OBSTACLES('scene_dynamic_independent',methods{k}, ...
  'ground_z0',struct('subdivisions',[1 1 1],'superSubdivisions',[1 1 1]));
 St=obs_world_at_time(S,2.0);
 for i=1:numel(St.objects)
  pname=St.objects(i).name;
  pr=St.parts(strcmp({St.parts.obstacleName},pname));
  assert(all(obs_world_contains(pr,St.objects(i).position)), ...
     'approximation %s does not include moved center of %s',methods{k},pname);
 end
end
fprintf('PASS: six dynamic approximation transforms\n');

%% (4) 描画：元形状・近似、3D/XY/XZ/YZ、動的更新
% -nodisplay 環境で描画できないときも幾何テストと分けて確認。
fig=figure('Visible','off');ax=axes('Parent',fig);
drawObj=DRAW_DRONE_OBSTACLES(W,'ax',ax);
drawObj.update(2.0);
assert(isgraphics(drawObj.originalGroups(1)));
assert(isgraphics(drawObj.approxGroups(1)));
close(fig);
fig=plot_obstacles_4views(W,'time',1.7);
assert(numel(findall(fig,'Type','axes'))>=4);
close(fig);
fprintf('PASS: static 4 views and dynamic graphics update\n');

fprintf('ALL PASS: test may be removed after checking the MATLAB environment.\n');
