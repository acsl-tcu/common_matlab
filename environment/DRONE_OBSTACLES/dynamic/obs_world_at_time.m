function snapshot=obs_world_at_time(world,t)
% 予め生成した近似を再計算せず、剛体運動で t 秒の形状へ移す。
% world は DRONE_OBSTACLES() が返した基準状態 (時刻0) を渡す。
% 静的障害物は変化しない。動的障害物のCBF対応はまだ未実装。
if ~isfield(world,'baseObjects'), error('DRONE_OBSTACLES Stage2 world required');end
snapshot=world;
base=world.baseObjects;
parts=world.baseParts;
objects=base;
for i=1:numel(base)
 [c1,R1,rpy1]=obs_pose_at_time(world.sourceObjects(i),t);
 if ~base(i).isDynamic,continue;end
 c0=base(i).position;
 R0=base(i).R;
 D=R1*R0';
 d=c1(:)-D*c0(:);
 objects(i).position=c1;
 objects(i).rpyDeg=rpy1;
 objects(i).R=R1;
 for k=1:numel(parts)
  if ~strcmp(parts(k).obstacleName,base(i).name),continue;end
  p0=world.baseParts(k);
  parts(k).center=(D*p0.center(:)+d)';
  parts(k).R=D*p0.R;
  if strcmp(p0.kind,'convex_polyhedron')
   parts(k).vertices=(D*p0.vertices')'+d';
   parts(k).H=p0.H*D';
   parts(k).b=p0.b+parts(k).H*d;
  end
 end
end
snapshot.objects=objects;
snapshot.parts=parts;
snapshot.time=t;
end
