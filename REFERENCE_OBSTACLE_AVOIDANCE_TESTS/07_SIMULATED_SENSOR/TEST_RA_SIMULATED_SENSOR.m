function T = TEST_RA_SIMULATED_SENSOR()
%TEST_RA_SIMULATED_SENSOR Self-contained true-position 20m detection tests.
% No HLC, EKF, CBF, DRONE_OBSTACLES generator, or MEX needed.
% NO unprovided past observations and no hidden safety thresholds.
caseCount=0;
% Test 1: boundary contact and WHOLE OBJECT return, sphere cover.
a=part('sphere',[21 0 0],[1 1 1],eye(3),'A');
a2=part('sphere',[50 0 0],[0.2 0.2 0.2],eye(3),'A');
b=part('sphere',[21.25 0 0],[1 1 1],eye(3),'B');
c=part('sphere',[-19.5 0 0],[0.2 0.2 0.2],eye(3),'C');
w=makeWorld([a a2 b c],{'A','B','C'},'multiple_spheres');
w.ground=struct('kind','halfspace','normal',[0;0;1], ...
    'point',[0;0;0],'forbiddenSide','negative');
w.baseParts=w.parts; % must NOT leak this hidden whole-world field
w.sourceObjects=w.objects;
[out,info]=ra_sim_sensor_detect(w,[0;0;0], ...
    ra_sim_sensor_config('multiple_spheres'));
assert(info.detectedObjectCount==2 && info.returnedPartCount==3);
assert(isequal(info.detectedObjectNames,{'A','C'}));
assert(nnz(strcmp({out.parts.obstacleName},'A'))==2); % a2 at 50m kept
assert(numel(out.objects)==2 && strcmp(out.objects(1).name,'A'));
assert(numel(out.meta)==2 && numel(out.partBudgets)==2);
assert(isequal(out.ground,w.ground));
assert(~isfield(out,'baseParts') && ~isfield(out,'sourceObjects'));
assert(~any(strcmp({out.parts.obstacleName},'B')));
caseCount=caseCount+1;
% Test 2: current snapshot ONLY, no keeping past obstacle IDs.
[out2,info2]=ra_sim_sensor_detect(w,[150;0;0], ...
    ra_sim_sensor_config('multiple_spheres'));
assert(isempty(out2.parts) && info2.detectedObjectCount==0);
caseCount=caseCount+1;
% Test 3: changing radius is fully externally configurable.
[out3,info3]=ra_sim_sensor_detect(w,[0;0;0], ...
    ra_sim_sensor_config('multiple_spheres',21));
assert(info3.detectedObjectCount==3 && numel(out3.parts)==4);
caseCount=caseCount+1;
% Test 4: rotated thin ellipsoid: bounding spheres overlap, true solids do not.
e1=part('ellipsoid',[0 24 0],[6 .1 .1],eye(3),'thin_far');
e2=part('ellipsoid',[25.5 0 0],[6 .2 .2],eye(3),'long_x');
R90=[0 -1 0;1 0 0;0 0 1];
e3=part('ellipsoid',[25.5 0 0],[6 .2 .2],R90,'long_y');
R45=[cos(pi/4) -sin(pi/4) 0;sin(pi/4) cos(pi/4) 0;0 0 1];
e4=part('ellipsoid',[24 0 0],[8 .2 .2],R45,'tilted_hit');
e5=part('ellipsoid',[21 0 0],[1 .5 .2],eye(3),'touch');
e6=part('ellipsoid',[60 0 0],[.2 .2 .2],eye(3),'long_x');
wE=makeWorld([e1 e2 e3 e4 e5 e6], ...
    {'thin_far','long_x','long_y','tilted_hit','touch'}, ...
    'multiple_ellipsoids');
[obsE,infE]=ra_sim_sensor_detect(wE,[0;0;0], ...
    ra_sim_sensor_config('multiple_ellipsoids'));
assert(isequal(sort(infE.detectedObjectNames), ...
    sort({'long_x','tilted_hit','touch'})));
assert(infE.returnedPartCount==4); % includes long_x's 60m part
assert(infE.exactEllipsoidDistancesSolved>0);
assert(~any(strcmp({obsE.parts.obstacleName},'long_y')));
assert(~any(strcmp({obsE.parts.obstacleName},'thin_far')));
caseCount=caseCount+1;
% Test 5: direct numerical distance: axis aligned and rotated, inside.
assert(abs(ra_sim_sensor_ellipsoid_point_distance([25;0;0], ...
    [0;0;0],eye(3),[5;1;1])-20)<1e-9);
assert(abs(ra_sim_sensor_ellipsoid_point_distance([0;25;0], ...
    [0;0;0],R90,[5;1;1])-20)<1e-9);
assert(ra_sim_sensor_ellipsoid_point_distance([0;0;0], ...
    [0;0;0],eye(3),[5;1;1])==0);
caseCount=caseCount+1;
% Test 6: wrong cover, invalid truth, invalid/unsupported orientation error.
assertFails(@()ra_sim_sensor_detect(w,[0;0;0], ...
    ra_sim_sensor_config('multiple_ellipsoids')));
assertFails(@()ra_sim_sensor_config('multiple_spheres',-3));
assertFails(@()ra_sim_sensor_detect(w,[NaN;0;0], ...
    ra_sim_sensor_config('multiple_spheres')));
wrong=wE;wrong.parts(1).R=diag([1 1 2]);
assertFails(@()ra_sim_sensor_detect(wrong,[0;0;0], ...
    ra_sim_sensor_config('multiple_ellipsoids')));
caseCount=caseCount+1;
% Test 7: dynamic pose already in CURRENT snapshot, no memory.
dyn=part('sphere',[19 0 0],[.5 .5 .5],eye(3),'moving');
WD=makeWorld(dyn,{'moving'},'multiple_spheres');
WD.objects.isDynamic=true;
[det,~]=ra_sim_sensor_detect(WD,[0;0;0],ra_sim_sensor_config('multiple_spheres'));
assert(numel(det.dynamicIndices)==1);
WD.parts.center=[30 0 0];WD.objects.position=[30 0 0];
[det2,~]=ra_sim_sensor_detect(WD,[0;0;0],ra_sim_sensor_config('multiple_spheres'));
assert(isempty(det2.parts));
caseCount=caseCount+1;
% Test 8: original trigger's observed-cover cache accepts the output.
if exist('wp_six_trigger_prepare','file')==2
    for cover={'multiple_spheres','multiple_ellipsoids'}
        if strcmp(cover{1},'multiple_spheres'), candidate=out;
        else,candidate=obsE;end
        K=wp_six_trigger_prepare(candidate,cover{1});
        assert(K.numParts==numel(candidate.parts));
        assert(all(strcmp(cover{1},K.approximation)));
    end
    cacheStatus='PASS_TWO_APPROXIMATION_CACHES';
else
    cacheStatus='SKIPPED_TRIGGER_CACHE_NOT_ON_PATH';
end
caseCount=caseCount+1;
% Test 9: empty scene returns empty current observation, ground retained.
WE=struct('parts',repmat(a,0,1),'objects',struct([]),'ground',w.ground, ...
    'approximation','multiple_spheres');
[oe,ie]=ra_sim_sensor_detect(WE,[0;0;0],ra_sim_sensor_config('multiple_spheres'));
assert(isempty(oe.parts) && ie.detectedObjectCount==0);
assert(isequal(oe.ground,w.ground));
caseCount=caseCount+1;
T=struct('status','PASS_SENSOR_CURRENT_RANGE_ONLY', ...
    'caseCount',caseCount,'sphereSelectedNames',{{'A','C'}}, ...
    'ellipsoidSelectedNames',{infE.detectedObjectNames}, ...
    'triggerCache',cacheStatus, ...
    'sensorRadiusDefaultM',20, ...
    'realFlightTested',false,'mathematicalCollisionProof',false);
end

function p=part(kind,center,axesLength,R,name)
p=struct('kind',kind,'center',center,'R',R, ...
    'params',struct('radius',axesLength(1),'axes',axesLength), ...
    'obstacleName',name,'method','', ...
    'componentIndex',1,'vertices',zeros(0,3), ...
    'H',zeros(0,3),'b',zeros(0,1));
end
function w=makeWorld(parts,names,method)
objs=repmat(struct('name','','position',[0 0 0], ...
    'isDynamic',false,'params',struct('tag','original')),1,numel(names));
meta=repmat(struct('obstacleName','','numParts',1),1,numel(names));
for i=1:numel(names)
    objs(i).name=names{i};
    meta(i).obstacleName=names{i};
    meta(i).numParts=nnz(strcmp({parts.obstacleName},names{i}));
end
w=struct('parts',parts,'objects',objs,'meta',meta, ...
    'partBudgets',ones(1,numel(names))*3, ...
    'desiredPartBudgets',ones(1,numel(names))*3, ...
    'ground',[],'approximation',method,'time',0,'scene','fixture');
end
function assertFails(f)
failed=false;
try,f();catch,failed=true;end
assert(failed,'Expected validation failure did not occur.');
end
