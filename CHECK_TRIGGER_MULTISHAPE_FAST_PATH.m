function R=CHECK_TRIGGER_MULTISHAPE_FAST_PATH()
% Fast geometry regression for all 12 supported source shapes, not cylinder-only.
% Uses only synthetic already-observed geometry. No Flight/HLC/plant/MEX.
% The production sweep is unchanged for failures to establish separation.
root=fileparts(mfilename('fullpath'));
base=fullfile(root,'reference','REPLANNING_SYSTEM','FUTURE_COLLISION_TRIGGER');
assert(isfile(fullfile(base,'TRIGGER_NOMINAL_SWEPT_INTERVALS.m')), ...
    'CHECK_TRIGGER_MULTISHAPE_FAST_PATH:Installation', ...
    'Extract the ZIP at common_matlab, not at REPLANNING_SYSTEM.');
addpath(base,'-begin');
t=[0 0.05 0.1 0.2];
A=[0.93 -0.31 0.20;0.34 0.85 -0.40;-0.046 0.425 0.904];
[Q,~]=qr(A);
if det(Q)<0,Q(:,1)=-Q(:,1);end
rots={eye(3),Q};
shapes=cell(1,12);
names={'sphere','ellipsoid','box','cylinder','capsule', ...
    'cone','frustum','torus','pyramid','polyhedron', ...
    'regular_prism','polygon_prism'};
count=0; failures=0;
for j=1:numel(names)
    for ir=1:numel(rots)
        sh=example_shape(j,rots{ir});
        % A known solid point; for the torus use the centre of its tube.
        local=[0;0;0];
        if strcmp(names{j},'torus'),local=[1.2;0;0];end
        if any(strcmp(names{j},{'pyramid','polyhedron'}))
            local=[0.15;0.15;0.15];
        end
        if strcmp(names{j},'polygon_prism'),local=[0.25;0.25;0];end
        u=sh.center0+sh.rotation*local;
        U=repmat(u,1,numel(t));
        L=U-repmat(sh.rotation*[0;0;2],1,numel(t));
        W=TRIGGER_NOMINAL_SWEPT_INTERVALS(U,L,t,sh, ...
            0.02,0.02,0.01,[],[]);
        assert(~isempty(W.first_obstacle_ids) && ...
            isequal(W.first_obstacle_ids,sh.id) && ...
            isequal(W.first_time_bracket_s,t(1:2)), ...
            'CHECK_TRIGGER_MULTISHAPE_FAST_PATH:Contact', ...
            '%s (rotation %d) missed a point inside the true source.', ...
            names{j},ir);
        count=count+1;
        % A very distant hull must clear, despite a deliberately generous
        % yet VALID enclosing sphere. Exercises the same generic path.
        U=repmat(sh.center0+sh.rotation*[12;12;12],1,numel(t));
        L=U-repmat(sh.rotation*[0;0;2],1,numel(t));
        W=TRIGGER_NOMINAL_SWEPT_INTERVALS(U,L,t,sh, ...
            0.02,0.02,0.01,[],[]);
        assert(isempty(W.first_obstacle_ids), ...
            'CHECK_TRIGGER_MULTISHAPE_FAST_PATH:Far', ...
            '%s (rotation %d) distant body false alarm.',names{j},ir);
        count=count+1;
        % Cylinder, ellipsoid, box and all the other nonspherical shapes:
        % inside the generous outer ball but STRICTLY above the top face.
        % Fast support must not report a contact from that enclosing ball.
        % (Sphere's legacy broadphase special case requires its tight b.)
        if ~strcmp(names{j},'sphere')
            U=repmat(sh.center0+sh.rotation*[0;0;4],1,numel(t));
            L=U-repmat(sh.rotation*[0;0;0.1],1,numel(t));
            W=TRIGGER_NOMINAL_SWEPT_INTERVALS(U,L,t,sh, ...
                0.02,0.02,0.01,[],[]);
            assert(isempty(W.first_obstacle_ids), ...
                'CHECK_TRIGGER_MULTISHAPE_FAST_PATH:AxialClear', ...
                '%s (rotation %d) failed strict source-axis clearance.', ...
                names{j},ir);
            count=count+1;
        end
        % A near-line geometry with potential broadphase candidates. A
        % conservative false positive is acceptable, but tie/ID rules are
        % not. Reversing obstacle order must not change earliest result.
        S=sh; S.id=sh.id+1000;
        U=repmat(sh.center0+sh.rotation*[0.7;0.55;0.3],1,numel(t));
        L=U-repmat(sh.rotation*[0;0;2],1,numel(t));
        W=TRIGGER_NOMINAL_SWEPT_INTERVALS(U,L,t,[sh S], ...
            0.02,0.02,0.01,sh.id,[]);
        X=TRIGGER_NOMINAL_SWEPT_INTERVALS(U,L,t,[S sh], ...
            0.02,0.02,0.01,sh.id,[]);
        assert(isequal(sort(W.first_obstacle_ids), ...
                       sort(X.first_obstacle_ids)) && ...
               isequal(sort(W.new_obstacle_ids),sort(X.new_obstacle_ids)) && ...
               isequaln(W.first_time_bracket_s,X.first_time_bracket_s), ...
            'CHECK_TRIGGER_MULTISHAPE_FAST_PATH:ID', ...
            '%s changed ID/time with enumeration order.',names{j});
        count=count+1;
    end
end
R=struct('status','PASS_MULTISHAPE_DIRECTIONAL_SUPPORT_SWEEP', ...
    'cases',count,'shape_count',numel(names),'rotations_per_shape', ...
    numel(rots),'failures',failures,'real_flight_tested',false, ...
    'hard_wcet_proven',false);
fid=fopen(fullfile(root,'TRIGGER_MULTISHAPE_FAST_PATH_RESULT.txt'),'w');
if fid>0
    fprintf(fid,'%s\ncases=%d shapes=%d failures=%d\n', ...
        R.status,count,numel(names),failures);
    fclose(fid);
end
end

function sh=example_shape(j,R)
name={'sphere','ellipsoid','box','cylinder','capsule','cone', ...
    'frustum','torus','pyramid','polyhedron', ...
    'regular_prism','polygon_prism'};
base=struct('id',j,'type',name{j},'dimensions',[], ...
    'rotation',R,'center0',[1;2;3],'is_stationary',true, ...
    'bounding_radius_m',10,'vertices',zeros(0,3), ...
    'outline',zeros(0,2));
sh=base;
switch j
    case 1,sh.dimensions=0.8;
    case 2,sh.dimensions=[1.5 0.65 0.4];
    case 3,sh.dimensions=[2 1.4 2.8];
    case 4,sh.dimensions=[0.55 4];
    case 5,sh.dimensions=[0.7 3];
    case 6,sh.dimensions=[0.8 2.5];
    case 7,sh.dimensions=[1.0 0.3 2.5];
    case 8,sh.dimensions=[1.2 0.35];
    case {9,10}
        sh.vertices=[0 0 0;1 0 0;0 1 0;0 0 1];
        sh.dimensions=[1 2];
    case 11
        sh.outline=[1 0;0.5 0.866;-0.5 0.866; ...
            -1 0;-0.5 -0.866;0.5 -0.866];
        sh.dimensions=[6 1 3];
    case 12
        sh.outline=[0 0;2 0;2 0.5;0.5 0.5;0.5 2;0 2];
        sh.dimensions=struct('outline',sh.outline,'height',2);
end
end
