function T=TEST_RA_FOUR_METHOD_COVER_BRIDGE_20261115()
%TEST_RA_FOUR_METHOD_COVER_BRIDGE_20261115
% Focused regression for source method -> two geometry kernel types.
% Tests real 4-second geometry APIs, but not actual Flight or physical safety.
% Requires previously installed REFERENCE_OBSTACLE_AVOIDANCE_TESTS/SUPPORT.
methods={'single_sphere','multiple_spheres', ...
    'single_ellipsoid','multiple_ellipsoids'};
modes={'C','B','D'};
agent=RA_TEST_AGENT();
rows=struct('sourceMethod',{},'systemMode',{},'kind',{}, ...
    'kernelCover',{},'partsReceived',{},'geometryStatus',{});
for i=1:numel(methods)
    source=methods{i};
    if contains(source,'sphere')
        kind='sphere';cover='multiple_spheres';
        p=makePart('sphere',[1;0;1]);
    else
        kind='ellipsoid';cover='multiple_ellipsoids';
        p=makePart('ellipsoid',[1;0;1]);
    end
    if startsWith(source,'multiple_')
        p2=makePart(kind,[40;0;1]);
        parts=[p,p2];
    else
        parts=p;
    end
    original=struct('parts',parts, ...
        'objects',struct('name','A','isDynamic',false), ...
        'ground',[],'approximation',source);
    [seen,rep]=ra_sim_sensor_detect(original,[0;0;1],ra_sim_sensor_config(20));
    assert(isequaln(seen.parts,original.parts));
    assert(rep.detectedObjectCount==1);
    assert(strcmp(seen.approximation,source));
    for j=1:numel(modes)
        prepared=reference_avoidance_prepare_observed( ...
            agent,modes{j},seen,[],struct('allowMexBuild',false));
        c=prepared.cfg;k=prepared.known;
        assert(strcmp(c.systemMode,modes{j}));
        assert(strcmp(c.obstacleKind,kind));
        assert(strcmp(c.obstacleSourceApproximation,source));
        assert(strcmp(c.obstacleApproximation,cover));
        assert(strcmp(k.obstacleKind,kind));
        assert(strcmp(k.obstacleSourceApproximation,source));
        assert(strcmp(k.approximation,cover));
        assert(strcmp(prepared.obstacleMethodFromWorld,source));
        assert(k.numParts==numel(parts));
        for h=1:numel(parts)
            assert(isequal(k.center(:,h),parts(h).center(:)));
            assert(isequal(k.R(:,:,h),parts(h).R));
            if strcmp(kind,'sphere')
                axesExpected=repmat(parts(h).params.radius,3,1);
                assert(k.kind(h)==1);
            else
                axesExpected=parts(h).params.axes(:);
                assert(k.kind(h)==2);
            end
            assert(isequal(k.axes(:,h),axesExpected));
        end
        F=RA_TEST_FORECAST(c);
        g=wp_six_trigger_evaluate_fast_v8(k,c,F);
        assert(~strcmp(g.status,'UNKNOWN'), ...
            'Prepared geometry rejected source method %s with mode %s: %s', ...
            source,modes{j},g.reason);
        rows(end+1)=struct('sourceMethod',source, ...
            'systemMode',modes{j},'kind',kind, ...
            'kernelCover',cover,'partsReceived',k.numParts, ...
            'geometryStatus',g.status); %#ok<AGROW>
    end
end
T=struct('status','PASS_4_SOURCE_METHODS_2_SHAPES_3_SYSTEM_MODES', ...
    'cases',numel(rows),'records',rows, ...
    'safetyCertified',false,'realFlightTested',false);
end

function p=makePart(kind,center)
if strcmp(kind,'sphere')
    params=struct('radius',2,'axes',[2;2;2]);
else
    params=struct('radius',2,'axes',[2;1;.5]);
end
p=struct('kind',kind,'center',center,'R',eye(3), ...
    'params',params,'obstacleName','A','method','');
end
