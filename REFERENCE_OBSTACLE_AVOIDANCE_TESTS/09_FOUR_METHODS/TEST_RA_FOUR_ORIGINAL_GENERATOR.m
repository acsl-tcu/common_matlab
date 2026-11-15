function T=TEST_RA_FOUR_ORIGINAL_GENERATOR()
% Original DRONE_OBSTACLES is read only; test all four generator modes.
if exist('DRONE_OBSTACLES','file')~=2 || exist('scene_mixed_obstacles','file')~=2
    T=struct('status','SKIPPED_ORIGINAL_GENERATOR_NOT_ON_PATH', ...
       'methodsChecked',0,'handoffs',0,'originalPartsPreserved',false);return
end
methods={'single_sphere','single_ellipsoid','multiple_spheres','multiple_ellipsoids'};
agent=RA_TEST_AGENT();handoffs=0;
rows=struct('method',{},'kind',{},'originalObjects',{}, ...
    'originalParts',{},'detectedObjects',{},'detectedParts',{});
for i=1:numel(methods)
    method=methods{i};
    options=struct('maxPartsPerObstacle',6,'maxTotalParts',36, ...
        'targetPartSize',2);
    w=DRONE_OBSTACLES('scene_mixed_obstacles',method,'none',options);
    I=ra_obstacle_method_info(w);
    assert(strcmp(I.method,method) && I.kindFromParts);
    % Large radius ensures all original parts are returned; no refitting.
    [obs,rep]=ra_sim_sensor_detect(w,[0;0;0],ra_sim_sensor_config(1000));
    assert(isequaln(obs.parts,w.parts));
    assert(isequaln(obs.objects,w.objects));
    assert(rep.detectedObjectCount==numel(w.objects));
    if startsWith(method,'single_')
        assert(numel(w.parts)==numel(w.objects));
    else
        assert(numel(w.parts)<=options.maxTotalParts);
    end
    for mode={'C','B','D'}
        P=reference_avoidance_prepare_observed(agent,mode{1},obs,[], ...
            struct('allowMexBuild',false));
        assert(P.known.numParts==numel(w.parts));
        assert(strcmp(P.cfg.obstacleKind,I.kind));
        for j=1:numel(w.parts)
            q=w.parts(j);
            assert(isequal(P.known.center(:,j),q.center(:)));
            assert(isequal(P.known.R(:,:,j),q.R));
            if strcmp(q.kind,'sphere'),a=repmat(q.params.radius,3,1);
            else,a=q.params.axes(:);end
            assert(isequal(P.known.axes(:,j),a));
        end
        handoffs=handoffs+1;
    end
    rows(end+1)=struct('method',method,'kind',I.kind, ...
        'originalObjects',numel(w.objects),'originalParts',numel(w.parts), ...
        'detectedObjects',rep.detectedObjectCount, ...
        'detectedParts',rep.returnedPartCount); %#ok<AGROW>
end
T=struct('status','PASS_ORIGINAL_GENERATOR_FOUR_METHODS', ...
    'methodsChecked',4,'handoffs',handoffs,'originalPartsPreserved',true, ...
    'records',rows,'actualFlightTested',false);
end
