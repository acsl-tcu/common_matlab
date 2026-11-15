function report = TEST_TRIGGER_V2_RESULT_FIELDS()
%TEST_TRIGGER_V2_RESULT_FIELDS Regression test for FAST V2 result schema.
%
% Independently exercise each of the six *alternative* geometries.
% One evaluator call selects exactly one C/B/D + obstacle approximation.
% All diagnostic counters must exist even on early-return paths.
% This test does not validate real-HLC predictions or physical safety.
modes = {'C','B','D'};
covers = {'multiple_spheres','multiple_ellipsoids'};
fields = {'globalBroadphaseCleared','intervalBroadphaseCleared', ...
    'componentBroadphaseCleared','globalAabbCleared', ...
    'intervalAabbCleared','componentAabbCleared', ...
    'componentCandidatePairs','narrowphasePairs','supportNormalTests'};
scenariosChecked = 0;

for mi = 1:numel(modes)
    for ci = 1:numel(covers)
        mode = modes{mi};
        cover = covers{ci};
        [~,cfg,F] = selected_test_context(mode,cover);

        % Early return: no known obstacle, no ground.
        checkScenario(zeros(3,0),zeros(3,0),cfg,F,cover,'CLEAR_CONDITIONAL');

        % Early return: whole-horizon broadphase removes all parts.
        checkScenario([60;40;20],[.2;.1;.1],cfg,F,cover,'CLEAR_CONDITIONAL');

        % Active broadphase candidates: exercises k-loop and diagnostic sums.
        centers = [.3+.09*(0:35);.38*ones(1,36);2*ones(1,36)];
        axesLength = repmat([.20;.12;.09],1,36);
        checkScenario(centers,axesLength,cfg,F,cover,'');

        % Profiled execution must expose all stage timing counters.
        cfgProfile = cfg;
        cfgProfile.enableStageTiming = true;
        checkScenario(centers,axesLength,cfgProfile,F,cover,'');

        % Fail-closed path must also retain a complete result schema.
        [S,E] = six_test_fixture(centers,axesLength);
        if strcmp(cover,'multiple_spheres')
            known = wp_six_trigger_prepare(S,cover);
        else
            known = wp_six_trigger_prepare(E,cover);
        end
        invalidF = rmfield(F,'rhoLoad');
        r = wp_six_trigger_evaluate(known,cfg,invalidF);
        assert(strcmp(r.status,'UNKNOWN'));
        checkFields(r);
        scenariosChecked = scenariosChecked+1;
    end
end
report = struct('status','PASS_V2_RESULT_SCHEMA_INITIALIZED', ...
    'scenariosChecked',scenariosChecked, ...
    'oneModePerCall',true,'physicalSafetyCertified',false);
fprintf('PASS V2 result fields: %d independent single-mode calls\n',scenariosChecked);

    function checkScenario(centers,axesLength,configuration,forecast,kind,expected)
        [spheres,ellipsoids] = six_test_fixture(centers,axesLength);
        if strcmp(kind,'multiple_spheres')
            prepared = wp_six_trigger_prepare(spheres,kind);
        else
            prepared = wp_six_trigger_prepare(ellipsoids,kind);
        end
        output = wp_six_trigger_evaluate(prepared,configuration,forecast);
        assert(strcmp(output.systemMode,configuration.systemMode));
        assert(strcmp(output.obstacleApproximation,kind));
        if ~isempty(expected), assert(strcmp(output.status,expected)); end
        checkFields(output);
        scenariosChecked = scenariosChecked+1;
    end

    function checkFields(output)
        for fi = 1:numel(fields)
            fieldName = fields{fi};
            assert(isfield(output,fieldName), ...
                'Missing V2 diagnostic field: %s',fieldName);
            value = output.(fieldName);
            assert(isnumeric(value) && isscalar(value) && isfinite(value) && value>=0, ...
                'Invalid V2 diagnostic field: %s',fieldName);
        end
        stageNames = {'validation','inflation','globalBroadphase', ...
            'intervalBroadphase','sweepBuild','componentBroadphase','supportSeparation'};
        assert(isfield(output,'stageMs'));
        for fi = 1:numel(stageNames)
            assert(isfield(output.stageMs,stageNames{fi}));
        end
    end
end
