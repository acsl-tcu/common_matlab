% TEST_DRONE_OBSTACLES_BUDGET_TEMP.m
% Place in commonmatlab root. Delete after MATLAB passes.
% Original 132-case & DRAW tests were already passed by the user; this test
% checks NEW global-method, budget and conservative coverage logic.
clearvars; close all; clc;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'environment','DRONE_OBSTACLES')));
rng(20261111);
methods={'single_sphere','single_ellipsoid','multiple_spheres', ...
    'multiple_ellipsoids','superellipsoid','convex_polyhedra'};
scenes={'scene_showcase_13','scene_gate_square','scene_mixed_obstacles','scene_dynamic_independent'};
opts=struct('maxPartsPerObstacle',6,'maxTotalParts',36, ...
    'targetPartSize',3.0,'superExponent',8,'hollowSides',16,'polySides',8);
passes=0;
for si=1:numel(scenes)
    for mi=1:numel(methods)
        tic;
        W=DRONE_OBSTACLES(scenes{si},methods{mi},'ground_z0',opts);
        preprocessSec=toc;
        assert(numel(W.parts)<=opts.maxTotalParts, 'Global budget exceeded');
        assert(all([W.meta.numParts]<=opts.maxPartsPerObstacle), ...
            'Per-obstacle budget exceeded');
        expected=methods{mi};
        if strcmp(expected,'multiple_spheres') || strcmp(expected,'single_sphere')
            kind='sphere';
        elseif strcmp(expected,'multiple_ellipsoids') || strcmp(expected,'single_ellipsoid')
            kind='ellipsoid';
        elseif strcmp(expected,'superellipsoid')
            kind='superellipsoid';
        else
            kind='convex_polyhedron';
        end
        if ~isempty(W.parts)
            assert(all(strcmp({W.parts.kind},kind)), ...
                'Mixed shape families detected in one scene');
        end
        falseCount=0; coverCount=0;
        % Source shape vs approximation, obstacle by obstacle (not just
        % union of different obstacles); test static and independent motions.
        if strcmp(scenes{si},'scene_dynamic_independent')
            testTimes=[0 2.0 6.0];
        else
            testTimes=0;
        end
        for t=testTimes
            if t==0
                S=W;
            else
                S=obs_world_at_time(W,t);
            end
            for oi=1:numel(S.objects)
                o=S.objects(oi);
                C=obs_components(o,opts);
                [lo,hi]=obs_local_bounds(C);
                Q=lo+rand(1200,3).*(hi-lo);
                X=(o.R*Q')'+o.position;
                X=X(obs_original_contains(o,X,opts),:);
                for ci=1:numel(C)
                    if strcmp(C(ci).kind,'poly')
                        X=[X;(o.R*C(ci).vertices')'+o.position]; %#ok<AGROW>
                    end
                end
                assert(~isempty(X),'No source points for %s',o.name);
                own=S.parts(strcmp({S.parts.obstacleName},o.name));
                assert(~isempty(own),'Obstacle missing approximation');
                hit=obs_world_contains(own,X,1e-6);
                assert(all(hit),'Containment failure: %s / %s @ t=%.2f', ...
                    o.name,methods{mi},t);
                if t==0
                    % Descriptive Monte Carlo indicator only. This is NOT a
                    % certified volume error, and depends on the sample box.
                    extend=max(.3,.15*max(hi-lo));
                    aa=lo-extend;bb=hi+extend;
                    Z=aa+rand(500,3).*(bb-aa);
                    ZZ=(o.R*Z')'+o.position;
                    realMask=obs_original_contains(o,ZZ,opts);
                    approxMask=obs_world_contains(own,ZZ,1e-6);
                    falseCount=falseCount+sum(approxMask & ~realMask);
                    coverCount=coverCount+sum(approxMask);
                end
            end
        end
        approxExtraPct=100*falseCount/max(1,coverCount);
        fprintf('PASS %-24s %-22s total=%3d max/obj=%d init=%.3fs extra~%.1f%%\n', ...
            scenes{si},methods{mi},numel(W.parts), ...
            max([W.meta.numParts]),preprocessSec,approxExtraPct);
        passes=passes+1;
    end
end
% Per-obstacle method overrides must fail (scene-wide type consistency).
old=opts;
old.perObstacle=struct('square_gate','single_sphere');
blocked=false;
try
    DRONE_OBSTACLES('scene_gate_square','multiple_ellipsoids','ground_z0',old);
catch ME
    blocked=strcmp(ME.identifier,'DRONE_OBSTACLES:MixedApproximation');
end
assert(blocked,'Mixed-method override should be rejected');
fprintf('PASS perObstacle override rejected\n');
fprintf('ALL NEW BUDGET TESTS PASS (%d scene-method combinations).\n',passes);
