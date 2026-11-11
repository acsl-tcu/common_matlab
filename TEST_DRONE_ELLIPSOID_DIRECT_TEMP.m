% TEST_DRONE_ELLIPSOID_DIRECT_TEMP.m
% Place temporarily under commonmatlab/. Delete after MATLAB passes.
% Independent from the previously-passed 132 shape and 24 budget tests.
clearvars;close all;clc;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'environment','DRONE_OBSTACLES')));
rng(20261111);
opts=struct('maxPartsPerObstacle',6,'maxTotalParts',36, ...
    'targetPartSize',3.0,'polySides',8,'hollowSides',16, ...
    'superExponent',8,'mveeTolerance',2e-3, ...
    'mveeMaxIterations',2500,'mveeSafetyFactor',1+1e-8);
methods={'single_ellipsoid','multiple_ellipsoids'};
scenes={'scene_showcase_13','scene_gate_square', ...
    'scene_mixed_obstacles','scene_dynamic_independent'};
if exist('scene_cylinders_high_side','file')==2
    scenes{end+1}='scene_cylinders_high_side';
end
passes=0;
for si=1:numel(scenes)
    for mi=1:numel(methods)
        tic;
        W=DRONE_OBSTACLES(scenes{si},methods{mi},'ground_z0',opts);
        prepTime=toc;
        assert(numel(W.parts)<=opts.maxTotalParts, ...
            'Global approximation budget exceeded');
        assert(all(strcmp({W.parts.kind},'ellipsoid')), ...
            'Non-ellipsoid slipped into ellipsoid-only scene');
        if strcmp(methods{mi},'multiple_ellipsoids')
            assert(all([W.meta.numParts]<=opts.maxPartsPerObstacle), ...
                'Per obstacle approximation budget exceeded');
        else
            assert(all([W.meta.numParts]==1),'Single method must have one part');
        end
        if strcmp(scenes{si},'scene_dynamic_independent')
            times=[0 2 5];
        else
            times=0;
        end
        nSamples=0;
        for t=times
            if t==0,S=W;else,S=obs_world_at_time(W,t);end
            for oi=1:numel(S.objects)
                o=S.objects(oi);
                own=S.parts(strcmp({S.parts.obstacleName},o.name));
                assert(~isempty(own),'Object is missing its ellipsoid(s)');
                C=obs_components(o,opts);
                % Important: when the source itself is a sphere or
                % ellipsoid, direct-fitting intentionally uses its EXACT
                % analytic geometry.  A circumscribed support-polytope has
                % vertices OUTSIDE that source. Requiring the exact source
                % ellipsoid to enclose those artificial vertices is a
                % false-negative test (sphere_demo was failing here).
                isExactSource = (numel(C)==1) && ...
                    any(strcmp(C(1).kind,{'sphere','ellipsoid'}));
                if isExactSource
                    assert(numel(own)==1 && strcmp(own(1).kind,'ellipsoid'), ...
                        'Exact source should produce one ellipsoid');
                    if strcmp(C(1).kind,'sphere')
                        expectedAxes=C(1).params.radius*[1 1 1];
                    else
                        expectedAxes=reshape(C(1).params.axes,1,3);
                    end
                    % This is an ANALYTIC enclosure certificate, not
                    % only a surface-sampling test: same center, same
                    % orientation, and axes no smaller than the source.
                    assert(norm(own(1).center-o.position,Inf)<1e-9, ...
                        'Exact-source ellipsoid center mismatch');
                    assert(norm(own(1).R-o.R,'fro')<1e-9, ...
                        'Exact-source ellipsoid attitude mismatch');
                    assert(all(own(1).params.axes>=expectedAxes.*(1-1e-12)), ...
                        'Exact-source ellipsoid does not enclose source');
                    % Check the world-frame implementation at principal
                    % boundary points too. These are checks, not the proof.
                    Y=[diag(expectedAxes);-diag(expectedAxes)];
                    X=(o.R*Y')'+o.position;
                    assert(all(obs_world_contains(own,X,1e-7)), ...
                        'Exact source principal point leaked: %s @ t=%g', ...
                        o.name,t);
                    nSamples=nSamples+size(X,1);
                else
                    % For non-ellipsoidal sources, the fitting routine
                    % actually uses a certified support-plane outer
                    % polytope, so verify ALL of that cover's vertices.
                    for ci=1:numel(C)
                        V=obs_ellipsoid_cover_vertices(C(ci),opts);
                        X=(o.R*V')'+o.position;
                        assert(all(obs_world_contains(own,X,1e-7)), ...
                            'Outer support cover vertex leaked: %s / %s @ t=%g', ...
                            o.name,methods{mi},t);
                        nSamples=nSamples+size(X,1);
                    end
                end
                % Also sample actual physical volume. Samples alone are
                % not an all-points containment proof.
                [lo,hi]=obs_local_bounds(C);
                Q=lo+rand(3000,3).*(hi-lo);
                X=(o.R*Q')'+o.position;
                X=X(obs_original_contains(o,X,opts),:);
                assert(all(obs_world_contains(own,X,1e-7)), ...
                    'Source sample leaked: %s / %s @ t=%g', ...
                    o.name,methods{mi},t);
                nSamples=nSamples+size(X,1);
            end
        end
        fprintf('PASS %-25s %-22s parts=%2d outer+inner samples=%5d preprocessing=%.3fs\n', ...
            scenes{si},methods{mi},numel(W.parts),nSamples,prepTime);
        passes=passes+1;
    end
end

% Direct source fit MUST beat old AABB->ellipsoid for a slender cylinder.
o=obs_cylinder('long_cylinder',[9 7 8],0.60,8,[25 14 50]);
C=obs_components(o,opts);
p=obs_approx_single_ellipsoid(o,C,opts);
[lo,hi]=obs_local_bounds(C);
oldAxes=sqrt(3)*(hi-lo)/2;
oldVolume=4*pi/3*prod(oldAxes);
newVolume=4*pi/3*prod(p.params.axes);
fprintf('LONG CYLINDER: old AABB volume=%.4g new direct MVEE volume=%.4g ratio=%.4f\n', ...
    oldVolume,newVolume,newVolume/oldVolume);
assert(newVolume<0.90*oldVolume, ...
    'Direct fit did not improve on AABB-based ellipsoid for cylinder');

% A box MVEE is geometrically equal in volume to the sqrt(3)*halfsize
% construction; no fictitious improvement expected for the single box.
o2=obs_box('large_box_check',[9 7 8],[15 15 15],[23 17 61]);
C2=obs_components(o2,opts);
p2=obs_approx_single_ellipsoid(o2,C2,opts);
boxVolume=4*pi/3*prod(sqrt(3)*[7.5 7.5 7.5]);
fitVolume=4*pi/3*prod(p2.params.axes);
fprintf('15m BOX: analytic MVEE volume=%.4g numerical fit=%.4g ratio=%.5f\n', ...
    boxVolume,fitVolume,fitVolume/boxVolume);
assert(fitVolume/boxVolume>=1-1e-6 && fitVolume/boxVolume<=1.08, ...
    '15m cube fit is unexpectedly far from analytic MVEE');

% No project run/HLC/CBF change is required for this patch.
fprintf('ALL DIRECT ELLIPSOID TESTS PASS (%d scene-method cases).\n',passes);
fprintf('Temporary test may be deleted AFTER MATLAB verification.\n');
