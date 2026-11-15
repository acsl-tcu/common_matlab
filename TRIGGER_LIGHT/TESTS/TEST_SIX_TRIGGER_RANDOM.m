function out=TEST_SIX_TRIGGER_RANDOM(numCases)
%TEST_SIX_TRIGGER_RANDOM Independent 6xN analytic pendulum comparisons.
% The original supplied collision code is only a dense collision WITNESS.
% No sampled noncollision is claimed as a continuous-time proof.
if nargin<1,numCases=30;end
assert(numCases>=1 && numCases==floor(numCases));
rng(20261112);
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
witnesses=zeros(3,2);falseClears=zeros(3,2);
clearCounts=zeros(3,2);possibleCounts=zeros(3,2);
for trial=1:numCases
    center=[.1+2.6*rand();-.65+1.3*rand();.8+2.6*rand()];
    axesLengths=[.08;.08;.08]+[.24;.18;.28].*rand(3,1);
    [S,E]=six_test_fixture(center,axesLengths);
    for mi=1:3
        mode=modes{mi};
        original=wp_system_geometry_config(mode);
        for ci=1:2
            cover=covers{ci};
            [~,cfg,F]=selected_test_context(mode,cover,linspace(0,4,41),'swing',0);
            if ci==1,world=S;else,world=E;end
            known=wp_six_trigger_prepare(world,cover);
            R=wp_six_trigger_evaluate(known,cfg,F);
            if strcmp(R.status,'CLEAR_CONDITIONAL')
                clearCounts(mi,ci)=clearCounts(mi,ci)+1;
            elseif strcmp(R.status,'POSSIBLE_CONTACT')
                possibleCounts(mi,ci)=possibleCounts(mi,ci)+1;
            else
                error('LightTrigger:Test','Unexpected UNKNOWN for analytic trajectory');
            end
            hit=false;
            for instant=linspace(0,4,81)
                truth=six_test_forecast(instant,'swing',0);
                geometry=wp_system_shape_at_pose(original,truth.pDrone,truth.pLoad);
                report=wp_system_collision(original,geometry,world,instant);
                if report.collision, hit=true;break;end
            end
            if hit
                witnesses(mi,ci)=witnesses(mi,ci)+1;
                if strcmp(R.status,'CLEAR_CONDITIONAL')
                    falseClears(mi,ci)=falseClears(mi,ci)+1;
                    error('LightTrigger:FalseClear','Trial %d mode %s cover %s',trial,mode,cover);
                end
            end
        end
    end
end
out=struct('status','PASS_INDEPENDENT_ANALYTIC_SWING', ...
    'numCases',numCases,'totalDecisions',numCases*6, ...
    'witnessesByMode',witnesses,'falseClearByMode',falseClears, ...
    'clearByMode',clearCounts,'possibleByMode',possibleCounts, ...
    'totalFalseClear',sum(falseClears,'all'), ...
    'actualHlcForecastEvaluated',false,'physicalSafetyCertified',false);
fprintf('PASS 6 isolated pendulum tests: %d decisions, %d witnessed hits, 0 false CLEAR\n', ...
    numCases*6,sum(witnesses,'all'));
end
