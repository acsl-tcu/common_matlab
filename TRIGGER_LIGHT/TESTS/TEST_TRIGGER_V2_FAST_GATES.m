function report=TEST_TRIGGER_V2_FAST_GATES(randomCases)
%TEST_TRIGGER_V2_FAST_GATES Verify AABB and batched separation against old.
% The old and new routines each run ONE selected C/B/D x obstacle cover.
if nargin<1,randomCases=60;end
rng(20261112);
modes={'C','B','D'};
covers={'multiple_spheres','multiple_ellipsoids'};
scenarios={'empty','distant_36','cable_contact','near_noncontact_36'};
unchangedClear=0;unchangedPossible=0;improvedClear=0;
for mi=1:3
    for ci=1:2
        mode=modes{mi};cover=covers{ci};
        [~,cfg,F]=selected_test_context(mode,cover);
        for si=1:numel(scenarios)
            [S,E]=fixtureForScenario(scenarios{si});
            if ci==1,known=wp_six_trigger_prepare(S,cover);
            else,known=wp_six_trigger_prepare(E,cover);end
            old=wp_six_trigger_evaluate_baseline_20261112(known,cfg,F);
            new=wp_six_trigger_evaluate(known,cfg,F);
            assert(strcmp(new.systemMode,mode));
            assert(strcmp(new.obstacleApproximation,cover));
            assert(~strcmp(new.status,'UNKNOWN'));
            if strcmp(old.status,'CLEAR_CONDITIONAL')
                assert(strcmp(new.status,'CLEAR_CONDITIONAL'), ...
                    'Regression: previous clearance lost for %s/%s/%s',mode,cover,scenarios{si});
                unchangedClear=unchangedClear+1;
            elseif strcmp(old.status,'POSSIBLE_CONTACT')
                if strcmp(new.status,'CLEAR_CONDITIONAL'),improvedClear=improvedClear+1;
                else,unchangedPossible=unchangedPossible+1;end
            else
                error('LightTrigger:Regression','Unexpected baseline unknown');
            end
        end
        % Random geometries and separation direction parity.
        for k=1:randomCases
            [~,cfg,F]=selected_test_context(mode,cover,linspace(0,4,41),'swing');
            point=[-0.2+3.6*rand();-.9+1.8*rand();0.6+2.9*rand()];
            a=[.03;.03;.03]+rand(3,1).*[.30;.20;.28];
            [S,E]=six_test_fixture(point,a);
            if ci==1,known=wp_six_trigger_prepare(S,cover);
            else,known=wp_six_trigger_prepare(E,cover);end
            old=wp_six_trigger_evaluate_baseline_20261112(known,cfg,F);
            new=wp_six_trigger_evaluate(known,cfg,F);
            if strcmp(old.status,'CLEAR_CONDITIONAL')
                assert(strcmp(new.status,'CLEAR_CONDITIONAL'), ...
                    'False conservative regression trial %d %s %s',k,mode,cover);
            end
        end
        % New batched support function must reproduce scalar proof status.
        [~,cfg,F]=selected_test_context(mode,cover);
        bodies=wp_six_trigger_make_sweep(cfg,F,1,mode);
        centers=[.5+3*rand(1,30);-.5+rand(1,30);1+2*rand(1,30)];
        axesLength=repmat([.25;.14;.10],1,30);
        [S,E]=six_test_fixture(centers,axesLength);
        if ci==1,known=wp_six_trigger_prepare(S,cover);
        else,known=wp_six_trigger_prepare(E,cover);end
        for b=1:numel(bodies)
            [newSep,~]=wp_six_trigger_separation_batch(bodies(b),known,1:30);
            for j=1:30
                oldSep=wp_six_trigger_separation(bodies(b),known,j);
                assert(newSep(j)==oldSep, ...
                    'Batched support direction mismatch mode=%s part=%d j=%d',mode,b,j);
            end
        end
    end
end
report=struct('status','PASS_FAST_GATES_BASELINE_PARITY', ...
    'perModeIndependent',true,'baselineClearPreserved',unchangedClear, ...
    'baselineRisksPreserved',unchangedPossible,'newClearFromOldRisk',improvedClear, ...
    'randomCasesPerModeCover',randomCases,'numericNotPhysicalProof',true);
fprintf('PASS V2 fast gates; %d previous CLEAR kept, %d previous risks kept, %d new CLEAR\n', ...
    unchangedClear,unchangedPossible,improvedClear);
end

function [S,E]=fixtureForScenario(s)
switch s
    case 'empty'
        c=zeros(3,0);a=zeros(3,0);
    case 'distant_36'
        c=[40+(1:36);15*ones(1,36);2*ones(1,36)];
        a=repmat([.20;.12;.09],1,36);
    case 'cable_contact'
        c=[.8;0;2];a=[.18;.18;.18];
    case 'near_noncontact_36'
        c=[.3+.09*(0:35);.38*ones(1,36);2*ones(1,36)];
        a=repmat([.20;.12;.09],1,36);
end
[S,E]=six_test_fixture(c,a);
end
