function out=BENCH_TRIGGER_V2_STAGE_TIMES(repetitions,profileRepetitions)
%BENCH_TRIGGER_V2_STAGE_TIMES Old/new per-selected-mode A/B and stage costs.
%
% Two distinct measurements:
%  (1) real uninstrumented wall time of geometry gate (old vs new),
%  (2) instrumented stage time attribution (overhead is included, NOT
%      a substitute for uninstrumented latency).
% All 6 mode combinations execute SEPARATELY. No six-way OR.
if nargin<1,repetitions=100;end
if nargin<2,profileRepetitions=30;end
assert(repetitions>=10 && profileRepetitions>=5);
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
scenarios={'empty','distant_36','cable_contact','near_noncontact_36','thin_ellipsoid_36'};
stageFields={'validation','inflation','globalBroadphase', ...
    'intervalBroadphase','sweepBuild','componentBroadphase','supportSeparation'};
out=struct();
for si=1:numel(scenarios)
    scenario=scenarios{si};
    [S,E]=fixture(scenario);
    results=repmat(struct('mode','','cover','','oldMedianMs',0, ...
        'oldP95Ms',0,'oldMaxMs',0,'newMedianMs',0,'newP95Ms',0, ...
        'newMaxMs',0,'newUnder2Ms',false,'newStatus','', ...
        'stageMedianMs',struct(), 'stageMedianMeasuredTotalMs',0, ...
        'narrowphasePairs',0,'componentCandidatePairs',0, ...
        'globalAabbCleared',0,'intervalAabbCleared',0, ...
        'componentAabbCleared',0),3,2);
    for mi=1:3
        for ci=1:2
            mode=modes{mi};cover=covers{ci};
            if ci==1,world=S;else,world=E;end
            known=wp_six_trigger_prepare(world,cover);
            [~,cfg,F]=selected_test_context(mode,cover);
            for warm=1:5
                wp_six_trigger_evaluate(known,cfg,F);
                wp_six_trigger_evaluate_baseline_20261112(known,cfg,F);
            end
            oldTimes=zeros(1,repetitions);newTimes=oldTimes;
            for rep=1:repetitions
                if mod(rep,2)==0
                    a=tic;old=wp_six_trigger_evaluate_baseline_20261112(known,cfg,F);
                    oldTimes(rep)=1000*toc(a);
                    a=tic;new=wp_six_trigger_evaluate(known,cfg,F);
                    newTimes(rep)=1000*toc(a);
                else
                    a=tic;new=wp_six_trigger_evaluate(known,cfg,F);
                    newTimes(rep)=1000*toc(a);
                    a=tic;old=wp_six_trigger_evaluate_baseline_20261112(known,cfg,F);
                    oldTimes(rep)=1000*toc(a);
                end
                assert(strcmp(new.status,old.status) || ...
                       (strcmp(old.status,'POSSIBLE_CONTACT') && ...
                        strcmp(new.status,'CLEAR_CONDITIONAL')));
            end
            % Instrumentation separately to avoid timing overhead leakage.
            cfg.enableStageTiming=true;
            stageSamples=zeros(profileRepetitions,numel(stageFields));
            profiledTotal=zeros(1,profileRepetitions);
            for rep=1:profileRepetitions
                r=wp_six_trigger_evaluate(known,cfg,F);
                profiledTotal(rep)=r.elapsedMs;
                for field=1:numel(stageFields)
                    stageSamples(rep,field)=r.stageMs.(stageFields{field});
                end
            end
            record=results(mi,ci);
            record.mode=mode;record.cover=cover;
            record.oldMedianMs=median(oldTimes);
            record.oldP95Ms=p95(oldTimes);record.oldMaxMs=max(oldTimes);
            record.newMedianMs=median(newTimes);
            record.newP95Ms=p95(newTimes);record.newMaxMs=max(newTimes);
            record.newUnder2Ms=all(newTimes<=2);
            record.newStatus=new.status;
            record.stageMedianMs=cell2struct(num2cell(median(stageSamples,1)),stageFields,2);
            record.stageMedianMeasuredTotalMs=median(profiledTotal);
            record.narrowphasePairs=new.narrowphasePairs;
            record.componentCandidatePairs=new.componentCandidatePairs;
            record.globalAabbCleared=new.globalAabbCleared;
            record.intervalAabbCleared=new.intervalAabbCleared;
            record.componentAabbCleared=new.componentAabbCleared;
            results(mi,ci)=record;
            fprintf('%-18s %s %-19s  old %.4f / new %.4f ms, P95 %.4f, MAX %.4f  broad parts %d narrow %d\n', ...
                scenario,mode,cover,record.oldMedianMs,record.newMedianMs, ...
                record.newP95Ms,record.newMaxMs,record.componentCandidatePairs,record.narrowphasePairs);
        end
    end
    out.(scenario)=results;
end
out.goalMsPerSelectedMode=2;
out.allSixSummed=false;
out.geometryOnly=true;
out.sourceOfMass='current EKF estimated mL; synthetic fixture in timing test';
out.realHlcPredictionIncluded=false;
out.full25msFlightIncluded=false;
end
function y=p95(v)
v=sort(v);y=v(ceil(.95*numel(v)));
end
function [S,E]=fixture(s)
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
    case 'thin_ellipsoid_36'
        % Stress: large circumspheres overlap, slender rotated ellipsoids
        % may separate in narrow phase. The two cover types differ.
        c=[.3+.09*(0:35);.24*ones(1,36);2*ones(1,36)];
        a=repmat([.20;.04;.08],1,36);
end
[S,E]=six_test_fixture(c,a);
end
