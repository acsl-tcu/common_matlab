function out=BENCH_TRIGGER_V3_ONE_MODE_AB(n,profileN)
%BENCH_TRIGGER_V3_ONE_MODE_AB V2/V3 separate single-mode wall times.
% Never sum six modes. Does NOT include model/HLC future predictor.
if nargin<1,n=100;end
if nargin<2,profileN=20;end
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
scenarios={'empty','distant_36','cable_contact','near_noncontact_36','thin_ellipsoid_36'};
records=repmat(struct('mode','','cover','','scenario','', ...
    'v2MedianMs',NaN,'v3MedianMs',NaN,'v2P95Ms',NaN,'v3P95Ms',NaN, ...
    'v2MaxMs',NaN,'v3MaxMs',NaN,'speedRatio',NaN,'v3Goal2ms',false, ...
    'precomputedPairsCleared',0,'stageMs',struct()),1,30);
idx=0;
for s=1:numel(scenarios)
    [S,E]=fixture(scenarios{s});
    for m=1:3
        for c=1:2
            mode=modes{m};cover=covers{c};
            if c==1,world=S;else,world=E;end
            known=wp_six_trigger_prepare(world,cover);
            [~,cfg,F]=selected_test_context(mode,cover);
            for warm=1:4
                wp_six_trigger_evaluate(known,cfg,F);
                wp_six_trigger_evaluate_fast_v3(known,cfg,F);
            end
            a=zeros(1,n);b=a;
            for rep=1:n
                if mod(rep,2)==0
                    t=tic;r2=wp_six_trigger_evaluate(known,cfg,F);a(rep)=toc(t)*1000;
                    t=tic;r3=wp_six_trigger_evaluate_fast_v3(known,cfg,F);b(rep)=toc(t)*1000;
                else
                    t=tic;r3=wp_six_trigger_evaluate_fast_v3(known,cfg,F);b(rep)=toc(t)*1000;
                    t=tic;r2=wp_six_trigger_evaluate(known,cfg,F);a(rep)=toc(t)*1000;
                end
                assert(strcmp(r2.status,r3.status), 'V3 cannot change gate result');
            end
            cfg.enableStageTiming=true;
            stages=cell(1,profileN);
            for k=1:profileN
                x=wp_six_trigger_evaluate_fast_v3(known,cfg,F);
                stages{k}=x.stageMs;
            end
            stage=stages{1};names=fieldnames(stage);
            for k=1:numel(names)
                field=names{k};y=zeros(1,profileN);
                for j=1:profileN,y(j)=stages{j}.(field);end
                stage.(field)=median(y);
            end
            idx=idx+1;
            records(idx)=struct('mode',mode,'cover',cover,'scenario',scenarios{s}, ...
                'v2MedianMs',median(a),'v3MedianMs',median(b), ...
                'v2P95Ms',percentile95(a),'v3P95Ms',percentile95(b), ...
                'v2MaxMs',max(a),'v3MaxMs',max(b), ...
                'speedRatio',median(a)/max(median(b),eps), ...
                'v3Goal2ms',max(b)<=2, ...
                'precomputedPairsCleared',r3.precomputedPairsCleared, ...
                'stageMs',stage);
            fprintf('%-19s %s × %-19s V2 %.4f V3 %.4f P95 %.4f MAX %.4f ms precleared %d\n', ...
                scenarios{s},mode,cover,median(a),median(b),percentile95(b),max(b), ...
                r3.precomputedPairsCleared);
        end
    end
end
out=struct('records',records,'targetPerModeMs',2, ...
    'modesAreAlternatives',true,'geometryOnly',true, ...
    'hlcPredictionIncluded',false,'fullFlightIncluded',false);
end
function v=percentile95(x)
y=sort(x);v=y(ceil(.95*numel(y)));
end
function [S,E]=fixture(s)
switch s
    case 'empty', c=zeros(3,0);a=zeros(3,0);
    case 'distant_36', c=[40+(1:36);15*ones(1,36);2*ones(1,36)];a=repmat([.2;.12;.09],1,36);
    case 'cable_contact', c=[.8;0;2];a=.18*ones(3,1);
    case 'near_noncontact_36', c=[.3+.09*(0:35);.38*ones(1,36);2*ones(1,36)];a=repmat([.2;.12;.09],1,36);
    case 'thin_ellipsoid_36', c=[.3+.09*(0:35);.24*ones(1,36);2*ones(1,36)];a=repmat([.2;.04;.08],1,36);
end
[S,E]=six_test_fixture(c,a);
end
