function R=BENCH_TRIGGER_V4_ONE_MODE_AB(repeats)
%BENCH_TRIGGER_V4_ONE_MODE_AB Exact same input V3/V4 alternating timing.
% Results always per ONE selected C/B/D x obstacle-cover alternative.
% There is no "6 modes summed" benchmark. Forecast construction excluded.
if nargin<1,repeats=100;end
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
scenarios={'empty','distant_36','cable_contact', ...
    'near_noncontact_36','thin_ellipsoid_36'};
rows=repmat(struct('scenario','','mode','','cover','','v3MedianMs',NaN, ...
    'v4MedianMs',NaN,'v3P95Ms',NaN,'v4P95Ms',NaN, ...
    'v3MaxMs',NaN,'v4MaxMs',NaN,'sameStatus',false, ...
    'v4ConditionalClear',false,'v4TightBoxCleared',0),1,30);
row=0;
for si=1:numel(scenarios)
    [S,E]=fixture(scenarios{si});
    for mi=1:3
        for ci=1:2
            mode=modes{mi};cover=covers{ci};
            if ci==1,world=S;else,world=E;end
            known=wp_six_trigger_prepare(world,cover);
            [~,cfg,F]=selected_test_context(mode,cover);
            for k=1:4
                wp_six_trigger_evaluate_fast_v3(known,cfg,F);
                wp_six_trigger_evaluate_fast_v4(known,cfg,F);
            end
            a=zeros(1,repeats);b=a;
            for k=1:repeats
                if mod(k,2)
                    ticId=tic; old=wp_six_trigger_evaluate_fast_v3(known,cfg,F);a(k)=toc(ticId)*1000;
                    ticId=tic; new=wp_six_trigger_evaluate_fast_v4(known,cfg,F);b(k)=toc(ticId)*1000;
                else
                    ticId=tic; new=wp_six_trigger_evaluate_fast_v4(known,cfg,F);b(k)=toc(ticId)*1000;
                    ticId=tic; old=wp_six_trigger_evaluate_fast_v3(known,cfg,F);a(k)=toc(ticId)*1000;
                end
                assert(~(strcmp(old.status,'CLEAR_CONDITIONAL') && ...
                    ~strcmp(new.status,'CLEAR_CONDITIONAL')), 'V4 lost V3 CLEAR');
            end
            row=row+1;
            rows(row)=struct('scenario',scenarios{si},'mode',mode,'cover',cover, ...
                'v3MedianMs',median(a),'v4MedianMs',median(b), ...
                'v3P95Ms',p95(a),'v4P95Ms',p95(b), ...
                'v3MaxMs',max(a),'v4MaxMs',max(b), ...
                'sameStatus',strcmp(old.status,new.status), ...
                'v4ConditionalClear',strcmp(new.status,'CLEAR_CONDITIONAL'), ...
                'v4TightBoxCleared',getFieldOrZero(new,'tightAabbCleared'));
            fprintf('V4 %-19s %s x %-19s V3 %.4f V4 %.4f P95 %.4f MAX %.4f ms, tight=%d\n', ...
                scenarios{si},mode,cover,rows(row).v3MedianMs, ...
                rows(row).v4MedianMs,rows(row).v4P95Ms,rows(row).v4MaxMs, ...
                rows(row).v4TightBoxCleared);
        end
    end
end
R=struct('status','PASS_V3_V4_PER_MODE_BENCHMARK', ...
    'records',rows,'goalOneSelectedModeMs',2, ...
    'sixModesAreMutuallyExclusive',true,'geometryOnly',true, ...
    'futurePredictionTimeIncluded',false);
end
function x=getFieldOrZero(s,name)
if isfield(s,name),x=s.(name);else,x=0;end
end
function y=p95(x)
a=sort(x);y=a(ceil(.95*numel(a)));
end
function [S,E]=fixture(s)
switch s
    case 'empty',c=zeros(3,0);a=zeros(3,0);
    case 'distant_36', c=[40+(1:36);15*ones(1,36);2*ones(1,36)];a=repmat([.2;.12;.09],1,36);
    case 'cable_contact', c=[.8;0;2];a=.18*ones(3,1);
    case 'near_noncontact_36', c=[.3+.09*(0:35);.38*ones(1,36);2*ones(1,36)];a=repmat([.2;.12;.09],1,36);
    case 'thin_ellipsoid_36', c=[.3+.09*(0:35);.24*ones(1,36);2*ones(1,36)];a=repmat([.2;.04;.08],1,36);
end
[S,E]=six_test_fixture(c,a);
end
