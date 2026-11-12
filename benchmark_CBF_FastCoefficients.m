function results=benchmark_CBF_FastCoefficients(repeats,counts)
%BENCHMARK_CBF_FASTCOEFFICIENTS 高次係数の旧方式と解析勾配方式を比較。
% 例: results=benchmark_CBF_FastCoefficients(10,[1 3 5 10]);
% 同一のQP係数を持つか先にtest_CBF_FastCoefficientEquivalenceで確認する。
if nargin<1 || isempty(repeats),repeats=10;end
if nargin<2 || isempty(counts),counts=[1 3 5 10];end
assert(repeats>=1 && repeats==floor(repeats));
test_CBF_SetupPaths();s=test_CBF_Fixture();
xR=zeros(29,1);xR(1)=1;xR(8:10)=[0;0;1.5];xR(14:16)=[0;0;-1];
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xV=zeros(20,1);xV(1:3)=[0;0;1.5];
records=cell(2*3*numel(counts),8);ir=0;
for modeCell={'real','virtual'}
    mode=modeCell{1};
    if strcmp(mode,'real'),x=xR;else,x=xV;end
    for shapeCell={'C','B','D'}
        shape=shapeCell{1};
        for n=counts
            obs=makeObstacles(n);
            t=zeros(repeats,2);
            for method=1:2
                s.cbf.fastGeometrySensitivity=(method==2);
                for warm=1:3,cbf_hocbf_scene(mode,shape,x,s,obs);end
                for k=1:repeats
                    t0=tic;cbf_hocbf_scene(mode,shape,x,s,obs);t(k,method)=toc(t0);
                end
            end
            ir=ir+1;slow=median(t(:,1))*1e3;fast=median(t(:,2))*1e3;
            records(ir,:)={mode,shape,n,slow,fast,slow/fast, ...
                max(t(:,1))*1e3,max(t(:,2))*1e3};
            fprintf('%7s %s %2d障害物: 旧 %.3f ms / 新 %.3f ms（%.2fx）\n', ...
                mode,shape,n,slow,fast,slow/fast);
        end
    end
end
results=cell2table(records,'VariableNames', ...
    {'mode','shape','nObstacles','legacyMedianMs','analyticMedianMs', ...
     'speedup','legacyMaxMs','analyticMaxMs'});
paths=test_CBF_SetupPaths();dest=fullfile(paths.generator,'CBF','test_results');
if ~exist(dest,'dir'),mkdir(dest);end
writetable(results,fullfile(dest,'fast_coefficient_benchmark.csv'));
end
function obs=makeObstacles(n)
obs=cell(n,1);
for j=1:n
    c=[4+.7*j;(-1)^j*.4;1.45];
    if mod(j,2)
        obs{j}=struct('type','sphere','center',c,'radius',.2);
    else
        a=.13*j;R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
        obs{j}=struct('type','ellipsoid','center',c, ...
            'axes',[.15;.22;.28],'R',R);
    end
end
end
