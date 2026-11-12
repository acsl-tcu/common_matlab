function results=benchmark_CBF_SceneOptimization(repeats,obstacleCounts)
%BENCHMARK_CBF_SCENEOPTIMIZATION 旧ループと重複計算除去後を同じ条件で時間比較。
% 例: benchmark_CBF_SceneOptimization(10,[1 3 5 10]);
% 各試験は同じQP行（pruneRedundantGround=false）を使う。
% CBF係数の計算時間のみ比較。QP含む総時間はbenchmark_CBF_MultiObstacleを参照。
if nargin<1 || isempty(repeats),repeats=10;end
if nargin<2 || isempty(obstacleCounts),obstacleCounts=[1 3 5 10];end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
s.cbf.pruneRedundantGround=false;
xR=zeros(29,1);xR(1)=1;xR(8:10)=[0;0;1.5];
xR(14:16)=[0;0;-1];
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xV=zeros(20,1);xV(1:3)=[0;0;1.5];
row=0;tbl=cell(2*3*numel(obstacleCounts),9);
for modeCell={'real','virtual'}
    mode=modeCell{1};
    if strcmp(mode,'real'),x=xR;else,x=xV;end
    for shapeCell={'C','B','D'}
        shape=shapeCell{1};
        for n=obstacleCounts(:).'
            obs=localObstacles(n);
            a=test_CBF_BaselineScene(mode,shape,x,s,obs);
            b=cbf_hocbf_scene(mode,shape,x,s,obs);
            assert(isequal(a.labels,b.labels));
            assert(norm(a.A(:)-b.A(:),Inf)<1e-7*(1+norm(a.A(:),Inf)));
            assert(norm(a.b(:)-b.b(:),Inf)<1e-7*(1+norm(a.b(:),Inf)));
            tOld=zeros(repeats,1);tNew=zeros(repeats,1);
            for k=1:repeats
                if mod(k,2)
                    tic; test_CBF_BaselineScene(mode,shape,x,s,obs);tOld(k)=toc;
                    tic; cbf_hocbf_scene(mode,shape,x,s,obs);tNew(k)=toc;
                else
                    tic; cbf_hocbf_scene(mode,shape,x,s,obs);tNew(k)=toc;
                    tic; test_CBF_BaselineScene(mode,shape,x,s,obs);tOld(k)=toc;
                end
            end
            row=row+1;
            tbl(row,:)={mode,shape,n,numel(b.b),1000*median(tOld),1000*median(tNew), ...
                1000*max(tOld),1000*max(tNew),median(tOld)/median(tNew)};
            fprintf('%7s %s %2d障害物: 旧%.2f ms -> 新%.2f ms（%.2fx）\n', ...
                mode,shape,n,1000*median(tOld),1000*median(tNew), ...
                median(tOld)/median(tNew));
        end
    end
end
results=cell2table(tbl,'VariableNames', ...
 {'mode','shape','obstacles','nRows','oldMedianMs','newMedianMs', ...
 'oldMaxMs','newMaxMs','ratioOldOverNew'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(results,fullfile(folder,'scene_optimization_benchmark.csv'));
end
function obs=localObstacles(n)
obs=cell(n,1);
for j=1:n
    p=[8+1.6*(j-1);(-1)^j*.4;1.5];
    if mod(j,2)
        obs{j}=struct('type','sphere','center',p,'radius',.19);
    else
        a=.19*j;b=.12*j;
        Rz=[cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];
        Rx=[1,0,0;0,cos(b),-sin(b);0,sin(b),cos(b)];
        obs{j}=struct('type','ellipsoid','center',p, ...
            'axes',[.18;.23;.3],'R',Rz*Rx);
    end
end
end
