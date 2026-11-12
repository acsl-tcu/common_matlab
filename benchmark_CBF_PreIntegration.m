function results=benchmark_CBF_PreIntegration(repeats)
%BENCHMARK_CBF_PREINTEGRATION HLC抜きのQP係数評価とQP解の処理時間を計測。
% 生成コードの性能比較に使う。ベンチマークはテスト合否から分離。
%   benchmark_CBF_PreIntegration(3);
if nargin<1 || isempty(repeats),repeats=3;end
assert(isscalar(repeats) && repeats>=1 && repeats==floor(repeats));
paths=test_CBF_SetupPaths();
if exist('quadprog','file')~=2,error('CBF:MissingQuadprog','quadprogが必要です。');end
s=test_CBF_Fixture();W=eye(4);
obsS=struct('center',[8;0;2],'radius',0.20);
obsE=struct('center',[8;0;2],'axes',[0.15;0.3;0.25], ...
            'R',[0 -1 0;1 0 0;0 0 1]);
xR=zeros(29,1);xR(1)=1;xR(8:10)=[0;0;1.5];xR(14:16)=[0;0;-1];
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xV=zeros(20,1);xV(1:3)=[0;0;1.5];
rows=cell(12,6);index=0;
for im=1:2
    if im==1,mode='real';x=xR;else,mode='virtual';x=xV;end
    for sh={'C','B','D'}
        for typ={'sphere','ellipsoid'}
            index=index+1;
            if strcmp(typ{1},'sphere'),obs=obsS;else,obs=obsE;end
            fn=str2func(sprintf('CBF_%s_%s_%s',mode,sh{1},typ{1}));
            [~,~,qp]=fn(x,s,obs); %#ok<ASGLU> warm-up
            E=zeros(repeats,1);Q=zeros(repeats,1);
            for k=1:repeats
                tic;[~,~,qp]=fn(x,s,obs);E(k)=toc;
                tic;sol=cbf_solve_hocbf_qp(qp,zeros(4,1),W);Q(k)=toc;
                assert(sol.exitflag>0,'ベンチマーク条件でQP解がありません。');
            end
            rows(index,:)={mode,sh{1},typ{1},size(qp.A,1),median(E)*1e3,median(Q)*1e3};
            fprintf('%s %s %s: %d制約, 係数 %.2f ms / QP %.2f ms (中央値)\n', ...
                  mode,sh{1},typ{1},size(qp.A,1),median(E)*1e3,median(Q)*1e3);
        end
    end
end
results=cell2table(rows,'VariableNames',{'inputMode','shape','obstacle','nRows','coeffMs','qpMs'});
folder=fullfile(paths.generator,'CBF','test_results');
if exist(folder,'dir')~=7,mkdir(folder);end
writetable(results,fullfile(folder,'standalone_benchmark.csv'));
fprintf('※ PC性能・初回コンパイル・MATLAB版で変わるため、この数値は安全保証ではありません。\n');
end
