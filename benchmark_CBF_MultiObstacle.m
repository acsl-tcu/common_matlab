function results = benchmark_CBF_MultiObstacle(repeats, obstacleCounts)
%BENCHMARK_CBF_MULTIOBSTACLE 既存CBF生成器を変更せず障害物数に対する計算時間を測る。
%
% 配置: common_matlab/benchmark_CBF_MultiObstacle.m
% 使用:
%   results = benchmark_CBF_MultiObstacle();
%   results = benchmark_CBF_MultiObstacle(10,[1 3 5 10]);
%
% 重要事項:
% ・障害物を間引かず、渡した全ての障害物の制約を生成する。
% ・生成済みCBF関数が存在する状態で実行する。
% ・静止状態の1周期ベンチマークであり、閉ループ回避や安全保証の試験ではない。
% ・地面/入力上下限の処理を含む現行実装の所要時間を計測する。
% ・平均ではなく中央値。100Hz等への適合には最大値・99パーセンタイルも必要。

if nargin<1 || isempty(repeats), repeats=5; end
if nargin<2 || isempty(obstacleCounts), obstacleCounts=[1 3 5 10]; end
assert(isscalar(repeats) && isfinite(repeats) && repeats==floor(repeats) && repeats>=1, ...
    'repeatsは1以上の整数を指定してください。');
assert(isnumeric(obstacleCounts) && isvector(obstacleCounts) && ...
    all(isfinite(obstacleCounts)) && all(obstacleCounts>=1) && ...
    all(mod(obstacleCounts,1)==0),'障害物数は1以上の整数配列です。');

paths=test_CBF_SetupPaths();
s=test_CBF_Fixture();
W=eye(4);
xReal=zeros(29,1);
xReal(1)=1;                   % 姿勢 quaternion
xReal(8:10)=[0;0;1.5];       % 吊り荷位置
xReal(14:16)=[0;0;-1];       % 機体→吊り荷のケーブル方向
xReal(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xVirtual=zeros(20,1);
xVirtual(1:3)=[0;0;1.5];

nTotal=2*3*numel(obstacleCounts);
record=cell(nTotal,13);
idx=0;
for im=1:2
    if im==1, mode='real'; x=xReal;
    else, mode='virtual'; x=xVirtual; end
    for shapeCell={'C','B','D'}
        shape=shapeCell{1};
        for n=obstacleCounts(:).'
            obs=buildFarObstacles(n);
            % 関数ロードと最初の評価を計時対象から外す（既存ベンチと同条件）。
            for iwarm=1:3
                qp=cbf_hocbf_scene(mode,shape,x,s,obs);
                tmp=cbf_solve_hocbf_qp(qp,zeros(4,1),W); %#ok<NASGU>
            end
            tCoef=zeros(repeats,1);tQP=zeros(repeats,1);tTotal=zeros(repeats,1);
            exitflags=zeros(repeats,1);
            residuals=nan(repeats,1);
            for k=1:repeats
                wholeStart=tic;
                t0=tic;
                qp=cbf_hocbf_scene(mode,shape,x,s,obs);
                tCoef(k)=toc(t0);
                t0=tic;
                sol=cbf_solve_hocbf_qp(qp,zeros(4,1),W);
                tQP(k)=toc(t0);
                tTotal(k)=toc(wholeStart);
                exitflags(k)=sol.exitflag;
                if sol.exitflag>0,residuals(k)=sol.minResidual;end
            end
            idx=idx+1;
            record(idx,:)={mode,shape,n,size(qp.A,1), ...
                median(tCoef)*1000,median(tQP)*1000,median(tTotal)*1000, ...
                max(tTotal)*1000,sum(exitflags<=0),min(exitflags), ...
                min(residuals,[],'omitnan'),localPercentile(tTotal,95)*1000, ...
                max(tCoef+tQP)*1000};
            fprintf('%7s %s, 障害物%2d個: 行%3d  係数%7.2fms + QP%7.2fms / 全体%7.2fms, 失敗%d回\n', ...
                mode,shape,n,size(qp.A,1),median(tCoef)*1000,median(tQP)*1000, ...
                median(tTotal)*1000,sum(exitflags<=0));
        end
    end
end
results=cell2table(record,'VariableNames',{'inputMode','shape','nObstacles', ...
    'nRows','coefficientMedianMs','qpMedianMs','totalMedianMs', ...
    'totalMaxMs','nQPFailures','minimumExitflag','minimumSuccessfulResidual', ...
    'totalP95Ms','sumOfStagesMaxMs'});
folder=fullfile(paths.generator,'CBF','test_results');
if exist(folder,'dir')~=7,mkdir(folder);end
filename=fullfile(folder,'multi_obstacle_benchmark.csv');
writetable(results,filename);
fprintf('\n結果CSV: %s\n',filename);
fprintf('注意: 障害物は遠方に置いた固定姿勢の球/楕円体混在。衝突回避性能・可行性保証の検証ではありません。\n');
end

function obs=buildFarObstacles(n)
% 位置を固定することで回ごとに同じ比較条件とする。
% 常に遠方: 計算時間を測るテスト。接近・QP実行不能を試すテストとは分離する。
obs=cell(n,1);
for j=1:n
    center=[8+2.0*(j-1); (-1)^j*0.7; 1.6+0.1*mod(j,3)];
    if mod(j,2)==1
        obs{j}=struct('type','sphere','center',center,'radius',0.18);
    else
        angle=0.19*j; c=cos(angle); ss=sin(angle);
        R=[c,-ss,0;ss,c,0;0,0,1];
        obs{j}=struct('type','ellipsoid','center',center, ...
            'axes',[0.15;0.26;0.34],'R',R);
    end
end
end

function v=localPercentile(x,p)
% 統計ツールボックスなしの最近順位パーセンタイル。
x=sort(x(:));
v=x(max(1,min(numel(x),ceil(p/100*numel(x)))));
end
