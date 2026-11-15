function results=benchmark_CBF_DetailedProfile(repeats,obstacleCounts)
%BENCHMARK_CBF_DETAILEDPROFILE V5のCBF係数・QPを内部段階まで時間計測。
%
%  球/回転楕円体は別々のシーン・CSVに分離。混合しない。
%  ベンチマーク条件は遠方の静止障害物、ホバリング状態、名目入力0。
%  「計測値は計測用のtic/tocのオーバーヘッドを含む」ことに注意。
%  MATLAB Profilerによる関数別内訳はprofile_CBF_Hotspots()を利用。
%
%  使用例:
%   test_CBF_ProfilingEquivalence();
%   results = benchmark_CBF_DetailedProfile(20,[1 3 5 10]);
%   results = benchmark_CBF_DetailedProfile(5,[1 3]);
if nargin<1 || isempty(repeats),repeats=10;end
if nargin<2 || isempty(obstacleCounts),obstacleCounts=[1 3 5 10];end
validateattributes(repeats,{'numeric'},{'scalar','integer','positive','finite'});
validateattributes(obstacleCounts,{'numeric'},{'vector','integer','positive','finite'});

%% 1. 環境、試験状態
paths=test_CBF_SetupPaths();
s=test_CBF_Fixture(); W=eye(4);
xReal=zeros(29,1);xReal(1)=1;
xReal(8:10)=[0;0;1.5]; xReal(14:16)=[0;0;-1];
xReal(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xVirtual=zeros(20,1);xVirtual(1:3)=[0;0;1.5];

%% 2. 計測する段階とCSV列
% coefTotal = [jet + hGeometry + hocbf + merge + prune]
%   jet           : 元の力学から6階までのTaylor級数（入力0と4方向）
%   hGeometry     : 通常の安全関数h(x)の数値計算
%   zeroTaylor    : 各障害物の0入力の高次幾何Taylor評価
%   basisTaylor   : 各障害物の4入力方向の高次幾何Taylor評価
%   hocbfAssembly : (D+k)^r h の組み立てと補助条件
%   realInput     : 実入力の推力・トルク上下限HOCBF
%   otherHocbf    : 係数計算のパラメータ化とモデル診断等
%   merge/prune   : 制約配列結合と証明済み冗長地面行の整理
% qpTotal = [validation + options + quadprog + postprocess]
%   validation    : 引数チェック・Wの正定値検査
%   options       : optimoptions生成
%   quadprog      : QP本体の呼び出し（MATLAB/Toolbox内部を含む）
%   postprocess   : 解の検査と残差計算
% メディアンの和は必ずしも全体メディアンと一致しない。
fields={'inputMode','bodyShape','obstacleType','nObstacles','nRows', ...
    'jetMs','hGeometryMs','zeroTaylorMs','basisTaylorMs', ...
    'hocbfAssemblyMs','inputBoundsMs','otherHocbfMs','mergeMs','pruneMs', ...
    'coefficientTotalMs','qpValidationMs','qpOptionsMs', 'quadprogMs', ...
    'qpPostprocessMs','qpTotalMs','totalMedianMs','totalP95Ms','totalMaxMs', ...
    'coefMaxMs','qpMaxMs','nQPFailures','minExitflag'};
records=cell(2*3*2*numel(obstacleCounts),numel(fields)); row=0;

%% 3. 球のみ / 任意姿勢楕円体のみで完全分離して評価
for modeCell={'real','virtual'}
    mode=modeCell{1};
    if strcmp(mode,'real'),x=xReal;else,x=xVirtual;end
    for shapeCell={'C','B','D'}
        shape=shapeCell{1};
        for kindCell={'sphere','ellipsoid'}
            kind=kindCell{1};
            fprintf('\n--- %s / 機体形状 %s / 障害物 %s のみ ---\n',mode,shape,kind);
            for n=obstacleCounts(:).'
                obs=makeProfileObstacles(n,kind);
                % 初回ロード・JIT等を計測から外す。ただしホバリング1状態の結果。
                for iw=1:3
                    qp=cbf_hocbf_scene(mode,shape,x,s,obs);
                    sol=cbf_solve_hocbf_qp(qp,zeros(4,1),W); %#ok<NASGU>
                end
                times=zeros(repeats,16);flags=zeros(repeats,1);
                for k=1:repeats
                    tAll=tic;
                    [qp,co]=cbf_hocbf_scene(mode,shape,x,s,obs);
                    [sol,qs]=cbf_solve_hocbf_qp(qp,zeros(4,1),W);
                    total=1000*toc(tAll);
                    flags(k)=sol.exitflag;
                    % 内側タイマーは外側の時間の内訳。処理の重複加算なし。
                    otherHocbf=co.parameterMs+co.diagnosticsMs+ ...
                         max(0,co.highOrderMs-co.zeroTaylorMs-co.fourBasisTaylorMs- ...
                             co.hocbfAssemblyMs-co.realInputBoundsMs- ...
                             co.parameterMs-co.diagnosticsMs);
                    times(k,:)=[co.jetMs,co.geometryMs,co.zeroTaylorMs, ...
                        co.fourBasisTaylorMs,co.hocbfAssemblyMs, ...
                        co.realInputBoundsMs,otherHocbf,co.mergeMs,co.pruneMs, ...
                        co.totalMs,qs.validationMs,qs.optionsMs,qs.quadprogMs, ...
                        qs.postprocessMs,qs.totalMs,total];
                end
                mm=median(times,1);
                row=row+1;
                records(row,:)={mode,shape,kind,n,size(qp.A,1), ...
                    mm(1),mm(2),mm(3),mm(4),mm(5),mm(6),mm(7), ...
                    mm(8),mm(9),mm(10),mm(11),mm(12),mm(13),mm(14),mm(15), ...
                    mm(16),localP95(times(:,16)),max(times(:,16)), ...
                    max(times(:,10)),max(times(:,15)),sum(flags<=0),min(flags)};
                fprintf('%2d障害物 %3d行 | 力学 %.2f + h %.2f + Taylor0 %.2f + Taylor4 %.2f + QP(設定 %.2f,求解 %.2f) = 全体 %.2f ms\n', ...
                    n,size(qp.A,1),mm(1),mm(2),mm(3),mm(4),mm(12),mm(13),mm(16));
                if any(flags<=0)
                    fprintf('  [警告] QP失敗 %d/%d 回。失敗時の求解時間は参考値です。\n',sum(flags<=0),repeats);
                end
            end
        end
    end
end
results=cell2table(records,'VariableNames',fields);

%% 4. 出力: 全体・球のみ・楕円体のみを別CSVに保存
folder=fullfile(paths.generator,'CBF','test_results');
if exist(folder,'dir')~=7,mkdir(folder);end
writetable(results,fullfile(folder,'CBF_profile_all.csv'));
writetable(results(strcmp(results.obstacleType,'sphere'),:), ...
    fullfile(folder,'CBF_profile_sphere.csv'));
writetable(results(strcmp(results.obstacleType,'ellipsoid'),:), ...
    fullfile(folder,'CBF_profile_ellipsoid.csv'));
fprintf('\n[保存先] %s\n',folder);
fprintf('  CBF_profile_sphere.csv     : 球障害物のみ\n');
fprintf('  CBF_profile_ellipsoid.csv  : 楕円体障害物のみ\n');
fprintf('  CBF_profile_all.csv        : 両者の比較用\n');
fprintf('※ 段階ごと中央値の合計と全体中央値は必ずしも一致しません。\n');
fprintf('※ 計測用タイマーによる追加オーバーヘッドあり。最終性能は従来版も別途測定してください。\n');
end

function val=localP95(x)
x=sort(x(:));val=x(max(1,min(numel(x),ceil(numel(x)*0.95))));
end
