function result = generateCBF_VirtualInput()
%GENERATECBF_VIRTUALINPUT 仮想入力側CBFの幾何関数と4/6次HOCBF係数を生成します。
%
% 【生成時】質量、半径、ケーブル長、障害物の値は一切指定しません。
%   generateCBF_VirtualInput();
%
% 【生成後・実行時】各種数値を引数として渡します。
%   [h,info] = CBF_virtual_C_sphere(xVirtual,settings,obstacle);
%
% 理想的な吊り荷6次平坦出力系を使った幾何安全関数です。
% 理想6次系のCBF行を生成します。既存HLCの逆変換・実入力制限の接続は別途必要です。
% 生成結果はこのファイルと同じ場所の CBF/virtual/ に保存します。

%% 1. 配置場所に依存しないパスを取得
paths = CBF_initialize();

%% 2. CBF本体の保存先を新規作成
outputDir = paths.virtual;
if exist(outputDir,'dir') ~= 7
    mkdir(outputDir);
end
addpath(outputDir);

%% 3. 3形状 × 2障害物 = 6個のCBF本体を生成
shapeNames = {'C','B','D'};
obstacleNames = {'sphere','ellipsoid'};
created = cell(6,1);
index = 0;
fprintf('\n[仮想入力CBF] 生成開始（数値パラメータは埋め込みません）\n');
for i = 1:numel(shapeNames)
    for j = 1:numel(obstacleNames)
        shape = shapeNames{i};
        obstacleType = obstacleNames{j};
        index = index + 1;
        functionName = sprintf('CBF_virtual_%s_%s',shape,obstacleType);
        fileName = fullfile(outputDir,[functionName '.m']);

        fid = fopen(fileName,'w','n','UTF-8');
        if fid < 0
            error('CBF:WriteFailed','生成ファイルに書き込めません: %s',fileName);
        end
        fprintf(fid,'function [h,info,qp] = %s(x,settings,obstacle)\n',functionName);
        fprintf(fid,'%%%s  3次元・仮想入力系の幾何安全関数（実行時パラメータ指定）。\n',upper(functionName));
        fprintf(fid,'%% x:理想平坦出力状態20要素 / settings:形状・物理値・制限 / obstacle:球または回転楕円体\n');
        fprintf(fid,'%% ケーブル球数は実行時のcableLengthとmaxExtraRadiusから算出します。\n');
        fprintf(fid,'%% 出力1,2:幾何hと情報。出力3: A*nu>=b の高次CBF条件。\n');
        fprintf(fid,'[h,info] = cbf_evaluate_case(''virtual'',''%s'',''%s'',x,settings,obstacle);\n',shape,obstacleType);
        fprintf(fid,'if nargout>=3\n');
        fprintf(fid,'    qp=cbf_hocbf_evaluate(''%s'',''%s'',''%s'',x,settings,obstacle,h,info);\n','virtual',shape,obstacleType);
        fprintf(fid,'end\n');
        fprintf(fid,'end\n');
        fclose(fid);

        created{index} = fileName;
        fprintf('  (%d/6) %s\n',index,functionName);
    end
end

rehash; % MATLABに新しく生成したCBF関数を認識させる

%% 4. 実行結果
result.mode = 'virtual';
result.generatedFiles = created;
result.outputDirectory = outputDir;
result.note = '数値は実行時指定。第3出力でHOCBF行(A*nu>=b)を取得。可行性は別途確認。';
fprintf('[仮想入力CBF] 完了: %s\n\n',outputDir);
end
