function test_CBF_MEX_Status()
%TEST_CBF_MEX_STATUS 生成済み.m関数とMATLAB Coderの有無を確認。
% MEXをコンパイルしたり、コードを変更したりはしない。
test_CBF_SetupPaths();
files={'CBF_real_B_sphere','CBF_virtual_B_sphere','cbf_hocbf_scene'};
for i=1:numel(files)
    f=files{i};
    fprintf('%s: %s\n',f,which(f));
    fprintf('  MEX候補: %s\n',which([f '_mex']));
end
if exist('codegen','file')==2
    fprintf('MATLAB Coder codegen コマンドは見つかりました。\n');
else
    fprintf('MATLAB Coder codegen コマンドは見つかりません。\n');
end
fprintf(['現行のgenerateCBF_*は.mファイル出力で、MEXコンパイルはしていません。\n' ...
 'C/C++へのコード生成には関数の対応可否・可変サイズ構造体・メモリ割当の確認が必要です。\n']);
end
