function summary=profile_CBF_Hotspots(mode,shape,kind,n,repeats)
%PROFILE_CBF_HOTSPOTS MATLAB組込みProfilerで時間を使う関数を特定する。
% この試験ではProfiler自体のオーバーヘッドが大きいため、msの絶対値は
% benchmark_CBF_DetailedProfileの結果と比較しないこと。
%
% 例:
% profile_CBF_Hotspots('real','B','ellipsoid',5,5);
% profile_CBF_Hotspots('virtual','C','sphere',5,5);
if nargin<1 || isempty(mode),mode='real';end
if nargin<2 || isempty(shape),shape='B';end
if nargin<3 || isempty(kind),kind='ellipsoid';end
if nargin<4 || isempty(n),n=5;end
if nargin<5 || isempty(repeats),repeats=5;end
assert(any(strcmp(mode,{'real','virtual'})),'realまたはvirtualを指定');
assert(any(strcmp(shape,{'C','B','D'})),'C/B/Dを指定');
assert(any(strcmp(kind,{'sphere','ellipsoid'})),'sphereまたはellipsoidを指定');
validateattributes(n,{'numeric'},{'scalar','integer','positive'});
validateattributes(repeats,{'numeric'},{'scalar','integer','positive'});

paths=test_CBF_SetupPaths();s=test_CBF_Fixture();W=eye(4);
if strcmp(mode,'real')
    x=zeros(29,1);x(1)=1;x(8:10)=[0;0;1.5];x(14:16)=[0;0;-1];
    x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
else
    x=zeros(20,1);x(1:3)=[0;0;1.5];
end
obs=makeProfileObstacles(n,kind);
for k=1:3
    qp=cbf_hocbf_scene(mode,shape,x,s,obs);
    sol=cbf_solve_hocbf_qp(qp,zeros(4,1),W); %#ok<NASGU>
end
% プロファイル有効化前にclear: 前回の別ケースの数値を混ぜない。
profile('off');profile('clear');profile('on');
try
    for k=1:repeats
        qp=cbf_hocbf_scene(mode,shape,x,s,obs);
        sol=cbf_solve_hocbf_qp(qp,zeros(4,1),W); %#ok<NASGU>
    end
catch ME
    profile('off');rethrow(ME);
end
profile('off');
info=profile('info');
ft=info.FunctionTable;
if isempty(ft)
    warning('CBF:ProfileEmpty','Profilerの関数記録がありません。');
    summary=table();return;
end
names=cell(numel(ft),1);total=zeros(numel(ft),1);calls=zeros(numel(ft),1);
for k=1:numel(ft)
    names{k}=ft(k).FunctionName;
    total(k)=ft(k).TotalTime*1000;
    calls(k)=ft(k).NumCalls;
end
summary=table(names,total,calls, ...
    'VariableNames',{'functionName','totalTimeMs','numCalls'});
summary=sortrows(summary,'totalTimeMs','descend');
folder=fullfile(paths.generator,'CBF','test_results');
if exist(folder,'dir')~=7,mkdir(folder);end
filename=fullfile(folder,sprintf('CBF_matlab_profiler_%s_%s_%s_%d.csv', ...
    mode,shape,kind,n));
writetable(summary,filename);
fprintf('\nMATLAB Profiler 上位15関数：%s/%s/%s/%d障害物\n',mode,shape,kind,n);
disp(summary(1:min(15,height(summary)),:));
fprintf('保存先: %s\n',filename);
fprintf('注意: Profiler計測中の時間は通常の実行時間を表しません。\n');
% 必要なら MATLAB側で「profile viewer」を実行して呼び出し階層を確認。
end
