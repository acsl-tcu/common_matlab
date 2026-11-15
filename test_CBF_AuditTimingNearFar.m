function report=test_CBF_AuditTimingNearFar(options)
%TEST_CBF_AUDITTIMINGNEARFAR 障害物を「省略せず」、距離と数別に実時間計測。
% 制約数・入力方式・システムC/B/D・球/楕円体を独立に比較する。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
counts=getopt(options,'nObstacles',[1 3 5 10]);
repeats=getopt(options,'repeats',12);
shapes=getopt(options,'shapes',{'C','B','D'});
rows=cell(0,14);
xR=CBF_Audit_State('real',s,1.5,0,0,0,0,0,0);
xV=CBF_Audit_State('virtualfromreal',xR,s);
for im=1:2
    if im==1,mode='real';x=xR;else,mode='virtual';x=xV;end
    for sh=shapes
        shape=sh{1};
        for kindCell={'sphere','ellipsoid'}
            kind=kindCell{1};
            for distanceCell={'near','far'}
                distanceName=distanceCell{1};
                for n=counts
                    obs=cell(1,n);
                    for j=1:n
                        if strcmp(distanceName,'near')
                            d=.58+.16*mod(j-1,4);
                        else
                            d=4+.35*j;
                        end
                        % 同一中心を重ねず、数値的に異なる制約を与える。
                        base=xR(8:10)+[0;.20*sin(j);.15*cos(j)];
                        obs{j}=CBF_Audit_State('obstacle',kind,base,d, ...
                            'front',.15+.09*j);
                    end
                    jetMs=zeros(repeats,1);coeffMs=zeros(repeats,1);
                    qpMs=zeros(repeats,1);totalMs=zeros(repeats,1);
                    nRows=NaN;nOK=0;
                    % JIT等の最初の実行コストを分けるウォームアップ。
                    for k=1:3
                        qp=cbf_hocbf_scene(mode,shape,x,s,obs);
                        cbf_solve_hocbf_qp(qp,zeros(4,1),eye(4));
                    end
                    for k=1:repeats
                        tall=tic;
                        [qp,ct]=cbf_hocbf_scene(mode,shape,x,s,obs);
                        coef=toc(tall)*1000;
                        [sol,qt]=cbf_solve_hocbf_qp(qp,zeros(4,1),eye(4));
                        elapsed=toc(tall)*1000;
                        jetMs(k)=ct.jetMs;
                        coeffMs(k)=coef;
                        qpMs(k)=qt.totalMs;
                        totalMs(k)=elapsed;
                        nRows=size(qp.A,1);
                        nOK=nOK+(sol.exitflag>0 && sol.minResidual>=-1e-7);
                    end
                    sorted=sort(totalMs);
                    p95=sorted(max(1,ceil(.95*repeats)));
                    rows(end+1,:)={mode,shape,kind,distanceName,n,nRows, ...
                        median(jetMs),median(coeffMs),median(qpMs), ...
                        median(totalMs),p95,max(totalMs),repeats,nOK}; %#ok<AGROW>
                    fprintf('%-7s %s %-9s %-4s N=%2d  median=%6.2f ms  p95=%6.2f ms\n', ...
                        mode,shape,kind,distanceName,n,median(totalMs),p95);
                end
            end
        end
    end
end
report=cell2table(rows,'VariableNames',{'mode','shape','obstacleType', ...
    'distanceCategory','nObstacles','nRows', 'jetMedianMs', ...
    'coefficientMedianMs','qpMedianMs','totalMedianMs','totalP95Ms', ...
    'totalMaxMs','repetitions','qpSuccesses'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'audit_timing_near_far.csv'));
fprintf('[速度監査] 全障害物を評価。近距離でのQP不可行は速度の失敗と区別する。\n');
end
function v=getopt(o,k,d)
if isfield(o,k)&&~isempty(o.(k)),v=o.(k);else,v=d;end
end
