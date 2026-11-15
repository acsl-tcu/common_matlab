function report=test_CBF_AuditRelativeDegree(options)
%TEST_CBF_AUDITRELATIVEDEGREE 現行モデルに対する入力作用の初出階数を独立チェック。
% 目標: 一律6次などの指定値を鵜呑みにせず、0～6次の入力作用を実測。
% 既存CBF本体・QPは書き換えない。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};
rows=cell(0,15);checks=0;badAffine=0;earlyCases=0;
% 平衡、下降、傾斜、ケーブル傾斜・角速度の複数条件
configs=[1.50 0 0 0 0 0; ...
         1.00 -.45 0 0 0 0; ...
         2.00 .30 .15 .12 .25 0.20; ...
         1.40 -.50 -.20 .18 -.30 .15; ...
         3.00 0 .27 0 .20 0.05];
P=cbf_parameter_vector(s);plan=cbf_choose_cable_spheres(s);
for i=1:size(configs,1)
    c=configs(i,:);
    xR=CBF_Audit_State('real',s,c(1),c(2),.12*sin(i),c(3), ...
         c(4),c(5),c(6),.15*i);
    xV=CBF_Audit_State('virtualfromreal',xR,s);
    for im=1:2
        if im==1,mode='real';x=xR;else,mode='virtual';x=xV;end
        cache=cbf_precompute_jets(mode,x,s);
        for typ=kinds
            kind=typ{1};
            ob=CBF_Audit_State('obstacle',kind,xR(8:10),.9,'front',.51);
            O=cbf_obstacle_vector(ob,kind);
            for shapeCell=shapes
                shape=shapeCell{1};
                [h,info]=cbf_evaluate_case(mode,shape,kind,x,s,ob);
                H=cell(5,1);
                for k=1:5
                    J=cache.jets{k};
                    H{k}=cbf_geometry_taylor(shape,kind, ...
                        J.pLoad,J.qCable,J.pDrone,P,O,plan,'all');
                end
                assert(all(size(H{1})==[numel(h),7]),'Taylor形式が不一致');
                if norm(H{1}(:,1)-h,Inf)>1e-7*(1+norm(h,Inf))
                    error('CBF:AuditZeroOrder','0階のhが幾何と一致しません。');
                end
                % 単位基底の重ね合わせが高階の一次入力作用と整合するか。
                u=[.3;-.2;.17;.11];
                if strcmp(mode,'real')
                    Z=cbf_real_state_taylor(x,u,P,6);
                    J=struct('pLoad',Z(8:10,:), 'qCable',Z(14:16,:), ...
                        'pDrone',Z(8:10,:)-s.physical.cableLength*Z(14:16,:));
                else
                    [pl,qc,pd]=cbf_virtual_state_taylor(x,u,P,6);
                    J=struct('pLoad',pl,'qCable',qc,'pDrone',pd);
                end
                Hu=cbf_geometry_taylor(shape,kind,J.pLoad,J.qCable, ...
                    J.pDrone,P,O,plan,'all');
                expected=H{1};
                for k=1:4,expected=expected+u(k)*(H{k+1}-H{1});end
                affErr=norm(Hu-expected,Inf);
                checks=checks+1;
                if affErr>1e-8*(1+norm(Hu,Inf)),badAffine=badAffine+1;end
                for j=1:numel(h)
                    if strcmp(mode,'real'),r=6;else,r=info.orderHint(j);end
                    maxLow=0;coeff=zeros(1,4);first=nan(1,4);
                    for k=1:4
                        change=(H{k+1}(j,:)-H{1}(j,:)).*factorial(0:6);
                        maxLow=max(maxLow,max(abs(change(1:r))));
                        coeff(k)=change(r+1);
                        tol=1e-8*(1+max(abs(H{1}(j,:).*factorial(0:6))));
                        found=find(abs(change(2:7))>tol,1,'first');
                        if ~isempty(found),first(k)=found;end
                    end
                    if maxLow>1e-7,earlyCases=earlyCases+1;end
                    rows(end+1,:)={mode,shape,kind,i,info.labels{j},r, ...
                        min(h),h(j),maxLow,norm(coeff), ...
                        first(1),first(2),first(3),first(4),affErr}; %#ok<AGROW>
                end
            end
        end
    end
end
report=cell2table(rows,'VariableNames',{'mode','shape','obstacleType', ...
 'stateCase','barrier','designOrder','minHInCase','h', ...
 'maxInputEffectBelowDesignOrder','decisionEffectNorm', ...
 'firstNuTEffectOrder','firstNuTauXEffectOrder', ...
 'firstNuTauYEffectOrder','firstNuTauZEffectOrder','affineError'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'audit_relative_degree.csv'));
fprintf('[次数監査] %d状態・方式の係数検証、6階までの行=%d、早期入力作用行=%d、アフィン不一致ケース=%d\n', ...
    checks,size(report,1),earlyCases,badAffine);
if badAffine>0
    warning('CBF:AuditAffine','入力アフィン性の不一致が検出されました。');
end
end
