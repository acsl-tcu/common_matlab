function report=test_CBF_AuditRollouts(options)
%TEST_CBF_AUDITROLLOUTS 元のCBF/QPを変更せず短時間の実/仮想閉ループ。
% ホバリング、下降、前方/側方障害物への接近、傾いた状態を比較する。
% 名目入力はHLCではない。補助集合/入力/張力/距離が崩れたらそのまま記録。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
steps=getopt(options,'steps',15);dt=getopt(options,'dt',.01);
shapes=getopt(options,'shapes',{'B','D'});
kinds=getopt(options,'obstacleTypes',{'sphere','ellipsoid'});
scenarios={'hover_only','descent_only','front_slow','front_fast','side_entry','tilt_entry'};
rows=cell(0,19);id=0;
for modeCell={'real','virtual'}
    mode=modeCell{1};
    for sh=shapes
        shape=sh{1};
        for kindCell=kinds
            kind=kindCell{1};
            for nameCell=scenarios
                name=nameCell{1};id=id+1;
                [x,obs,groundOnly]=scenarioState(mode,name,s,kind);
                firstFail='';status='completed';completed=0;
                minH=inf;minPsi=inf;minResidual=inf;
                maxNu=0;minTension=inf;maxAccel=0;
                firstQPFails=0;minActualH=inf;
                % 初期補助集合違反で即停止しても、幾何hは必ず保存する。
                [hInitial,labelsInitial]=cbf_evaluate_scene(mode,shape,x,s,obs);
                if groundOnly,hInitial=hInitial(contains(labelsInitial,'ground_'));end
                minH=min(hInitial);minActualH=minH;
                for k=1:steps
                    try
                        qp=cbf_hocbf_scene(mode,shape,x,s,obs);
                        % 地面単独モード: 遠方のダミー障害物CBFを除く。
                        if groundOnly
                            keep=contains(qp.labels,'ground_') | ...
                                contains(qp.labels,'_thrust_') | contains(qp.labels,'_tau_');
                            qp=selectRows(qp,keep);
                        end
                        minPsi=min(minPsi,min(cellfun(@min,qp.psi)));
                        if ~all(qp.psiInitiallyNonnegative)
                            bad=find(~qp.psiInitiallyNonnegative,1);
                            firstFail=qp.labels{bad};
                            status='initial_HOCBF_set_violation';break;
                        end
                        nom=zeros(4,1);% 物理平衡からの逸脱を観察。HLCを模倣しない。
                        sol=cbf_solve_hocbf_qp(qp,nom,eye(4));
                        if sol.exitflag<=0 || ~isfinite(sol.minResidual) || ...
                           sol.minResidual<-1e-7
                            status='qp_infeasible_or_numerical';firstQPFails=k;
                            if strcmp(mode,'real')
                                gg=contains(qp.labels,'ground_');
                                ii=contains(qp.labels,'_thrust_') | ...
                                   contains(qp.labels,'_tau_');
                                lpGI=CBF_Audit_LP(qp,gg|ii);
                                if ~lpGI.feasible
                                    firstFail='ground_and_input_conflict';
                                else
                                    firstFail='other_QP_or_numerical';
                                end
                            else
                                firstFail='virtual_QP';
                            end
                            break;
                        end
                        maxNu=max(maxNu,norm(sol.nu,Inf));
                        minResidual=min(minResidual,sol.minResidual);
                        % 制限のない仮想決定入力は実行前に異常として報告。
                        % 人工のクリップや実入力制限の代用はしない。
                        if strcmp(mode,'virtual') && norm(sol.nu,Inf)>1e5
                            status='unbounded_virtual_input';firstFail='large_virtual_nu';break;
                        end
                        if strcmp(mode,'real')
                            xn=CBF_Audit_State('rk4real',x,sol.nu,dt,s);
                            diag=cbf_model_diagnostics('real',xn,s);
                            if any(~isfinite(xn)) || ...
                               diag.quaternionNormError>1e-5 || ...
                               diag.cableDirectionNormError>1e-5
                                status='state_or_unit_norm_invalid';break;
                            end
                            if ~diag.thrustInBounds || ~diag.torqueInBounds
                                status='actual_input_bounds_violation';break;
                            end
                            if diag.tension<=0 || ~isfinite(diag.tension)
                                status='nonpositive_cable_tension';break;
                            end
                            minTension=min(minTension,diag.tension);
                            accel=test_CBF_IndependentPlantRHS(xn,sol.nu,s);
                            maxAccel=max(maxAccel,norm(accel(11:13),Inf));
                        else
                            xn=CBF_Audit_State('stepvirtual',x,sol.nu,dt);
                            diag=cbf_model_diagnostics('virtual',xn,s);
                            if any(~isfinite(xn)) || ~diag.flatMapValid || ...
                               norm(xn(1:18),Inf)>1e5
                                status='virtual_state_or_flat_map_invalid';break;
                            end
                        end
                        [hNext,~]=cbf_evaluate_scene(mode,shape,xn,s,obs);
                        if groundOnly
                            % 幾何安全関数のうち地面ラベルのみを監視。
                            [~,labs]=cbf_evaluate_scene(mode,shape,xn,s,obs);
                            if isstruct(labs),labs=labs.labels;end
                            hNext=hNext(contains(labs,'ground_'));
                        end
                        minH=min(minH,min(hNext));
                        minActualH=min(minActualH,min(hNext));
                        if any(~isfinite(hNext)) || min(hNext)<-1e-7
                            status='geometric_violation_after_integration';break;
                        end
                        x=xn;completed=k;
                    catch err
                        status='coefficient_or_integration_error';firstFail=err.message;break;
                    end
                end
                if completed==steps && strcmp(status,'completed')
                    % 最終積分点についても補助安全集合をチェック。
                    try
                        qp=cbf_hocbf_scene(mode,shape,x,s,obs);
                        if groundOnly
                            keep=contains(qp.labels,'ground_') | ...
                                contains(qp.labels,'_thrust_') | contains(qp.labels,'_tau_');
                            qp=selectRows(qp,keep);
                        end
                        if ~all(qp.psiInitiallyNonnegative)
                            status='final_HOCBF_set_violation';
                            firstFail=qp.labels{find(~qp.psiInitiallyNonnegative,1)};
                        end
                    catch err
                        status='final_coefficient_error';firstFail=err.message;
                    end
                end
                rows(end+1,:)={id,mode,shape,kind,name,groundOnly,steps, ...
                    completed,status,firstFail,firstQPFails, ...
                    minH,minPsi,minResidual,maxNu,minTension,maxAccel, ...
                    minActualH,dt}; %#ok<AGROW>
                fprintf('%-7s %s %-9s %-13s %2d/%2d  %-36s\n', ...
                    mode,shape,kind,name,completed,steps,status);
            end
        end
    end
end
report=cell2table(rows,'VariableNames',{'caseId','mode','shape', ...
    'obstacleType','scenario','groundOnly','steps','completedSteps', ...
    'status','firstProblem','firstQPStep','minH','minPsi', ...
    'minQPResidual','maxDecisionInput','minTension','maxLoadAccel', ...
    'minCheckedGeometry','dt'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'audit_entry_rollouts.csv'));
fprintf('[短時間閉ループ] 完走=%d/%d。完走はHLC回避成功や前方不変性保証ではありません。\n', ...
    sum(strcmp(report.status,'completed')),size(report,1));
end
function [x,obstacles,onlyGround]=scenarioState(mode,name,s,kind)
onlyGround=endsWith(name,'only');
switch name
    case 'hover_only', h=1.5;vz=0;pitch=0;tilt=0;vx=0;ws=0;dist=20;dir='front';
    case 'descent_only',h=1.1;vz=-.65;pitch=0;tilt=0;vx=0;ws=0;dist=20;dir='front';
    case 'front_slow', h=1.5;vz=0;pitch=0;tilt=0;vx=.4;ws=0;dist=.75;dir='front';
    case 'front_fast', h=1.5;vz=0;pitch=0;tilt=0;vx=1.5;ws=0;dist=.75;dir='front';
    case 'side_entry',h=1.5;vz=0;pitch=.1;tilt=.1;vx=.5;ws=.1;dist=.55;dir='side';
    case 'tilt_entry',h=1.4;vz=-.2;pitch=.22;tilt=.14;vx=.6;ws=.1;dist=.65;dir='front';
    otherwise,error('CBF:AuditScenario','ケースが不明');
end
xr=CBF_Audit_State('real',s,h,vz,0,pitch,tilt,vx,ws);
if strcmp(mode,'real'),x=xr;else,x=CBF_Audit_State('virtualfromreal',xr,s);end
obstacles={CBF_Audit_State('obstacle',kind,xr(8:10),dist,dir,.5)};
end
function qp=selectRows(qp,keep)
qp.A=qp.A(keep,:);qp.b=qp.b(keep);
qp.labels=qp.labels(keep);qp.psi=qp.psi(keep);
qp.psiInitiallyNonnegative=qp.psiInitiallyNonnegative(keep);
end
function v=getopt(o,k,d)
if isfield(o,k)&&~isempty(o.(k)),v=o.(k);else,v=d;end
end
