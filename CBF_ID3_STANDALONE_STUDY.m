function result = CBF_ID3_STANDALONE_STUDY(varargin)
% CBF_ID3_STANDALONE_STUDY  ID3を変更せずに実行するオフライン独立診断。
%
%   R = CBF_ID3_STANDALONE_STUDY;
%       現行Simの障害物シーンから実障害物楕円体を取得し、
%       26.2/26.4/26.425/26.450s近傍の「近似再構成状態」で比較する。
%       ログには完全な19状態がないため、26.450sを厳密再生した結果ではない。
%
%   R = CBF_ID3_STANDALONE_STUDY('snapshot', '.../snapshot.mat');
%       既存の診断プログラム等で別途記録済みの完全な状態があれば比較する。
%       ファイル内 data.firstFailure または snapshot / s を認識する。
%
%   R = CBF_ID3_STANDALONE_STUDY('snapshot', snapshotStruct);
%       snapshotStructは x(19), P/Praw, parts, dt, old(Tkm1/Tkm2),
%       previousStages, nominal, lambda を含む構造体。
%
% 本体ファイル・生成ファイル・実飛行コードの書換えなし。
% Current original ID3: raw HLC -> internally clipped reference -> QP -> final bounds.
% This standalone weight comparison instead minimizes distance to RAW HLC input
% subject to physical input bounds; it DOES NOT change the original ID3.
% Negative raw HLC thrust is valid; it is not classified as a controller fault.
% 出力: 本ファイルと同じフォルダに CBF_ID3_STANDALONE_RESULTS.zip のみ。
% CSV/MAT/TXTは一時フォルダに作りZIPに含め、実行後に一時フォルダを削除。
% 3変数の箱制約＋全線形化制約をKKT列挙で解く（Toolbox不要）。
% 追加診断: 0) 正推力での多点線形化 + 非線形同時最適化の比較
%           幾何学（回転楕円体主軸/サイズ）と運動状態から左右の探索種を生成
%           非線形CBF条件を満たさないQP解は不採用。グローバル最適保証なし。
%           1) QP線形予測と非線形の差（行ごと）
%           2) 有限差分刻みの感度・安定性 3) 入力経路に沿う曲率
%           4) HLC生値/クリップ後の重み比較
%           5) 連続拡張と離散差分の量的対応 6) 空間方向別の幾何クリアランス
% 注: 既存の本体/生成器のファイルは読み取りだけ。結果はZIP一つに集約。
%
% 重要: ここでの「連続」はddot(T)一定保持による21状態の比較モデル。
% 物理プラントの現在のZOH推力入力とは同じ実現系でない。
% 結果は数値証明・実時間保証・閉ループ安全保証ではない。

originalPath = path;
pathGuard = onCleanup(@() path(originalPath)); %#ok<NASGU>
here = fileparts(mfilename('fullpath'));
if exist('CBF_REAL_D2T_SAMPLED_EVAL','file')~=2 || ...
        exist('DRONE_OBSTACLES','file')~=2
    addpath(genpath(here),'-begin');
end
required = {'CBF_REAL_D2T_SAMPLED_EVAL','CBF_REAL_D2T_ZOH_EVAL', ...
    'CBF_COMMON_MODEL','CBF_REAL_D2T_OBSTACLE_EVAL','DRONE_OBSTACLES', ...
    'DRONE_PARAM_SUSPENDED_LOAD'};
for i=1:numel(required)
    if exist(required{i},'file')~=2
        error('ID3Study:MissingDependency', ...
            '%s が見つかりません。ファイルは元のproject直下へ置いてください。',required{i});
    end
end
mode='reconstructed';
provided=[];
if ~isempty(varargin)
    if numel(varargin)<2 || ~strcmpi(char(varargin{1}),'snapshot')
        error('ID3Study:Usage','Use () or (''snapshot'',filenameOrStruct).');
    end
    provided=read_snapshot(varargin{2});
    mode='full_snapshot';
end
% One ZIP next to THIS .m file; no result directory left behind here.
% Put staging in system temporary storage and always clean it on exit/error.
runDir=tempname;
[mkOk,mkMsg]=mkdir(runDir);
if ~mkOk,error('ID3Study:TempDir','Cannot create temporary directory: %s',mkMsg);end
cleanupRun=onCleanup(@() cleanup_temporary_directory(runDir)); %#ok<NASGU>
zipPath=fullfile(here,'CBF_ID3_STANDALONE_RESULTS.zip');

if strcmp(mode,'reconstructed')
    cases=reconstructed_cases();
else
    cases=provided;
end
nCases=numel(cases);
weightSpecs = { ...
    'physical_identity',  [1,1,1]; ...
    'physical_box',       [10,1,1]; ...
    'thrust_expensive',   [5,1,1]; ...
    'thrust_cheaper',     [20,1,1]; ...
    'torque_expensive_x16',[10,0.25,0.25]; ...
    'torque_cheaper_x4', [10,2,2]; ...
    'torque_expensive_x100',[10,0.10,0.10]; ...
    'paper_scale_as_if_delta2',[0.0625,2.5,2.5]};
% NOTE: paper_scale is (100 N/s^2)*(0.025 s)^2, not paper aircraft's thrust bound.
% For an arbitrary dt this entry is updated below.

records=cell(0,19);
weightsRecords=cell(0,24);
contRecords=cell(0,14);
linAudit=cell(0,17);
stepAudit=cell(0,12);
pathAudit=cell(0,13);
centerAudit=cell(0,13);
nuAudit=cell(0,11);
geomAudit=cell(0,8);
positiveAnchorAudit=cell(0,15);
geomActuationAudit=cell(0,17);
nonlinearSelectionAudit=cell(0,24);
nonlinearCandidatesAudit=cell(0,15);
snapDetails=cell(nCases,1);
for ci=1:nCases
    fprintf('[ID3 STUDY] %d/%d: multistart joint safety study\n',ci,nCases);
    s=cases(ci);
    x=double(s.x(:));P=s.P;dt=s.dt;
    geom=s.geom;parts=s.parts;lambda=s.lambda(:);
    nom=double(s.nominal(:));old=double(s.old(:));
    base=2*old(1)-old(2);
    % IMPORTANT: keep RAW pre-clipping HLC input as the QP objective target.
    % A valid point inside the input box is used ONLY for evaluating and
    % linearizing the nonlinear dynamics, never as an objective reference.
    rawRef=[nom(1)-base;nom(2:3)];
    Tnom=min(20,max(0,nom(1)));
    nomPhys=[Tnom;min(1,max(-1,nom(2:4)))];
    ynom=[Tnom-base;nomPhys(2:3)];
    lb=[-base;-1;-1];ub=[20-base;1;1];
    evalF=@(v) CBF_REAL_D2T_SAMPLED_EVAL(x,old(1),old(2),v, ...
       nomPhys(4),P,parts,geom,dt,3,lambda,0.005,s.previousStages);
    enom=evalF(ynom);
    [J,Jhalf,step]=jacobian_box(evalF,ynom,lb,ub,[0.5;0.05;0.05]);
    r=enom.residual(:);
    cur=enom.currentStages;
    h=enom.h0;
    if isempty(cur),minCur=NaN;else,minCur=min(cur(:));end
    if isempty(h),minH=NaN;else,minH=min(h(:));end
    if isempty(r),minR=NaN;else,minR=min(r);end
    S=[10;1;1];
    scaled=bsxfun(@times,J,S.');
    nrm=sqrt(sum(scaled.^2,2));
    firstJ=J(1,:);
    % Finite-difference consistency is a warning, not a theorem.
    jacError=norm(J-Jhalf,'fro')/(1+norm(J,'fro'));
    [maxLin,corner]=max_linearized_margin(J,r,ynom,lb,ub);
    [linJointFeasible,linWitness,linJointMargin]= ...
        linear_joint_box_feasibility(J,r,ynom,lb,ub);
    [nonWitness,witness,witnessResidual,bestMargin,count]= ...
        nonlinear_witness(evalF,ynom,base,dt,nomPhys,lb,ub);
    % Numeric maxLinear and witness are different logical statements.
    % maxLin<0 proves local *linearized box* infeasibility only.
    if ~linJointFeasible
        classification='LINEARIZED_JOINT_BOX_INFEASIBLE';
    elseif nonWitness
        classification='NONLINEAR_WITNESS_FOUND';
    else
        classification='LINEARIZED_POSSIBLE_NONLINEAR_UNKNOWN';
    end
    if minCur<0
        classification=[classification '_NESTED_STAGE_OUTSIDE'];
    end
    if minH<0
        % When interval lower condition includes t=0, outside cannot pass.
        classification=[classification '_PHYSICAL_BARRIER_OUTSIDE'];
    end
    [geometry,xyzDh]=geometry_gradient(x,P,geom,parts);
    records(end+1,:)={s.label,s.t,mode,minH,minCur,minR, ...
        maxLin,nonWitness,bestMargin,count,classification, ...
        firstJ(1),firstJ(2),firstJ(3),jacError, ...
        norm(scaled(1,:)),geometry.aspectRatio, ...
       s.loggedH, minH-s.loggedH}; %#ok<AGROW>
    % Check whether a reported zero torque gradient persists across FD scales.
    % Every step is a PHYSICAL delta2T [N] or torque [Nm] increment.
    fdScales=[0.125,0.25,0.5,1,2,4,8];
    for fi=1:numel(fdScales)
        fdH=[0.5;0.05;0.05]*fdScales(fi);
        [js,~,~]=jacobian_box(evalF,ynom,lb,ub,fdH);
        stepAudit(end+1,:)={s.label,s.t,fdScales(fi),fdH(1), ...
            fdH(2),js(1,1),js(1,2),js(1,3), ...
            js(2,1),js(2,2),js(2,3), ...
            norm(js-J,'fro')/max(norm(J,'fro'),1e-12)}; %#ok<AGROW>
    end
    % For the second-difference digital acceleration and a continuous
    % hold of nuT, the SAME ENDPOINT thrust generally requires 2*nuDigital
    % when q0=(Tkm1-Tkm2)/dt. The within-sample thrust is still different.
    qPrev=(old(1)-old(2))/dt;
    nuDigital=(nomPhys(1)-base)/dt^2;
    nuEndMatch=2*(nomPhys(1)-old(1)-qPrev*dt)/dt^2;
    nuAudit(end+1,:)={s.label,s.t,old(1),old(2), ...
       nomPhys(1),qPrev,nuDigital,nuEndMatch, ...
       nuEndMatch-2*nuDigital,dt, ...
       'The two actuator models have different intra-sample thrust.'}; %#ok<AGROW>
    % Spatial movement of the load (not an actuator input) isolates the
    % geometry from the 25-ms dynamics and normal-vector support model.
    dhSpatial=0.1;
    hBase=geometry_clearance(x,P,geom,parts);
    for axisId=1:3
        for sgn=[-1,1]
            xProbe=x;xProbe(7+axisId)=xProbe(7+axisId)+sgn*dhSpatial;
            hp=geometry_clearance(xProbe,P,geom,parts);
            geomAudit(end+1,:)={s.label,s.t,axisId,sgn,dhSpatial, ...
                hBase(1),hp(1),hp(1)-hBase(1)}; %#ok<AGROW>
        end
    end
    % Optimize the same linearized condition for many QP weights.
    % Both constraints enter ONE joint 3-variable QP using KKT enumeration.
    for wi=1:size(weightSpecs,1)
        wName=weightSpecs{wi,1};sig=weightSpecs{wi,2}(:);
        if strcmp(wName,'paper_scale_as_if_delta2')
            sig=[100*dt*dt;2.5;2.5];
        end
        [uLin,st,violLin]=solve_box_qp(J,r,ynom,lb,ub,sig,rawRef);
        eReal=evalF(uLin);
        linearPred=r+J*(uLin-ynom);
        for rowId=1:numel(linearPred)
            % Detect rows that pass the local QP but fail after re-evaluation.
            linAudit(end+1,:)={s.label,s.t,wName,rowId, ...
                uLin(1)+base,uLin(2),uLin(3), ...
                linearPred(rowId),eReal.residual(rowId), ...
                eReal.residual(rowId)-linearPred(rowId), ...
                linearPred(rowId)>=0,eReal.residual(rowId)>=0, ...
                norm(uLin-ynom), ...
                abs(uLin(1)-ynom(1)), ...
                norm(uLin(2:3)-ynom(2:3)), ...
                enom.residual(rowId), ...
                sum(J(rowId,:).*(uLin-ynom).')}; %#ok<AGROW>
        end
        % Test the objective-centering change WITHOUT changing the model.
        % Clipped and raw references share exactly the same box constraints.
        [uClip,stClip]=solve_box_qp(J,r,ynom,lb,ub,sig,ynom);
        eClip=evalF(uClip);
        centerAudit(end+1,:)={s.label,s.t,wName,nom(1),Tnom, ...
            uLin(1)+base,uClip(1)+base, ...
            norm(uLin-uClip), ...
            min(eReal.residual(:)),min(eClip.residual(:)), ...
            st,stClip, ...
            sum(((uLin-rawRef)./sig).^2)-sum(((uClip-ynom)./sig).^2)}; %#ok<AGROW>
        % Path from the physically bounded nominal anchor to QP choice;
        % the nonlinear witness path is appended outside the weight loop.
        if strcmp(wName,'physical_box')
            pathAudit=append_input_path(pathAudit,evalF,s, ...
                'to_physical_box_QP',ynom,uLin,r,J,base);
        end
        gS=(J(1,:).'.*sig).^2;
        if sum(gS)>0,effectFraction=gS/sum(gS);
        else,effectFraction=[NaN;NaN;NaN];end
        scoreReal=min(eReal.residual(:));
        weightsRecords(end+1,:)={s.label,s.t,wName, ...
            sig(1),sig(2),sig(3),uLin(1)+base,uLin(2),uLin(3), ...
            (uLin(1)-rawRef(1)),uLin(2)-rawRef(2),uLin(3)-rawRef(3), ...
            st,violLin,scoreReal, ...
            norm(bsxfun(@times,J,sig.'),'fro'), ...
            dot((uLin-rawRef)./sig,(uLin-rawRef)./sig), ...
            effectFraction(1),effectFraction(2),effectFraction(3), ...
            nom(1),nomPhys(1),uLin(1)+base-nom(1), ...
            uLin(1)+base-nomPhys(1)}; %#ok<AGROW>
    end
    if nonWitness
        witnessV=[witness(1)-base;witness(2:3)];
        pathAudit=append_input_path(pathAudit,evalF,s, ...
            'to_nonlinear_witness',ynom,witnessV,r,J,base);
    end
    % NEW: all controller actions below are OFFLINE experiments only.
    % Re-evaluate every candidate against the ORIGINAL full nonlinear ZOH oracle.
    % The physical thrust is NEVER replaced by an actual continuous ddot(T).
    [paRows,geomRows,selRows,seedRows,nonlinearDetail]= ...
        nonlinear_multistart_comparison(s,evalF,base,ynom,lb,ub, ...
            rawRef,witness,nonWitness,geometry);
    positiveAnchorAudit=[positiveAnchorAudit;paRows]; %#ok<AGROW>
    geomActuationAudit=[geomActuationAudit;geomRows]; %#ok<AGROW>
    nonlinearSelectionAudit=[nonlinearSelectionAudit;selRows]; %#ok<AGROW>
    nonlinearCandidatesAudit=[nonlinearCandidatesAudit;seedRows]; %#ok<AGROW>
    [contRow,continuousInfo]=continuous_compare(s,enom,ynom);
    contRecords(end+1,:)=contRow; %#ok<AGROW>
    snapDetails{ci}=struct('caseData',s,'nominalEval',enom, ...
       'jointLinearizedFeasible',linJointFeasible, ...
       'jointLinearizedWitness',linWitness, ...
       'jointLinearizedMargin',linJointMargin, ...
       'jacobian',J,'jacobianHalfStep',Jhalf,'jacobianDelta',step, ...
       'jacobianConsistency',jacError,'normalizedJacobian',scaled, ...
       'normalizedRows',nrm,'maxLinearizedMargin',maxLin, ...
       'bestLinearizedCorner',corner,'rawObjectiveReference',rawRef, ...
       'linearizationAnchor',ynom,'witnessFound',nonWitness, ...
       'bestNonlinearPhysicalInput',witness, ...
       'bestNonlinearResidual',witnessResidual, ...
       'bestNonlinearMargin',bestMargin,'geometry',geometry, ...
       'spatialClearanceGradXYZ',xyzDh, ...
       'continuousComparison',continuousInfo, ...
       'nonlinearJointComparison',nonlinearDetail);
end
csv_write(fullfile(runDir,'linearization_audit.csv'), ...
    {'case','time_s','weight_method','constraint_row','T_candidate_N', ...
    'tauX_candidate_Nm','tauY_candidate_Nm', ...
    'linearized_residual','nonlinear_residual','nonlinear_minus_linear', ...
    'linear_predict_pass','nonlinear_actual_pass','distance_from_anchor', ...
    'abs_deltaT_from_anchor_N','torque_distance_from_anchor_Nm', ...
    'nominal_residual','linearized_change'},linAudit);
csv_write(fullfile(runDir,'fd_sensitivity_sweep.csv'), ...
    {'case','time_s','step_factor','delta2T_step_N','torque_step_Nm', ...
    'dPhi4_dDelta2T','dPhi4_dTauX','dPhi4_dTauY', ...
    'dZOH_dDelta2T','dZOH_dTauX','dZOH_dTauY', ...
    'jacobian_relative_change_from_default'},stepAudit);
csv_write(fullfile(runDir,'input_path_nonlinearity.csv'), ...
    {'case','time_s','path_name','path_fraction', ...
    'T_N','tauX_Nm','tauY_Nm', ...
    'linear_phi4','real_phi4','linear_ZOH','real_ZOH', ...
    'min_linear_residual','min_nonlinear_residual'},pathAudit);
csv_write(fullfile(runDir,'raw_vs_clipped_objective.csv'), ...
    {'case','time_s','weight_method','raw_HLC_T_N','clipped_HLC_T_N', ...
    'raw_centered_QP_T_N','clipped_centered_QP_T_N','distance_between_QP_solutions', ...
    'raw_centered_nonlin_min','clipped_centered_nonlin_min', ...
    'raw_centered_QP_status','clipped_centered_QP_status', ...
    'difference_in_objective_values_NOT_comparable'},centerAudit);
csv_write(fullfile(runDir,'digital_vs_continuous_accel.csv'), ...
    {'case','time_s','Tkm1_N','Tkm2_N', ...
    'current_bounded_T_N','qTprev_Nps','digital_delta2T_over_dt2_Nps2', ...
    'continuous_endpoint_matched_nuT_Nps2', ...
    'endpoint_minus_2xdigital_Nps2','dt_s','comment'},nuAudit);
csv_write(fullfile(runDir,'geometry_direction.csv'), ...
    {'case','time_s','axis_1X_2Y_3Z','direction_sign','shift_m', ...
    'h_before_m','h_after_m','delta_h_m'},geomAudit);
csv_write(fullfile(runDir,'cases.csv'), ...
    {'case','time_s','mode','h_current_m','min_current_phi_m', ...
    'min_nominal_residual','max_linearized_margin', ...
    'nonlinear_witness_found','best_nonlinear_margin', ...
    'nonlinear_evaluations','classification', ...
    'dPhi4_dDelta2T','dPhi4_dTauX','dPhi4_dTauY', ...
    'fd_relative_discrepancy','scaled_jac_row_norm','obs_max_to_min_axis_ratio', ...
    'logged_h0_if_available','reconstruction_h_minus_log_h'},records);
csv_write(fullfile(runDir,'weight_sweep.csv'), ...
    {'case','time_s','weight_method','scale_Delta2T_N', ...
    'scale_tauX_Nm','scale_tauY_Nm','T_candidate_N', ...
    'tauX_candidate_Nm','tauY_candidate_Nm','deltaT_N_from_raw_HLC', ...
    'deltaTauX_Nm_from_raw_HLC','deltaTauY_Nm_from_raw_HLC','linear_qp_status', ...
    'linear_qp_margin_min','nonlinear_candidate_margin_min', ...
    'scaled_control_jac_fro','objective', ...
    'predicted_linear_share_T','predicted_linear_share_tauX', ...
    'predicted_linear_share_tauY', ...
    'HLC_raw_thrust_N','ID3_clipped_nominal_thrust_N', ...
    'CBF_candidate_minus_HLC_raw_thrust_N', ...
    'CBF_candidate_minus_ID3_clipped_nominal_thrust_N'},weightsRecords);
csv_write(fullfile(runDir,'continuous_vs_ZOH.csv'), ...
    {'case','time_s','Tprev_N','qTprev_Nps', ...
    'nu_to_match_endpoint_Nps2','Tend_cont_N', ...
    'hEnd_ZOH_m','hEnd_cont_m','difference_h_m', ...
    'dPsi4_dNuT','dPsi4_dTauX','dPsi4_dTauY', ...
    'continuous_valid','comment'},contRecords);
csv_write(fullfile(runDir,'positive_thrust_anchor_QP.csv'), ...
    {'case','time_s','anchor_T_N','anchor_tauX','anchor_tauY', ...
    'anchor_phi4','anchor_ZOH','dPhi4_dT','dPhi4_dTauX','dPhi4_dTauY', ...
    'QP_T_N','QP_tauX','QP_tauY','QP_nonlin_phi4','QP_nonlin_min'}, ...
    positiveAnchorAudit);
csv_write(fullfile(runDir,'geometry_actuation_map.csv'), ...
    {'case','time_s','obstacle','obs_axis_ratio','major_axis_xyz', ...
    'lateral_unit_xyz','approach_velocity_xyz','anchor_T_N', ...
    'dCenter_dTauX_xyz','dCenter_dTauY_xyz', ...
    'lateral_dTauX','lateral_dTauY','dPhi4_dTauX','dPhi4_dTauY', ...
    'h_current','obs_support_along_normal_m','lateral_observability_m_per_Nm'}, ...
    geomActuationAudit);
csv_write(fullfile(runDir,'nonlinear_joint_selection.csv'), ...
    {'case','time_s','weight_name','scale_T_N','scale_tauX_Nm','scale_tauY_Nm', ...
    'method','solver_available','feasible','T_N','tauX_Nm','tauY_Nm', ...
    'objective_cost','phi4_next','ZOH_guard','min_nonlinear_residual', ...
    'linearized_baseline_T_N','linearized_baseline_min_nonlinear', ...
    'best_seed_cost','eval_count','CBF_induced_lateral_delta_m', ...
    'geometry_side','current_nested_stages_nonnegative', ...
    'model_sampled_conditions_only'}, ...
    nonlinearSelectionAudit);
csv_write(fullfile(runDir,'nonlinear_seed_candidates.csv'), ...
    {'case','time_s','source','T_N','tauX_Nm','tauY_Nm', ...
    'phi4_next','ZOH_guard','min_nonlinear_residual', ...
    'input_cost_box_scale','CBF_induced_lateral_delta_m', ...
    'next_conditions_met', ...
    'hEnd','max_abs_torque','reconstruction_mode'},nonlinearCandidatesAudit);
result=struct('outputZip',zipPath,'mode',mode,'cases',{cases}, ...
    'caseRows',{records},'weightSweep',{weightsRecords}, ...
    'continuousRows',{contRecords},'details',{snapDetails}, ...
    'linearizationAudit',{linAudit},'sensitivityStepAudit',{stepAudit}, ...
    'positiveAnchorAudit',{positiveAnchorAudit}, ...
    'geometryActuationAudit',{geomActuationAudit}, ...
    'nonlinearJointSelectionAudit',{nonlinearSelectionAudit}, ...
    'nonlinearSeedCandidatesAudit',{nonlinearCandidatesAudit}, ...
    'inputPathAudit',{pathAudit},'rawVsClippedAudit',{centerAudit}, ...
    'digitalVsContinuousAudit',{nuAudit},'geometryDirectionAudit',{geomAudit}, ...
    'source','Original project functions, no source modifications', ...
    'inputPolicy',['Standalone QP objective is RAW preclip HLC input; ', ...
       'physical box bounds remain hard constraints. Nonlinear evaluation ', ...
       'and finite differences use a valid bounded anchor. ', ...
       'The original ID3 is untouched and still clips its internal reference.'], ...
    'certification',false);
save(fullfile(runDir,'study.mat'),'result','-v7');
write_summary(fullfile(runDir,'summary.txt'),result);
try
    % Build a complete ZIP in the temp directory, then publish it next to
    % the test .m. Do not leave partial ZIPs at the destination on errors.
    stagingZip=fullfile(runDir,'_complete_results.zip');
    zip(stagingZip, ...
        {'cases.csv','weight_sweep.csv','continuous_vs_ZOH.csv', ...
         'linearization_audit.csv','fd_sensitivity_sweep.csv', ...
         'input_path_nonlinearity.csv','raw_vs_clipped_objective.csv', ...
         'digital_vs_continuous_accel.csv','geometry_direction.csv', ...
         'positive_thrust_anchor_QP.csv','geometry_actuation_map.csv', ...
         'nonlinear_joint_selection.csv','nonlinear_seed_candidates.csv', ...
         'summary.txt','study.mat'},runDir);
    [copyOK,copyMsg]=copyfile(stagingZip,zipPath,'f');
    if ~copyOK,error('ID3Study:ZipPublish','%s',copyMsg);end
catch ME
    error('ID3Study:ZipOutput','Could not write result ZIP: %s',ME.message);
end
fprintf('\n[ID3 STANDALONE] mode=%s, cases=%d\n',mode,nCases);
fprintf('[ID3 STANDALONE] Result ZIP: %s\n',zipPath);
if strcmp(mode,'reconstructed')
    fprintf(['[ID3 STANDALONE] CAUTION: logged positions do not contain ', ...
       'exact attitude/cable directions or barrier memory. ', ...
       'This is NOT an exact replay of 26.450 s.\n']);
end
end

function cases=reconstructed_cases()
% Anchors from 2026-11-24 diagnostic text; unspecified states are assumptions.
% These values are for an isolated hypothesis test, NOT recorded 19-state data.
par=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
% Reconstructed-case assumptions from the supplied flight log only.
% DO NOT call par.set('loadmass', ...): DRONE_PARAM_SUSPENDED_LOAD.set
% expects a scalar string/string array of property NAMES, not a char vector.
% Build a LOCAL parameter struct; do not change any plant/controller object.
P=struct('m',double(par.mass),'mL',0.14, ...
 'J',double([par.jx;par.jy;par.jz]),'gravity',double(par.gravity), ...
 'cableL',2.0,'dstx',0,'dsty',0);
world=DRONE_OBSTACLES('scene_cylinders_high_side','single_ellipsoid','none');
idx=find(strcmp({world.parts.obstacleName},'a'));
if isempty(idx)
    error('ID3Study:ObstacleA','Existing scene obstacle a not found');
end
parts=world.parts(idx);
dt=0.025;lambda=[2;4;8;16];
geom=struct('a',P.cableL/2+0.02,'b',0.20,'centerFromLoad',0.5);
% [t, droneY, droneZ, uNomT, uNomTauX, uNomTauY, Tkm1, Tkm2]
logData=[26.200,8.137,3.001,8.45,-0.00013,0.0000342,9.102,9.014; ...
         26.400,8.237,3.019,2.28,-0.00203,0.000534,10.999,10.364; ...
         26.425,8.250,3.026,-0.375,0.0348,-0.00933,12.278,10.999; ...
         26.450,8.262,3.036,-5.65,-0.0466,0.0122,16.074,12.278];
cases=repmat(struct('label','','t',NaN,'x',[], 'P',P, ...
    'parts',parts,'geom',geom,'dt',dt,'lambda',lambda, ...
    'nominal',[],'old',[],'previousStages',[],'loggedH',NaN, ...
    'assumptions',''),4,1);
for i=1:size(logData,1)
    d=logData(i,:);t=d(1);
    x=assumed_x(d(2),d(3),P.cableL);
    cases(i).label=sprintf('RECON_t%07.3f',t);
    cases(i).t=t;
    cases(i).x=x;
    cases(i).nominal=[d(4:6).';0];
    cases(i).old=d(7:8).';
    cases(i).previousStages=assumed_barrier_memory(d(2),d(3),P,parts,geom,dt,lambda);
    loggedH=[0.388,0.298,0.287,0.277];
    cases(i).loggedH=loggedH(i);
    cases(i).assumptions=['Physical P drone position from log; ', ...
      'q=[1;0;0;0], cable vertical, vL=[0;0.5;0], angular rates zero. ', ...
      'Synthetic measured history from constant-velocity approach.'];
end
end

function x=assumed_x(droneY,droneZ,L)
pT=[0;0;-1];pDrone=[0;droneY;droneZ];
pLoad=pDrone+L*pT;
x=[1;0;0;0; 0;0;0; pLoad; 0;0.5;0; pT; 0;0;0];
end

function prev=assumed_barrier_memory(droneY,droneZ,P,parts,geom,dt,lambda)
% Generate synthetic causal stages using only PAST positions.
% Previous stages at (k-1), no future input presumed.
N=36;alpha=lambda(:)*dt;
hist=zeros(4,N);
for k=1:N
    yy=droneY-(N-k+1)*0.5*dt;
    xx=assumed_x(yy,droneZ,P.cableL);
    hist(1,k)=geometry_clearance(xx,P,geom,parts);
    if k>=2
        for level=2:4
            hist(level,k)=(1+alpha(level-1))*hist(level-1,k)-hist(level-1,k-1);
        end
    end
end
prev=hist(:,end);
end

function s=read_snapshot(source)
if ischar(source) || isstring(source)
    payload=load(char(source));
    if isfield(payload,'data') && isfield(payload.data,'firstFailure')
        source=payload.data.firstFailure;
    elseif isfield(payload,'snapshot')
        source=payload.snapshot;
    elseif isfield(payload,'s')
        source=payload.s;
    elseif isfield(payload,'result') && isfield(payload.result,'snapshot')
        source=payload.result.snapshot;
    else
        error('ID3Study:Snapshot','Recognized snapshot variable not found.');
    end
end
if numel(source)>1, source=source(1); end
if ~isstruct(source) || ~isfield(source,'x')
    error('ID3Study:Snapshot','Snapshot needs x, P/Praw, parts, histories.');
end
if isfield(source,'P'),P=source.P;
elseif isfield(source,'Praw')
    p=double(source.Praw(:));
    P=struct('m',p(1),'J',p(2:4),'gravity',p(5),'mL',p(6), ...
        'cableL',p(7),'dstx',p(8),'dsty',p(9));
else,error('ID3Study:Snapshot','Missing P or Praw');end
if isfield(source,'geom'),geom=source.geom;
else,geom=struct('a',P.cableL/2+0.02,'b',0.20,'centerFromLoad',0.5);end
if isfield(source,'lambda'),lambda=source.lambda;
elseif isfield(source,'cfg'),lambda=source.cfg.d2tLambda;
else,lambda=[2;4;8;16];end
if isfield(source,'old'),old=source.old;
elseif isfield(source,'cfg') && isfield(source.cfg,'d2tHistory')
    old=[source.cfg.d2tHistory.Tkm1;source.cfg.d2tHistory.Tkm2];
else,error('ID3Study:Snapshot','Missing old applied thrust history');end
if isfield(source,'previousStages'),prev=source.previousStages;
elseif isfield(source,'cfg') && isfield(source.cfg,'d2tBarrierHistory')
    prev=source.cfg.d2tBarrierHistory.previousStages;
else,error('ID3Study:Snapshot','Missing measured previousStages');end
if isfield(source,'nominal'),nom=source.nominal;
elseif isfield(source,'nom'),nom=source.nom;
else,error('ID3Study:Snapshot','Missing nominal');end
if isfield(source,'dt'),dt=source.dt;
else,error('ID3Study:Snapshot','Missing dt');end
if isfield(source,'t'),tt=source.t;else,tt=NaN;end
s=struct('label','EXACT_SNAPSHOT','t',tt,'x',source.x(:), ...
    'P',P,'parts',source.parts,'geom',geom, ...
    'dt',dt,'lambda',lambda(:),'nominal',nom(:), ...
    'old',old(:),'previousStages',prev,'loggedH',NaN, ...
    'assumptions','Original recorded state; no synthetic histories');
end

function [J,Jhalf,step]=jacobian_box(fun,y,lb,ub,steps)
ref=fun(y);m=numel(ref.residual);
J=zeros(m,3);Jhalf=zeros(m,3);step=zeros(3,1);
for j=1:3
    for r=1:2
        dh=steps(j)/(2^(r-1));
        a=y;b=y;a(j)=min(ub(j),y(j)+dh);b(j)=max(lb(j),y(j)-dh);
        if a(j)-b(j)<1e-12,error('ID3Study:GradientBox','No gradient room');end
        ea=fun(a);eb=fun(b);
        if r==1,J(:,j)=(ea.residual(:)-eb.residual(:))/(a(j)-b(j));
        else,Jhalf(:,j)=(ea.residual(:)-eb.residual(:))/(a(j)-b(j));end
    end
    step(j)=steps(j);
end
end

function [m,corner]=max_linearized_margin(J,r,ynom,lb,ub)
% If any row's MAX over the box is negative, THAT LINEARIZED ROW
% is rigorously box-infeasible (subject only to arithmetic rounding).
r0=r(:)-J*ynom;
upper=zeros(size(r));
for i=1:numel(r)
    us=lb;
    us(J(i,:).'>=0)=ub(J(i,:).'>=0);
    upper(i)=r0(i)+J(i,:)*us;
end
m=min(upper);[~,k]=min(upper);
corner=lb;corner(J(k,:).'>=0)=ub(J(k,:).'>=0);
end

function [yes,uBest,resBest,best,count]=nonlinear_witness(fun,yn,base,dt,nom,lb,ub) %#ok<INUSD>
% Nonlinear feasible WITNESS scan, not a proof of infeasibility if absent.
Tgrid=unique([0,4,8,10,12,16,20,nom(1)]);
Qgrid=[-1,-0.25,0,0.25,1];
best=-Inf;uBest=[NaN;NaN;NaN];resBest=[];yes=false;count=0;
for ti=1:numel(Tgrid)
    for a=1:numel(Qgrid)
        for b=1:numel(Qgrid)
            yy=[Tgrid(ti)-base;Qgrid(a);Qgrid(b)];
            if any(yy<lb-1e-12) || any(yy>ub+1e-12),continue;end
            try
                ee=fun(yy); margin=min(ee.residual(:));count=count+1;
            catch
                continue
            end
            if margin>best
                best=margin;uBest=[Tgrid(ti);Qgrid(a);Qgrid(b)];
                resBest=ee.residual(:);
            end
            if margin>=0,yes=true;end
        end
    end
end
end

function rows=append_input_path(rows,fun,s,name,anchor,target,r,J,base)
% Scan both safety constraints along a physically valid input path.
% A Taylor remainder is exposed directly, without claiming convexity.
for frac=linspace(0,1,17)
    y=anchor+(target-anchor)*frac;
    eo=fun(y);
    pr=r+J*(y-anchor);
    rows(end+1,:)={s.label,s.t,name,frac, ...
        y(1)+base,y(2),y(3),pr(1),eo.residual(1), ...
        pr(2),eo.residual(2),min(pr),min(eo.residual(:))}; %#ok<AGROW>
end
end

function [feasible,witness,margin]=linear_joint_box_feasibility(J,r,yn,lb,ub)
% Exact (up to numeric tolerance) 3D feasibility test by vertex enumeration.
% Any nonempty bounded 3D polytope has a vertex; no optimization toolbox.
[JN,rN]=scaled_rows(J,r,lb,ub);
A=[JN;eye(3);-eye(3)];b=[JN*yn-rN;lb;-ub];
feasible=false;witness=[NaN;NaN;NaN];margin=-Inf;
if all(A*yn>=b-1e-10)
    feasible=true;witness=yn;margin=min(r);return
end
triples=nchoosek(1:size(A,1),3);
for i=1:size(triples,1)
    ids=triples(i,:);M=A(ids,:);
    if rcond(M)<1e-11,continue;end
    v=M\b(ids);
    err=min(A*v-b);
    if err>margin,margin=err;witness=v;end
    if err>=-1e-9
        feasible=true;margin=min(r+J*(v-yn));return
    end
end
end

function [v,status,margin]=solve_box_qp(J,r,yn,lb,ub,sig,rawRef)
% Exact convex 3-variable linearized QP by enumerating KKT active sets.
% No quadprog needed. Every row and all physical box bounds included.
% Minimize 0.5*(v-rawRef)'*W*(v-rawRef) with J*(v-yn)+r>=0.
% Here yn is an IN-BOX linearization anchor, rawRef may be OUTSIDE the box.
[JN,rN]=scaled_rows(J,r,lb,ub);
A=[JN;eye(3);-eye(3)];b=[JN*yn-rN;lb;-ub];
WInv=diag(sig(:).^2);best=Inf;v=[];status='KKT_ENUMERATION';
if all(A*rawRef>=b-1e-9)
    v=rawRef;margin=min(r+J*(v-yn));status='KKT_RAW_REFERENCE_FEASIBLE';return
end
for k=1:3
    act=nchoosek(1:size(A,1),k);
    for i=1:size(act,1)
        ids=act(i,:);M=A(ids,:);
        Q=M*WInv*M.';
        if rcond(Q)<1e-13,continue;end
        mult=Q\(b(ids)-M*rawRef);
        if any(mult<-1e-8),continue;end
        z=rawRef+WInv*M.'*mult;
        if any(A*z < b-1e-9),continue;end
        score=0.5*sum(((z-rawRef)./sig(:)).^2);
        if score<best
            best=score;v=z;
        end
    end
end
if isempty(v)
    [feas,witness]=linear_joint_box_feasibility(J,r,yn,lb,ub);
    if feas
        v=witness;status='LINEAR_FEASIBLE_KKT_NUMERIC_ISSUE';
    else
        v=max(lb,min(ub,rawRef));status='LINEAR_JOINT_INFEASIBLE';
    end
end
margin=min(r+J*(v-yn));
end

function [JN,rN]=scaled_rows(J,r,lb,ub)
% Protect tiny 4th-stage residuals from an absolute 1e-9 QP tolerance.
% Divide each safety row by its own reachable magnitude, never drop rows.
span=(ub(:)-lb(:));
rs=max([abs(r(:)),sum(abs(bsxfun(@times,J,span.')),2), ...
       1e-14*ones(size(r(:)))],[],2);
JN=bsxfun(@rdivide,J,rs);rN=r(:)./rs;
end

function [g,dh]=geometry_gradient(x,P,geom,parts)
% Obstacles are their ORIGINAL single-ellipsoid approximation.
part=parts(1);Q=part.R*diag(part.params.axes(:).^2)*part.R.';
ax=sort(sqrt(eig(Q)));g=struct('axes',ax, ...
    'aspectRatio',max(ax)/min(ax),'center',part.center(:), ...
    'kind',part.kind,'name',part.obstacleName);
dh=zeros(3,1);e=1e-3;
for i=1:3
    xp=x;xm=x;xp(7+i)=xp(7+i)+e;xm(7+i)=xm(7+i)-e;
    dh(i)=(geometry_clearance(xp,P,geom,parts)- ...
         geometry_clearance(xm,P,geom,parts))/(2*e);
end
end

function h=geometry_clearance(x,P,geom,parts)
C=x(8:10)-0.5*P.cableL*x(14:16);
h=NaN(numel(parts),1);
for j=1:numel(parts)
    pt=parts(j);d=C-pt.center(:);n=d/norm(d);
    sa=sqrt(geom.b^2+(geom.a^2-geom.b^2)*(n.'*x(14:16))^2);
    if strcmp(char(pt.kind),'sphere')
        if isfield(pt,'params'),so=pt.params.radius;else,so=pt.radius;end
    else
        if isfield(pt,'params'),aa=pt.params.axes(:);else,aa=pt.axes(:);end
        if isfield(pt,'R'),R=pt.R;else,R=eye(3);end
        so=norm(diag(aa)*R.'*n);
    end
    h(j)=norm(d)-sa-so;
end
end

function [row,details]=continuous_compare(s,discreteNom,ynom)
% Compare two DIFFERENT actuator models for same endpoint T on ONE step.
% Continuous model: dT=qT, dqT=nuT held; physical model: T=constant.
try
    dt=s.dt;old=s.old(:);
    % Algebra: nu=2*(Tcandidate - Tkm1 - qTprev*dt)/dt^2.
    q0=(old(1)-old(2))/dt;
    Tnew=2*old(1)-old(2)+ynom(1);
    nu=2*(Tnew-old(1)-q0*dt)/(dt*dt);
    yy=[s.x(:);old(1);q0];vv=[nu;ynom(2:3)];
    z=yy;N=12;h=dt/N;
    for i=1:N
        k1=extended_rhs(z,vv,s.nominal(4),s.P);
        k2=extended_rhs(z+h/2*k1,vv,s.nominal(4),s.P);
        k3=extended_rhs(z+h/2*k2,vv,s.nominal(4),s.P);
        k4=extended_rhs(z+h*k3,vv,s.nominal(4),s.P);
        z=z+h/6*(k1+2*k2+2*k3+k4);
    end
    hContinuous=geometry_clearance(z(1:19),s.P,s.geom,s.parts);
    cG=struct('collisionMode','cable_ellipsoid', ...
       'cableAxisShort',s.geom.b,'cableAxisLong',s.geom.a, ...
       'cableCenterFromLoad',0.5);
    yc=[s.x(:);old(1);q0];
    c0=CBF_REAL_D2T_OBSTACLE_EVAL(yc,[0;0;0],s.nominal(4), ...
        s.P,cG,s.parts,s.lambda(:));
    ct=CBF_REAL_D2T_OBSTACLE_EVAL(yc,[100;0;0],s.nominal(4), ...
        s.P,cG,s.parts,s.lambda(:));
    cx=CBF_REAL_D2T_OBSTACLE_EVAL(yc,[0;0.1;0],s.nominal(4), ...
        s.P,cG,s.parts,s.lambda(:));
    cy=CBF_REAL_D2T_OBSTACLE_EVAL(yc,[0;0;0.1],s.nominal(4), ...
        s.P,cG,s.parts,s.lambda(:));
    jc=[(ct.stages(5,1)-c0.stages(5,1))/100, ...
       (cx.stages(5,1)-c0.stages(5,1))/.1, ...
       (cy.stages(5,1)-c0.stages(5,1))/.1];
    row={s.label,s.t,old(1),q0,nu,z(20), ...
       discreteNom.hEnd(1),hContinuous(1), ...
       hContinuous(1)-discreteNom.hEnd(1), ...
       jc(1),jc(2),jc(3),true, ...
       'Endpoint thrust matched; intra-period input differs'};
    details=struct('augmentedEnd',z,'continuousSensitivity',jc, ...
        'continuousH',hContinuous,'nuT',nu,'qTprev',q0);
catch ME
    row={s.label,s.t,NaN,NaN,NaN,NaN, ...
        discreteNom.hEnd(1),NaN,NaN,NaN,NaN,NaN,false,ME.message};
    details=struct('error',ME.message,'identifier',ME.identifier);
end
end

function f=extended_rhs(z,v,tauZ,P)
input=[z(20);v(2:3);tauZ];
f=[CBF_COMMON_MODEL(z(1:19),input,P);z(21);v(1)];
end

function [anchorRows,geomRows,selectionRows,seedRows,detail]= ...
        nonlinear_multistart_comparison(s,evalF,base,ynom,lb,ub,rawRef, ...
            gridWitness,hasWitness,geometry)
% Strictly OFFLINE study: choose [T,tauX,tauY] with ONE nonlinear joint
% constraint set, accounting for true obstacle axes+orientation and x(19).
% No forward multi-period constant-input assumption, no controller edits.
anchorRows=cell(0,15);geomRows=cell(0,17);
selectionRows=cell(0,24);seedRows=cell(0,15);
lo=[0;-1;-1];hi=[20;1;1];
raw=[rawRef(1)+base;rawRef(2:3)];
clipped=min(hi,max(lo,raw));
physicalEval=@(z) evalF([z(1)-base;z(2:3)]);
refEval=physicalEval(clipped);
referenceCenter=cable_center_from_x(refEval.xEnd,s.P.cableL);
% Sample and evaluate the SAME actual ZOH map at SEVERAL positive thrusts.
% At T=0 the torque sensitivity may vanish; this is a linearization-point
% phenomenon, not proof that torques are ineffective at a positive T.
probeT=[0,2,5,8,10,12,16,20];
allAnchorQP=cell(0,1);
for i=1:numel(probeT)
    z=[probeT(i);clipped(2:3)];
    try
        ee=physicalEval(z);
        [J,~,~]=jacobian_box(evalF,[z(1)-base;z(2:3)],lb,ub, ...
            [0.25;0.075;0.075]);
        [candidate,~,~]=solve_box_qp(J,ee.residual(:), ...
            [z(1)-base;z(2:3)],lb,ub,[10;1;1],rawRef);
        zQ=[candidate(1)+base;candidate(2:3)];
        ec=physicalEval(zQ);
        allAnchorQP{end+1}=zQ; %#ok<AGROW>
        anchorRows(end+1,:)={s.label,s.t,z(1),z(2),z(3), ...
            min(ee.residual(1:numel(s.parts))), ...
            min(ee.residual(numel(s.parts)+1:end)),J(1,1),J(1,2),J(1,3), ...
            zQ(1),zQ(2),zQ(3), ...
            min(ec.residual(1:numel(s.parts))),min(ec.residual(:))}; %#ok<AGROW>
    catch
        anchorRows(end+1,:)={s.label,s.t,z(1),z(2),z(3), ...
            NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN}; %#ok<AGROW>
    end
end
% The geometry is *not* a synthetic sphere. Obstacles are the actual
% rotated single-ellipsoid parts, and the cable geometry is unchanged.
ctx=obstacle_geometry_context(s,geometry);
% Map the actual small-time actuator effectiveness to LATERAL motion.
% This is used for symmetric +/- steering SEEDS, not for imposing a
% made-up route choice or sacrificing hard safety constraints.
geomSigns=zeros(2,3);
for i=1:3
    T=[5,10,15];Tval=T(i);
    z=[Tval;0;0];
    eh=physicalEval(z);
    Jc=zeros(3,2);Jphi=zeros(1,2);
    for j=1:2
        zp=z;zm=z;zp(j+1)=0.4;zm(j+1)=-0.4;
        ep=physicalEval(zp);em=physicalEval(zm);
        Jc(:,j)=(cable_center_from_x(ep.xEnd,s.P.cableL)- ...
                 cable_center_from_x(em.xEnd,s.P.cableL))/0.8;
        Jphi(j)=(ep.residual(1)-em.residual(1))/0.8;
    end
    projection=ctx.lateral.'*Jc;
    geomSigns(:,i)=sign(projection(:));
    geomRows(end+1,:)={s.label,s.t,ctx.name,ctx.aspect, ...
        mat2str(ctx.major.',7),mat2str(ctx.lateral.',7), ...
        mat2str(ctx.vApproach.',7),Tval, ...
        mat2str(Jc(:,1).',8),mat2str(Jc(:,2).',8), ...
        projection(1),projection(2),Jphi(1),Jphi(2), ...
        eh.h0(1),ctx.support,norm(projection)}; %#ok<AGROW>
end
% Candidate pool covers the input box, both orientation-dependent lateral
% directions, multiple thrusts, and positive-thrust relinearized QPs.
pool=zeros(3,0);source=cell(1,0);
[pool,source]=add_unique_seed(pool,source,clipped,'clipped_raw');
[pool,source]=add_unique_seed(pool,source,[10;clipped(2:3)],'mid_thrust');
[pool,source]=add_unique_seed(pool,source,[15;clipped(2:3)],'high_thrust');
if hasWitness && all(isfinite(gridWitness(:)))
    [pool,source]=add_unique_seed(pool,source,gridWitness(:),'original_grid_witness');
end
for j=1:numel(allAnchorQP)
    [pool,source]=add_unique_seed(pool,source,allAnchorQP{j},'positive_thrust_QP');
end
% Include the whole original physical torque square, NOT just x direction.
Tgrid=[0,2,4,6,8,10,12,14,16,18,20];
if hasWitness && all(isfinite(gridWitness(:)))
    Tgrid=unique([Tgrid,clipped(1),max(0,min(20,gridWitness(1)))]);
else
    Tgrid=unique([Tgrid,clipped(1)]);
end
tauGrid=[-1,-0.5,0,0.5,1];
for k=1:numel(Tgrid)
    for j=1:numel(tauGrid)
        for l=1:numel(tauGrid)
            [pool,source]=add_unique_seed(pool,source, ...
                [Tgrid(k);tauGrid(j);tauGrid(l)],'physical_box_grid');
        end
    end
end
for k=1:3
    T= [5,10,15];Tval=T(k);
    signs=geomSigns(:,k);signs(signs==0)=1;
    [pool,source]=add_unique_seed(pool,source, ...
        [Tval;signs],'positive_lateral_geometric_seed');
    [pool,source]=add_unique_seed(pool,source, ...
        [Tval;-signs],'negative_lateral_geometric_seed');
end
n=size(pool,2);R=NaN(2*numel(s.parts),n);scoreBase=Inf(1,n);
lateralShift=NaN(1,n);hEnd=NaN(1,n);isSafe=false(1,n);
for j=1:n
    try
        e=physicalEval(pool(:,j));rr=e.residual(:);
        if numel(rr)~=size(R,1),error('ID3Study:ConstraintCount','Unexpected number of constraints');end
        R(:,j)=rr;
        scoreBase(j)=normalized_input_cost(pool(:,j),raw,[10;1;1]);
        lateralShift(j)=ctx.lateral.'*(cable_center_from_x(e.xEnd,s.P.cableL)-referenceCenter);
        hEnd(j)=min(e.hEnd(:));
        isSafe(j)=all(rr>=1e-9);
    catch
    end
end
% Lateral changes are measured relative to the same-state clipped-NOMINAL
% one-step ZOH continuation, not relative to stationary world coordinates.
% Save the source of each evaluated feasible/grid/geometry candidate only when
% it carries independent information. The ZIP includes full optimization
% data and does not create any persistent uncompressed result directories.
for j=1:n
    if ~all(isfinite(R(:,j))),continue;end
    if isSafe(j) || ~strcmp(source{j},'physical_box_grid')
        seedRows(end+1,:)={s.label,s.t,source{j},pool(1,j), ...
            pool(2,j),pool(3,j),min(R(1:numel(s.parts),j)), ...
            min(R(numel(s.parts)+1:end,j)),min(R(:,j)), ...
            scoreBase(j),lateralShift(j),isSafe(j),hEnd(j), ...
            max(abs(pool(2:3,j))),s.assumptions}; %#ok<AGROW>
    end
end
weightMethods={ ...
    'box_normalized',[10;1;1]; ...
    'thrust_more_expensive',[5;1;1]; ...
    'torques_less_expensive',[10;2;2]; ...
    'paper_scale_comparison_only',[100*s.dt^2;2.5;2.5]};
% This test does not automatically adopt the OTHER aircraft's scale.
useFmincon=(exist('fmincon','file')==2 || exist('fmincon','file')==6);
opts=[];
if useFmincon
    try
        opts=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
            'MaxIterations',45,'MaxFunctionEvaluations',220, ...
            'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-8, ...
            'StepTolerance',1e-10);
    catch
        useFmincon=false;
    end
end
% Scaling each inequality is a numerical transformation ONLY.
% Its sign, the fourth-order lambda, and the original CBF are unchanged.
e0=physicalEval(clipped);
try
    [J0,~,~]=jacobian_box(evalF,[clipped(1)-base;clipped(2:3)], ...
        lb,ub,[.25;.075;.075]);
    rowScale=max([abs(e0.residual(:)),sum(abs(bsxfun(@times, ...
        J0,([20;2;2]).')),2),1e-8*ones(numel(e0.residual),1)],[],2);
catch
    rowScale=max(abs(e0.residual(:)),1e-6);
end
assert(numel(rowScale)==numel(e0.residual));
bestDetail=cell(size(weightMethods,1),1);
for wi=1:size(weightMethods,1)
    wName=weightMethods{wi,1};sig=weightMethods{wi,2}(:);
    costs=Inf(1,n);
    for j=1:n
        if isSafe(j)
            costs(j)=normalized_input_cost(pool(:,j),raw,sig);
        end
    end
    [bestCost,bestIndex]=min(costs);
    best=[NaN;NaN;NaN]; method='NO_FEASIBLE_SEED';
    if isfinite(bestCost)
        best=pool(:,bestIndex);method='FEASIBLE_GRID_SEED';
    end
    optEvals=0;
    % Try the most promising distinct seeds, plus the physical nominal
    % anchor, so the NLP is not trapped at only ONE linearized T=0 point.
    sorted=sortrows([(1:n).',costs(:), ...
        normalized_cost_for_pool(pool,raw,sig).'],[2,3]);
    startIdx=sorted(isfinite(sorted(:,2)),1);
    startIdx=unique([startIdx(1:min(3,numel(startIdx))); ...
        nearest_seed_index(pool,clipped)],'stable');
    if useFmincon
        for kk=1:numel(startIdx)
            z0=pool(:,startIdx(kk));
            try
                % Change coordinates to dimensionless xi=(u-raw)/sig.
                % The nonlinear CBF still sees only the physical ZOH u.
                xi0=(z0-raw)./sig;
                xiLo=(lo-raw)./sig;xiHi=(hi-raw)./sig;
                [xiTry,~,~,out]=fmincon( ...
                    @(xi) .5*sum(xi(:).^2), ...
                    xi0,[],[],[],[],xiLo,xiHi, ...
                    @(xi) original_nonlinear_constraint(raw+sig.*xi(:), ...
                        physicalEval,rowScale,1e-9),opts);
                if isstruct(out) && isfield(out,'funcCount')
                    optEvals=optEvals+out.funcCount;
                end
                ztry=raw+sig.*xiTry(:);
                ztry=min(hi,max(lo,ztry));
                ee=physicalEval(ztry);rr=ee.residual(:);
                if all(rr>=1e-9) && ...
                        normalized_input_cost(ztry,raw,sig)<bestCost
                    bestCost=normalized_input_cost(ztry,raw,sig);
                    best=ztry;method='MULTISTART_FMINCON_SQP';
                end
            catch
                % Preserve the independent feasible-grid certificate even
                % if a local optimizer fails on one start.
            end
        end
    end
    if isfinite(bestCost)
        % Toolbox-free feasible line refinement never declares an unsafe
        % numerical candidate to be safe; fmincon solutions also checked.
        [zref,cref,ev]=feasible_line_refine(best,raw,sig,physicalEval,1e-9);
        optEvals=optEvals+ev;
        if cref<bestCost
            bestCost=cref;best=zref;
            method=[method '+FEASIBLE_LINE_REFINE'];
        end
    end
    if all(isfinite(best))
        eb=physicalEval(best);
        minres=min(eb.residual(:)); pass=all(eb.residual(:)>=1e-9);
        phi=min(eb.residual(1:numel(eb.h0)));
        guard=min(eb.residual(numel(eb.h0)+1:end));
        lat=ctx.lateral.'*(cable_center_from_x(eb.xEnd,s.P.cableL)-referenceCenter);
        if lat>1e-10,side='positive_lateral';
        elseif lat<-1e-10,side='negative_lateral';
        else,side='indeterminate';end
    else
        phi=NaN;guard=NaN;minres=NaN;lat=NaN;pass=false;side='none';
    end
    % Reference is the positive/zero-anchored QP from the original
    % standalone (not claimed feasible without nonlinear recheck).
    if ~isempty(allAnchorQP)
        baseQP=allAnchorQP{1};ebQ=physicalEval(baseQP);
        baseQpt=baseQP(1);baseQpr=min(ebQ.residual(:));
    else
        baseQpt=NaN;baseQpr=NaN;
    end
    currentStagesNonnegative=all(refEval.currentStages(:)>=0);
    selectionRows(end+1,:)={s.label,s.t,wName,sig(1),sig(2),sig(3), ...
        method,useFmincon,pass,best(1),best(2),best(3),bestCost, ...
        phi,guard,minres,baseQpt,baseQpr, ...
        min(costs),optEvals,lat,side, ...
        currentStagesNonnegative,'RK4_NUMERIC_ONLY_NOT_CERTIFIED'}; %#ok<AGROW>
    bestDetail{wi}=struct('weightName',wName,'bestPhysicalInput',best, ...
        'cost',bestCost,'passedBothOriginalNonlinearRows',pass, ...
        'method',method,'numericalNotCertified',true);
end
detail=struct('geometry',ctx,'selection',{bestDetail}, ...
    'candidateCount',n,'feasibleSeeds',sum(isSafe), ...
    'optimizerAvailable',useFmincon,'model','actual physical ZOH', ...
    'warning','Nonconvex best-found only, reconstruct modes are not exact flight replay');
end

function [c,ceq]=original_nonlinear_constraint(z,physicalEval,rs,margin)
% fmincon requires c(z)<=0; all ORIGINAL constraints remain hard.
e=physicalEval(z(:));c=(margin-e.residual(:))./rs(:);ceq=[];
end

function [best,bestCost,count]=feasible_line_refine(z,raw,sig,fun,margin)
% Along a path from a verified-safe point toward the clipped HLC target,
% search for the cheapest VERIFIED-safe point (not just the last safe point).
lo=[0;-1;-1];hi=[20;1;1];best=z(:);count=0;
bestCost=normalized_input_cost(best,raw,sig);
target=min(hi,max(lo,raw));
for t=linspace(0,1,65)
    v=z(:)*(1-t)+target*t;
    e=fun(v);count=count+1;
    if all(e.residual(:)>=margin)
        j=normalized_input_cost(v,raw,sig);
        if j<bestCost,bestCost=j;best=v;end
    end
end
end

function score=normalized_input_cost(z,raw,sig)
score=0.5*sum(((double(z(:))-double(raw(:)))./double(sig(:))).^2);
end

function scores=normalized_cost_for_pool(pool,raw,sig)
scores=0.5*sum(bsxfun(@rdivide,bsxfun(@minus,pool,raw(:)),sig(:)).^2,1);
end

function idx=nearest_seed_index(pool,z)
[~,idx]=min(sum(bsxfun(@minus,pool,z(:)).^2,1));
end

function [pool,source]=add_unique_seed(pool,source,z,label)
z=z(:);
if numel(z)~=3 || any(~isfinite(z)),return;end
lo=[0;-1;-1];hi=[20;1;1];z=max(lo,min(hi,z));
if ~isempty(pool) && any(max(abs(bsxfun(@minus,pool,z)),[],1)<1e-9)
    return;
end
pool(:,end+1)=z;source{end+1}=label;
end

function C=cable_center_from_x(x,L)
C=x(8:10)-0.5*L*x(14:16);
end

function ctx=obstacle_geometry_context(s,geometry)
% Obstacles can be rotated; their principal axes are eigenvectors of Q.
parts=s.parts; C=cable_center_from_x(s.x,s.P.cableL);
vC=s.x(11:13)-.5*s.P.cableL*cross(s.x(17:19),s.x(14:16));
h=geometry_clearance(s.x,s.P,s.geom,parts);
[~,ix]=min(h);p=parts(ix);
if strcmp(char(p.kind),'sphere')
    if isfield(p,'params'),a=double(p.params.radius);else,a=double(p.radius);end
    Q=(a*a)*eye(3);
else
    if isfield(p,'params'),a=double(p.params.axes(:));else,a=double(p.axes(:));end
    if isfield(p,'R'),Q=double(p.R)*diag(a.^2)*double(p.R).';
    else,Q=diag(a.^2);end
end
[V,D]=eig((Q+Q.')/2);[vals,order]=sort(diag(D),'descend');
major=V(:,order(1));if dot(major,[0;0;1])<0,major=-major;end
vApproach=vC-major*dot(major,vC);
if norm(vApproach)<1e-8
    r=C-p.center(:);vApproach=r-major*dot(major,r);
end
if norm(vApproach)<1e-8
    tmp=[1;0;0];if abs(dot(tmp,major))>.9,tmp=[0;1;0];end
    vApproach=tmp-major*dot(major,tmp);
end
vApproach=vApproach/norm(vApproach);
lateral=cross(major,vApproach);lateral=lateral/norm(lateral);
n=(C-p.center(:));n=n/norm(n);
if isfield(p,'obstacleName'),name=char(p.obstacleName);
elseif isfield(p,'name'),name=char(p.name);
else,name=sprintf('obs_%d',ix);end
ctx=struct('name',name,'C0',C,'major',major,'lateral',lateral, ...
    'vApproach',vApproach,'support',sqrt(n.'*Q*n), ...
    'aspect',sqrt(vals(1)/max(vals(end),realmin)), ...
    'allGeometry',geometry,'warning', ...
    'Lateral axis is orientation/approach-informed, not a route decision');
end

function csv_write(file,headers,data)
f=fopen(file,'w');
if f<0,error('ID3Study:Save','Cannot write %s',file);end
cleaner=onCleanup(@() fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',strjoin(headers,','));
for i=1:size(data,1)
   a=cell(1,size(data,2));
   for j=1:size(data,2)
       v=data{i,j};
       if islogical(v),a{j}=num2str(double(v));
       elseif isnumeric(v)
           if isempty(v),a{j}='';
           elseif isscalar(v),a{j}=sprintf('%.15g',v);
           else,a{j}=mat2str(v);end
       else
           a{j}=char(string(v));
       end
       a{j}=strrep(a{j},',',';');
   end
   fprintf(f,'%s\n',strjoin(a,','));
end
end

function write_summary(file,R)
f=fopen(file,'w');
if f<0,error('ID3Study:Save','Cannot write summary');end
cleaner=onCleanup(@() fclose(f)); %#ok<NASGU>
fprintf(f,'CBF ID3 standalone, offline, ORIGINAL PROGRAM UNMODIFIED\n');
fprintf(f,'STANDALONE ONLY: QP objective centered on RAW HLC input, physical bounds as hard constraints.\n');
fprintf(f,'Original ID3 still internally clips; no production controller changes.\n');
fprintf(f,'Mode: %s\n',R.mode);
if strcmp(R.mode,'reconstructed')
  fprintf(f,['RECONSTRUCTION ONLY: drone position/input from log, ', ...
     'other 19-state components and barrier memory are hypothesized.\n']);
  fprintf(f,'Do not attribute 26.450 s failure cause to an exact replay.\n');
end
fprintf(f,['Original obstacle: scene_cylinders_high_side / a, ', ...
    'single_ellipsoid, not a synthetic sphere.\n']);
fprintf(f,'P from DRONE_PARAM_SUSPENDED_LOAD, mL=0.14, L=2 in log reconstruction.\n');
fprintf(f,'lambda=[2 4 8 16], dt=0.025 if reconstructed.\n\n');
fprintf(f,'THEORY: H=diag(w_i/s_i^2), dimensionless input xi=deltaU_i/s_i.\n');
fprintf(f,['For discrete second difference delta2T_N,', ...
    ' physically weighted by s_T [N], equivalent nuT=delta2T/dt^2', ...
    ' uses s_nu=s_T/dt^2 [N/s^2].\n']);
fprintf(f,['Paper scale nuT=100 N/s^2 -> delta2T scale = ', ...
    '100*dt^2 N (at 25 ms = 0.0625 N). Paper torque scale 2.5 Nm ', ...
    'belongs to OTHER aircraft; NOT imposed as actual limits.\n']);
fprintf(f,['Pure QP diagonal reparameterization without input limits ', ...
    'cannot change feasibility; it changes the selected correction.\n']);
fprintf(f,['For a single active linear row a*v>=b (without saturation), ', ...
    'delta_v=(b-a*vNom)_+ H^-1*a/(a^T H^-1 a).\n']);
fprintf(f,['Its predicted constraint-correction share is ', ...
    '(s_i*a_i)^2 / sum_j (s_j*a_j)^2 for identity normalized weights. ', ...
    'See weight_sweep.csv. Bound saturation / multiple rows alter it.\n']);
fprintf(f,['A very tall obstacle may make lateral geometric avoidance ', ...
    'preferable, but a one-step high-order constraint plus nominal-', ...
    'distance QP has NO explicit cost for going around it.\n']);
fprintf(f,['Continuous dynamic extension qTdot=nuT, Tdot=qT is DIFFERENT ', ...
    'from sample-constant physical T. See continuous_vs_ZOH.csv.\n\n']);
for j=1:numel(R.details)
    d=R.details{j};c=d.caseData;
    fprintf(f,'CASE %s t=%g\n',c.label,c.t);
    fprintf(f,'  Assumptions: %s\n',c.assumptions);
    fprintf(f,'  Raw HLC nominal [T,tx,ty,tz]: %s\n', ...
        mat2str(c.nominal(:).',8));
    boundedNom=min(max(c.nominal(:),[0;-1;-1;-1]),[20;1;1;1]);
    fprintf(f,'  Original ID3 internally clipped QP reference (comparison only): %s\n', ...
        mat2str(boundedNom(:).',8));
    fprintf(f,['  Standalone objective uses raw HLC reference and QP box constraints. ', ...
        'Original ID3 internally clips the objective reference; ', ...
        'the HLC applies final bounds. Original code remains unchanged.\n']);
    fprintf(f,'  Obstacle axes [m]: %s\n',mat2str(d.geometry.axes.',7));
    fprintf(f,'  Gradient of geometric h wrt load XYZ [m/m]: %s\n', ...
        mat2str(d.spatialClearanceGradXYZ.',7));
    fprintf(f,'  Barrier start h: %s (recorded h=%g; reconstructed discrepancy=%g)\n', ...
      mat2str(d.nominalEval.h0.',7),c.loggedH,d.nominalEval.h0(1)-c.loggedH);
    fprintf(f,'  Phi stages at now: %s\n',mat2str(d.nominalEval.currentStages,7));
    if any(d.nominalEval.currentStages(:)<0)
        fprintf(f,'  ALERT: already outside the nested 4-stage safe set.\n');
    end
    fprintf(f,'  Nominal residual [phi4;ZOH]: %s\n',mat2str(d.nominalEval.residual.',7));
    fprintf(f,'  Jacobian rows [delta2T,tauX,tauY]:\n');
    for k=1:size(d.jacobian,1)
        fprintf(f,'     %s\n',mat2str(d.jacobian(k,:),8));
    end
    fprintf(f,'  FD relative consistency: %.7g\n',d.jacobianConsistency);
    fprintf(f,'  Max achievable margin of EACH individual linearized row: %.7g\n',d.maxLinearizedMargin);
    fprintf(f,'  Joint linearized input-box feasible: %d; witness: %s; margin: %.7g\n', ...
       d.jointLinearizedFeasible,mat2str(d.jointLinearizedWitness.',7),d.jointLinearizedMargin);
    fprintf(f,'  Nonlinear grid witness: %d, best margin: %.7g\n', ...
         d.witnessFound,d.bestNonlinearMargin);
    fprintf(f,'  Best physical [T,tauX,tauY]: %s\n', ...
        mat2str(d.bestNonlinearPhysicalInput.',7));
    fprintf(f,'  Continuous endpoint-matched nuT [N/s^2]: %.7g (paper comparison scale 100)\n', ...
        R.continuousRows{j,5});
    fprintf(f,'  Continuous - ZOH h end [m]: %s\n\n', ...
        mat2str(R.continuousRows{j,9},8));
end
fprintf(f,['INTERPRETATION: Current h and nested phi_i are distinct. ', ...
    'If a current stage is already negative, satisfying a future stage ', ...
    'does not retroactively establish forward invariance.\n']);
fprintf(f,['INTERPRETATION: A negative max of one linearized row proves ', ...
    'only local linearized box infeasibility. No nonlinear witness ', ...
    'does not prove nonlinear global infeasibility.\n']);
fprintf(f,['Generated functions and obstacle approximation were called ', ...
    'read-only. Neither are independently mathematically certified by this test.\n']);
% This final table distinguishes linearized feasibility from the
% re-evaluated nonlinear safety inequalities at the SAME QP candidate.
la=R.linearizationAudit;
validIdx=find(cellfun(@(v) logical(v),la(:,11)));
falsePass=0;
for ii=validIdx(:).'
    if ~logical(la{ii,12}),falsePass=falsePass+1;end
end
fprintf(f,'\nNEW DIAGNOSTIC: linearized safety passes but nonlinear row fails = %d / %d passing linearized rows.\n', ...
    falsePass,numel(validIdx));
fprintf(f,['Each row in linearization_audit.csv shows its Taylor prediction, ', ...
    'actual nonlinear result, and curvature remainder.\n']);
fprintf(f,['fd_sensitivity_sweep.csv checks small/large torque steps; ', ...
    'a zero numerical difference is not proof of zero true sensitivity.\n']);
fprintf(f,['input_path_nonlinearity.csv follows physically bounded candidates ', ...
    'from nominal to linear QP and to the nonlinear witness.\n']);
fprintf(f,['raw_vs_clipped_objective.csv changes ONLY the objective centre; ', ...
    'the physical input bounds and CBF constraints stay unchanged.\n']);
fprintf(f,['digital_vs_continuous_accel.csv verifies nu_endmatch=2*nu_digital ', ...
    'for the specified initial continuous T/q. These are different ', ...
    'actuator realizations, not interchangeable control laws.\n']);
fprintf(f,['geometry_direction.csv shifts the load centre XYZ by 0.1 m ', ...
    'to isolate obstacle geometry from actuator response.\n']);
fprintf(f,['NEW JOINT NONLINEAR STUDY: positive_thrust_anchor_QP.csv compares ', ...
    '8 physical in-box linearization anchors; every resulting QP candidate ', ...
    'is re-evaluated under the nonlinear generated CBF.\n']);
fprintf(f,['geometry_actuation_map.csv computes the obstacle principal axis ', ...
    'from the ORIGINAL rotated ellipsoid covariance, the approach-based ', ...
    'lateral direction, and the actual ZOH load-centre torque sensitivities.\n']);
fprintf(f,['nonlinear_joint_selection.csv minimizes RAW HLC normalized ', ...
    'quadratic deviation subject to both ORIGINAL nonlinear CBF residuals ', ...
    'and the physical input box, with multistart fmincon SQP when available; ', ...
    'otherwise a deterministic feasible restoration search.\n']);
fprintf(f,['The selected solution is BEST FOUND in a NONCONVEX 3-input ', ...
    'problem; not certified global optimum, not a control-law deployment.\n']);
fprintf(f,'\nNEW JOINT NONLINEAR SELECTION BY CASE:\n');
ns=R.nonlinearJointSelectionAudit;
for jj=1:size(ns,1)
    fprintf(f,['  %s %s | method=%s | nextCond=%g | ', ...
        '[T,tauX,tauY]=[%g,%g,%g] | objective=%g | ', ...
        'minResidual=%g | currentNestedOK=%g\n'], ...
        ns{jj,1},ns{jj,3},ns{jj,7},ns{jj,9}, ...
        ns{jj,10},ns{jj,11},ns{jj,12},ns{jj,13}, ...
        ns{jj,16},ns{jj,23});
end
fprintf(f,['Passing the next nonlinear inequalities is NOT forward ', ...
    'invariance if any current nested stage is negative, nor is RK4 ', ...
    'an interval-validated physical safety proof.\n']);
fprintf(f,['No arbitrary left/right route preference is imposed before ', ...
    'the later reference-trajectory correction is introduced.\n']);
fprintf(f,['The positive-anchor and nonlinear searches do not change ', ...
    'lambda, dynamics, obstacle geometry, or any source files.\n']);
if strcmp(R.mode,'reconstructed')
    fprintf(f,['CAUTION: All nonlinear/linear classification conclusions ', ...
       'are for RECONSTRUCTED states, NOT exact flight snapshots.\n']);
end
end

function cleanup_temporary_directory(dirPath)
% Remove staging CSV/MAT/TXT and temporary ZIP; never touch program files.
if exist(dirPath,'dir')==7
    try
        rmdir(dirPath,'s');
    catch ME
        warning('ID3Study:Cleanup','Could not clean temporary directory: %s',ME.message);
    end
end
end
