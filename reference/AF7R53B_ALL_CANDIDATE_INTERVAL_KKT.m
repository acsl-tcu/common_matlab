function OUT = AF7R53B_ALL_CANDIDATE_INTERVAL_KKT(replanner,B49,R50A)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R53B | ALL-CANDIDATE OUTWARD INTERVAL KKT SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | outwardKKT=1 | StageBIntervalAuthority=0 | productionT1Selected=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r53_allcandidate_mex','file')==3);
assert(exist('af7r53_interval_kkt_mex','file')==3);

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);
if nargin<2||isempty(B49)
    assert(isfield(AUX,'B49'),'[AF7R53B] B49 unavailable.');B49=AUX.B49;
end
if nargin<3||isempty(R50A)
    if isfield(AUX,'R50A'),R50A=AUX.R50A;else,R50A=struct();end %#ok<NASGU>
end

AF7R53B_PRIME_ALLCANDIDATE_REFERENCE(replanner,M);

CTX=af7r49_replay_context(replanner,E);
pkt=af7r34_make_core_packet(replanner,CTX,[],M);
SC0=E.speed_certificate;
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);

t=double(B49.t1(:).');pass=logical(B49.success(:).');
d=diff([false pass false]);beg=find(d==1);fin=find(d==-1)-1;
term=find(fin==numel(pass),1,'last');assert(~isempty(term));
kL=beg(term);assert(kL>1);
tFail=t(kL-1);tPass=t(kL);

lo=tFail;hi=tPass;
for it=1:64
    mid=0.5*(lo+hi);
    if ~(mid>lo&&mid<hi),break;end
    P=pkt;P.t1=mid;
    R=af7r53_allcandidate_mex(1,P);
    if logical(R.success),hi=mid;else,lo=mid;end
end

P=pkt;P.t1=lo;RL=af7r53_allcandidate_mex(1,P);
P.t1=hi;RU=af7r53_allcandidate_mex(1,P);

fprintf(' authority source                         = %s\n',authoritySource);
fprintf(' diagnostic boundary bracket [s]          = [%.17g, %.17g]\n',lo,hi);
fprintf(' width [s]                                = %.17g\n',hi-lo);
fprintf(' lower/upper success                      = %d / %d\n',logical(RL.success),logical(RU.success));
fprintf(' lower/upper reason                       = %g / %g\n',double(RL.reason_code),double(RU.reason_code));

PL=pkt;PL.t1=lo;A0=af7r36_single_core_mex(1,PL);
PU=pkt;PU.t1=hi;A1=af7r36_single_core_mex(1,PU);
eqEnds=isequal(logical(A0.success),logical(RL.success)) && ...
    isequal(double(A0.reason_code),double(RL.reason_code)) && ...
    isequal(logical(A1.success),logical(RU.success)) && ...
    isequal(double(A1.reason_code),double(RU.reason_code));

nL=double(RL.all_candidate_count);nU=double(RU.all_candidate_count);
sameCount=(nL==nU&&nL>=1);

fprintf('\n[A. ALL-CANDIDATE TOPOLOGY]\n');
fprintf(' AF7R36 endpoint result equivalence        = %d\n',eqEnds);
fprintf(' lower/upper candidate count              = %.0f / %.0f\n',nL,nU);
fprintf(' candidate count unchanged                = %d\n',sameCount);

allBranchStable=sameCount;
allRowsStable=sameCount;
allKKTClosed=sameCount;
K=cell(1,nU);

if sameCount
    for c=1:nU
        sgL=double(RL.all_candidate_signs{c}(:).');
        sgU=double(RU.all_candidate_signs{c}(:).');
        acL=double(RL.all_candidate_active_set{c}(:).');
        acU=double(RU.all_candidate_active_set{c}(:).');
        AL=double(RL.all_candidate_rows_A{c});
        AU=double(RU.all_candidate_rows_A{c});
        bL=double(RL.all_candidate_rows_b{c});
        bU=double(RU.all_candidate_rows_b{c});

        branchStable=isequal(sgL,sgU)&&isequal(acL,acU);
        rowsStable=isequal(AL,AU)&&isequal(bL,bU);
        allBranchStable=allBranchStable&&branchStable;
        allRowsStable=allRowsStable&&rowsStable;

        fprintf(' candidate %.0f signs                      = %s\n',c,mat2str(sgU));
        fprintf(' candidate %.0f active set                 = %s\n',c,mat2str(acU));
        fprintf(' candidate %.0f branch/rows stable         = %d / %d\n',c,branchStable,rowsStable);
        fprintf(' candidate %.0f lower speed/witness        = %d / %d | witness=%.15g\n', ...
            c,logical(RL.all_candidate_speed_certified(c)), ...
            logical(RL.all_candidate_finite_witness(c)), ...
            double(RL.all_candidate_witness_max(c)));
        fprintf(' candidate %.0f upper speed certified      = %d | VQ=%.15g\n', ...
            c,logical(RU.all_candidate_speed_certified(c)), ...
            double(RU.all_candidate_speed_uav(c)));

        if branchStable&&rowsStable&&numel(acU)==1&&numel(RU.component_start)==1
            signTag=0;if ~isempty(sgU),signTag=sgU(1);end
            K{c}=af7r53_interval_kkt_mex( ...
                lo,hi,double(RU.t0),double(RU.component_start(1)), ...
                double(RU.component_end(1)),AU,bU,acU(1),signTag,c);
            fprintf(' candidate %.0f interval KKT closed         = %d\n',c,logical(K{c}.closed));
            fprintf(' candidate %.0f lambda interval             = [%.17g %.17g]\n', ...
                c,K{c}.lambda(1),K{c}.lambda(2));
            fprintf(' candidate %.0f min inactive slack LB       = %.17g\n', ...
                c,double(K{c}.min_inactive_slack_lb));
            fprintf(' candidate %.0f cost interval               = [%.17g %.17g]\n', ...
                c,K{c}.cost_interval(1),K{c}.cost_interval(2));
            allKKTClosed=allKKTClosed&&logical(K{c}.closed);
        else
            allKKTClosed=false;
        end
    end
end

costOrderClosed=true;
for i=1:numel(K)
    for j=i+1:numel(K)
        if isempty(K{i})||isempty(K{j}),costOrderClosed=false;continue;end
        Ii=double(K{i}.cost_interval);Ij=double(K{j}.cost_interval);
        if ~((Ii(2)<Ij(1))||(Ij(2)<Ii(1))),costOrderClosed=false;end
    end
end

fprintf('\n[B. OUTWARD KKT PROOF]\n');
fprintf(' all candidate branches stable            = %d\n',allBranchStable);
fprintf(' all corridor rows endpoint-identical     = %d\n',allRowsStable);
fprintf(' all candidate interval KKT closed        = %d\n',allKKTClosed);
fprintf(' pairwise cost intervals disjoint         = %d\n',costOrderClosed);

rho=max([double(E.dependency_signature.r_drone), ...
    double(E.dependency_signature.cable_length)+double(E.dependency_signature.r_load), ...
    double(E.dependency_signature.cable_length)+double(E.dependency_signature.r_cable)]);
Vlim=(double(E.dependency_signature.sensor_range)-rho)/ ...
    (double(E.dependency_signature.horizon)+double(E.dependency_signature.dt));

fprintf('\n[C. STAGE-B INTERVAL GATE]\n');
fprintf(' finite-sensing VQ limit [m/s]            = %.15g\n',Vlim);
fprintf(' lower selected finite witness [m/s]      = %.15g\n',double(RL.diagnostic_witness_max));
fprintf(' lower witness - limit [m/s]              = %.15g\n',double(RL.diagnostic_witness_max)-Vlim);
fprintf(' upper selected certified VQ [m/s]        = %.15g\n',double(RU.speed_uav));
fprintf(' upper limit - VQ [m/s]                   = %.15g\n',Vlim-double(RU.speed_uav));
fprintf(' outward Stage-B whole-interval proof     = 0\n');
fprintf(' reason                                   = leaf/control/reference interval propagation is next obligation\n');

prereqClosed=eqEnds&&allBranchStable&&allRowsStable&&allKKTClosed;
fprintf('\n[D. FORMAL GATE]\n');
fprintf(' all-candidate KKT prerequisite closed    = %d\n',prereqClosed);
fprintf(' Stage-B interval proof closed            = 0\n');
fprintf(' terminal-boundary proof closed           = 0\n');
fprintf(' production t1 selected                   = 0\n');
fprintf(' unresolved interval policy               = FAIL CLOSED\n');
fprintf(' next proof                               = OUTWARD_STAGE_B_LEAF_PROPAGATION_ON_FIXED_KKT_INTERVAL\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R53B_ALL_CANDIDATE_INTERVAL_KKT_V1';
OUT.boundary_bracket=[lo hi];
OUT.af7r36_endpoint_equivalence=eqEnds;
OUT.candidate_count=nU;
OUT.all_branches_stable=allBranchStable;
OUT.all_rows_stable=allRowsStable;
OUT.all_kkt_closed=allKKTClosed;
OUT.cost_order_closed=costOrderClosed;
OUT.kkt=K;
OUT.stageB_interval_closed=false;
OUT.production_t1_selected=false;
assignin('base','AF7R53B_ALL_CANDIDATE_INTERVAL_KKT_OUT',OUT);
end
