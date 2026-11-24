function result = CBF_ID3_PAPER_EXTENSION_TEST(varargin)
% CBF_ID3_PAPER_EXTENSION_TEST - OFFLINE, SINGLE FILE; NO PROJECT MODIFICATION.
%
% Liu et al. (2026), arXiv:2610.14522, equations (13), (15), (17):
%   dot(T)=qT, dot(qT)=nuT;  w=[nuT;tauX;tauY], tauZ=nominal (fixed).
%   T(k+1)=T(k)+dt*qT(k)+0.5*dt^2*nuT(k),
%   qT(k+1)=qT(k)+dt*nuT(k), w(t)=w(k) for the whole sample.
%   One affine fourth-order HOCBF constraint and one 3-variable QP.
% The PAPER proves its CT claim for a different (13-state, spherical obstacle)
% system. Here ONLY the extension equations/QP architecture are adopted;
% the pre-existing GENERATED 21-state jet implements this project's ORIGINAL
% 19-state drone/cable + full rotating ellipsoid safety function.
%
% Usage, after placing this file in common_matlab root:
%    result = CBF_ID3_PAPER_EXTENSION_TEST;
%    result = CBF_ID3_PAPER_EXTENSION_TEST('snapshot','my_snapshot.mat');
% Snapshot must contain x19 (19x1), optionally T0, qT0, uHLC (4x1), P.
% It is not necessary to run or modify Sim, HLC, CBF, or the generator.
% Output: ONE CBF_ID3_PAPER_EXTENSION_RESULTS.zip beside THIS file.
%
% IMPORTANT: Default states are RECONSTRUCTED from rounded console P and
% assumed level body/vertical cable/zero angular rates. These are NOT the
% actual 19-state, torque data, or previous CBF memory from the flight.
% The code does NOT claim 25ms safety, forward invariance, or real-time WCET.

oldpath=path; pathGuard=onCleanup(@() path(oldpath)); %#ok<NASGU>
here=fileparts(mfilename('fullpath'));
need={'DRONE_PARAM_SUSPENDED_LOAD','DRONE_OBSTACLES', ...
  'CBF_COMMON_MODEL','CBF_REAL_D2T_OBSTACLE_EVAL'};
for i=1:numel(need)
    if exist(need{i},'file')~=2
        addpath(genpath(here),'-begin');break;
    end
end
for i=1:numel(need)
    if exist(need{i},'file')~=2
        error('PaperIHOCBF:Missing','%s not found; place file at project root',need{i});
    end
end

par=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
P=struct('m',double(par.mass),'mL',0.14, ...
    'J',double([par.jx;par.jy;par.jz]),'gravity',double(par.gravity), ...
    'cableL',2.0,'dstx',0,'dsty',0);
world=DRONE_OBSTACLES('scene_cylinders_high_side','single_ellipsoid','none');
idx=find(strcmp({world.parts.obstacleName},'a'),1);
if isempty(idx),error('PaperIHOCBF:Scene','Obstacle a missing');end
part=world.parts(idx);
G=struct('collisionMode','cable_ellipsoid', ...
    'cableAxisShort',0.20,'cableAxisLong',P.cableL/2+0.02, ...
    'cableCenterFromLoad',0.5);
lambda=[2;4;8;16];dt=0.025;
Tbox=[0,20];tauBox=1;
% ν limits are a STUDY SWEEP, not values claimed by Liu et al.
nuCaps=[100,1000,10000];
% qT bounds are ALSO study assumptions; no actuator identification is implied.
qLimit=40;
% Paper eq.(15a) uses an unscaled 2-norm: 'paper_identity'. Two alternative
% unit normalizations are clearly marked as sensitivity tests, not the paper.
scaleSet=[1,1,1;100,1,1;1000,1,1];
scaleNames={'paper_identity','test_scaled_nu100','test_scaled_nu1000'};

% Columns: time, P_est(xyz), Tprevious_applied, HLC_raw(T,tx,ty,tz).
% P is the estimator drone position; NOT the payload position.
logEarly=[
  26.025,0,8.050,3,8.849,8.850,0,0,0;
  26.050,0,8.062,3,8.849,8.850,0,0,0;
  26.075,0,8.075,3,8.849,8.850,0,0,0;
  26.100,0,8.087,3,8.862,8.840,-2.33e-6,6.0e-7,0];
qValues=[-2,0,2];
snapshotUsed=false;
if numel(varargin)>0
    if numel(varargin)~=2 || ~strcmpi(char(varargin{1}),'snapshot')
        error('PaperIHOCBF:Usage','Use () or (''snapshot'',''filename.mat'')');
    end
    snap=load(char(varargin{2}));
    if ~isfield(snap,'x19')
        error('PaperIHOCBF:Snapshot','Snapshot needs field x19 (19x1)');
    end
    assert(isnumeric(snap.x19)&&numel(snap.x19)==19&&all(isfinite(snap.x19(:))));
    xs=snap.x19(:);
    assert(abs(norm(xs(1:4))-1)<1e-5 && ...
           abs(norm(xs(14:16))-1)<1e-5,'Invalid quaternion/cable unit vectors');
    if isfield(snap,'P'),P=snap.P;end
    if isfield(snap,'T0'),T0=double(snap.T0);else,T0=8.849;end
    if isfield(snap,'qT0'),qValues=double(snap.qT0);else,qValues=0;end
    if isfield(snap,'uHLC')
        uRaw=double(snap.uHLC(:));
    else
        uRaw=[8.85;0;0;0];
    end
    assert(numel(uRaw)==4&&all(isfinite(uRaw)));
    logEarly=[0,zeros(1,3),T0,uRaw(:).'];
    snapshotUsed=true;
end

% Results: each scenario is a *sample-wise* paper-QP experiment.
% exit 1 = affine QP feasible; -1 = affine QP infeasible, 0 = invalid.
head={'time_s','reconstructed','qT0_Nps','nuCap_Nps2','scaleCase', ...
    'T0_N','nominal_Traw_N','nuNominal_Nps2','nuLow','nuHigh', ...
    'HOCBF_h0','HOCBF_phi1','HOCBF_phi2','HOCBF_phi3', ...
    'affine_b','affine_aNu','affine_aTauX','affine_aTauY', ...
    'affine_error_unit_test','qpExit','nuChosen','tauXChosen','tauYChosen', ...
    'HOCBF_phi4_chosen','QP_objective','Tnext_exact','qTnext_exact', ...
    'Tmin_interval_exact','Tmax_interval_exact', ...
    'min_h_grid_uncertified','min_phi1_grid_uncertified', ...
    'min_phi2_grid_uncertified','min_phi3_grid_uncertified', ...
    'end_h','end_phi1','end_phi2','end_phi3', ...
    'qp_ms','validation_ms','current_stage_nonnegative','grid_stage_nonnegative', ...
    'Tbound_exact','qbound_endpoint','affine_row_nonzero'};
res=zeros(0,numel(head));
bench=zeros(0,5);
scens=cell(0,10);
for ii=1:size(logEarly,1)
    row=logEarly(ii,:);t=row(1);
    if snapshotUsed
        x=xs;
    else
        pDrone=row(2:4).';pCable=[0;0;-1];
        pLoad=pDrone+P.cableL*pCable;
        x=[1;0;0;0; 0;0;0; pLoad; 0;0.5;0; pCable; 0;0;0];
    end
    assert(numel(x)==19,'bad reconstruction');
    T0=row(5);raw=row(6:9).';tauZ=raw(4);
    for qq=1:numel(qValues)
        qT0=qValues(qq);y0=[x;T0;qT0];
        tic;
        [b,a,stages,linerr]=affine_constraint(y0,tauZ,P,G,part,lambda);
        tJet=toc*1000;
        % paper eq.(15) one-step HOCBF condition; exact affine in w.
        if linerr>1e-6
            error('PaperIHOCBF:NonAffine', ...
                 'Generated HOCBF jet not affine. error=%.6g',linerr);
        end
        for cc=1:numel(nuCaps)
            nuCap=nuCaps(cc);
            [nuLo,nuHi]=full_interval_nu_bounds(T0,qT0,dt,Tbox(1),Tbox(2));
            nuLo=max([nuLo,-nuCap,(-qLimit-qT0)/dt]);
            nuHi=min([nuHi,nuCap,(qLimit-qT0)/dt]);
            % HLC's raw thrust is an ENDPOINT target. This adapter is
            % necessary because existing HLC does not output nuT; it is
            % NOT in the paper. Raw HLC is never clipped in the objective.
            nuRef=2*(raw(1)-T0-dt*qT0)/dt^2;
            wNom=[nuRef;raw(2:3)];
            lower=[nuLo;-tauBox;-tauBox];upper=[nuHi;tauBox;tauBox];
            for ww=1:size(scaleSet,1)
                s=scaleSet(ww,:).';
                tic;
                [w,exitCode]=one_row_box_qp(b,a,wNom,lower,upper,s);
                qpMs=toc*1000;
                if exitCode==1
                    tic;
                    % y(t) is 21-state, with exactly constant nuT and tau.
                    [yEnd,minH,minStages]=integrate_extension(y0,w,tauZ, ...
                        dt,8,P,G,part,lambda);
                    validationMs=toc*1000;
                    % For constant nuT, thrust extrema are at endpoints
                    % and potentially the parabola's interior vertex.
                    [minT,maxT]=thrust_extrema(T0,qT0,w(1),dt);
                    finalJet=CBF_REAL_D2T_OBSTACLE_EVAL(yEnd, ...
                        [0;0;0],tauZ,P,G,part,lambda);
                    finalStages=finalJet.stages(1:4,1);
                    h4=b+a*w;
                    J=0.5*sum(((w-wNom)./s).^2);
                    validThrust=(minT>=Tbox(1)-1e-10 && ...
                                 maxT<=Tbox(2)+1e-10);
                    validq=abs(yEnd(21))<=qLimit+1e-10;
                    gridStages=all(minStages>=-1e-9);
                else
                    validationMs=NaN;minH=NaN;minStages=NaN(4,1);
                    finalStages=NaN(4,1);w=NaN(3,1);h4=NaN;
                    J=NaN;minT=NaN;maxT=NaN;validThrust=false;
                    validq=false;gridStages=false;yEnd=NaN(21,1);
                end
                currentStages=all(stages(1:4)>=-1e-9);
                datum=[t,double(~snapshotUsed),qT0,nuCap,ww,T0,raw(1),nuRef, ...
                    nuLo,nuHi,stages(1:4).',b,a(:).',linerr,exitCode, ...
                    w(:).',h4,J,yEnd(20),yEnd(21),minT,maxT,minH, ...
                    minStages(2:4).',finalStages(:).',qpMs,validationMs, ...
                    double(currentStages),double(gridStages), ...
                    double(validThrust),double(validq),double(norm(a)>1e-12)];
                assert(numel(datum)==numel(head),'CSV column count mismatch');
                res(end+1,:)=datum; %#ok<AGROW>
                bench(end+1,:)=[t,qT0,tJet,qpMs,validationMs]; %#ok<AGROW>
            end
        end
        obsA=double(part.params.axes(:));
        if isfield(part,'R'),RR=double(part.R);else,RR=eye(3);end
        scens(end+1,:)={t,qT0,T0,raw(1),sprintf('%.9g;',obsA), ...
            sprintf('%.9g;',RR(:)),sprintf('%.9g;',double(part.center(:))), ...
            sprintf('%.9g;',x(8:10)-P.cableL*x(14:16)), ...
            sprintf('%.9g;',x(8:10)-.5*P.cableL*x(14:16)), ...
            sprintf('%.9g;',x)}; %#ok<AGROW>
    end
end

% Independently verify paper Eq.(17) against the exact polynomial solution.
checkY=[8.849;0];wCheck=[100;0.1;-0.1];
T1=checkY(1)+dt*checkY(2)+0.5*dt^2*wCheck(1);
q1=checkY(2)+dt*wCheck(1);
assert(abs(T1-8.88025)<1e-12 && abs(q1-2.5)<1e-12);
assert(all(res(:,end-3)==0 | res(:,end-3)==1));

work=tempname;[ok,msg]=mkdir(work);
if ~ok,error('PaperIHOCBF:Temp','%s',msg);end
clean=onCleanup(@() cleanup_temp(work)); %#ok<NASGU>
write_numeric_csv(fullfile(work,'paper_ihocbf_qp.csv'),head,res);
write_numeric_csv(fullfile(work,'timing_ms.csv'), ...
    {'time_s','qT0','jet_four_evaluations_ms','small_qp_ms','rk8_recheck_ms'},bench);
write_cell_csv(fullfile(work,'scene_and_state.csv'), ...
    {'time_s','qT0_Nps','T0_N','raw_HLC_T_N','obstacle_axes_m', ...
     'obstacle_R_colmajor','obstacle_center_xyz','drone_pos_xyz', ...
     'cable_midpoint_xyz','state_x19_RECONSTRUCTED_UNLESS_SNAPSHOT'},scens);
readme={
 'Paper reference: Yanze Liu et al., arXiv:2610.14522, eqs (13),(15),(17).';
 'ONLY NEW FILE is this test; Sim/HLC/CBF/generator are NEVER modified.';
 'PAPER EXACT: Tdot=qT; qTdot=nuT; Tnext=T+dt*qT+0.5*dt^2*nuT; qTnext=qT+dt*nuT.';
 'PAPER QP: minimize squared distance of w to wNom, subject to fourth-order HOCBF.';
 'APPLICATION DIFFERENCE: 19-state suspended cable, rotating obstacle ellipsoid; tauZ fixed.';
 'APPLICATION DIFFERENCE: original quadrotor paper uses p-sphere h, not cable ellipsoid.';
 'APPLICATION DIFFERENCE: HLC raw T command mapped to nuNom via requested endpoint T.';
 'APPLICATION DIFFERENCE: nuCap, qT bounds, full-interval T bounds are test safety limits.';
 'study scale ID 1=identity paper form, 2=[100,1,1], 3=[1000,1,1] (unit-scaled alternatives).';
 'nuCap=100,1000,10000 N/s^2 are PARAMETER SWEEP, not copied paper actuator bounds.';
 'qT0=-2,0,+2 N/s are hypothetical; actual flight qT0 is NOT in console log.';
 'TIME t is derived from the early flight log, but default x19 is reconstructed.';
 'P in log is drone estimator position, not payload position; cable assumed vertical.';
 'The generated continuous h4 is affine in [nuT,tauX,tauY] at fixed y.';
 'QP is analytically solved for ONE obstacle a and 3 box-bounded inputs.';
 'QP exit=1 means only the affine fourth-stage inequality and boxes are feasible.';
 'For HOCBF invariance, the current stages h,phi1,phi2,phi3 MUST be nonnegative.';
 'No forward-invariance, recursive-feasibility, or intersample certification in this test.';
 'min_h_grid and min_phi*_grid are RK4 sample values, not verified bounds.';
 'The physical simulator currently holds T itself (not nuT) constant over 25 ms.';
 'Consequently, paper-style optimized w must NOT be plugged into the existing Sim as if T-ZOH.';
 'Before installation: match plant thrust update law and certify actuator + sampling errors.';
 'The benchmark measures THIS MATLAB machine only and is NOT a WCET guarantee.';
 'This test does not optimize over a future horizon or fix a lateral escape direction.'};
fid=fopen(fullfile(work,'README.txt'),'w');
if fid<0,error('PaperIHOCBF:Write','Cannot write README');end
fc=onCleanup(@() fclose(fid));
for k=1:numel(readme),fprintf(fid,'%s\n',readme{k});end
clear fc
output=fullfile(here,'CBF_ID3_PAPER_EXTENSION_RESULTS.zip');
zipTmp=fullfile(work,'results.zip');
zip(zipTmp,{'paper_ihocbf_qp.csv','timing_ms.csv','scene_and_state.csv', ...
    'README.txt'},work);
[ok,msg]=movefile(zipTmp,output,'f');
if ~ok,error('PaperIHOCBF:ZIP','%s',msg);end
result=struct('zip',output,'cases',size(res,1), ...
    'qpFeasible',sum(res(:,20)==1), ...
    'assumedPhysicalStates',~snapshotUsed,'certifiedIntersample',false, ...
    'projectModified',false,'paperEquation17Exact',true);
fprintf('[PAPER IHOCBF] %d cases (%d QP feasible). ZIP: %s\n', ...
    result.cases,result.qpFeasible,output);
end

function [b,a,s0,err]=affine_constraint(y,tauZ,P,G,part,lambda)
base=CBF_REAL_D2T_OBSTACLE_EVAL(y,zeros(3,1),tauZ,P,G,part,lambda);
b=base.stages(5,1);s0=base.stages(:,1);
a=zeros(1,3);
for j=1:3
    v=zeros(3,1);v(j)=1;
    t=CBF_REAL_D2T_OBSTACLE_EVAL(y,v,tauZ,P,G,part,lambda);
    a(j)=t.stages(5,1)-b;
end
v=[37;0.07;-0.05];
t=CBF_REAL_D2T_OBSTACLE_EVAL(y,v,tauZ,P,G,part,lambda);
err=abs(t.stages(5,1)-(b+a*v))/max([1,abs(t.stages(5,1)),abs(b+a*v)]);
end

function [nuLow,nuHigh]=full_interval_nu_bounds(T,q,dt,Tmin,Tmax)
% EXACT continuous-time bounds for T(s)=T+q*s+.5*nu*s^2, 0<=s<=dt.
% Each s>0 creates a linear bound on nu. The extreme over all s occurs
% at s=dt, s=2*(T_bound-T)/q (when inside), or s->0.
nuLow=-Inf;nuHigh=Inf;
if T<Tmin-1e-10 || T>Tmax+1e-10,nuLow=Inf;nuHigh=-Inf;return;end
for k=1:2
    if k==1,B=Tmin;else,B=Tmax;end
    A=B-T;
    ss=dt;
    if abs(q)>1e-13
        st=2*A/q;
        if st>0 && st<dt,ss=[ss,st];end %#ok<AGROW>
    end
    cand=2*(A-q*ss)./(ss.^2);
    if k==1
        nuLow=max(nuLow,max(cand));
        if abs(A)<1e-12 && q<0,nuLow=Inf;end
        if abs(A)<1e-12 && abs(q)<1e-13,nuLow=max(nuLow,0);end
    else
        nuHigh=min(nuHigh,min(cand));
        if abs(A)<1e-12 && q>0,nuHigh=-Inf;end
        if abs(A)<1e-12 && abs(q)<1e-13,nuHigh=min(nuHigh,0);end
    end
end
end

function [w,exitCode]=one_row_box_qp(b,a,desired,lo,hi,scale)
% Exact 3-variable strictly convex box QP + ONE linear inequality.
% Lagrange multiplier root: w(lambda)=clip(wNom+lambda*diag(S.^2)*a).
% No Optimization Toolbox, no iterative integrations / nonlinear solver.
w=NaN(3,1);exitCode=0;
if any(lo>hi) || any(isnan([b;a(:);desired(:);lo(:);hi(:);scale(:)]))
    exitCode=-1;return;
end
lo=lo(:);hi=hi(:);desired=desired(:);a=a(:);scale=scale(:);
if any(~isfinite([lo;hi;scale])) || any(scale<=0)
    exitCode=-1;return;
end
base=min(max(desired,lo),hi);
if b+a.'*base>=-1e-10,w=base;exitCode=1;return;end
fMax=b+a.'*(lo.*(a<0)+hi.*(a>=0));
if fMax < -1e-10,exitCode=-1;return;end
if norm(a.*scale)<1e-14,exitCode=-1;return;end
L=0;H=1;
for iter=1:120
    guess=min(max(desired+H*(scale.^2).*a,lo),hi);
    if b+a.'*guess >= 0,break;end
    H=H*2;
end
for iter=1:64
    mid=0.5*(L+H);
    guess=min(max(desired+mid*(scale.^2).*a,lo),hi);
    if b+a.'*guess>=0,H=mid;else,L=mid;end
end
w=min(max(desired+H*(scale.^2).*a,lo),hi);
if b+a.'*w>=-1e-8,exitCode=1;else,exitCode=-1;end
end

function [y,minH,minStages]=integrate_extension(y,w,tauZ,dt,N,P,G,part,lambda)
step=dt/N;minH=Inf;minStages=Inf(4,1);
for j=0:N
    if j>0
        f1=rhs(y,w,tauZ,P);
        f2=rhs(y+0.5*step*f1,w,tauZ,P);
        f3=rhs(y+0.5*step*f2,w,tauZ,P);
        f4=rhs(y+step*f3,w,tauZ,P);
        y=y+step*(f1+2*f2+2*f3+f4)/6;
    end
    stages=CBF_REAL_D2T_OBSTACLE_EVAL(y,[0;0;0],tauZ,P,G,part,lambda);
    minH=min(minH,stages.stages(1,1));
    minStages=min(minStages,stages.stages(1:4,1));
end
end
function dy=rhs(y,w,tauZ,P)
U=[y(20);w(2);w(3);tauZ];
dy=[CBF_COMMON_MODEL(y(1:19),U,P);y(21);w(1)];
end
function [low,high]=thrust_extrema(T,q,nu,dt)
y=[T,T+q*dt+0.5*nu*dt^2];
if abs(nu)>1e-14
    s=-q/nu;
    if s>0 && s<dt,y(end+1)=T+q*s+0.5*nu*s^2;end %#ok<AGROW>
end
low=min(y);high=max(y);
end
function write_numeric_csv(path,head,mat)
fid=fopen(path,'w');if fid<0,error('PaperIHOCBF:CSV','%s',path);end
fc=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',strjoin(head,','));
for k=1:size(mat,1)
    fprintf(fid,'%s\n',strjoin(arrayfun(@(v) sprintf('%.15g',v),mat(k,:), ...
        'UniformOutput',false),','));
end
end
function write_cell_csv(path,head,rows)
fid=fopen(path,'w');if fid<0,error('PaperIHOCBF:CSV','%s',path);end
fc=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',strjoin(head,','));
for k=1:size(rows,1)
    vals=cell(1,size(rows,2));
    for j=1:numel(vals)
        v=rows{k,j};
        if isnumeric(v),vals{j}=sprintf('%.15g',v);
        else,vals{j}=strrep(char(v),',',';');end
    end
    fprintf(fid,'%s\n',strjoin(vals,','));
end
end
function cleanup_temp(path)
if exist(path,'dir'),rmdir(path,'s');end
end
