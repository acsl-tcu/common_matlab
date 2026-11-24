function report = CBF_ID3_CT_VS_ZOH_THEORY_TEST()
% OFFLINE / SINGLE FILE -- original controller, HLC, generator, Sim unchanged.
% Quantitatively audit continuous thrust dynamic extension vs physical T-ZOH
% using the ORIGINAL 19-state suspended-load model and obstacle-a ellipsoid.
%
% Place in common_matlab root; run:
%   report = CBF_ID3_CT_VS_ZOH_THEORY_TEST;
%
% The ONLY permanent result file is CBF_ID3_CT_VS_ZOH_RESULTS.zip, written
% beside this .m. All CSV/TXT/MAT files live inside the zip. The flight
% display lacks actual 19-state, so the scenario is explicitly RECONSTRUCTED.
% NO result below is a certification of 25-ms intersample invariance.
%
% Key input distinction:
%    Physical ZOH:          T(t)=T_k during [t_k,t_k+dt).
%    Continuous extension: dT/dt=qT, dqT/dt=nuT (nuT held for a period).
%    With qT(t_k)=0:       T(t_k+s)=T0+0.5*nuT*s^2.
% These are NOT equivalent even when the interval end thrust coincides.

oldPath=path; restorePath=onCleanup(@() path(oldPath)); %#ok<NASGU>
here=fileparts(mfilename('fullpath'));
need={'DRONE_PARAM_SUSPENDED_LOAD','DRONE_OBSTACLES', ...
      'CBF_COMMON_MODEL','CBF_REAL_D2T_OBSTACLE_EVAL','CBF_REAL_D2T_ZOH_EVAL'};
for i=1:numel(need)
    if exist(need{i},'file')~=2
        addpath(genpath(here),'-begin');
        break
    end
end
for i=1:numel(need)
    if exist(need{i},'file')~=2
        error('ID3ZOH:Missing','Missing %s; put this test in project root.',need{i});
    end
end

par=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
P=struct('m',double(par.mass),'mL',0.14, ...
    'J',double([par.jx;par.jy;par.jz]), ...
    'gravity',double(par.gravity),'cableL',2.0, ...
    'dstx',0,'dsty',0);
world=DRONE_OBSTACLES('scene_cylinders_high_side','single_ellipsoid','none');
ix=find(strcmp({world.parts.obstacleName},'a'),1);
if isempty(ix),error('ID3ZOH:ObstacleA','The original obstacle a is missing');end
part=world.parts(ix);
G=struct('collisionMode','cable_ellipsoid', ...
    'cableAxisShort',0.20,'cableAxisLong',P.cableL/2+0.02, ...
    'cableCenterFromLoad',0.5);
geom=struct('a',G.cableAxisLong,'b',G.cableAxisShort,'centerFromLoad',0.5);
lambda=[2;4;8;16];dt=0.025;Nhi=24;
Tprev=8.849; % From the preceding *printed* flight log, NOT precise internal T.

% Actual flight q,w,pT,wT,vLoad are NOT contained in the console display.
% Only printed estimator drone position P is used; remaining values are
% an explicit hypothesis for sensitivity and math checks, not a replay.
locations=[26.025,0,8.050,3.000; ...
           26.075,0,8.075,3.000; ...
           26.125,0,8.100,3.000];
scenario=cell(0,8); scale=cell(0,13); order4=cell(0,10);
dyn=cell(0,13); compare=cell(0,11); convergence=cell(0,9);
% For unit-conversion diagnostics: exp continuity is a mathematical MODEL,
% not an actuator that the user's existing physical Sim implements.
for ii=1:size(locations,1)
    t=locations(ii,1); pos=locations(ii,2:4).';
    x=make_reconstructed_state(pos,P.cableL);
    h0=variable_normal_clearance(x,part,geom,P.cableL);
    scenario(end+1,:)={t,pos(1),pos(2),pos(3),P.m,P.mL, ...
        h0,'reconstructed vertical cable; level body; vLoad=[0;.5;0]'}; %#ok<AGROW>

    % Analytical 4th derivative: physical thrust held, qT=nuT=0.
    % Unlike the physical plant, continuous extension with nuT~=0 changes
    % thrust smoothly. Coefficients therefore answer different questions.
    ts=[0,5,Tprev,12,20];
    dnu=20;dtau=.1;
    for jj=1:numel(ts)
        T=ts(jj);
        fn=@(nu,tx,ty) jet_h4(x,T,nu,tx,ty,P,G,part,lambda);
        f0=fn(0,0,0);
        aNu=(fn(dnu,0,0)-fn(-dnu,0,0))/(2*dnu);
        aX=(fn(0,dtau,0)-fn(0,-dtau,0))/(2*dtau);
        aY=(fn(0,0,dtau)-fn(0,0,-dtau))/(2*dtau);
        b1=(fn(0,dtau,0)-fn(0,-dtau,0))/(2*dtau);
        % T dependence of the torque coefficient, indicating T*tau terms:
        Tp=min(T+0.5,20);Tm=max(T-0.5,0);
        bp=(jet_h4(x,Tp,0,dtau,0,P,G,part,lambda)- ...
            jet_h4(x,Tp,0,-dtau,0,P,G,part,lambda))/(2*dtau);
        bm=(jet_h4(x,Tm,0,dtau,0,P,G,part,lambda)- ...
            jet_h4(x,Tm,0,-dtau,0,P,G,part,lambda))/(2*dtau);
        mixed=(bp-bm)/(Tp-Tm);
        order4(end+1,:)={t,T,f0,aNu,aX,aY,mixed, ...
            norm([100*aNu,2.5*aX,2.5*aY]), ...
            norm([16000*aNu,aX,aY]),b1}; %#ok<AGROW>
    end

    if ii~=2,continue;end % detailed studies at FIRST intervention t=26.075

    % Exact sampled physical dynamics, not finite differences of T history.
    % Each column is a genuine physical input, jointly capable of changing
    % the state over one sample; no future-constant-input assumption.
    holdTimes=[dt/4,dt/2,dt];
    dTestT=.5;dTestTau=.2;
    for j=1:numel(holdTimes)
        d=holdTimes(j);
        u0=[Tprev;0;0;0];
        h00=zoh_end_h(x,u0,d,Nhi,P,part,geom);
        hT=zoh_end_h(x,u0+[dTestT;0;0;0],d,Nhi,P,part,geom);
        hX=zoh_end_h(x,u0+[0;dTestTau;0;0],d,Nhi,P,part,geom);
        hY=zoh_end_h(x,u0+[0;0;dTestTau;0],d,Nhi,P,part,geom);
        hTX=zoh_end_h(x,u0+[dTestT;dTestTau;0;0],d,Nhi,P,part,geom);
        hTY=zoh_end_h(x,u0+[dTestT;0;dTestTau;0],d,Nhi,P,part,geom);
        dHdT=(hT-h00)/dTestT;
        dHdX=(hX-h00)/dTestTau;
        dHdY=(hY-h00)/dTestTau;
        cTX=(hTX-hT-hX+h00)/(dTestT*dTestTau);
        cTY=(hTY-hT-hY+h00)/(dTestT*dTestTau);
        % Nonzero cross interactions show WHY a physical-input joint-QP
        % cannot be called globally control-affine without qualification.
        scale(end+1,:)={t,d,h00,dHdT,dHdX,dHdY,cTX,cTY, ...
            abs(dHdT)/(abs(dHdX)+abs(dHdY)+eps), ...
            hT-h00,hX-h00,hY-h00, ...
            norm([dHdT*10,dHdX*1,dHdY*1])}; %#ok<AGROW>
    end

    % Compare Taylor jet and physical-ZOH numerical integration. Both use
    % the same state-dependent centre-normal separation h(x), rather than
    % comparing a moving-normal Taylor jet with a fixed-plane guard.
    ctrl=[8.862,0,0;10,0,0;10,.2,-.2;16,0,0];
    for j=1:size(ctrl,1)
        uc=[ctrl(j,:) 0].';
        hnum=zoh_end_h(x,uc,dt,Nhi,P,part,geom);
        e=jet_eval(x,uc(1),0,uc(2),uc(3),P,G,part,lambda);
        powers=dt.^(0:4);
        hTaylor=powers*e.hTaylor(:,1);
        g=CBF_REAL_D2T_ZOH_EVAL(x,uc,P,part,geom,dt,3);
        compare(end+1,:)={t,uc(1),uc(2),uc(3),h0,hnum, ...
            hTaylor,hTaylor-hnum,g.hEnd(1),g.intervalLower(1), ...
            hnum-g.hEnd(1)}; %#ok<AGROW>
    end

    % Holding nuT constant for a step yields a QUADRATIC physical thrust,
    % unlike the user's T-ZOH. Match T(end), NOT T(t) for 0<s<dt.
    for target=[8.862,10,12,16,20]
        nu=2*(target-Tprev)/(dt^2);q0=0;
        xz=zoh_end_state(x,[target;0;0;0],dt,Nhi,P);
        [yc,minCt,minPt]=extended_end_state(x,Tprev,q0,nu,[0;0;0], ...
            dt,Nhi,P,part,geom);
        hZ=variable_normal_clearance(xz,part,geom,P.cableL);
        hC=variable_normal_clearance(yc(1:19),part,geom,P.cableL);
        % Integral thrust over this interval (q0=0) is Tprev*dt+nu*dt^3/6.
        favg=Tprev+(target-Tprev)/3;
        ec=jet_eval(x,Tprev,nu,0,0,P,G,part,lambda);
        hct=dt.^(0:4)*ec.hTaylor(:,1);
        dyn(end+1,:)={t,target,nu,target-Tprev, ...
            favg,hZ,hC,hC-hZ,hct,hct-hC, ...
            minCt,minPt,yc(20)}; %#ok<AGROW>
    end

    % RK step convergence, numerical agreement is NOT validated ODE enclosure.
    uu=[10;.2;-.2;0];
    ref=zoh_end_h(x,uu,dt,96,P,part,geom);
    for n=[2,4,8,16,24,48]
        hn=zoh_end_h(x,uu,dt,n,P,part,geom);
        % h(min within step) is evaluated only on grid of RK nodes:
        [~,hmin]=zoh_path(x,uu,dt,n,P,part,geom);
        convergence(end+1,:)={t,uu(1),uu(2),uu(3),n,hn,ref,hn-ref,hmin}; %#ok<AGROW>
    end
end

% Theoretical statements used in paper comparisons: exact under their
% stated assumptions, not measured proof on this 19-state model.
assert(abs(100*dt^2/2-0.03125)<1e-12);
assert(abs(prod(1+dt*lambda)-1.9404)<1e-12);

% ONE ZIP next to this .m, no loose results or output subfolders.
work=tempname;[yes,msg]=mkdir(work);
if ~yes,error('ID3ZOH:Temp','%s',msg);end
cleanup=onCleanup(@() remove_temp(work)); %#ok<NASGU>
write_table(work,'scenario.csv', ...
 {'time_s','est_drone_x','est_drone_y','est_drone_z','vehicle_mass_kg', ...
  'payload_mass_kg','reconstructed_h_m','assumptions'},scenario);
write_table(work,'sampled_input_sensitivity.csv', ...
 {'time_s','hold_s','h_base_m','dh_dT_perN','dh_dtauX_perNm', ...
  'dh_dtauY_perNm','d2h_dT_dtauX','d2h_dT_dtauY', ...
  'T_to_tau_gradient_ratio','delta_h_by_plus0p5N', ...
  'delta_h_by_plus0p2Nm_tauX','delta_h_by_plus0p2Nm_tauY', ...
  'scaled_gradient_norm_sT10_sTau1'},scale);
write_table(work,'analytic_fourth_order_coefficients.csv', ...
 {'time_s','physical_T_N','h4_at_zero_extended_inputs', ...
  'dh4_dnuT','dh4_dtauX','dh4_dtauY','d_dT_dh4_dtauX', ...
  'normalized_gradient_norm_paperScale','normalized_gradient_norm_D2Tscale', ...
  'torqueX_coefficient_crosscheck'},order4);
write_table(work,'zoh_vs_fourth_order_taylor.csv', ...
 {'time_s','physical_T_N','tauX_Nm','tauY_Nm','h_start', ...
  'h_end_ZOH_RK24','h_end_fourth_order_Taylor', ...
  'Taylor_minus_RK24','fixed_plane_hEnd_ZOH_rk3', ...
  'fixed_plane_intersample_lower_UNCERTIFIED','moving_hEnd_minus_fixed_hEnd'},compare);
write_table(work,'continuous_thrust_integrator_vs_ZOH.csv', ...
 {'time_s','matched_end_thrust_N','nu_continuous_Nps2', ...
  'delta_T_end_N','continuous_mean_thrust_N', ...
  'hEnd_physical_ZOH','hEnd_continuous_Tddot', ...
  'continuous_minus_ZOH_h_m','continuous_fourth_order_Taylor_hEnd', ...
  'continuous_Taylor_minus_RK_h_m','min_h_continuous_grid_ONLY', ...
  'min_h_fixed_plane_grid_ONLY','continuous_recovered_end_T_N'},dyn);
write_table(work,'rk_convergence_NOT_certification.csv', ...
 {'time_s','physical_T_N','tauX_Nm','tauY_Nm', ...
  'rk_substeps','hEnd','reference_RK96','difference_from_RK96', ...
  'sampled_grid_min_h_NOT_verified'},convergence);

note={
    'CBF ID3 CONTINUOUS-vs-ZOH THEORY AUDIT (OFFLINE; ORIGINAL CODE UNMODIFIED)';
    'IMPORTANT: console flight log DOES NOT include 19-state or barrier memory.';
    'All states are RECONSTRUCTED: q=[1,0,0,0], body/cable angular rates=0,';
    'vertical cable, vLoad=[0,0.5,0]. These are NOT a replay or safety proof.';
    'Scene: scene_cylinders_high_side obstacle a, original single ellipsoid.';
    'Physical model: CBF_COMMON_MODEL, 19 states, 25 ms ZOH of T and all torques.';
    'Analytic jet: CBF_REAL_D2T_OBSTACLE_EVAL, hTaylor order4.';
    'The evaluated h is centre-direction support-function SEPARATION SUFFICIENCY.';
    'Negative h does NOT prove literal ellipsoid intersection.';
    'Paper scheme: qTdot=nuT, Tdot=qT, T(s)=T0+q0*s+0.5*nuT*s^2.';
    'ZOH scheme: T(s)=T_k (0<=s<dt), even at sample switches.';
    'With q0=0 and nuT=100 N/s^2, delta T at 25 ms =0.03125 N.';
    'The discrete second difference is a parameterization of a NEW T_k,';
    'NOT a continuous thrust actuator state. It does not change reachability.';
    'Fourth derivative generally contains cross-couplings T*tau. Inspect';
    'analytic_fourth_order_coefficients.csv for their local magnitude.';
    'CT HOCBF checked only at sample times does NOT guarantee true ZOH safety.';
    'Numerical RK and Taylor truncation have NO certified enclosure here.';
    'The endpoint model and fixed-plane intersample guard use different h.';
    'For exact ZOH safety, sufficient condition is min_{s in [0,dt]} h(x(s))>=0';
    'for each protected obstacle, with a PROVEN enclosure of h(x(s)).';
    'An efficient analytic remainder/interval certificate requires additional';
    'derivative bounds and feasibility analysis, NOT established by this test.';
    'Online use: do not install any new controller. Only examine test results.';
    'Original MATLAB files and user state/parameters have not been modified.'};
fid=fopen(fullfile(work,'THEORY_README.txt'),'w');
if fid<0,error('ID3ZOH:Write','Unable to create README');end
finish=onCleanup(@() fclose(fid)); %#ok<NASGU>
for i=1:numel(note),fprintf(fid,'%s\n',note{i});end
clear finish

outZip=fullfile(here,'CBF_ID3_CT_VS_ZOH_RESULTS.zip');
names=dir(fullfile(work,'*.csv'));
fn=cell(numel(names)+1,1);
for i=1:numel(names),fn{i}=names(i).name;end
fn{end}='THEORY_README.txt';
zipTmp=fullfile(work,'result.zip');
zip(zipTmp,fn,work);
[yes,msg]=movefile(zipTmp,outZip,'f');
if ~yes,error('ID3ZOH:ZIPMove','%s',msg);end
report=struct('resultZip',outZip,'cases',size(locations,1), ...
    'isRealFlightReplay',false,'intersampleCertified',false, ...
    'originalProgramModified',false,'physicalInputZOH',true, ...
    'theory','continuous feedback and discrete implementation are distinct');
fprintf('[ID3 CT vs ZOH] Wrote only %s\n',outZip);
end

function x=make_reconstructed_state(pDrone,L)
pT=[0;0;-1];
pLoad=pDrone+L*pT;
x=[1;0;0;0; 0;0;0; pLoad; 0;0.5;0; pT; 0;0;0];
assert(numel(x)==19);
end
function out=jet_eval(x,T,nu,tx,ty,P,G,part,lambda)
y=[x;T;0];v=[nu;tx;ty];
out=CBF_REAL_D2T_OBSTACLE_EVAL(y,v,0,P,G,part,lambda);
end
function h4=jet_h4(x,T,nu,tx,ty,P,G,part,lambda)
r=jet_eval(x,T,nu,tx,ty,P,G,part,lambda);
h4=r.hDerivatives(5,1);
end
function y=zoh_end_state(x,u,dt,steps,P)
y=x;step=dt/steps;
for k=1:steps
    k1=CBF_COMMON_MODEL(y,u,P);
    k2=CBF_COMMON_MODEL(y+0.5*step*k1,u,P);
    k3=CBF_COMMON_MODEL(y+0.5*step*k2,u,P);
    k4=CBF_COMMON_MODEL(y+step*k3,u,P);
    y=y+(step/6)*(k1+2*k2+2*k3+k4);
end
end
function h=zoh_end_h(x,u,dt,steps,P,part,geom)
y=zoh_end_state(x,u,dt,steps,P);
h=variable_normal_clearance(y,part,geom,P.cableL);
end
function [y,hmin,planeMin]=extended_end_state(x,T,q0,nu,tau,dt,steps,P,part,geom)
y=[x;T;q0];step=dt/steps;
hmin=variable_normal_clearance(x,part,geom,P.cableL);
n0=separating_normal(x,part,P.cableL);
planeMin=fixed_plane_clearance(x,part,geom,P.cableL,n0);
for k=1:steps
    k1=ext_rhs(y,nu,tau,P);
    k2=ext_rhs(y+0.5*step*k1,nu,tau,P);
    k3=ext_rhs(y+0.5*step*k2,nu,tau,P);
    k4=ext_rhs(y+step*k3,nu,tau,P);
    y=y+(step/6)*(k1+2*k2+2*k3+k4);
    hmin=min(hmin,variable_normal_clearance(y(1:19),part,geom,P.cableL));
    planeMin=min(planeMin,fixed_plane_clearance(y(1:19),part,geom,P.cableL,n0));
end
end
function yp=ext_rhs(y,nu,tau,P)
u=[y(20);tau(:)];
yp=[CBF_COMMON_MODEL(y(1:19),u,P);y(21);nu];
end
function [states,hmin]=zoh_path(x,u,dt,steps,P,part,geom)
states=x;step=dt/steps;
hmin=variable_normal_clearance(x,part,geom,P.cableL);
for k=1:steps
    states=zoh_end_state(states,u,step,1,P);
    hmin=min(hmin,variable_normal_clearance(states,part,geom,P.cableL));
end
end
function n=separating_normal(x,part,L)
c=x(8:10)-.5*L*x(14:16);
d=c-double(part.center(:));n=d/norm(d);
end
function h=variable_normal_clearance(x,part,geom,L)
n=separating_normal(x,part,L);
h=fixed_plane_clearance(x,part,geom,L,n);
end
function h=fixed_plane_clearance(x,part,geom,L,n)
c=x(8:10)-.5*L*x(14:16);
pT=x(14:16);Q=obstacle_Q(part);
sC=sqrt(geom.b^2+(geom.a^2-geom.b^2)*(n.'*pT)^2);
sO=sqrt(n.'*Q*n);
h=n.'*(c-double(part.center(:)))-sC-sO;
end
function Q=obstacle_Q(part)
if strcmp(char(part.kind),'ellipsoid')
    if isfield(part,'params'),a=double(part.params.axes(:));
    else,a=double(part.axes(:));end
    if isfield(part,'R'),R=double(part.R);else,R=eye(3);end
    Q=R*diag(a.^2)*R.';
elseif strcmp(char(part.kind),'sphere')
    if isfield(part,'params'),a=double(part.params.radius);
    else,a=double(part.radius);end
    Q=a^2*eye(3);
else
    error('ID3ZOH:ObstacleKind','Expected ellipsoid/sphere obstacle');
end
end
function write_table(folder,name,headers,rows)
p=fullfile(folder,name);
fid=fopen(p,'w');
if fid<0,error('ID3ZOH:WriteCSV','Cannot open %s',p);end
clean=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',strjoin(headers,','));
for i=1:size(rows,1)
    str=cell(1,size(rows,2));
    for j=1:size(rows,2)
        value=rows{i,j};
        if isnumeric(value)
            if isscalar(value),str{j}=sprintf('%.16g',value);
            else,error('ID3ZOH:CellNumeric','CSV field not scalar');end
        else
            str{j}=char(value);
            str{j}=strrep(str{j},',',';');
        end
    end
    fprintf(fid,'%s\n',strjoin(str,','));
end
end
function remove_temp(folder)
if exist(folder,'dir'),rmdir(folder,'s');end
end
