function OUT = AF7R49B_REJOIN_DOMAIN_FORENSIC(replanner)
% AF7R49B_REJOIN_DOMAIN_FORENSIC
% Read-only/offline diagnostic for the CURRENT true trigger.
%
% Purpose:
%   Determine whether the large detour is caused by treating the finite
%   reference-validity bound as the actual rejoin time t1.
%
% IMPORTANT:
%   - This script does NOT modify production source or runtime authority.
%   - Each tested t1 is accepted/rejected by the existing continuous-time
%     AF7R36 certificates (speed/rejoin/whole-body), never by sampled safety.
%   - The t1 grid is diagnostic only.  It is NOT a production selection rule.
%
% Requires the AF7R49 package helper files to remain on the MATLAB path.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R49B V2 | TRUE-TRIGGER REJOIN-DOMAIN FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | offlineDiagnostic=1 | sampledSafetyAcceptance=0 | productionModified=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r36_single_core_mex','file')==3, ...
    '[AF7R49B] AF7R36 native core missing.');
assert(exist('af7r49_replay_context','file')==2, ...
    '[AF7R49B] af7r49_replay_context.m missing; keep AF7R49 package on path.');
assert(exist('af7r34_make_core_packet','file')==2, ...
    '[AF7R49B] af7r34_make_core_packet.m missing.');

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid), ...
    '[AF7R49B] first true trigger event unavailable.');
assert(strcmpi(char(E.reference.mode),'NOMINAL'), ...
    '[AF7R49B] first trigger is not NOMINAL authority.');

CTX=af7r49_replay_context(replanner,E);

assert(replanner.nominal_batch_model_cache_valid && ...
    isstruct(replanner.nominal_batch_model_cache) && ...
    isfield(replanner.nominal_batch_model_cache,'certified') && ...
    logical(replanner.nominal_batch_model_cache.certified), ...
    '[AF7R49B] nominal batch model cache unavailable.');

pkt=af7r34_make_core_packet( ...
    replanner,CTX,[],replanner.nominal_batch_model_cache);

% Restore exact trigger-time speed authority.
SC0=E.speed_certificate;
assert(isstruct(SC0)&&isfield(SC0,'certified')&&logical(SC0.certified), ...
    '[AF7R49B] trigger-time speed certificate unavailable.');
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);

% Authority result at the original search-domain end.
R0=af7r36_single_core_mex(1,pkt);
assert(logical(R0.success),'[AF7R49B] baseline AF7R36 trigger replay failed.');

t0=double(R0.t0);
tm=double(R0.tm);
holdEnd=double(R0.hold_end);
tUpper=double(R0.t1);
dt=double(CTX.live.dt);

% The diagnostic grid is exactly the live control spacing.  It does NOT
% define production feasibility or production timing selection.
tFirst=holdEnd+dt;
assert(tFirst<tUpper,'[AF7R49B] no nonempty rejoin-domain interval.');
t1v=tFirst:dt:tUpper;
if t1v(end)<tUpper-8*eps(max(1,abs(tUpper)))
    t1v=[t1v tUpper];
else
    t1v(end)=tUpper;
end

N=numel(t1v);
success=false(1,N);
reason=nan(1,N);
maxCtrl=nan(1,N);
geomMargin=nan(1,N);
VL=nan(1,N);
VQ=nan(1,N);
nativeMs=nan(1,N);
corrEnergy=nan(1,N);
speedPass=nan(1,N);
rejoinPass=nan(1,N);
geomTests=nan(1,N);

fprintf('\n[A. DOMAIN]\n');
fprintf(' trigger / collision / unsafe hold end [s] = %.15g / %.15g / %.15g\n',t0,tm,holdEnd);
fprintf(' current source-validity upper bound [s]   = %.15g\n',tUpper);
fprintf(' diagnostic t1 spacing [s]                 = %.15g\n',dt);
fprintf(' diagnostic t1 count                       = %d\n',N);

% Prime once and audit the exact AF7R36 output contract before the scan.
Rcontract=af7r36_single_core_mex(1,pkt);
requiredFields={ ...
    'success','reason_code','t0','tm','hold_end','t1', ...
    'segment_t0','segment_t1','control_points','max_control_norm_m', ...
    'speed_payload','speed_uav','geometry_min_margin', ...
    'speed_pass_count','rejoin_pass_count','geometry_test_count'};
missing=requiredFields(~isfield(Rcontract,requiredFields));
assert(isempty(missing), ...
    '[AF7R49B V2] AF7R36 result contract missing field(s): %s', ...
    strjoin(missing,', '));

fprintf(' AF7R36 result contract checked             = 1\n');
fprintf(' objective_code requested from AF7R36       = 0\n');

for i=1:N
    P=pkt;
    P.t1=double(t1v(i));
    q=tic;
    R=af7r36_single_core_mex(1,P);
    nativeMs(i)=1e3*toc(q);

    success(i)=logical(R.success);
    reason(i)=double(R.reason_code);
    speedPass(i)=double(R.speed_pass_count);
    rejoinPass(i)=double(R.rejoin_pass_count);
    geomTests(i)=double(R.geometry_test_count);

    if success(i)
        maxCtrl(i)=double(R.max_control_norm_m);
        geomMargin(i)=double(R.geometry_min_margin);
        VL(i)=double(R.speed_payload);
        VQ(i)=double(R.speed_uav);
        corrEnergy(i)=correction_energy_from_raw(R);
    end
end

idx=find(success);
fprintf('\n[B. CERTIFIED t1 RESULTS]\n');
fprintf(' certified diagnostic t1 count            = %d / %d\n',numel(idx),N);

if isempty(idx)
    firstCertified=NaN;
    bestCtrlT=NaN;
    bestEnergyT=NaN;
    fprintf(' no shorter or terminal t1 replay certified; current synthesis requires redesign before shortening.\n');
else
    firstCertified=t1v(idx(1));
    [~,jc]=min(maxCtrl(idx)); bestCtrlIdx=idx(jc); bestCtrlT=t1v(bestCtrlIdx);
    [~,je]=min(corrEnergy(idx)); bestEnergyIdx=idx(je); bestEnergyT=t1v(bestEnergyIdx);

    fprintf(' earliest grid-certified t1 [s]           = %.15g\n',firstCertified);
    fprintf(' current t1 - earliest [s]                = %.15g\n',tUpper-firstCertified);
    fprintf(' min-max-control-norm t1 [s]              = %.15g\n',bestCtrlT);
    fprintf(' old/new max-control norm [m]             = %.15g / %.15g\n', ...
        double(R0.max_control_norm_m),maxCtrl(bestCtrlIdx));
    fprintf(' min-correction-energy t1 [s]             = %.15g\n',bestEnergyT);
    fprintf(' old/new correction energy [m^2 s]        = %.15g / %.15g\n', ...
        correction_energy_from_raw(R0),corrEnergy(bestEnergyIdx));
    fprintf(' geometry margin at earliest t1 [m]       = %.15g\n',geomMargin(idx(1)));
    fprintf(' speed VL/VQ at earliest t1 [m/s]         = %.15g / %.15g\n', ...
        VL(idx(1)),VQ(idx(1)));
end

% Continuous-certificate success pattern only.  Grid itself is diagnostic.
if ~isempty(idx)
    transitions=find(diff(success)~=0);
    fprintf(' success/failure transitions on grid      = %d\n',numel(transitions));
    if numel(transitions)<=12
        for j=1:numel(transitions)
            k=transitions(j);
            fprintf('  transition near t1 [s]                  = %.15g -> %.15g (%d -> %d)\n', ...
                t1v(k),t1v(k+1),success(k),success(k+1));
        end
    end
end

% Failure reason histogram before first certified point.
fprintf('\n[C. FAILURE FORENSIC]\n');
uf=unique(reason(~success & isfinite(reason)));
for r=uf(:).'
    fprintf(' reason %g count                           = %d\n',r,nnz(~success & reason==r));
end
if ~isempty(idx)
    pre=1:idx(1)-1;
else
    pre=1:N;
end
if ~isempty(pre)
    fprintf(' pre-first-cert speed-pass max             = %.0f\n',max(speedPass(pre)));
    fprintf(' pre-first-cert rejoin-pass max            = %.0f\n',max(rejoinPass(pre)));
    fprintf(' pre-first-cert geometry-test max          = %.0f\n',max(geomTests(pre)));
end

% Plot only diagnostic quantities. Safety is always core certificate status.
figure(49); clf;

subplot(2,2,1);
stairs(t1v,double(success),'LineWidth',1.3);
grid on; ylim([-0.05 1.05]);
xlabel('candidate rejoin t_1 [s]'); ylabel('continuous certificate pass');
title('AF7R36 continuous certificate vs t_1');

subplot(2,2,2);
plot(t1v,reason,'LineWidth',1.0); grid on;
xlabel('candidate rejoin t_1 [s]'); ylabel('reason code');
title('Failure reason');

subplot(2,2,3);
plot(t1v,maxCtrl,'LineWidth',1.2); grid on;
xlabel('candidate rejoin t_1 [s]'); ylabel('max correction control norm [m]');
title('Certified detour upper bound');

subplot(2,2,4);
plot(t1v,nativeMs,'LineWidth',1.0); grid on;
xlabel('candidate rejoin t_1 [s]'); ylabel('native replay [ms]');
title('Offline per-t_1 native runtime');

sgtitle('AF7R49B — diagnostic only; grid is not production authority');
drawnow;

shorterExists=~isempty(idx) && firstCertified<tUpper-8*eps(max(1,abs(tUpper)));
fprintf('\n[D. ROOT-CAUSE CLASSIFICATION]\n');
fprintf(' shorter continuously-certified t1 exists = %d\n',shorterExists);
if shorterExists
    fprintf([' classification                            = SOURCE VALIDITY END IS BEING USED AS ACTUAL REJOIN;\n' ...
             '                                             shorter certified rejoin candidates exist.\n']);
else
    fprintf([' classification                            = NO SHORTER CERTIFIED GRID POINT FOUND;\n' ...
             '                                             objective/tube/synthesis constraints require deeper audit.\n']);
end
fprintf(' production source modified                = 0\n');
fprintf(' sampled safety acceptance                 = 0\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R49B_REJOIN_DOMAIN_FORENSIC_V2';
OUT.t1=t1v;
OUT.success=success;
OUT.reason=reason;
OUT.max_control_norm_m=maxCtrl;
OUT.geometry_margin_m=geomMargin;
OUT.payload_speed_m_s=VL;
OUT.uav_speed_m_s=VQ;
OUT.native_ms=nativeMs;
OUT.correction_energy_m2s=corrEnergy;
OUT.speed_pass_count=speedPass;
OUT.rejoin_pass_count=rejoinPass;
OUT.geometry_test_count=geomTests;
OUT.earliest_grid_certified_t1=firstCertified;
OUT.shorter_certified_t1_exists=shorterExists;
OUT.current_t1=tUpper;
OUT.hold_end=holdEnd;
OUT.production_modified=false;

assignin('base','AF7R49B_REJOIN_DOMAIN_FORENSIC_OUT',OUT);
end


function E=correction_energy_from_raw(R)
E=0;
s0=double(R.segment_t0(:).');
s1=double(R.segment_t1(:).');
C=R.control_points;
for j=1:numel(C)
    cp=double(C{j});
    T=s1(j)-s0(j);
    n=size(cp,2)-1;
    q=0;
    for a=0:n
        for b=0:n
            q=q+dot(cp(:,a+1),cp(:,b+1))* ...
                nchoosek(n,a)*nchoosek(n,b)/ ...
                ((2*n+1)*nchoosek(2*n,a+b));
        end
    end
    E=E+T*q;
end
end
