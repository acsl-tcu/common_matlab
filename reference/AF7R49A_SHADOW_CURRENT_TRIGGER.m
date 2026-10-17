function OUT = AF7R49A_SHADOW_CURRENT_TRIGGER(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R49A | CURRENT TRUE-TRIGGER MIN-CORRECTION-ENERGY SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionSourceUnmodified=1 | continuousSafetyAcceptanceOnly=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
src=which('REPLANNING_SYSTEM');
expectedSource='890876116b4782725d8c337e53923a14892a9b5f4f00a0e7de030b9d4433c446';
hSource=sha256_file(src);
assert(strcmp(hSource,expectedSource), ...
    '[AF7R49A] current production source is not exact AF7R46.');
assert(exist('af7r36_single_core_mex','file')==3,'[AF7R49A] AF7R36 MEX missing.');
assert(exist('af7r49_single_core_mex','file')==3,'[AF7R49A] AF7R49 MEX missing; run build first.');

af7r49_prime_core_reference_runtime(replanner);
% Re-prime old core too, so the comparison is independent of previous MEX state.
if exist('AF7R36_PRIME_CANONICAL_SINGLE_CORE_REFERENCE','file')==2
    evalc('AF7R36_PRIME_CANONICAL_SINGLE_CORE_REFERENCE(replanner);');
end

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid), ...
    '[AF7R49A] no captured real Flight trigger event in current AF7R46 run.');
assert(strcmpi(char(E.reference.mode),'NOMINAL'), ...
    '[AF7R49A] first trigger must be NOMINAL-authority for this comparison.');
CTX=af7r49_replay_context(replanner,E);

assert(replanner.nominal_batch_model_cache_valid && ...
    isstruct(replanner.nominal_batch_model_cache) && ...
    isfield(replanner.nominal_batch_model_cache,'certified') && ...
    logical(replanner.nominal_batch_model_cache.certified), ...
    '[AF7R49A] nominal batch model cache unavailable.');
model=replanner.nominal_batch_model_cache;
pkt=af7r34_make_core_packet(replanner,CTX,[],model);

% af7r34_make_core_packet() normally reads the CURRENT active speed
% certificate from replanner.  The object is already REPAIRED after the
% completed run, whereas this shadow deliberately replays the FIRST NOMINAL
% trigger.  Restore the exact trigger-time speed authority captured in E so
% OLD and NEW receive the same packet that the real request had.
SC0=E.speed_certificate;
assert(isstruct(SC0)&&isfield(SC0,'certified')&&logical(SC0.certified), ...
    '[AF7R49A] trigger-time speed certificate unavailable.');
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);

OLD=af7r36_single_core_mex(1,pkt);
NEW=af7r49_single_core_mex(1,pkt);

sameEpisode = double(OLD.episode_count)==double(NEW.episode_count) && ...
    double(OLD.component_count)==double(NEW.component_count) && ...
    isequal(double(OLD.episode_obstacle_id),double(NEW.episode_obstacle_id)) && ...
    maxdiff(double(OLD.episode_start),double(NEW.episode_start))==0 && ...
    maxdiff(double(OLD.episode_end),double(NEW.episode_end))==0 && ...
    maxdiff(double(OLD.component_start),double(NEW.component_start))==0 && ...
    maxdiff(double(OLD.component_end),double(NEW.component_end))==0;
sameTiming = isequal(double([OLD.t0 OLD.tm OLD.t1]),double([NEW.t0 NEW.tm NEW.t1]));
newSafety = logical(NEW.success) && logical(NEW.geometry_pass) && ...
    double(NEW.reason_code)==0 && double(NEW.geometry_reason)==0 && ...
    isfinite(double(NEW.speed_payload)) && isfinite(double(NEW.speed_uav)) && ...
    double(NEW.geometry_min_margin)>0;

% Performance diagnostics only; never safety acceptance.
dt=double(CTX.live.dt);
tg=double(NEW.t0):dt:double(NEW.t1);
if tg(end)<double(NEW.t1), tg=[tg double(NEW.t1)]; end
Y=af7r49_single_core_mex(2,pkt,tg);
nom=double(Y(1:3,:));
dOld=eval_correction(OLD,tg); dNew=eval_correction(NEW,tg);
peakOld=max(vecnorm(dOld,2,1)); peakNew=max(vecnorm(dNew,2,1));
pathNom=sum(vecnorm(diff(nom,1,2),2,1));
pathOld=sum(vecnorm(diff(nom+dOld,1,2),2,1));
pathNew=sum(vecnorm(diff(nom+dNew,1,2),2,1));

% Offline timing only.
Nbench=200; to=zeros(1,Nbench); tn=zeros(1,Nbench);
for k=1:Nbench
    q=tic; af7r36_single_core_mex(1,pkt); to(k)=1e3*toc(q);
    q=tic; af7r49_single_core_mex(1,pkt); tn(k)=1e3*toc(q);
end

energyOld=correction_energy_from_raw(OLD);
energyNew=double(NEW.correction_energy_m2s);
% Performance gate uses only exact/continuous quantities: exact integrated
% correction energy and the Bezier convex-hull peak upper bound.  The dt-grid
% peak/path values above remain diagnostics only and never gate installation.
ctrlOld=double(OLD.max_control_norm_m);
ctrlNew=double(NEW.max_control_norm_m);
improved = logical(NEW.success) && double(NEW.objective_code)==1 && ...
    energyNew < energyOld && ctrlNew < ctrlOld;
closed = logical(OLD.success) && newSafety && sameEpisode && sameTiming && improved;

fprintf(' true trigger t0 / collision / t1 [s]     = %.15g / %.15g / %.15g\n',double(NEW.t0),double(NEW.tm),double(NEW.t1));
fprintf(' old/new success                          = %d / %d\n',logical(OLD.success),logical(NEW.success));
fprintf(' episode/timing unchanged                 = %d / %d\n',sameEpisode,sameTiming);
fprintf(' new objective code                       = %.0f (1=min correction energy)\n',double(NEW.objective_code));
fprintf(' old/new correction energy [m^2 s]        = %.15g / %.15g\n',energyOld,energyNew);
fprintf(' old/new diagnostic peak correction [m]   = %.15g / %.15g\n',peakOld,peakNew);
fprintf(' nominal/old/new path length [m]          = %.15g / %.15g / %.15g\n',pathNom,pathOld,pathNew);
fprintf(' old/new max control norm [m]             = %.15g / %.15g\n',double(OLD.max_control_norm_m),double(NEW.max_control_norm_m));
fprintf(' old/new min geometry margin [m]          = %.15g / %.15g\n',double(OLD.geometry_min_margin),double(NEW.geometry_min_margin));
fprintf(' old/new native med [ms]                  = %.6g / %.6g\n',median(to),median(tn));
fprintf(' new continuous safety closed             = %d\n',newSafety);
fprintf(' shadow improvement closed                = %d\n',improved);
fprintf(' AF7R49 shadow closed                     = %d\n',closed);
fprintf('====================================================================================================\n');

mexPath=which('af7r49_single_core_mex'); hMex=sha256_file(mexPath);
token=struct('schema','AF7R49_SHADOW_TOKEN_V1','closed',closed, ...
    'source_hash',hSource,'mex_hash',hMex,'event_time',double(E.event_time), ...
    'objective_code',double(NEW.objective_code),'new_safety',newSafety, ...
    'peak_old_m',peakOld,'peak_new_m',peakNew, ...
    'energy_old',energyOld,'energy_new',energyNew);
save(fullfile(fileparts(src),'AF7R49_SHADOW_TOKEN.mat'),'token');

OUT=struct('schema','AF7R49A_SHADOW_CURRENT_TRIGGER_V1','closed',closed, ...
    'old',OLD,'new',NEW,'same_episode',sameEpisode,'same_timing',sameTiming, ...
    'new_safety',newSafety,'improved',improved,'peak_old_m',peakOld, ...
    'peak_new_m',peakNew,'path_nominal_m',pathNom,'path_old_m',pathOld, ...
    'path_new_m',pathNew,'old_median_ms',median(to),'new_median_ms',median(tn));
assignin('base','AF7R49_SHADOW_OUT',OUT);
end

function d=eval_correction(R,tg)
d=zeros(3,numel(tg));
s0=double(R.segment_t0(:).'); s1=double(R.segment_t1(:).'); C=R.control_points;
for k=1:numel(tg)
    t=double(tg(k));
    if t<=s0(1) || t>=s1(end), continue; end
    j=find(t<=s1,1,'first'); if isempty(j)||t<s0(j), continue; end
    u=(t-s0(j))/(s1(j)-s0(j)); u=max(0,min(1,u));
    cp=double(C{j}); d(:,k)=decasteljau(cp,u);
end
end
function p=decasteljau(cp,u)
q=cp; n=size(q,2)-1;
for r=1:n, q(:,1:n-r+1)=(1-u).*q(:,1:n-r+1)+u.*q(:,2:n-r+2); end
p=q(:,1);
end
function d=maxdiff(a,b)
a=a(:);b=b(:);if isempty(a)&&isempty(b),d=0;elseif numel(a)~=numel(b),d=Inf;else,d=max(abs(a-b));end
end

function E=correction_energy_from_raw(R)
E=0;
s0=double(R.segment_t0(:).');s1=double(R.segment_t1(:).');C=R.control_points;
for k=1:numel(C)
    cp=double(C{k});T=s1(k)-s0(k);n=15;
    q=0;
    for a=0:n
        for b=0:n
            q=q+dot(cp(:,a+1),cp(:,b+1))*bernstein_gram(n,a,b);
        end
    end
    E=E+T*q;
end
end
function g=bernstein_gram(n,i,j)
g=nchoosek(n,i)*nchoosek(n,j)/((2*n+1)*nchoosek(2*n,i+j));
end
function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
