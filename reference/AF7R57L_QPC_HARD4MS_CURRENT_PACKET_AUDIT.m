function OUT = AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT()
% AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT
% =========================================================================
% High-resolution hard-deadline audit of the AF7R57K corrected native core.
%
% Acceptance metric:
%   RAW QPC wall time from a stamp immediately before the corrected MEX call
%   to a stamp immediately after it.
%
% No overhead subtraction is used for PASS.
% p50/p95 are diagnostic only.
% HARD PASS requires EVERY measured trial < 4 ms.
%
% Thread/process cycle deltas are work-only diagnostics, matching the
% established AF7R25 timing semantics. They are not converted to wall time.
%
% IMPORTANT SCOPE:
% This closes a hard timing gate for the CURRENT live packet only.
% The current obstacle/component counts are reported, never hardcoded.
% It does NOT prove <=4 ms for arbitrary unbounded obstacle counts.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57L | QPC HARD-4ms CURRENT-LIVE-PACKET TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] QPCwall=1 | maxNotPercentile=1 | threadProcessCyclesWorkOnly=1 | ' ...
         'warmupAllowed=1 | currentPacketOnly=1 | productionModified=0\n']);

OUT=struct();
OUT.schema='AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_V1';
OUT.closed=false;
OUT.current_packet_hard_4ms=false;
OUT.production_general_hard_4ms=false;
OUT.production_modified=false;

%% A. Required theorem authority
fprintf('\n[A. REQUIRED THEOREM AUTHORITY]\n');

K=get_authority('AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT', ...
                'AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT.mat');
assert(isstruct(K)&&isfield(K,'closed')&&logical(K.closed), ...
    '[AF7R57L] AF7R57K corrected native theorem not closed.');

assert(isfield(K,'tU')&&isfinite(double(K.tU)));
tU=double(K.tU);

fprintf(' AF7R57K corrected theorem closed        = 1\n');
fprintf(' fixed source-derived tU                 = %.17g\n',tU);
fprintf(' production source modified              = 0\n');

%% B. Reconstruct exact current packet / nominal authority
fprintf('\n[B. CURRENT PACKET AUTHORITY]\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[AF7R57L] replanner_instance missing.');
replanner=evalin('base','replanner_instance');
assert(isa(replanner,'REPLANNING_SYSTEM'));

C=get_authority('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
                'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');
assert(logical(C.closed));

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner); %#ok<ASGLU>
assert(isstruct(E)&&logical(E.valid)&&isstruct(M)&&logical(M.certified));

rt1=record_for_candidate(C.trace_records,1);
pkt=rt1.pkt;
assert(double(rt1.tU)==tU,'[AF7R57L] tU authority changed.');

F=replanner.nominal_flat_derivative_fn(1:7);
S=cellfun(@func2str,F,'UniformOutput',false);
sig=sha256_text(strjoin(S,char(10)));
assert(strcmp(sig,char(pkt.nominal_signature)));
assert(strcmp(sig,char(M.nominal_signature)));

obsCount=size(pkt.centers,2);
componentCount=numel(record_for_candidate(C.kkt_records,1).U.component_start);

fprintf(' authority source                         = %s\n',char(string(authoritySource)));
fprintf(' nominal signature equal                 = 1\n');
fprintf(' runtime obstacle count                  = %d\n',obsCount);
fprintf(' runtime component count                 = %d\n',componentCount);
fprintf(' obstacle count hardcoded                = 0\n');

%% C. Corrected MEX / stamp MEX
fprintf('\n[C. TIMING PRIMITIVES]\n');

assert(exist('af7r57k_corrected_core_mex','file')==3, ...
    '[AF7R57L] corrected native shadow MEX missing. Run R57K first.');

root=fileparts(mfilename('fullpath'));
cpp=fullfile(root,'af7r57l_qpc_stamp_mex.cpp');
assert(isfile(cpp));
name='af7r57l_qpc_stamp_mex';

needBuild=exist(name,'file')~=3;
if ~needBuild
    p=which(name);
    needBuild=isempty(p)||dir(p).datenum<dir(cpp).datenum;
end
if needBuild
    mex('-R2018a','-outdir',root,cpp);
    rehash;
end
assert(exist(name,'file')==3);

s0=af7r57l_qpc_stamp_mex();
assert(logical(s0.qpc_available)&&isfinite(double(s0.qpc_frequency))&&double(s0.qpc_frequency)>0);
fprintf(' QPC available                           = 1\n');
fprintf(' QPC frequency [Hz]                      = %.0f\n',double(s0.qpc_frequency));
fprintf(' thread cycle counter available          = %d\n',logical(s0.thread_cycles_available));
fprintf(' process cycle counter available         = %d\n',logical(s0.process_cycles_available));

%% D. Re-prime corrected MEX if needed
fprintf('\n[D. PRIME / WARMUP]\n');

[expr,signature]=nominal_ast(replanner,M);
ok=af7r57k_corrected_core_mex(0,expr,signature);
assert(logical(ok));

% Verify one untimed call before warmup.
R0=af7r57k_corrected_core_mex(4,pkt,tU,tU);
assert(logical(R0.success)&&logical(R0.geometry_pass)&&double(R0.rejoin_pass_count)>=1);

% Warmup is explicitly permitted before Flight.  None of these samples enter
% the hard timing population.
warmN=50;
for k=1:warmN
    Rw=af7r57k_corrected_core_mex(4,pkt,tU,tU); %#ok<NASGU>
end
fprintf(' explicit pre-Flight warmup calls         = %d\n',warmN);

%% E. QPC stamp-pair overhead authority
fprintf('\n[E. QPC STAMP-PAIR OVERHEAD]\n');

Nstamp=1000;
stampMs=zeros(Nstamp,1);
for k=1:Nstamp
    a=af7r57l_qpc_stamp_mex();
    b=af7r57l_qpc_stamp_mex();
    stampMs(k)=1000*(double(b.qpc_ticks)-double(a.qpc_ticks))/double(a.qpc_frequency);
end
assert(all(isfinite(stampMs))&&all(stampMs>=0));

fprintf(' stamp pairs                              = %d\n',Nstamp);
fprintf(' stamp min/median/p95/max [ms]            = %.6f / %.6f / %.6f / %.6f\n', ...
    min(stampMs),median(stampMs),prctile(stampMs,95),max(stampMs));
fprintf(' overhead subtracted for acceptance       = 0\n');

%% F. First Flight-equivalent hard sample
fprintf('\n[F. FIRST POST-WARMUP FLIGHT-EQUIVALENT SAMPLE]\n');

[a,Rfirst,b]=timed_call(pkt,tU);
firstMs=qpc_ms(a,b);
[firstTC,firstPC]=cycle_delta(a,b);

firstValid=certificate_still_valid(Rfirst,K);
fprintf(' QPC wall [ms]                           = %.6f\n',firstMs);
fprintf(' thread cycle delta                      = %.0f\n',firstTC);
fprintf(' process cycle delta                     = %.0f\n',firstPC);
fprintf(' certificate output still valid          = %d\n',firstValid);
fprintf(' first post-warmup < 4 ms                = %d\n',firstMs<4);
assert(firstValid,'[AF7R57L] corrected certificate changed during timing audit.');

%% G. Hard timing population
fprintf('\n[G. HARD TIMING POPULATION]\n');

N=2000;
wall=zeros(N,1);
threadCycles=nan(N,1);
processCycles=nan(N,1);
valid=false(N,1);

for k=1:N
    [a,R,b]=timed_call(pkt,tU);
    wall(k)=qpc_ms(a,b);
    [threadCycles(k),processCycles(k)]=cycle_delta(a,b);
    valid(k)=certificate_still_valid(R,K);
end

assert(all(valid),'[AF7R57L] certificate output changed in timing population.');
assert(all(isfinite(wall))&&all(wall>=0));

hardLimit=4.0;
hardPass=firstMs<hardLimit && all(wall<hardLimit);

[worst,worstIdx]=max(wall);

fprintf(' hard trials                              = %d\n',N);
fprintf(' wall min / median / p95 / max [ms]       = %.6f / %.6f / %.6f / %.6f\n', ...
    min(wall),median(wall),prctile(wall,95),worst);
fprintf(' worst trial index                       = %d\n',worstIdx);
fprintf(' deadline [ms]                           = %.6f\n',hardLimit);
fprintf(' minimum hard slack [ms]                 = %.6f\n',hardLimit-max(firstMs,worst));
fprintf(' all certificate outputs valid           = %d\n',all(valid));
fprintf(' EVERY measured trial < 4 ms             = %d\n',hardPass);

if all(isfinite(threadCycles))
    fprintf(' thread cycles min/median/max            = %.0f / %.0f / %.0f\n', ...
        min(threadCycles),median(threadCycles),max(threadCycles));
else
    fprintf(' thread cycles                           = UNAVAILABLE\n');
end
if all(isfinite(processCycles))
    fprintf(' process cycles min/median/max           = %.0f / %.0f / %.0f\n', ...
        min(processCycles),median(processCycles),max(processCycles));
else
    fprintf(' process cycles                          = UNAVAILABLE\n');
end

%% H. Spike policy / scope
fprintf('\n[H. SPIKE POLICY / SCOPE]\n');

spikeCount=nnz(wall>=hardLimit) + (firstMs>=hardLimit);
fprintf(' >=4 ms spike count                      = %d\n',spikeCount);
fprintf(' percentile relaxation allowed           = 0\n');
fprintf(' first post-warmup included in hard gate  = 1\n');
fprintf(' current obstacle count                  = %d\n',obsCount);
fprintf(' current component count                 = %d\n',componentCount);
fprintf(' arbitrary obstacle-count WCET proved     = 0\n');
fprintf(' arbitrary component-count WCET proved    = 0\n');

%% I. Active-set serializer interpretation
fprintf('\n[I. R57K ACTIVE-SET DISPLAY INTERPRETATION]\n');

activeComparable=false;
activeEqual=false;
if isfield(K,'original')&&isfield(K,'corrected') && ...
        isfield(K.original,'qp_active_set')&&isfield(K.corrected,'qp_active_set')
    activeComparable=true;
    activeEqual=isequaln(double(K.original.qp_active_set),double(K.corrected.qp_active_set));
end

fprintf(' active-set serializer comparable         = %d\n',activeComparable);
if activeComparable
    fprintf(' active-set identity                     = %d\n',activeEqual);
else
    fprintf(' active-set identity                     = NOT_COMPARABLE_IN_SERIALIZER\n');
end
fprintf(' candidate synthesis identity from R57K   = %d\n',logical(K.identity.all));
fprintf(' control-point max |diff|                 = %.17g\n', ...
    double(K.identity.control_point_max_abs_error));

%% J. Formal gate
fprintf('\n[J. FORMAL CURRENT-PACKET HARD GATE]\n');

currentClosed=logical(K.closed)&&hardPass&&all(valid);

fprintf(' corrected mathematical theorem closed    = %d\n',logical(K.closed));
fprintf(' raw-QPC EVERY-trial hard timing closed   = %d\n',hardPass);
fprintf(' current-live-packet <=4 ms gate          = %d\n',currentClosed);
fprintf(' production-general <=4 ms gate           = 0\n');
fprintf(' production source modified               = 0\n');
fprintf(' production t1 selected                   = 0\n');

if currentClosed
    next='BUILD_OBSTACLE_COMPONENT_COMPLEXITY_ENVELOPE__THEN_SHADOW_PRODUCTION_INTEGRATION';
else
    next='PROFILE_QPC_WORST_TRIAL_AND_REMOVE_RUNTIME_SPIKES';
end
fprintf(' next action                              = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=currentClosed;
OUT.current_packet_hard_4ms=currentClosed;
OUT.production_general_hard_4ms=false;
OUT.tU=tU;
OUT.obstacle_count=obsCount;
OUT.component_count=componentCount;
OUT.first_post_warmup_ms=firstMs;
OUT.wall_ms=wall;
OUT.thread_cycles=threadCycles;
OUT.process_cycles=processCycles;
OUT.stamp_pair_ms=stampMs;
OUT.worst_ms=worst;
OUT.worst_index=worstIdx;
OUT.hard_slack_ms=hardLimit-max(firstMs,worst);
OUT.spike_count=spikeCount;
OUT.certificate_valid=valid;
OUT.production_modified=false;
OUT.next_action=next;

assignin('base','AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_OUT.mat'),'OUT','-v7.3');
end


function [a,R,b]=timed_call(pkt,tU)
a=af7r57l_qpc_stamp_mex();
R=af7r57k_corrected_core_mex(4,pkt,tU,tU);
b=af7r57l_qpc_stamp_mex();
end


function ms=qpc_ms(a,b)
assert(logical(a.qpc_available)&&logical(b.qpc_available));
f=double(a.qpc_frequency);
assert(f>0&&isfinite(f));
ms=1000*(double(b.qpc_ticks)-double(a.qpc_ticks))/f;
end


function [tc,pc]=cycle_delta(a,b)
tc=NaN;pc=NaN;
if logical(a.thread_cycles_available)&&logical(b.thread_cycles_available)
    tc=double(b.thread_cycles)-double(a.thread_cycles);
end
if logical(a.process_cycles_available)&&logical(b.process_cycles_available)
    pc=double(b.process_cycles)-double(a.process_cycles);
end
end


function tf=certificate_still_valid(R,K)
tf=logical(R.success) && logical(R.geometry_pass) && ...
   double(R.rejoin_pass_count)>=1 && ...
   isfinite(double(R.speed_uav)) && isfinite(double(R.speed_sigma)) && ...
   double(R.speed_sigma)>0;

if tf && isfield(K,'corrected')
    % Deterministic theorem-relevant outputs must remain bitwise equal to the
    % already closed R57K corrected authority for this fixed packet.
    A=K.corrected;
    fields={'reason_code','speed_uav','speed_payload','speed_accel', ...
            'speed_jerk','speed_sigma','speed_qdot', ...
            'geometry_min_payload_margin','geometry_min_uav_margin', ...
            'geometry_min_cable_margin','geometry_min_margin'};
    for i=1:numel(fields)
        f=fields{i};
        if isfield(A,f)&&isfield(R,f)
            tf=tf&&isequaln(A.(f),R.(f));
        end
    end
end
end


function [expr,signature]=nominal_ast(replanner,M)
F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(f)isa(f,'function_handle'),F)));
S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));
assert(strcmp(signature,char(M.nominal_signature)));

expr=cell(21,1);kk=0;
for r=1:7
    ss=strtrim(S{r});
    tok=regexp(ss,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2);
    body=strtrim(char(tok{2}));
    body=strrep(body,'.^','^');body=strrep(body,'.*','*');body=strrep(body,'./','/');
    z=str2sym(body);z=z(:);assert(numel(z)==3);
    t=sym('t','real');sv0=symvar(z);assert(numel(sv0)<=1);
    if ~isempty(sv0),z=subs(z,sv0(1),t);end
    sv=symvar(z);for j=1:numel(sv),assert(strcmp(char(sv(j)),'t'));end
    for j=1:3,kk=kk+1;expr{kk}=char(z(j));end
end
end


function r=record_for_candidate(R,c)
ids=cellfun(@(q)double(q.c),R);
k=find(ids==c,1,'last');assert(~isempty(k));r=R{k};
end


function A=get_authority(varName,fileName)
if evalin('base',sprintf('exist(''%s'',''var'')==1',varName))
    A=evalin('base',varName);return;
end
cand={which(fileName),fullfile(pwd,fileName),fullfile(fileparts(mfilename('fullpath')),fileName)};
u=getenv('USERPROFILE');
if ~isempty(u),cand{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference',fileName);end %#ok<AGROW>
p='';
for i=1:numel(cand)
    if ~isempty(cand{i})&&isfile(cand{i}),p=cand{i};break;end
end
assert(~isempty(p),'[AF7R57L] authority unavailable: %s',fileName);
q=load(p,'OUT');assert(isfield(q,'OUT'));A=q.OUT;
end


function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
