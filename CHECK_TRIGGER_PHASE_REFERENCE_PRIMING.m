function R = CHECK_TRIGGER_PHASE_REFERENCE_PRIMING(varargin)
% A standalone synthetic unit test (no Flight agent, plant or sensor).
% With a REAL completed Flight, pass replanner_instance for read-only status.
% Does not touch any Flight references, obstacle history, EKF or HLC.
if nargin>0
    replanner=varargin{1};
    assert(isobject(replanner)&&ismethod(replanner,'get_status'), ...
        'CHECK_TRIGGER_PHASE_REFERENCE_PRIMING:Input', ...
        'Pass the actual replanner_instance after Flight.');
    Q=replanner.get_status();
    C=Q.trigger_reference_cache;
    R=struct('status','READ_ONLY_PHASE_PRIMING_FLIGHT_STATUS', ...
        'prime_evaluations',C.takeoff_prime_evaluations, ...
        'prime_accepted',C.takeoff_prime_accepted, ...
        'prime_discarded',C.takeoff_prime_discarded, ...
        'prime_status',C.takeoff_prime_status, ...
        'prediction_safety_certified',false);
    write_result(R);
    return
end
% Isolated artificial nominal. This is a TEST ONLY, not a Flight warmup.
assert(exist('TIME_VARYING_REFERENCE','class')==8 && ...
    exist('STATE_CLASS','class')==8, ...
    'CHECK_TRIGGER_PHASE_REFERENCE_PRIMING:Dependencies', ...
    'The original reference and STATE_CLASS must be on the MATLAB path.');
state=STATE_CLASS(struct('state_list',["p","q","v"], ...
    'num_list',[3,3,3]));
state.set_state('p',[0;0;0],'q',[0;0;0],'v',[0;0;0]);
self=struct('estimator',struct('result',struct('state',state)));
nominal=TIME_VARYING_REFERENCE(self,{"gen_ref_line", ...
    {"p0",[0,0,3],"velocity",1.5,"direction",[0,1,0]},6});
% Do NOT set up a fake Phase trajectory in a real agent. This is the
% independent source object created above, owned only by this test.
dt=0.025; H=4; L=2;
epoch=0;
while epoch<5
    epoch=epoch+dt;
end
nominal.t=epoch;
c=TRIGGER_REFERENCE_WINDOW_CACHE();
a=c.audit_immutable_source(nominal.func);
assert(a.eligible, ...
    'CHECK_TRIGGER_PHASE_REFERENCE_PRIMING:Purity', ...
    'The unit-test source was rejected by the preflight purity audit.');
for k=1:160
    c.prime_takeoff_step(nominal,epoch,epoch,dt,H,L);
end
grid=(0:floor(H/dt))*dt;
grid=grid(grid<H);grid=[grid,H];
times=epoch+grid(1:end-1);
fn=@(t) adjusted(nominal,t,L);
now=fn(times(1));
[prepared,~]=c.evaluate(nominal,times,L,fn,now, ...
    'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL');
control=TRIGGER_REFERENCE_WINDOW_CACHE();
control.audit_immutable_source(nominal.func);
[original,~]=control.evaluate(nominal,times,L,fn,now, ...
    'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL');
q=c.get_status();
assert(isequal(prepared,original) && ...
    isequal(typecast(prepared(:),'uint64'),typecast(original(:),'uint64')) && ...
    q.takeoff_prime_accepted==160 && q.takeoff_prime_evaluations==160, ...
    'CHECK_TRIGGER_PHASE_REFERENCE_PRIMING:Equality', ...
    'Prepared source columns differ from original exact-time evaluation.');
% Deliberately wrong anticipated Flight epoch must discard every column.
c2=TRIGGER_REFERENCE_WINDOW_CACHE();
c2.audit_immutable_source(nominal.func);
for k=1:2
    c2.prime_takeoff_step(nominal,epoch+dt,epoch,dt,H,L);
end
[wrong_epoch,~]=c2.evaluate(nominal,times,L,fn,now, ...
    'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL');
q2=c2.get_status();
assert(isequal(original,wrong_epoch) && ...
    q2.takeoff_prime_accepted==0 && q2.takeoff_prime_discarded==1, ...
    'CHECK_TRIGGER_PHASE_REFERENCE_PRIMING:Discard', ...
    'An epoch mismatch must never consume a prepared reference.');
% Sim's original initialization may have ALREADY set nominal.t=0
% during its one-time phase sweep. That is a different (valid) contract:
% prepare first-flight absolute times, but evaluate them relative to 0.
nominal.t=0;
c3=TRIGGER_REFERENCE_WINDOW_CACHE();
c3.audit_immutable_source(nominal.func);
for k=1:160
    c3.prime_takeoff_step(nominal,epoch,0,dt,H,L);
end
now_zero=fn(times(1));
[primed_zero,~]=c3.evaluate(nominal,times,L,fn,now_zero, ...
    'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL');
c4=TRIGGER_REFERENCE_WINDOW_CACHE();
c4.audit_immutable_source(nominal.func);
[uncached_zero,~]=c4.evaluate(nominal,times,L,fn,now_zero, ...
    'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL');
q3=c3.get_status();
assert(isequal(typecast(primed_zero(:),'uint64'), ...
    typecast(uncached_zero(:),'uint64')) && ...
    q3.takeoff_prime_accepted==160, ...
    'CHECK_TRIGGER_PHASE_REFERENCE_PRIMING:PreinitializedEpoch', ...
    'A source whose epoch is fixed before Takeoff must stay bitwise exact.');
R=struct('status','PASS_SYNTHETIC_BITWISE_PHASE_REFERENCE_ONLY', ...
    'first_epoch',epoch,'checked_columns',numel(grid)-1, ...
    'prime_evaluations',q.takeoff_prime_evaluations, ...
    'prime_accepted',q.takeoff_prime_accepted, ...
    'wrong_epoch_discarded',q2.takeoff_prime_discarded, ...
    'preinitialized_epoch_accepted',q3.takeoff_prime_accepted, ...
    'bitwise_equal',isequal(typecast(prepared(:),'uint64'), ...
        typecast(original(:),'uint64')), ...
    'prediction_safety_certified',false);
write_result(R);
end
function y=adjusted(nominal,t,L)
y=double(nominal.func(double(t)-double(nominal.t)));
y(3)=y(3)-double(L);
end
function write_result(R)
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_PHASE_REFERENCE_PRIMING_RESULT.mat'),'R');
fid=fopen(fullfile(folder,'TRIGGER_PHASE_REFERENCE_PRIMING_RESULT.txt'),'wt');
if fid>=0
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    f=fieldnames(R);
    for k=1:numel(f)
        val=R.(f{k});
        if isnumeric(val) && isscalar(val)
            fprintf(fid,'%s=%.17g\n',f{k},double(val));
        elseif ischar(val)
            fprintf(fid,'%s=%s\n',f{k},val);
        end
    end
end
end
