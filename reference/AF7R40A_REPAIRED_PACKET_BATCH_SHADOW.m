function OUT = AF7R40A_REPAIRED_PACKET_BATCH_SHADOW(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R40A | REPAIRED FUTURE-PACKET BATCH EQUIVALENCE + TIMING SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | implementationEquivalence=1 | safetyAcceptance=0 | sourceUnmodified=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(strcmpi(char(replanner.active_reference_mode),'REPAIRED'), ...
    '[AF7R40A] active reference must be REPAIRED. Run this on the just-completed AF7R39 flight workspace.');

R=replanner.active_repaired_reference;
assert(isstruct(R)&&isfield(R,'eval_fn')&&isa(R.eval_fn,'function_handle'));
assert(isfield(R,'flat_eval_fn')&&isa(R.flat_eval_fn,'function_handle'));
assert(isfield(R,'batch_flat_model')&&isstruct(R.batch_flat_model)&& ...
    isfield(R.batch_flat_model,'certified')&&logical(R.batch_flat_model.certified), ...
    '[AF7R40A] committed repaired reference lacks the certified AF7R33 batch model.');
assert(exist('af7r33_flat_jet_batch_model','file')==2, ...
    '[AF7R40A] af7r33_flat_jet_batch_model.m is not on path.');

dt=double(replanner.prediction_dt);
H=double(replanner.horizon_s);
N=round(H/dt)+1;

C=R.batch_flat_model.corrections;
assert(~isempty(C),'[AF7R40A] no committed correction exists.');
t0=min(arrayfun(@(x)double(x.t0),C));
t1=max(arrayfun(@(x)double(x.t1),C));

% Full live-dt grid over the repaired interval plus one horizon after rejoin.
% This is implementation-equivalence evidence only, never safety acceptance.
tg=t0:dt:(t1+H);
if tg(end)<t1+H-0.25*dt
    tg=[tg,t1+H]; %#ok<AGROW>
end

[Y,v]=af7r33_flat_jet_batch_model(replanner,tg,R.batch_flat_model);
assert(isequal(size(Y),[21 numel(tg)])&&all(v));

Pold=zeros(3,numel(tg));
Qold=zeros(3,numel(tg));
for k=1:numel(tg)
    xd=double(R.eval_fn(tg(k))); xd=xd(:);
    xf=double(R.flat_eval_fn(tg(k))); xf=xf(:);
    assert(numel(xd)>=3&&numel(xf)>=11);
    Pold(:,k)=xd(1:3);
    sf=xf(9:11)+[0;0;double(replanner.gravity)];
    ns=norm(sf);
    assert(isfinite(ns)&&ns>0);
    Qold(:,k)=-sf/ns;
end

Pnew=double(Y(1:3,:));
Anew=double(Y(7:9,:));
SF=Anew+[0;0;double(replanner.gravity)];
NS=sqrt(sum(SF.^2,1));
assert(all(isfinite(NS))&&all(NS>0));
Qnew=-SF./NS;

dp=max(abs(Pold-Pnew),[],'all');
dq=max(abs(Qold-Qnew),[],'all');
scale=max([1,max(abs(Pold),[],'all'),max(abs(Pnew),[],'all')]);
tol=8192*eps(scale);
eqClosed=(dp<=tol)&&(dq<=tol);

% Benchmark one complete trigger horizon. Prime both paths first.
tbench=max(t0,double(replanner.last_replan_commit_time));
tq=tbench+(0:N-1)*dt;

scalar_packet();
batch_packet();
nrep=21;
ts=zeros(1,nrep); tb=zeros(1,nrep);
for r=1:nrep
    z=tic; scalar_packet(); ts(r)=1e3*toc(z);
    z=tic; batch_packet();  tb(r)=1e3*toc(z);
end

fprintf(' active reference version                 = %.0f\n',double(replanner.active_reference_version));
fprintf(' correction count                         = %d\n',numel(C));
fprintf(' horizon N                                = %d\n',N);
fprintf(' equivalence grid count                   = %d\n',numel(tg));
fprintf(' max |p scalar-batch|                     = %.17g\n',dp);
fprintf(' max |q scalar-batch|                     = %.17g\n',dq);
fprintf(' diagnostic equivalence tolerance         = %.17g\n',tol);
fprintf(' scalar packet min/med/max [ms]           = %.6g / %.6g / %.6g\n',min(ts),median(ts),max(ts));
fprintf(' batch  packet min/med/max [ms]           = %.6g / %.6g / %.6g\n',min(tb),median(tb),max(tb));
fprintf(' equivalence closed                       = %d\n',eqClosed);
fprintf(' safety theorem changed                   = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R40A_REPAIRED_PACKET_BATCH_SHADOW_V1', ...
    'closed',eqClosed,'max_p_diff',dp,'max_q_diff',dq,'tol',tol, ...
    'scalar_ms',ts,'batch_ms',tb,'N',N,'grid_count',numel(tg), ...
    'correction_count',numel(C));
assignin('base','AF7R40A_REPAIRED_PACKET_BATCH_SHADOW_OUT',OUT);

    function [p,q]=scalar_packet()
        p=zeros(3,N); q=zeros(3,N);
        for kk=1:N
            xd=double(R.eval_fn(tq(kk))); xd=xd(:);
            xf=double(R.flat_eval_fn(tq(kk))); xf=xf(:);
            p(:,kk)=xd(1:3);
            rr=xf(9:11)+[0;0;double(replanner.gravity)];
            nr=norm(rr);
            q(:,kk)=-rr/nr;
        end
    end

    function [p,q]=batch_packet()
        [Z,vv]=af7r33_flat_jet_batch_model(replanner,tq,R.batch_flat_model);
        assert(all(vv));
        p=Z(1:3,:);
        rr=Z(7:9,:)+[0;0;double(replanner.gravity)];
        nr=sqrt(sum(rr.^2,1));
        q=-rr./nr;
    end
end
