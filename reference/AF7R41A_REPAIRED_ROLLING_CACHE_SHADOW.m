function OUT = AF7R41A_REPAIRED_ROLLING_CACHE_SHADOW(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R41A | REPAIRED ROLLING-CACHE EXACT EQUIVALENCE + TIMING SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | implementationEquivalence=1 | safetyAcceptance=0 | sourceUnmodified=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(strcmpi(char(replanner.active_reference_mode),'REPAIRED'), ...
    '[AF7R41A] run on the completed AF7R40 REPAIRED workspace.');
R=replanner.active_repaired_reference;
assert(isfield(R,'eval_fn')&&isa(R.eval_fn,'function_handle'));
assert(isfield(R,'flat_eval_fn')&&isa(R.flat_eval_fn,'function_handle'));
assert(isfield(R,'batch_flat_model')&&isstruct(R.batch_flat_model)&& ...
    isfield(R.batch_flat_model,'certified')&&logical(R.batch_flat_model.certified));
assert(exist('af7r33_flat_jet_batch_model','file')==2);

dt=double(replanner.prediction_dt); H=double(replanner.horizon_s);
N=round(H/dt)+1;
C=R.batch_flat_model.corrections; assert(~isempty(C));
tStart=double(replanner.last_replan_commit_time);
if ~(isfinite(tStart)&&tStart>=0), tStart=min(arrayfun(@(x)double(x.t0),C)); end
tLast=max(arrayfun(@(x)double(x.t1),C));
tStop=max(tLast+H,tStart+2*H);
nt=max(2,round((tStop-tStart)/dt)+1);
times=tStart+(0:nt-1)*dt;

[Pcache,Qcache]=full_packet(times(1));
maxP=0; maxQ=0; maxFlatPos=0;
for ii=1:numel(times)
    t=times(ii); [Pfull,Qfull]=full_packet(t);
    if ii==1
        Proll=Pcache; Qroll=Qcache;
    else
        Proll=[Pcache(:,2:end),zeros(3,1)];
        Qroll=[Qcache(:,2:end),zeros(3,1)];
        tf=t+(N-1)*dt;
        xf=double(R.flat_eval_fn(tf)); xf=xf(:);
        xd=double(R.eval_fn(tf)); xd=xd(:);
        assert(numel(xf)>=11&&numel(xd)>=3);
        maxFlatPos=max(maxFlatPos,max(abs(xd(1:3)-xf(1:3))));
        Proll(:,end)=xf(1:3);
        sf=xf(9:11)+[0;0;double(replanner.gravity)];
        Qroll(:,end)=-sf/norm(sf);
    end
    maxP=max(maxP,max(abs(Pfull-Proll),[],'all'));
    maxQ=max(maxQ,max(abs(Qfull-Qroll),[],'all'));
    Pcache=Proll; Qcache=Qroll;
end
scale=max([1,max(abs(Pcache),[],'all')]); tol=8192*eps(scale);
closed=maxP<=tol && maxQ<=tol && maxFlatPos<=tol;

tBench=times(min(numel(times),max(2,round(numel(times)/2))));
[P0,Q0]=full_packet(tBench-dt);
full_packet(tBench); rolling_once(P0,Q0,tBench);
nrep=51; tfull=zeros(1,nrep); troll=zeros(1,nrep);
for rr=1:nrep
    z=tic; full_packet(tBench); tfull(rr)=1e3*toc(z);
    z=tic; rolling_once(P0,Q0,tBench); troll(rr)=1e3*toc(z);
end

fprintf(' active reference version                 = %.0f\n',double(replanner.active_reference_version));
fprintf(' correction count                         = %d\n',numel(C));
fprintf(' horizon N                                = %d\n',N);
fprintf(' rolling cycles checked                   = %d\n',numel(times));
fprintf(' max |p full-roll|                        = %.17g\n',maxP);
fprintf(' max |q full-roll|                        = %.17g\n',maxQ);
fprintf(' max |eval_fn p-flat_eval_fn p|           = %.17g\n',maxFlatPos);
fprintf(' diagnostic equivalence tolerance         = %.17g\n',tol);
fprintf(' full batch min/med/max [ms]              = %.6g / %.6g / %.6g\n',min(tfull),median(tfull),max(tfull));
fprintf(' rolling shift1 min/med/max [ms]          = %.6g / %.6g / %.6g\n',min(troll),median(troll),max(troll));
fprintf(' rolling equivalence closed               = %d\n',closed);
fprintf(' safety theorem changed                   = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R41A_REPAIRED_ROLLING_CACHE_SHADOW_V1', ...
    'closed',closed,'max_p_diff',maxP,'max_q_diff',maxQ, ...
    'max_flat_position_diff',maxFlatPos,'tol',tol, ...
    'full_ms',tfull,'rolling_ms',troll,'N',N,'cycles_checked',numel(times));
assignin('base','AF7R41A_REPAIRED_ROLLING_CACHE_SHADOW_OUT',OUT);

    function [p,q]=full_packet(t)
        tq=t+(0:N-1)*dt;
        [Y,v]=af7r33_flat_jet_batch_model(replanner,tq,R.batch_flat_model);
        assert(all(v)); Y=double(Y); p=Y(1:3,:);
        sf=Y(7:9,:)+[0;0;double(replanner.gravity)];
        q=-sf./sqrt(sum(sf.^2,1));
    end
    function [p,q]=rolling_once(pOld,qOld,t)
        p=[pOld(:,2:end),zeros(3,1)]; q=[qOld(:,2:end),zeros(3,1)];
        xf=double(R.flat_eval_fn(t+(N-1)*dt)); xf=xf(:);
        p(:,end)=xf(1:3); sf=xf(9:11)+[0;0;double(replanner.gravity)];
        q(:,end)=-sf/norm(sf);
    end
end
