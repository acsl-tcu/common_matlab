function OUT = AF7R49C_PRODUCTION_CLOSEOUT(logger,replanner,runtime_profile)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R49C | PRODUCTION TIMING + DETOUR CLOSEOUT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] postRunOnly=1 | noOnlineInstrumentation=1 | externalRuntimeProfile=1\n');

D=logger.Data;A=D.agent;if numel(A)>1,A=A(1);end
Cr=result_cells(A,'reference');
if isfield(D,'t')&&~isempty(D.t),t=double(D.t(:));else,t=(0:numel(Cr)-1)'*0.025;end
n=min(numel(t),numel(Cr));t=t(1:n);
req=nan(n,1);stc=nan(n,1);cb=nan(n,1);internal=nan(n,1);obj=nan(n,1);
for k=1:n
 st=member(Cr{k},'state');
 req(k)=scalar(member(st,'generator_request_count'));stc(k)=scalar(member(st,'generator_last_status_code'));
 cb(k)=scalar(member(st,'generator_last_callback_ms'));internal(k)=scalar(member(st,'r9_stage_total_internal_ms'));
end

K=double(runtime_profile.k);tr=double(runtime_profile.t(1:K));ph=runtime_profile.phase(1:K);
on=1e3*double(runtime_profile.online_before_plant_s(1:K));fu=1e3*double(runtime_profile.full_iteration_s(1:K));pl=1e3*double(runtime_profile.simulation_plant_s(1:K));
wf=(ph=='f'); w10=wf & tr>=9.80 & tr<=10.20;

fprintf('\n[A. EXTERNAL FLIGHT TIMING]\n');
show('online Flight',on(wf));show('full iteration Flight',fu(wf));show('simulation plant Flight',pl(wf));
fprintf(' Flight online >=25 ms cycles             = %d\n',nnz(on(wf)>=25));
fprintf(' 9.80--10.20 online >=20 ms cycles        = %d\n',nnz(on(w10)>=20));
show('9.80--10.20 online',on(w10));show('9.80--10.20 plant',pl(w10));

fprintf('\n[B. REQUESTS]\n');
prev=0;Nreq=0;
for k=1:n
 if isfinite(req(k))&&req(k)>prev
   Nreq=Nreq+1;fprintf(' %2d t=%.3f status=%g callback=%.4g ms internal=%.4g ms\n',Nreq,t(k),stc(k),cb(k),internal(k));prev=req(k);
 end
end
fprintf(' request count                            = %d\n',Nreq);

fprintf('\n[C. SYNTHESIS / AUTHORITY]\n');
fprintf(' objective code/name                      = %.0f / %s\n',double(replanner.synthesis_objective_code),objective_name(replanner.synthesis_objective_code));
fprintf(' exact integrated correction energy       = %.15g m^2 s\n',double(replanner.synthesis_correction_energy_m2s));
fprintf(' active reference mode/version            = %s / %.0f\n',char(replanner.active_reference_mode),double(replanner.active_reference_version));
fprintf(' reference-only commits                   = %.0f\n',double(replanner.reference_only_commit_count));
fprintf(' execution-aware commits                  = %.0f\n',double(replanner.execution_aware_commit_count));
fprintf(' light runtime audit enabled              = %d\n',logical(replanner.ENABLE_LIGHT_RUNTIME_AUDIT));
fprintf(' trigger forensic capture enabled         = %d\n',logical(replanner.CAPTURE_REAL_TRIGGER_EVENT));
fprintf(' proposal diagnostic mirror enabled       = %d\n',logical(replanner.ENABLE_PROPOSAL_DIAGNOSTIC_MIRROR));
fprintf(' observer hard support enabled            = %d\n',logical(replanner.ENABLE_OBSERVER_HARD_SUPPORT));
fprintf(' observer runtime stats enabled           = %d\n',logical(replanner.ENABLE_OBSERVER_RUNTIME_STATS));
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R49C_PRODUCTION_CLOSEOUT_V1','request_count',Nreq, ...
 'flight_online_ge25',nnz(on(wf)>=25),'window_online_ge20',nnz(on(w10)>=20), ...
 'objective_code',double(replanner.synthesis_objective_code), ...
 'correction_energy_m2s',double(replanner.synthesis_correction_energy_m2s));
assignin('base','AF7R49C_PRODUCTION_CLOSEOUT_OUT',OUT);
end
function show(n,v),v=double(v(:));v=v(isfinite(v));if isempty(v),fprintf(' %-38s = NaN\n',n);else,fprintf(' %-38s min/med/max = %.6g / %.6g / %.6g ms\n',n,min(v),median(v),max(v));end,end
function C=result_cells(A,n),C={};if isstruct(A)&&isfield(A,n),S=A.(n);if isstruct(S)&&isfield(S,'result'),R=S.result;if iscell(R),C=R(:);else,C=num2cell(R(:));end,end,end,end
function x=member(S,n),x=[];try,if isstruct(S)&&isfield(S,n),x=S.(n);elseif isobject(S)&&isprop(S,n),x=S.(n);end,catch,end,end
function y=scalar(x),y=NaN;if (isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x)),y=double(x);end,end
function n=objective_name(c),if double(c)==1,n='MIN_CORRECTION_ENERGY';elseif double(c)==2,n='MIN_SNAP_RECOVERABILITY';else,n='UNKNOWN';end,end
