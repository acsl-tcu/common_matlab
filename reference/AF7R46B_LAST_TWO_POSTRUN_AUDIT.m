function OUT = AF7R46B_LAST_TWO_POSTRUN_AUDIT(logger,replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R46B | FLIGHT ENTRY + REQUEST WINDOW POST-RUN AUDIT\n');
fprintf('====================================================================================================\n');
D=logger.Data; A=D.agent; if numel(A)>1,A=A(1);end
Cr=getcells(A,'reference'); Cc=getcells(A,'controller');
if isfield(D,'t')&&~isempty(D.t),t=double(D.t(:));else,t=(0:max(numel(Cr),numel(Cc))-1)'*double(replanner.prediction_dt);end
n=min([numel(t),numel(Cr),numel(Cc)]); t=t(1:n);
on=nan(n,1); ob=nan(n,1); dep=nan(n,1); bg=nan(n,1); sg=nan(n,1); os=nan(n,1); hd=nan(n,1); stc=nan(n,1); re=nan(n,1);
for k=1:n
 st=getm(Cr{k},'state'); on(k)=1e3*sc(getm(Cc{k},'online_compute_total')); ob(k)=sc(getm(st,'rt_observer_ms')); dep(k)=sc(getm(st,'rt_spike_stage_dependency_ms')); bg(k)=sc(getm(st,'rt_spike_stage_base_cert_geom_ms')); sg(k)=sc(getm(st,'rt_spike_stage_sensor_trigger_ms')); os(k)=sc(getm(st,'rt_spike_stage_output_state_ms')); hd(k)=sc(getm(st,'generator_last_cycle_critical_ms')); stc(k)=sc(getm(st,'generator_last_status_code')); re(k)=sc(getm(st,'rt_light_e2e_ms'));
end
fprintf('\n[A. FLIGHT ENTRY]\n'); fprintf(' t online refE2E observer depAggregate baseGeom sensorTrig outState [ms]\n'); ii=find(t>=5&t<=5.15); for k=ii(:).',fprintf('%5.3f %6.2f %6.2f %7.3f %11.3f %8.3f %10.3f %8.3f\n',t(k),on(k),re(k),ob(k),dep(k),bg(k),sg(k),os(k));end
fprintf('\n[B. REQUEST WINDOW]\n'); fprintf(' t status online refE2E hard [ms]\n'); jj=find(t>=9.8&t<=10.2&isfinite(stc)&stc~=0); for k=jj(:).',fprintf('%5.3f %6.0f %6.2f %6.2f %6.3f\n',t(k),stc(k),on(k),re(k),hd(k));end
O=replanner.get_observer_hard_support_status(); S=replanner.get_avoidance_mode_status(); avoid=strcmpi(char(replanner.active_reference_mode),'REPAIRED')&&double(S.reference_only_commit_count)>0;
fprintf('\n[C. STATUS]\n'); fprintf(' compact native proposal count           = %.0f\n',double(replanner.compact_native_proposal_count)); fprintf(' fast changed-speed packet rebuild count = %.0f\n',double(replanner.reference_only_fast_packet_rebuild_count)); fprintf(' observer static setup cache valid       = %d\n',logical(replanner.observer_static_setup_cache_valid)); fprintf(' observer mean/max [ms]                  = %.6g / %.6g\n',double(O.runtime_mean_ms),double(O.runtime_max_ms)); fprintf(' active reference mode/version           = %s / %.0f\n',char(replanner.active_reference_mode),double(replanner.active_reference_version)); fprintf(' repaired avoidance reference active     = %d\n',avoid); fprintf(' full execution-aware authority active   = %d\n',strcmpi(char(S.commit_policy),'EXECUTION_AWARE')); fprintf('====================================================================================================\n'); OUT=struct('schema','AF7R46B_V1','avoidance_reference_active',avoid,'observer',O,'avoidance',S); assignin('base','AF7R46B_LAST_TWO_POSTRUN_AUDIT_OUT',OUT);
end
function C=getcells(A,n),C={};if~isstruct(A)||~isfield(A,n),return;end;S=A.(n);if~isstruct(S)||~isfield(S,'result'),return;end;R=S.result;if iscell(R),C=R(:);else,C=num2cell(R(:));end;end
function x=getm(a,n),x=[];try,if isstruct(a)&&isfield(a,n),x=a.(n);elseif isobject(a)&&isprop(a,n),x=a.(n);end;catch,end;end
function y=sc(x),y=NaN;if(isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x)),y=double(x);end;end
