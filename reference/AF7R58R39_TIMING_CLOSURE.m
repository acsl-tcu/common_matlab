function R = AF7R58R39_TIMING_CLOSURE(path)
% Source-independent, read-only cycle-by-cycle attribution. NO safety modification.
S=load(path,'T'); assert(isfield(S,'T'),'R23 timing MAT missing T.');
T=S.T; t=double(T.time_s(:)); X=double(T.values_ms);
assert(size(X,1)==numel(t),'Timing time/value row mismatch.');
assert(all(isfinite(t)) && all(diff(t)>0),'Invalid Flight times.');
names=cellstr(string(T.labels)); names=cellfun(@strtrim,names,'UniformOutput',false);
assert(numel(names)==size(X,2),'Timing labels mismatch.');
get=@(name) local_col(X,names,name);
reference=get('reference_replanner'); controller=get('controller');
full=get('full_iteration'); online=get('online_before_plant');
components={'sensor_acquisition','sensor','estimator','reference_replanner', ...
 'controller','input_transform','simulation_plant','logger','other_overhead'};
Y=zeros(numel(t),numel(components));
for j=1:numel(components),Y(:,j)=get(components{j});end
full_residual=full-sum(Y,2);
online_components={'sensor_acquisition','sensor','estimator', ...
 'reference_replanner','controller','input_transform'};
Z=zeros(numel(t),numel(online_components));
for j=1:numel(online_components),Z(:,j)=get(online_components{j});end
online_residual=online-sum(Z,2);
assert(all(isfinite(X(:))),'Nonfinite cycle timer: cannot claim closure.');
E=T.stage_events.events; [ev_t,ev_name,ev_ms]=local_events(E);
which=contains(ev_name,'NATIVE_RETURNED')|contains(ev_name,'CERTIFIED_AND_PREPARED')|contains(ev_name,'ATOMIC_COMMIT_PASS');
ev_t=ev_t(which);ev_name=ev_name(which);ev_ms=ev_ms(which);
ind=zeros(numel(ev_t),1);
for j=1:numel(ind)
 [d,ind(j)]=min(abs(t-ev_t(j)));
 assert(d<=1e-6*max(1,abs(ev_t(j))),'Stage event is not in the Flight cycle grid.');
end
R=struct();R.schema='AF7R58R39_TIME_CLOSURE_READONLY_V1';
R.p50_reference_ms=median(reference);R.p95_reference_ms=prctile(reference,95);
R.max_reference_ms=max(reference);R.max_full_residual_ms=max(abs(full_residual));
R.max_online_residual_ms=max(abs(online_residual));
R.stage_table=table(ev_t,ev_name,reference(ind),ev_ms,reference(ind)-ev_ms,full(ind), ...
 'VariableNames',{'Time_s','Event','Reference_ms','Stage_ms','Unattributed_ms','Full_ms'});
[~,ix]=sort(reference,'descend');ix=ix(1:min(30,numel(ix)));
R.top_table=table(t(ix),reference(ix),online(ix),full(ix),controller(ix), ...
 'VariableNames',{'Time_s','Reference_ms','Online_ms','Full_ms','Controller_ms'});
R.all_cycles=table(t,reference,online,full,controller,full_residual,online_residual, ...
 'VariableNames',{'Time_s','Reference_ms','Online_ms','Full_ms','Controller_ms','FullResidual_ms','OnlineResidual_ms'});
R.n_flight=numel(t);R.max_internal_residual_ms=NaN; % no matching internal trace supplied
R.internal_closure_proved=false;
fprintf('[R39] %d cycles | reference p50/p95/max %.4f / %.4f / %.4f ms\n', ...
 R.n_flight,R.p50_reference_ms,R.p95_reference_ms,R.max_reference_ms);
fprintf('[R39] full/online closure maxima %.4g / %.4g ms | INTERNAL closure: NOT MEASURED\n', ...
 R.max_full_residual_ms,R.max_online_residual_ms);
disp(R.stage_table);
end
function v=local_col(X,names,name)
i=find(strcmp(names,name));assert(isscalar(i),'Missing or duplicate timer %s',name);v=X(:,i);
end
function [t,n,v]=local_events(E)
if iscell(E),E=[E{:}];end
assert(isstruct(E),'Malformed stage events');N=numel(E);t=nan(N,1);n=strings(N,1);v=nan(N,1);
for k=1:N
 e=E(k);if isfield(e,'name'),n(k)=string(e.name);end
 if isfield(e,'time_s'),t(k)=double(e.time_s);elseif isfield(e,'sim_t'),t(k)=double(e.sim_t);end
 if isfield(e,'stage_ms'),v(k)=double(e.stage_ms);end
end
end
