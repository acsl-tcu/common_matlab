function S=AF7R58R122_SOURCE_CHECK()
% Read-only dependency verification. No Flight mutation or installation.
req={'REPLANNING_SYSTEM','TIME_VARYING_REFERENCE', ...
    'SUSPENDED_LOAD_REF_ADJUST','HLC_SUSPENDED_LOAD', ...
    'AF7R58R118_YAW_JETS_C6','AF7R58R118_HEADING_C6_CERT', ...
    'AF7R58R121_WHOLEBODY_PLANNER', ...
    'AF7R58R122_BIND_LIVE_C7', ...
    'AF7R58R122_NOMINAL_PAYLOAD_JETS', ...
    'AF7R58R122_C7_TO_RAW_XD28'};
missing={}; paths=cell(1,numel(req));
for k=1:numel(req)
    paths{k}=which(req{k});
    if isempty(paths{k})
        missing{end+1}=req{k}; %#ok<AGROW>
        fprintf('[R122][MISSING] %s\n',req{k});
    else
        fprintf('[R122][FOUND] %s => %s\n',req{k},paths{k});
    end
end
S=struct('status','MISSING_R122_DEPENDENCY','missing',{missing}, ...
    'dependency_paths',{paths},'source_static_contract',false, ...
    'native_flight_modified',false);
if ~isempty(missing),return;end
src=fileread(paths{1});
sload=fileread(paths{3});hlc=fileread(paths{4});
valid=contains(src,'nominal_flat_derivative_fn') && ...
    contains(src,'r63_live_c7_authority') && ...
    contains(src,'active_jets_0_to_7') && ...
    contains(sload,'xd(3) - L') && ...
    contains(hlc,'28 - size(xd, 1)');
% More importantly, the actual static implementation must declare the
% 28-wide HLC command and the one-time SLOAD raw-z transform.
S.source_static_contract=logical(valid);
if valid
    S.status='PASS_R122_SOURCE_0_TO_7_EXISTS_AND_HLC_28_CONTRACT';
else
    S.status='BLOCKED_R122_SOURCE_CONTRACT_CHANGED';
end
fprintf('[R122] %s; native old trigger/generator unchanged.\n',S.status);
end
