function A = AF7R47D1_STAGE2_TRIGGER_AUDIT
% AF7R47D1 diagnostic-only. Run after:
%   D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT;

assert(evalin('base','exist(''D38'',''var'')==1'), ...
    'Run: D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT; first.');
D38=evalin('base','D38');
assert(isstruct(D38)&&isfield(D38,'pipeline'));
P=D38.pipeline;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R47D1 | STAGE2 + TRIGGER TIMING AUDIT (DIAGNOSTIC ONLY)\n');
fprintf('====================================================================================================\n');

fprintf('\n[A. STAGE1 REQUESTS]\n');
R=double(P.r47d1_request_ring);
n=min(double(P.r47d1_request_ring_count),size(R,1));
h=double(P.r47d1_request_ring_head);
REQ=[];
if n>0
    cap=size(R,1);
    first=mod(h-n,cap)+1;
    idx=mod((first-1)+(0:n-1),cap)+1;
    REQ=R(idx,:);
    Treq=array2table(REQ,'VariableNames',{ ...
        'seq','request_t','activation_t','trigger_ttc','activation_ttc', ...
        'collision_t','active_version','repaired_mode','obstacle_id', ...
        'component_code','success','reason','native_total_ms','stage1_ms'});
    disp(Treq);
else
    fprintf('No request rows captured.\n');
end

fprintf('\n[B. STAGE2 SUBSTAGES]\n');
R=double(P.r47d1_stage2_ring);
n=min(double(P.r47d1_stage2_ring_count),size(R,1));
h=double(P.r47d1_stage2_ring_head);
S2=[];
if n>0
    cap=size(R,1);
    first=mod(h-n,cap)+1;
    idx=mod((first-1)+(0:n-1),cap)+1;
    S2=R(idx,:);
    Ts2=array2table(S2,'VariableNames',{ ...
        'seq','time_s','source_version','stage2_total_ms','reference_cert_ms', ...
        'prepare_outer_ms','precheck_ms','resource_ms','sweep_ms', ...
        'packet_ms','sensor_ms','authority_eval_ms','postcache_ms', ...
        'bundle_ms','prepare_inner_ms'});
    disp(Ts2);
else
    fprintf('No Stage2 rows captured.\n');
end

fprintf('\n[C. KEYS]\n');
fprintf(' reason 22 : native unsafe episode empty/not certified at activation-time active reference.\n');
fprintf(' postcache_ms: 4-s flat-jet batch prebuild for the first post-commit cycle.\n');
fprintf(' authority_eval_ms: dependency snapshot + evaluator/yaw handle + reference metadata construction.\n');
fprintf(' All fields here are diagnostic only and have no safety/acceptance authority.\n');
fprintf('====================================================================================================\n');

A=struct('requests',REQ,'stage2',S2);
assignin('base','R47D1',A);
end
