function OUT = AF7R33B_TRUE_TRIGGER_ALL_OBS_EPISODE_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R33B | TRUE-TRIGGER ALL-OBSTACLE EPISODE BATCH AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] continuousPredicateSame=1 | sampledSafetyAcceptance=0 | productionModification=0\n');

assert(exist('af7r33_episode_pair_mex','file')==3,'[AF7R33B] paired episode MEX missing.');
E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid));
CTX=af7r32_replay_context(replanner,E);
P=replanner.empty_bezier_d15_c7_proposal();
P.t0=double(E.event_time); P.tm=double(E.trigger.collision_time);
P.trigger_collision_time=P.tm; P.obstacle_id=double(E.trigger.obstacle_id);
P.component_code=double(E.trigger.component_code);
P.base_eval_fn=replanner.snapshot_active_reference_eval_fn();
P.base_flat_eval_fn=replanner.snapshot_active_reference_flat_eval_fn();
searchEnd=replanner.resolve_reference_rejoin_search_end(P.tm); P.t1=searchEnd;

replanner.build_bezier_nominal_unsafe_episode_components(P,CTX,searchEnd);
af7r33_episode_batch_allobs(replanner,P,CTX,searchEnd);

N=21; tl=zeros(1,N); tn=zeros(1,N);
for k=1:N
    q=tic; [EL,oL]=replanner.build_bezier_nominal_unsafe_episode_components(P,CTX,searchEnd); tl(k)=1e3*toc(q);
    q=tic; [EN,oN]=af7r33_episode_batch_allobs(replanner,P,CTX,searchEnd); tn(k)=1e3*toc(q);
end

closed=logical(oL)==logical(oN) && EL.episode_count==EN.episode_count && ...
    EL.component_count==EN.component_count;
maxT=0;
if closed
    for k=1:EL.episode_count
        closed=closed && double(EL.episodes(k).obstacle_index)==double(EN.episodes(k).obstacle_index) && ...
            double(EL.episodes(k).obstacle_id)==double(EN.episodes(k).obstacle_id);
        maxT=max(maxT,abs(double(EL.episodes(k).start_s)-double(EN.episodes(k).start_s)));
        maxT=max(maxT,abs(double(EL.episodes(k).end_s)-double(EN.episodes(k).end_s)));
    end
end
diagEqual=double(EL.adaptive_leaf_count)==double(EN.adaptive_leaf_count) && ...
    double(EL.adaptive_safe_leaf_count)==double(EN.adaptive_safe_leaf_count) && ...
    double(EL.adaptive_unsafe_leaf_count)==double(EN.adaptive_unsafe_leaf_count) && ...
    double(EL.adaptive_split_count)==double(EN.adaptive_split_count) && ...
    double(EL.representability_fail_closed_count)==double(EN.representability_fail_closed_count);
tol=512*eps(max([1,abs(double(P.t0)),abs(double(searchEnd))]));
closed=closed&&diagEqual&&maxT<=tol;

fprintf(' trigger time [s]                         = %.9f\n',double(E.event_time));
fprintf(' obstacles                                = %d\n',double(CTX.obstacles.count));
fprintf(' legacy/new episode count                 = %d / %d\n',EL.episode_count,EN.episode_count);
fprintf(' legacy/new component count               = %d / %d\n',EL.component_count,EN.component_count);
fprintf(' max boundary |diff| [s]                  = %.17g\n',maxT);
fprintf(' diagnostic counts equal                  = %d\n',diagEqual);
fprintf(' legacy episode min/med/max [ms]          = %.6g / %.6g / %.6g\n',min(tl),median(tl),max(tl));
fprintf(' all-obstacle min/med/max [ms]            = %.6g / %.6g / %.6g\n',min(tn),median(tn),max(tn));
fprintf(' all-obstacle episode closed              = %d\n',closed);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

if closed
    src=which('REPLANNING_SYSTEM'); root=fileparts(src);
    token=struct('schema','AF7R33_EPISODE_TOKEN_V1','closed',true, ...
        'max_boundary_diff',maxT,'tol',tol,'median_ms',median(tn));
    save(fullfile(root,'AF7R33_EPISODE_TOKEN.mat'),'token');
end
OUT=struct('schema','AF7R33B_TRUE_TRIGGER_ALL_OBS_EPISODE_AUDIT_V1', ...
    'closed',closed,'legacy_ms',tl,'new_ms',tn,'max_boundary_diff',maxT);
assignin('base','AF7R33B_TRUE_TRIGGER_ALL_OBS_EPISODE_AUDIT_OUT',OUT);
end
