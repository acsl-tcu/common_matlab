function R = AF7R58R45R1_TERMINAL_AUDIT(snapshotFile,liveInstance)
% READ-ONLY. Snapshot CP end value is the left-limit geometry ONLY.
% Whether it continues as a hold, jumps or gets retimed is determined by
% the live committed representation, not by this snapshot's CP arrays.
root=fileparts(mfilename('fullpath'));
if nargin<1 || isempty(snapshotFile)
    snapshotFile=fullfile(root,'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
end
assert(isfile(snapshotFile),'[R45R1] Source snapshot missing: %s',snapshotFile);
D=load(snapshotFile,'Snapshot');
assert(isfield(D,'Snapshot'),'[R45R1] Snapshot field missing.');
S=D.Snapshot;
assert(isfield(S,'corrections') && numel(S.corrections)==2, ...
    '[R45R1] Exactly two correction layers required.');
G=S.corrections;
P1=local_last_cp(G(1));P2=local_last_cp(G(2));
assert(double(G(1).segment_t1(end))==double(G(2).segment_t1(end)), ...
    '[R45R1] Different layer end times; no combined terminal claim.');
tEnd=double(G(1).segment_t1(end));
z=P1(:,end)+P2(:,end);
R=struct('schema','AF7R58R45R1_TERMINAL_AUDIT_V1', ...
    'source_snapshot',char(snapshotFile), ...
    'commits',double(S.commit_count), ...
    'stored_end_s',tEnd, ...
    'combined_left_limit_m',z, ...
    'combined_left_limit_norm_m',norm(z), ...
    'terminal_continuation_mode','UNKNOWN_FROM_POLYGON_SNAPSHOT', ...
    'terminal_continuous_rejoin_proved',false, ...
    'extension_alone_guarantees_rejoin',false, ...
    'live_representation_found',false, ...
    'simulation_flight_end_s',NaN, 'simulation_total_end_s',NaN, ...
    'runtime_assumptions_used',false);
runner=fullfile(fileparts(root),'run_flight_headless.m');
if isfile(runner)
    code=fileread(runner);
    tok=regexp(code,'elseif\s+time\.t\s*<\s*([0-9]+(?:\.[0-9]+)?)', ...
        'tokens','once');
    if ~isempty(tok),R.simulation_flight_end_s=str2double(tok{1});end
    tok=regexp(code,'(?m)^\s*te\s*=\s*([0-9]+(?:\.[0-9]+)?)\s*;', ...
        'tokens','once');
    if ~isempty(tok),R.simulation_total_end_s=str2double(tok{1});end
end
if nargin<2 || isempty(liveInstance)
    liveInstance=[];
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        liveInstance=evalin('base','replanner_instance');
    end
end
if ~isempty(liveInstance) && isobject(liveInstance) && ...
        isprop(liveInstance,'active_repaired_reference')
    A=liveInstance.active_repaired_reference;
    if isstruct(A) && isfield(A,'representation') && ...
            isstruct(A.representation)
        P=A.representation;
        R.live_representation_found=true;
        if isfield(A,'path_parameterization') && ...
                isstruct(A.path_parameterization) && ...
                isfield(A.path_parameterization,'kind')
            R.path_parameterization_kind=char(A.path_parameterization.kind);
        end
        if isfield(P,'delay_preserving')
            R.delay_preserving=logical(P.delay_preserving);
        end
        if isfield(P,'terminal_hold_start')
            R.live_terminal_hold_start_s=double(P.terminal_hold_start);
        end
        if isfield(P,'terminal_offset')
            q=double(P.terminal_offset(:));
            if numel(q)==3 && all(isfinite(q))
                R.last_commit_terminal_offset_m=q;
            end
        end
        if isfield(A,'rejoin_time')
            R.live_rejoin_time_s=double(A.rejoin_time);
        end
        R.terminal_continuation_mode='SEE_LIVE_REPRESENTATION_FIELDS';
    end
end
fprintf('[R45R1] 2-layer correction left-limit at %.6f s = [%+.9g, %+.9g, %+.9g] m; norm=%.9g m\n', ...
    tEnd,z(1),z(2),z(3),norm(z));
if isnan(R.simulation_flight_end_s)
    fprintf('[R45R1] Flight-to-landing switch not identified from runner source.\n');
else
    fprintf('[R45R1] Flight-to-landing switch in runner: %.6f s.\n',R.simulation_flight_end_s);
end
fprintf('[R45R1] Longer plotting/running alone does NOT certify return; continuation authority must be inspected.\n');
end

function P=local_last_cp(G)
C=G.control_points;
if iscell(C),P=double(C{end});
elseif isnumeric(C) && ndims(C)==3,P=double(C(:,:,end));
elseif isnumeric(C) && isequal(size(C),[3 16]),P=double(C);
else,error('[R45R1] Unexpected polygon layout.');end
assert(isequal(size(P),[3 16]) && all(isfinite(P(:))), ...
    '[R45R1] Invalid last control polygon.');
end
