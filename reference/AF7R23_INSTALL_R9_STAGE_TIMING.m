function OUT = AF7R23_INSTALL_R9_STAGE_TIMING()
% AF7R23_INSTALL_R9_STAGE_TIMING
% =========================================================================
% Diagnostic-only instrumentation for the active R9 REPLANNING_SYSTEM.m.
%
% Adds scalar timing fields for:
%   1) unsafe-episode construction
%   2) piecewise towing synthesis
%   3) candidate certification loop
%
% It does NOT change any acceptance/rejection condition.
% It does NOT change the 4 ms deadline.
% It does NOT change sampled/non-sampled authority.
% =========================================================================

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R23 | INSTALL R9 TOP-LEVEL STAGE TIMING\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R23] REPLANNING_SYSTEM.m not found.');
raw=fileread(src);

assert(contains(raw,'AF7R21-R9'), ...
    '[AF7R23] R9 marker not found. Install R9 first.');
assert(~contains(raw,'AF7R23_STAGE_TIMING'), ...
    '[AF7R23] Stage timing already installed.');

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss'));
backup=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_R23_' stamp '.m']);
copyfile(src,backup);

% -------------------------------------------------------------------------
% 1) Add scalar properties near existing BEZIER shadow diagnostics.
% -------------------------------------------------------------------------
old1=['        bezier_shadow_last_dz                 = NaN;' newline];
new1=[old1 ...
'        % AF7R23_STAGE_TIMING diagnostic-only scalar mirrors' newline ...
'        r9_stage_episode_ms                    = NaN;' newline ...
'        r9_stage_synthesis_ms                  = NaN;' newline ...
'        r9_stage_candidate_loop_ms             = NaN;' newline ...
'        r9_stage_total_internal_ms              = NaN;' newline];
raw=replace_once(raw,old1,new1,'timing properties');

% -------------------------------------------------------------------------
% 2) Add logger state property names.
% -------------------------------------------------------------------------
old2=['                "bezier_shadow_last_dz", ...' newline ...
      '                "geometry_shadow_pass", ...'];
new2=['                "bezier_shadow_last_dz", ...' newline ...
      '                "r9_stage_episode_ms", ...' newline ...
      '                "r9_stage_synthesis_ms", ...' newline ...
      '                "r9_stage_candidate_loop_ms", ...' newline ...
      '                "r9_stage_total_internal_ms", ...' newline ...
      '                "geometry_shadow_pass", ...'];
raw=replace_once(raw,old2,new2,'sensor property list');

% -------------------------------------------------------------------------
% 3) Log scalar property values into state.
% There are two state writers in the current class; replace all exact blocks.
% -------------------------------------------------------------------------
old3=['            st.bezier_shadow_last_dz=obj.bezier_shadow_last_dz;' newline ...
      '            st.geometry_shadow_pass=logical(obj.geometry_shadow_pass);'];
new3=['            st.bezier_shadow_last_dz=obj.bezier_shadow_last_dz;' newline ...
      '            st.r9_stage_episode_ms=obj.r9_stage_episode_ms;' newline ...
      '            st.r9_stage_synthesis_ms=obj.r9_stage_synthesis_ms;' newline ...
      '            st.r9_stage_candidate_loop_ms=obj.r9_stage_candidate_loop_ms;' newline ...
      '            st.r9_stage_total_internal_ms=obj.r9_stage_total_internal_ms;' newline ...
      '            st.geometry_shadow_pass=logical(obj.geometry_shadow_pass);'];
raw=replace_all_at_least(raw,old3,new3,1,'state timing mirrors');

% -------------------------------------------------------------------------
% 4) Capture proposal timing in record_bezier_shadow_proposal().
% -------------------------------------------------------------------------
anchor4=['            if isfield(P,''geometry_shadow_certificate'') && ...' newline ...
         '                    isstruct(P.geometry_shadow_certificate)' newline];
insert4=[ ...
'            if isfield(P,''r9_stage_timing'') && isstruct(P.r9_stage_timing)' newline ...
'                T9=P.r9_stage_timing;' newline ...
'                if isfield(T9,''episode_ms''), obj.r9_stage_episode_ms=double(T9.episode_ms); end' newline ...
'                if isfield(T9,''synthesis_ms''), obj.r9_stage_synthesis_ms=double(T9.synthesis_ms); end' newline ...
'                if isfield(T9,''candidate_loop_ms''), obj.r9_stage_candidate_loop_ms=double(T9.candidate_loop_ms); end' newline ...
'                if isfield(T9,''total_internal_ms''), obj.r9_stage_total_internal_ms=double(T9.total_internal_ms); end' newline ...
'            end' newline ...
'' newline ...
anchor4];
raw=replace_once(raw,anchor4,insert4,'proposal timing capture');

% -------------------------------------------------------------------------
% 5) Add proposal schema timing struct.
% -------------------------------------------------------------------------
old5=['                ''local_vector_speed_certificate'',struct(), ...' newline ...
      '                ''piecewise_segment_t0'',[], ...'];
new5=['                ''local_vector_speed_certificate'',struct(), ...' newline ...
      '                ''r9_stage_timing'',struct(''episode_ms'',NaN, ...' newline ...
      '                    ''synthesis_ms'',NaN,''candidate_loop_ms'',NaN, ...' newline ...
      '                    ''total_internal_ms'',NaN), ...' newline ...
      '                ''piecewise_segment_t0'',[], ...'];
raw=replace_once(raw,old5,new5,'proposal timing schema');

% -------------------------------------------------------------------------
% 6) Time unsafe episode construction.
% -------------------------------------------------------------------------
old6=['            [EPI,okEpi]=obj.build_bezier_nominal_unsafe_episode_components( ...' newline ...
      '                OUT,CTX,searchEnd);' newline];
new6=['            r9TotalTic=tic;' newline ...
      '            r9StageTic=tic;' newline ...
      old6 ...
      '            OUT.r9_stage_timing.episode_ms=1e3*toc(r9StageTic);' newline];
raw=replace_once(raw,old6,new6,'episode timing');

% -------------------------------------------------------------------------
% 7) Time piecewise synthesis.
% -------------------------------------------------------------------------
old7=['            [CANDS,SYN]=obj.build_bezier_piecewise_towing_candidates( ...' newline ...
      '                OUT,CTX,EPI,OUT.t1);' newline];
new7=['            r9StageTic=tic;' newline ...
      old7 ...
      '            OUT.r9_stage_timing.synthesis_ms=1e3*toc(r9StageTic);' newline];
raw=replace_once(raw,old7,new7,'synthesis timing');

% -------------------------------------------------------------------------
% 8) Time the complete candidate certificate loop.
% -------------------------------------------------------------------------
old8=['            for ic=1:numel(CANDS)' newline];
new8=['            r9StageTic=tic;' newline ...
      old8];
raw=replace_once(raw,old8,new8,'candidate loop start');

old9=['            OUT.piecewise_speed_pass_count=speedPass;' newline];
new9=['            OUT.r9_stage_timing.candidate_loop_ms=1e3*toc(r9StageTic);' newline ...
      '            OUT.r9_stage_timing.total_internal_ms=1e3*toc(r9TotalTic);' newline ...
      old9];
raw=replace_once(raw,old9,new9,'candidate loop end');

% Marker.
oldH=['% AF7R21-R9: two-stage global-triangle -> local-vector continuous speed certificate.' newline];
newH=[oldH ...
'% AF7R23_STAGE_TIMING: diagnostic-only top-level R9 timing instrumentation.' newline];
raw=replace_once(raw,oldH,newH,'header marker');

fid=fopen(src,'w');
assert(fid>=0,'[AF7R23] cannot open source for write.');
fwrite(fid,raw,'char');
fclose(fid);

OUT=struct('schema','AF7R23_INSTALL_R9_STAGE_TIMING_V1', ...
    'source',src,'backup',backup,'installed',true, ...
    'acceptance_logic_modified',false);

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',backup);
fprintf(' timing instrumentation installed        = 1\n');
fprintf(' acceptance logic modified               = 0\n');
fprintf(' hard deadline modified                  = 0\n');
fprintf(' sampled acceptance modified             = 0\n');
fprintf('====================================================================================================\n');
end


function out=replace_once(in,old,new,label)
idx=strfind(in,old);
assert(numel(idx)==1, ...
    '[AF7R23] Expected one "%s" target; found %d.',label,numel(idx));
out=[in(1:idx-1) new in(idx+length(old):end)];
end


function out=replace_all_at_least(in,old,new,nmin,label)
idx=strfind(in,old);
assert(numel(idx)>=nmin, ...
    '[AF7R23] Expected at least %d "%s" targets; found %d.',nmin,label,numel(idx));
out=strrep(in,old,new);
end
