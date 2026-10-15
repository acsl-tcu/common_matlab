function OUT = AF7R27_INSTALL_R10_REALTIME_CORE_V2()
% AF7R27_INSTALL_R10_REALTIME_CORE_V2
% Corrected installer. R10 code is loaded from text snippets to avoid
% MATLAB quote/string corruption in generated patch files.

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R27 V2 | INSTALL R10 REAL-TIME CORE\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R27V2] REPLANNING_SYSTEM.m not found.');
raw=fileread(src);

assert(contains(raw,'AF7R21-R9'),'[AF7R27V2] R9 marker missing.');
assert(~contains(raw,'AF7R27-R10'),'[AF7R27V2] R10 already appears installed.');

here=fileparts(mfilename('fullpath'));
fa=fullfile(here,'AF7R27_R10A_SNIPPET.txt');
fb=fullfile(here,'AF7R27_R10B_SNIPPET.txt');
fc=fullfile(here,'AF7R27_R10C_SNIPPET.txt');
assert(isfile(fa)&&isfile(fb)&&isfile(fc), ...
    '[AF7R27V2] R10 snippet files must be in the same folder as this installer.');

newEpisode=fileread(fa);
newCorridor=fileread(fb);
newQP=fileread(fc);

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss'));
backup=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_R10_' stamp '.m']);
copyfile(src,backup);

% R10-A
a0='        function [EPI,ok] = build_bezier_nominal_unsafe_episode_components( ...';
a1='        function tf = nominal_reference_cell_continuously_safe_one_obstacle( ...';
[i0,i1]=unique_bounds(raw,a0,a1,'R10-A episode function');
raw=[raw(1:i0-1) newEpisode raw(i1:end)];

% R10-B
b0='        function [A,b,ok] = bezier_piecewise_episode_corridor_rows( ...';
b1='        function CANDS = enumerate_piecewise_sign_qp_recursive( ...';
[i0,i1]=unique_bounds(raw,b0,b1,'R10-B corridor function');
raw=[raw(1:i0-1) newCorridor raw(i1:end)];

% R10-C
c0='        function [z,cost,minSlack,ok] = solve_piecewise_fixed_sign_qp( ...';
c1='        function H = piecewise_minimum_snap_hessian( ...';
[i0,i1]=unique_bounds(raw,c0,c1,'R10-C QP solver');
raw=[raw(1:i0-1) newQP raw(i1:end)];

% Header marker.
hdr='classdef REPLANNING_SYSTEM < handle';
ih=strfind(raw,hdr);
assert(numel(ih)==1,'[AF7R27V2] class header not unique.');
pos=ih+length(hdr);
raw=[raw(1:pos) newline ...
    '% AF7R27-R10: adaptive interval cover + projected corridor hull + active-set QP.' ...
    raw(pos+1:end)];

% Fail-closed source invariants.
compact=regexprep(raw,'\s+','');
assert(~contains(compact,'''sampled_acceptance'',true'), ...
    '[AF7R27V2] sampled_acceptance=true detected.');
assert(contains(raw,'AF7R27-R10A ADAPTIVE CONTINUOUS SUPPORT COVER'), ...
    '[AF7R27V2] R10-A marker missing.');
assert(contains(raw,'AF7R27-R10B CERTIFIED PROJECTED CORRIDOR COMPRESSION'), ...
    '[AF7R27V2] R10-B marker missing.');
assert(contains(raw,'AF7R27-R10C SMALL CONVEX QP'), ...
    '[AF7R27V2] R10-C marker missing.');
assert(~contains(raw,'r'''''''), ...
    '[AF7R27V2] Python raw-string marker detected.');

fid=fopen(src,'w');
assert(fid>=0,'[AF7R27V2] cannot open production source.');
cleanup=onCleanup(@()fclose(fid));
fwrite(fid,raw,'char');
clear cleanup;

OUT=struct();
OUT.schema='AF7R27_INSTALL_R10_REALTIME_CORE_V2';
OUT.source=src;
OUT.backup=backup;
OUT.installed=true;
OUT.sampled_acceptance=false;
OUT.fixed_depth_acceptance=false;
OUT.deadline_modified=false;
OUT.current_case_constants_hardcoded=false;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',backup);
fprintf(' R10-A adaptive continuous cover         = 1\n');
fprintf(' R10-B projected corridor compression    = 1\n');
fprintf(' R10-C active-set/codegen QP path        = 1\n');
fprintf(' sampled acceptance introduced           = 0\n');
fprintf(' fixed-depth acceptance introduced       = 0\n');
fprintf(' hard deadline modified                  = 0\n');
fprintf('====================================================================================================\n');
end


function [i0,i1]=unique_bounds(raw,a0,a1,label)
p0=strfind(raw,a0);
p1=strfind(raw,a1);
assert(numel(p0)==1,['[AF7R27V2] nonunique start: ' label]);
assert(numel(p1)==1&&p1>p0,['[AF7R27V2] nonunique end: ' label]);
i0=p0;
i1=p1;
end
