function OUT=AF7R58ZZY_FINAL_RUN_CANONICAL()
root=find_root();
runner=fullfile(root,'run_flight_headless.m');
source=fullfile(root,'reference','REPLANNING_SYSTEM.m');
assert(isfile(runner)&&isfile(source),'[FINAL RUN] canonical files missing.');

% Remove only the previous replanner object before reloading the integrated class.
try
    evalin('base','clear replanner_instance');
catch
end
try
    clear REPLANNING_SYSTEM;
catch
end
rehash;
cd(root); addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin');

K=fileread(runner);
assert(count_occ(K,'REPLANNING_SYSTEM(agent,nominal_ref,replan_opts)')==1, ...
    '[FINAL RUN] runner is not bound directly to REPLANNING_SYSTEM.');
assert(isempty(regexp(K,'REPLANNING_SYSTEM_R\d+_[A-Za-z0-9_]*\(agent,nominal_ref,replan_opts\)','once')), ...
    '[FINAL RUN] historical replanner class binding remains.');

AF7R58ZZY_FINAL_SELFTEST(source,runner);

outdir=fullfile(root,'generated','AF7R58ZZY_FINAL');
if ~isfolder(outdir), mkdir(outdir); end
assignin('base','AF7R58ZZY_FINAL_OUTPUT_DIR',outdir);
assignin('base','AF7R58ZZY_FINAL_ABSOLUTE_RUNNER',runner);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY FINAL | CANONICAL FLIGHT LAUNCH\n');
fprintf('====================================================================================================\n');
fprintf('runner = %s\n',runner);
fprintf('source = %s\n',source);
fprintf('output = %s\n',outdir);
evalin('base','run(AF7R58ZZY_FINAL_ABSOLUTE_RUNNER);');

OUT=struct('closed',true,'runner',runner,'source',source,'output_dir',outdir);
assignin('base','AF7R58ZZY_FINAL_RUN_CANONICAL_OUT',OUT);
end

function root=find_root()
p=fileparts(mfilename('fullpath'));
for k=1:6
    if isfile(fullfile(p,'run_flight_headless.m')) && isfolder(fullfile(p,'reference'))
        root=p; return;
    end
    q=fileparts(p); if isempty(q)||strcmp(q,p),break;end; p=q;
end
error('[FINAL RUN] common_matlab root not found.');
end
function n=count_occ(s,p), n=numel(strfind(s,p)); end
