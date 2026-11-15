function M=reference_avoidance_mex_ensure(action)
%REFERENCE_AVOIDANCE_MEX_ENSURE Auto-build missing/stale C-mode MEX.
% Call during PREPARATION before the 25ms Flight loop. The official tick
% also calls this if the compiled file has disappeared. In that exceptional
% case a synchronous rebuild may block for seconds and MISSES the deadline.
% Compilation/parity failure always falls back to pure MATLAB geometry.
% action: 'prepare' (default), 'runtime', 'force', 'probe'.
if nargin<1 || isempty(action),action='prepare';end
action=lower(char(action));
if ~ismember(action,{'prepare','runtime','ensure','force','probe'})
    error('RefAvoidance:MEXAction','Use prepare, runtime, force, or probe');
end
persistent failedKey verifiedKey
if isempty(failedKey), failedKey='';end
if isempty(verifiedKey), verifiedKey='';end
folder=fileparts(mfilename('fullpath'));
source=fullfile(folder,'SOURCE','wp_six_trigger_C_chain_numeric_mex_v8.c');
outdir=fullfile(folder,'BIN');
name='wp_six_trigger_C_chain_numeric_mex_v8';
outfile=fullfile(outdir,[name '.' mexext]);
M=struct('status','MATLAB_FALLBACK','available',false,'rebuilt',false, ...
    'source',source,'binary',outfile,'errorText','','parityTested',false, ...
    'parityCases',0,'buildMs',0,'requiresPreflightFor25ms',true);
if ~isfile(source)
    M.status='SOURCE_MISSING_FALLBACK';M.errorText=['Missing C source: ' source];return
end
if ~isfolder(outdir)
    try,mkdir(outdir);catch ME,M.status='BIN_UNWRITABLE_FALLBACK';M.errorText=ME.message;return,end
end
s=dir(source);
if isfile(outfile),b=dir(outfile);else,b=[];end
key=sprintf('%s|%.12g|%.12g|%s',source,s.datenum,s.bytes,mexext);
needsBuild=strcmp(action,'force') || isempty(b) || b.datenum<s.datenum;
if strcmp(action,'probe')
    if needsBuild,M.status='NEEDS_BUILD';else,M.status='BUILT_NOT_VERIFIED';end
    return
end
% Suppress repeated compiles when a compiler is unavailable. Explicit
% 'force' retries, or edit the source file (changes the key) to retry.
if needsBuild && strcmp(action,'runtime') && strcmp(failedKey,key)
    M.status='PREVIOUS_BUILD_FAILED_FALLBACK';
    M.errorText='No repeated compile in 25ms loop; use force after configuring compiler';
    return
end
if needsBuild
    clock=tic;
    try
        clear('wp_six_trigger_C_chain_numeric_mex_v8');
        mex('-R2018a','-outdir',outdir,'-output',name,source);
        M.buildMs=1000*toc(clock);
        M.rebuilt=true;
        clear('wp_six_trigger_C_chain_numeric_mex_v8');
        rehash;
        failedKey='';verifiedKey='';
    catch ME
        M.buildMs=1000*toc(clock);failedKey=key;
        M.status='BUILD_FAILED_FALLBACK';
        M.errorText=[ME.identifier ': ' ME.message];
        return
    end
end
% Always put this trigger's own native bin ahead of legacy MEX folders.
addpath(outdir,'-begin');rehash;
actual=which(name);
if ~strcmpi(actual,outfile) || exist(name,'file')~=3
    M.status='WRONG_OR_MISSING_MEX_FALLBACK';
    M.errorText=['Resolved native file: ' actual];return
end
if ~strcmp(verifiedKey,key)
    V=reference_avoidance_mex_verify();
    M.parityTested=true;M.parityCases=V.tested;
    if ~strcmp(V.status,'PASS_NUMERIC_PARITY')
        M.status='PARITY_FAILED_FALLBACK';M.errorText=V.errorText;
        failedKey=key;return
    end
    verifiedKey=key;
end
M.status='READY_VERIFIED';M.available=true;
end
