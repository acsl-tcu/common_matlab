function OUT = AF7R31_BUILD_SOURCE_BOUND_FLAT_BATCH_MEX_V2(replanner)
% AF7R31_BUILD_SOURCE_BOUND_FLAT_BATCH_MEX_V2
%
% No MATLAB Coder required.
%
% The CURRENT exact nominal flat derivative handles
%   nominal_flat_derivative_fn{1:7}
% are reconstructed as symbolic scalar expressions exactly the same way as
% REPLANNING_SYSTEM.function_handle_to_symbolic_vector() does:
%   @(t) body -> remove elementwise scalar operators -> str2sym.
%
% Each of the 21 scalar expressions is then translated by Symbolic Math
% Toolbox ccode() into C and embedded in one ordinary C++ MEX.  The MEX is
% source-bound by a SHA-256 signature of the exact seven function handles.
%
% This is an implementation accelerator only.  It does NOT modify
% REPLANNING_SYSTEM.m and it creates NO safety certificate.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R31 V2 | BUILD SOURCE-BOUND NOMINAL FLAT BATCH MEX -- NO MATLAB CODER\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] buildOnly=1 | productionModification=0 | safetyAcceptanceChanged=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'), ...
    '[AF7R31V2] input must be REPLANNING_SYSTEM.');

% Symbolic toolbox is already an authority in the current replanner source.
assert(exist('str2sym','file')==2 || exist('str2sym','builtin')==5, ...
    '[AF7R31V2] Symbolic Math Toolbox str2sym is unavailable.');
assert(exist('ccode','file')==2 || exist('ccode','builtin')==5, ...
    '[AF7R31V2] Symbolic Math Toolbox ccode is unavailable.');

F=replanner.nominal_flat_derivative_fn;
assert(iscell(F)&&numel(F)>=7&& ...
    all(cellfun(@(f)isa(f,'function_handle'),F(1:7))), ...
    '[AF7R31V2] nominal flat derivative handles unavailable.');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R31V2] REPLANNING_SYSTEM source unresolved.');
root=fileparts(src);

mexName='af7r31_nominal_flat_batch_mex';
cppPath=fullfile(root,[mexName '.cpp']);
mexPath=fullfile(root,[mexName '.' mexext]);
metaPath=fullfile(root,'AF7R31_FLAT_BATCH_META.mat');

% -------------------------------------------------------------------------
% Reconstruct the 7 x 3 exact scalar symbolic expressions from the current
% generated handles.  Do not assume line / circle / saddle / constant speed.
% -------------------------------------------------------------------------
S=cell(1,7);
JET=cell(1,7);
for r=1:7
    S{r}=func2str(F{r});
    [JET{r},varName]=parse_single_time_symbolic_handle(S{r});
    assert(numel(JET{r})==3, ...
        '[AF7R31V2] derivative order %d must contain exactly 3 components.',r-1);
    if r==1
        commonVar=varName;
    else
        assert(strcmp(varName,commonVar), ...
            '[AF7R31V2] inconsistent anonymous time-variable names.');
    end
end
signature=sha256_text(strjoin(S,char(10)));

% -------------------------------------------------------------------------
% Translate each scalar symbolic expression to one C RHS.
% ccode(scalar) normally emits "t0 = <rhs>;".  Fail closed if it emits a
% form we do not understand; never guess a translation.
% -------------------------------------------------------------------------
rhs=cell(21,1);
kk=0;
for r=1:7
    z=JET{r}(:);
    for j=1:3
        kk=kk+1;
        csrc=strtrim(ccode(z(j)));
        tok=regexp(csrc, ...
            '^\s*[A-Za-z_]\w*\s*=\s*(.*?)\s*;\s*$', ...
            'tokens','once');
        assert(~isempty(tok)&&numel(tok)==1, ...
            ['[AF7R31V2] ccode for flat component %d emitted an unsupported ' ...
             'multi-statement form. No production source was modified.\nccode:\n%s'], ...
            kk,csrc);
        rhs{kk}=char(tok{1});
        assert(~contains(rhs{kk},char(10)) && ~contains(rhs{kk},char(13)), ...
            '[AF7R31V2] multiline scalar C expression rejected fail-closed.');
    end
end

% -------------------------------------------------------------------------
% Write one ordinary MEX source.  This uses the same mex compiler already
% proven by AF7R31 geometry MEX and does not need MATLAB Coder.
% -------------------------------------------------------------------------
fid=fopen(cppPath,'w');
assert(fid>=0,'[AF7R31V2] cannot create generated C++ source.');
cleanupFile=onCleanup(@()safe_fclose(fid));

fprintf(fid,'#define _USE_MATH_DEFINES\n');
fprintf(fid,'#include "mex.h"\n');
fprintf(fid,'#include <math.h>\n');
fprintf(fid,'#include <cmath>\n');
fprintf(fid,'#include <limits>\n\n');
fprintf(fid,'// AF7R31 source-bound flat batch MEX\n');
fprintf(fid,'// source signature: %s\n\n',signature);

for k=1:21
    % Symbolic ccode uses the anonymous symbolic variable name.  Expose that
    % exact variable inside every scalar helper.
    fprintf(fid,'static inline double af7r31_f%02d(double tv) {\n',k);
    fprintf(fid,'    const double %s = tv;\n',commonVar);
    fprintf(fid,'    return %s;\n',rhs{k});
    fprintf(fid,'}\n\n');
end

fprintf(fid,'void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {\n');
fprintf(fid,'    if(nrhs!=1) mexErrMsgIdAndTxt("AF7R31Flat:Arity","One input required.");\n');
fprintf(fid,'    if(nlhs>1) mexErrMsgIdAndTxt("AF7R31Flat:Arity","One output maximum.");\n');
fprintf(fid,'    if(!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]))\n');
fprintf(fid,'        mexErrMsgIdAndTxt("AF7R31Flat:Input","Input must be real double.");\n');
fprintf(fid,'    const mwSize N=mxGetNumberOfElements(prhs[0]);\n');
fprintf(fid,'    const double* tv=mxGetPr(prhs[0]);\n');
fprintf(fid,'    plhs[0]=mxCreateDoubleMatrix(21,N,mxREAL);\n');
fprintf(fid,'    double* Y=mxGetPr(plhs[0]);\n');
fprintf(fid,'    for(mwIndex i=0;i<N;++i) {\n');
fprintf(fid,'        const double x=tv[i];\n');
fprintf(fid,'        if(!std::isfinite(x)) mexErrMsgIdAndTxt("AF7R31Flat:Input","Time must be finite.");\n');
for k=1:21
    fprintf(fid,'        Y[%d + 21*i] = af7r31_f%02d(x);\n',k-1,k);
end
fprintf(fid,'    }\n');
fprintf(fid,'}\n');

fclose(fid);
fid=-1;
clear cleanupFile;

if isfile(mexPath), delete(mexPath); end

fprintf(' reference-source signature              = %s\n',signature);
fprintf(' generated C++                           = %s\n',cppPath);
fprintf(' compiler route                          = ordinary mex (NOT codegen)\n');

mex('-R2018a','-O','-outdir',root,cppPath);
rehash;
assert(isfile(mexPath),'[AF7R31V2] generated flat batch MEX not found after mex build.');

% Exact build metadata used by the unchanged equivalence audit.
meta=struct();
meta.schema='AF7R31_FLAT_BATCH_META_V2_CCODE';
meta.signature=signature;
meta.handle_text=S;
meta.variable_name=commonVar;
meta.generated_cpp=cppPath;
meta.mex_path=mexPath;
meta.matlab_coder_required=false;
meta.symbolic_ccode_used=true;
save(metaPath,'meta');

addpath(root,'-begin');

% Warm both scalar and vector paths.
af7r31_nominal_flat_batch_mex(0);
af7r31_nominal_flat_batch_mex([0 0.125 1.0]);

OUT=struct();
OUT.schema='AF7R31_BUILD_SOURCE_BOUND_FLAT_BATCH_MEX_V2_CCODE';
OUT.signature=signature;
OUT.generated_cpp=cppPath;
OUT.mex_path=mexPath;
OUT.meta_path=metaPath;
OUT.matlab_coder_required=false;
OUT.production_source_modified=false;
OUT.built=true;
assignin('base','AF7R31_BUILD_SOURCE_BOUND_FLAT_BATCH_MEX_V2_OUT',OUT);

fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' MATLAB Coder required                   = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' build/warmup PASS                       = 1\n');
fprintf('====================================================================================================\n');
end


function [z,varName]=parse_single_time_symbolic_handle(fstr)
% Mirrors the current REPLANNING_SYSTEM symbolic parser, but does not call a
% class-private/helper method.
s=strtrim(char(fstr));
tok=regexp(s,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
assert(~isempty(tok)&&numel(tok)==2, ...
    '[AF7R31V2] nominal flat handle must be a one-variable anonymous function.');
varName=char(tok{1});
body=strtrim(char(tok{2}));
body=strrep(body,'.^','^');
body=strrep(body,'.*','*');
body=strrep(body,'./','/');
z=str2sym(body);
z=z(:);

% Fail closed if an unexpected symbolic variable leaked into the expression.
sv=symvar(z);
for k=1:numel(sv)
    assert(strcmp(char(sv(k)),varName), ...
        '[AF7R31V2] unexpected symbolic variable %s in flat expression.',char(sv(k)));
end
end


function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function safe_fclose(fid)
if isnumeric(fid)&&isscalar(fid)&&fid>=0
    try, fclose(fid); catch, end
end
end
