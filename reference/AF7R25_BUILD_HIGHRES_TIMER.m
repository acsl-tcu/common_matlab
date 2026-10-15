function OUT = AF7R25_BUILD_HIGHRES_TIMER()
% AF7R25_BUILD_HIGHRES_TIMER
% =========================================================================
% Build and self-test a tiny Windows high-resolution counter MEX:
%   - QueryPerformanceCounter      : wall-time interval authority
%   - QueryThreadCycleTime         : current-thread work counter
%   - QueryProcessCycleTime        : all-MATLAB-process threads work counter
%
% IMPORTANT:
%   Thread/process cycles are WORK counters only. They are NOT converted
%   to elapsed time. QPC is the only wall-time counter.
% =========================================================================

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R25 | BUILD HIGH-RESOLUTION WINDOWS TIMER MEX\n');
fprintf('====================================================================================================\n');

src = which('REPLANNING_SYSTEM');
assert(~isempty(src) && isfile(src),'[AF7R25] REPLANNING_SYSTEM.m not found.');
root = fileparts(src);
name = 'af7r25_stamp_mex';
cpp = fullfile(root,[name '.cpp']);

lines = { ...
'#define _WIN32_WINNT 0x0600', ...
'#include "mex.h"', ...
'#include <windows.h>', ...
'#include <cstdint>', ...
'', ...
'static void req(bool ok,const char* msg){', ...
'    if(!ok) mexErrMsgIdAndTxt("AF7R25:StampMEX","%s",msg);', ...
'}', ...
'', ...
'void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){', ...
'    (void)prhs;', ...
'    req(nrhs==0,"No inputs expected.");', ...
'    req(nlhs<=1,"At most one output.");', ...
'    LARGE_INTEGER qpc,freq;', ...
'    ULONG64 tc=0, pc=0;', ...
'    req(QueryPerformanceCounter(&qpc)!=0,"QueryPerformanceCounter failed.");', ...
'    req(QueryPerformanceFrequency(&freq)!=0,"QueryPerformanceFrequency failed.");', ...
'    req(QueryThreadCycleTime(GetCurrentThread(),&tc)!=0,"QueryThreadCycleTime failed.");', ...
'    req(QueryProcessCycleTime(GetCurrentProcess(),&pc)!=0,"QueryProcessCycleTime failed.");', ...
'    if(nlhs==0) return;', ...
'    plhs[0]=mxCreateNumericMatrix(1,4,mxUINT64_CLASS,mxREAL);', ...
'    uint64_T* o=(uint64_T*)mxGetData(plhs[0]);', ...
'    o[0]=(uint64_T)qpc.QuadPart;', ...
'    o[1]=(uint64_T)freq.QuadPart;', ...
'    o[2]=(uint64_T)tc;', ...
'    o[3]=(uint64_T)pc;', ...
'}' ...
};

txt=sprintf('%s\n',lines{:});
need=true;
if isfile(cpp)
    need=~strcmp(fileread(cpp),txt);
end
if need
    fid=fopen(cpp,'w');
    assert(fid>=0,'[AF7R25] Cannot write %s',cpp);
    c=onCleanup(@()fclose(fid));
    fprintf(fid,'%s',txt);
    clear c;
end

mexfile=fullfile(root,[name '.' mexext]);
if need || ~isfile(mexfile)
    fprintf(' building                               = %s\n',cpp);
    try, clear(name); catch, end
    mex('-R2018a','-O','-outdir',root,cpp);
    rehash;
end
assert(isfile(mexfile),'[AF7R25] MEX build failed.');

addpath(root,'-begin');

% Self-test monotonicity and pair overhead.
N=5000;
w=nan(N,1);
td=nan(N,1);
pd=nan(N,1);
for k=1:N
    a=af7r25_stamp_mex();
    b=af7r25_stamp_mex();
    assert(all(b([1 3 4])>=a([1 3 4])),'[AF7R25] Non-monotone counter.');
    w(k)=1e3*double(b(1)-a(1))/double(a(2));
    td(k)=double(b(3)-a(3));
    pd(k)=double(b(4)-a(4));
end

OUT=struct();
OUT.schema='AF7R25_HIGHRES_TIMER_V1';
OUT.mex_path=mexfile;
OUT.qpc_pair_overhead_ms=struct( ...
    'min',min(w),'median',median(w),'p95',pct(w,95),'max',max(w));
OUT.thread_pair_cycles_median=median(td);
OUT.process_pair_cycles_median=median(pd);
OUT.wall_authority='QUERY_PERFORMANCE_COUNTER';
OUT.work_counters='THREAD_AND_PROCESS_CYCLE_COUNTS_NOT_TIME';

fprintf(' MEX                                     = %s\n',mexfile);
fprintf(' stamp-pair wall min/med/p95/max [ms]    = %.6g / %.6g / %.6g / %.6g\n', ...
    min(w),median(w),pct(w,95),max(w));
fprintf(' median stamp-pair thread cycles         = %.6g\n',median(td));
fprintf(' median stamp-pair process cycles        = %.6g\n',median(pd));
fprintf(' wall authority                          = QueryPerformanceCounter\n');
fprintf(' cycle counters converted to ms          = 0\n');
fprintf('====================================================================================================\n');

assignin('base','AF7R25_HIGHRES_TIMER_BUILD_OUT',OUT);
end

function y=pct(x,p)
x=sort(double(x(:)));
x=x(isfinite(x));
if isempty(x), y=NaN; return; end
u=1+(numel(x)-1)*p/100;
i=floor(u); j=ceil(u);
if i==j, y=x(i); else, y=x(i)+(u-i)*(x(j)-x(i)); end
end
