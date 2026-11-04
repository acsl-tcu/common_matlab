// Temporary Windows-only MATLAB timing probe for 6Ckidou audits.
// Gives monotonic elapsed milliseconds, CURRENT THREAD CPU milliseconds,
// and the Windows thread ID. It neither reads Flight data nor simulates.
// This is a diagnostic, not a real-time scheduler or WCET certificate.
#define NOMINMAX
#include "mex.h"
#ifdef _WIN32
# include <windows.h>
#else
# error This temporary timing probe is Windows-only.
#endif
#include <stdint.h>
static double filetime_ms(const FILETIME& f) {
    const uint64_t t=(uint64_t(f.dwHighDateTime)<<32)|uint64_t(f.dwLowDateTime);
    return double(t)*1.e-4;
}
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray*[]) {
    if(nrhs!=0 || nlhs!=1) {
        mexErrMsgIdAndTxt("TRIGGER_THREAD_CPU_WALL_NATIVE:Usage","Call with no arguments and one output.");
    }
    LARGE_INTEGER counter,freq;
    FILETIME creation,exit_time,kernel,user;
    if(!QueryPerformanceCounter(&counter) || !QueryPerformanceFrequency(&freq) ||
       !GetThreadTimes(GetCurrentThread(),&creation,&exit_time,&kernel,&user) ||
       freq.QuadPart<=0) {
        mexErrMsgIdAndTxt("TRIGGER_THREAD_CPU_WALL_NATIVE:Clock","Win32 timing source unavailable.");
    }
    ULONG64 cycles=0;
    if(!QueryThreadCycleTime(GetCurrentThread(),&cycles) ||
       cycles>9007199254740992ULL) {
        mexErrMsgIdAndTxt("TRIGGER_THREAD_CPU_WALL_NATIVE:Cycles",
            "Thread cycle count is unavailable or not exactly representable.");
    }
    plhs[0]=mxCreateDoubleMatrix(1,4,mxREAL);
    double *p=mxGetPr(plhs[0]);
    p[0]=1000.0*double(counter.QuadPart)/double(freq.QuadPart);
    p[1]=filetime_ms(kernel)+filetime_ms(user);
    p[2]=double(GetCurrentThreadId());
    p[3]=double(cycles); // RAW thread CPU cycles, NOT milliseconds (CPU frequency can vary).
}
