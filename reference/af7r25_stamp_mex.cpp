#define _WIN32_WINNT 0x0600
#include "mex.h"
#include <windows.h>
#include <cstdint>

static void req(bool ok,const char* msg){
    if(!ok) mexErrMsgIdAndTxt("AF7R25:StampMEX","%s",msg);
}

void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    (void)prhs;
    req(nrhs==0,"No inputs expected.");
    req(nlhs<=1,"At most one output.");
    LARGE_INTEGER qpc,freq;
    ULONG64 tc=0, pc=0;
    req(QueryPerformanceCounter(&qpc)!=0,"QueryPerformanceCounter failed.");
    req(QueryPerformanceFrequency(&freq)!=0,"QueryPerformanceFrequency failed.");
    req(QueryThreadCycleTime(GetCurrentThread(),&tc)!=0,"QueryThreadCycleTime failed.");
    req(QueryProcessCycleTime(GetCurrentProcess(),&pc)!=0,"QueryProcessCycleTime failed.");
    if(nlhs==0) return;
    plhs[0]=mxCreateNumericMatrix(1,4,mxUINT64_CLASS,mxREAL);
    uint64_T* o=(uint64_T*)mxGetData(plhs[0]);
    o[0]=(uint64_T)qpc.QuadPart;
    o[1]=(uint64_T)freq.QuadPart;
    o[2]=(uint64_T)tc;
    o[3]=(uint64_T)pc;
}
