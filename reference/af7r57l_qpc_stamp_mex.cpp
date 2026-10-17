
#include "mex.h"
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <chrono>
#endif
#include <cstdint>

static mxArray* makeStamp() {
    const char* fn[]={"qpc_ticks","qpc_frequency","thread_cycles","process_cycles",
                     "qpc_available","thread_cycles_available","process_cycles_available"};
    mxArray* s=mxCreateStructMatrix(1,1,7,fn);

#ifdef _WIN32
    LARGE_INTEGER q{},f{};
    BOOL oq=QueryPerformanceCounter(&q);
    BOOL of=QueryPerformanceFrequency(&f);

    ULONG64 tc=0,pc=0;
    BOOL ot=QueryThreadCycleTime(GetCurrentThread(),&tc);
    BOOL op=QueryProcessCycleTime(GetCurrentProcess(),&pc);

    mxSetField(s,0,"qpc_ticks",mxCreateDoubleScalar(oq?(double)q.QuadPart:mxGetNaN()));
    mxSetField(s,0,"qpc_frequency",mxCreateDoubleScalar(of?(double)f.QuadPart:mxGetNaN()));
    mxSetField(s,0,"thread_cycles",mxCreateDoubleScalar(ot?(double)tc:mxGetNaN()));
    mxSetField(s,0,"process_cycles",mxCreateDoubleScalar(op?(double)pc:mxGetNaN()));
    mxSetField(s,0,"qpc_available",mxCreateLogicalScalar(oq&&of));
    mxSetField(s,0,"thread_cycles_available",mxCreateLogicalScalar(ot));
    mxSetField(s,0,"process_cycles_available",mxCreateLogicalScalar(op));
#else
    using clock=std::chrono::steady_clock;
    auto t=clock::now().time_since_epoch();
    auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(t).count();
    mxSetField(s,0,"qpc_ticks",mxCreateDoubleScalar((double)ns));
    mxSetField(s,0,"qpc_frequency",mxCreateDoubleScalar(1e9));
    mxSetField(s,0,"thread_cycles",mxCreateDoubleScalar(mxGetNaN()));
    mxSetField(s,0,"process_cycles",mxCreateDoubleScalar(mxGetNaN()));
    mxSetField(s,0,"qpc_available",mxCreateLogicalScalar(true));
    mxSetField(s,0,"thread_cycles_available",mxCreateLogicalScalar(false));
    mxSetField(s,0,"process_cycles_available",mxCreateLogicalScalar(false));
#endif
    return s;
}

void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]) {
    (void)prhs;
    if(nrhs!=0) mexErrMsgIdAndTxt("AF7R57L:Input","No inputs expected.");
    if(nlhs>1) mexErrMsgIdAndTxt("AF7R57L:Output","At most one output.");
    plhs[0]=makeStamp();
}
