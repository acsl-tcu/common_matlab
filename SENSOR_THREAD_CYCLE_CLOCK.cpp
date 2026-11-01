// SENSOR_THREAD_CYCLE_CLOCK.cpp
// Temporary native spike diagnostic for Windows MATLAB (R2018a API).
// Measures QPC elapsed milliseconds and executing-thread cycles; NOT a hard real-time proof.
// Build manually before Flight: mex('-R2018a','-outdir',pwd,'SENSOR_THREAD_CYCLE_CLOCK.cpp')
#include "mex.h"
#ifndef _WIN32
#error SENSOR_THREAD_CYCLE_CLOCK requires Windows.
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <stdint.h>
#include <string.h>

static uint64_t qpc_now() {
    LARGE_INTEGER t;
    if (!QueryPerformanceCounter(&t))
        mexErrMsgIdAndTxt("SENSOR_THREAD_CYCLE_CLOCK:QPC", "QueryPerformanceCounter failed.");
    return (uint64_t)t.QuadPart;
}
static uint64_t cycle_now() {
    ULONG64 c=0;
    if (!QueryThreadCycleTime(GetCurrentThread(), &c))
        mexErrMsgIdAndTxt("SENSOR_THREAD_CYCLE_CLOCK:Cycles", "QueryThreadCycleTime failed.");
    return (uint64_t)c;
}
static uint64_t frequency_now() {
    LARGE_INTEGER f;
    if (!QueryPerformanceFrequency(&f) || f.QuadPart<=0)
        mexErrMsgIdAndTxt("SENSOR_THREAD_CYCLE_CLOCK:Frequency", "QueryPerformanceFrequency failed.");
    return (uint64_t)f.QuadPart;
}
static thread_local bool active=false;
static thread_local uint64_t start_qpc=0;
static thread_local uint64_t start_cycles=0;
static thread_local DWORD start_thread_id=0;

static void begin_measurement() {
    if(active)
        mexErrMsgIdAndTxt("SENSOR_THREAD_CYCLE_CLOCK:Nested", "Nested begin is not supported.");
    start_thread_id=GetCurrentThreadId();
    start_qpc=qpc_now();
    start_cycles=cycle_now();
    active=true;
}
static void end_measurement(mxArray **out) {
    if(!active || GetCurrentThreadId()!=start_thread_id)
        mexErrMsgIdAndTxt("SENSOR_THREAD_CYCLE_CLOCK:Thread", "No matching begin on this MATLAB thread.");
    const uint64_t end_cycles=cycle_now();
    const uint64_t end_qpc=qpc_now();
    active=false;
    *out=mxCreateDoubleMatrix(1,2,mxREAL);
    double *p=mxGetPr(*out);
    p[0]=1000.0*(double)(end_qpc-start_qpc)/(double)frequency_now();
    p[1]=(double)(end_cycles-start_cycles);
}
static void selftest(mxArray **out) {
    const uint64_t freq=frequency_now();
    const uint64_t s_q0=qpc_now();
    const uint64_t s_c0=cycle_now();
    Sleep(80); // The native current thread is not runnable while sleeping.
    const uint64_t s_c1=cycle_now();
    const uint64_t s_q1=qpc_now();
    volatile uint64_t sink=1;
    const uint64_t b_q0=qpc_now();
    const uint64_t b_c0=cycle_now();
    uint64_t q=b_q0;
    const uint64_t target=(freq/1000)*60;
    while(q-b_q0<target) {
        for(int i=0;i<512;i++) sink=sink*2862933555777941757ULL+3037000493ULL;
        q=qpc_now();
    }
    const uint64_t b_c1=cycle_now();
    const uint64_t b_q1=qpc_now();
    (void)sink;
    *out=mxCreateDoubleMatrix(1,6,mxREAL);
    double *p=mxGetPr(*out);
    p[0]=1000.0*(double)(s_q1-s_q0)/(double)freq;
    p[1]=(double)(s_c1-s_c0);
    p[2]=1000.0*(double)(b_q1-b_q0)/(double)freq;
    p[3]=(double)(b_c1-b_c0);
    p[4]=(double)freq;
    p[5]=(p[2]>0)?p[3]/p[2]:mxGetNaN(); // CPU cycles per busy ms.
}
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {
    if(nrhs!=1 || !mxIsChar(prhs[0]))
        mexErrMsgIdAndTxt("SENSOR_THREAD_CYCLE_CLOCK:Arguments", "Use 'begin', 'end', or 'selftest'.");
    char cmd[24]={0};
    if(mxGetString(prhs[0],cmd,sizeof(cmd))!=0)
        mexErrMsgIdAndTxt("SENSOR_THREAD_CYCLE_CLOCK:Arguments", "Invalid command.");
    if(strcmp(cmd,"begin")==0 && nlhs==0) {begin_measurement();return;}
    if(strcmp(cmd,"end")==0 && nlhs==1) {end_measurement(&plhs[0]);return;}
    if(strcmp(cmd,"selftest")==0 && nlhs==1) {selftest(&plhs[0]);return;}
    mexErrMsgIdAndTxt("SENSOR_THREAD_CYCLE_CLOCK:Arguments", "Use no output for begin; one output for end/selftest.");
}
