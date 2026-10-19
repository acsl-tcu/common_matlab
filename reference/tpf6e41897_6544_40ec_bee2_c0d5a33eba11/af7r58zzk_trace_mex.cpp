#include "mex.h"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdint.h>
#include <string.h>

struct Event { uint64_t seq, code, qpc, thr, proc, aux; };
static const size_t CAP=1024;
static Event ringbuf[CAP];
static size_t head=0, countEv=0;
static uint64_t qpc_now(){ LARGE_INTEGER x; QueryPerformanceCounter(&x); return (uint64_t)x.QuadPart; }
static uint64_t qpc_freq(){ LARGE_INTEGER x; QueryPerformanceFrequency(&x); return (uint64_t)x.QuadPart; }
static uint64_t thread_cycles(){ ULONG64 x=0; QueryThreadCycleTime(GetCurrentThread(),&x); return (uint64_t)x; }
static uint64_t process_cycles(){ ULONG64 x=0; QueryProcessCycleTime(GetCurrentProcess(),&x); return (uint64_t)x; }
static uint64_t scalar_u64(const mxArray* a){ if(mxIsUint64(a)&&mxGetNumberOfElements(a)==1) return *((uint64_t*)mxGetData(a)); return (uint64_t)mxGetScalar(a); }
static void push(uint64_t seq,uint64_t code,uint64_t aux){ Event e; e.seq=seq; e.code=code; e.qpc=qpc_now(); e.thr=thread_cycles(); e.proc=process_cycles(); e.aux=aux; ringbuf[head]=e; head=(head+1)%CAP; if(countEv<CAP) ++countEv; }
static mxArray* u64row4(uint64_t a,uint64_t b,uint64_t c,uint64_t d){ mxArray* y=mxCreateNumericMatrix(1,4,mxUINT64_CLASS,mxREAL); uint64_t* p=(uint64_t*)mxGetData(y); p[0]=a; p[1]=b; p[2]=c; p[3]=d; return y; }
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
 if(nrhs<1||!mxIsChar(prhs[0])) mexErrMsgIdAndTxt("AF7R58ZZK:Args","command required");
 char cmd[32]; mxGetString(prhs[0],cmd,sizeof(cmd));
 if(strcmp(cmd,"reset")==0){ head=0; countEv=0; memset(ringbuf,0,sizeof(ringbuf)); return; }
 if(strcmp(cmd,"mark")==0){ if(nrhs<4) mexErrMsgIdAndTxt("AF7R58ZZK:Args","mark needs seq,code,aux"); push(scalar_u64(prhs[1]),scalar_u64(prhs[2]),scalar_u64(prhs[3])); return; }
 if(strcmp(cmd,"stamp")==0){ uint64_t a=qpc_now(); uint64_t b=thread_cycles(); uint64_t c=process_cycles(); uint64_t d=qpc_freq(); if(nlhs>0) plhs[0]=u64row4(a,b,c,d); return; }
 if(strcmp(cmd,"dump")==0){ const char* fn[]={"freq","events"}; mxArray* st=mxCreateStructMatrix(1,1,2,fn); mxArray* fq=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL); *((uint64_t*)mxGetData(fq))=qpc_freq(); mxSetField(st,0,"freq",fq); mwSize N=(mwSize)countEv; mxArray* ev=mxCreateNumericMatrix(N,6,mxUINT64_CLASS,mxREAL); uint64_t* p=(uint64_t*)mxGetData(ev); size_t start=(head+CAP-countEv)%CAP; for(mwSize r=0;r<N;++r){ const Event& e=ringbuf[(start+r)%CAP]; p[r+N*0]=e.seq; p[r+N*1]=e.code; p[r+N*2]=e.qpc; p[r+N*3]=e.thr; p[r+N*4]=e.proc; p[r+N*5]=e.aux; } mxSetField(st,0,"events",ev); if(nlhs>0) plhs[0]=st; else mxDestroyArray(st); return; }
 mexErrMsgIdAndTxt("AF7R58ZZK:Command","unknown command");
}
