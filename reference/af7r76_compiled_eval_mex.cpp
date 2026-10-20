#include "mex.h"
#include <cmath>
#include <cstddef>

// AF7R58ZZY-R23 strict-FP runtime evaluator for a PRECOMPILED canonical
// correction packet. It changes representation only, never the trajectory.
//
// op=1: Y = mex(1,tvec,baseXD,packet)
//       baseXD is MxN (M>=28). Corrections r=0..6 are added to the exact
//       canonical slots in ascending correction-group order.
// op=2: [P,A] = mex(2,tvec,Pbase,Abase,packet)
//       Pbase/Abase are 3xN. r=0/r=2 corrections are added in the same order.
//
// Runtime work deliberately contains NO control-point derivative creation,
// NO cell traversal, and NO correction-struct traversal. Those operations are
// closed once by af7r76_compile_correction_packet_R23.

static void fail(const char* id,const char* msg){ mexErrMsgIdAndTxt(id,"%s",msg); }
static void req(bool ok,const char* msg){ if(!ok) fail("AF7R76:MEX",msg); }
static const mxArray* fld(const mxArray* s,const char* n){
    const mxArray* a=mxGetField(s,0,n);
    if(!a) mexErrMsgIdAndTxt("AF7R76:Field","Missing packet field: %s",n);
    return a;
}
static double scalar(const mxArray* a,const char* n){
    if(!(mxIsDouble(a)&&!mxIsComplex(a)&&mxGetNumberOfElements(a)==1))
        mexErrMsgIdAndTxt("AF7R76:Scalar","%s must be a real double scalar.",n);
    const double v=mxGetScalar(a);
    if(!std::isfinite(v)) mexErrMsgIdAndTxt("AF7R76:Scalar","%s must be finite.",n);
    return v;
}
static void requireRealDouble(const mxArray* a,const char* n){
    if(!(mxIsDouble(a)&&!mxIsComplex(a)))
        mexErrMsgIdAndTxt("AF7R76:Type","%s must be real double.",n);
}

struct Packet {
    mwSize G;
    mwSize S;
    const double* gt0;
    const double* gt1;
    const double* gf;
    const double* gc;
    const double* s0;
    const double* s1;
    const double* dcp;
};

static Packet parsePacket(const mxArray* p){
    req(mxIsStruct(p)&&mxGetNumberOfElements(p)==1,"packet must be a scalar struct.");
    const mxArray* aG=fld(p,"group_count");
    const mxArray* aS=fld(p,"segment_count");
    const double Gd=scalar(aG,"group_count");
    const double Sd=scalar(aS,"segment_count");
    req(Gd>=0.0&&Sd>=0.0&&std::floor(Gd)==Gd&&std::floor(Sd)==Sd,"invalid packet counts.");
    Packet P;
    P.G=(mwSize)Gd; P.S=(mwSize)Sd;

    const mxArray* a0=fld(p,"group_t0");
    const mxArray* a1=fld(p,"group_t1");
    const mxArray* af=fld(p,"group_first");
    const mxArray* ac=fld(p,"group_segments");
    const mxArray* s0=fld(p,"segment_t0");
    const mxArray* s1=fld(p,"segment_t1");
    const mxArray* d =fld(p,"derivative_control_points");
    requireRealDouble(a0,"group_t0"); requireRealDouble(a1,"group_t1");
    requireRealDouble(af,"group_first"); requireRealDouble(ac,"group_segments");
    requireRealDouble(s0,"segment_t0"); requireRealDouble(s1,"segment_t1");
    requireRealDouble(d,"derivative_control_points");
    req(mxGetNumberOfElements(a0)==P.G&&mxGetNumberOfElements(a1)==P.G&&
        mxGetNumberOfElements(af)==P.G&&mxGetNumberOfElements(ac)==P.G,
        "group-array length mismatch.");
    req(mxGetNumberOfElements(s0)==P.S&&mxGetNumberOfElements(s1)==P.S,
        "segment-array length mismatch.");
    const mwSize nd=mxGetNumberOfDimensions(d);
    const mwSize* dd=mxGetDimensions(d);
    req(nd==4&&dd[0]==3&&dd[1]==16&&dd[2]==7&&dd[3]==P.S,
        "derivative_control_points must be 3x16x7xS.");
    P.gt0=mxGetDoubles(a0); P.gt1=mxGetDoubles(a1);
    P.gf=mxGetDoubles(af); P.gc=mxGetDoubles(ac);
    P.s0=mxGetDoubles(s0); P.s1=mxGetDoubles(s1); P.dcp=mxGetDoubles(d);

    for(mwSize g=0;g<P.G;++g){
        req(std::isfinite(P.gt0[g])&&std::isfinite(P.gt1[g])&&P.gt1[g]>P.gt0[g],"invalid group support.");
        const double fd=P.gf[g], cd=P.gc[g];
        req(std::isfinite(fd)&&std::isfinite(cd)&&fd>=1.0&&cd>=1.0&&
            std::floor(fd)==fd&&std::floor(cd)==cd,"invalid group segment index/count.");
        const mwSize first=(mwSize)fd-1, count=(mwSize)cd;
        req(first<P.S&&first+count<=P.S,"group segment range outside packet.");
    }
    for(mwSize s=0;s<P.S;++s){
        req(std::isfinite(P.s0[s])&&std::isfinite(P.s1[s])&&P.s1[s]>P.s0[s],"invalid segment support.");
    }
    return P;
}

static mwSize activeSegment(const Packet& P,mwSize g,double t,bool& active){
    active=false;
    if(!(t>P.gt0[g] && t<P.gt1[g])) return 0;
    const mwSize first=(mwSize)P.gf[g]-1;
    const mwSize count=(mwSize)P.gc[g];
    mwSize local=0;
    while(local<count && !(t<=P.s1[first+local])) ++local;
    req(local<count,"active correction time outside segment authority.");
    if(t<P.s0[first+local]){
        if(local==0) return 0;
        --local;
    }
    active=true;
    return first+local;
}

static void decasteljau(const Packet& P,mwSize seg,mwSize r,double u,double out[3]){
    const mwSize cols=16-r;
    req(r<=6&&cols>=10,"invalid derivative order.");
    double W[48];
    const mwSize base=3*16*r + 3*16*7*seg;
    for(mwSize j=0;j<cols;++j){
        for(mwSize i=0;i<3;++i) W[i+3*j]=P.dcp[base+i+3*j];
    }
    const volatile double omu=1.0-u;
    mwSize c=cols;
    while(c>1){
        const mwSize nc=c-1;
        for(mwSize j=0;j<nc;++j){
            for(mwSize i=0;i<3;++i){
                const volatile double a=omu*W[i+3*j];
                const volatile double b=u*W[i+3*(j+1)];
                const volatile double z=a+b;
                W[i+3*j]=z;
            }
        }
        c=nc;
    }
    out[0]=W[0]; out[1]=W[1]; out[2]=W[2];
}

static void addOrder(const Packet& P,mwSize seg,double t,mwSize r,double* dst){
    const double T=P.s1[seg]-P.s0[seg];
    volatile double u=(t-P.s0[seg])/T;
    if(u<0.0) u=0.0;
    if(u>1.0) u=1.0;
    double y[3]; decasteljau(P,seg,r,(double)u,y);
    for(mwSize i=0;i<3;++i){
        const volatile double z=dst[i]+y[i];
        dst[i]=z;
    }
}

void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    req(nrhs>=1,"operation code required.");
    const double op=scalar(prhs[0],"op");

    if(op==1.0){
        req(nrhs==4&&nlhs==1,"op=1 requires tvec, baseXD, packet and one output.");
        requireRealDouble(prhs[1],"tvec"); requireRealDouble(prhs[2],"baseXD");
        const mwSize N=mxGetNumberOfElements(prhs[1]);
        req(N>=1,"tvec must be nonempty.");
        const double* t=mxGetDoubles(prhs[1]);
        const mwSize M=mxGetM(prhs[2]);
        const mwSize Nb=mxGetN(prhs[2]);
        req(M>=28&&Nb==N,"baseXD must be MxN with M>=28.");
        const double* b=mxGetDoubles(prhs[2]);
        Packet P=parsePacket(prhs[3]);
        plhs[0]=mxDuplicateArray(prhs[2]);
        double* y=mxGetDoubles(plhs[0]);
        for(mwSize n=0;n<N;++n){
            req(std::isfinite(t[n]),"tvec contains nonfinite values.");
            for(mwSize g=0;g<P.G;++g){
                bool active=false; const mwSize seg=activeSegment(P,g,t[n],active);
                if(!active) continue;
                for(mwSize r=0;r<=6;++r){
                    double* dst=&y[M*n+4*r];
                    addOrder(P,seg,t[n],r,dst);
                }
            }
        }
        return;
    }

    if(op==2.0){
        req(nrhs==5&&nlhs==2,"op=2 requires tvec, Pbase, Abase, packet and two outputs.");
        requireRealDouble(prhs[1],"tvec"); requireRealDouble(prhs[2],"Pbase"); requireRealDouble(prhs[3],"Abase");
        const mwSize N=mxGetNumberOfElements(prhs[1]);
        req(N>=1,"tvec must be nonempty.");
        req(mxGetM(prhs[2])==3&&mxGetN(prhs[2])==N,"Pbase must be 3xN.");
        req(mxGetM(prhs[3])==3&&mxGetN(prhs[3])==N,"Abase must be 3xN.");
        const double* t=mxGetDoubles(prhs[1]);
        Packet P=parsePacket(prhs[4]);
        plhs[0]=mxDuplicateArray(prhs[2]);
        plhs[1]=mxDuplicateArray(prhs[3]);
        double* po=mxGetDoubles(plhs[0]); double* ao=mxGetDoubles(plhs[1]);
        for(mwSize n=0;n<N;++n){
            req(std::isfinite(t[n]),"tvec contains nonfinite values.");
            for(mwSize g=0;g<P.G;++g){
                bool active=false; const mwSize seg=activeSegment(P,g,t[n],active);
                if(!active) continue;
                addOrder(P,seg,t[n],0,&po[3*n]);
                addOrder(P,seg,t[n],2,&ao[3*n]);
            }
        }
        return;
    }

    fail("AF7R76:Op","unknown operation code.");
}
