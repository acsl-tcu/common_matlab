
#include "mex.h"
#include <cmath>
#include <algorithm>
#include <limits>

static inline double dot3(const double* a,const double* b){
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static inline double norm3(const double* a){ return std::sqrt(dot3(a,a)); }
static inline void sub3(const double* a,const double* b,double* y){
    y[0]=a[0]-b[0]; y[1]=a[1]-b[1]; y[2]=a[2]-b[2];
}
static inline void Rt_mul(const double* R,const double* x,double* y){
    y[0]=R[0]*x[0]+R[1]*x[1]+R[2]*x[2];
    y[1]=R[3]*x[0]+R[4]*x[1]+R[5]*x[2];
    y[2]=R[6]*x[0]+R[7]*x[1]+R[8]*x[2];
}
static inline void R_mul(const double* R,const double* x,double* y){
    y[0]=R[0]*x[0]+R[3]*x[1]+R[6]*x[2];
    y[1]=R[1]*x[0]+R[4]*x[1]+R[7]*x[2];
    y[2]=R[2]*x[0]+R[5]*x[1]+R[8]*x[2];
}
static inline double spacing(double x){
    return std::nextafter(x,std::numeric_limits<double>::infinity())-x;
}
static inline bool finite3(const double* x){
    return std::isfinite(x[0])&&std::isfinite(x[1])&&std::isfinite(x[2]);
}

static bool point_best(const double* x,const double* c,const double* R,
                       const double* a,double rho,double& best){
    double d[3]; sub3(x,c,d);
    double y[3]; Rt_mul(R,d,y);
    double y2[3]={y[0]/(a[0]*a[0]),y[1]/(a[1]*a[1]),y[2]/(a[2]*a[2])};
    double n0[3]; R_mul(R,y2,n0);
    double n1[3]={d[0],d[1],d[2]};
    const double* N[2]={n0,n1};
    best=-std::numeric_limits<double>::infinity();
    bool ok=false;
    for(int k=0;k<2;++k){
        const double* n=N[k];
        double nn=norm3(n);
        if(!(std::isfinite(nn)&&nn>0.0)) continue;
        double rn[3]; Rt_mul(R,n,rn);
        double ar[3]={a[0]*rn[0],a[1]*rn[1],a[2]*rn[2]};
        double hs=dot3(n,c)+norm3(ar);
        double m=(dot3(n,x)-hs-rho*nn)/nn;
        if(std::isfinite(m)){ best=std::max(best,m); ok=true; }
    }
    return ok;
}

static bool segment_best(const double* p0,const double* p1,const double* c,
                         const double* R,const double* a,double rho,double& best){
    double d0[3],d1[3],z0[3],z1[3],s[3];
    sub3(p0,c,d0); sub3(p1,c,d1);
    Rt_mul(R,d0,z0); Rt_mul(R,d1,z1);
    for(int i=0;i<3;++i){
        z0[i]/=a[i]; z1[i]/=a[i]; s[i]=z1[i]-z0[i];
    }
    double ss=dot3(s,s);
    double lam=0.0;
    if(ss>0.0){
        lam=-dot3(z0,s)/ss;
        lam=std::max(0.0,std::min(1.0,lam));
    }
    double xs[3]={
        p0[0]+lam*(p1[0]-p0[0]),
        p0[1]+lam*(p1[1]-p0[1]),
        p0[2]+lam*(p1[2]-p0[2])
    };
    const double* Y[3]={xs,p0,p1};
    best=-std::numeric_limits<double>::infinity();
    bool ok=false;
    for(int j=0;j<3;++j){
        double d[3]; sub3(Y[j],c,d);
        double yr[3]; Rt_mul(R,d,yr);
        double y2[3]={yr[0]/(a[0]*a[0]),yr[1]/(a[1]*a[1]),yr[2]/(a[2]*a[2])};
        double n0[3]; R_mul(R,y2,n0);
        double n1[3]={d[0],d[1],d[2]};
        const double* N[2]={n0,n1};
        for(int k=0;k<2;++k){
            const double* n=N[k];
            double nn=norm3(n);
            if(!(std::isfinite(nn)&&nn>0.0)) continue;
            double rn[3]; Rt_mul(R,n,rn);
            double ar[3]={a[0]*rn[0],a[1]*rn[1],a[2]*rn[2]};
            double hs=dot3(n,c)+norm3(ar);
            double proj=std::min(dot3(n,p0),dot3(n,p1));
            double m=(proj-hs-rho*nn)/nn;
            if(std::isfinite(m)){ best=std::max(best,m); ok=true; }
        }
    }
    return ok;
}

static double scalar(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetNumberOfElements(x)!=1)
        mexErrMsgIdAndTxt("AF7R31:Input","%s must be a real double scalar.",name);
    double v=mxGetScalar(x);
    if(!std::isfinite(v))
        mexErrMsgIdAndTxt("AF7R31:Input","%s must be finite.",name);
    return v;
}
static void req3xN(const mxArray* x,const char* name,mwSize& N){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetM(x)!=3)
        mexErrMsgIdAndTxt("AF7R31:Input","%s must be real double 3xN.",name);
    N=mxGetN(x);
}
static void req3(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetNumberOfElements(x)!=3)
        mexErrMsgIdAndTxt("AF7R31:Input","%s must be a real double 3-vector.",name);
}
static void reqR(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetM(x)!=3||mxGetN(x)!=3)
        mexErrMsgIdAndTxt("AF7R31:Input","%s must be real double 3x3.",name);
}
static double vec_at(const mxArray* x,mwIndex i,mwSize N,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x))
        mexErrMsgIdAndTxt("AF7R31:Input","%s must be real double.",name);
    mwSize ne=mxGetNumberOfElements(x);
    if(ne==1) return mxGetScalar(x);
    if(ne!=N) mexErrMsgIdAndTxt("AF7R31:Input","%s must be scalar or length N.",name);
    return mxGetPr(x)[i];
}

static void op_episode(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    // op,pL,pQ,half,c,R,a,rhoL0,rhoQ0,rhoC0,VL,VQ
    if(nrhs!=12) mexErrMsgIdAndTxt("AF7R31:Arity","episode op expects 11 data inputs.");
    mwSize N1,N2; req3xN(prhs[1],"pL",N1); req3xN(prhs[2],"pQ",N2);
    if(N1!=N2) mexErrMsgIdAndTxt("AF7R31:Input","pL/pQ N mismatch.");
    mwSize N=N1;
    req3(prhs[4],"c"); reqR(prhs[5],"R"); req3(prhs[6],"a");
    const double* pL=mxGetPr(prhs[1]); const double* pQ=mxGetPr(prhs[2]);
    const double* c=mxGetPr(prhs[4]); const double* R=mxGetPr(prhs[5]); const double* a=mxGetPr(prhs[6]);
    double rhoL0=scalar(prhs[7],"rhoL0");
    double rhoQ0=scalar(prhs[8],"rhoQ0");
    double rhoC0=scalar(prhs[9],"rhoC0");
    double VL=scalar(prhs[10],"VL");
    double VQ=scalar(prhs[11],"VQ");
    double VC=std::max(VL,VQ);

    plhs[0]=mxCreateLogicalMatrix(1,N);
    mxLogical* safe=mxGetLogicals(plhs[0]);
    if(nlhs>1) plhs[1]=mxCreateDoubleMatrix(3,N,mxREAL);
    double* MM=(nlhs>1)?mxGetPr(plhs[1]):nullptr;

    for(mwIndex i=0;i<N;++i){
        const double* xL=pL+3*i; const double* xQ=pQ+3*i;
        double h=vec_at(prhs[3],i,N,"half");
        double mL=-mxGetInf(),mQ=-mxGetInf(),mC=-mxGetInf();
        bool ok=std::isfinite(h)&&h>=0.0&&finite3(xL)&&finite3(xQ);
        if(ok) ok=point_best(xL,c,R,a,rhoL0+VL*h,mL);
        if(ok) ok=point_best(xQ,c,R,a,rhoQ0+VQ*h,mQ);
        if(ok) ok=segment_best(xQ,xL,c,R,a,rhoC0+VC*h,mC);
        safe[i]=ok&&mL>0.0&&mQ>0.0&&mC>0.0;
        if(MM){
            MM[0+3*i]=mL; MM[1+3*i]=mQ; MM[2+3*i]=mC;
        }
    }
}

static void op_corridor(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    // op,pL,pQ,half,c,R,a,n,rhoL0,rhoQ0,rhoC0,VL,VQ
    if(nrhs!=13) mexErrMsgIdAndTxt("AF7R31:Arity","corridor op expects 12 data inputs.");
    mwSize N1,N2; req3xN(prhs[1],"pL",N1); req3xN(prhs[2],"pQ",N2);
    if(N1!=N2) mexErrMsgIdAndTxt("AF7R31:Input","pL/pQ N mismatch.");
    mwSize N=N1;
    req3(prhs[4],"c"); reqR(prhs[5],"R"); req3(prhs[6],"a"); req3(prhs[7],"n");
    const double* pL=mxGetPr(prhs[1]); const double* pQ=mxGetPr(prhs[2]);
    const double* c=mxGetPr(prhs[4]); const double* R=mxGetPr(prhs[5]); const double* a=mxGetPr(prhs[6]);
    const double* nin=mxGetPr(prhs[7]);
    double nrm=norm3(nin);
    if(!(std::isfinite(nrm)&&nrm>0.0))
        mexErrMsgIdAndTxt("AF7R31:Input","invalid corridor normal.");
    double n[3]={nin[0]/nrm,nin[1]/nrm,nin[2]/nrm};
    double rn[3]; Rt_mul(R,n,rn);
    double ar[3]={a[0]*rn[0],a[1]*rn[1],a[2]*rn[2]};
    double hObs=dot3(n,c)+norm3(ar);
    double rhoL0=scalar(prhs[8],"rhoL0");
    double rhoQ0=scalar(prhs[9],"rhoQ0");
    double rhoC0=scalar(prhs[10],"rhoC0");
    double VL=scalar(prhs[11],"VL");
    double VQ=scalar(prhs[12],"VQ");
    double VC=std::max(VL,VQ);

    plhs[0]=mxCreateDoubleMatrix(N,1,mxREAL);
    double* req=mxGetPr(plhs[0]);
    for(mwIndex i=0;i<N;++i){
        const double* xL=pL+3*i; const double* xQ=pQ+3*i;
        double h=vec_at(prhs[3],i,N,"half");
        if(!(std::isfinite(h)&&h>=0.0&&finite3(xL)&&finite3(xQ))){
            req[i]=mxGetNaN(); continue;
        }
        double rhoL=rhoL0+VL*h;
        double rhoQ=rhoQ0+VQ*h;
        double rhoC=rhoC0+VC*h;
        double r=std::max({
            hObs+rhoL-dot3(n,xL),
            hObs+rhoQ-dot3(n,xQ),
            hObs+rhoC-std::min(dot3(n,xQ),dot3(n,xL))
        });
        double scale=std::max(1.0,std::fabs(r));
        r += 64.0*spacing(scale);
        req[i]=r;
    }
}

static void op_wholebody(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    // op,pL,pQ,half,C,Rflat,A,rhoL0,rhoQ0,rhoC0,VL,VQ
    if(nrhs!=12) mexErrMsgIdAndTxt("AF7R31:Arity","wholebody op expects 11 data inputs.");
    mwSize N1,N2; req3xN(prhs[1],"pL",N1); req3xN(prhs[2],"pQ",N2);
    if(N1!=N2) mexErrMsgIdAndTxt("AF7R31:Input","pL/pQ N mismatch.");
    mwSize N=N1;
    if(!mxIsDouble(prhs[4])||mxIsComplex(prhs[4])||mxGetM(prhs[4])!=3)
        mexErrMsgIdAndTxt("AF7R31:Input","C must be 3xM.");
    mwSize M=mxGetN(prhs[4]);
    if(!mxIsDouble(prhs[5])||mxIsComplex(prhs[5])||mxGetM(prhs[5])!=9||mxGetN(prhs[5])!=M)
        mexErrMsgIdAndTxt("AF7R31:Input","Rflat must be 9xM.");
    if(!mxIsDouble(prhs[6])||mxIsComplex(prhs[6])||mxGetM(prhs[6])!=3||mxGetN(prhs[6])!=M)
        mexErrMsgIdAndTxt("AF7R31:Input","A must be 3xM.");

    const double* pL=mxGetPr(prhs[1]); const double* pQ=mxGetPr(prhs[2]);
    const double* C=mxGetPr(prhs[4]); const double* RF=mxGetPr(prhs[5]); const double* A=mxGetPr(prhs[6]);
    double rhoL0=scalar(prhs[7],"rhoL0");
    double rhoQ0=scalar(prhs[8],"rhoQ0");
    double rhoC0=scalar(prhs[9],"rhoC0");
    double VL=scalar(prhs[10],"VL");
    double VQ=scalar(prhs[11],"VQ");
    double VC=std::max(VL,VQ);

    plhs[0]=mxCreateLogicalMatrix(1,N);
    mxLogical* pass=mxGetLogicals(plhs[0]);
    if(nlhs>1) plhs[1]=mxCreateDoubleMatrix(3,N,mxREAL);
    double* MM=(nlhs>1)?mxGetPr(plhs[1]):nullptr;

    for(mwIndex i=0;i<N;++i){
        const double* xL=pL+3*i; const double* xQ=pQ+3*i;
        double h=vec_at(prhs[3],i,N,"half");
        double minL=mxGetInf(),minQ=mxGetInf(),minC=mxGetInf();
        bool ok=std::isfinite(h)&&h>=0.0&&finite3(xL)&&finite3(xQ);
        for(mwIndex j=0;j<M&&ok;++j){
            const double* c=C+3*j; const double* R=RF+9*j; const double* a=A+3*j;
            double mL,mQ,mC;
            if(!point_best(xL,c,R,a,rhoL0,mL)){ ok=false; break; }
            if(!point_best(xQ,c,R,a,rhoQ0,mQ)){ ok=false; break; }
            if(!segment_best(xQ,xL,c,R,a,rhoC0,mC)){ ok=false; break; }
            minL=std::min(minL,mL);
            minQ=std::min(minQ,mQ);
            minC=std::min(minC,mC);
        }
        bool cell=false;
        if(ok&&minL>0.0&&minQ>0.0&&minC>0.0){
            double rL=(VL==0.0)?mxGetInf():minL/VL;
            double rQ=(VQ==0.0)?mxGetInf():minQ/VQ;
            double rC=(VC==0.0)?mxGetInf():minC/VC;
            double r=std::min(rL,std::min(rQ,rC));
            cell=std::isinf(r) || h<r;
        }
        pass[i]=cell;
        if(MM){
            MM[0+3*i]=ok?minL:-mxGetInf();
            MM[1+3*i]=ok?minQ:-mxGetInf();
            MM[2+3*i]=ok?minC:-mxGetInf();
        }
    }
}

void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    if(nrhs<1||!mxIsDouble(prhs[0])||mxIsComplex(prhs[0])||mxGetNumberOfElements(prhs[0])!=1)
        mexErrMsgIdAndTxt("AF7R31:Input","first input must be numeric op code.");
    int op=(int)mxGetScalar(prhs[0]);
    if(op==0) op_episode(nlhs,plhs,nrhs,prhs);
    else if(op==1) op_corridor(nlhs,plhs,nrhs,prhs);
    else if(op==2) op_wholebody(nlhs,plhs,nrhs,prhs);
    else mexErrMsgIdAndTxt("AF7R31:Op","unknown op code.");
}
