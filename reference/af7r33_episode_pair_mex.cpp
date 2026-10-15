
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
        if(std::isfinite(m)){best=std::max(best,m);ok=true;}
    }
    return ok;
}
static bool segment_best(const double* p0,const double* p1,const double* c,
                         const double* R,const double* a,double rho,double& best){
    double d0[3],d1[3],z0[3],z1[3],s[3];
    sub3(p0,c,d0); sub3(p1,c,d1);
    Rt_mul(R,d0,z0); Rt_mul(R,d1,z1);
    for(int i=0;i<3;++i){ z0[i]/=a[i]; z1[i]/=a[i]; s[i]=z1[i]-z0[i]; }
    double ss=dot3(s,s),lam=0.0;
    if(ss>0.0){ lam=-dot3(z0,s)/ss; lam=std::max(0.0,std::min(1.0,lam)); }
    double xs[3]={p0[0]+lam*(p1[0]-p0[0]),p0[1]+lam*(p1[1]-p0[1]),p0[2]+lam*(p1[2]-p0[2])};
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
            if(std::isfinite(m)){best=std::max(best,m);ok=true;}
        }
    }
    return ok;
}
static double scalar(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetNumberOfElements(x)!=1)
        mexErrMsgIdAndTxt("AF7R33:Input","%s scalar required.",name);
    double v=mxGetScalar(x);
    if(!std::isfinite(v)) mexErrMsgIdAndTxt("AF7R33:Input","%s finite required.",name);
    return v;
}

void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    // pL,pQ,half,C,Rflat,A,obsIndex,rhoL0,rhoQ0,rhoC0,VL,VQ
    if(nrhs!=12) mexErrMsgIdAndTxt("AF7R33:Arity","Expected 12 inputs.");
    if(!mxIsDouble(prhs[0])||mxIsComplex(prhs[0])||mxGetM(prhs[0])!=3)
        mexErrMsgIdAndTxt("AF7R33:Input","pL must be 3xN.");
    const mwSize N=mxGetN(prhs[0]);
    if(!mxIsDouble(prhs[1])||mxIsComplex(prhs[1])||mxGetM(prhs[1])!=3||mxGetN(prhs[1])!=N)
        mexErrMsgIdAndTxt("AF7R33:Input","pQ must be 3xN.");
    if(!mxIsDouble(prhs[2])||mxIsComplex(prhs[2])||mxGetNumberOfElements(prhs[2])!=N)
        mexErrMsgIdAndTxt("AF7R33:Input","half must have N elements.");
    if(!mxIsDouble(prhs[3])||mxIsComplex(prhs[3])||mxGetM(prhs[3])!=3)
        mexErrMsgIdAndTxt("AF7R33:Input","C must be 3xM.");
    const mwSize M=mxGetN(prhs[3]);
    if(!mxIsDouble(prhs[4])||mxIsComplex(prhs[4])||mxGetM(prhs[4])!=9||mxGetN(prhs[4])!=M)
        mexErrMsgIdAndTxt("AF7R33:Input","Rflat must be 9xM.");
    if(!mxIsDouble(prhs[5])||mxIsComplex(prhs[5])||mxGetM(prhs[5])!=3||mxGetN(prhs[5])!=M)
        mexErrMsgIdAndTxt("AF7R33:Input","A must be 3xM.");
    if(!mxIsDouble(prhs[6])||mxIsComplex(prhs[6])||mxGetNumberOfElements(prhs[6])!=N)
        mexErrMsgIdAndTxt("AF7R33:Input","obsIndex must have N elements.");

    const double* pL=mxGetPr(prhs[0]); const double* pQ=mxGetPr(prhs[1]);
    const double* half=mxGetPr(prhs[2]); const double* C=mxGetPr(prhs[3]);
    const double* RF=mxGetPr(prhs[4]); const double* AX=mxGetPr(prhs[5]);
    const double* oi=mxGetPr(prhs[6]);
    double rhoL0=scalar(prhs[7],"rhoL0"),rhoQ0=scalar(prhs[8],"rhoQ0"),rhoC0=scalar(prhs[9],"rhoC0");
    double VL=scalar(prhs[10],"VL"),VQ=scalar(prhs[11],"VQ"),VC=std::max(VL,VQ);

    plhs[0]=mxCreateLogicalMatrix(1,N);
    mxLogical* out=mxGetLogicals(plhs[0]);

    for(mwIndex i=0;i<N;++i){
        long j=(long)std::llround(oi[i])-1;
        if(j<0||j>=(long)M){ out[i]=false; continue; }
        const double* xL=pL+3*i; const double* xQ=pQ+3*i;
        const double* c=C+3*j; const double* R=RF+9*j; const double* a=AX+3*j;
        double h=half[i],mL,mQ,mC;
        bool ok=std::isfinite(h)&&h>=0.0&&finite3(xL)&&finite3(xQ)&&finite3(c)&&finite3(a)&&a[0]>0&&a[1]>0&&a[2]>0;
        if(ok) ok=point_best(xL,c,R,a,rhoL0+VL*h,mL);
        if(ok) ok=point_best(xQ,c,R,a,rhoQ0+VQ*h,mQ);
        if(ok) ok=segment_best(xQ,xL,c,R,a,rhoC0+VC*h,mC);
        out[i]=ok&&mL>0.0&&mQ>0.0&&mC>0.0;
    }
}
