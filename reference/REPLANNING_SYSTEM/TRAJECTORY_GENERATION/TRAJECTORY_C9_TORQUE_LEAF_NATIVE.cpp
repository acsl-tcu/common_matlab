// Single-call interval-jet nominal torque leaf for the C9 (degree 23) model.
// C++17.  Build using MATLAB's mex -R2018a ... BEFORE Flight.
// Inputs to MEX: lo,hi 3x6 [pL',...,pL^(6)], physics [mu,mLlo,mLhi,L,
// gx,gy,gz,Jx,Jy,Jz,minFz,minPlanarSpeed], flags [cancellation,gyro,symmetric].
// Outputs: torqueBounds 3x2 [lower upper], valid scalar logical.
// Mathematically a conservative REAL-ARITHMETIC interval extension;
// WITHOUT directed rounding of input Bernstein polygons and each operation
// this is NOT machine-verified continuous-time proof and never COMMITs.
// This native routine does NOT access HLC/EKF/obstacles or use fixed physical values.
#include <cmath>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <limits>
#ifndef C9_STANDALONE
#include "mex.h"
#endif
namespace c9 {
static const char* const kernel_version = "C9_FUSED_TORQUE_LEAF_20261110_V1";
struct I {double lo,hi;};
using V = std::array<I,3>;
using Jet = std::array<V,5>;
using SJet = std::array<I,5>;
inline I pt(double x) { return {x,x}; }
inline I add(I a,I b) { return {a.lo+b.lo,a.hi+b.hi}; }
inline I neg(I a) { return {-a.hi,-a.lo}; }
inline I sub(I a,I b) { return add(a,neg(b)); }
inline I mul(I a,I b) {
    double x[]={a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi};
    return {*std::min_element(x,x+4),*std::max_element(x,x+4)};
}
inline I divi(I a,I b) {
    if (!(b.lo>0||b.hi<0))throw std::domain_error("zero interval divisor");
    return mul(a,{1.0/b.hi,1.0/b.lo});
}
inline I sq(I a) {
    double x=a.lo*a.lo,y=a.hi*a.hi;
    return {(a.lo<=0&&a.hi>=0)?0.0:std::min(x,y),std::max(x,y)};
}
inline I nsq(const V& a) {I s=pt(0);for(const I& t:a)s=add(s,sq(t));return s;}
inline I sqrt_i(I a) {if(!(a.lo>0))throw std::domain_error("zero norm"); return {std::sqrt(a.lo),std::sqrt(a.hi)};}
inline V ptv(double x) {return {pt(x),pt(x),pt(x)};}
inline V add(V a,V b) {for(int j=0;j<3;j++)a[j]=add(a[j],b[j]);return a;}
inline V sub(V a,V b) {for(int j=0;j<3;j++)a[j]=sub(a[j],b[j]);return a;}
inline V neg(V a) {for(I& z:a)z=neg(z);return a;}
inline V mul(V a,I b) {for(I& z:a)z=mul(z,b);return a;}
inline V divi(V a,I b) {for(I& z:a)z=divi(z,b);return a;}
inline I dot(V a,V b) {I s=pt(0);for(int j=0;j<3;j++)s=add(s,mul(a[j],b[j]));return s;}
inline V cross(V a,V b) {
    V z;for(int i=0;i<3;i++){int j=(i+1)%3,k=(i+2)%3;
        z[i]=sub(mul(a[j],b[k]),mul(a[k],b[j]));}
    return z;
}
inline int cnk(int n,int k) {
    static const int C[5][5]={{1,0,0,0,0},{1,1,0,0,0},{1,2,1,0,0},{1,3,3,1,0},{1,4,6,4,1}};
    return C[n][k];
}
inline Jet unit(Jet X,int nmax,bool sym) {
    Jet U{};SJet s{};
    s[0]=sqrt_i(nsq(X[0]));U[0]=divi(X[0],s[0]);
    for(int n=1;n<=nmax;n++){
        I rhs=pt(0);
        if(sym){
            for(int k=0;k<=n/2;k++){
                int c=cnk(n,k);
                I prod=(2*k==n)?nsq(X[k]):dot(X[k],X[n-k]);
                rhs=add(rhs,mul(pt(double((2*k==n)?c:2*c)),prod));
            }
            for(int k=1;k<=n/2;k++){
                int c=cnk(n,k);
                I prod=(2*k==n)?sq(s[k]):mul(s[k],s[n-k]);
                rhs=sub(rhs,mul(pt(double((2*k==n)?c:2*c)),prod));
            }
        }else{
            for(int k=0;k<=n;k++)
                rhs=add(rhs,mul(pt(double(cnk(n,k))),dot(X[k],X[n-k])));
            for(int k=1;k<n;k++)
                rhs=sub(rhs,mul(pt(double(cnk(n,k))),mul(s[k],s[n-k])));
        }
        s[n]=divi(rhs,mul(pt(2),s[0]));
        V num=X[n];
        for(int k=1;k<=n;k++)
            num=sub(num,mul(U[n-k],mul(pt(double(cnk(n,k))),s[k])));
        U[n]=divi(num,s[0]);
    }
    return U;
}
inline Jet jet_cross(const Jet& a,const Jet& b,int maxn){
    Jet z{};
    for(int n=0;n<=maxn;n++){
        V q=ptv(0);
        for(int k=0;k<=n;k++)q=add(q,mul(cross(a[k],b[n-k]),pt(double(cnk(n,k)))));
        z[n]=q;
    }
    return z;
}
inline V lateral(V h) {return {neg(h[1]),h[0],pt(0)};}
inline double planar_lower(const V& h) {
    double d=0;
    for(int j=0;j<2;j++){
        double v=std::max({h[j].lo,-h[j].hi,0.0});d+=v*v;
    }return std::sqrt(d);
}
inline bool finite(V q) {for(auto& t:q)if(!std::isfinite(t.lo)||!std::isfinite(t.hi)||t.lo>t.hi)return false;return true;}
inline bool evaluate(const double* low,const double* high,const double* phys,const double* flags,double out[6]){
    // MATLAB column-major 3x6. The parameter counts/shapes are checked at MEX entry.
    const double mu=phys[0],mlo=phys[1],mhi=phys[2],L=phys[3];
    const bool cancel=flags[0]!=0,factored=flags[1]!=0,sym=flags[2]!=0;
    Jet w{}; Jet v0{};
    for(int d=0;d<3;d++){
        I acc={low[3+d],high[3+d]};
        w[0][d]=sub(pt(phys[4+d]),acc);
        for(int k=1;k<=4;k++)w[k][d]=neg(I{low[3*(k+1)+d],high[3*(k+1)+d]});
        for(int k=0;k<=2;k++)v0[k][d]={low[3*k+d],high[3*k+d]};
    }
    Jet q=unit(w,4,sym);
    Jet f{},v{};
    const I mass=add(pt(mu),{mlo,mhi});
    for(int k=0;k<=2;k++){
        f[k]=sub(neg(mul(w[k],mass)),mul(q[k+2],pt(mu*L)));
        v[k]=sub(v0[k],mul(q[k+1],pt(L)));
    }
    if(!(f[0][2].lo>phys[10]))return false;
    Jet h{};
    for(int k=0;k<=2;k++)h[k]={v[k][0],v[k][1],pt(0)};
    if(!(planar_lower(h[0])>phys[11]))return false;
    Jet lateralJet{};Jet raw{};Jet b1{},b3{};
    if(cancel){
        for(int k=0;k<=2;k++)lateralJet[k]=lateral(h[k]);
        raw=jet_cross(lateralJet,f,2);
        b1=unit(raw,2,sym);b3=unit(f,2,sym);
    }else{
        Jet normalizedH=unit(h,2,sym);
        for(int k=0;k<=2;k++)lateralJet[k]=lateral(normalizedH[k]);
        b3=unit(f,2,sym);raw=jet_cross(lateralJet,b3,2);
        b1=unit(raw,2,sym);
    }
    Jet b2=jet_cross(b3,b1,2);
    V omega={dot(b3[0],b2[1]),dot(b1[0],b3[1]),dot(b2[0],b1[1])};
    V alpha={add(dot(b3[1],b2[1]),dot(b3[0],b2[2])),
             add(dot(b1[1],b3[1]),dot(b1[0],b3[2])),
             add(dot(b2[1],b1[1]),dot(b2[0],b1[2]))};
    const double J[3]={phys[7],phys[8],phys[9]};
    V gy{};
    if(factored){
        gy[0]=mul(pt(J[2]-J[1]),mul(omega[1],omega[2]));
        gy[1]=mul(pt(J[0]-J[2]),mul(omega[2],omega[0]));
        gy[2]=mul(pt(J[1]-J[0]),mul(omega[0],omega[1]));
    }else{
        V Jomega=omega;for(int j=0;j<3;j++)Jomega[j]=mul(omega[j],pt(J[j]));
        gy=cross(omega,Jomega);
    }
    V torque{};
    for(int j=0;j<3;j++)torque[j]=add(mul(pt(J[j]),alpha[j]),gy[j]);
    if(!finite(torque))return false;
    for(int j=0;j<3;j++){out[j]=torque[j].lo;out[j+3]=torque[j].hi;}
    return true;
}
}
// Standalone C interface only for independent randomized differential testing.
#ifdef C9_STANDALONE
extern "C" int c9_torque_leaf_test(const double* lo,const double* hi,const double* physics,const double* opts,double* out){
    try {return c9::evaluate(lo,hi,physics,opts,out)?1:0;}catch(...){return 0;}
}
#else
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    if(nrhs==0 && nlhs==1){
        plhs[0]=mxCreateString(c9::kernel_version);
        return;
    }
    if(nrhs!=4||nlhs!=2)mexErrMsgIdAndTxt("C9_NATIVE:Signature","Need [bounds,valid]=native(lo,hi,physics,flags).");
    const mwSize r[]={3,3,12,3};
    for(int k=0;k<4;k++){
        const mxArray* a=prhs[k];
        const mwSize cnt=mxGetNumberOfElements(a);
        if(!mxIsDouble(a)||mxIsComplex(a)||cnt!=((k<=1)?18:r[k]))
            mexErrMsgIdAndTxt("C9_NATIVE:InputShape","Expect real doubles lo/hi 3x6, physics 12x1, flags 3x1.");
        const double* p=mxGetPr(a);
        for(mwIndex i=0;i<cnt;i++)if(!std::isfinite(p[i]))mexErrMsgIdAndTxt("C9_NATIVE:Nonfinite","Finite inputs required.");
    }
    const double* lo=mxGetPr(prhs[0]);const double* hi=mxGetPr(prhs[1]);
    const double* p=mxGetPr(prhs[2]);const double* sw=mxGetPr(prhs[3]);
    for(int i=0;i<18;i++)if(lo[i]>hi[i])mexErrMsgIdAndTxt("C9_NATIVE:Interval","lo>hi.");
    if(!(p[0]>0&&p[1]>0&&p[2]>=p[1]&&p[3]>0&&p[7]>0&&p[8]>0&&p[9]>0&&p[10]>=0&&p[11]>=0))
        mexErrMsgIdAndTxt("C9_NATIVE:Physics","Invalid mass, inertia, or nondegeneracy guard.");
    for(int k=0;k<3;k++)if(!(sw[k]==0||sw[k]==1))mexErrMsgIdAndTxt("C9_NATIVE:Flag","Flags must be 0/1.");
    plhs[0]=mxCreateDoubleMatrix(3,2,mxREAL);
    plhs[1]=mxCreateLogicalScalar(false);
    double* out=mxGetPr(plhs[0]);
    for(int j=0;j<6;j++)out[j]=mxGetNaN();
    bool ok=false;
    try{ok=c9::evaluate(lo,hi,p,sw,out);}catch(const std::exception&){ok=false;}
    if(!ok){for(int j=0;j<6;j++)out[j]=mxGetNaN();}
    *mxGetLogicals(plhs[1])=ok;
}
#endif
