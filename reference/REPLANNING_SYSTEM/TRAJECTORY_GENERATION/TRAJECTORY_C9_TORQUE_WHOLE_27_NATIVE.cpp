// Whole-tree degree-27 C9 nominal torque interval certifier.
// Inputs: 3x28 payload Bernstein controls; duration; 12 live physical
// values; 3 algebra flags; 3x2 preclamp torque limits; 3 torque guards;
// [max_depth,max_nodes]. Outputs: 3x2 torque enclosure, PASS, node count,
// accepted leaves, reason code. Real-time use is opt-in after MATLAB A/B.
// Uses per-primitive outward one-ULP inflation for initial differentiation,
// de Casteljau subintervals and the full 4th/2nd order rigid-body jet.
// Requires IEEE-754 round-to-nearest, no unsafe fast-math/FTZ/DAZ, and
// finite nonsingular intervals. This proves a *nominal model* only, NOT
// HLC execution, virtual tube, obstacle geometry or tracking.
#include <cmath>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <limits>
#include <vector>
#include <utility>
#include <cfenv>
#include <cfloat>
#if defined(__FAST_MATH__)
#error Unsafe fast math forbidden for interval enclosure.
#endif
#ifdef _MSC_VER
#pragma float_control(precise,on)
#pragma fp_contract(off)
#endif
#ifndef C9_STANDALONE
#include "mex.h"
#endif
namespace c9whole {
static const char* const kernel_version = "C9_DIRECTED_WHOLE_TREE_27_20261110_V1";
struct I {double lo,hi;};
using V = std::array<I,3>;
using Jet = std::array<V,5>;
using SJet = std::array<I,5>;
// Round each arithmetic primitive outwards by one representable IEEE-754
// number. No fast-math, FMA contraction, FTZ/DAZ, or non-default runtime
// rounding are permitted in the mathematical justification.
inline double dn(double x) {if(!std::isfinite(x)) throw std::overflow_error("nonfinite"); return std::nextafter(x,-std::numeric_limits<double>::infinity());}
inline double up(double x) {if(!std::isfinite(x)) throw std::overflow_error("nonfinite"); return std::nextafter(x, std::numeric_limits<double>::infinity());}
inline I pt(double x) {if(!std::isfinite(x)) throw std::overflow_error("nonfinite point");return {x,x};}
inline I add(I a,I b) {return {dn(a.lo+b.lo),up(a.hi+b.hi)};}
inline I neg(I a) {return {-a.hi,-a.lo};}
inline I sub(I a,I b) {return {dn(a.lo-b.hi),up(a.hi-b.lo)};}
inline I mul(I a,I b) {
    double x[]={a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi};
    for(double e:x)if(!std::isfinite(e))throw std::overflow_error("product nonfinite");
    return {dn(*std::min_element(x,x+4)),up(*std::max_element(x,x+4))};
}
inline I divi(I a,I b) {
    if (!(b.lo>0||b.hi<0))throw std::domain_error("zero interval divisor");
    I inv={dn(1.0/b.hi),up(1.0/b.lo)};
    return mul(a,inv);
}
inline I sq(I a) {
    double x=a.lo*a.lo,y=a.hi*a.hi;
    if(!std::isfinite(x)||!std::isfinite(y))throw std::overflow_error("square nonfinite");
    return {(a.lo<=0&&a.hi>=0)?0.0:dn(std::min(x,y)),up(std::max(x,y))};
}
inline I nsq(const V& a) {I s=pt(0);for(const I& t:a)s=add(s,sq(t));return s;}
inline I sqrt_i(I a) {
    if(!(a.lo>0))throw std::domain_error("zero norm");
    return {dn(std::sqrt(a.lo)),up(std::sqrt(a.hi))};
}
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
    I norm2=pt(0);
    for(int j=0;j<2;j++){
        double d=std::max({h[j].lo,-h[j].hi,0.0});
        norm2=add(norm2,sq(pt(d)));
    }
    return dn(std::sqrt(std::max(0.0,norm2.lo)));
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
        f[k]=sub(neg(mul(w[k],mass)),mul(q[k+2],mul(pt(mu),pt(L))));
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
        gy[0]=mul(sub(pt(J[2]),pt(J[1])),mul(omega[1],omega[2]));
        gy[1]=mul(sub(pt(J[0]),pt(J[2])),mul(omega[2],omega[0]));
        gy[2]=mul(sub(pt(J[1]),pt(J[0])),mul(omega[0],omega[1]));
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

using Control = std::vector<V>;
using Poly = std::array<Control,6>;  // derivative polynomials D1 .. D6
struct Proof {
    std::array<I,3> bounds{pt(0),pt(0),pt(0)};
    int nodes=0, leaves=0, native_valid=0, reason=0;
    bool has_bounds=false, success=false;
};
inline bool valid_input(const double* c,double T,const double* p,const double* fl,
                        const double* lim,const double* guard,int depth,int nodes){
    if(!(std::isfinite(T)&&T>0 && p[0]>0&&p[1]>0&&p[2]>=p[1]&&p[3]>0&&
         p[7]>0&&p[8]>0&&p[9]>0&&p[10]>=0&&p[11]>=0&&
         depth>=0&&depth<=30&&nodes>0&&nodes<=1000000))return false;
    for(int i=0;i<84;i++)if(!std::isfinite(c[i]))return false;
    for(int i=0;i<12;i++)if(!std::isfinite(p[i]))return false;
    for(int i=0;i<6;i++)if(!std::isfinite(lim[i]))return false;
    for(int i=0;i<3;i++)if(!std::isfinite(guard[i])||guard[i]<0||lim[i]+guard[i]>lim[i+3]-guard[i])return false;
    for(int i=0;i<3;i++)if(!(fl[i]==0||fl[i]==1))return false;
    return true;
}
inline Poly derivs(const double* c,double duration){
    Control d(28);
    for(int k=0;k<28;k++)for(int j=0;j<3;j++)d[k][j]=pt(c[3*k+j]);
    Poly P;
    for(int k=0;k<6;k++){
        I factor=divi(pt(double(27-k)),pt(duration));
        Control next(d.size()-1);
        for(size_t i=0;i<next.size();i++)for(int j=0;j<3;j++){
            next[i][j]=mul(sub(d[i+1][j],d[i][j]),factor);
        }
        P[k]=next;d=std::move(next);
    }
    return P;
}
inline void split(const Poly& P,Poly& left,Poly& right){
    for(int d=0;d<6;d++){
        const size_t n=P[d].size();
        left[d].resize(n);right[d].resize(n);
        Control tmp=P[d];
        left[d][0]=tmp[0];right[d][n-1]=tmp[n-1];
        for(size_t k=1;k<n;k++){
            for(size_t i=0;i<n-k;i++)for(int j=0;j<3;j++)
                tmp[i][j]=mul(add(tmp[i][j],tmp[i+1][j]),pt(0.5));
            left[d][k]=tmp[0];right[d][n-k-1]=tmp[n-k-1];
        }
    }
}
inline bool torque_node(const Poly& P,const double* phys,const double* flags,I* out){
    // Bernstein nonnegative partition of unity: every leaf derivative
    // lies inside the convex hull of interval-valued derivative controls.
    // Internal interval arithmetic in evaluate_box is outward rounded.
    double lo[18],hi[18];
    for(int k=0;k<6;k++)for(int j=0;j<3;j++){
        double a=std::numeric_limits<double>::infinity();
        double b=-std::numeric_limits<double>::infinity();
        for(const V& z:P[k]){a=std::min(a,z[j].lo);b=std::max(b,z[j].hi);}
        lo[3*k+j]=a;hi[3*k+j]=b;
    }
    double out6[6];
    if(!evaluate(lo,hi,phys,flags,out6))return false;
    for(int j=0;j<3;j++)out[j]={out6[j],out6[j+3]};
    return true;
}
inline bool traverse(const Poly& P,int depth,int maxdepth,int maxnodes,
                     const double* phys,const double* flags,
                     const double* lim,const double* guard,Proof& r){
    if(r.nodes>=maxnodes){r.reason=2;return false;}
    r.nodes++;
    try{
        I bounds[3];
        if(torque_node(P,phys,flags,bounds)){
            r.native_valid++;
            bool inside=true,excluded=false;
            for(int j=0;j<3;j++){
                // conservative interval limits: round the allowed band inward
                const double reqlo=up(lim[j]+guard[j]);
                const double reqhi=dn(lim[j+3]-guard[j]);
                inside &= bounds[j].lo >= reqlo && bounds[j].hi <= reqhi;
                excluded |= bounds[j].lo>reqhi || bounds[j].hi<reqlo;
            }
            if(inside){
                r.leaves++;
                for(int j=0;j<3;j++){
                    if(!r.has_bounds)r.bounds[j]=bounds[j];
                    else{r.bounds[j].lo=std::min(r.bounds[j].lo,bounds[j].lo);
                         r.bounds[j].hi=std::max(r.bounds[j].hi,bounds[j].hi);}
                }
                r.has_bounds=true;
                return true;
            }
            if(excluded){r.reason=3;return false;}
        }
    }catch(const std::exception&){ /* cannot claim a proof */ }
    if(depth>=maxdepth){r.reason=4;return false;}
    Poly L,R;
    try{split(P,L,R);}catch(const std::exception&){r.reason=5;return false;}
    if(!traverse(L,depth+1,maxdepth,maxnodes,phys,flags,lim,guard,r))return false;
    return traverse(R,depth+1,maxdepth,maxnodes,phys,flags,lim,guard,r);
}
inline Proof prove(const double* c,double duration,const double* phys,const double* flags,
                   const double* lim,const double* guard,int maxdepth,int maxnodes){
    Proof r;
    if(!std::numeric_limits<double>::is_iec559 || std::fegetround()!=FE_TONEAREST){
        r.reason=7;return r;
    }
    if(!valid_input(c,duration,phys,flags,lim,guard,maxdepth,maxnodes)){
        r.reason=6;return r;
    }
    try{
        Poly P=derivs(c,duration);
        r.success=traverse(P,0,maxdepth,maxnodes,phys,flags,lim,guard,r);
    }catch(const std::exception&){r.success=false;r.reason=5;}
    return r;
}
} // namespace c9whole

#ifdef C9_STANDALONE
extern "C" int c9_whole_torque_test(const double* c,double T,const double* phys,const double* flags,
          const double* limits,const double* guard,int depth,int nodes,double* out,int* stats){
    c9whole::Proof r=c9whole::prove(c,T,phys,flags,limits,guard,depth,nodes);
    for(int i=0;i<6;i++)out[i]=std::numeric_limits<double>::quiet_NaN();
    if(r.success)for(int j=0;j<3;j++){
        out[j]=r.bounds[j].lo;out[j+3]=r.bounds[j].hi;
    }
    stats[0]=r.nodes;stats[1]=r.leaves;stats[2]=r.reason;stats[3]=r.native_valid;
    return r.success?1:0;
}
#else
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    using namespace c9whole;
    if(nrhs==0&&nlhs==1){plhs[0]=mxCreateString(kernel_version);return;}
    if(nrhs!=7||nlhs!=5)mexErrMsgIdAndTxt("C9_WHOLE:Signature",
        "Need [bounds,passed,nodeCount,leafCount,reason]=MEX(P,T,physics,flags,limits,guard,budget).");
    const mwSize counts[]={84,1,12,3,6,3,2};
    for(int k=0;k<7;k++){
        const mxArray* a=prhs[k];
        if(!mxIsDouble(a)||mxIsComplex(a)||mxGetNumberOfElements(a)!=counts[k])
            mexErrMsgIdAndTxt("C9_WHOLE:InputShape","Invalid input size or type.");
        const double* x=mxGetPr(a);
        for(mwIndex i=0;i<counts[k];i++)if(!std::isfinite(x[i]))
            mexErrMsgIdAndTxt("C9_WHOLE:Nonfinite","Finite real numeric inputs required.");
    }
    const double* o=mxGetPr(prhs[6]);
    if(o[0]!=std::floor(o[0])||o[1]!=std::floor(o[1])||
       o[0]<0||o[0]>30||o[1]<1||o[1]>1e6)
       mexErrMsgIdAndTxt("C9_WHOLE:Budget","Invalid proof subdivision limits.");
    Proof r=prove(mxGetPr(prhs[0]),mxGetScalar(prhs[1]),
        mxGetPr(prhs[2]),mxGetPr(prhs[3]),mxGetPr(prhs[4]),
        mxGetPr(prhs[5]),int(o[0]),int(o[1]));
    plhs[0]=mxCreateDoubleMatrix(3,2,mxREAL);
    double* a=mxGetPr(plhs[0]);
    for(int i=0;i<6;i++)a[i]=mxGetNaN();
    if(r.success)for(int j=0;j<3;j++){
        a[j]=r.bounds[j].lo;a[j+3]=r.bounds[j].hi;
    }
    plhs[1]=mxCreateLogicalScalar(r.success);
    plhs[2]=mxCreateDoubleScalar(r.nodes);
    plhs[3]=mxCreateDoubleScalar(r.leaves);
    plhs[4]=mxCreateDoubleScalar(r.reason);
}
#endif
