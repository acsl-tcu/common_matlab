
#include "mex.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

struct V3{double x=0,y=0,z=0;};
static V3 add(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 sub(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static V3 mul(V3 a,double s){return {a.x*s,a.y*s,a.z*s};}
using Poly=std::vector<V3>;

struct Corr{
    std::vector<double> t0,t1;
    std::vector<Poly> cp;
};
static std::vector<Corr> G;
static std::atomic<int> ACTIVE{0};
static bool LOCKED=false;

static double scalar(const mxArray*a,const char*n){
    if(!a||!mxIsDouble(a)||mxIsComplex(a)||mxGetNumberOfElements(a)!=1)
        mexErrMsgIdAndTxt("AF7R55:Input","%s must be real scalar",n);
    double v=mxGetScalar(a);
    if(!std::isfinite(v))mexErrMsgIdAndTxt("AF7R55:Input","%s nonfinite",n);
    return v;
}
static const mxArray* fld(const mxArray*s,const char*n){
    if(!mxIsStruct(s))mexErrMsgIdAndTxt("AF7R55:Input","packet must be struct");
    const mxArray*x=mxGetField(s,0,n);
    if(!x)mexErrMsgIdAndTxt("AF7R55:Input","missing field %s",n);
    return x;
}
static std::vector<double> vec(const mxArray*a,const char*n){
    if(!mxIsDouble(a)||mxIsComplex(a))mexErrMsgIdAndTxt("AF7R55:Input","%s must be real double",n);
    size_t N=mxGetNumberOfElements(a);const double*p=mxGetPr(a);
    std::vector<double>v(p,p+N);for(double x:v)if(!std::isfinite(x))mexErrMsgIdAndTxt("AF7R55:Input","%s nonfinite",n);
    return v;
}
static Corr parse_corr(const mxArray*s){
    Corr C;C.t0=vec(fld(s,"segment_t0"),"segment_t0");C.t1=vec(fld(s,"segment_t1"),"segment_t1");
    if(C.t0.empty()||C.t0.size()!=C.t1.size())mexErrMsgIdAndTxt("AF7R55:Input","segment sizes");
    const mxArray*A=fld(s,"control_points_matrix");
    if(!mxIsDouble(A)||mxIsComplex(A)||mxGetM(A)!=48||mxGetN(A)!=C.t0.size())
        mexErrMsgIdAndTxt("AF7R55:Input","control_points_matrix must be 48xS");
    const double*p=mxGetPr(A);C.cp.resize(C.t0.size());
    for(size_t s0=0;s0<C.t0.size();++s0){
        if(!(C.t1[s0]>C.t0[s0]))mexErrMsgIdAndTxt("AF7R55:Input","nonpositive segment");
        C.cp[s0].resize(16);
        for(int k=0;k<16;++k){
            size_t b=48*s0+3*k;
            C.cp[s0][k]={p[b],p[b+1],p[b+2]};
        }
    }return C;
}
static V3 decast(Poly C,double u){
    while(C.size()>1){
        Poly D(C.size()-1);
        for(size_t i=0;i+1<C.size();++i)D[i]=add(mul(C[i],1-u),mul(C[i+1],u));
        C.swap(D);
    }return C.front();
}
static Poly deriv(Poly D,double T,int r){
    int deg=(int)D.size()-1;
    for(int q=0;q<r;++q){
        Poly E(D.size()-1);
        for(size_t i=0;i+1<D.size();++i)E[i]=mul(sub(D[i+1],D[i]),double(deg)/T);
        D.swap(E);--deg;
    }return D;
}
static std::array<V3,7> eval_corr(const Corr&C,double t){
    std::array<V3,7>Y{};
    if(C.cp.empty()||t<=C.t0.front()||t>=C.t1.back())return Y;
    int s=-1;for(int i=0;i<(int)C.cp.size();++i){if(t<=C.t1[i]){s=i;break;}}
    if(s<0||t<C.t0[s])return Y;
    double T=C.t1[s]-C.t0[s],u=std::max(0.0,std::min(1.0,(t-C.t0[s])/T));
    for(int r=0;r<=6;++r)Y[r]=decast(deriv(C.cp[s],T,r),u);
    return Y;
}
static mxArray* eval_chain(int version,const mxArray*tin){
    if(version<0||version>(int)G.size())mexErrMsgIdAndTxt("AF7R55:Version","invalid version");
    if(!mxIsDouble(tin)||mxIsComplex(tin))mexErrMsgIdAndTxt("AF7R55:Input","t must be real double");
    size_t N=mxGetNumberOfElements(tin);const double*t=mxGetPr(tin);
    mxArray*out=mxCreateDoubleMatrix(21,N,mxREAL);double*p=mxGetPr(out);
    for(size_t k=0;k<N;++k){
        if(!std::isfinite(t[k]))mexErrMsgIdAndTxt("AF7R55:Input","t nonfinite");
        std::array<V3,7>S{};
        for(int v=0;v<version;++v){
            auto Y=eval_corr(G[v],t[k]);
            for(int r=0;r<7;++r)S[r]=add(S[r],Y[r]);
        }
        for(int r=0;r<7;++r){
            p[(3*r+0)+21*k]=S[r].x;p[(3*r+1)+21*k]=S[r].y;p[(3*r+2)+21*k]=S[r].z;
        }
    }return out;
}
static void cleanup(){G.clear();ACTIVE.store(0,std::memory_order_release);if(LOCKED){mexUnlock();LOCKED=false;}}

void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    if(nrhs<1)mexErrMsgIdAndTxt("AF7R55:Arity","need op");
    int op=(int)scalar(prhs[0],"op");
    if(op==0){
        cleanup();mexLock();LOCKED=true;mexAtExit(cleanup);
        if(nlhs)plhs[0]=mxCreateLogicalScalar(true);return;
    }
    if(op==1){
        if(nrhs!=2)mexErrMsgIdAndTxt("AF7R55:Arity","append: op, correction");
        Corr C=parse_corr(prhs[1]);G.push_back(std::move(C));
        if(nlhs)plhs[0]=mxCreateDoubleScalar((double)G.size());return;
    }
    if(op==2){
        if(nrhs!=2)mexErrMsgIdAndTxt("AF7R55:Arity","activate: op, version");
        int v=(int)scalar(prhs[1],"version");
        if(v<0||v>(int)G.size())mexErrMsgIdAndTxt("AF7R55:Version","invalid activation version");
        ACTIVE.store(v,std::memory_order_release);
        if(nlhs)plhs[0]=mxCreateDoubleScalar(v);return;
    }
    if(op==3){
        if(nrhs!=2)mexErrMsgIdAndTxt("AF7R55:Arity","eval active: op,t");
        int v=ACTIVE.load(std::memory_order_acquire);plhs[0]=eval_chain(v,prhs[1]);return;
    }
    if(op==4){
        if(nrhs!=3)mexErrMsgIdAndTxt("AF7R55:Arity","eval version: op,version,t");
        int v=(int)scalar(prhs[1],"version");plhs[0]=eval_chain(v,prhs[2]);return;
    }
    if(op==5){
        const char*fn[]={"active_version","stored_versions"};
        mxArray*S=mxCreateStructMatrix(1,1,2,fn);
        mxSetField(S,0,"active_version",mxCreateDoubleScalar(ACTIVE.load(std::memory_order_acquire)));
        mxSetField(S,0,"stored_versions",mxCreateDoubleScalar((double)G.size()));
        plhs[0]=S;return;
    }
    mexErrMsgIdAndTxt("AF7R55:Op","unknown op");
}
