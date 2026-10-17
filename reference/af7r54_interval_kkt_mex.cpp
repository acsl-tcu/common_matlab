
#include "mex.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

struct I{double lo,hi;};
static inline double dn(double x){return std::nextafter(x,-std::numeric_limits<double>::infinity());}
static inline double up(double x){return std::nextafter(x, std::numeric_limits<double>::infinity());}
static inline I cI(double x){return {dn(x),up(x)};}
static inline I mk(double a,double b){return {std::min(a,b),std::max(a,b)};}
static inline I add(I a,I b){return {dn(a.lo+b.lo),up(a.hi+b.hi)};}
static inline I neg(I a){return {-a.hi,-a.lo};}
static inline I sub(I a,I b){return add(a,neg(b));}
static inline I mul(I a,I b){
    double v[4]={a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi};
    return {dn(*std::min_element(v,v+4)),up(*std::max_element(v,v+4))};
}
static inline I divi(I a,I b){
    if(b.lo<=0&&b.hi>=0) mexErrMsgIdAndTxt("AF7R54:Interval","division interval crosses zero");
    double r0=1.0/b.lo,r1=1.0/b.hi;
    I r={dn(std::min(r0,r1)),up(std::max(r0,r1))};
    return mul(a,r);
}
static inline I ipow(I a,int n){
    if(n==0)return cI(1.0);
    I r=cI(1.0),b=a;int k=n;
    while(k){if(k&1)r=mul(r,b);k>>=1;if(k)b=mul(b,b);}
    return r;
}
static double choose_int(int n,int k){
    if(k<0||k>n)return 0;if(k>n-k)k=n-k;double v=1;
    for(int i=1;i<=k;++i)v*=double(n-k+i)/i;return v;
}
static I gramI(int n,int i,int j){
    double g=choose_int(n,i)*choose_int(n,j)/((2*n+1)*choose_int(2*n,i+j));
    return cI(g);
}
using J8=std::array<I,8>;
static std::vector<I> scalar_control(const J8&j0,const J8&j1,I T){
    std::array<I,8>ds{},de{};for(int r=0;r<8;++r){ds[r]=cI(0);de[r]=cI(0);}
    ds[0]=j0[0];de[0]=j1[0];
    double ff=1;
    for(int r=1;r<=7;++r){
        ff*=16-r;
        ds[r]=divi(mul(j0[r],ipow(T,r)),cI(ff));
        de[r]=divi(mul(j1[r],ipow(T,r)),cI(ff));
    }
    std::vector<I>P(16,cI(0));
    for(int k=0;k<=7;++k){
        I v0=cI(0),v1=cI(0);
        for(int r=0;r<=k;++r){
            v0=add(v0,mul(cI(choose_int(k,r)),ds[r]));
            v1=add(v1,mul(cI(choose_int(k,r)*((r&1)?-1.0:1.0)),de[r]));
        }
        P[k]=v0;P[15-k]=v1;
    }
    return P;
}
static std::vector<I> diff_scalar(std::vector<I>p,int r){
    for(int q=0;q<r;++q){
        std::vector<I>d(p.size()-1);
        for(size_t i=0;i+1<p.size();++i)d[i]=sub(p[i+1],p[i]);
        p.swap(d);
    }
    return p;
}
static void add_transition(I H[4],const J8 J0[2],const J8 J1[2],I T){
    std::vector<I>B[2];
    const double ff=15.0*14*13*12;
    for(int k=0;k<2;++k){
        auto P=scalar_control(J0[k],J1[k],T);
        auto d=diff_scalar(P,4);
        I fac=divi(cI(ff),ipow(T,4));
        for(auto&v:d)v=mul(v,fac);
        B[k]=std::move(d);
    }
    for(int i=0;i<2;++i)for(int j=0;j<2;++j){
        I v=cI(0);
        for(int a=0;a<12;++a)for(int b=0;b<12;++b)
            v=add(v,mul(mul(B[i][a],gramI(11,a,b)),B[j][b]));
        H[2*i+j]=add(H[2*i+j],mul(T,v));
    }
}
static double scalar(const mxArray*a,const char*n){
    if(!a||!mxIsDouble(a)||mxIsComplex(a)||mxGetNumberOfElements(a)!=1)
        mexErrMsgIdAndTxt("AF7R54:Input","%s must be real scalar",n);
    double v=mxGetScalar(a);
    if(!std::isfinite(v)) mexErrMsgIdAndTxt("AF7R54:Input","%s nonfinite",n);
    return v;
}
static mxArray* iout(I a){
    mxArray*x=mxCreateDoubleMatrix(1,2,mxREAL);mxGetPr(x)[0]=a.lo;mxGetPr(x)[1]=a.hi;return x;
}
static mxArray* imat(const std::vector<I>&v,size_t m,size_t n){
    mxArray*x=mxCreateDoubleMatrix(m,2*n,mxREAL);double*p=mxGetPr(x);
    for(size_t j=0;j<n;++j)for(size_t i=0;i<m;++i){
        I q=v[i+m*j];p[i+m*(2*j)]=q.lo;p[i+m*(2*j+1)]=q.hi;
    }return x;
}

void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    if(nrhs!=10||nlhs>1)
        mexErrMsgIdAndTxt("AF7R54:Arity",
            "R=mex(tlo,thi,t0,compStart,compEnd,A,b,activeIndex,signTag,candidateIndex)");
    double tlo=scalar(prhs[0],"tlo"),thi=scalar(prhs[1],"thi");
    double t0=scalar(prhs[2],"t0"),cs=scalar(prhs[3],"componentStart"),ce=scalar(prhs[4],"componentEnd");
    if(!(tlo<=thi&&tlo>ce&&cs>t0&&ce>cs))
        mexErrMsgIdAndTxt("AF7R54:Domain","invalid interval/component domain");

    const mxArray*Ain=prhs[5],*bin=prhs[6];
    if(!mxIsDouble(Ain)||mxIsComplex(Ain)||mxGetN(Ain)!=6)
        mexErrMsgIdAndTxt("AF7R54:Rows","A must be m x 6");
    size_t m=mxGetM(Ain);
    if(!mxIsDouble(bin)||mxIsComplex(bin)||mxGetNumberOfElements(bin)!=m)
        mexErrMsgIdAndTxt("AF7R54:Rows","b size mismatch");
    int ai=(int)std::llround(scalar(prhs[7],"activeIndex"))-1;
    if(ai<0||ai>=(int)m)mexErrMsgIdAndTxt("AF7R54:Rows","active index invalid");
    double signTag=scalar(prhs[8],"signTag"),candIndex=scalar(prhs[9],"candidateIndex");
    const double*A=mxGetPr(Ain),*b=mxGetPr(bin);

    I H[4]={cI(0),cI(0),cI(0),cI(0)};
    J8 J0[2],J1[2];
    for(int k=0;k<2;++k)for(int r=0;r<8;++r){J0[k][r]=cI(0);J1[k][r]=cI(0);}
    J1[0][0]=cI(1);J1[1][1]=cI(1);
    add_transition(H,J0,J1,cI(cs-t0));

    for(int k=0;k<2;++k)for(int r=0;r<8;++r){J0[k][r]=cI(0);J1[k][r]=cI(0);}
    double D0=ce-cs;
    J0[0][0]=cI(1);J0[1][0]=cI(D0);J0[1][1]=cI(1);
    add_transition(H,J0,J1,mk(tlo-ce,thi-ce));

    I det=sub(mul(H[0],H[3]),mul(H[1],H[2]));
    bool spd=H[0].lo>0&&H[3].lo>0&&det.lo>0;
    I inv00=divi(H[3],det),inv01=neg(divi(H[1],det));
    I inv10=neg(divi(H[2],det)),inv11=divi(H[0],det);

    std::vector<I>x(6),z(6);
    for(int d=0;d<3;++d){
        I ap=cI(A[ai+m*d]),av=cI(A[ai+m*(3+d)]);
        x[d]=add(mul(inv00,ap),mul(inv01,av));
        x[3+d]=add(mul(inv10,ap),mul(inv11,av));
    }
    I den=cI(0);
    for(int j=0;j<6;++j)den=add(den,mul(cI(A[ai+m*j]),x[j]));
    I lam=divi(cI(b[ai]),den);
    for(int j=0;j<6;++j)z[j]=mul(x[j],lam);

    std::vector<I>slack(m);
    double minInactiveLB=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<m;++i){
        I az=cI(0);
        for(int j=0;j<6;++j)az=add(az,mul(cI(A[i+m*j]),z[j]));
        slack[i]=sub(az,cI(b[i]));
        if((int)i!=ai)minInactiveLB=std::min(minInactiveLB,slack[i].lo);
    }
    bool lambdaPositive=lam.lo>0;
    bool inactiveStrict=(m<=1)||minInactiveLB>0;
    bool closed=spd&&lambdaPositive&&inactiveStrict;
    I cost=mul(cI(0.5*b[ai]),lam);

    const char*fn[]={"closed","spd","lambda_positive","inactive_strict","h00","h01","h10","h11",
        "det","lambda","z_interval","slack_interval","min_inactive_slack_lb","cost_interval",
        "t_interval","active_index","sign_tag","candidate_index"};
    mxArray*R=mxCreateStructMatrix(1,1,sizeof(fn)/sizeof(fn[0]),fn);
    auto set=[&](const char*n,mxArray*v){mxSetField(R,0,n,v);};
    set("closed",mxCreateLogicalScalar(closed));set("spd",mxCreateLogicalScalar(spd));
    set("lambda_positive",mxCreateLogicalScalar(lambdaPositive));set("inactive_strict",mxCreateLogicalScalar(inactiveStrict));
    set("h00",iout(H[0]));set("h01",iout(H[1]));set("h10",iout(H[2]));set("h11",iout(H[3]));
    set("det",iout(det));set("lambda",iout(lam));
    set("z_interval",imat(z,6,1));set("slack_interval",imat(slack,m,1));
    set("min_inactive_slack_lb",mxCreateDoubleScalar(minInactiveLB));set("cost_interval",iout(cost));
    set("t_interval",iout(mk(tlo,thi)));set("active_index",mxCreateDoubleScalar(ai+1));
    set("sign_tag",mxCreateDoubleScalar(signTag));set("candidate_index",mxCreateDoubleScalar(candIndex));
    plhs[0]=R;
}
