#include "mex.h"
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <string>

/*
 AF7R57F V3 stable-derivative outward interval Stage-B evaluator.

 Source-bound native algebra:
   af7r56_trace_core.hpp
   SHA-256 = 9f1c7054bbac0dadc2e82fbb107183937143b39431d8996ee13a620cc62ef821

 Bound formulas:
   scalar_control_from_jets : native lines 286..303
   vector_control_from_jets : native lines 304..310
   deriv_poly               : native lines 262..272
   build_candidate          : native lines 781..822
   make_vleaf               : native lines 876..892

 This file does NOT call production code and does NOT change production state.
 Every interval primitive is widened with std::nextafter.

 Numerical-stability identity:
   D_t^r( Restrict_[u0,u1](P) )
      == Restrict_[u0,u1]( D_t^r(P) )

 V2 restricted C0 first and then differentiated with the tiny physical leaf
 duration.  Exact common-tree endpoint unions can contain ULP-width slivers;
 outward noise in C0 was therefore divided by T_leaf^r and could explode to
 Inf.  V3 differentiates on the parent segment first and only then restricts
 each derivative polygon.  This is algebraically identical to the native
 source but avoids artificial 1/T_leaf^r amplification.
*/

static const double PINF = std::numeric_limits<double>::infinity();

struct I {
    double lo, hi;
};

static inline double dn(double x) {
    if (std::isnan(x)) return x;
    return std::nextafter(x, -PINF);
}
static inline double up(double x) {
    if (std::isnan(x)) return x;
    return std::nextafter(x, PINF);
}
static inline I point(double x) { return {x,x}; }

static inline I hull(double a,double b) {
    return {std::min(a,b),std::max(a,b)};
}
static inline I addi(I a,I b) { return {dn(a.lo+b.lo),up(a.hi+b.hi)}; }
static inline I subi(I a,I b) { return {dn(a.lo-b.hi),up(a.hi-b.lo)}; }

static inline I muli(I a,I b) {
    double p[4]={a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi};
    double l=PINF,h=-PINF;
    for(int i=0;i<4;++i){ l=std::min(l,dn(p[i])); h=std::max(h,up(p[i])); }
    return {l,h};
}
static inline I mulp(I a,double b) { return muli(a,point(b)); }

static inline I reci(I b) {
    if (b.lo<=0.0 && b.hi>=0.0) return {-PINF,PINF};
    double p1=1.0/b.lo, p2=1.0/b.hi;
    return {dn(std::min(p1,p2)),up(std::max(p1,p2))};
}
static inline I divi(I a,I b) { return muli(a,reci(b)); }

static inline I sqri(I a) {
    if(a.lo<=0.0 && a.hi>=0.0) {
        double h=std::max(a.lo*a.lo,a.hi*a.hi);
        return {0.0,up(h)};
    }
    double p1=a.lo*a.lo,p2=a.hi*a.hi;
    return {dn(std::min(p1,p2)),up(std::max(p1,p2))};
}
static inline I sqrti(I a) {
    double l=std::max(0.0,a.lo),h=std::max(0.0,a.hi);
    return {dn(std::sqrt(l)),up(std::sqrt(h))};
}
static inline I powi_pos(I a,int n) {
    if(n==0) return point(1.0);
    I r=point(1.0);
    for(int k=0;k<n;++k) r=muli(r,a);
    return r;
}

struct VI { I x,y,z; };
using PolyI = std::vector<VI>;

static inline VI vadd(VI a,VI b){ return {addi(a.x,b.x),addi(a.y,b.y),addi(a.z,b.z)}; }
static inline VI vsub(VI a,VI b){ return {subi(a.x,b.x),subi(a.y,b.y),subi(a.z,b.z)}; }
static inline VI vmul(VI a,I s){ return {muli(a.x,s),muli(a.y,s),muli(a.z,s)}; }
static inline VI vmulp(VI a,double s){ return {mulp(a.x,s),mulp(a.y,s),mulp(a.z,s)}; }

static inline I normi(VI a) {
    I s=addi(addi(sqri(a.x),sqri(a.y)),sqri(a.z));
    return sqrti(s);
}
static inline I max_norm(const PolyI& P) {
    double lo=0.0,hi=0.0;
    for(const auto& v:P){
        I n=normi(v);
        lo=std::max(lo,n.lo);
        hi=std::max(hi,n.hi);
    }
    return {lo,hi};
}

static inline double choose_int(int n,int k) {
    if(k<0||k>n) return 0.0;
    double v=1.0;
    for(int i=1;i<=k;++i) v*=double(n-k+i)/double(i);
    return v;
}

static std::vector<I> scalar_control_from_jets_I(
        const std::array<I,8>& j0,const std::array<I,8>& j1,I T){
    std::array<I,8> ds,de;
    ds[0]=j0[0]; de[0]=j1[0];
    double ff=1.0;
    for(int r=1;r<=7;++r){
        ff*=16-r;
        I tr=powi_pos(T,r);
        ds[r]=divi(muli(j0[r],tr),point(ff));
        de[r]=divi(muli(j1[r],tr),point(ff));
    }
    std::vector<I>P(16,point(0.0));
    for(int k=0;k<=7;++k){
        I v0=point(0.0),v1=point(0.0);
        for(int r=0;r<=k;++r){
            double bc=choose_int(k,r);
            v0=addi(v0,mulp(ds[r],bc));
            double sg=(r&1)?-bc:bc;
            v1=addi(v1,mulp(de[r],sg));
        }
        P[k]=v0; P[15-k]=v1;
    }
    return P;
}

static PolyI vector_control_from_jets_I(
        const std::array<VI,8>& j0,const std::array<VI,8>& j1,I T){
    std::array<I,8>x0,x1,y0,y1,z0,z1;
    for(int r=0;r<8;++r){
        x0[r]=j0[r].x;x1[r]=j1[r].x;
        y0[r]=j0[r].y;y1[r]=j1[r].y;
        z0[r]=j0[r].z;z1[r]=j1[r].z;
    }
    auto X=scalar_control_from_jets_I(x0,x1,T);
    auto Y=scalar_control_from_jets_I(y0,y1,T);
    auto Z=scalar_control_from_jets_I(z0,z1,T);
    PolyI P(16);
    for(int i=0;i<16;++i) P[i]={X[i],Y[i],Z[i]};
    return P;
}

static PolyI deriv_poly_I(const PolyI&C,I T,int order=1){
    PolyI D=C;
    int deg=(int)D.size()-1;
    for(int r=0;r<order;++r){
        if(T.lo<=0.0 || D.size()<2) return {};
        PolyI E(D.size()-1);
        I fac=divi(point(double(deg)),T);
        for(size_t i=0;i+1<D.size();++i) E[i]=vmul(vsub(D[i+1],D[i]),fac);
        D.swap(E);--deg;
    }
    return D;
}

static std::pair<PolyI,PolyI> split_at_I(const PolyI&C,I u){
    const size_t m=C.size();
    PolyI L(m),R(m),T=C;
    L[0]=T[0];R[m-1]=T[m-1];
    I oneMinus=subi(point(1.0),u);
    for(size_t k=1;k<m;++k){
        PolyI U(T.size()-1);
        for(size_t i=0;i+1<T.size();++i)
            U[i]=vadd(vmul(T[i],oneMinus),vmul(T[i+1],u));
        T.swap(U);L[k]=T.front();R[m-1-k]=T.back();
    }
    return {L,R};
}

static PolyI restrict_I(const PolyI&C,double u0,double u1){
    if(u0==0.0 && u1==1.0) return C;
    I U1=point(u1);
    auto s1=split_at_I(C,U1);
    if(u0==0.0) return s1.first;
    I alpha=divi(point(u0),point(u1));
    auto s2=split_at_I(s1.first,alpha);
    return s2.second;
}

static VI decasteljau_half_I(PolyI C){
    I h=point(0.5);
    I h2=point(0.5);
    while(C.size()>1){
        PolyI D(C.size()-1);
        for(size_t i=0;i+1<C.size();++i)
            D[i]=vadd(vmul(C[i],h2),vmul(C[i+1],h));
        C.swap(D);
    }
    return C.front();
}

static VI zeroVI(){ return {point(0),point(0),point(0)}; }

struct CandidateI {
    std::vector<I> seg0,seg1;
    std::vector<PolyI> cp;
};

static CandidateI build_candidate_I(
        const std::vector<double>& cs,const std::vector<double>& ce,
        double t0,I t1,const std::vector<I>& z){
    const int nc=(int)cs.size();
    const int nslot=2*nc;
    if((int)z.size()!=3*nslot) throw std::runtime_error("z interval size mismatch");

    std::vector<VI> pos(nc),vel(nc);
    for(int j=0;j<nc;++j){
        pos[j]={z[3*(2*j)],z[3*(2*j)+1],z[3*(2*j)+2]};
        vel[j]={z[3*(2*j+1)],z[3*(2*j+1)+1],z[3*(2*j+1)+2]};
    }

    CandidateI C;
    std::array<VI,8>a,b;
    for(auto &q:a) q=zeroVI();
    for(auto &q:b) q=zeroVI();

    // start transition
    for(auto &q:a) q=zeroVI();
    for(auto &q:b) q=zeroVI();
    b[0]=pos[0]; b[1]=vel[0];
    C.seg0.push_back(point(t0));
    C.seg1.push_back(point(cs[0]));
    C.cp.push_back(vector_control_from_jets_I(a,b,point(cs[0]-t0)));

    for(int j=0;j<nc;++j){
        double T=ce[j]-cs[j];
        PolyI p(16);
        for(int k=0;k<16;++k){
            double s=T*(double(k)/15.0);
            p[k]=vadd(pos[j],vmulp(vel[j],s));
        }
        C.seg0.push_back(point(cs[j]));
        C.seg1.push_back(point(ce[j]));
        C.cp.push_back(p);

        if(j<nc-1){
            for(auto &q:a) q=zeroVI();
            for(auto &q:b) q=zeroVI();
            a[0]=vadd(pos[j],vmulp(vel[j],ce[j]-cs[j]));a[1]=vel[j];
            b[0]=pos[j+1];b[1]=vel[j+1];
            double Tt=cs[j+1]-ce[j];
            C.seg0.push_back(point(ce[j]));
            C.seg1.push_back(point(cs[j+1]));
            C.cp.push_back(vector_control_from_jets_I(a,b,point(Tt)));
        }
    }

    // exit to end: only segment with t1 interval dependence
    int j=nc-1;
    for(auto &q:a) q=zeroVI();
    for(auto &q:b) q=zeroVI();
    a[0]=vadd(pos[j],vmulp(vel[j],ce[j]-cs[j]));a[1]=vel[j];
    I Tlast=subi(t1,point(ce[j]));
    C.seg0.push_back(point(ce[j]));
    C.seg1.push_back(t1);
    C.cp.push_back(vector_control_from_jets_I(a,b,Tlast));

    return C;
}

static inline double scalar(const mxArray* a,const char* what){
    if(!mxIsDouble(a)||mxIsComplex(a)||mxGetNumberOfElements(a)!=1)
        mexErrMsgIdAndTxt("AF7R57F:Input","%s must be real scalar double",what);
    return mxGetScalar(a);
}
static std::vector<double> vec(const mxArray*a,const char*what){
    if(!mxIsDouble(a)||mxIsComplex(a))
        mexErrMsgIdAndTxt("AF7R57F:Input","%s must be real double",what);
    size_t n=mxGetNumberOfElements(a);double*p=mxGetPr(a);
    return std::vector<double>(p,p+n);
}
static std::vector<I> intervals(const mxArray*a,const char*what){
    if(!mxIsDouble(a)||mxIsComplex(a)||mxGetN(a)!=2)
        mexErrMsgIdAndTxt("AF7R57F:Input","%s must be N x 2",what);
    mwSize n=mxGetM(a);double*p=mxGetPr(a);
    std::vector<I> v(n);
    for(mwSize i=0;i<n;++i){
        double lo=p[i],hi=p[i+n];
        if(!(std::isfinite(lo)&&std::isfinite(hi)&&lo<=hi))
            mexErrMsgIdAndTxt("AF7R57F:Input","invalid interval in %s",what);
        v[i]={lo,hi};
    }
    return v;
}

struct TreeRow{ int seg; double u0,u1; };
static std::vector<TreeRow> tree_rows(const mxArray*a){
    if(!mxIsDouble(a)||mxIsComplex(a)||mxGetN(a)!=3)
        mexErrMsgIdAndTxt("AF7R57F:Input","tree must be N x 3");
    mwSize n=mxGetM(a);double*p=mxGetPr(a);
    std::vector<TreeRow> T(n);
    for(mwSize i=0;i<n;++i){
        double sg=p[i],u0=p[i+n],u1=p[i+2*n];
        if(sg!=std::floor(sg)||u0<0||u1>1||!(u0<u1))
            mexErrMsgIdAndTxt("AF7R57F:Input","invalid tree row");
        T[i]={(int)sg,u0,u1};
    }
    return T;
}

static void segment_time_I(
        int seg,const std::vector<double>&cs,const std::vector<double>&ce,
        double t0,I t1,I&sa,I&sb){
    int nc=(int)cs.size();
    if(seg<0||seg>2*nc) throw std::runtime_error("segment index");
    if(seg==0){sa=point(t0);sb=point(cs[0]);return;}
    if(seg==2*nc){sa=point(ce[nc-1]);sb=t1;return;}
    if(seg&1){
        int j=(seg-1)/2;sa=point(cs[j]);sb=point(ce[j]);return;
    }
    int j=seg/2-1;sa=point(ce[j]);sb=point(cs[j+1]);
}

static I leaf_tc_I(
        const TreeRow&r,const std::vector<double>&cs,const std::vector<double>&ce,
        double t0,I t1){
    I sa,sb;segment_time_I(r.seg,cs,ce,t0,t1,sa,sb);
    I dur=subi(sb,sa);
    I m=divi(addi(point(r.u0),point(r.u1)),point(2.0));
    return addi(sa,muli(dur,m));
}

static mxArray* mode_times(
        I t1,double t0,const std::vector<double>&cs,const std::vector<double>&ce,
        const std::vector<TreeRow>&T){
    const char* fields[]={"tc_interval","tc_center","tc_radius"};
    mxArray* out=mxCreateStructMatrix(1,1,3,fields);
    mwSize n=T.size();
    mxArray* A=mxCreateDoubleMatrix(n,2,mxREAL);
    mxArray* C=mxCreateDoubleMatrix(n,1,mxREAL);
    mxArray* R=mxCreateDoubleMatrix(n,1,mxREAL);
    double*pa=mxGetPr(A),*pc=mxGetPr(C),*pr=mxGetPr(R);

    for(mwSize i=0;i<n;++i){
        I ti=leaf_tc_I(T[i],cs,ce,t0,t1);
        double c=ti.lo + 0.5*(ti.hi-ti.lo);
        double rr=up(std::max(std::fabs(c-ti.lo),std::fabs(ti.hi-c)));
        pa[i]=ti.lo;pa[i+n]=ti.hi;pc[i]=c;pr[i]=rr;
    }
    mxSetField(out,0,"tc_interval",A);
    mxSetField(out,0,"tc_center",C);
    mxSetField(out,0,"tc_radius",R);
    return out;
}

static I row_vec_interval(const mxArray*A,mwSize col,mwSize row,double rad){
    mwSize m=mxGetM(A),n=mxGetN(A);
    if(m!=3) mexErrMsgIdAndTxt("AF7R57F:Input","reference center must be 3 x N");
    if(col>=n||row>=3) mexErrMsgIdAndTxt("AF7R57F:Input","reference center index");
    double x=mxGetPr(A)[row+3*col];
    return {dn(x-rad),up(x+rad)};
}

static mxArray* mode_eval(
        const std::vector<I>&z,I t1,double t0,
        const std::vector<double>&cs,const std::vector<double>&ce,
        const std::vector<TreeRow>&T,
        double baseA,double baseJ,double g,double cableL,
        const mxArray*VC,const mxArray*AC,
        const std::vector<double>&vRad,const std::vector<double>&aRad,
        double limit){

    if(baseA<0||baseJ<0||g<=0||cableL<=0)
        mexErrMsgIdAndTxt("AF7R57F:Input","invalid physical/bound parameter");
    if(vRad.size()!=T.size()||aRad.size()!=T.size())
        mexErrMsgIdAndTxt("AF7R57F:Input","radius length mismatch");
    if(mxGetM(VC)!=3||mxGetN(VC)!=T.size()||mxGetM(AC)!=3||mxGetN(AC)!=T.size())
        mexErrMsgIdAndTxt("AF7R57F:Input","reference centers must be 3 x leaf_count");

    CandidateI C=build_candidate_I(cs,ce,t0,t1,z);

    const char* fields[]={
        "seg","u0","u1","vl","al","jl","sigma","qdot","vq","G","class_code"
    };
    mxArray*out=mxCreateStructMatrix(1,1,11,fields);
    mwSize n=T.size();
    auto mat=[&](const char*fn,int cols){
        mxArray*A=mxCreateDoubleMatrix(n,cols,mxREAL);mxSetField(out,0,fn,A);return A;
    };
    mxArray*Mseg=mat("seg",1),*Mu0=mat("u0",1),*Mu1=mat("u1",1);
    mxArray*Mvl=mat("vl",2),*Mal=mat("al",2),*Mjl=mat("jl",2);
    mxArray*Msig=mat("sigma",2),*Mqd=mat("qdot",2),*Mvq=mat("vq",2),*MG=mat("G",2);
    mxArray*Mcl=mat("class_code",1);

    for(mwSize k=0;k<n;++k){
        const TreeRow&r=T[k];
        if(r.seg<0||r.seg>=(int)C.cp.size())
            mexErrMsgIdAndTxt("AF7R57F:Input","tree segment outside candidate");

        I segT=subi(C.seg1[r.seg],C.seg0[r.seg]);
        I leafT=mulp(segT,r.u1-r.u0);
        I h=mulp(leafT,0.5);

        // EXACT BEZIER IDENTITY:
        // derivative after exact subcurve restriction equals restriction of
        // the parent derivative polynomial.  Use the latter interval form to
        // avoid dividing de-Casteljau outward noise by an ULP-scale leafT.
        PolyI D1=deriv_poly_I(C.cp[r.seg],segT,1);
        PolyI D2=deriv_poly_I(C.cp[r.seg],segT,2);
        PolyI D3=deriv_poly_I(C.cp[r.seg],segT,3);
        if(D1.empty()||D2.empty()||D3.empty())
            mexErrMsgIdAndTxt("AF7R57F:Eval","invalid parent derivative interval");

        PolyI C1=restrict_I(D1,r.u0,r.u1);
        PolyI C2=restrict_I(D2,r.u0,r.u1);
        PolyI C3=restrict_I(D3,r.u0,r.u1);
        if(C1.empty()||C2.empty()||C3.empty())
            mexErrMsgIdAndTxt("AF7R57F:Eval","invalid restricted derivative interval");

        double vr=vRad[k],ar=aRad[k];
        VI v0={row_vec_interval(VC,k,0,vr),row_vec_interval(VC,k,1,vr),row_vec_interval(VC,k,2,vr)};
        VI a0={row_vec_interval(AC,k,0,ar),row_vec_interval(AC,k,1,ar),row_vec_interval(AC,k,2,ar)};

        PolyI V=C1,A=C2;
        for(auto&v:V) v=vadd(v,v0);
        for(auto&a:A) a=vadd(a,a0);

        I vl=addi(max_norm(V),mulp(h,baseA));
        I al=addi(max_norm(A),mulp(h,baseJ));
        I jl=addi(point(baseJ),max_norm(C3));

        VI d2=decasteljau_half_I(C2);
        VI sm=vadd(vadd(a0,d2),VI{point(0),point(0),point(g)});
        I sigma=subi(normi(sm),muli(jl,h));

        I qdot;
        if(sigma.lo>0.0) {
            qdot=divi(jl,sigma);
        } else if(sigma.hi<=0.0) {
            qdot={PINF,PINF};
        } else {
            qdot={0.0,PINF};
        }
        I vq=addi(vl,mulp(qdot,cableL));
        I GG=subi(vq,point(limit));

        int cls=0;
        if(sigma.lo>0.0 && GG.hi<0.0) cls=-1;       // CERTIFIED_SAFE
        else if(GG.lo>0.0) cls=1;                  // CERTIFIED_STAGEB_VIOLATION

        mxGetPr(Mseg)[k]=r.seg;mxGetPr(Mu0)[k]=r.u0;mxGetPr(Mu1)[k]=r.u1;
        auto put2=[&](mxArray*M,I x){double*p=mxGetPr(M);p[k]=x.lo;p[k+n]=x.hi;};
        put2(Mvl,vl);put2(Mal,al);put2(Mjl,jl);put2(Msig,sigma);
        put2(Mqd,qdot);put2(Mvq,vq);put2(MG,GG);
        mxGetPr(Mcl)[k]=cls;
    }
    return out;
}

void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    try{
        if(nlhs>1) mexErrMsgIdAndTxt("AF7R57F:Output","one output only");
        if(nrhs<1) mexErrMsgIdAndTxt("AF7R57F:Input","mode required");
        int mode=(int)scalar(prhs[0],"mode");

        if(mode==0){
            if(nrhs!=7) mexErrMsgIdAndTxt("AF7R57F:Input","mode0: mode,tI,t0,cs,ce,tree,reserved");
            auto ti=intervals(prhs[1],"tI"); if(ti.size()!=1) mexErrMsgIdAndTxt("AF7R57F:Input","tI 1x2");
            double t0=scalar(prhs[2],"t0");
            auto cs=vec(prhs[3],"component_start"),ce=vec(prhs[4],"component_end");
            auto T=tree_rows(prhs[5]);
            (void)prhs[6]; // reserved/version scalar
            if(cs.size()!=ce.size()||cs.empty()) mexErrMsgIdAndTxt("AF7R57F:Input","component arrays");
            plhs[0]=mode_times(ti[0],t0,cs,ce,T);return;
        }

        if(mode==1){
            if(nrhs!=17) mexErrMsgIdAndTxt("AF7R57F:Input",
                "mode1 expects 17 inputs");
            auto z=intervals(prhs[1],"zI");
            auto ti=intervals(prhs[2],"tI"); if(ti.size()!=1) mexErrMsgIdAndTxt("AF7R57F:Input","tI");
            double t0=scalar(prhs[3],"t0");
            auto cs=vec(prhs[4],"component_start"),ce=vec(prhs[5],"component_end");
            auto T=tree_rows(prhs[6]);
            double baseA=scalar(prhs[7],"baseA"),baseJ=scalar(prhs[8],"baseJ");
            double g=scalar(prhs[9],"g"),L=scalar(prhs[10],"L");
            const mxArray*VC=prhs[11];const mxArray*AC=prhs[12];
            auto vr=vec(prhs[13],"vRad"),ar=vec(prhs[14],"aRad");
            double lim=scalar(prhs[15],"limit");
            (void)prhs[16]; // reserved/version scalar
            if(cs.size()!=ce.size()||cs.empty()) mexErrMsgIdAndTxt("AF7R57F:Input","component arrays");
            plhs[0]=mode_eval(z,ti[0],t0,cs,ce,T,baseA,baseJ,g,L,VC,AC,vr,ar,lim);return;
        }

        mexErrMsgIdAndTxt("AF7R57F:Mode","unknown mode");
    } catch(const std::exception&e){
        mexErrMsgIdAndTxt("AF7R57F:Exception","%s",e.what());
    }
}
