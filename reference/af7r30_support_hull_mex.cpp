#include "mex.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <cstdint>

static inline double dot3(const double* a,const double* b){
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static inline double norm3(const double* a){ return std::sqrt(dot3(a,a)); }
static inline void sub3(const double* a,const double* b,double* y){
    y[0]=a[0]-b[0]; y[1]=a[1]-b[1]; y[2]=a[2]-b[2];
}
static inline void add_scaled3(const double* a,double s,const double* b,double* y){
    y[0]=a[0]+s*b[0]; y[1]=a[1]+s*b[1]; y[2]=a[2]+s*b[2];
}
static inline void Rt_mul(const double* R,const double* x,double* y){
    for(int j=0;j<3;++j)
        y[j]=R[0+3*j]*x[0]+R[1+3*j]*x[1]+R[2+3*j]*x[2];
}
static inline void R_mul(const double* R,const double* x,double* y){
    for(int i=0;i<3;++i)
        y[i]=R[i+3*0]*x[0]+R[i+3*1]*x[1]+R[i+3*2]*x[2];
}
static inline bool finite3(const double* x){
    return std::isfinite(x[0])&&std::isfinite(x[1])&&std::isfinite(x[2]);
}
static inline bool support_point(const double* x,const double* c,const double* R,
                                 const double* a,double rho,const double* n,
                                 double& margin){
    double nn=norm3(n);
    if(!(std::isfinite(nn)&&nn>0.0)) return false;
    double rtn[3]; Rt_mul(R,n,rtn);
    double ar[3]={a[0]*rtn[0],a[1]*rtn[1],a[2]*rtn[2]};
    double h=dot3(n,c)+norm3(ar);
    double raw=dot3(n,x)-h-rho*nn;
    margin=raw/nn;
    return std::isfinite(margin);
}
static inline bool support_segment(const double* p0,const double* p1,const double* c,
                                   const double* R,const double* a,double rho,
                                   const double* n,double& margin){
    double nn=norm3(n);
    if(!(std::isfinite(nn)&&nn>0.0)) return false;
    double rtn[3]; Rt_mul(R,n,rtn);
    double ar[3]={a[0]*rtn[0],a[1]*rtn[1],a[2]*rtn[2]};
    double h=dot3(n,c)+norm3(ar);
    double proj=std::min(dot3(n,p0),dot3(n,p1));
    double raw=proj-h-rho*nn;
    margin=raw/nn;
    return std::isfinite(margin);
}
static void require_vec3(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetNumberOfElements(x)!=3)
        mexErrMsgIdAndTxt("AF7R30:Input","%s must be a real double 3-vector.",name);
}
static void require_R(const mxArray* x){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetM(x)!=3||mxGetN(x)!=3)
        mexErrMsgIdAndTxt("AF7R30:Input","R must be real double 3x3.");
}
static double scalar(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetNumberOfElements(x)!=1)
        mexErrMsgIdAndTxt("AF7R30:Input","%s must be a real double scalar.",name);
    double v=mxGetScalar(x);
    if(!std::isfinite(v))
        mexErrMsgIdAndTxt("AF7R30:Input","%s must be finite.",name);
    return v;
}

static void op_point(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    if(nrhs!=6) mexErrMsgIdAndTxt("AF7R30:Arity","point op expects 5 data inputs.");
    require_vec3(prhs[1],"x"); require_vec3(prhs[2],"c"); require_R(prhs[3]);
    require_vec3(prhs[4],"a"); double rho=scalar(prhs[5],"rho");
    const double* x=mxGetPr(prhs[1]); const double* c=mxGetPr(prhs[2]);
    const double* R=mxGetPr(prhs[3]); const double* a=mxGetPr(prhs[4]);
    if(!finite3(x)||!finite3(c)||!finite3(a)||a[0]<=0||a[1]<=0||a[2]<=0)
        mexErrMsgIdAndTxt("AF7R30:Input","invalid point/ellipsoid input.");

    double d[3]; sub3(x,c,d);
    double y[3]; Rt_mul(R,d,y);
    double ya2[3]={y[0]/(a[0]*a[0]),y[1]/(a[1]*a[1]),y[2]/(a[2]*a[2])};
    double n0[3]; R_mul(R,ya2,n0);
    double n1[3]={d[0],d[1],d[2]};

    double best=-std::numeric_limits<double>::infinity(),m;
    bool ok=false;
    if(support_point(x,c,R,a,rho,n0,m)){best=std::max(best,m);ok=true;}
    if(support_point(x,c,R,a,rho,n1,m)){best=std::max(best,m);ok=true;}
    bool pass=ok&&std::isfinite(best)&&best>0.0;

    plhs[0]=mxCreateDoubleScalar(best);
    if(nlhs>1) plhs[1]=mxCreateLogicalScalar(pass);
}

static void op_segment(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    if(nrhs!=7) mexErrMsgIdAndTxt("AF7R30:Arity","segment op expects 6 data inputs.");
    require_vec3(prhs[1],"p0"); require_vec3(prhs[2],"p1"); require_vec3(prhs[3],"c");
    require_R(prhs[4]); require_vec3(prhs[5],"a"); double rho=scalar(prhs[6],"rho");
    const double* p0=mxGetPr(prhs[1]); const double* p1=mxGetPr(prhs[2]);
    const double* c=mxGetPr(prhs[3]); const double* R=mxGetPr(prhs[4]);
    const double* a=mxGetPr(prhs[5]);
    if(!finite3(p0)||!finite3(p1)||!finite3(c)||!finite3(a)||a[0]<=0||a[1]<=0||a[2]<=0)
        mexErrMsgIdAndTxt("AF7R30:Input","invalid segment/ellipsoid input.");

    double d0[3],d1[3],z0[3],z1[3],s[3];
    sub3(p0,c,d0); sub3(p1,c,d1);
    Rt_mul(R,d0,z0); Rt_mul(R,d1,z1);
    for(int i=0;i<3;++i){z0[i]/=a[i];z1[i]/=a[i];s[i]=z1[i]-z0[i];}
    double ss=dot3(s,s);
    double lam=0.0;
    if(ss>0.0){
        lam=-dot3(z0,s)/ss;
        if(lam<0.0)lam=0.0; else if(lam>1.0)lam=1.0;
    }
    double dp[3]={p1[0]-p0[0],p1[1]-p0[1],p1[2]-p0[2]},xstar[3];
    add_scaled3(p0,lam,dp,xstar);

    double Y[3][3]={{xstar[0],xstar[1],xstar[2]},
                    {p0[0],p0[1],p0[2]},
                    {p1[0],p1[1],p1[2]}};
    double best=-std::numeric_limits<double>::infinity(),m;
    bool ok=false;
    for(int k=0;k<3;++k){
        double dy[3]; sub3(Y[k],c,dy);
        double yr[3]; Rt_mul(R,dy,yr);
        double ya2[3]={yr[0]/(a[0]*a[0]),yr[1]/(a[1]*a[1]),yr[2]/(a[2]*a[2])};
        double n0[3]; R_mul(R,ya2,n0);
        double n1[3]={dy[0],dy[1],dy[2]};
        if(support_segment(p0,p1,c,R,a,rho,n0,m)){best=std::max(best,m);ok=true;}
        if(support_segment(p0,p1,c,R,a,rho,n1,m)){best=std::max(best,m);ok=true;}
    }
    bool pass=ok&&std::isfinite(best)&&best>0.0;
    plhs[0]=mxCreateDoubleScalar(best);
    if(nlhs>1) plhs[1]=mxCreateLogicalScalar(pass);
}

static double spacing(double x){
    return std::nextafter(x,std::numeric_limits<double>::infinity())-x;
}
static void op_hull(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    if(nrhs!=3) mexErrMsgIdAndTxt("AF7R30:Arity","hull op expects S,B.");
    if(!mxIsDouble(prhs[1])||mxIsComplex(prhs[1])||
       !mxIsDouble(prhs[2])||mxIsComplex(prhs[2]))
        mexErrMsgIdAndTxt("AF7R30:Input","S/B must be real double.");
    size_t n=mxGetNumberOfElements(prhs[1]);
    if(n==0||mxGetNumberOfElements(prhs[2])!=n)
        mexErrMsgIdAndTxt("AF7R30:Input","S/B size mismatch.");
    const double* S=mxGetPr(prhs[1]); const double* B=mxGetPr(prhs[2]);

    struct P{double s,b;mwIndex idx;};
    std::vector<P> v; v.reserve(n);
    for(mwIndex i=0;i<(mwIndex)n;++i){
        if(!std::isfinite(S[i])||!std::isfinite(B[i]))
            mexErrMsgIdAndTxt("AF7R30:Input","S/B must be finite.");
        v.push_back({S[i],B[i],i});
    }
    std::stable_sort(v.begin(),v.end(),[](const P&x,const P&y){return x.s<y.s;});

    std::vector<P> u; u.reserve(n);
    for(size_t k=0;k<v.size();){
        size_t j=k; P best=v[k];
        while(j+1<v.size() && v[j+1].s==v[k].s){
            ++j; if(v[j].b>best.b) best=v[j];
        }
        u.push_back(best); k=j+1;
    }

    std::vector<size_t> H; H.reserve(u.size());
    for(size_t k=0;k<u.size();++k){
        while(H.size()>=2){
            size_t i1=H[H.size()-2],i2=H[H.size()-1],i3=k;
            double dx1=u[i2].s-u[i1].s, dy1=u[i2].b-u[i1].b;
            double dx2=u[i3].s-u[i2].s, dy2=u[i3].b-u[i2].b;
            double t1=dx1*dy2, t2=dy1*dx2, cr=t1-t2;
            double scale=std::max({1.0,std::fabs(t1),std::fabs(t2),std::fabs(dx1),
                                  std::fabs(dx2),std::fabs(dy1),std::fabs(dy2)});
            double guard=128.0*spacing(scale);
            if(cr>guard || cr==0.0) H.pop_back(); else break;
        }
        H.push_back(k);
    }

    std::vector<double> out; out.reserve(H.size());
    for(size_t k:H) out.push_back((double)u[k].idx+1.0);
    std::sort(out.begin(),out.end());
    plhs[0]=mxCreateDoubleMatrix(out.size(),1,mxREAL);
    double* po=mxGetPr(plhs[0]);
    for(size_t i=0;i<out.size();++i)po[i]=out[i];
}

void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    if(nrhs<1||!mxIsDouble(prhs[0])||mxGetNumberOfElements(prhs[0])!=1)
        mexErrMsgIdAndTxt("AF7R30:Input","first input must be numeric op code.");
    int op=(int)mxGetScalar(prhs[0]);
    if(op==0) op_point(nlhs,plhs,nrhs,prhs);
    else if(op==1) op_segment(nlhs,plhs,nrhs,prhs);
    else if(op==2) op_hull(nlhs,plhs,nrhs,prhs);
    else mexErrMsgIdAndTxt("AF7R30:Op","unknown op code.");
}
