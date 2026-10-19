#include "mex.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <algorithm>

static void fail(const char* id, const char* msg) { mexErrMsgIdAndTxt(id, "%s", msg); }

static const mxArray* fld(const mxArray* s, const char* name) {
    if (!mxIsStruct(s) || mxGetNumberOfElements(s) != 1) fail("AF7R66:Struct", "Expected scalar struct input.");
    const mxArray* v = mxGetField(s,0,name);
    if (!v) mexErrMsgIdAndTxt("AF7R66:Field", "Missing struct field: %s", name);
    return v;
}
static const mxArray* fld2(const mxArray* s, const char* a, const char* b) { return fld(fld(s,a),b); }
static double scalar(const mxArray* a, const char* what) {
    if (!mxIsDouble(a) || mxIsComplex(a) || mxGetNumberOfElements(a)!=1) mexErrMsgIdAndTxt("AF7R66:Scalar", "%s must be real scalar double.", what);
    return mxGetScalar(a);
}
static const double* vec(const mxArray* a, mwSize n, const char* what) {
    if (!mxIsDouble(a) || mxIsComplex(a) || mxGetNumberOfElements(a)!=n) mexErrMsgIdAndTxt("AF7R66:Vector", "%s size mismatch.", what);
    return mxGetDoubles(a);
}
static mxArray* col(const double* x, mwSize n) {
    mxArray* a=mxCreateDoubleMatrix(n,1,mxREAL); std::memcpy(mxGetDoubles(a),x,n*sizeof(double)); return a;
}
static mxArray* dscalar(double x) { return mxCreateDoubleScalar(x); }

static double matlab_unary(const char* fn, double x) {
    mxArray* in=mxCreateDoubleScalar(x); mxArray* out=nullptr;
    if (mexCallMATLAB(1,&out,1,&in,fn)!=0 || !out) { mxDestroyArray(in); fail("AF7R66:MATLABCall","MATLAB unary builtin call failed."); }
    double y=mxGetScalar(out); mxDestroyArray(out); mxDestroyArray(in); return y;
}
static double sqrtv(double x, int mode) { return mode==1 ? matlab_unary("sqrt",x) : std::sqrt(x); }
static double sinv(double x, int mode) { return mode==1 ? matlab_unary("sin",x) : std::sin(x); }
static double cosv(double x, int mode) { return mode==1 ? matlab_unary("cos",x) : std::cos(x); }
static double tanv(double x, int mode) { return mode==1 ? matlab_unary("tan",x) : std::tan(x); }

static double stable_norm(const double* x, mwSize n) {
    double scale=0.0, ssq=1.0;
    for (mwSize i=0;i<n;++i) {
        double ax=std::fabs(x[i]);
        if (ax!=0.0) {
            if (scale<ax) { double r=scale/ax; ssq=1.0+ssq*r*r; scale=ax; }
            else { double r=ax/scale; ssq += r*r; }
        }
    }
    return scale==0.0 ? 0.0 : scale*std::sqrt(ssq);
}
static double matlab_norm(const double* x, mwSize n) {
    mxArray* in=col(x,n); mxArray* out=nullptr;
    if (mexCallMATLAB(1,&out,1,&in,"norm")!=0 || !out) { mxDestroyArray(in); fail("AF7R66:MATLABCall","MATLAB norm call failed."); }
    double y=mxGetScalar(out); mxDestroyArray(out); mxDestroyArray(in); return y;
}
static double normv(const double* x, mwSize n, int mode) { return mode==1 ? matlab_norm(x,n) : stable_norm(x,n); }

static void cross_native(const double* a,const double* b,double* c) {
    c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0];
}
static void crossv(const double* a,const double* b,double* c,int mode) {
    if (mode!=1) { cross_native(a,b,c); return; }
    mxArray* in[2]={col(a,3),col(b,3)}; mxArray* out=nullptr;
    if (mexCallMATLAB(1,&out,2,in,"cross")!=0 || !out) { mxDestroyArray(in[0]); mxDestroyArray(in[1]); fail("AF7R66:MATLABCall","MATLAB cross call failed."); }
    const double* p=vec(out,3,"cross output"); std::memcpy(c,p,3*sizeof(double));
    mxDestroyArray(out); mxDestroyArray(in[0]); mxDestroyArray(in[1]);
}
static void matvec3_native(const double* Acol,const double* x,double* y) {
    y[0]=(Acol[0]*x[0]+Acol[3]*x[1])+Acol[6]*x[2];
    y[1]=(Acol[1]*x[0]+Acol[4]*x[1])+Acol[7]*x[2];
    y[2]=(Acol[2]*x[0]+Acol[5]*x[1])+Acol[8]*x[2];
}
static void matvec3(const double* Acol,const double* x,double* y,int mode) {
    if (mode!=1) { matvec3_native(Acol,x,y); return; }
    mxArray* A=mxCreateDoubleMatrix(3,3,mxREAL); std::memcpy(mxGetDoubles(A),Acol,9*sizeof(double));
    mxArray* xv=col(x,3); mxArray* in[2]={A,xv}; mxArray* out=nullptr;
    if (mexCallMATLAB(1,&out,2,in,"mtimes")!=0 || !out) { mxDestroyArray(A); mxDestroyArray(xv); fail("AF7R66:MATLABCall","MATLAB mtimes call failed."); }
    const double* p=vec(out,3,"mtimes output"); std::memcpy(y,p,3*sizeof(double));
    mxDestroyArray(out); mxDestroyArray(A); mxDestroyArray(xv);
}

static double nextup_nonnegative(double x) {
    if (std::isnan(x) || x==std::numeric_limits<double>::infinity()) return x;
    if (!(x>=0.0)) fail("AF7R66:NextUp","nextfloat input must be nonnegative.");
    if (x==0.0) return std::numeric_limits<double>::denorm_min();
    std::uint64_t u; std::memcpy(&u,&x,sizeof(double)); ++u; double y; std::memcpy(&y,&u,sizeof(double)); return y;
}

static mxArray* mk_err3(const double* center,const double* radius) {
    const char* fn[]={"center","radius","abs_ub"}; mxArray* s=mxCreateStructMatrix(1,1,3,fn);
    double au[3]; for(int i=0;i<3;++i) au[i]=std::fabs(center[i])+radius[i];
    mxSetField(s,0,"center",col(center,3)); mxSetField(s,0,"radius",col(radius,3)); mxSetField(s,0,"abs_ub",col(au,3)); return s;
}
static mxArray* mk_bound3(const double* center,double radius,double trueub,double errub) {
    const char* fn[]={"center","radius","true_norm_ub","observer_error_norm_ub"}; mxArray* s=mxCreateStructMatrix(1,1,4,fn);
    mxSetField(s,0,"center",col(center,3)); mxSetField(s,0,"radius",dscalar(radius)); mxSetField(s,0,"true_norm_ub",dscalar(trueub)); mxSetField(s,0,"observer_error_norm_ub",dscalar(errub)); return s;
}

static mxArray* one_step(int mode,double h,const mxArray* uA,const mxArray* I,const mxArray* K0) {
    const double* u=vec(uA,4,"u");
    const mxArray* H=fld(I,"hidden");
    const double* q0=vec(fld(I,"q_abs_ub_current"),3,"I.q_abs_ub_current");
    double Pnorm=scalar(fld(I,"pT_norm_ub"),"I.pT_norm_ub");
    const double* mLint=vec(fld(I,"true_mL_interval"),2,"I.true_mL_interval");
    const double* Jd=vec(fld(K0,"Jdiag"),3,"K0.Jdiag");
    double m=scalar(fld(K0,"mass"),"K0.mass"), g=scalar(fld(K0,"gravity"),"K0.gravity"), L=scalar(fld(K0,"cableL"),"K0.cableL");
    if (!(Jd[0]>0&&Jd[1]>0&&Jd[2]>0&&std::isfinite(m)&&m>0&&std::isfinite(g)&&g>=0&&std::isfinite(L)&&L>0)) fail("AF7R66:Domain","physical parameter domain invalid.");
    double Jmin=std::min(Jd[0],std::min(Jd[1],Jd[2])); double Jmax=std::max(Jd[0],std::max(Jd[1],Jd[2]));
    double tauv[3]={u[1],u[2],u[3]}; double tauNorm=normv(tauv,3,mode); double u1abs=std::fabs(u[0]);
    double W0=scalar(fld(H,"w_norm_ub"),"H.w_norm_ub"), WL0=scalar(fld(H,"wL_norm_ub"),"H.wL_norm_ub"), VL0=scalar(fld(H,"vL_norm_ub"),"H.vL_norm_ub");
    double mLlb=mLint[0];
    double W0sq=W0*W0; double V0=(0.5*Jmax)*W0sq;
    double denJ=2.0*Jmin; double rootDen=sqrtv(denJ,mode); double sqrtVend=sqrtv(V0,mode)+(tauNorm*h)/rootDen;
    double VwEnd=sqrtVend*sqrtVend; double Wabs0=sqrtv((2.0*V0)/Jmin,mode); double WabsEnd=sqrtv((2.0*VwEnd)/Jmin,mode);
    double h2=h*h; double Iw=(Wabs0*h)+(tauNorm*h2)/(2.0*Jmin);
    double Theta=q0[1]+Iw; double pi=3.141592653589793238462643383279502884; bool pitchClosed=Theta<pi/2.0;
    double qAbs[3]={mxGetInf(),Theta,mxGetInf()}; bool fullClosed=false;
    if (pitchClosed) { double tt=tanv(Theta,mode); double ct=cosv(Theta,mode); qAbs[0]=q0[0]+(1.0+tt)*Iw; qAbs[2]=q0[2]+(1.0/ct)*Iw; fullClosed=(qAbs[0]<pi && qAbs[2]<pi); }
    double aWL=(Pnorm*u1abs)/(m*L); double WLEnd=WL0+aWL*h; double IWL=WL0*h+0.5*aWL*h2; volatile double pTSpan=Pnorm*IWL; (void)pTSpan;
    double den=m+mLlb; if (!(den>0.0)) fail("AF7R66:Domain","total mass lower bound invalid.");
    double P2=Pnorm*Pnorm; double Aconst=g+(P2*u1abs)/den; double P3=P2*Pnorm; double Aquad=((P3*m)*L)/den;
    double WL02=WL0*WL0; double a2=aWL*aWL; double h3=h2*h; double intWL2=WL02*h+(WL0*aWL)*h2+(a2*h3)/3.0;
    double VLEnd=VL0+Aconst*h+Aquad*intWL2; double ALmax=Aconst+Aquad*(WLEnd*WLEnd);
    const char* fn[]={"closed","h","held_input","Jdiag","w_norm_ub_end","q_abs_ub_interval","wL_accel_norm_ub","wL_norm_ub_end","vL_accel_norm_ub","vL_norm_ub_end","pT_norm_ub","sampled_acceptance","guessed_bound"};
    mxArray* S=mxCreateStructMatrix(1,1,13,fn);
    mxSetField(S,0,"closed",mxCreateLogicalScalar(pitchClosed&&fullClosed)); mxSetField(S,0,"h",dscalar(h)); mxSetField(S,0,"held_input",mxDuplicateArray(uA)); mxSetField(S,0,"Jdiag",col(Jd,3));
    mxSetField(S,0,"w_norm_ub_end",dscalar(WabsEnd)); mxSetField(S,0,"q_abs_ub_interval",col(qAbs,3)); mxSetField(S,0,"wL_accel_norm_ub",dscalar(aWL)); mxSetField(S,0,"wL_norm_ub_end",dscalar(WLEnd));
    mxSetField(S,0,"vL_accel_norm_ub",dscalar(ALmax)); mxSetField(S,0,"vL_norm_ub_end",dscalar(VLEnd)); mxSetField(S,0,"pT_norm_ub",dscalar(Pnorm)); mxSetField(S,0,"sampled_acceptance",mxCreateLogicalScalar(false)); mxSetField(S,0,"guessed_bound",mxCreateLogicalScalar(false));
    return S;
}

static mxArray* contraction(int mode,const mxArray* C0,const mxArray* C1,const mxArray* F,const mxArray* X) {
    const double* y0=vec(fld(C0,"y"),12,"C0.y"); const double* y1=vec(fld(C1,"y"),12,"C1.y");
    const double* n0=vec(fld(C0,"nu_abs_ub"),12,"C0.nu_abs_ub"); const double* n1=vec(fld(C1,"nu_abs_ub"),12,"C1.nu_abs_ub");
    double h=scalar(fld(C1,"event_time"),"C1.event_time")-scalar(fld(C0,"event_time"),"C0.event_time"); if (!(h>0.0&&std::isfinite(h))) fail("AF7R66:Domain","contraction h invalid.");
    const double* xp=vec(fld(X,"p"),3,"xhat.p"); const double* xq=vec(fld(X,"q"),3,"xhat.q"); const double* xpL=vec(fld(X,"pL"),3,"xhat.pL"); const double* xpT=vec(fld(X,"pT"),3,"xhat.pT");
    const double* xvL=vec(fld(X,"vL"),3,"xhat.vL"); const double* xwL=vec(fld(X,"wL"),3,"xhat.wL"); const double* xw=vec(fld(X,"w"),3,"xhat.w");
    double cp[3],cq[3],cpL[3],cpT[3],rp[3],rq[3],rpL[3],rpT[3];
    for(int i=0;i<3;++i){ cp[i]=xp[i]-y1[i]; cq[i]=xq[i]-y1[3+i]; cpL[i]=xpL[i]-y1[6+i]; cpT[i]=xpT[i]-y1[9+i]; rp[i]=n1[i]; rq[i]=n1[3+i]; rpL[i]=n1[6+i]; rpT[i]=n1[9+i]; }
    const char* mfn[]={"p","q","pL","pT"}; mxArray* MEAS=mxCreateStructMatrix(1,1,4,mfn); mxSetField(MEAS,0,"p",mk_err3(cp,rp)); mxSetField(MEAS,0,"q",mk_err3(cq,rq)); mxSetField(MEAS,0,"pL",mk_err3(cpL,rpL)); mxSetField(MEAS,0,"pT",mk_err3(cpT,rpT));
    double VLF=scalar(fld(F,"vL_norm_ub_end"),"F.vL_norm_ub_end"), ALmax=scalar(fld(F,"vL_accel_norm_ub"),"F.vL_accel_norm_ub");
    double dPL[3]; for(int i=0;i<3;++i) dPL[i]=(y1[6+i]-y0[6+i])/h; double n0pL[3]={n0[6],n0[7],n0[8]}, n1pL[3]={n1[6],n1[7],n1[8]};
    double rVL=0.5*ALmax*h+(normv(n0pL,3,mode)+normv(n1pL,3,mode))/h; double cVL[3]={dPL[0],dPL[1],dPL[2]}; double VLtrue=std::min(VLF,normv(cVL,3,mode)+rVL);
    double xvLdiff[3]; for(int i=0;i<3;++i)xvLdiff[i]=xvL[i]-cVL[i]; double eVL=std::min(normv(xvL,3,mode)+VLF,normv(xvLdiff,3,mode)+rVL);
    double WLF=scalar(fld(F,"wL_norm_ub_end"),"F.wL_norm_ub_end"), Pbar=scalar(fld(F,"pT_norm_ub"),"F.pT_norm_ub"), aWL=scalar(fld(F,"wL_accel_norm_ub"),"F.wL_accel_norm_ub");
    double pTddot=Pbar*(aWL+WLF*WLF); double dPT[3]; for(int i=0;i<3;++i)dPT[i]=(y1[9+i]-y0[9+i])/h; double n0pT[3]={n0[9],n0[10],n0[11]}, n1pT[3]={n1[9],n1[10],n1[11]};
    double rPTdot=0.5*pTddot*h+(normv(n0pT,3,mode)+normv(n1pT,3,mode))/h; double y1pT[3]={y1[9],y1[10],y1[11]}, cWL[3]; crossv(y1pT,dPT,cWL,mode);
    double pTdotUb=Pbar*WLF; double rWL=normv(n1pT,3,mode)*pTdotUb+normv(y1pT,3,mode)*rPTdot; double WLtrue=std::min(WLF,normv(cWL,3,mode)+rWL); double xwLdiff[3]; for(int i=0;i<3;++i)xwLdiff[i]=xwL[i]-cWL[i]; double eWL=std::min(normv(xwL,3,mode)+WLF,normv(xwLdiff,3,mode)+rWL);
    double Wf=scalar(fld(F,"w_norm_ub_end"),"F.w_norm_ub_end"); const double* Jd=vec(fld(F,"Jdiag"),3,"F.Jdiag"); const double* held=vec(fld(F,"held_input"),4,"F.held_input");
    double Jmin=std::min(Jd[0],std::min(Jd[1],Jd[2])), Jmax=std::max(Jd[0],std::max(Jd[1],Jd[2])); double tauv[3]={held[1],held[2],held[3]}, tauNorm=normv(tauv,3,mode); const double* qab=vec(fld(F,"q_abs_ub_interval"),3,"F.q_abs_ub_interval"); double Theta=qab[1]; double pi=3.141592653589793238462643383279502884;
    if (!(std::isfinite(Theta)&&Theta<pi/2.0)) fail("AF7R66:Domain","Euler chart is not closed.");
    double tanT=tanv(Theta,mode), secT=1.0/cosv(Theta,mode), Kphi=1.0+tanT; double Wf2=Wf*Wf;
    double Aomega=tauNorm/Jmin+(Jmax/Jmin)*Wf2; double adot=Aomega+Kphi*Wf2; double secT2=secT*secT; double qd[3]={Aomega+secT2*Wf2+tanT*adot, Aomega+Kphi*Wf2, secT*(tanT*Wf2+adot)}; double qddotUb=normv(qd,3,mode);
    double dq[3]; for(int i=0;i<3;++i)dq[i]=(y1[3+i]-y0[3+i])/h; double n0q[3]={n0[3],n0[4],n0[5]}, n1q[3]={n1[3],n1[4],n1[5]}; double rQdot=0.5*qddotUb*h+(normv(n0q,3,mode)+normv(n1q,3,mode))/h;
    double lim[3]={pi,pi/2.0,pi}; for(int i=0;i<3;++i){ if (!(std::fabs(y0[3+i])+n0[3+i]<lim[i] && std::fabs(y1[3+i])+n1[3+i]<lim[i])) fail("AF7R66:Domain","measured Euler box reaches chart boundary."); }
    double phi=y1[3], theta=y1[4]; double sphi=sinv(phi,mode), cphi=cosv(phi,mode), stheta=sinv(theta,mode), ctheta=cosv(theta,mode); double E[9]={1.0,0.0,0.0, 0.0,cphi,-sphi, -stheta,sphi*ctheta,cphi*ctheta}; double cW[3]; matvec3(E,dq,cW,mode);
    double dphi=n1[3], dtheta=n1[4]; double dsum=dphi+dtheta; double LE=sqrtv(dtheta*dtheta+2.0*dphi*dphi+2.0*dsum*dsum,mode); double Kq=sqrtv(Kphi*Kphi+1.0+secT2,mode); double qdotUb=Kq*Wf; double EyNorm2=nextup_nonnegative(sqrtv(1.0+std::fabs(sinv(y1[4],mode)),mode)); double rW=LE*qdotUb+EyNorm2*rQdot; double Wtrue=std::min(Wf,normv(cW,3,mode)+rW); double xwdiff[3]; for(int i=0;i<3;++i)xwdiff[i]=xw[i]-cW[i]; double eW=std::min(normv(xw,3,mode)+Wf,normv(xwdiff,3,mode)+rW);
    const char* fn[]={"closed","h","measured_error","vL","wL","w"}; mxArray* O=mxCreateStructMatrix(1,1,6,fn); mxSetField(O,0,"closed",mxCreateLogicalScalar(true)); mxSetField(O,0,"h",dscalar(h)); mxSetField(O,0,"measured_error",MEAS); mxSetField(O,0,"vL",mk_bound3(cVL,rVL,VLtrue,eVL)); mxSetField(O,0,"wL",mk_bound3(cWL,rWL,WLtrue,eWL)); mxSetField(O,0,"w",mk_bound3(cW,rW,Wtrue,eW)); return O;
}

void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {
    if (nlhs>1) fail("AF7R66:Output","One output only.");
    if (nrhs<2) fail("AF7R66:Input","Missing operation/mode inputs.");
    int op=(int)scalar(prhs[0],"op"), mode=(int)scalar(prhs[1],"mode"); if (!(mode==0||mode==1)) fail("AF7R66:Mode","mode must be 0 or 1.");
    if (op==1) { if (nrhs!=6) fail("AF7R66:Input","one-step expects op,mode,h,u,I,K0."); plhs[0]=one_step(mode,scalar(prhs[2],"h"),prhs[3],prhs[4],prhs[5]); return; }
    if (op==2) { if (nrhs!=6) fail("AF7R66:Input","contraction expects op,mode,C0,C1,F,xhat."); plhs[0]=contraction(mode,prhs[2],prhs[3],prhs[4],prhs[5]); return; }
    fail("AF7R66:Op","Unknown operation.");
}
