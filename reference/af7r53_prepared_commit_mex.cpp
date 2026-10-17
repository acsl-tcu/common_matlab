
#include "mex.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

static const mxArray* fld(const mxArray* s,const char* n){
    const mxArray* v=mxGetField(s,0,n);
    if(!v) mexErrMsgIdAndTxt("AF7R53:Field","missing field %s",n);
    return v;
}
static double scalar(const mxArray* a,const char* n){
    if(!a || !mxIsDouble(a) || mxIsComplex(a) || mxGetNumberOfElements(a)!=1)
        mexErrMsgIdAndTxt("AF7R53:Scalar","%s must be a real double scalar.",n);
    double v=mxGetScalar(a);
    if(!std::isfinite(v)) mexErrMsgIdAndTxt("AF7R53:Scalar","%s must be finite.",n);
    return v;
}
static double sfld(const mxArray* s,const char* n){ return scalar(fld(s,n),n); }
static mxArray* dscalar(double v){ return mxCreateDoubleScalar(v); }
static mxArray* logicalScalar(bool v){ return mxCreateLogicalScalar(v); }

static std::vector<double> vec(const mxArray* a,const char* n){
    if(!a || !mxIsDouble(a) || mxIsComplex(a))
        mexErrMsgIdAndTxt("AF7R53:Vector","%s must be real double.",n);
    size_t N=mxGetNumberOfElements(a); const double* p=mxGetPr(a);
    return std::vector<double>(p,p+N);
}
static mxArray* row(const std::vector<double>&v){
    mxArray* a=mxCreateDoubleMatrix(1,v.size(),mxREAL);
    std::copy(v.begin(),v.end(),mxGetPr(a)); return a;
}
static mxArray* col(const std::vector<double>&v){
    mxArray* a=mxCreateDoubleMatrix(v.size(),1,mxREAL);
    std::copy(v.begin(),v.end(),mxGetPr(a)); return a;
}

static mxArray* build(const mxArray* RAW,const mxArray* ST,const mxArray* LIVE,double targetVersion){
    if(!mxIsStruct(RAW)||!mxIsStruct(ST)||!mxIsStruct(LIVE))
        mexErrMsgIdAndTxt("AF7R53:Type","RAW, STATIC and LIVE must be scalar structs.");
    if(!mxIsLogicalScalarTrue(fld(RAW,"success")))
        mexErrMsgIdAndTxt("AF7R53:Raw","RAW.success must be true.");

    const double t0=sfld(RAW,"t0"), tm=sfld(RAW,"tm"), hold=sfld(RAW,"hold_end"), t1=sfld(RAW,"t1");
    const double VL=sfld(RAW,"speed_payload"), A=sfld(RAW,"speed_accel"), J=sfld(RAW,"speed_jerk");
    const double sigma=sfld(RAW,"speed_sigma"), qdot=sfld(RAW,"speed_qdot"), VQ=sfld(RAW,"speed_uav");

    const double dt=sfld(LIVE,"dt"), g=sfld(LIVE,"gravity"), L=sfld(LIVE,"cable_length");
    const double rL=sfld(LIVE,"r_load"), rQ=sfld(LIVE,"r_drone"), rC=sfld(LIVE,"r_cable");
    const double dm=sfld(LIVE,"d_margin"), Rs=sfld(LIVE,"sensor_range"), H=sfld(LIVE,"horizon");
    (void)g; // retained in the packet contract; not needed for sweep/sensor formulas.

    auto seg0=vec(fld(RAW,"segment_t0"),"segment_t0");
    auto seg1=vec(fld(RAW,"segment_t1"),"segment_t1");
    if(seg0.size()!=seg1.size() || seg0.empty())
        mexErrMsgIdAndTxt("AF7R53:Segments","invalid segment arrays.");
    const size_t S=seg0.size();

    const mxArray* CP= fld(RAW,"control_points");
    if(!mxIsCell(CP) || mxGetNumberOfElements(CP)!=S)
        mexErrMsgIdAndTxt("AF7R53:CP","RAW.control_points cell size mismatch.");
    mxArray* cpMat=mxCreateDoubleMatrix(48,S,mxREAL);
    double* cpo=mxGetPr(cpMat);
    for(size_t s=0;s<S;++s){
        const mxArray* c=mxGetCell(CP,s);
        if(!c || !mxIsDouble(c) || mxIsComplex(c) || mxGetM(c)!=3 || mxGetN(c)!=16)
            mexErrMsgIdAndTxt("AF7R53:CP","each control point block must be 3x16.");
        const double* p=mxGetPr(c);
        for(size_t k=0;k<48;++k){
            if(!std::isfinite(p[k])) mexErrMsgIdAndTxt("AF7R53:CP","nonfinite control point.");
            cpo[k+48*s]=p[k];
        }
    }

    const double spL=VL*dt/2.0, spQ=VQ*dt/2.0;
    const double spC=std::max(VL,VQ)*dt/2.0, one=VQ*dt;
    const double Dp=rL+dm+spL, Dq=rQ+dm+spQ, Dc=rC+dm+spC;

    const double rho=std::max({rQ,L+rL,L+rC});
    const double horizonMargin=Rs-(rho+VQ*H);
    const double dDetLB=Rs-VQ*dt;
    const double safeClear=dDetLB-rho;
    const double collisionLead=(VQ>0)?safeClear/VQ:std::numeric_limits<double>::infinity();
    const double beforeHorizon=collisionLead-H;
    const bool sensorCertified=(horizonMargin>0 && safeClear>0 && beforeHorizon>0);

    const mxArray* tcA=fld(ST,"type_code");
    const mxArray* rpA=fld(ST,"raw_param");
    const mxArray* arA=fld(ST,"Araw");
    if(!mxIsDouble(tcA)||mxIsComplex(tcA)||!mxIsDouble(rpA)||mxIsComplex(rpA)||!mxIsDouble(arA)||mxIsComplex(arA))
        mexErrMsgIdAndTxt("AF7R53:Static","STATIC numeric fields invalid.");
    const size_t M=mxGetNumberOfElements(tcA);
    if(mxGetM(rpA)!=2 || mxGetN(rpA)!=M || mxGetM(arA)!=3 || mxGetN(arA)!=M)
        mexErrMsgIdAndTxt("AF7R53:Static","STATIC dimensions invalid.");
    const double* tc=mxGetPr(tcA); const double* rp=mxGetPr(rpA); const double* ar=mxGetPr(arA);

    mxArray* Ap=mxCreateDoubleMatrix(3,M,mxREAL);
    mxArray* Aq=mxCreateDoubleMatrix(3,M,mxREAL);
    mxArray* Ac=mxCreateDoubleMatrix(3,M,mxREAL);
    double* ap=mxGetPr(Ap);double* aq=mxGetPr(Aq);double* ac=mxGetPr(Ac);

    auto inflate=[&](size_t i,double D,double* out){
        int type=(int)std::llround(tc[i]);
        const double* base=ar+3*i;
        if(type==1){ // cylinder: raw_param = [radius;height]
            double r=rp[0+2*i], h=rp[1+2*i];
            double xy=std::sqrt(3.0/2.0)*(r+D);
            out[0]=xy;out[1]=xy;out[2]=std::sqrt(3.0)*(h/2.0+D);
        }else if(type==2){ // sphere
            double r=rp[0+2*i];out[0]=out[1]=out[2]=r+D;
        }else{
            double amin=std::min({base[0],base[1],base[2]});
            if(!(std::isfinite(amin)&&amin>0))
                mexErrMsgIdAndTxt("AF7R53:Static","generic Araw must be positive.");
            double sc=1.0+D/amin;
            out[0]=sc*base[0];out[1]=sc*base[1];out[2]=sc*base[2];
        }
    };
    for(size_t i=0;i<M;++i){
        inflate(i,Dp,ap+3*i);inflate(i,Dq,aq+3*i);inflate(i,Dc,ac+3*i);
    }

    const char* fn[]={
        "valid","target_version","t0","tm","hold_end","t1",
        "segment_t0","segment_t1","control_points_matrix",
        "speed","speed_stageA","sweep","sensor","inflation",
        "Apay","Auav","Acable","geometry_margin","geometry_reason"
    };
    mxArray* Q=mxCreateStructMatrix(1,1,sizeof(fn)/sizeof(fn[0]),fn);
    auto set=[&](const char*n,mxArray*v){mxSetField(Q,0,n,v);};
    set("valid",logicalScalar(true));set("target_version",dscalar(targetVersion));
    set("t0",dscalar(t0));set("tm",dscalar(tm));set("hold_end",dscalar(hold));set("t1",dscalar(t1));
    set("segment_t0",row(seg0));set("segment_t1",row(seg1));set("control_points_matrix",cpMat);
    set("speed",col({VL,A,J,sigma,qdot,VQ}));
    set("speed_stageA",mxDuplicateArray(fld(RAW,"speed_stageA")));
    set("sweep",col({spL,spQ,spC,one,dt}));
    set("sensor",col({sensorCertified?1.0:0.0,VL,VQ,dt,Rs,H,rho,horizonMargin,dDetLB,collisionLead,beforeHorizon}));
    set("inflation",col({Dp,Dq,Dc}));
    set("Apay",Ap);set("Auav",Aq);set("Acable",Ac);
    set("geometry_margin",col({
        sfld(RAW,"geometry_min_payload_margin"),
        sfld(RAW,"geometry_min_uav_margin"),
        sfld(RAW,"geometry_min_cable_margin"),
        sfld(RAW,"geometry_min_margin")}));
    set("geometry_reason",mxDuplicateArray(fld(RAW,"geometry_reason")));
    return Q;
}

static void evalPrepared(const mxArray* Q,const mxArray* T,mxArray*&Y){
    if(!mxIsStruct(Q) || !mxIsDouble(T) || mxIsComplex(T))
        mexErrMsgIdAndTxt("AF7R53:Eval","invalid prepared packet or time array.");
    const double t0=sfld(Q,"t0"),t1=sfld(Q,"t1");
    auto seg0=vec(fld(Q,"segment_t0"),"segment_t0");
    auto seg1=vec(fld(Q,"segment_t1"),"segment_t1");
    const mxArray* CPa=fld(Q,"control_points_matrix");
    if(!mxIsDouble(CPa)||mxIsComplex(CPa)||mxGetM(CPa)!=48||mxGetN(CPa)!=seg0.size())
        mexErrMsgIdAndTxt("AF7R53:Eval","control_points_matrix invalid.");
    const double* CP=mxGetPr(CPa);
    const double* tp=mxGetPr(T); size_t N=mxGetNumberOfElements(T);
    Y=mxCreateDoubleMatrix(21,N,mxREAL);double* yp=mxGetPr(Y);

    for(size_t it=0;it<N;++it){
        double t=tp[it];
        if(!std::isfinite(t)) mexErrMsgIdAndTxt("AF7R53:Eval","nonfinite time.");
        if(t<=t0 || t>=t1) continue;

        int is=-1;
        for(size_t j=0;j<seg1.size();++j){ if(t<=seg1[j]){is=(int)j;break;} }
        if(is<0)continue;
        if(t<seg0[is]){
            if(is==0)continue;
            --is;
        }
        double ta=seg0[is],tb=seg1[is],dur=tb-ta;
        if(!(dur>0)) mexErrMsgIdAndTxt("AF7R53:Eval","nonpositive segment duration.");
        double u=(t-ta)/dur; u=std::max(0.0,std::min(1.0,u));

        // D stores 3 x (deg+1), column-major by control-point index.
        double D[3*16];
        for(int k=0;k<48;++k)D[k]=CP[k+48*is];
        int ncp=16,deg=15;
        for(int r=0;r<=6;++r){
            // de Casteljau on a local copy.
            double W[3*16];
            for(int k=0;k<3*ncp;++k)W[k]=D[k];
            int m=ncp;
            while(m>1){
                for(int j=0;j<m-1;++j)for(int d=0;d<3;++d)
                    W[d+3*j]=(1.0-u)*W[d+3*j]+u*W[d+3*(j+1)];
                --m;
            }
            for(int d=0;d<3;++d)yp[(3*r+d)+21*it]=W[d];

            if(r==6) break;
            for(int j=0;j<ncp-1;++j)for(int d=0;d<3;++d)
                D[d+3*j]=(double(deg)/dur)*(D[d+3*(j+1)]-D[d+3*j]);
            --ncp;--deg;
        }
    }
}

void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    if(nrhs<1) mexErrMsgIdAndTxt("AF7R53:Arity","missing operation code.");
    int op=(int)std::llround(scalar(prhs[0],"op"));
    if(op==1){
        if(nrhs!=5||nlhs>1) mexErrMsgIdAndTxt("AF7R53:Arity","op1: Q=mex(1,RAW,STATIC,LIVE,targetVersion)");
        double v=scalar(prhs[4],"targetVersion");
        plhs[0]=build(prhs[1],prhs[2],prhs[3],v);
        return;
    }
    if(op==2){
        if(nrhs!=3||nlhs>1) mexErrMsgIdAndTxt("AF7R53:Arity","op2: Y=mex(2,Q,t)");
        evalPrepared(prhs[1],prhs[2],plhs[0]);return;
    }
    mexErrMsgIdAndTxt("AF7R53:Op","unsupported op.");
}
