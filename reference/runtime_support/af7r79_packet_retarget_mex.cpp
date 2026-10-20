#include "mex.h"
#include <cmath>
#include <string>
#include <algorithm>
#include <cctype>

static const char* REV="AF7R58ZZY_R29_PACKET_RETARGET_V1";

static void req(bool q,const char* id,const char* msg){
    if(!q) mexErrMsgIdAndTxt(id,"%s",msg);
}
static double scalar(const mxArray* a,const char* name){
    req(a && mxIsDouble(a) && !mxIsComplex(a) && mxGetNumberOfElements(a)==1,
        "AF7R79:Input",name);
    double v=mxGetScalar(a);
    req(std::isfinite(v),"AF7R79:Input","scalar must be finite.");
    return v;
}
static std::string lowerstr(const mxArray* a){
    req(a && mxIsChar(a),"AF7R79:Type","obstacle type must be char.");
    char* c=mxArrayToString(a);
    req(c!=nullptr,"AF7R79:Type","cannot read obstacle type.");
    std::string s(c); mxFree(c);
    std::transform(s.begin(),s.end(),s.begin(),
        [](unsigned char x){return (char)std::tolower(x);});
    return s;
}
static const mxArray* field(const mxArray* s,mwIndex i,const char* name){
    const mxArray* a=mxGetField(s,i,name);
    if(!a) mexErrMsgIdAndTxt("AF7R79:Field","missing obstacle field %s.",name);
    return a;
}
static void set3(double* p,mwSize j,double x,double y,double z){
    p[3*j+0]=x; p[3*j+1]=y; p[3*j+2]=z;
}

static void compute(const mxArray* mapA,const mxArray* rawA,
                    double Dp,double Dq,double Dc,double kxy,double kz,
                    mxArray*& ApA,mxArray*& AqA,mxArray*& AcA){
    req(mxIsStruct(mapA),"AF7R79:Map","static map must be struct array.");
    const mwSize M=mxGetNumberOfElements(mapA);
    req(mxIsDouble(rawA)&&!mxIsComplex(rawA)&&mxGetM(rawA)==3&&mxGetN(rawA)==M,
        "AF7R79:Araw","Araw must be 3xM.");
    const double* ar=mxGetPr(rawA);
    ApA=mxCreateDoubleMatrix(3,M,mxREAL);
    AqA=mxCreateDoubleMatrix(3,M,mxREAL);
    AcA=mxCreateDoubleMatrix(3,M,mxREAL);
    double* ap=mxGetPr(ApA); double* aq=mxGetPr(AqA); double* ac=mxGetPr(AcA);

    for(mwSize j=0;j<M;++j){
        const std::string typ=lowerstr(field(mapA,j,"type"));
        const mxArray* rpA=field(mapA,j,"raw_param");
        req(mxIsDouble(rpA)&&!mxIsComplex(rpA),"AF7R79:Raw","raw_param must be real double.");
        const double* rp=mxGetPr(rpA); const mwSize nr=mxGetNumberOfElements(rpA);

        if(typ=="cylinder"){
            req(nr>=2,"AF7R79:Cylinder","cylinder raw_param requires [r,h].");
            const double r=rp[0], h=rp[1];
            req(std::isfinite(r)&&std::isfinite(h),"AF7R79:Cylinder","nonfinite cylinder raw_param.");
            set3(ap,j,kxy*(r+Dp),kxy*(r+Dp),kz*(h/2.0+Dp));
            set3(aq,j,kxy*(r+Dq),kxy*(r+Dq),kz*(h/2.0+Dq));
            set3(ac,j,kxy*(r+Dc),kxy*(r+Dc),kz*(h/2.0+Dc));
        }else if(typ=="sphere"){
            req(nr>=1&&std::isfinite(rp[0]),"AF7R79:Sphere","sphere raw_param requires finite r.");
            const double r=rp[0];
            const double xp=r+Dp, xq=r+Dq, xc=r+Dc;
            set3(ap,j,xp,xp,xp); set3(aq,j,xq,xq,xq); set3(ac,j,xc,xc,xc);
        }else{
            const double a0=ar[3*j+0],a1=ar[3*j+1],a2=ar[3*j+2];
            req(std::isfinite(a0)&&std::isfinite(a1)&&std::isfinite(a2)&&a0>0&&a1>0&&a2>0,
                "AF7R79:Araw","invalid raw ellipsoid axis.");
            double amin=a0; if(a1<amin) amin=a1; if(a2<amin) amin=a2;
            const double fp=1.0+Dp/amin, fq=1.0+Dq/amin, fc=1.0+Dc/amin;
            set3(ap,j,fp*a0,fp*a1,fp*a2);
            set3(aq,j,fq*a0,fq*a1,fq*a2);
            set3(ac,j,fc*a0,fc*a1,fc*a2);
        }
    }
}

void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    req(nrhs>=1&&mxIsDouble(prhs[0])&&mxGetNumberOfElements(prhs[0])==1,
        "AF7R79:Arity","first input must be op.");
    const int op=(int)mxGetScalar(prhs[0]);
    if(op==2){
        req(nrhs==1&&nlhs<=1,"AF7R79:Arity","revision: op only.");
        if(nlhs) plhs[0]=mxCreateString(REV);
        return;
    }
    req(op==1,"AF7R79:Op","unknown op.");
    req(nrhs==8&&nlhs==3,"AF7R79:Arity",
        "compute: op,map,Araw,Dp,Dq,Dc,sqrt3over2,sqrt3 -> Ap,Aq,Ac.");
    const double Dp=scalar(prhs[3],"Dp"),Dq=scalar(prhs[4],"Dq"),Dc=scalar(prhs[5],"Dc");
    const double kxy=scalar(prhs[6],"sqrt(3/2)"),kz=scalar(prhs[7],"sqrt(3)");
    req(Dp>=0&&Dq>=0&&Dc>=0&&kxy>0&&kz>0,"AF7R79:Input","invalid inflation/constants.");
    compute(prhs[1],prhs[2],Dp,Dq,Dc,kxy,kz,plhs[0],plhs[1],plhs[2]);
}
