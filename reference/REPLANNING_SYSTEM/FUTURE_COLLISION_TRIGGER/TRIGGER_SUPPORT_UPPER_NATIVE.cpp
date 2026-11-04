// Verified-upper-bound candidate for static observed shape support functions.
// Compile with standard C++ floating-point semantics (no fast-math).
// Exact occupied shape comes from the sensor; this is exclusion-only.
#include "mex.h"
#include <cmath>
#include <cfloat>
#include <cstring>
#include <algorithm>
#include <limits>
#include <vector>

namespace {
struct I { double l,h; I(double a=0,double b=0):l(a),h(b){} };
static double down(double x){return std::nextafter(x,-std::numeric_limits<double>::infinity());}
static double up(double x){return std::nextafter(x,std::numeric_limits<double>::infinity());}
static void finite(const I&a){if(!std::isfinite(a.l)||!std::isfinite(a.h)||a.l>a.h) mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Range","Non-finite interval");}
static I add(I a,I b){I c(down(a.l+b.l),up(a.h+b.h));finite(c);return c;}
static I neg(I a){return I(-a.h,-a.l);}
static I sub(I a,I b){return add(a,neg(b));}
static I mul(I a,I b){double v[4]={a.l*b.l,a.l*b.h,a.h*b.l,a.h*b.h}; double mn=*std::min_element(v,v+4),mx=*std::max_element(v,v+4);I r(down(mn),up(mx));finite(r);return r;}
static I sq(I a){double lo=0.0,hi=std::max(a.l*a.l,a.h*a.h);if(a.l>0)lo=a.l*a.l;else if(a.h<0)lo=a.h*a.h;I r(lo>0?down(lo):0.0,up(hi));finite(r);return r;}
// A nonnegative quantity (sum of squares) can acquire a slightly negative
// INTERVAL lower endpoint when down(0+0) becomes -denorm_min.  Intersecting
// with the mathematical domain [0,+inf) is exact enclosure-preserving.
// Reject only when the UPPER bound is below zero.
static I root(I a){
 if(a.h<0) mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Sqrt","Sqrt interval strictly below zero");
 const double lo=std::max(0.0,a.l);
 const double hi=std::max(0.0,a.h);
 I r(lo==0.0?0.0:std::max(0.0,down(std::sqrt(lo))),up(std::sqrt(hi)));
 finite(r);return r;
}
static I ab(I a){if(a.l>=0)return a;if(a.h<=0)return neg(a);return I(0.0,std::max(-a.l,a.h));}
static I uppermax(I a,I b){return I(std::max(a.l,b.l),std::max(a.h,b.h));}
static I dot3(const double *v,const I*q){I s(0,0);for(int k=0;k<3;++k)s=add(s,mul(I(v[k],v[k]),q[k]));return s;}
static const mxArray* field(const mxArray* s,const char* key){const mxArray* f=mxGetField(s,0,key);if(!f)mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Field","Missing shape field: %s",key);return f;}
static const double* data(const mxArray* a){if(!mxIsDouble(a)||mxIsComplex(a)||mxIsSparse(a))mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Type","Expected real dense double");return mxGetPr(a);}
static double scalar(const mxArray* a){if(mxGetNumberOfElements(a)!=1)mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Scalar","Expected scalar");double x=*data(a);if(!std::isfinite(x))mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Scalar","Nonfinite scalar");return x;}
static I from(double a){return I(a,a);}
static I norm3(const I*q){I s(0,0);for(int i=0;i<3;++i)s=add(s,sq(q[i]));return root(s);}
static I rho2(const I*q){return root(add(sq(q[0]),sq(q[1])));}
static I maxmatrix(const mxArray *m,const I*q,int columns){
 const double* a=data(m);mwSize rows=mxGetM(m);if(mxGetN(m)!=(mwSize)columns || rows<1)mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Vertices","Invalid outline/vertices matrix");
 I best(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());
 for(mwSize j=0;j<rows;++j){I v(0,0);for(int k=0;k<columns;++k)v=add(v,mul(from(a[j+k*rows]),q[k]));best=uppermax(best,v);}return best;
}
static I support(const mxArray* shape, const char* type, const I*q){
 const mxArray* dim=field(shape,"dimensions");
 const double* d=nullptr;mwSize dn=0;
 if(mxIsDouble(dim)&&!mxIsComplex(dim)){d=data(dim);dn=mxGetNumberOfElements(dim);}
 auto req=[&](mwSize n){if(!d||dn<n)mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Dimensions","Unsupported shape dimensions");};
 if(!std::strcmp(type,"sphere")){req(1);return mul(from(d[0]),norm3(q));}
 if(!std::strcmp(type,"ellipsoid")){req(3);I v[3];for(int k=0;k<3;++k)v[k]=mul(from(d[k]),q[k]);return norm3(v);}
 if(!std::strcmp(type,"box")){req(3);I s(0,0);for(int k=0;k<3;++k)s=add(s,mul(from(d[k]*0.5),ab(q[k])));return s;}
 if(!std::strcmp(type,"cylinder")){req(2);return add(mul(from(d[0]),rho2(q)),mul(from(d[1]*0.5),ab(q[2])));}
 if(!std::strcmp(type,"capsule")){req(2);return add(mul(from(d[0]),norm3(q)),mul(from(d[1]*0.5),ab(q[2])));}
 if(!std::strcmp(type,"cone")){req(2);I a=sub(mul(from(d[0]),rho2(q)),mul(from(d[1]*0.5),q[2]));I b=mul(from(d[1]*0.5),q[2]);return uppermax(a,b);}
 if(!std::strcmp(type,"frustum")){req(3);I rho=rho2(q);I a=sub(mul(from(d[0]),rho),mul(from(d[2]*0.5),q[2]));I b=add(mul(from(d[1]),rho),mul(from(d[2]*0.5),q[2]));return uppermax(a,b);}
 if(!std::strcmp(type,"pyramid")||!std::strcmp(type,"polyhedron"))return maxmatrix(field(shape,"vertices"),q,3);
 if(!std::strcmp(type,"regular_prism")){req(3);I a=maxmatrix(field(shape,"outline"),q,2);return add(a,mul(from(d[2]*0.5),ab(q[2])));}
 if(!std::strcmp(type,"polygon_prism")){double h=scalar(field(dim,"height"));I a=maxmatrix(field(shape,"outline"),q,2);return add(a,mul(from(h*0.5),ab(q[2])));}
 if(!std::strcmp(type,"torus")){req(2);return add(mul(from(d[0]),rho2(q)),mul(from(d[1]),norm3(q)));}
 return mul(from(scalar(field(shape,"bounding_radius_m"))),norm3(q));
}
}
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray*prhs[]){
 if(nrhs!=2||nlhs>1||!mxIsStruct(prhs[0])||mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Contract","Usage: upper=TRIGGER_SUPPORT_UPPER_NATIVE(shape, normals3xN)");
 const mxArray* normals=prhs[1];
 if(!mxIsDouble(normals)||mxIsComplex(normals)||mxIsSparse(normals)||mxGetM(normals)!=3)
    mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Normals","Expected 3xN double normals");
 const mwSize N=mxGetN(normals);
 plhs[0]=mxCreateDoubleMatrix(1,N,mxREAL);double* out=mxGetPr(plhs[0]);
 if(N==0)return;
 const mxArray* ty=field(prhs[0],"type");char* type=mxArrayToString(ty);
 if(!type)mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Type","Expected type string");
 const mxArray* rot=field(prhs[0],"rotation");
 if(mxGetM(rot)!=3||mxGetN(rot)!=3)mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Rotation","Expected 3x3 rotation");
 const double* r=data(rot);const double* n=data(normals);
 for(mwSize j=0;j<N;++j){
    I q[3];
    for(int k=0;k<3;++k){q[k]=I(0,0);for(int i=0;i<3;++i){double a=r[i+3*k],b=n[3*j+i];if(!std::isfinite(a)||!std::isfinite(b))mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:NaN","Nonfinite rotation/normal");q[k]=add(q[k],mul(from(a),from(b)));}}
    const I value=support(prhs[0],type,q);if(!std::isfinite(value.h))mexErrMsgIdAndTxt("TRIGGER_SUPPORT_UPPER_NATIVE:Overflow","Unbounded support");out[j]=value.h;
 }
 mxFree(type);
}
