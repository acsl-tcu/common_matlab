// TRAJECTORY_C9_SEPARATOR_NORMAL_NATIVE.cpp
// Accelerated candidate NORMAL GENERATION only (Gilbert/Frank-Wolfe).
// The MATLAB caller MUST independently verify the returned normal by
// testing min(n'*H) > TRAJECTORY_OBSTACLE_SUPPORT.value(S,n)+margin.
// Never a collision certificate, no shape- or pose-specific bypass paths.
// Live validated canonical geometry is passed on each call.
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>
#include <string>
#include <stdexcept>
#include <limits>
#include <cstdio>
namespace c9sep {
using V=std::array<double,3>;
static V add(V a,V b){return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
static V sub(V a,V b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
static V scale(V a,double b){return {a[0]*b,a[1]*b,a[2]*b};}
static double dot(V a,V b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static double norm(V a){return std::hypot(a[0],std::hypot(a[1],a[2]));}
static bool finite(V a){return std::isfinite(a[0])&&std::isfinite(a[1])&&std::isfinite(a[2]);}
static double sign(double x){return x>0?1:(x<0?-1:0);}
struct Shape {
 V center{};std::array<double,9> R{};
 std::string kind;std::vector<double> dimensions;std::vector<V> vertices;
 V world_to_local(V n)const{
  // MATLAB R.'*n, R stored in column-major.
  V out{};for(int j=0;j<3;++j)for(int i=0;i<3;++i)out[j]+=R[j*3+i]*n[i];return out;
 }
 V local_to_world(V l)const{
  V out{};for(int j=0;j<3;++j)for(int i=0;i<3;++i)out[i]+=R[j*3+i]*l[j];return out;
 }
 V local_support_point(V w)const{
  if(!vertices.empty()){
    double h=-std::numeric_limits<double>::infinity();V p{};
    for(auto v:vertices){double a=dot(v,w);if(a>h){h=a;p=v;}}
    return p;
  }
  auto d=[this](int i)->double{return dimensions.at(static_cast<std::size_t>(i));};
  double rho=std::hypot(w[0],w[1]),mag=norm(w);
  V radial=rho>0?V{w[0]/rho,w[1]/rho,0}:V{};
  if(kind=="sphere")return mag>0?scale(w,d(0)/mag):V{};
  if(kind=="ellipsoid"){
    double a[3]={d(0),d(1),d(2)};
    double den=std::hypot(a[0]*w[0],std::hypot(a[1]*w[1],a[2]*w[2]));
    return den>0?V{a[0]*a[0]*w[0]/den,a[1]*a[1]*w[1]/den,a[2]*a[2]*w[2]/den}:V{};
  }
  if(kind=="box")return V{d(0)*sign(w[0])/2,d(1)*sign(w[1])/2,d(2)*sign(w[2])/2};
  if(kind=="cylinder")return V{d(0)*radial[0],d(0)*radial[1],d(1)*sign(w[2])/2};
  if(kind=="capsule"){
    V v=mag>0?scale(w,d(0)/mag):V{};v[2]+=d(1)*sign(w[2])/2;return v;
  }
  if(kind=="cone"){
    double zh=d(1)/2;
    return d(0)*rho-zh*w[2]>=zh*w[2]?V{d(0)*radial[0],d(0)*radial[1],-zh}:V{0,0,zh};
  }
  if(kind=="frustum"){
    double zh=d(2)/2;
    return d(0)*rho-zh*w[2]>=d(1)*rho+zh*w[2]?
           V{d(0)*radial[0],d(0)*radial[1],-zh}:V{d(1)*radial[0],d(1)*radial[1],zh};
  }
  if(kind=="torus"){
    V v=mag>0?scale(w,d(1)/mag):V{};
    v[0]+=d(0)*radial[0];v[1]+=d(0)*radial[1];return v;
  }
  throw std::domain_error("unsupported canonical shape");
 }
 V support_point(V n)const {return add(center,local_to_world(local_support_point(world_to_local(n))));}
 double support_value(V n)const {return dot(n,support_point(n));}
};
struct Result {bool separated=false;V normal{};int calls=0;int iterations=0;};
Result run(const std::vector<V>&H,const Shape&shape,double margin,V ref,int maxiter){
 Result out{};
 if(H.empty()||maxiter<0||!std::isfinite(margin)||margin<0)return out;
 V center{};
 for(V v:H)center=add(center,v);
 center=scale(center,1./static_cast<double>(H.size()));
 V axes[2]={sub(center,shape.center),sub(ref,shape.center)};
 auto try_normal=[&](V raw)->bool{
  const double m=norm(raw);
  if(!(std::isfinite(m)&&m>0))return false;
  V n=scale(raw,1./m);
  double h=shape.support_value(n);
  out.calls++;
  double hmin=std::numeric_limits<double>::infinity();
  for(V p:H)hmin=std::min(hmin,dot(n,p));
  if(std::isfinite(h)&&hmin>h+margin){out.normal=n;out.separated=true;return true;}
  return false;
 };
 for(auto a:axes)if(try_normal(a))return out;
 V a=axes[0];
 if(norm(a)==0)a=axes[1];
 if(norm(a)==0){
  double mx=-1;std::size_t ii=0;
  for(std::size_t i=0;i<H.size();++i){V z=sub(H[i],center);double sq=dot(z,z);if(sq>mx){mx=sq;ii=i;}}
  a=sub(H[ii],shape.center);
 }
 if(norm(a)==0)return out;
 double best=std::numeric_limits<double>::infinity();std::size_t idx=0;
 for(std::size_t i=0;i<H.size();++i){double v=dot(a,H[i]);if(v<best){best=v;idx=i;}}
 V q=sub(H[idx],shape.support_point(a));
 for(int i=1;i<=maxiter;++i){
  out.iterations=i;
  if(try_normal(q))return out;
  if(!(finite(q)&&norm(q)>0))return out;
  best=std::numeric_limits<double>::infinity();
  for(std::size_t j=0;j<H.size();++j){double v=dot(q,H[j]);if(v<best){best=v;idx=j;}}
  V d=sub(sub(H[idx],shape.support_point(q)),q);
  double d2=dot(d,d);
  if(!(std::isfinite(d2)&&d2>0))return out;
  double step=std::min(1.,std::max(0.,-dot(q,d)/d2));
  if(!(std::isfinite(step)&&step>0))return out;
  q=add(q,scale(d,step));
 }
 try_normal(q);
 return out;
}
}
#ifndef C9_STANDALONE_TEST
#include "mex.h"
static void check(bool p,const char*id,const char*msg){if(!p)mexErrMsgIdAndTxt(id,"%s",msg);}
static const mxArray *field(const mxArray*s,const char*n){
 const mxArray*x=mxGetField(s,0,n);check(x!=nullptr,"C9_NATIVE_SEPARATOR:Shape", "Missing canonical shape field");return x;
}
static c9sep::V vec3(const mxArray*x){
 check(mxIsDouble(x)&&!mxIsComplex(x)&&mxGetNumberOfElements(x)==3,
 "C9_NATIVE_SEPARATOR:Vector","Expected real three-vector");
 auto p=mxGetPr(x);c9sep::V v{p[0],p[1],p[2]};
 check(c9sep::finite(v),"C9_NATIVE_SEPARATOR:Finite","Nonfinite point or centre");return v;
}
void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
 using namespace c9sep;
 if(nrhs==0){
  check(nlhs==1,"C9_NATIVE_SEPARATOR:Signature","Signature output required");
  plhs[0]=mxCreateString("C9_NATIVE_SEPARATOR_NORMAL_20261110_V2_EMPTY_VERTEX_COMPAT");return;
 }
 check(nrhs==5&&nlhs==4,"C9_NATIVE_SEPARATOR:Signature", "[normal,proposed,calls,iters] = native(H,preparedShape,margin,reference,maxiter)");
 const mxArray *a=prhs[0];
 check(mxIsDouble(a)&&!mxIsComplex(a)&&mxGetM(a)==3&&mxGetN(a)>=1,
  "C9_NATIVE_SEPARATOR:Hull","Expected real 3xN hull");
 std::vector<V> hull;hull.reserve(mxGetN(a));const double *p=mxGetPr(a);
 for(mwSize i=0;i<mxGetN(a);++i){V v{p[3*i],p[3*i+1],p[3*i+2]};
  check(finite(v),"C9_NATIVE_SEPARATOR:Hull","Nonfinite hull vertex");hull.push_back(v);}
 check(mxIsStruct(prhs[1])&&mxGetNumberOfElements(prhs[1])==1,
  "C9_NATIVE_SEPARATOR:Shape","Canonical prepared scalar required");
 Shape shape;shape.center=vec3(field(prhs[1],"center"));
 const mxArray*R=field(prhs[1],"R");
 check(mxIsDouble(R)&&mxGetM(R)==3&&mxGetN(R)==3,
  "C9_NATIVE_SEPARATOR:Rotation","Expected finite SO3 matrix");
 for(int i=0;i<9;++i)shape.R[i]=mxGetPr(R)[i];
 const mxArray *kind=field(prhs[1],"kind");
 char*k=mxArrayToString(kind);check(k!=nullptr,"C9_NATIVE_SEPARATOR:Kind","Missing kind");
 shape.kind=k;mxFree(k);
 const mxArray *v=field(prhs[1],"local_vertices");
 // Canonical prepared shapes use zeros(0,3); MATLAB diagnostic fixtures may
 // express the SAME empty vertex set as [] (0x0).  Empty carries no
 // geometric information and must not turn a valid analytic shape into a
 // MEX exception / silent slow MATLAB fallback. Nonempty vertices MUST Vx3.
 check(mxIsDouble(v)&&!mxIsComplex(v)&&
   (mxGetNumberOfElements(v)==0 || mxGetN(v)==3),
   "C9_NATIVE_SEPARATOR:Vertices", "Expected empty or real Vx3 vertices");
 mwSize nv=mxGetM(v);const double*vv=mxGetPr(v);
 for(mwSize i=0;i<nv;++i)shape.vertices.push_back(V{vv[i],vv[nv+i],vv[2*nv+i]});
 if(shape.vertices.empty()){
  const mxArray*dim=field(prhs[1],"dim");
  check(mxIsDouble(dim)&&!mxIsComplex(dim),"C9_NATIVE_SEPARATOR:Dimensions","Expected canonical dimensions");
  const double*dd=mxGetPr(dim);
  for(mwSize i=0;i<mxGetNumberOfElements(dim);++i)shape.dimensions.push_back(dd[i]);
 }
 const mxArray *mar=prhs[2],*it=prhs[4];
 check(mxIsDouble(mar)&&mxGetNumberOfElements(mar)==1&&
   mxIsDouble(it)&&mxGetNumberOfElements(it)==1,
  "C9_NATIVE_SEPARATOR:Controls","Expected scalar margin and integer iteration count");
 double margin=mxGetScalar(mar),lim=mxGetScalar(it);
 check(std::isfinite(margin)&&margin>=0&&std::isfinite(lim)&&lim>=0&&lim==std::floor(lim)&&lim<=static_cast<double>(std::numeric_limits<int>::max()),
  "C9_NATIVE_SEPARATOR:Controls","Invalid margin or iteration bound");
 V ref=vec3(prhs[3]);
 Result o;
 try{o=run(hull,shape,margin,ref,static_cast<int>(lim));}
 catch(const std::exception&e){mexErrMsgIdAndTxt("C9_NATIVE_SEPARATOR:Geometry","%s",e.what());}
 plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);double *np=mxGetPr(plhs[0]);
 for(int j=0;j<3;++j)np[j]=o.normal[j];
 plhs[1]=mxCreateLogicalScalar(o.separated);
 plhs[2]=mxCreateDoubleScalar(static_cast<double>(o.calls));
 plhs[3]=mxCreateDoubleScalar(static_cast<double>(o.iterations));
}
#else
#include <random>
#include <iostream>
int main(){
 using namespace c9sep;
 std::mt19937_64 rng(31);std::uniform_real_distribution<double>u(-1,1);
 int success=0;
 for(int i=0;i<3000;++i){
  Shape s; s.center={0,0,0};s.R={1,0,0,0,1,0,0,0,1};s.kind="ellipsoid";
  s.dimensions={.4,.6,.8};
  V ref{2+u(rng),u(rng),u(rng)};
  std::vector<V>h;
  for(int j=0;j<16;++j)h.push_back(add(ref,V{.1*u(rng),.1*u(rng),.1*u(rng)}));
  auto r=run(h,s,.1,ref,64);
  if(r.separated){
   double lhs=1e99;for(V p:h)lhs=std::min(lhs,dot(r.normal,p));
   if(!(lhs>s.support_value(r.normal)+.1))return 3;
   ++success;
  }
 }
 std::cout<<"PASS_SEPARATOR_STANDALONE candidates="<<3000<<" proposed="<<success<<"\n";
 return success>0?0:2;
}
#endif
