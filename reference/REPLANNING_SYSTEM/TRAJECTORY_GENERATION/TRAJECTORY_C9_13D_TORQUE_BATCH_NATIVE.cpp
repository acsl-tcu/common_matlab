// TRAJECTORY_C9_13D_TORQUE_BATCH_NATIVE.cpp
// One native evaluation of the SAME pointwise 13D dual-jet torque formula
// used by TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN_13D.m.
// Output is only a local optimization gradient; NEVER a continuous-time
// input certificate. No fixed Flight mass/obstacle/trajectory/gain values.
// Compile pre-Flight with mex -R2018a. Do not enable fast-math.
#include <cmath>
#include <algorithm>
#include <array>
#include <stdexcept>
#include <limits>
#include <cstring>
#include <cstddef>

namespace c9native {
constexpr int D=14;
using Scalar=std::array<double,D>;
using Vec=std::array<Scalar,3>;
using Jet=std::array<Vec,5>;
constexpr double C[5][5]={{1,0,0,0,0},{1,1,0,0,0},
                          {1,2,1,0,0},{1,3,3,1,0},{1,4,6,4,1}};
Scalar add(const Scalar&a,const Scalar&b) {
    Scalar z{}; for(int i=0;i<D;++i)z[i]=a[i]+b[i];return z;
}
Scalar sub(const Scalar&a,const Scalar&b) {
    Scalar z{}; for(int i=0;i<D;++i)z[i]=a[i]-b[i];return z;
}
Scalar mul(const Scalar&a,const Scalar&b) {
    Scalar z{};z[0]=a[0]*b[0];
    for(int i=1;i<D;++i)z[i]=a[i]*b[0]+a[0]*b[i];
    return z;
}
Scalar scale(const Scalar&a,double f) {
    Scalar z{};for(int i=0;i<D;++i)z[i]=a[i]*f;return z;
}
Vec vadd(const Vec&a,const Vec&b) {
    Vec z{};for(int i=0;i<3;++i)z[i]=add(a[i],b[i]);return z;
}
Vec vsub(const Vec&a,const Vec&b) {
    Vec z{};for(int i=0;i<3;++i)z[i]=sub(a[i],b[i]);return z;
}
Vec vscale(const Vec&a,double f){Vec z{};for(int i=0;i<3;++i)z[i]=scale(a[i],f);return z;}
Vec smul(const Scalar&s,const Vec&a){Vec z{};for(int i=0;i<3;++i)z[i]=mul(s,a[i]);return z;}
Scalar dot(const Vec&a,const Vec&b) {
    Scalar s{};for(int j=0;j<3;++j)s=add(s,mul(a[j],b[j]));return s;
}
Vec cross(const Vec&a,const Vec&b) {
    Vec z{};
    z[0]=sub(mul(a[1],b[2]),mul(a[2],b[1]));
    z[1]=sub(mul(a[2],b[0]),mul(a[0],b[2]));
    z[2]=sub(mul(a[0],b[1]),mul(a[1],b[0]));return z;
}
Vec div(const Vec&a,const Scalar&s) {
    Vec z{};
    if(!(s[0]>0.0)||!std::isfinite(s[0]))throw std::domain_error("singular unit jet");
    for(int j=0;j<3;++j){
        z[j][0]=a[j][0]/s[0];
        for(int i=1;i<D;++i)z[j][i]=(a[j][i]-z[j][0]*s[i])/s[0];
    }
    return z;
}
Jet unit(const Jet &x,int nmax) {
    Jet u{};std::array<Scalar,5> s{};
    double mag=0;
    for(int j=0;j<3;++j)mag+=x[0][j][0]*x[0][j][0];
    s[0][0]=std::sqrt(mag);
    if(!(s[0][0]>0.0)||!std::isfinite(s[0][0]))throw std::domain_error("zero jet norm");
    for(int i=1;i<D;++i){
        double v=0;for(int j=0;j<3;++j)v+=x[0][j][0]*x[0][j][i];
        s[0][i]=v/s[0][0];
    }
    u[0]=div(x[0],s[0]);
    for(int n=1;n<=nmax;++n){
        Scalar rhs{};
        for(int k=0;k<=n;++k)
            rhs=add(rhs,scale(dot(x[k],x[n-k]),C[n][k]));
        for(int k=1;k<n;++k)
            rhs=sub(rhs,scale(mul(s[k],s[n-k]),C[n][k]));
        s[n][0]=rhs[0]/(2*s[0][0]);
        for(int i=1;i<D;++i)
            s[n][i]=(rhs[i]-2*s[0][i]*s[n][0])/(2*s[0][0]);
        Vec num=x[n];
        for(int k=1;k<=n;++k)
            num=vsub(num,vscale(smul(s[k],u[n-k]),C[n][k]));
        u[n]=div(num,s[0]);
    }
    return u;
}
Jet jcross(const Jet&a,const Jet&b,int nmax) {
    Jet x{};
    for(int n=0;n<=nmax;++n){
        Vec z{};
        for(int k=0;k<=n;++k)z=vadd(z,vscale(cross(a[k],b[n-k]),C[n][k]));
        x[n]=z;
    }
    return x;
}
struct Output {bool valid=false;std::array<Scalar,3> torque{};double heading=std::numeric_limits<double>::quiet_NaN();};
// Input jets[k] is 42 doubles, for 3x14 channels in MATLAB column order.
Output evaluate(const std::array<std::array<double,42>,7>& input,
                const double *gravity,const double *massesCable,
                const double *inertia,const double *guards) {
    Output out{};
    const double mu=massesCable[0],ml=massesCable[1],L=massesCable[2];
    if(!(mu>0&&ml>=0&&L>0&&guards[0]>=0&&guards[1]>=0))return out;
    for(int j=0;j<3;++j)if(!(inertia[j]>0))return out;
    try{
        std::array<Vec,7>d{};
        for(int k=0;k<7;++k)for(int j=0;j<3;++j)
            for(int a=0;a<D;++a)d[k][j][a]=input[k][3*a+j];
        Jet w{};
        w[0]=vscale(d[2],-1.0);
        for(int j=0;j<3;++j)w[0][j][0]+=gravity[j];
        for(int k=1;k<5;++k)w[k]=vscale(d[k+2],-1.0);
        if(!(dot(w[0],w[0])[0]>0))return out;
        Jet q=unit(w,4);
        Jet force{},velocity{};
        for(int k=0;k<3;++k){
            force[k]=vsub(vscale(w[k],-(mu+ml)),vscale(q[k+2],mu*L));
            velocity[k]=vsub(d[k+1],vscale(q[k+1],L));
        }
        double speed=std::hypot(velocity[0][0][0],velocity[0][1][0]);
        out.heading=speed;
        double fsq=dot(force[0],force[0])[0];
        if(!(force[0][2][0]>guards[0] && speed>guards[1]&&speed>0&&fsq>0))return out;
        Jet lateral{};
        for(int k=0;k<3;++k){
            lateral[k][0]=scale(velocity[k][1],-1.0);
            lateral[k][1]=velocity[k][0];
        }
        Jet raw=jcross(lateral,force,2);
        if(!(dot(raw[0],raw[0])[0]>0))return out;
        Jet b1=unit(raw,2),b3=unit(force,2),b2=jcross(b3,b1,2);
        Vec omega{},alpha{};
        omega[0]=dot(b3[0],b2[1]);
        omega[1]=dot(b1[0],b3[1]);
        omega[2]=dot(b2[0],b1[1]);
        alpha[0]=add(dot(b3[1],b2[1]),dot(b3[0],b2[2]));
        alpha[1]=add(dot(b1[1],b3[1]),dot(b1[0],b3[2]));
        alpha[2]=add(dot(b2[1],b1[1]),dot(b2[0],b1[2]));
        out.torque[0]=add(scale(alpha[0],inertia[0]),scale(mul(omega[1],omega[2]),inertia[2]-inertia[1]));
        out.torque[1]=add(scale(alpha[1],inertia[1]),scale(mul(omega[2],omega[0]),inertia[0]-inertia[2]));
        out.torque[2]=add(scale(alpha[2],inertia[2]),scale(mul(omega[0],omega[1]),inertia[1]-inertia[0]));
        for(int j=0;j<3;++j)for(double z:out.torque[j])if(!std::isfinite(z))return out;
        out.valid=true;
    }catch(const std::domain_error&){return out;}
    return out;
}
} // namespace

#ifndef C9_STANDALONE_TEST
#include "mex.h"
using namespace c9native;
static void check(bool pred,const char *id,const char*msg){if(!pred)mexErrMsgIdAndTxt(id,"%s",msg);}
static const double* vec(const mxArray *a,mwSize n){
    check(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a)&&mxGetNumberOfElements(a)==n,
        "C9_TORQUE_BATCH:Input","Expected finite real double vector of correct length");
    const double *p=mxGetPr(a);for(mwSize i=0;i<n;++i)
        check(std::isfinite(p[i]),"C9_TORQUE_BATCH:Finite","Nonfinite physical argument");
    return p;
}
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]){
    if(nrhs==0){
        check(nlhs==1,"C9_TORQUE_BATCH:Signature","Signature requires one output");
        plhs[0]=mxCreateString("C9_13D_TORQUE_BATCH_NATIVE_20261110_V1");return;
    }
    check(nrhs==5&&nlhs==4,"C9_TORQUE_BATCH:Signature","Expected [tau,gradient,valid,heading]=native(jets,g,mL,inertia,guards)");
    check(mxIsCell(prhs[0])&&mxGetNumberOfElements(prhs[0])==7,"C9_TORQUE_BATCH:Jets","Expected 7 derivative-order cells");
    const mwSize N=mxGetN(mxGetCell(prhs[0],0));
    check(N>0,"C9_TORQUE_BATCH:Jets","At least one phase required");
    for(int k=0;k<7;++k){
        const mxArray *a=mxGetCell(prhs[0],k);
        check(a&&mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a)&&mxGetM(a)==42&&mxGetN(a)==N,
              "C9_TORQUE_BATCH:Jets","All derivative-order cells must be 42xN real doubles");
    }
    const double *g=vec(prhs[1],3),*mass=vec(prhs[2],3),*J=vec(prhs[3],3),*guard=vec(prhs[4],2);
    plhs[0]=mxCreateDoubleMatrix(3,N,mxREAL);
    mwSize dims[3]={3,13,N};plhs[1]=mxCreateNumericArray(3,dims,mxDOUBLE_CLASS,mxREAL);
    plhs[2]=mxCreateLogicalMatrix(1,N);
    plhs[3]=mxCreateDoubleMatrix(1,N,mxREAL);
    double *torque=mxGetPr(plhs[0]), *grad=mxGetPr(plhs[1]), *heading=mxGetPr(plhs[3]);
    mxLogical *valid=mxGetLogicals(plhs[2]);
    for(mwSize k=0;k<N;++k){
        std::array<std::array<double,42>,7> in{};
        for(int r=0;r<7;++r){
            const double *src=mxGetPr(mxGetCell(prhs[0],r))+42*k;
            std::copy(src,src+42,in[r].begin());
        }
        Output out=evaluate(in,g,mass,J,guard);
        valid[k]=out.valid;heading[k]=out.heading;
        for(int a=0;a<3;++a){
            torque[a+3*k]=out.valid?out.torque[a][0]:mxGetNaN();
            for(int j=0;j<13;++j)
                grad[a+3*j+3*13*k]=out.valid?out.torque[a][j+1]:mxGetNaN();
        }
    }
}
#else
#include <random>
#include <cstdio>
#include <cstdlib>
int main(){
    using namespace c9native;
    std::mt19937_64 rng(15031);std::uniform_real_distribution<double>u(-1,1);
    double g[3]={0,0,-9.80665};double m[3]={1.2,0.15,2};double J[3]={0.02,0.03,0.04};double guard[2]={0,0};
    int valid=0;double worst=0;
    for(int sample=0;sample<350;++sample){
        std::array<std::array<double,42>,7> d{};
        for(int r=0;r<7;++r)for(int a=0;a<3;++a){
            double b=0.04*u(rng);
            if(r==1&&a==0)b=1.5+0.15*u(rng);
            if(r==1&&a==1)b=0.6+0.15*u(rng);
            d[r][a]=b;
            for(int j=1;j<14;++j)d[r][3*j+a]=0.03*u(rng);
        }
        auto out=evaluate(d,g,m,J,guard);
        if(!out.valid){std::fprintf(stderr,"Unexpected invalid sample %d\n",sample);return 2;}
        ++valid;
        // Independent centered finite difference on THE REAL torque equations,
        // not on the dual gradients. Only synthetic feasible jets are tested.
        for(int j=0;j<13;++j){
            auto p=d,minus=d;
            double h=1e-5;
            for(int r=0;r<7;++r)for(int a=0;a<3;++a){
                p[r][a]+=h*d[r][3*(j+1)+a];
                minus[r][a]-=h*d[r][3*(j+1)+a];
                for(int k=1;k<14;++k){p[r][3*k+a]=0;minus[r][3*k+a]=0;}
            }
            auto yp=evaluate(p,g,m,J,guard),ym=evaluate(minus,g,m,J,guard);
            if(!(yp.valid&&ym.valid)){std::fprintf(stderr,"Unexpected invalid finite difference\n");return 3;}
            for(int a=0;a<3;++a){
                double fd=(yp.torque[a][0]-ym.torque[a][0])/(2*h);
                double der=out.torque[a][j+1];
                double err=std::abs(fd-der)/(1+std::abs(fd));
                worst=std::max(worst,err);
                if(err>2e-5){std::fprintf(stderr,"AD error %d/%d/%d %.14g vs %.14g\n",sample,j,a,fd,der);return 4;}
            }
        }
    }
    std::printf("PASS_NATIVE_DUAL_TORQUE_SELFTEST cases=%d gradients=%d worst_relative=%.3g\n",valid,valid*13*3,worst);
    return 0;
}
#endif
