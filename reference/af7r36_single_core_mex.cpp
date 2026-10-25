
#include "mex.h"
#include "af7r36_core.hpp"
#include <string>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <chrono>
#include <cmath>

using namespace af7r34;

static NominalExpr G_nominal;
static bool G_compiled=false;

// AF7_SELECTED_MOTION_AUDIT_R11_RING -- sampled diagnostic only.
struct MotionR11Row {
    double seq=0,t0=NAN,tm=NAN,t1=NAN,ok=0,family=0;
    double full3=0,fullxy=0,base3=0,corr3=0,end3=0;
    double vmax=0,vxymax=0,vbase=0,vcorr=0,moving=0,env=NAN;
    double p0x=NAN,p0y=NAN,p0z=NAN,pmx=NAN,pmy=NAN,pmz=NAN,p1x=NAN,p1y=NAN,p1z=NAN;
    double cmx=NAN,cmy=NAN,cmz=NAN;
};
static std::vector<MotionR11Row> G_motion11;
static double G_motion11_seq=0;
static void motion11_push(const PlanResult&Q,const ActiveReference&R,double t0,double tm,double t1){
    MotionR11Row d;d.seq=++G_motion11_seq;d.t0=t0;d.tm=tm;d.t1=t1;d.ok=Q.success?1:0;
    if(Q.success){
        d.family=Q.cand.family;d.env=Q.cand.max_ctrl;
        const int N=65;auto F0=candidate_eval(R,Q.cand,t0);auto B0=R.eval(t0);Vec3 p0=F0[0];
        d.p0x=p0.x;d.p0y=p0.y;d.p0z=p0.z;
        for(int k=0;k<N;++k){
            double u=double(k)/double(N-1),t=t0+(t1-t0)*u;auto F=candidate_eval(R,Q.cand,t);auto B=R.eval(t);
            Vec3 df=F[0]-p0,db=B[0]-B0[0],dc=F[0]-B[0],dv=F[1]-B[1];
            d.full3=std::max(d.full3,norm(df));d.fullxy=std::max(d.fullxy,std::hypot(df.x,df.y));
            d.base3=std::max(d.base3,norm(db));d.corr3=std::max(d.corr3,norm(dc));
            d.vmax=std::max(d.vmax,norm(F[1]));d.vxymax=std::max(d.vxymax,std::hypot(F[1].x,F[1].y));
            d.vbase=std::max(d.vbase,norm(B[1]));d.vcorr=std::max(d.vcorr,norm(dv));
            if(norm(F[1])>1e-9)++d.moving;
            if(k==(N-1)/2){d.pmx=F[0].x;d.pmy=F[0].y;d.pmz=F[0].z;d.cmx=dc.x;d.cmy=dc.y;d.cmz=dc.z;}
            if(k==N-1){d.p1x=F[0].x;d.p1y=F[0].y;d.p1z=F[0].z;d.end3=norm(F[0]-p0);}
        }
    }
    if(G_motion11.size()>=512)G_motion11.erase(G_motion11.begin());G_motion11.push_back(d);
}
static mxArray* motion11_matrix(){
    const mwSize N=(mwSize)G_motion11.size(),C=29;mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    for(mwSize i=0;i<N;++i){const auto&d=G_motion11[i];double v[C]={
        d.seq,d.t0,d.tm,d.t1,d.ok,d.family,d.full3,d.fullxy,d.base3,d.corr3,d.end3,
        d.vmax,d.vxymax,d.vbase,d.vcorr,d.moving,d.env,
        d.p0x,d.p0y,d.p0z,d.pmx,d.pmy,d.pmz,d.p1x,d.p1y,d.p1z,d.cmx,d.cmy,d.cmz};
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];}return M;
}

struct HeadTopoMexR10B {
    double seq=0,t0=NAN,tm=NAN,t1=NAN,ok=0,family=0,cls=0,strict=0,core=0,lp=0,rp=0;
    double v0=NAN,v1=NAN,a0=NAN,a1=NAN,J=NAN,dl=NAN,dr=NAN,c0=NAN,c1=NAN;
};
static std::vector<HeadTopoMexR10B> G_headtopo10b;
static double G_headtopo10b_seq=0;
static void headtopo10b_push(const PlanResult&P,const ActiveReference&ref,double t0,double tm,double t1){
    HeadTopoMexR10B d;d.seq=++G_headtopo10b_seq;d.t0=t0;d.tm=tm;d.t1=t1;d.ok=P.success?1:0;
    if(P.success){auto H=heading_topology_r10b(ref,P.cand,P.speed,t0,t1);d.family=P.cand.family;d.cls=H.cls;
        d.strict=H.strict?1:0;d.core=H.core?1:0;d.lp=H.left_proven?1:0;d.rp=H.right_proven?1:0;
        d.v0=H.v0;d.v1=H.v1;d.a0=H.a0;d.a1=H.a1;d.J=H.J;d.dl=H.dl;d.dr=H.dr;d.c0=H.core0;d.c1=H.core1;}
    if(G_headtopo10b.size()>=512)G_headtopo10b.erase(G_headtopo10b.begin());G_headtopo10b.push_back(d);
}
static mxArray* headtopo10b_matrix(){
    const mwSize N=(mwSize)G_headtopo10b.size(),C=20;mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    for(mwSize i=0;i<N;++i){const auto&d=G_headtopo10b[i];double v[C]={
        d.seq,d.t0,d.tm,d.t1,d.ok,d.family,d.cls,d.strict,d.core,d.lp,d.rp,d.v0,d.v1,d.a0,d.a1,d.J,d.dl,d.dr,d.c0,d.c1};
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];}return M;
}

// AF7_HEADING_TOPOLOGY_R10_RING -- diagnostic only.
struct HeadTopoMexR10 {
    double seq=0,t0=NAN,tm=NAN,t1=NAN,ok=0,family=0,strict=0,loc=0;
    double fail_t=NAN,width=NAN,vxy=NAN,v0=NAN,v1=NAN;
    double d1=NAN,d2=NAN,d3=NAN,d4=NAN,d5=NAN,d6=NAN,unresolved=0,depthhit=0;
};
static std::vector<HeadTopoMexR10> G_headtopo10;
static double G_headtopo10_seq=0;
static void headtopo10_push(const PlanResult&P,const ActiveReference&ref,double t0,double tm,double t1){
    HeadTopoMexR10 d;d.seq=++G_headtopo10_seq;d.t0=t0;d.tm=tm;d.t1=t1;d.ok=P.success?1:0;
    if(P.success){
        d.family=P.cand.family;auto H=heading_topology_r10(ref,P.cand,P.speed,t0,t1);
        d.strict=H.strict_pass?1:0;d.loc=H.location;d.fail_t=H.fail_t;d.width=H.fail_width;d.vxy=H.vxy;d.v0=H.v0;d.v1=H.v1;
        d.d1=H.hder[0];d.d2=H.hder[1];d.d3=H.hder[2];d.d4=H.hder[3];d.d5=H.hder[4];d.d6=H.hder[5];
        d.unresolved=H.unresolved;d.depthhit=H.max_depth_hit;
    }
    if(G_headtopo10.size()>=512)G_headtopo10.erase(G_headtopo10.begin());G_headtopo10.push_back(d);
}
static mxArray* headtopo10_matrix(){
    const mwSize N=(mwSize)G_headtopo10.size(),C=21;mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    for(mwSize i=0;i<N;++i){const auto&d=G_headtopo10[i];double v[C]={
        d.seq,d.t0,d.tm,d.t1,d.ok,d.family,d.strict,d.loc,d.fail_t,d.width,d.vxy,d.v0,d.v1,
        d.d1,d.d2,d.d3,d.d4,d.d5,d.d6,d.unresolved,d.depthhit};
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];
    }return M;
}

// AF7_PRIMARY_HEADING_R7_MAP_R9BB_RING -- diagnostic only.
struct PHR9Row {
    double seq=0,t0=NAN,tm=NAN,t1=NAN,limit=NAN;
    double prod_ok=0,prod_family=0,prod_heading=0,prod_env=NAN,prod_vq=NAN,prod_qdot=NAN;
    double r7_ok=0,r7_reason=0,r7_cand=0,r7_speed=0,r7_rejoin=0,r7_geom=0,r7_heading=0;
    double r7_env=NAN,r7_vq=NAN,r7_qdot=NAN;
    double wc=0,wmin=NAN,wmint=NAN,wmins=NAN,wminc=NAN,wmax=NAN,wmaxt=NAN,wmaxs=NAN,wmaxc=NAN;
};
static std::vector<PHR9Row> G_phr9;
static double G_phr9_seq=0;
static void phr9_push(const PlanResult&P,const C7R8ShadowResult&Q7,
                      const ActiveReference&ref,const Live&L,double t0,double tm,double t1){
    PHR9Row d;d.seq=++G_phr9_seq;d.t0=t0;d.tm=tm;d.t1=t1;
    double rho=std::max({L.rQ,L.L+L.rL,L.L+L.rC});d.limit=(L.sensor_range-rho)/(L.horizon+L.dt);
    d.prod_ok=P.success?1:0;
    if(P.success){
        d.prod_family=P.cand.family;d.prod_env=P.cand.max_ctrl;d.prod_vq=P.speed.VQ;d.prod_qdot=P.speed.qdot;
        d.prod_heading=c7r7_heading_regular(ref,P.cand,P.speed,t0,t1)?1:0;
    }
    d.r7_ok=Q7.success?1:0;d.r7_reason=Q7.reason;d.r7_cand=Q7.candidate_count;d.r7_speed=Q7.speed_pass;
    d.r7_rejoin=Q7.rejoin_pass;d.r7_geom=Q7.geom_pass;d.r7_heading=Q7.heading_pass;
    if(Q7.success){d.r7_env=Q7.cand.max_ctrl;d.r7_vq=Q7.speed.VQ;d.r7_qdot=Q7.speed.qdot;}
    const auto&W=G_towmap_towing_witness;d.wc=W.count;
    d.wmin=std::isfinite(W.min_vq)?W.min_vq:NAN;d.wmint=W.min_t;d.wmins=W.min_seg;d.wminc=W.min_class;
    d.wmax=std::isfinite(W.max_vq)?W.max_vq:NAN;d.wmaxt=W.max_t;d.wmaxs=W.max_seg;d.wmaxc=W.max_class;
    if(G_phr9.size()>=512)G_phr9.erase(G_phr9.begin());G_phr9.push_back(d);
}
static mxArray* phr9_matrix(){
    const mwSize N=(mwSize)G_phr9.size(),C=30;mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    for(mwSize i=0;i<N;++i){const auto&d=G_phr9[i];double v[C]={
        d.seq,d.t0,d.tm,d.t1,d.limit,d.prod_ok,d.prod_family,d.prod_heading,d.prod_env,d.prod_vq,d.prod_qdot,
        d.r7_ok,d.r7_reason,d.r7_cand,d.r7_speed,d.r7_rejoin,d.r7_geom,d.r7_heading,d.r7_env,d.r7_vq,d.r7_qdot,
        d.wc,d.wmin,d.wmint,d.wmins,d.wminc,d.wmax,d.wmaxt,d.wmaxs,d.wmaxc};
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];
    }return M;
}

// AF7_R7_SHADOW_AUTHORITY_R8BB_RING -- diagnostic only.
struct C7R8MexRow {
    double seq=0,t0=NAN,tm=NAN,t1=NAN,limit=NAN,success=0,reason=0;
    double cand=0,speed=0,rejoin=0,geom=0,heading=0;
    double env=NAN,VQ=NAN,VL=NAN,A=NAN,J=NAN,sigma=NAN,qdot=NAN,total_ms=NAN;
    double wc=0,wmin=NAN,wmint=NAN,wmins=NAN,wminc=NAN,wmax=NAN,wmaxt=NAN,wmaxs=NAN,wmaxc=NAN;
};
static std::vector<C7R8MexRow> G_r7shadow;
static double G_r7shadow_seq=0;
static void r7shadow_push(const C7R8ShadowResult&Q,const Live&L,double t0,double tm,double t1){
    C7R8MexRow d;d.seq=++G_r7shadow_seq;d.t0=t0;d.tm=tm;d.t1=t1;
    double rho=std::max({L.rQ,L.L+L.rL,L.L+L.rC});d.limit=(L.sensor_range-rho)/(L.horizon+L.dt);
    d.success=Q.success?1:0;d.reason=Q.reason;d.cand=Q.candidate_count;d.speed=Q.speed_pass;d.rejoin=Q.rejoin_pass;d.geom=Q.geom_pass;d.heading=Q.heading_pass;
    if(Q.success){d.env=Q.cand.max_ctrl;d.VQ=Q.speed.VQ;d.VL=Q.speed.VL;d.A=Q.speed.A;d.J=Q.speed.J;d.sigma=Q.speed.sigma;d.qdot=Q.speed.qdot;}
    d.total_ms=Q.total_ms;
    const auto&W=G_towmap_towing_witness;d.wc=W.count;d.wmin=std::isfinite(W.min_vq)?W.min_vq:NAN;d.wmint=W.min_t;d.wmins=W.min_seg;d.wminc=W.min_class;
    d.wmax=std::isfinite(W.max_vq)?W.max_vq:NAN;d.wmaxt=W.max_t;d.wmaxs=W.max_seg;d.wmaxc=W.max_class;
    if(G_r7shadow.size()>=512)G_r7shadow.erase(G_r7shadow.begin());G_r7shadow.push_back(d);
}
static mxArray* r7shadow_matrix(){
    const mwSize N=(mwSize)G_r7shadow.size(),C=29;mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    for(mwSize i=0;i<N;++i){const auto&d=G_r7shadow[i];double v[C]={
        d.seq,d.t0,d.tm,d.t1,d.limit,d.success,d.reason,d.cand,d.speed,d.rejoin,d.geom,d.heading,
        d.env,d.VQ,d.VL,d.A,d.J,d.sigma,d.qdot,d.total_ms,d.wc,d.wmin,d.wmint,d.wmins,d.wminc,d.wmax,d.wmaxt,d.wmaxs,d.wmaxc};
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];
    }return M;
}

// AF7_C7_HOMOTOPY_FRONTIER_R6BB_RING -- diagnostic only.
struct C7FrontMexRow {
    double seq=0,t0=NAN,tm=NAN,t1=NAN,alpha=NAN,sign_code=0;
    double generated=0,speed=0,rejoin=0,geometry=0,heading=0,c7=1;
    double envelope=NAN,VL=NAN,A=NAN,J=NAN,sigma=NAN,qdot=NAN,VQ=NAN,heading_lb=NAN,heading_splits=0;
};
static std::vector<C7FrontMexRow> G_c7front_mex;
static double G_c7front_seq=0;
static void c7front_push(double t0,double tm,double t1){
    for(const auto&r:G_c7front_rows){
        C7FrontMexRow d;d.seq=++G_c7front_seq;d.t0=t0;d.tm=tm;d.t1=t1;d.alpha=r.alpha;d.sign_code=r.sign_code;
        d.generated=r.generated?1:0;d.speed=r.speed?1:0;d.rejoin=r.rejoin?1:0;d.geometry=r.geometry?1:0;d.heading=r.heading?1:0;
        d.envelope=r.envelope;d.VL=r.VL;d.A=r.A;d.J=r.J;d.sigma=r.sigma;d.qdot=r.qdot;d.VQ=r.VQ;d.heading_lb=r.heading_lb;d.heading_splits=r.heading_splits;
        if(G_c7front_mex.size()>=2048)G_c7front_mex.erase(G_c7front_mex.begin());
        G_c7front_mex.push_back(d);
    }
}
static mxArray* c7front_matrix(){
    const mwSize N=(mwSize)G_c7front_mex.size(),C=21;
    mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    for(mwSize i=0;i<N;++i){const auto&d=G_c7front_mex[i];double v[C]={
        d.seq,d.t0,d.tm,d.t1,d.alpha,d.sign_code,d.generated,d.speed,d.rejoin,d.geometry,d.heading,d.c7,
        d.envelope,d.VL,d.A,d.J,d.sigma,d.qdot,d.VQ,d.heading_lb,d.heading_splits};
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];
    }return M;
}

// AF7_TOWMAP_R4C_RING -- diagnostic only.
struct TowMapEntry {
    double seq=0,t0=NAN,tm=NAN,t1=NAN,limit=NAN,success=0,reason=0,family=0;
    double envelope=NAN,VQ=NAN,VL=NAN,A=NAN,J=NAN,sigma=NAN,qdot=NAN,total_ms=NAN;
    double pc=0,ps=0,pr=0,pg=0,tc=0,ts=0,tr=0,tg=0,fc=0,fs=0,fr=0,fg=0;
    double pwc=0,pwmin=NAN,pwmint=NAN,pwmins=NAN,pwminc=NAN,pwmax=NAN,pwmaxt=NAN,pwmaxs=NAN,pwmaxc=NAN;
    double twc=0,twmin=NAN,twmint=NAN,twmins=NAN,twminc=NAN,twmax=NAN,twmaxt=NAN,twmaxs=NAN,twmaxc=NAN;
};
static std::vector<TowMapEntry> G_towmap;
static double G_towmap_seq=0;
static inline double towmap_finite_or_nan(double x){return std::isfinite(x)?x:NAN;}
static void towmap_push(const PlanResult&Q,const Live&L,double t0,double tm,double t1){
    TowMapEntry d;d.seq=++G_towmap_seq;d.t0=t0;d.tm=tm;d.t1=t1;
    double rho=std::max({L.rQ,L.L+L.rL,L.L+L.rC});d.limit=(L.sensor_range-rho)/(L.horizon+L.dt);
    d.success=Q.success?1.0:0.0;d.reason=Q.reason;d.family=(double)Q.cand.family;
    if(Q.success){d.envelope=Q.cand.max_ctrl;d.VQ=Q.speed.VQ;d.VL=Q.speed.VL;d.A=Q.speed.A;d.J=Q.speed.J;d.sigma=Q.speed.sigma;d.qdot=Q.speed.qdot;}
    d.total_ms=Q.tim.total;
    d.pc=Q.primary_candidate_count;d.ps=Q.primary_speed_pass;d.pr=Q.primary_rejoin_pass;d.pg=Q.primary_geom_tests;
    d.tc=Q.towing_candidate_count;d.ts=Q.towing_speed_pass;d.tr=Q.towing_rejoin_pass;d.tg=Q.towing_geom_tests;
    d.fc=Q.fallback_candidate_count;d.fs=Q.fallback_speed_pass;d.fr=Q.fallback_rejoin_pass;d.fg=Q.fallback_geom_tests;
    const auto&P=G_towmap_primary_witness;const auto&T=G_towmap_towing_witness;
    d.pwc=P.count;d.pwmin=towmap_finite_or_nan(P.min_vq);d.pwmint=P.min_t;d.pwmins=P.min_seg;d.pwminc=P.min_class;
    d.pwmax=towmap_finite_or_nan(P.max_vq);d.pwmaxt=P.max_t;d.pwmaxs=P.max_seg;d.pwmaxc=P.max_class;
    d.twc=T.count;d.twmin=towmap_finite_or_nan(T.min_vq);d.twmint=T.min_t;d.twmins=T.min_seg;d.twminc=T.min_class;
    d.twmax=towmap_finite_or_nan(T.max_vq);d.twmaxt=T.max_t;d.twmaxs=T.max_seg;d.twmaxc=T.max_class;
    if(G_towmap.size()>=512)G_towmap.erase(G_towmap.begin());
    G_towmap.push_back(d);
}
static mxArray* towmap_matrix(){
    const mwSize N=(mwSize)G_towmap.size(),C=46;
    mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    for(mwSize i=0;i<N;++i){const auto&d=G_towmap[i];double v[C]={
        d.seq,d.t0,d.tm,d.t1,d.limit,d.success,d.reason,d.family,d.envelope,d.VQ,d.VL,d.A,d.J,d.sigma,d.qdot,d.total_ms,
        d.pc,d.ps,d.pr,d.pg,d.tc,d.ts,d.tr,d.tg,d.fc,d.fs,d.fr,d.fg,
        d.pwc,d.pwmin,d.pwmint,d.pwmins,d.pwminc,d.pwmax,d.pwmaxt,d.pwmaxs,d.pwmaxc,
        d.twc,d.twmin,d.twmint,d.twmins,d.twminc,d.twmax,d.twmaxt,d.twmaxs,d.twmaxc};
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];
    }return M;
}


static mxArray* r52_gate_diag_matrix(){
    // 49 columns; diagnostic only.
    const mwSize N=(mwSize)G_r52_gate_diag.size(),C=49;
    mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    auto f=[](double x){return std::isfinite(x)?x:NAN;};
    for(mwSize i=0;i<N;++i){
        const auto&d=G_r52_gate_diag[i];
        double v[C]={
            d.seq,(double)d.family,(double)d.exact_c7,f(d.max_ctrl),f(d.du),f(d.da),f(d.dj),
            f(d.phase_alpha),f(d.phase_delay),f(d.actual_rejoin),
            f(d.limit),f(d.base_VL),f(d.base_A),f(d.base_J),f(d.base_sigma),
            f(d.A_Uv),f(d.A_Ua),f(d.A_Uj),f(d.A_sigma),f(d.A_VQ),
            (double)d.A_cert,(double)d.A_impossible,(double)d.A_repr,
            (double)d.B.attempted,(double)d.B.result_code,(double)d.B.leaf_count,(double)d.B.split_count,
            f(d.B.worst_vq),f(d.B.worst_t),(double)d.B.worst_seg,f(d.B.worst_vl),
            f(d.B.worst_sigma),f(d.B.worst_qdot),f(d.B.worst_witness),
            (double)d.speed_pass,(double)d.H.attempted,(double)d.heading_pass,(double)d.H.reason,
            f(d.H.min_g0),f(d.H.min_lb),(double)d.H.leaves,(double)d.H.splits,
            f(d.heading_gamma),(double)d.rejoin_attempt,(double)d.rejoin_pass,
            (double)d.geom_attempt,(double)d.geom_pass,f(d.geom_min_margin),
            0.0
        };
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];
    }
    return M;
}


static mxArray* r53_knot_diag_matrix(){
    const mwSize N=(mwSize)G_r53_knot_diag.size(),C=18;
    mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    auto f=[](double x){return std::isfinite(x)?x:NAN;};
    for(mwSize i=0;i<N;++i){
        const auto&r=G_r53_knot_diag[i];
        double v[C]={
            r.build_id,r.side,f(r.rejoin),f(r.corr_v),f(r.corr_a),f(r.corr_j),
            r.node,f(r.knot),f(r.P.x),f(r.P.y),f(r.P.z),f(r.Pnorm),
            f(r.V.x),f(r.V.y),f(r.V.z),f(r.Vnorm),f(r.prevT),f(r.nextT)
        };
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];
    }
    return M;
}
static mxArray* r53_segment_diag_matrix(){
    const mwSize N=(mwSize)G_r53_segment_diag.size(),C=15;
    mxArray*M=mxCreateDoubleMatrix(N,C,mxREAL);double*p=mxGetPr(M);
    auto f=[](double x){return std::isfinite(x)?x:NAN;};
    for(mwSize i=0;i<N;++i){
        const auto&r=G_r53_segment_diag[i];
        double v[C]={
            r.build_id,r.side,f(r.rejoin),f(r.corr_v),f(r.corr_a),f(r.corr_j),
            r.seg,f(r.ta),f(r.tb),f(r.T),f(r.cp_norm),
            f(r.vel_ub),f(r.acc_ub),f(r.jerk_ub),0.0
        };
        for(mwSize j=0;j<C;++j)p[i+N*j]=v[j];
    }
    return M;
}

static const mxArray* fld(const mxArray* s,const char* name){
    if(!mxIsStruct(s)) mexErrMsgIdAndTxt("AF7R34:Packet","packet must be struct.");
    const mxArray* v=mxGetField(s,0,name);
    if(!v) mexErrMsgIdAndTxt("AF7R34:Packet","missing field %s.",name);
    return v;
}
static double scalar(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetNumberOfElements(x)!=1)
        mexErrMsgIdAndTxt("AF7R34:Input","%s must be real double scalar.",name);
    double v=mxGetScalar(x);
    if(!std::isfinite(v)) mexErrMsgIdAndTxt("AF7R34:Input","%s must be finite.",name);
    return v;
}
static std::vector<double> dvec(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x))
        mexErrMsgIdAndTxt("AF7R34:Input","%s must be real double.",name);
    size_t n=mxGetNumberOfElements(x);const double* p=mxGetPr(x);
    return std::vector<double>(p,p+n);
}
static std::string str(const mxArray* x,const char* name){
    if(!mxIsChar(x)) mexErrMsgIdAndTxt("AF7R34:Input","%s must be char.",name);
    char* c=mxArrayToString(x);if(!c)mexErrMsgIdAndTxt("AF7R34:Input","cannot read %s.",name);
    std::string s(c);mxFree(c);return s;
}
static mxArray* logicalScalar(bool v){return mxCreateLogicalScalar(v);}
static mxArray* dscalar(double v){return mxCreateDoubleScalar(v);}


// ============================================================================
// AF7R58ZZY-R28 native compiled-representation helpers.
// These functions DO NOT participate in trajectory acceptance. They construct
// the exact R23 compiled evaluator representation from the already generated
// degree-15 candidate and an already-certified active compiled base.
// ============================================================================
static mxArray* vecrow(const std::vector<double>&v);
static const char* R28_REV="AF7R58ZZY_R28_NATIVE_ONCE_V1";
static const char* PLANNER_REV="AF7R58R16_C7_LOWJET_REFERENCE_BRIDGE_V1";

static const mxArray* optfld(const mxArray* s,const char* name){
    if(!s||!mxIsStruct(s)||mxGetNumberOfElements(s)!=1) return nullptr;
    return mxGetField(s,0,name);
}
static bool logical_value(const mxArray* x,const char* name){
    if(!x||mxGetNumberOfElements(x)!=1)
        mexErrMsgIdAndTxt("AF7R28:CompiledBase","%s must be scalar.",name);
    if(mxIsLogical(x)) return mxIsLogicalScalarTrue(x);
    if(mxIsDouble(x)&&!mxIsComplex(x)) return mxGetScalar(x)!=0.0;
    mexErrMsgIdAndTxt("AF7R28:CompiledBase","%s must be logical/double scalar.",name);
    return false;
}
static long nonneg_int(double x,const char* name){
    if(!std::isfinite(x)||x<0.0||x!=std::floor(x))
        mexErrMsgIdAndTxt("AF7R28:CompiledBase","%s must be a nonnegative integer.",name);
    return (long)x;
}
static void require_bitwise_finite_array(const double* p,size_t n,const char* name){
    for(size_t i=0;i<n;++i) if(!std::isfinite(p[i]))
        mexErrMsgIdAndTxt("AF7R28:Finite","%s contains non-finite data.",name);
}
static void write_segment_dcp(double* out,size_t outSeg,const Poly& cp,double T){
    if(cp.size()!=16||!(std::isfinite(T)&&T>0.0))
        mexErrMsgIdAndTxt("AF7R28:DCP","invalid degree-15 segment.");
    double q[3*16]; double qnext[3*16];
    for(size_t k=0;k<16;++k){
        if(!(std::isfinite(cp[k].x)&&std::isfinite(cp[k].y)&&std::isfinite(cp[k].z)))
            mexErrMsgIdAndTxt("AF7R28:DCP","candidate control point is non-finite.");
        q[3*k+0]=cp[k].x; q[3*k+1]=cp[k].y; q[3*k+2]=cp[k].z;
    }
    const size_t block=3*16;
    double* dst0=out + block*(0 + 7*outSeg);
    for(size_t i=0;i<block;++i) dst0[i]=q[i];
    size_t ncols=16;
    for(size_t r=1;r<=6;++r){
        const double factor=static_cast<double>(16-r)/T;
        const size_t newcols=ncols-1;
        for(size_t j=0;j<newcols;++j){
            for(size_t k=0;k<3;++k){
                volatile double d=q[k+3*(j+1)]-q[k+3*j];
                qnext[k+3*j]=factor*d;
            }
        }
        double* dst=out + block*(r + 7*outSeg);
        for(size_t j=0;j<newcols;++j)
            for(size_t k=0;k<3;++k) dst[k+3*j]=qnext[k+3*j];
        for(size_t i=0;i<3*newcols;++i) q[i]=qnext[i];
        ncols=newcols;
    }
}
static std::vector<Poly> cp3_to_poly(const mxArray* A,size_t S){
    if(!mxIsDouble(A)||mxIsComplex(A)) mexErrMsgIdAndTxt("AF7R28:Shape","cp0 must be real double.");
    const mwSize nd=mxGetNumberOfDimensions(A); const mwSize* d=mxGetDimensions(A);
    if(!(nd==2||nd==3)||d[0]!=3||d[1]!=16||((nd>=3?d[2]:1)!=S))
        mexErrMsgIdAndTxt("AF7R28:Shape","cp0 must be 3x16xS.");
    const double* p=mxGetPr(A); std::vector<Poly> out(S);
    for(size_t s=0;s<S;++s){
        out[s].resize(16);
        for(size_t k=0;k<16;++k){
            double x=p[(3*k+0)+48*s],y=p[(3*k+1)+48*s],z=p[(3*k+2)+48*s];
            if(!(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)))
                mexErrMsgIdAndTxt("AF7R28:Finite","cp0 contains non-finite value.");
            out[s][k]={x,y,z};
        }
    }
    return out;
}

struct BaseCompiledView{
    bool present=false;
    long G=0,S=0;
    std::vector<double> gt0,gt1,gfirst,gcount,s0,s1;
    const double* dcp=nullptr;
};
static BaseCompiledView parse_base_compiled(const mxArray* B,double targetVersion){
    BaseCompiledView V;
    if(!B||!mxIsStruct(B)||mxGetNumberOfElements(B)!=1||mxGetNumberOfFields(B)==0) return V;
    const mxArray* sch=optfld(B,"schema");
    if(!sch) return V;
    if(str(sch,"schema")!="AF7R58ZZY_R23_COMPILED_CORRECTION_V1")
        mexErrMsgIdAndTxt("AF7R28:CompiledBase","base schema mismatch.");
    double rv=scalar(fld(B,"reference_version"),"reference_version");
    if(rv!=targetVersion-1.0) mexErrMsgIdAndTxt("AF7R28:CompiledBase","base version mismatch.");
    V.G=nonneg_int(scalar(fld(B,"group_count"),"group_count"),"group_count");
    V.S=nonneg_int(scalar(fld(B,"segment_count"),"segment_count"),"segment_count");
    V.gt0=dvec(fld(B,"group_t0"),"group_t0"); V.gt1=dvec(fld(B,"group_t1"),"group_t1");
    V.gfirst=dvec(fld(B,"group_first"),"group_first"); V.gcount=dvec(fld(B,"group_segments"),"group_segments");
    V.s0=dvec(fld(B,"segment_t0"),"segment_t0"); V.s1=dvec(fld(B,"segment_t1"),"segment_t1");
    if((long)V.gt0.size()!=V.G||(long)V.gt1.size()!=V.G||(long)V.gfirst.size()!=V.G||(long)V.gcount.size()!=V.G||
       (long)V.s0.size()!=V.S||(long)V.s1.size()!=V.S)
        mexErrMsgIdAndTxt("AF7R28:CompiledBase","base metadata size mismatch.");
    if(nonneg_int(scalar(fld(B,"degree"),"degree"),"degree")!=15||
       nonneg_int(scalar(fld(B,"max_derivative_order"),"max_derivative_order"),"max_derivative_order")!=6||
       !logical_value(fld(B,"strict_support"),"strict_support")||
       !logical_value(fld(B,"certified_numeric_representation"),"certified_numeric_representation")||
       str(fld(B,"group_addition_order"),"group_addition_order")!="ASCENDING_CANONICAL_GROUP_INDEX")
        mexErrMsgIdAndTxt("AF7R28:CompiledBase","base representation contract mismatch.");
    const mxArray* D=fld(B,"derivative_control_points");
    if(!mxIsDouble(D)||mxIsComplex(D)) mexErrMsgIdAndTxt("AF7R28:CompiledBase","base DCP type invalid.");
    const mwSize nd=mxGetNumberOfDimensions(D); const mwSize* ds=mxGetDimensions(D);
    if(nd!=4||ds[0]!=3||ds[1]!=16||ds[2]!=7||(long)ds[3]!=V.S)
        mexErrMsgIdAndTxt("AF7R28:CompiledBase","base DCP shape invalid.");
    V.dcp=mxGetPr(D); require_bitwise_finite_array(V.dcp,(size_t)V.S*3*16*7,"base DCP");
    for(long g=0;g<V.G;++g){
        long f=nonneg_int(V.gfirst[g],"group_first"); long n=nonneg_int(V.gcount[g],"group_segments");
        if(f<1||n<1||f+n-1>V.S) mexErrMsgIdAndTxt("AF7R28:CompiledBase","base group range invalid.");
        if(!(std::isfinite(V.gt0[g])&&std::isfinite(V.gt1[g])&&V.gt1[g]>V.gt0[g]))
            mexErrMsgIdAndTxt("AF7R28:CompiledBase","base group support invalid.");
    }
    V.present=true; return V;
}
static mxArray* compiled_packet_struct(const mxArray* baseA,const std::vector<double>& ns0,
    const std::vector<double>& ns1,const std::vector<Poly>& ncp,
    double nt0,double nt1,double targetVersion){
    if(!(std::isfinite(nt0)&&std::isfinite(nt1)&&nt1>nt0&&std::isfinite(targetVersion)&&targetVersion>=0.0))
        mexErrMsgIdAndTxt("AF7R28:Compile","new-group support/version invalid.");
    const size_t Snew=ns0.size();
    if(Snew<1||ns1.size()!=Snew||ncp.size()!=Snew)
        mexErrMsgIdAndTxt("AF7R28:Compile","new-group segment size invalid.");
    for(size_t s=0;s<Snew;++s)
        if(!(std::isfinite(ns0[s])&&std::isfinite(ns1[s])&&ns1[s]>ns0[s]))
            mexErrMsgIdAndTxt("AF7R28:Compile","new segment duration invalid.");

    BaseCompiledView B=parse_base_compiled(baseA,targetVersion);
    std::vector<long> keepG; size_t Sk=0;
    for(long g=0;g<B.G;++g){
        if(B.gt1[g]>nt0){ // exact complement of AF7R58I prune t1<=t_commit
            keepG.push_back(g); Sk+=(size_t)nonneg_int(B.gcount[g],"group_segments");
        }
    }
    const size_t G=keepG.size()+1, S=Sk+Snew;
    std::vector<double> gt0,gt1,gfirst,gcount,s0,s1;
    gt0.reserve(G);gt1.reserve(G);gfirst.reserve(G);gcount.reserve(G);s0.reserve(S);s1.reserve(S);
    size_t cursor=1;
    for(long g:keepG){
        long first=nonneg_int(B.gfirst[g],"group_first"); long n=nonneg_int(B.gcount[g],"group_segments");
        gt0.push_back(B.gt0[g]);gt1.push_back(B.gt1[g]);gfirst.push_back((double)cursor);gcount.push_back((double)n);
        for(long j=0;j<n;++j){long si=first-1+j;s0.push_back(B.s0[si]);s1.push_back(B.s1[si]);}
        cursor+=(size_t)n;
    }
    gt0.push_back(nt0);gt1.push_back(nt1);gfirst.push_back((double)cursor);gcount.push_back((double)Snew);
    s0.insert(s0.end(),ns0.begin(),ns0.end()); s1.insert(s1.end(),ns1.begin(),ns1.end());
    if(cursor+Snew!=S+1) mexErrMsgIdAndTxt("AF7R28:Compile","group cursor mismatch.");

    const char* fn[]={"schema","reference_version","group_count","segment_count","group_t0","group_t1",
        "group_first","group_segments","segment_t0","segment_t1","derivative_control_points",
        "max_derivative_order","degree","strict_support","group_addition_order","certified_numeric_representation"};
    mxArray* O=mxCreateStructMatrix(1,1,sizeof(fn)/sizeof(fn[0]),fn);
    auto set=[&](const char*n,mxArray*v){mxSetField(O,0,n,v);};
    set("schema",mxCreateString("AF7R58ZZY_R23_COMPILED_CORRECTION_V1"));
    set("reference_version",dscalar(targetVersion));set("group_count",dscalar((double)G));set("segment_count",dscalar((double)S));
    set("group_t0",vecrow(gt0));set("group_t1",vecrow(gt1));set("group_first",vecrow(gfirst));set("group_segments",vecrow(gcount));
    set("segment_t0",vecrow(s0));set("segment_t1",vecrow(s1));
    mwSize dims[4]={3,16,7,(mwSize)S}; mxArray* D=mxCreateNumericArray(4,dims,mxDOUBLE_CLASS,mxREAL); double* out=mxGetPr(D);
    size_t outSeg=0; const size_t segBlock=3*16*7;
    for(long g:keepG){
        long first=nonneg_int(B.gfirst[g],"group_first"); long n=nonneg_int(B.gcount[g],"group_segments");
        for(long j=0;j<n;++j){
            long si=first-1+j;
            std::memcpy(out+segBlock*outSeg,B.dcp+segBlock*(size_t)si,segBlock*sizeof(double));
            ++outSeg;
        }
    }
    for(size_t s=0;s<Snew;++s){write_segment_dcp(out,outSeg,ncp[s],ns1[s]-ns0[s]);++outSeg;}
    if(outSeg!=S) mexErrMsgIdAndTxt("AF7R28:Compile","DCP segment cursor mismatch.");
    set("derivative_control_points",D);set("max_derivative_order",dscalar(6));set("degree",dscalar(15));
    set("strict_support",logicalScalar(true));set("group_addition_order",mxCreateString("ASCENDING_CANONICAL_GROUP_INDEX"));
    set("certified_numeric_representation",logicalScalar(true));
    return O;
}
static mxArray* dcp_only_from_cp(const mxArray* cpA,const mxArray* durA){
    auto dur=dvec(durA,"duration"); size_t S=dur.size(); auto cps=cp3_to_poly(cpA,S);
    mwSize dims[4]={3,16,7,(mwSize)S}; mxArray* D=mxCreateNumericArray(4,dims,mxDOUBLE_CLASS,mxREAL); double*out=mxGetPr(D);
    for(size_t s=0;s<S;++s) write_segment_dcp(out,s,cps[s],dur[s]);
    return D;
}

static ActiveReference parse_ref(const mxArray* P){
    if(!G_compiled) mexErrMsgIdAndTxt("AF7R34:Reference","nominal AST is not compiled.");
    ActiveReference R;R.nominal=&G_nominal;
    R.flight_t0=scalar(fld(P,"flight_t0"),"flight_t0");

    auto offs=dvec(fld(P,"repair_offsets"),"repair_offsets");
    auto rt0=dvec(fld(P,"repair_t0"),"repair_t0");
    auto rt1=dvec(fld(P,"repair_t1"),"repair_t1");
    auto s0=dvec(fld(P,"repair_seg0"),"repair_seg0");
    auto s1=dvec(fld(P,"repair_seg1"),"repair_seg1");
    const mxArray* cpA=fld(P,"repair_cp");
    if(!mxIsDouble(cpA)||mxIsComplex(cpA)||mxGetM(cpA)!=48)
        mexErrMsgIdAndTxt("AF7R34:Packet","repair_cp must be 48xS.");
    size_t S=mxGetN(cpA);const double* cp=mxGetPr(cpA);
    if(s0.size()!=S||s1.size()!=S) mexErrMsgIdAndTxt("AF7R34:Packet","repair segment size mismatch.");
    size_t G=rt0.size();
    if(rt1.size()!=G||offs.size()!=G+1) mexErrMsgIdAndTxt("AF7R34:Packet","repair group size mismatch.");

    // AF7R51 optional general-C7 phase metadata. Historical packets without
    // these fields remain exact identity-time references.
    std::vector<double> pen,pt0,ptA,ptB,ptE,psA,psB,palpha,pdelay;
    const mxArray* penA=optfld(P,"repair_phase_enabled");
    if(penA){
        pen=dvec(penA,"repair_phase_enabled");
        pt0=dvec(fld(P,"repair_phase_t0"),"repair_phase_t0");
        ptA=dvec(fld(P,"repair_phase_tA"),"repair_phase_tA");
        ptB=dvec(fld(P,"repair_phase_tB"),"repair_phase_tB");
        ptE=dvec(fld(P,"repair_phase_tE"),"repair_phase_tE");
        psA=dvec(fld(P,"repair_phase_sA"),"repair_phase_sA");
        psB=dvec(fld(P,"repair_phase_sB"),"repair_phase_sB");
        palpha=dvec(fld(P,"repair_phase_alpha"),"repair_phase_alpha");
        pdelay=dvec(fld(P,"repair_phase_delay"),"repair_phase_delay");
        if(pen.size()!=G||pt0.size()!=G||ptA.size()!=G||ptB.size()!=G||
                ptE.size()!=G||psA.size()!=G||psB.size()!=G||
                palpha.size()!=G||pdelay.size()!=G)
            mexErrMsgIdAndTxt("AF7R51:Packet","repair phase group size mismatch.");
    }
    R.repairs.resize(G);
    for(size_t g=0;g<G;++g){
        long a=(long)std::llround(offs[g]),b=(long)std::llround(offs[g+1]);
        if(a<0||b<a||b>(long)S) mexErrMsgIdAndTxt("AF7R34:Packet","invalid repair offsets.");
        R.repairs[g].t0=rt0[g];R.repairs[g].t1=rt1[g];
        if(!pen.empty() && pen[g]>0.5){
            auto &ph=R.repairs[g].phase;
            ph.enabled=true;ph.t0=pt0[g];ph.tA=ptA[g];ph.tB=ptB[g];ph.tE=ptE[g];
            ph.sA=psA[g];ph.sB=psB[g];ph.alpha=palpha[g];ph.delay=pdelay[g];
            if(!r51_phase_valid(ph))
                mexErrMsgIdAndTxt("AF7R51:Packet","invalid repair phase metadata.");
        }
        for(long j=a;j<b;++j){
            CorrSegment sg;sg.t0=s0[j];sg.t1=s1[j];sg.cp.resize(16);
            for(int k=0;k<16;++k)sg.cp[k]={cp[(3*k+0)+48*j],cp[(3*k+1)+48*j],cp[(3*k+2)+48*j]};
            R.repairs[g].seg.push_back(std::move(sg));
        }
    }
    return R;
}
static Live parse_live(const mxArray* P){
    Live L;
    L.dt=scalar(fld(P,"dt"),"dt");L.g=scalar(fld(P,"gravity"),"gravity");L.L=scalar(fld(P,"cable_length"),"cable_length");
    L.rL=scalar(fld(P,"r_load"),"r_load");L.rQ=scalar(fld(P,"r_drone"),"r_drone");L.rC=scalar(fld(P,"r_cable"),"r_cable");
    L.dmargin=scalar(fld(P,"d_margin"),"d_margin");L.sensor_range=scalar(fld(P,"sensor_range"),"sensor_range");
    L.horizon=scalar(fld(P,"horizon"),"horizon");L.tracking=scalar(fld(P,"tracking_inflation"),"tracking_inflation");
    // R13 module-internal diagnostic switch, defaults OFF if absent.
    const mxArray* bridge16=optfld(P,"r58r16_reference_bridge");
    if(bridge16){
        const double f=scalar(bridge16,"r58r16_reference_bridge");
        if(f!=0.0&&f!=1.0)
            mexErrMsgIdAndTxt("R16:Bridge","r58r16_reference_bridge must be 0 or 1.");
        L.r58r16_reference_bridge=(f==1.0);
    }
    const mxArray* detail58=optfld(P,"r58_detail_diagnostic_enabled");
    if(detail58){
        const double f=mxIsLogical(detail58) ?
            (mxIsLogicalScalarTrue(detail58)?1.0:0.0) :
            scalar(detail58,"r58_detail_diagnostic_enabled");
        if(mxGetNumberOfElements(detail58)!=1||(f!=0.0&&f!=1.0))
            mexErrMsgIdAndTxt("R58:Detail","r58_detail_diagnostic_enabled must be scalar 0 or 1.");
        L.r58_detail_diagnostic_enabled=(f==1.0);
    }
    const mxArray* r58diag=optfld(P,"r58_shadow_diagnostic_enabled");
    if(r58diag){
        if(mxGetNumberOfElements(r58diag)!=1)
            mexErrMsgIdAndTxt("AF7R13:Flag","r58 shadow diagnostic flag must be 0 or 1.");
        double f=mxIsLogical(r58diag)?(mxIsLogicalScalarTrue(r58diag)?1.0:0.0):
                 scalar(r58diag,"r58_shadow_diagnostic_enabled");
        if(!(f==0.0||f==1.0))
            mexErrMsgIdAndTxt("AF7R13:Flag","r58 shadow diagnostic flag must be 0 or 1.");
        L.r58_shadow_diagnostic_enabled=(f==1.0);
    }
    // R14 diagnostic-only nested C7 candidate family, strict 0/1 flag.
    const mxArray* r14mode=optfld(P,"r58r14_lowjet_seed_only");
    if(r14mode){
        if(mxGetNumberOfElements(r14mode)!=1)
            mexErrMsgIdAndTxt("AF7R14:Flag","R14 seed flag must be 0 or 1.");
        const double f=mxIsLogical(r14mode) ?
            (mxIsLogicalScalarTrue(r14mode)?1.0:0.0) :
            scalar(r14mode,"r58r14_lowjet_seed_only");
        if(!(f==0.0||f==1.0))
            mexErrMsgIdAndTxt("AF7R14:Flag","R14 seed flag must be 0 or 1.");
        L.r58r14_lowjet_seed_only=(f==1.0);
    }
    return L;
}
static SpeedCert parse_speed(const mxArray* P){
    SpeedCert S;S.certified=true;
    S.VL=scalar(fld(P,"base_payload_speed"),"base_payload_speed");
    S.A=scalar(fld(P,"base_payload_accel"),"base_payload_accel");
    S.J=scalar(fld(P,"base_payload_jerk"),"base_payload_jerk");
    S.sigma=scalar(fld(P,"base_sigma"),"base_sigma");
    S.qdot=scalar(fld(P,"base_qdot"),"base_qdot");
    S.VQ=scalar(fld(P,"base_uav_speed"),"base_uav_speed");
    return S;
}
static std::vector<Obstacle> parse_obs(const mxArray* P){
    const mxArray* C=fld(P,"centers"),*R=fld(P,"rotations_flat"),*A=fld(P,"base_axes"),*I=fld(P,"ids");
    if(!mxIsDouble(C)||mxIsComplex(C)||mxGetM(C)!=3) mexErrMsgIdAndTxt("AF7R34:Packet","centers must be 3xM.");
    size_t M=mxGetN(C);
    if(!mxIsDouble(R)||mxIsComplex(R)||mxGetM(R)!=9||mxGetN(R)!=M) mexErrMsgIdAndTxt("AF7R34:Packet","rotations_flat must be 9xM.");
    if(!mxIsDouble(A)||mxIsComplex(A)||mxGetM(A)!=3||mxGetN(A)!=M) mexErrMsgIdAndTxt("AF7R34:Packet","base_axes must be 3xM.");
    if(!mxIsDouble(I)||mxIsComplex(I)||mxGetNumberOfElements(I)!=M) mexErrMsgIdAndTxt("AF7R34:Packet","ids must have M entries.");
    const double* c=mxGetPr(C),*r=mxGetPr(R),*a=mxGetPr(A),*id=mxGetPr(I);
    std::vector<Obstacle> O(M);
    for(size_t j=0;j<M;++j){
        O[j].c={c[3*j],c[3*j+1],c[3*j+2]};
        for(int rr=0;rr<3;++rr)for(int cc=0;cc<3;++cc)O[j].R(rr,cc)=r[(rr+3*cc)+9*j];
        O[j].axes={a[3*j],a[3*j+1],a[3*j+2]};O[j].id=id[j];
    }
    return O;
}
static mxArray* vecrow(const std::vector<double>&v){
    mxArray* x=mxCreateDoubleMatrix(1,v.size(),mxREAL);double*p=mxGetPr(x);for(size_t i=0;i<v.size();++i)p[i]=v[i];return x;
}
static mxArray* ivecrow(const std::vector<int>&v){
    mxArray* x=mxCreateDoubleMatrix(1,v.size(),mxREAL);double*p=mxGetPr(x);for(size_t i=0;i<v.size();++i)p[i]=v[i];return x;
}
static mxArray* vec3mat(const std::vector<Vec3>&v){
    mxArray*x=mxCreateDoubleMatrix(3,v.size(),mxREAL);double*p=mxGetPr(x);
    for(size_t j=0;j<v.size();++j){p[3*j]=v[j].x;p[3*j+1]=v[j].y;p[3*j+2]=v[j].z;}return x;
}
static mxArray* cp_cell(const std::vector<Poly>&cp){
    mxArray*c=mxCreateCellMatrix(1,cp.size());
    for(size_t s=0;s<cp.size();++s){
        mxArray*x=mxCreateDoubleMatrix(3,cp[s].size(),mxREAL);double*p=mxGetPr(x);
        for(size_t k=0;k<cp[s].size();++k){p[3*k]=cp[s][k].x;p[3*k+1]=cp[s][k].y;p[3*k+2]=cp[s][k].z;}
        mxSetCell(c,s,x);
    }return c;
}
static mxArray* result_struct(const PlanResult&R,double t0,double tm,double searchEnd,const std::vector<Obstacle>&obs,const mxArray* packet){
    const char* fn[]={"success","reason_code","t0","tm","hold_end","t1",
        "episode_count","component_count","episode_obstacle_index","episode_obstacle_id",
        "episode_start","episode_end","episode_component_index","component_start","component_end",
        "segment_t0","segment_t1","control_points","component_position","component_velocity",
        "corridor_signs","qp_cost","correction_derivative_ub","max_control_norm_m",
        "displacement","direction","speed_payload","speed_accel","speed_jerk","speed_sigma",
        "speed_qdot","speed_uav","speed_stageA","speed_leaf_count","speed_split_count",
        "geometry_pass","geometry_reason","geometry_leaf_count","geometry_accepted_leaf_count",
        "geometry_split_count","geometry_max_depth","geometry_min_payload_margin",
        "geometry_min_uav_margin","geometry_min_cable_margin","geometry_min_margin",
        "speed_pass_count","rejoin_pass_count","geometry_test_count",
        "episode_ms","hessian_ms","corridor_ms","qp_ms","candidate_build_ms","synthesis_ms","candidate_ms","total_ms",
        "compiled_correction","compiled_native_exact","compiled_native_ms","compiled_native_revision",
        "search_end","selected_t1","rejoin_seed_duration_s","rejoin_selected_duration_s",
        "rejoin_next_gap_s","rejoin_attempt_count","rejoin_safe_trial_count",
        "rejoin_scope_component_count","rejoin_merged_component_count",
        "search_speed_pass_total","search_rejoin_pass_total","search_geometry_test_total",
        "spatial_envelope_priority","compact_constant_translation","stored_c7_exact","r24e_family_seen","r24e_family_exact","r24e_family_exact_reject","r24e_family_speed_pass","r24e_family_speed_fail","r24e_split_representability_reject","minimum_deviation_primary",
        "candidate_family","primary_candidate_count","primary_speed_pass_count",
        "primary_rejoin_pass_count","primary_geometry_test_count",
        "fallback_candidate_count","fallback_speed_pass_count",
        "fallback_rejoin_pass_count","fallback_geometry_test_count",
        "towing_candidate_count","towing_speed_pass_count",
        "towing_rejoin_pass_count","towing_geometry_test_count",
        "finite_sensing_limit_m_s","primary_min_stageA_uav_speed_UB_m_s",
        "primary_min_stageA_specific_force_LB_m_s2",
        "primary_min_stageA_correction_speed_UB_m_s",
        "primary_min_stageA_correction_accel_UB_m_s2",
        "primary_min_stageA_correction_jerk_UB_m_s3",
        "primary_min_envelope_m","primary_stageA_certified_count",
        "primary_stageB_certified_count","primary_stageB_finite_witness_count",
        "primary_stageB_repr_fail_count","towing_diag_revision",
        "r28_flow_side_assignments","r28_flow_qp_solutions","r28_flow_built",
        "r28_flow_row_reject","r28_flow_grid_reject","r28_flow_support_reject",
        "r28_flow_step_s","r28_flow_entry_end_s","r28_flow_exit_start_s",
        "r28_flow_exit_end_s",
        "r30_flow_support_start_s","r30_flow_wait_s",
        "r30_first_unsafe_s","r30_last_unsafe_s","r30_reference_end_s",
        "r30_lead_s","r30_tail_s",
        "r31_entry_duration_s","r31_flow_segment_duration_s","r31_exit_duration_s",
        "r31_best_correction_speed_ub","r31_best_stageA_uav_speed_ub",
        "r31_exact_join_reject",
        "r32_affine_base_detected","r32_timing_trial_count",
        "r32_exact_candidate_count","r32_monotone_reject_count",
        "r32_nominal_progress_speed","r32_best_pre_speed_ub",
        "r32_best_progress_scale","r32_best_entry_T",
        "r32_best_flow_T","r32_best_exit_T",
        "r33_exact_timing_candidate_count","r33_pareto_candidate_count",
        "r33_quantized_progress_reject_count","r33_time_quantum_s",
        "r33_support_start_s","r33_support_end_s",
        "r33_best_accel_ub","r33_best_jerk_ub",
        "delay_preserving","terminal_hold_start","terminal_offset",
        "schedule_delay_s",
        "r34_delay_candidate_count","r34_terminal_hold_exact_count",
        "r34_monotone_progress_count","r34_selected_delay_s",
        "r34_selected_hold_start_s","r34_selected_alpha",
        "r34_selected_velocity_ub","r34_selected_accel_ub",
        "r34_selected_jerk_ub",
        "path_tangent_yaw_c6","heading_gamma_m_s","heading_anchor_rad",
        "heading_base_velocity",
        "r35_heading_candidate_count","r35_heading_pass_count",
        "r35_heading_reject_count","r35_best_heading_gamma_m_s",
        "r35_selected_heading_gamma_m_s","r35_selected_heading_anchor_rad",
        "r36_spatial_qp_solutions","r36_spatial_row_rejects",
        "r36_timing_trials","r36_exact_curved_candidates",
        "r36_curved_heading_pass","r36_mid_progress_s",
        "r36_best_spatial_path_speed_ub","r36_best_total_velocity_ub",
        "r36_best_accel_ub","r36_best_jerk_ub",
        "trigger_to_free_rejoin","identity_time_law",
        "actual_rejoin_time","spatial_knot_count",
        "r37_side_assignments","r37_spatial_qp_solutions",
        "r37_exact_c7_candidates","r37_heading_pass_count",
        "r37_knot_count","r37_support_start_s","r37_actual_rejoin_s",
        "r37_best_total_velocity_ub","r37_best_accel_ub","r37_best_jerk_ub",
        "constructive_exact_c7","legacy_float_endpoint_checker_pass",
        "r38_constructive_c7_count","r38_legacy_float_checker_pass_count",
        "r38_legacy_float_false_reject_count",
        "r45_tangent_interval_count","r45_zero_offset_interval_count",
        "r45_convex_search_fallback_count",
        "r45_best_fixed_peak_req_m","r45_best_tangent_peak_req_m",
        "r45_max_peak_relief_m",
        "r47_active_base_replan_count","r47_nonaffine_base_count",
        "r47_heading_leaf_count","r47_heading_split_count",
        "r48_natural_rejoin_trial_count","r48_natural_rejoin_selected_count",
        "r48_exit_monotone_row_count","r48_natural_rejoin_seed_t1",
        "r49_phase_timing_trial_count","r49_phase_gate_pass_count",
        "r49_hard_certificate_trial_count",
        "r49_selected_Ke","r49_selected_Kflow","r49_selected_Kx",
        "r50_spatial_side_solution_count","r50_spatial_lateral_constraint_count",
        "r50_entry_candidate_count","r50_exit_candidate_count",
        "r50_best_spatial_velocity_norm",
        "r51_phase_seed_count","r51_phase_seed_reject_count",
        "r51_tangent_candidate_count","r51_tangent_certified_count",
        "r51_phase_alpha","r51_phase_delay_s","r51_phase_exit_time_s",
        "r52_phase_episode_count","r52_phase_episode_rebuild_fail_count",
        "r52_corridor_row_count","r52_support_plane_fail_count",
        "r52_spatial_qp_solution_count","r52_rejoin_time_trial_count",
        "r52_selected_spatial_rejoin_s",
        "r53_normal_bundle_node_count","r53_normal_bundle_row_count",
        "r53_normal_bundle_singular_count","r53_affine_reduction_count",
        "r54_phase_candidate_count","r54_phase_trial_count",
        "r54_phase_base_certified_count","r54_phase_episode_pass_count",
        "r54_phase_episode_empty_count","r54_selected_tau_multiple",
        "r54_selected_alpha","r54_selected_delay_s",
        "r54_selected_first_unsafe_lead_s","r54_max_first_unsafe_lead_s",
        "r54_maneuver_horizon_coupling_count",
        "r55_phase_build_count","r55_phase_build_fail_count",
        "r55_phase_episode_pass_count","r55_phase_episode_empty_count",
        "r55_mixed_side_episode_count","r55_side_plus_count",
        "r55_side_minus_count","r55_side_onebranch_count",
        "r55_side_both_failed_count","r55_corridor_row_count",
        "r55_obstacle_count","r55_old_clock_binding_count",
        "r55_maneuver_horizon_coupling_count",
        "r55_alpha","r55_delay_s",
        "r55_source_unsafe_start_s","r55_source_unsafe_end_s",
        "r55_predicted_first_unsafe_t_s","r55_predicted_last_unsafe_t_s",
        "r55_pre_unsafe_slack_s","r55_post_unsafe_slack_s",
        "r55_min_temporal_slack_s",
        "r55_phase_entry_transition_s","r55_phase_exit_transition_s",
        "r55_source_phase_at_rejoin_s","r55_rejoin_clock_delay_s",
        "r55_min_ellipsoid_axis_m","r55_max_ellipsoid_axis_m",
        "r55_max_rotation_orth_error",
        "r56_shadow_ran",
        "r56_source_qp_ok",
        "r56_source_exact_c7",
        "r56_source_knot_count",
        "r56_source_episode_count",
        "r56_source_corridor_row_count",
        "r56_source_normal_row_count",
        "r56_source_side_plus_count",
        "r56_source_side_minus_count",
        "r56_source_side_fail_count",
        "r56_const_sample_count",
        "r56_const_dynamic_pass",
        "r56_const_geometry_point_pass",
        "r56_entry_bestcase_pass",
        "r56_exit_bestcase_pass",
        "r56_yaw_source_regular",
        "r56_old_clock_binding_count",
        "r56_horizon_duration_coupling_count",
        "r56_source_support_start_s",
        "r56_source_rejoin_s",
        "r56_source_corr_v_ub",
        "r56_source_corr_a_ub",
        "r56_source_corr_j_ub",
        "r56_source_min_hxy",
        "r56_alpha_max_pointwise",
        "r56_const_max_vl",
        "r56_const_max_vq",
        "r56_const_min_sigma",
        "r56_const_max_qdot",
        "r56_const_min_geom_margin",
        "r56_entry_Tmax_s",
        "r56_entry_max_vq",
        "r56_entry_min_sigma",
        "r56_entry_max_qdot",
        "r56_exit_Tmax_s",
        "r56_exit_max_vq",
        "r56_exit_min_sigma",
        "r56_exit_max_qdot",
        "r56_chain_rule_max_error",
        "r56_shadow_ms",
        "r57_probe_ran",
        "r57_source_qp_ok",
        "r57_source_exact_c7",
        "r57_speed_attempt",
        "r57_speed_pass",
        "r57_speed_stageA_pass",
        "r57_speed_stageB_attempt",
        "r57_speed_stageB_finite_witness",
        "r57_speed_stageB_repr_fail",
        "r57_heading_attempt",
        "r57_heading_pass",
        "r57_heading_leaf_count",
        "r57_heading_split_count",
        "r57_geometry_attempt",
        "r57_geometry_pass",
        "r57_commit_attempted",
        "r57_rejoin_selected",
        "r57_old_clock_commit_binding_count",
        "r57_horizon_duration_coupling_count",
        "r57_speed_VL",
        "r57_speed_A",
        "r57_speed_J",
        "r57_speed_sigma",
        "r57_speed_qdot",
        "r57_speed_VQ",
        "r57_heading_gamma",
        "r57_geometry_min_margin",
        "r57_source_support_start_s",
        "r57_source_support_end_s",
        "r57_source_corr_v_ub",
        "r57_source_corr_a_ub",
        "r57_source_corr_j_ub",
        "r57_source_build_ms",
        "r57_speed_ms",
        "r57_heading_ms",
        "r57_geometry_ms",
        "r57_probe_ms",
        "r58_shadow_ran",
        "r58_reference_only",
        "r58_fixed_rejoin_constraint_count",
        "r58_same_clock_binding_count",
        "r58_horizon_duration_coupling_count",
        "r58_free_c7_constructive",
        "r58_terminal_offset_hold",
        "r58_terminal_zero_derivative_order",
        "r58_source_knot_count",
        "r58_scalar_dof_count",
        "r58r14_lowjet_seed_only",
        "r58r14_active_reduced_dof",
        "r58r14_full_reduced_dof",
        "r58_spatial_dof_count",
        "r58_episode_count",
        "r58_component_count",
        "r58_side_cache_plus_valid",
        "r58_side_cache_minus_valid",
        "r58_side_cache_fail",
        "r58_bnb_nodes_popped",
        "r58_bnb_qp_solves",
        "r58_bnb_qp_infeasible",
        "r58_bnb_full_assignments",
        "r58_bnb_speed_fail",
        "r58_bnb_heading_fail",
        "r58_bnb_geometry_fail",
        "r58_bnb_feasible_full",
        "r58_bnb_max_queue",
        "r58_selected_side_plus",
        "r58_selected_side_minus",
        "r58_speed_pass",
        "r58_heading_pass",
        "r58_geometry_pass",
        "r58_needs_time_law",
        "r58_commit_attempted",
        "r58_rejoin_selected",
        "r58_reference_tracking_inflation_m",
        "r58_support_start_s",
        "r58_support_end_s",
        "r58_terminal_offset_norm_m",
        "r58_quantized_jet_max_abs_error",
        "r58_legacy_join_max_abs_mismatch",
        "r58_best_qp_cost",
        "r58_speed_VL",
        "r58_speed_A",
        "r58_speed_J",
        "r58_speed_sigma",
        "r58_speed_qdot",
        "r58_speed_VQ",
        "r58_heading_gamma",
        "r58_geometry_min_margin",
        "r58_build_ms",
        "r58_homotopy_ms",
        "r58_certificate_ms",
        "r58_shadow_ms",
        "r58r2_node_scaled_variables",
        "r58r2_row_equilibrated",
        "r58r2_root_qp_elided",
        "r58r2_node_scale_min_s",
        "r58r2_node_scale_max_s",
        "r58r2_row_scale_min",
        "r58r2_row_scale_max",
        "r58r3_hessian_diag_equilibrated",
        "r58r3_equilibrated_cholesky_pass",
        "r58r3_raw_hdiag_min",
        "r58r3_raw_hdiag_max",
        "r58r3_raw_hdiag_ratio",
        "r58r3_eq_hdiag_min",
        "r58r3_eq_hdiag_max",
        "r58r4_normal_equalities_eliminated",
        "r58r4_reduced_spatial_dof_count",
        "r58r4_elimination_build_fail",
        "r58r4_elimination_max_residual",
        "r58r5_dykstra_used",
        "r58r5_dykstra_converged_feasible",
        "r58r5_dykstra_resource_fail",
        "r58r5_dykstra_degenerate_row_fail",
        "r58r5_dykstra_max_sweeps_used",
        "r58r5_dykstra_projection_count",
        "r58r5_dykstra_optimal_kkt_count",
        "r58r5_dykstra_max_violation",
        "r58r5_dykstra_duality_gap",
        "r58r5_dykstra_stationarity_residual",
        "r58r5_dykstra_ms",
        "r58r6_hildreth_used",
        "r58r6_kkt_converged_count",
        "r58r6_resource_fail_count",
        "r58r6_max_sweeps_used",
        "r58r6_coordinate_updates",
        "r58r6_max_primal_violation",
        "r58r6_max_complementarity",
        "r58r6_duality_gap",
        "r58r6_stationarity_residual",
        "r58r6_max_lambda",
        "r58r6_solver_ms",
        "r58r6_prequant_scaled_jet_max_abs",
        "r58r6_quantized_scaled_jet_rel_error",
        "r58r6_quantized_physical_jet_max_abs_error",
        "r58r6_failed_corr_v_ub",
        "r58r6_failed_corr_a_ub",
        "r58r6_failed_corr_j_ub",
        "r58r6_failed_speed_worst_vq",
        "r58r6_failed_speed_worst_vl",
        "r58r6_failed_speed_worst_sigma",
        "r58r6_failed_speed_worst_qdot",
        "r58r6_failed_speed_witness",
        "r58r7_constraint_generation_used",
        "r58r7_full_row_count_max",
        "r58r7_outer_iterations_max",
        "r58r7_full_scans",
        "r58r7_rows_added",
        "r58r7_active_rows_max",
        "r58r7_subset_kkt_solves",
        "r58r7_subset_kkt_converged",
        "r58r7_subset_resource_fail",
        "r58r7_duplicate_add_fail",
        "r58r7_full_optimality_closed",
        "r58r7_subset_sweeps_max",
        "r58r7_coordinate_updates",
        "r58r7_initial_worst_violation",
        "r58r7_final_max_violation",
        "r58r7_final_duality_gap",
        "r58r7_final_complementarity",
        "r58r7_final_stationarity",
        "r58r7_solver_ms",
        "r58r8_ipm_used",
        "r58r8_ipm_converged",
        "r58r8_ipm_resource_fail",
        "r58r8_ipm_factor_fail",
        "r58r8_ipm_max_iterations",
        "r58r8_ipm_newton_solves",
        "r58r8_ipm_primal_residual",
        "r58r8_ipm_dual_residual",
        "r58r8_ipm_complementarity",
        "r58r8_ipm_duality_gap",
        "r58r8_ipm_min_slack",
        "r58r8_ipm_min_lambda",
        "r58r8_ipm_ms",
        "r58r8_newton_diag_equilibrated",
        "r58r8_pcg_used",
        "r58r8_pcg_resource_fail",
        "r58r8_pcg_max_iterations",
        "r58r8_pcg_final_rel_residual",
        "r58r9_failure_count",
        "r58r9_failure_depth",
        "r58r9_failure_last_side",
        "r58r9_failure_outer_iteration",
        "r58r9_failure_active_rows",
        "r58r9_failure_added_row",
        "r58r9_failure_ipm_iteration",
        "r58r9_failure_stage",
        "r58r9_failure_pcg_iterations",
        "r58r9_failure_pcg_final_rel_residual",
        "r58r9_failure_pcg_rhs_inf",
        "r58r9_failure_D_min",
        "r58r9_failure_D_max",
        "r58r9_failure_D_ratio",
        "r58r9_failure_min_slack",
        "r58r9_failure_min_lambda",
        "r58r9_failure_metric_max_abs_corr",
        "r58r9_failure_metric_one_minus_corr",
        "r58r9_full_scan_ms",
        "r58r9_subset_solver_ms",
        "r58r9_pcg_ms",
        "r58r9_plus_failed",
        "r58r9_minus_failed",
        "r58r9_plus_failure_stage",
        "r58r9_minus_failure_stage",
        "r58r9_plus_failure_active_rows",
        "r58r9_minus_failure_active_rows",
        "r58r9_plus_failure_pcg_relres",
        "r58r9_minus_failure_pcg_relres",
        "r58r9_plus_metric_max_abs_corr",
        "r58r9_minus_metric_max_abs_corr",
        "r58r9_plus_cg_ms",
        "r58r9_minus_cg_ms",
        "r58r10_plus_primal_residual",
        "r58r10_plus_dual_residual",
        "r58r10_plus_mu",
        "r58r10_plus_duality_gap",
        "r58r10_plus_kkt_tol",
        "r58r10_plus_gap_tol",
        "r58r10_plus_last_sigma",
        "r58r10_plus_last_alpha_primal",
        "r58r10_plus_last_alpha_dual",
        "r58r10_plus_min_alpha_primal",
        "r58r10_plus_min_alpha_dual",
        "r58r10_plus_mu_ratio",
        "r58r10_plus_D_min",
        "r58r10_plus_D_max",
        "r58r10_plus_D_ratio",
        "r58r10_plus_min_slack",
        "r58r10_plus_min_lambda",
        "r58r10_plus_predictor_pcg_iterations",
        "r58r10_plus_corrector_pcg_iterations",
        "r58r10_plus_predictor_pcg_relres",
        "r58r10_plus_corrector_pcg_relres",
        "r58r10_minus_primal_residual",
        "r58r10_minus_dual_residual",
        "r58r10_minus_mu",
        "r58r10_minus_duality_gap",
        "r58r10_minus_kkt_tol",
        "r58r10_minus_gap_tol",
        "r58r10_minus_last_sigma",
        "r58r10_minus_last_alpha_primal",
        "r58r10_minus_last_alpha_dual",
        "r58r10_minus_min_alpha_primal",
        "r58r10_minus_min_alpha_dual",
        "r58r10_minus_mu_ratio",
        "r58r10_minus_D_min",
        "r58r10_minus_D_max",
        "r58r10_minus_D_ratio",
        "r58r10_minus_min_slack",
        "r58r10_minus_min_lambda",
        "r58r10_minus_predictor_pcg_iterations",
        "r58r10_minus_corrector_pcg_iterations",
        "r58r10_minus_predictor_pcg_relres",
        "r58r10_minus_corrector_pcg_relres",
        "r58r11_augmented_kkt_used",
        "r58r11_augmented_kkt_solve_fail",
        "r58r11_common_step_used",
        "r58r11_augmented_kkt_max_dim",
        "r58r11_newton_solves",
        "r58r11_newton_max_rel_residual",
        "r58r11_common_step_min",
        "r58r11_common_step_last",
        "r58r11_solver_ms","r58_speed_gate_rows","r58r16_connected_candidate",
        "phase_reparameterized","phase_t0","phase_entry_end","phase_flow_end",
        "phase_exit_end","phase_source_start","phase_source_end","phase_alpha","phase_delay_s",
        "rejoin_search_ms","planner_native_revision"};
    mxArray*S=mxCreateStructMatrix(1,1,sizeof(fn)/sizeof(fn[0]),fn);
    auto set=[&](const char*n,mxArray*v){mxSetField(S,0,n,v);};
    set("success",logicalScalar(R.success));set("reason_code",dscalar(R.reason));
    double outT1=fin(R.selected_t1)?R.selected_t1:searchEnd;
    set("t0",dscalar(t0));set("tm",dscalar(tm));set("hold_end",dscalar(R.hold_end));set("t1",dscalar(outT1));
    set("episode_count",dscalar((double)R.episode.ep.size()));set("component_count",dscalar((double)R.episode.comp.size()));
    std::vector<double> oi,oid,es,ee,ec,cs,ce;
    for(auto&e:R.episode.ep){oi.push_back(e.obs+1);oid.push_back(obs[e.obs].id);es.push_back(e.start);ee.push_back(e.end);ec.push_back(e.comp+1);}
    for(auto&c:R.episode.comp){cs.push_back(c.start);ce.push_back(c.end);}
    set("episode_obstacle_index",vecrow(oi));set("episode_obstacle_id",vecrow(oid));
    set("episode_start",vecrow(es));set("episode_end",vecrow(ee));set("episode_component_index",vecrow(ec));
    set("component_start",vecrow(cs));set("component_end",vecrow(ce));
    set("segment_t0",vecrow(R.cand.seg0));set("segment_t1",vecrow(R.cand.seg1));set("control_points",cp_cell(R.cand.cp));
    set("component_position",vec3mat(R.cand.pos));set("component_velocity",vec3mat(R.cand.vel));set("corridor_signs",ivecrow(R.cand.signs));
    set("qp_cost",dscalar(R.cand.cost));
    std::vector<double>du(R.cand.dub.begin(),R.cand.dub.end());set("correction_derivative_ub",vecrow(du));
    set("max_control_norm_m",dscalar(R.cand.max_ctrl));
    Vec3 disp=R.cand.pos.empty()?Vec3{NAN,NAN,NAN}:R.cand.pos.front();
    mxArray*d=mxCreateDoubleMatrix(3,1,mxREAL);mxGetPr(d)[0]=disp.x;mxGetPr(d)[1]=disp.y;mxGetPr(d)[2]=disp.z;set("displacement",d);
    Vec3 dir{0,0,0};double nd=norm(disp);if(nd>0)dir=disp/nd;
    mxArray*di=mxCreateDoubleMatrix(3,1,mxREAL);mxGetPr(di)[0]=dir.x;mxGetPr(di)[1]=dir.y;mxGetPr(di)[2]=dir.z;set("direction",di);
    set("speed_payload",dscalar(R.speed.VL));set("speed_accel",dscalar(R.speed.A));set("speed_jerk",dscalar(R.speed.J));
    set("speed_sigma",dscalar(R.speed.sigma));set("speed_qdot",dscalar(R.speed.qdot));set("speed_uav",dscalar(R.speed.VQ));
    set("speed_stageA",logicalScalar(R.speed.stageA));set("speed_leaf_count",dscalar(R.speed.leaf_count));set("speed_split_count",dscalar(R.speed.split_count));
    set("geometry_pass",logicalScalar(R.geom.pass));set("geometry_reason",dscalar(R.geom.reason));set("geometry_leaf_count",dscalar(R.geom.leaf_count));
    set("geometry_accepted_leaf_count",dscalar(R.geom.accepted));set("geometry_split_count",dscalar(R.geom.splits));set("geometry_max_depth",dscalar(R.geom.max_depth));
    set("geometry_min_payload_margin",dscalar(R.geom.minL));set("geometry_min_uav_margin",dscalar(R.geom.minQ));
    set("geometry_min_cable_margin",dscalar(R.geom.minC));set("geometry_min_margin",dscalar(R.geom.minMargin));
    set("speed_pass_count",dscalar(R.speed_pass));set("rejoin_pass_count",dscalar(R.rejoin_pass));set("geometry_test_count",dscalar(R.geom_tests));
    set("episode_ms",dscalar(R.tim.episode));set("hessian_ms",dscalar(R.tim.hessian));
    set("corridor_ms",dscalar(R.tim.corridor));set("qp_ms",dscalar(R.tim.qp));
    set("candidate_build_ms",dscalar(R.tim.candidate_build));set("synthesis_ms",dscalar(R.tim.synthesis));
    set("candidate_ms",dscalar(R.tim.candidate));set("total_ms",dscalar(R.tim.total));
    set("search_end",dscalar(R.search_end));
    set("selected_t1",dscalar(R.selected_t1));
    set("rejoin_seed_duration_s",dscalar(R.rejoin_seed_duration));
    set("rejoin_selected_duration_s",dscalar(R.rejoin_selected_duration));
    set("rejoin_next_gap_s",dscalar(R.rejoin_next_gap));
    set("rejoin_attempt_count",dscalar(R.rejoin_attempts));
    set("rejoin_safe_trial_count",dscalar(R.rejoin_safe_trials));
    set("rejoin_scope_component_count",dscalar(R.rejoin_scope_components));
    set("rejoin_merged_component_count",dscalar(R.rejoin_merged_components));
    set("search_speed_pass_total",dscalar(R.search_speed_pass_total));
    set("search_rejoin_pass_total",dscalar(R.search_rejoin_pass_total));
    set("search_geometry_test_total",dscalar(R.search_geom_tests_total));
    set("spatial_envelope_priority",logicalScalar(R.spatial_envelope_priority));
    set("compact_constant_translation",logicalScalar(R.cand.family==4||R.cand.family==5));
    set("stored_c7_exact",logicalScalar(R.cand.exact_c7));
    std::vector<int> r24cs(R.r24e_family_seen.begin(),R.r24e_family_seen.end());
    std::vector<int> r24ce(R.r24e_family_exact.begin(),R.r24e_family_exact.end());
    std::vector<int> r24cx(R.r24e_family_exact_reject.begin(),R.r24e_family_exact_reject.end());
    std::vector<int> r24cp(R.r24e_family_speed_pass.begin(),R.r24e_family_speed_pass.end());
    std::vector<int> r24cf(R.r24e_family_speed_fail.begin(),R.r24e_family_speed_fail.end());
    set("r24e_family_seen",ivecrow(r24cs));
    set("r24e_family_exact",ivecrow(r24ce));
    set("r24e_family_exact_reject",ivecrow(r24cx));
    set("r24e_family_speed_pass",ivecrow(r24cp));
    set("r24e_family_speed_fail",ivecrow(r24cf));
    set("r24e_split_representability_reject",dscalar(R.r24e_split_representability_reject));
    set("minimum_deviation_primary",logicalScalar(R.cand.family==1));
    const char* family=(R.cand.family==1)?"MINIMUM_DEVIATION_AFFINE":
                       ((R.cand.family==2)?"MINIMUM_SNAP_FALLBACK":
                       ((R.cand.family==3)?"TOWING_C7_CONDENSED_C3":"NONE"));
    if(R.cand.family==4)family="TOWING_C7_MIN_NORM_CONSTANT";
    if(R.cand.family==5)family="TOWING_C7_SPEED_OPT_SPLIT";
    if(R.cand.family==6)family="CURVED_C7_ZERO_JET_BELL";
    if(R.cand.family==7){
        if(R.r58r16_connected_candidate)
            family="AF7R58R16_LOWJET_FREE_TERMINAL_OFFSET_C7";
        else if(R.cand.phase_reparameterized)
            family="GENERAL_C7_WORLD3D_PHASE_CORRIDOR";
        else if(R.cand.terminal_hold)
            family="TOWING_PHASE_DELAY_C7";
        else
            family=(R.r47_active_base_replan_count>0)?
                "ACTIVE_REFERENCE_TANGENT_SUPPORT_CORRIDOR_C7":
                "TANGENT_SUPPORT_CORRIDOR_C7";
    }
    set("candidate_family",mxCreateString(family));
    set("primary_candidate_count",dscalar(R.primary_candidate_count));
    set("primary_speed_pass_count",dscalar(R.primary_speed_pass));
    set("primary_rejoin_pass_count",dscalar(R.primary_rejoin_pass));
    set("primary_geometry_test_count",dscalar(R.primary_geom_tests));
    set("fallback_candidate_count",dscalar(R.fallback_candidate_count));
    set("fallback_speed_pass_count",dscalar(R.fallback_speed_pass));
    set("fallback_rejoin_pass_count",dscalar(R.fallback_rejoin_pass));
    set("fallback_geometry_test_count",dscalar(R.fallback_geom_tests));
    set("towing_candidate_count",dscalar(R.towing_candidate_count));
    set("towing_speed_pass_count",dscalar(R.towing_speed_pass));
    set("towing_rejoin_pass_count",dscalar(R.towing_rejoin_pass));
    set("towing_geometry_test_count",dscalar(R.towing_geom_tests));
    set("finite_sensing_limit_m_s",dscalar(R.finite_sensing_limit));
    set("primary_min_stageA_uav_speed_UB_m_s",dscalar(R.primary_min_stageA_vq));
    set("primary_min_stageA_specific_force_LB_m_s2",dscalar(R.primary_min_stageA_sigma));
    set("primary_min_stageA_correction_speed_UB_m_s",dscalar(R.primary_min_stageA_U[0]));
    set("primary_min_stageA_correction_accel_UB_m_s2",dscalar(R.primary_min_stageA_U[1]));
    set("primary_min_stageA_correction_jerk_UB_m_s3",dscalar(R.primary_min_stageA_U[2]));
    set("primary_min_envelope_m",dscalar(R.primary_min_envelope));
    set("primary_stageA_certified_count",dscalar(R.primary_stageA_certified_count));
    set("primary_stageB_certified_count",dscalar(R.primary_stageB_certified_count));
    set("primary_stageB_finite_witness_count",dscalar(R.primary_stageB_finite_witness_count));
    set("primary_stageB_repr_fail_count",dscalar(R.primary_stageB_repr_fail_count));
    set("towing_diag_revision",mxCreateString("AF7_TOWING_DIAG_V1"));
    set("r28_flow_side_assignments",dscalar(R.r28_flow_side_assignments));
    set("r28_flow_qp_solutions",dscalar(R.r28_flow_qp_solutions));
    set("r28_flow_built",dscalar(R.r28_flow_built));
    set("r28_flow_row_reject",dscalar(R.r28_flow_row_reject));
    set("r28_flow_grid_reject",dscalar(R.r28_flow_grid_reject));
    set("r28_flow_support_reject",dscalar(R.r28_flow_support_reject));
    set("r28_flow_step_s",dscalar(R.r28_flow_step_s));
    set("r28_flow_entry_end_s",dscalar(R.r28_flow_entry_end_s));
    set("r28_flow_exit_start_s",dscalar(R.r28_flow_exit_start_s));
    set("r28_flow_exit_end_s",dscalar(R.r28_flow_exit_end_s));
    set("r30_flow_support_start_s",dscalar(R.r30_flow_support_start_s));
    set("r30_flow_wait_s",dscalar(R.r30_flow_wait_s));
    set("r30_first_unsafe_s",dscalar(R.r30_first_unsafe_s));
    set("r30_last_unsafe_s",dscalar(R.r30_last_unsafe_s));
    set("r30_reference_end_s",dscalar(R.r30_reference_end_s));
    set("r30_lead_s",dscalar(R.r30_lead_s));
    set("r30_tail_s",dscalar(R.r30_tail_s));
    set("r31_entry_duration_s",dscalar(R.r31_entry_duration_s));
    set("r31_flow_segment_duration_s",dscalar(R.r31_flow_segment_duration_s));
    set("r31_exit_duration_s",dscalar(R.r31_exit_duration_s));
    set("r31_best_correction_speed_ub",dscalar(R.r31_best_correction_speed_ub));
    set("r31_best_stageA_uav_speed_ub",dscalar(R.r31_best_stageA_uav_speed_ub));
    set("r31_exact_join_reject",dscalar(R.r31_exact_join_reject));
    set("r32_affine_base_detected",dscalar(R.r32_affine_base_detected));
    set("r32_timing_trial_count",dscalar(R.r32_timing_trial_count));
    set("r32_exact_candidate_count",dscalar(R.r32_exact_candidate_count));
    set("r32_monotone_reject_count",dscalar(R.r32_monotone_reject_count));
    set("r32_nominal_progress_speed",dscalar(R.r32_nominal_progress_speed));
    set("r32_best_pre_speed_ub",dscalar(R.r32_best_pre_speed_ub));
    set("r32_best_progress_scale",dscalar(R.r32_best_progress_scale));
    set("r32_best_entry_T",dscalar(R.r32_best_entry_T));
    set("r32_best_flow_T",dscalar(R.r32_best_flow_T));
    set("r32_best_exit_T",dscalar(R.r32_best_exit_T));
    set("r33_exact_timing_candidate_count",dscalar(R.r33_exact_timing_candidate_count));
    set("r33_pareto_candidate_count",dscalar(R.r33_pareto_candidate_count));
    set("r33_quantized_progress_reject_count",dscalar(R.r33_quantized_progress_reject_count));
    set("r33_time_quantum_s",dscalar(R.r33_time_quantum_s));
    set("r33_support_start_s",dscalar(R.r33_support_start_s));
    set("r33_support_end_s",dscalar(R.r33_support_end_s));
    set("r33_best_accel_ub",dscalar(R.r33_best_accel_ub));
    set("r33_best_jerk_ub",dscalar(R.r33_best_jerk_ub));
    set("delay_preserving",logicalScalar(R.cand.terminal_hold||R.cand.phase_reparameterized));
    set("terminal_hold_start",dscalar(R.cand.terminal_hold_start));
    {
        mxArray* v=mxCreateDoubleMatrix(3,1,mxREAL);
        double* p=mxGetPr(v);
        p[0]=R.cand.terminal_offset.x;
        p[1]=R.cand.terminal_offset.y;
        p[2]=R.cand.terminal_offset.z;
        set("terminal_offset",v);
    }
    set("schedule_delay_s",dscalar(R.cand.schedule_delay));
    set("r34_delay_candidate_count",dscalar(R.r34_delay_candidate_count));
    set("r34_terminal_hold_exact_count",dscalar(R.r34_terminal_hold_exact_count));
    set("r34_monotone_progress_count",dscalar(R.r34_monotone_progress_count));
    set("r34_selected_delay_s",dscalar(R.r34_selected_delay_s));
    set("r34_selected_hold_start_s",dscalar(R.r34_selected_hold_start_s));
    set("r34_selected_alpha",dscalar(R.r34_selected_alpha));
    set("r34_selected_velocity_ub",dscalar(R.r34_selected_velocity_ub));
    set("r34_selected_accel_ub",dscalar(R.r34_selected_accel_ub));
    set("r34_selected_jerk_ub",dscalar(R.r34_selected_jerk_ub));
    set("path_tangent_yaw_c6",logicalScalar(R.cand.heading_c6));
    set("heading_gamma_m_s",dscalar(R.cand.heading_gamma));
    set("heading_anchor_rad",dscalar(R.cand.heading_anchor_rad));
    {
        mxArray* v=mxCreateDoubleMatrix(3,1,mxREAL);
        double* p=mxGetPr(v);
        p[0]=R.cand.heading_base_velocity.x;
        p[1]=R.cand.heading_base_velocity.y;
        p[2]=R.cand.heading_base_velocity.z;
        set("heading_base_velocity",v);
    }
    set("r35_heading_candidate_count",dscalar(R.r35_heading_candidate_count));
    set("r35_heading_pass_count",dscalar(R.r35_heading_pass_count));
    set("r35_heading_reject_count",dscalar(R.r35_heading_reject_count));
    set("r35_best_heading_gamma_m_s",dscalar(R.r35_best_heading_gamma_m_s));
    set("r35_selected_heading_gamma_m_s",dscalar(R.r35_selected_heading_gamma_m_s));
    set("r35_selected_heading_anchor_rad",dscalar(R.r35_selected_heading_anchor_rad));
    set("r36_spatial_qp_solutions",dscalar(R.r36_spatial_qp_solutions));
    set("r36_spatial_row_rejects",dscalar(R.r36_spatial_row_rejects));
    set("r36_timing_trials",dscalar(R.r36_timing_trials));
    set("r36_exact_curved_candidates",dscalar(R.r36_exact_curved_candidates));
    set("r36_curved_heading_pass",dscalar(R.r36_curved_heading_pass));
    set("r36_mid_progress_s",dscalar(R.r36_mid_progress_s));
    set("r36_best_spatial_path_speed_ub",dscalar(R.r36_best_spatial_path_speed_ub));
    set("r36_best_total_velocity_ub",dscalar(R.r36_best_total_velocity_ub));
    set("r36_best_accel_ub",dscalar(R.r36_best_accel_ub));
    set("r36_best_jerk_ub",dscalar(R.r36_best_jerk_ub));
    set("trigger_to_free_rejoin",logicalScalar(R.cand.trigger_to_free_rejoin));
    set("identity_time_law",logicalScalar(R.cand.identity_time_law));
    set("actual_rejoin_time",dscalar(R.cand.actual_rejoin_time));
    set("spatial_knot_count",dscalar(R.cand.spatial_knot_count));
    set("r37_side_assignments",dscalar(R.r37_side_assignments));
    set("r37_spatial_qp_solutions",dscalar(R.r37_spatial_qp_solutions));
    set("r37_exact_c7_candidates",dscalar(R.r37_exact_c7_candidates));
    set("r37_heading_pass_count",dscalar(R.r37_heading_pass_count));
    set("r37_knot_count",dscalar(R.r37_knot_count));
    set("r37_support_start_s",dscalar(R.r37_support_start_s));
    set("r37_actual_rejoin_s",dscalar(R.r37_actual_rejoin_s));
    set("r37_best_total_velocity_ub",dscalar(R.r37_best_total_velocity_ub));
    set("r37_best_accel_ub",dscalar(R.r37_best_accel_ub));
    set("r37_best_jerk_ub",dscalar(R.r37_best_jerk_ub));
    set("constructive_exact_c7",logicalScalar(R.cand.exact_c7_constructive));
    set("legacy_float_endpoint_checker_pass",logicalScalar(R.cand.legacy_float_endpoint_checker_pass));
    set("r38_constructive_c7_count",dscalar(R.r38_constructive_c7_count));
    set("r38_legacy_float_checker_pass_count",dscalar(R.r38_legacy_float_checker_pass_count));
    set("r38_legacy_float_false_reject_count",dscalar(R.r38_legacy_float_false_reject_count));
    set("r45_tangent_interval_count",dscalar(R.r45_tangent_interval_count));
    set("r45_zero_offset_interval_count",dscalar(R.r45_zero_offset_interval_count));
    set("r45_convex_search_fallback_count",dscalar(R.r45_convex_search_fallback_count));
    set("r45_best_fixed_peak_req_m",dscalar(R.r45_best_fixed_peak_req_m));
    set("r45_best_tangent_peak_req_m",dscalar(R.r45_best_tangent_peak_req_m));
    set("r45_max_peak_relief_m",dscalar(R.r45_max_peak_relief_m));
    set("r47_active_base_replan_count",dscalar(R.r47_active_base_replan_count));
    set("r47_nonaffine_base_count",dscalar(R.r47_nonaffine_base_count));
    set("r47_heading_leaf_count",dscalar(R.r47_heading_leaf_count));
    set("r47_heading_split_count",dscalar(R.r47_heading_split_count));
    set("r48_natural_rejoin_trial_count",dscalar(R.r48_natural_rejoin_trial_count));
    set("r48_natural_rejoin_selected_count",dscalar(R.r48_natural_rejoin_selected_count));
    set("r48_exit_monotone_row_count",dscalar(R.r48_exit_monotone_row_count));
    set("r48_natural_rejoin_seed_t1",dscalar(R.r48_natural_rejoin_seed_t1));
    set("r49_phase_timing_trial_count",dscalar(R.r49_phase_timing_trial_count));
    set("r49_phase_gate_pass_count",dscalar(R.r49_phase_gate_pass_count));
    set("r49_hard_certificate_trial_count",dscalar(R.r49_hard_certificate_trial_count));
    set("r49_selected_Ke",dscalar((double)R.r49_selected_Ke));
    set("r49_selected_Kflow",dscalar((double)R.r49_selected_Kflow));
    set("r49_selected_Kx",dscalar((double)R.r49_selected_Kx));
    set("r50_spatial_side_solution_count",dscalar(R.r50_spatial_side_solution_count));
    set("r50_spatial_lateral_constraint_count",dscalar(R.r50_spatial_lateral_constraint_count));
    set("r50_entry_candidate_count",dscalar(R.r50_entry_candidate_count));
    set("r50_exit_candidate_count",dscalar(R.r50_exit_candidate_count));
    set("r50_best_spatial_velocity_norm",dscalar(R.r50_best_spatial_velocity_norm));
    set("r51_phase_seed_count",dscalar(R.r51_phase_seed_count));
    set("r51_phase_seed_reject_count",dscalar(R.r51_phase_seed_reject_count));
    set("r51_tangent_candidate_count",dscalar(R.r51_tangent_candidate_count));
    set("r51_tangent_certified_count",dscalar(R.r51_tangent_certified_count));
    set("r51_phase_alpha",dscalar(R.r51_phase_alpha));
    set("r51_phase_delay_s",dscalar(R.r51_phase_delay_s));
    set("r51_phase_exit_time_s",dscalar(R.r51_phase_exit_time_s));
    set("r52_phase_episode_count",dscalar(R.r52_phase_episode_count));
    set("r52_phase_episode_rebuild_fail_count",dscalar(R.r52_phase_episode_rebuild_fail_count));
    set("r52_corridor_row_count",dscalar(R.r52_corridor_row_count));
    set("r52_support_plane_fail_count",dscalar(R.r52_support_plane_fail_count));
    set("r52_spatial_qp_solution_count",dscalar(R.r52_spatial_qp_solution_count));
    set("r52_rejoin_time_trial_count",dscalar(R.r52_rejoin_time_trial_count));
    set("r52_selected_spatial_rejoin_s",dscalar(R.r52_selected_spatial_rejoin_s));
    set("r53_normal_bundle_node_count",dscalar(R.r53_normal_bundle_node_count));
    set("r53_normal_bundle_row_count",dscalar(R.r53_normal_bundle_row_count));
    set("r53_normal_bundle_singular_count",dscalar(R.r53_normal_bundle_singular_count));
    set("r53_affine_reduction_count",dscalar(R.r53_affine_reduction_count));
    set("r54_phase_candidate_count",dscalar(R.r54_phase_candidate_count));
    set("r54_phase_trial_count",dscalar(R.r54_phase_trial_count));
    set("r54_phase_base_certified_count",dscalar(R.r54_phase_base_certified_count));
    set("r54_phase_episode_pass_count",dscalar(R.r54_phase_episode_pass_count));
    set("r54_phase_episode_empty_count",dscalar(R.r54_phase_episode_empty_count));
    set("r54_selected_tau_multiple",dscalar((double)R.r54_selected_tau_multiple));
    set("r54_selected_alpha",dscalar(R.r54_selected_alpha));
    set("r54_selected_delay_s",dscalar(R.r54_selected_delay_s));
    set("r54_selected_first_unsafe_lead_s",dscalar(R.r54_selected_first_unsafe_lead_s));
    set("r54_max_first_unsafe_lead_s",dscalar(R.r54_max_first_unsafe_lead_s));
    set("r54_maneuver_horizon_coupling_count",dscalar(R.r54_maneuver_horizon_coupling_count));
    set("r55_phase_build_count",dscalar(R.r55_phase_build_count));
    set("r55_phase_build_fail_count",dscalar(R.r55_phase_build_fail_count));
    set("r55_phase_episode_pass_count",dscalar(R.r55_phase_episode_pass_count));
    set("r55_phase_episode_empty_count",dscalar(R.r55_phase_episode_empty_count));
    set("r55_mixed_side_episode_count",dscalar(R.r55_mixed_side_episode_count));
    set("r55_side_plus_count",dscalar(R.r55_side_plus_count));
    set("r55_side_minus_count",dscalar(R.r55_side_minus_count));
    set("r55_side_onebranch_count",dscalar(R.r55_side_onebranch_count));
    set("r55_side_both_failed_count",dscalar(R.r55_side_both_failed_count));
    set("r55_corridor_row_count",dscalar(R.r55_corridor_row_count));
    set("r55_obstacle_count",dscalar(R.r55_obstacle_count));
    set("r55_old_clock_binding_count",dscalar(R.r55_old_clock_binding_count));
    set("r55_maneuver_horizon_coupling_count",dscalar(R.r55_maneuver_horizon_coupling_count));
    set("r55_alpha",dscalar(R.r55_alpha));
    set("r55_delay_s",dscalar(R.r55_delay_s));
    set("r55_source_unsafe_start_s",dscalar(R.r55_source_unsafe_start_s));
    set("r55_source_unsafe_end_s",dscalar(R.r55_source_unsafe_end_s));
    set("r55_predicted_first_unsafe_t_s",dscalar(R.r55_predicted_first_unsafe_t_s));
    set("r55_predicted_last_unsafe_t_s",dscalar(R.r55_predicted_last_unsafe_t_s));
    set("r55_pre_unsafe_slack_s",dscalar(R.r55_pre_unsafe_slack_s));
    set("r55_post_unsafe_slack_s",dscalar(R.r55_post_unsafe_slack_s));
    set("r55_min_temporal_slack_s",dscalar(R.r55_min_temporal_slack_s));
    set("r55_phase_entry_transition_s",dscalar(R.r55_phase_entry_transition_s));
    set("r55_phase_exit_transition_s",dscalar(R.r55_phase_exit_transition_s));
    set("r55_source_phase_at_rejoin_s",dscalar(R.r55_source_phase_at_rejoin_s));
    set("r55_rejoin_clock_delay_s",dscalar(R.r55_rejoin_clock_delay_s));
    set("r55_min_ellipsoid_axis_m",dscalar(R.r55_min_ellipsoid_axis_m));
    set("r55_max_ellipsoid_axis_m",dscalar(R.r55_max_ellipsoid_axis_m));
    set("r55_max_rotation_orth_error",dscalar(R.r55_max_rotation_orth_error));
    set("r56_shadow_ran",dscalar(R.r56_shadow_ran));
    set("r56_source_qp_ok",dscalar(R.r56_source_qp_ok));
    set("r56_source_exact_c7",dscalar(R.r56_source_exact_c7));
    set("r56_source_knot_count",dscalar(R.r56_source_knot_count));
    set("r56_source_episode_count",dscalar(R.r56_source_episode_count));
    set("r56_source_corridor_row_count",dscalar(R.r56_source_corridor_row_count));
    set("r56_source_normal_row_count",dscalar(R.r56_source_normal_row_count));
    set("r56_source_side_plus_count",dscalar(R.r56_source_side_plus_count));
    set("r56_source_side_minus_count",dscalar(R.r56_source_side_minus_count));
    set("r56_source_side_fail_count",dscalar(R.r56_source_side_fail_count));
    set("r56_const_sample_count",dscalar(R.r56_const_sample_count));
    set("r56_const_dynamic_pass",dscalar(R.r56_const_dynamic_pass));
    set("r56_const_geometry_point_pass",dscalar(R.r56_const_geometry_point_pass));
    set("r56_entry_bestcase_pass",dscalar(R.r56_entry_bestcase_pass));
    set("r56_exit_bestcase_pass",dscalar(R.r56_exit_bestcase_pass));
    set("r56_yaw_source_regular",dscalar(R.r56_yaw_source_regular));
    set("r56_old_clock_binding_count",dscalar(R.r56_old_clock_binding_count));
    set("r56_horizon_duration_coupling_count",dscalar(R.r56_horizon_duration_coupling_count));
    set("r56_source_support_start_s",dscalar(R.r56_source_support_start_s));
    set("r56_source_rejoin_s",dscalar(R.r56_source_rejoin_s));
    set("r56_source_corr_v_ub",dscalar(R.r56_source_corr_v_ub));
    set("r56_source_corr_a_ub",dscalar(R.r56_source_corr_a_ub));
    set("r56_source_corr_j_ub",dscalar(R.r56_source_corr_j_ub));
    set("r56_source_min_hxy",dscalar(R.r56_source_min_hxy));
    set("r56_alpha_max_pointwise",dscalar(R.r56_alpha_max_pointwise));
    set("r56_const_max_vl",dscalar(R.r56_const_max_vl));
    set("r56_const_max_vq",dscalar(R.r56_const_max_vq));
    set("r56_const_min_sigma",dscalar(R.r56_const_min_sigma));
    set("r56_const_max_qdot",dscalar(R.r56_const_max_qdot));
    set("r56_const_min_geom_margin",dscalar(R.r56_const_min_geom_margin));
    set("r56_entry_Tmax_s",dscalar(R.r56_entry_Tmax_s));
    set("r56_entry_max_vq",dscalar(R.r56_entry_max_vq));
    set("r56_entry_min_sigma",dscalar(R.r56_entry_min_sigma));
    set("r56_entry_max_qdot",dscalar(R.r56_entry_max_qdot));
    set("r56_exit_Tmax_s",dscalar(R.r56_exit_Tmax_s));
    set("r56_exit_max_vq",dscalar(R.r56_exit_max_vq));
    set("r56_exit_min_sigma",dscalar(R.r56_exit_min_sigma));
    set("r56_exit_max_qdot",dscalar(R.r56_exit_max_qdot));
    set("r56_chain_rule_max_error",dscalar(R.r56_chain_rule_max_error));
    set("r56_shadow_ms",dscalar(R.r56_shadow_ms));
    set("r57_probe_ran",dscalar(R.r57_probe_ran));
    set("r57_source_qp_ok",dscalar(R.r57_source_qp_ok));
    set("r57_source_exact_c7",dscalar(R.r57_source_exact_c7));
    set("r57_speed_attempt",dscalar(R.r57_speed_attempt));
    set("r57_speed_pass",dscalar(R.r57_speed_pass));
    set("r57_speed_stageA_pass",dscalar(R.r57_speed_stageA_pass));
    set("r57_speed_stageB_attempt",dscalar(R.r57_speed_stageB_attempt));
    set("r57_speed_stageB_finite_witness",dscalar(R.r57_speed_stageB_finite_witness));
    set("r57_speed_stageB_repr_fail",dscalar(R.r57_speed_stageB_repr_fail));
    set("r57_heading_attempt",dscalar(R.r57_heading_attempt));
    set("r57_heading_pass",dscalar(R.r57_heading_pass));
    set("r57_heading_leaf_count",dscalar(R.r57_heading_leaf_count));
    set("r57_heading_split_count",dscalar(R.r57_heading_split_count));
    set("r57_geometry_attempt",dscalar(R.r57_geometry_attempt));
    set("r57_geometry_pass",dscalar(R.r57_geometry_pass));
    set("r57_commit_attempted",dscalar(R.r57_commit_attempted));
    set("r57_rejoin_selected",dscalar(R.r57_rejoin_selected));
    set("r57_old_clock_commit_binding_count",dscalar(R.r57_old_clock_commit_binding_count));
    set("r57_horizon_duration_coupling_count",dscalar(R.r57_horizon_duration_coupling_count));
    set("r57_speed_VL",dscalar(R.r57_speed_VL));
    set("r57_speed_A",dscalar(R.r57_speed_A));
    set("r57_speed_J",dscalar(R.r57_speed_J));
    set("r57_speed_sigma",dscalar(R.r57_speed_sigma));
    set("r57_speed_qdot",dscalar(R.r57_speed_qdot));
    set("r57_speed_VQ",dscalar(R.r57_speed_VQ));
    set("r57_heading_gamma",dscalar(R.r57_heading_gamma));
    set("r57_geometry_min_margin",dscalar(R.r57_geometry_min_margin));
    set("r57_source_support_start_s",dscalar(R.r57_source_support_start_s));
    set("r57_source_support_end_s",dscalar(R.r57_source_support_end_s));
    set("r57_source_corr_v_ub",dscalar(R.r57_source_corr_v_ub));
    set("r57_source_corr_a_ub",dscalar(R.r57_source_corr_a_ub));
    set("r57_source_corr_j_ub",dscalar(R.r57_source_corr_j_ub));
    set("r57_source_build_ms",dscalar(R.r57_source_build_ms));
    set("r57_speed_ms",dscalar(R.r57_speed_ms));
    set("r57_heading_ms",dscalar(R.r57_heading_ms));
    set("r57_geometry_ms",dscalar(R.r57_geometry_ms));
    set("r57_probe_ms",dscalar(R.r57_probe_ms));
    // R13 rows are diagnostic-only and cannot grant reference acceptance.
    {
        const mwSize nr=(mwSize)R.r58_speed_gate_rows.size();
        mxArray* g=mxCreateDoubleMatrix(nr,38,mxREAL);
        double* p=mxGetPr(g);
        for(mwSize i=0;i<nr;++i)
            for(mwSize j=0;j<38;++j)
                p[i+nr*j]=R.r58_speed_gate_rows[i][j];
        set("r58_speed_gate_rows",g);
    }
    set("r58r16_connected_candidate",dscalar(R.r58r16_connected_candidate));
    set("r58_shadow_ran",dscalar(R.r58_shadow_ran));
    set("r58_reference_only",dscalar(R.r58_reference_only));
    set("r58_fixed_rejoin_constraint_count",dscalar(R.r58_fixed_rejoin_constraint_count));
    set("r58_same_clock_binding_count",dscalar(R.r58_same_clock_binding_count));
    set("r58_horizon_duration_coupling_count",dscalar(R.r58_horizon_duration_coupling_count));
    set("r58_free_c7_constructive",dscalar(R.r58_free_c7_constructive));
    set("r58_terminal_offset_hold",dscalar(R.r58_terminal_offset_hold));
    set("r58_terminal_zero_derivative_order",dscalar(R.r58_terminal_zero_derivative_order));
    set("r58_source_knot_count",dscalar(R.r58_source_knot_count));
    set("r58_scalar_dof_count",dscalar(R.r58_scalar_dof_count));
    set("r58r14_lowjet_seed_only",dscalar(R.r58r14_lowjet_seed_only));
    set("r58r14_active_reduced_dof",dscalar(R.r58r14_active_reduced_dof));
    set("r58r14_full_reduced_dof",dscalar(R.r58r14_full_reduced_dof));
    set("r58_spatial_dof_count",dscalar(R.r58_spatial_dof_count));
    set("r58_episode_count",dscalar(R.r58_episode_count));
    set("r58_component_count",dscalar(R.r58_component_count));
    set("r58_side_cache_plus_valid",dscalar(R.r58_side_cache_plus_valid));
    set("r58_side_cache_minus_valid",dscalar(R.r58_side_cache_minus_valid));
    set("r58_side_cache_fail",dscalar(R.r58_side_cache_fail));
    set("r58_bnb_nodes_popped",dscalar(R.r58_bnb_nodes_popped));
    set("r58_bnb_qp_solves",dscalar(R.r58_bnb_qp_solves));
    set("r58_bnb_qp_infeasible",dscalar(R.r58_bnb_qp_infeasible));
    set("r58_bnb_full_assignments",dscalar(R.r58_bnb_full_assignments));
    set("r58_bnb_speed_fail",dscalar(R.r58_bnb_speed_fail));
    set("r58_bnb_heading_fail",dscalar(R.r58_bnb_heading_fail));
    set("r58_bnb_geometry_fail",dscalar(R.r58_bnb_geometry_fail));
    set("r58_bnb_feasible_full",dscalar(R.r58_bnb_feasible_full));
    set("r58_bnb_max_queue",dscalar(R.r58_bnb_max_queue));
    set("r58_selected_side_plus",dscalar(R.r58_selected_side_plus));
    set("r58_selected_side_minus",dscalar(R.r58_selected_side_minus));
    set("r58_speed_pass",dscalar(R.r58_speed_pass));
    set("r58_heading_pass",dscalar(R.r58_heading_pass));
    set("r58_geometry_pass",dscalar(R.r58_geometry_pass));
    set("r58_needs_time_law",dscalar(R.r58_needs_time_law));
    set("r58_commit_attempted",dscalar(R.r58_commit_attempted));
    set("r58_rejoin_selected",dscalar(R.r58_rejoin_selected));
    set("r58_reference_tracking_inflation_m",dscalar(R.r58_reference_tracking_inflation_m));
    set("r58_support_start_s",dscalar(R.r58_support_start_s));
    set("r58_support_end_s",dscalar(R.r58_support_end_s));
    set("r58_terminal_offset_norm_m",dscalar(R.r58_terminal_offset_norm_m));
    set("r58_quantized_jet_max_abs_error",dscalar(R.r58_quantized_jet_max_abs_error));
    set("r58_legacy_join_max_abs_mismatch",dscalar(R.r58_legacy_join_max_abs_mismatch));
    set("r58_best_qp_cost",dscalar(R.r58_best_qp_cost));
    set("r58_speed_VL",dscalar(R.r58_speed_VL));
    set("r58_speed_A",dscalar(R.r58_speed_A));
    set("r58_speed_J",dscalar(R.r58_speed_J));
    set("r58_speed_sigma",dscalar(R.r58_speed_sigma));
    set("r58_speed_qdot",dscalar(R.r58_speed_qdot));
    set("r58_speed_VQ",dscalar(R.r58_speed_VQ));
    set("r58_heading_gamma",dscalar(R.r58_heading_gamma));
    set("r58_geometry_min_margin",dscalar(R.r58_geometry_min_margin));
    set("r58_build_ms",dscalar(R.r58_build_ms));
    set("r58_homotopy_ms",dscalar(R.r58_homotopy_ms));
    set("r58_certificate_ms",dscalar(R.r58_certificate_ms));
    set("r58_shadow_ms",dscalar(R.r58_shadow_ms));
    set("r58r2_node_scaled_variables",dscalar(R.r58r2_node_scaled_variables));
    set("r58r2_row_equilibrated",dscalar(R.r58r2_row_equilibrated));
    set("r58r2_root_qp_elided",dscalar(R.r58r2_root_qp_elided));
    set("r58r2_node_scale_min_s",dscalar(R.r58r2_node_scale_min_s));
    set("r58r2_node_scale_max_s",dscalar(R.r58r2_node_scale_max_s));
    set("r58r2_row_scale_min",dscalar(R.r58r2_row_scale_min));
    set("r58r2_row_scale_max",dscalar(R.r58r2_row_scale_max));
    set("r58r3_hessian_diag_equilibrated",dscalar(R.r58r3_hessian_diag_equilibrated));
    set("r58r3_equilibrated_cholesky_pass",dscalar(R.r58r3_equilibrated_cholesky_pass));
    set("r58r3_raw_hdiag_min",dscalar(R.r58r3_raw_hdiag_min));
    set("r58r3_raw_hdiag_max",dscalar(R.r58r3_raw_hdiag_max));
    set("r58r3_raw_hdiag_ratio",dscalar(R.r58r3_raw_hdiag_ratio));
    set("r58r3_eq_hdiag_min",dscalar(R.r58r3_eq_hdiag_min));
    set("r58r3_eq_hdiag_max",dscalar(R.r58r3_eq_hdiag_max));
    set("r58r4_normal_equalities_eliminated",dscalar(R.r58r4_normal_equalities_eliminated));
    set("r58r4_reduced_spatial_dof_count",dscalar(R.r58r4_reduced_spatial_dof_count));
    set("r58r4_elimination_build_fail",dscalar(R.r58r4_elimination_build_fail));
    set("r58r4_elimination_max_residual",dscalar(R.r58r4_elimination_max_residual));
    set("r58r5_dykstra_used",dscalar(R.r58r5_dykstra_used));
    set("r58r5_dykstra_converged_feasible",dscalar(R.r58r5_dykstra_converged_feasible));
    set("r58r5_dykstra_resource_fail",dscalar(R.r58r5_dykstra_resource_fail));
    set("r58r5_dykstra_degenerate_row_fail",dscalar(R.r58r5_dykstra_degenerate_row_fail));
    set("r58r5_dykstra_max_sweeps_used",dscalar(R.r58r5_dykstra_max_sweeps_used));
    set("r58r5_dykstra_projection_count",dscalar(R.r58r5_dykstra_projection_count));
    set("r58r5_dykstra_optimal_kkt_count",dscalar(R.r58r5_dykstra_optimal_kkt_count));
    set("r58r5_dykstra_max_violation",dscalar(R.r58r5_dykstra_max_violation));
    set("r58r5_dykstra_duality_gap",dscalar(R.r58r5_dykstra_duality_gap));
    set("r58r5_dykstra_stationarity_residual",dscalar(R.r58r5_dykstra_stationarity_residual));
    set("r58r5_dykstra_ms",dscalar(R.r58r5_dykstra_ms));
    set("r58r6_hildreth_used",dscalar(R.r58r6_hildreth_used));
    set("r58r6_kkt_converged_count",dscalar(R.r58r6_kkt_converged_count));
    set("r58r6_resource_fail_count",dscalar(R.r58r6_resource_fail_count));
    set("r58r6_max_sweeps_used",dscalar(R.r58r6_max_sweeps_used));
    set("r58r6_coordinate_updates",dscalar(R.r58r6_coordinate_updates));
    set("r58r6_max_primal_violation",dscalar(R.r58r6_max_primal_violation));
    set("r58r6_max_complementarity",dscalar(R.r58r6_max_complementarity));
    set("r58r6_duality_gap",dscalar(R.r58r6_duality_gap));
    set("r58r6_stationarity_residual",dscalar(R.r58r6_stationarity_residual));
    set("r58r6_max_lambda",dscalar(R.r58r6_max_lambda));
    set("r58r6_solver_ms",dscalar(R.r58r6_solver_ms));
    set("r58r6_prequant_scaled_jet_max_abs",dscalar(R.r58r6_prequant_scaled_jet_max_abs));
    set("r58r6_quantized_scaled_jet_rel_error",dscalar(R.r58r6_quantized_scaled_jet_rel_error));
    set("r58r6_quantized_physical_jet_max_abs_error",dscalar(R.r58r6_quantized_physical_jet_max_abs_error));
    set("r58r6_failed_corr_v_ub",dscalar(R.r58r6_failed_corr_v_ub));
    set("r58r6_failed_corr_a_ub",dscalar(R.r58r6_failed_corr_a_ub));
    set("r58r6_failed_corr_j_ub",dscalar(R.r58r6_failed_corr_j_ub));
    set("r58r6_failed_speed_worst_vq",dscalar(R.r58r6_failed_speed_worst_vq));
    set("r58r6_failed_speed_worst_vl",dscalar(R.r58r6_failed_speed_worst_vl));
    set("r58r6_failed_speed_worst_sigma",dscalar(R.r58r6_failed_speed_worst_sigma));
    set("r58r6_failed_speed_worst_qdot",dscalar(R.r58r6_failed_speed_worst_qdot));
    set("r58r6_failed_speed_witness",dscalar(R.r58r6_failed_speed_witness));
    set("r58r7_constraint_generation_used",dscalar(R.r58r7_constraint_generation_used));
    set("r58r7_full_row_count_max",dscalar(R.r58r7_full_row_count_max));
    set("r58r7_outer_iterations_max",dscalar(R.r58r7_outer_iterations_max));
    set("r58r7_full_scans",dscalar(R.r58r7_full_scans));
    set("r58r7_rows_added",dscalar(R.r58r7_rows_added));
    set("r58r7_active_rows_max",dscalar(R.r58r7_active_rows_max));
    set("r58r7_subset_kkt_solves",dscalar(R.r58r7_subset_kkt_solves));
    set("r58r7_subset_kkt_converged",dscalar(R.r58r7_subset_kkt_converged));
    set("r58r7_subset_resource_fail",dscalar(R.r58r7_subset_resource_fail));
    set("r58r7_duplicate_add_fail",dscalar(R.r58r7_duplicate_add_fail));
    set("r58r7_full_optimality_closed",dscalar(R.r58r7_full_optimality_closed));
    set("r58r7_subset_sweeps_max",dscalar(R.r58r7_subset_sweeps_max));
    set("r58r7_coordinate_updates",dscalar(R.r58r7_coordinate_updates));
    set("r58r7_initial_worst_violation",dscalar(R.r58r7_initial_worst_violation));
    set("r58r7_final_max_violation",dscalar(R.r58r7_final_max_violation));
    set("r58r7_final_duality_gap",dscalar(R.r58r7_final_duality_gap));
    set("r58r7_final_complementarity",dscalar(R.r58r7_final_complementarity));
    set("r58r7_final_stationarity",dscalar(R.r58r7_final_stationarity));
    set("r58r7_solver_ms",dscalar(R.r58r7_solver_ms));
    set("r58r8_ipm_used",dscalar(R.r58r8_ipm_used));
    set("r58r8_ipm_converged",dscalar(R.r58r8_ipm_converged));
    set("r58r8_ipm_resource_fail",dscalar(R.r58r8_ipm_resource_fail));
    set("r58r8_ipm_factor_fail",dscalar(R.r58r8_ipm_factor_fail));
    set("r58r8_ipm_max_iterations",dscalar(R.r58r8_ipm_max_iterations));
    set("r58r8_ipm_newton_solves",dscalar(R.r58r8_ipm_newton_solves));
    set("r58r8_ipm_primal_residual",dscalar(R.r58r8_ipm_primal_residual));
    set("r58r8_ipm_dual_residual",dscalar(R.r58r8_ipm_dual_residual));
    set("r58r8_ipm_complementarity",dscalar(R.r58r8_ipm_complementarity));
    set("r58r8_ipm_duality_gap",dscalar(R.r58r8_ipm_duality_gap));
    set("r58r8_ipm_min_slack",dscalar(R.r58r8_ipm_min_slack));
    set("r58r8_ipm_min_lambda",dscalar(R.r58r8_ipm_min_lambda));
    set("r58r8_ipm_ms",dscalar(R.r58r8_ipm_ms));
    set("r58r8_newton_diag_equilibrated",dscalar(R.r58r8_newton_diag_equilibrated));
    set("r58r8_pcg_used",dscalar(R.r58r8_pcg_used));
    set("r58r8_pcg_resource_fail",dscalar(R.r58r8_pcg_resource_fail));
    set("r58r8_pcg_max_iterations",dscalar(R.r58r8_pcg_max_iterations));
    set("r58r8_pcg_final_rel_residual",dscalar(R.r58r8_pcg_final_rel_residual));
    set("r58r9_failure_count",dscalar(R.r58r9_failure_count));
    set("r58r9_failure_depth",dscalar(R.r58r9_failure_depth));
    set("r58r9_failure_last_side",dscalar(R.r58r9_failure_last_side));
    set("r58r9_failure_outer_iteration",dscalar(R.r58r9_failure_outer_iteration));
    set("r58r9_failure_active_rows",dscalar(R.r58r9_failure_active_rows));
    set("r58r9_failure_added_row",dscalar(R.r58r9_failure_added_row));
    set("r58r9_failure_ipm_iteration",dscalar(R.r58r9_failure_ipm_iteration));
    set("r58r9_failure_stage",dscalar(R.r58r9_failure_stage));
    set("r58r9_failure_pcg_iterations",dscalar(R.r58r9_failure_pcg_iterations));
    set("r58r9_failure_pcg_final_rel_residual",dscalar(R.r58r9_failure_pcg_final_rel_residual));
    set("r58r9_failure_pcg_rhs_inf",dscalar(R.r58r9_failure_pcg_rhs_inf));
    set("r58r9_failure_D_min",dscalar(R.r58r9_failure_D_min));
    set("r58r9_failure_D_max",dscalar(R.r58r9_failure_D_max));
    set("r58r9_failure_D_ratio",dscalar(R.r58r9_failure_D_ratio));
    set("r58r9_failure_min_slack",dscalar(R.r58r9_failure_min_slack));
    set("r58r9_failure_min_lambda",dscalar(R.r58r9_failure_min_lambda));
    set("r58r9_failure_metric_max_abs_corr",dscalar(R.r58r9_failure_metric_max_abs_corr));
    set("r58r9_failure_metric_one_minus_corr",dscalar(R.r58r9_failure_metric_one_minus_corr));
    set("r58r9_full_scan_ms",dscalar(R.r58r9_full_scan_ms));
    set("r58r9_subset_solver_ms",dscalar(R.r58r9_subset_solver_ms));
    set("r58r9_pcg_ms",dscalar(R.r58r9_pcg_ms));
    set("r58r9_plus_failed",dscalar(R.r58r9_plus_failed));
    set("r58r9_minus_failed",dscalar(R.r58r9_minus_failed));
    set("r58r9_plus_failure_stage",dscalar(R.r58r9_plus_failure_stage));
    set("r58r9_minus_failure_stage",dscalar(R.r58r9_minus_failure_stage));
    set("r58r9_plus_failure_active_rows",dscalar(R.r58r9_plus_failure_active_rows));
    set("r58r9_minus_failure_active_rows",dscalar(R.r58r9_minus_failure_active_rows));
    set("r58r9_plus_failure_pcg_relres",dscalar(R.r58r9_plus_failure_pcg_relres));
    set("r58r9_minus_failure_pcg_relres",dscalar(R.r58r9_minus_failure_pcg_relres));
    set("r58r9_plus_metric_max_abs_corr",dscalar(R.r58r9_plus_metric_max_abs_corr));
    set("r58r9_minus_metric_max_abs_corr",dscalar(R.r58r9_minus_metric_max_abs_corr));
    set("r58r9_plus_cg_ms",dscalar(R.r58r9_plus_cg_ms));
    set("r58r9_minus_cg_ms",dscalar(R.r58r9_minus_cg_ms));
    set("r58r10_plus_primal_residual",dscalar(R.r58r10_plus_primal_residual));
    set("r58r10_plus_dual_residual",dscalar(R.r58r10_plus_dual_residual));
    set("r58r10_plus_mu",dscalar(R.r58r10_plus_mu));
    set("r58r10_plus_duality_gap",dscalar(R.r58r10_plus_duality_gap));
    set("r58r10_plus_kkt_tol",dscalar(R.r58r10_plus_kkt_tol));
    set("r58r10_plus_gap_tol",dscalar(R.r58r10_plus_gap_tol));
    set("r58r10_plus_last_sigma",dscalar(R.r58r10_plus_last_sigma));
    set("r58r10_plus_last_alpha_primal",dscalar(R.r58r10_plus_last_alpha_primal));
    set("r58r10_plus_last_alpha_dual",dscalar(R.r58r10_plus_last_alpha_dual));
    set("r58r10_plus_min_alpha_primal",dscalar(R.r58r10_plus_min_alpha_primal));
    set("r58r10_plus_min_alpha_dual",dscalar(R.r58r10_plus_min_alpha_dual));
    set("r58r10_plus_mu_ratio",dscalar(R.r58r10_plus_mu_ratio));
    set("r58r10_plus_D_min",dscalar(R.r58r10_plus_D_min));
    set("r58r10_plus_D_max",dscalar(R.r58r10_plus_D_max));
    set("r58r10_plus_D_ratio",dscalar(R.r58r10_plus_D_ratio));
    set("r58r10_plus_min_slack",dscalar(R.r58r10_plus_min_slack));
    set("r58r10_plus_min_lambda",dscalar(R.r58r10_plus_min_lambda));
    set("r58r10_plus_predictor_pcg_iterations",dscalar(R.r58r10_plus_predictor_pcg_iterations));
    set("r58r10_plus_corrector_pcg_iterations",dscalar(R.r58r10_plus_corrector_pcg_iterations));
    set("r58r10_plus_predictor_pcg_relres",dscalar(R.r58r10_plus_predictor_pcg_relres));
    set("r58r10_plus_corrector_pcg_relres",dscalar(R.r58r10_plus_corrector_pcg_relres));
    set("r58r10_minus_primal_residual",dscalar(R.r58r10_minus_primal_residual));
    set("r58r10_minus_dual_residual",dscalar(R.r58r10_minus_dual_residual));
    set("r58r10_minus_mu",dscalar(R.r58r10_minus_mu));
    set("r58r10_minus_duality_gap",dscalar(R.r58r10_minus_duality_gap));
    set("r58r10_minus_kkt_tol",dscalar(R.r58r10_minus_kkt_tol));
    set("r58r10_minus_gap_tol",dscalar(R.r58r10_minus_gap_tol));
    set("r58r10_minus_last_sigma",dscalar(R.r58r10_minus_last_sigma));
    set("r58r10_minus_last_alpha_primal",dscalar(R.r58r10_minus_last_alpha_primal));
    set("r58r10_minus_last_alpha_dual",dscalar(R.r58r10_minus_last_alpha_dual));
    set("r58r10_minus_min_alpha_primal",dscalar(R.r58r10_minus_min_alpha_primal));
    set("r58r10_minus_min_alpha_dual",dscalar(R.r58r10_minus_min_alpha_dual));
    set("r58r10_minus_mu_ratio",dscalar(R.r58r10_minus_mu_ratio));
    set("r58r10_minus_D_min",dscalar(R.r58r10_minus_D_min));
    set("r58r10_minus_D_max",dscalar(R.r58r10_minus_D_max));
    set("r58r10_minus_D_ratio",dscalar(R.r58r10_minus_D_ratio));
    set("r58r10_minus_min_slack",dscalar(R.r58r10_minus_min_slack));
    set("r58r10_minus_min_lambda",dscalar(R.r58r10_minus_min_lambda));
    set("r58r10_minus_predictor_pcg_iterations",dscalar(R.r58r10_minus_predictor_pcg_iterations));
    set("r58r10_minus_corrector_pcg_iterations",dscalar(R.r58r10_minus_corrector_pcg_iterations));
    set("r58r10_minus_predictor_pcg_relres",dscalar(R.r58r10_minus_predictor_pcg_relres));
    set("r58r10_minus_corrector_pcg_relres",dscalar(R.r58r10_minus_corrector_pcg_relres));
    set("r58r11_augmented_kkt_used",dscalar(R.r58r11_augmented_kkt_used));
    set("r58r11_augmented_kkt_solve_fail",dscalar(R.r58r11_augmented_kkt_solve_fail));
    set("r58r11_common_step_used",dscalar(R.r58r11_common_step_used));
    set("r58r11_augmented_kkt_max_dim",dscalar(R.r58r11_augmented_kkt_max_dim));
    set("r58r11_newton_solves",dscalar(R.r58r11_newton_solves));
    set("r58r11_newton_max_rel_residual",dscalar(R.r58r11_newton_max_rel_residual));
    set("r58r11_common_step_min",dscalar(R.r58r11_common_step_min));
    set("r58r11_common_step_last",dscalar(R.r58r11_common_step_last));
    set("r58r11_solver_ms",dscalar(R.r58r11_solver_ms));
    set("phase_reparameterized",logicalScalar(R.cand.phase_reparameterized));
    set("phase_t0",dscalar(R.cand.phase.t0));
    set("phase_entry_end",dscalar(R.cand.phase.tA));
    set("phase_flow_end",dscalar(R.cand.phase.tB));
    set("phase_exit_end",dscalar(R.cand.phase.tE));
    set("phase_source_start",dscalar(R.cand.phase.sA));
    set("phase_source_end",dscalar(R.cand.phase.sB));
    set("phase_alpha",dscalar(R.cand.phase.alpha));
    set("phase_delay_s",dscalar(R.cand.phase.delay));
    set("rejoin_search_ms",dscalar(R.tim.rejoin_search));
    set("planner_native_revision",mxCreateString(PLANNER_REV));
    mxArray* compiled=nullptr; bool compiledExact=false; double compiledMs=0.0;
    const mxArray* base=optfld(packet,"r28_compiled_base"); const mxArray* tv=optfld(packet,"r28_target_version");
    if(R.success && !R.cand.phase_reparameterized && base && tv){
        const auto tc=std::chrono::steady_clock::now();
        const double targetVersion=scalar(tv,"r28_target_version");
        compiled=compiled_packet_struct(base,R.cand.seg0,R.cand.seg1,R.cand.cp,t0,outT1,targetVersion);
        compiledMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tc).count();
        compiledExact=true;
    }
    if(!compiled) compiled=mxCreateStructMatrix(0,0,0,nullptr);
    set("compiled_correction",compiled);set("compiled_native_exact",logicalScalar(compiledExact));
    set("compiled_native_ms",dscalar(compiledMs));set("compiled_native_revision",mxCreateString(R28_REV));
    return S;
}
// R78: correction 7-jet evaluated on exactly the native packet input.
// Returns only the CORRECTION part (3xN); MATLAB adds its genuine nominal
// seventh derivative.  For a phase-enabled group, throw rather than guess.
static mxArray* r78_correction_jet7_matrix(const ActiveReference&R,const std::vector<double>&t){
    mxArray*Y=mxCreateDoubleMatrix(3,t.size(),mxREAL);
    double*p=mxGetPr(Y);
    for(size_t k=0;k<t.size();++k){
        const auto y=r78_correction_jet7_identity(R,t[k]);
        p[3*k+0]=y.x;p[3*k+1]=y.y;p[3*k+2]=y.z;
    }
    return Y;
}

static mxArray* eval_matrix(const ActiveReference&R,const std::vector<double>&t){
    mxArray*Y=mxCreateDoubleMatrix(21,t.size(),mxREAL);double*p=mxGetPr(Y);
    for(size_t k=0;k<t.size();++k){auto y=R.eval(t[k]);for(int r=0;r<7;++r){p[(3*r+0)+21*k]=y[r].x;p[(3*r+1)+21*k]=y[r].y;p[(3*r+2)+21*k]=y[r].z;}}
    return Y;
}

void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    try{
        if(nrhs<1||!mxIsDouble(prhs[0])||mxGetNumberOfElements(prhs[0])!=1)
            mexErrMsgIdAndTxt("AF7R34:Input","first input must be op scalar.");
        int op=(int)mxGetScalar(prhs[0]);
        if(op==0){
            if(nrhs!=3) mexErrMsgIdAndTxt("AF7R34:Arity","compile op: op, exprCell, signature.");
            if(!mxIsCell(prhs[1])||mxGetNumberOfElements(prhs[1])!=21) mexErrMsgIdAndTxt("AF7R34:Input","exprCell must contain 21 cells.");
            std::vector<std::string> ex(21);
            for(int i=0;i<21;++i)ex[i]=str(mxGetCell(prhs[1],i),"expression");
            std::string sig=str(prhs[2],"signature");G_nominal.compile(ex,sig);G_compiled=true;
            if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==2){
            if(nrhs!=3) mexErrMsgIdAndTxt("AF7R34:Arity","eval op: op, packet, times.");
            ActiveReference R=parse_ref(prhs[1]);auto t=dvec(prhs[2],"times");plhs[0]=eval_matrix(R,t);return;
        }
        // R78 opt-in native correction derivative 7 for identity-time groups.
        // Existing op0/1/2 and revision contract are unchanged.
        if(op==25){
            if(nrhs!=3 || nlhs!=1)
                mexErrMsgIdAndTxt("AF7R78:Arity","op25: op, exactPacket, times -> 3xN correction jet7.");
            ActiveReference R=parse_ref(prhs[1]);
            auto t=dvec(prhs[2],"times");
            plhs[0]=r78_correction_jet7_matrix(R,t);
            return;
        }
        if(op==1){
            if(nrhs!=2) mexErrMsgIdAndTxt("AF7R34:Arity","plan op: op, packet.");
            const mxArray*P=prhs[1];
            std::string sig=str(fld(P,"nominal_signature"),"nominal_signature");
            if(!G_compiled||sig!=G_nominal.signature) mexErrMsgIdAndTxt("AF7R34:Reference","compiled nominal signature mismatch.");
            // R55 diagnostic-only: distinguish native packet parsing, exact
            // planner, and result serialization. No decision uses these clocks.
            const auto r55_t0=std::chrono::steady_clock::now();
            ActiveReference R=parse_ref(P);Live L=parse_live(P);SpeedCert B=parse_speed(P);auto O=parse_obs(P);
            double t0=scalar(fld(P,"t0"),"t0"),tm=scalar(fld(P,"tm"),"tm");
            const mxArray* se=optfld(P,"search_end");
            double searchEnd=se?scalar(se,"search_end"):scalar(fld(P,"t1"),"t1");
            const auto r55_t1=std::chrono::steady_clock::now();
            PlanResult Q=plan(R,L,B,O,t0,tm,searchEnd);
            const auto r55_t2=std::chrono::steady_clock::now();
            /* R11: R10B heading runtime hook disabled while motion authority is audited. */
            c7front_push(t0,tm,searchEnd);
            mxArray* r55_out=result_struct(Q,t0,tm,searchEnd,O,P);
            const auto r55_t3=std::chrono::steady_clock::now();
            const double r55_parse_ms=std::chrono::duration<double,std::milli>(r55_t1-r55_t0).count();
            const double r55_plan_ms=std::chrono::duration<double,std::milli>(r55_t2-r55_t1).count();
            const double r55_result_ms=std::chrono::duration<double,std::milli>(r55_t3-r55_t2).count();
            const char* r55_names[]={"r55_mex_parse_ms","r55_mex_plan_ms","r55_mex_result_ms"};
            const double r55_v[]={r55_parse_ms,r55_plan_ms,r55_result_ms};
            for(int i=0;i<3;++i){
                const int f=mxAddField(r55_out,r55_names[i]);
                if(f<0) mexErrMsgIdAndTxt("R55:Timing","Failed to add diagnostic field.");
                mxSetFieldByNumber(r55_out,0,f,dscalar(r55_v[i]));
            }
            plhs[0]=r55_out;return;
        }
        // R23 setup-only prewarm: no candidate, no obstacle data and no
        // acceptance side effects. This must run before Flight from MATLAB.
        if(op==24){
            if(nrhs!=1||nlhs!=1)
                mexErrMsgIdAndTxt("AF7R23:Arity","prewarm op24 -> logical true.");
            const auto& G=r58r23_gram_table();
            const auto& B=r58r23_binomial_table();
            if(!(fin(G[0][0])&&fin(B[7][3])))
                mexErrMsgIdAndTxt("AF7R23:Prewarm","constant coefficient initialization failed.");
            plhs[0]=logicalScalar(true);return;
        }
        if(op==6){
            if(nrhs!=1||nlhs!=1) mexErrMsgIdAndTxt("AF7TOWMAP:Arity","diag op: op -> matrix.");
            plhs[0]=towmap_matrix();return;
        }
        if(op==7){
            if(nrhs!=1||nlhs>1) mexErrMsgIdAndTxt("AF7TOWMAP:Arity","clear diag op: op.");
            G_towmap.clear();G_towmap_seq=0;towmap_core_reset_witness();
            if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==8){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7C7FRONT:Arity","diag op8: op -> matrix.");
            plhs[0]=c7front_matrix();return;
        }
        if(op==9){
            if(nrhs!=1||nlhs>1)mexErrMsgIdAndTxt("AF7C7FRONT:Arity","diag op9: clear.");
            G_c7front_mex.clear();G_c7front_seq=0;G_c7front_rows.clear();
            if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==10){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7R7SHADOW:Arity","op10 -> matrix.");
            plhs[0]=r7shadow_matrix();return;
        }
        if(op==20){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7R52D2:Arity","op20 -> candidate gate matrix.");
            plhs[0]=r52_gate_diag_matrix();return;
        }
        if(op==21){
            if(nrhs!=1||nlhs>1)mexErrMsgIdAndTxt("AF7R52D2:Arity","op21 -> reset candidate gate diagnostics.");
            r52_gate_diag_reset();
            if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==22){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7R53D3:Arity","op22 -> knot diagnostic matrix.");
            plhs[0]=r53_knot_diag_matrix();return;
        }
        if(op==23){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7R53D3:Arity","op23 -> segment diagnostic matrix.");
            plhs[0]=r53_segment_diag_matrix();return;
        }
        if(op==11){
            if(nrhs!=1||nlhs>1)mexErrMsgIdAndTxt("AF7R7SHADOW:Arity","op11 clear.");
            G_r7shadow.clear();G_r7shadow_seq=0;
            if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==12){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7PHR9:Arity","op12 -> matrix.");
            plhs[0]=phr9_matrix();return;
        }
        if(op==13){
            if(nrhs!=1||nlhs>1)mexErrMsgIdAndTxt("AF7PHR9:Arity","op13 clear.");
            G_phr9.clear();G_phr9_seq=0;if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==14){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7HEADTOPO10:Arity","op14 -> matrix.");
            plhs[0]=headtopo10_matrix();return;
        }
        if(op==15){
            if(nrhs!=1||nlhs>1)mexErrMsgIdAndTxt("AF7HEADTOPO10:Arity","op15 clear.");
            G_headtopo10.clear();G_headtopo10_seq=0;if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==16){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7HEADTOPO10B:Arity","op16 -> matrix.");
            plhs[0]=headtopo10b_matrix();return;
        }
        if(op==17){
            if(nrhs!=1||nlhs>1)mexErrMsgIdAndTxt("AF7HEADTOPO10B:Arity","op17 clear.");
            G_headtopo10b.clear();G_headtopo10b_seq=0;if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==18){
            if(nrhs!=1||nlhs!=1)mexErrMsgIdAndTxt("AF7MOTION11:Arity","op18 -> matrix.");
            plhs[0]=motion11_matrix();return;
        }
        if(op==19){
            if(nrhs!=1||nlhs>1)mexErrMsgIdAndTxt("AF7MOTION11:Arity","op19 clear.");
            G_motion11.clear();G_motion11_seq=0;if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==3){
            if(nrhs!=3||nlhs!=1) mexErrMsgIdAndTxt("AF7R28:Arity","DCP selftest op: op, cp0, duration -> dcp.");
            plhs[0]=dcp_only_from_cp(prhs[1],prhs[2]);return;
        }
        if(op==4){
            if(nrhs!=8||nlhs!=1) mexErrMsgIdAndTxt("AF7R28:Arity","append op: op, base, seg0, seg1, cp0, t0, t1, targetVersion.");
            auto s0=dvec(prhs[2],"segment_t0"),s1=dvec(prhs[3],"segment_t1");
            if(s0.size()!=s1.size()||s0.empty()) mexErrMsgIdAndTxt("AF7R28:Shape","append segment metadata mismatch.");
            auto cp=cp3_to_poly(prhs[4],s0.size());
            double nt0=scalar(prhs[5],"t0"),nt1=scalar(prhs[6],"t1"),tv=scalar(prhs[7],"targetVersion");
            plhs[0]=compiled_packet_struct(prhs[1],s0,s1,cp,nt0,nt1,tv);return;
        }
        if(op==5){
            if(nrhs!=1||nlhs>1) mexErrMsgIdAndTxt("AF7R28:Arity","revision op: op -> string.");
            if(nlhs>0)plhs[0]=mxCreateString(R28_REV);return;
        }
        mexErrMsgIdAndTxt("AF7R34:Op","unknown op.");
    }catch(const std::exception&e){
        mexErrMsgIdAndTxt("AF7R34:Exception","%s",e.what());
    }
}
