
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
    R.repairs.resize(G);
    for(size_t g=0;g<G;++g){
        long a=(long)std::llround(offs[g]),b=(long)std::llround(offs[g+1]);
        if(a<0||b<a||b>(long)S) mexErrMsgIdAndTxt("AF7R34:Packet","invalid repair offsets.");
        R.repairs[g].t0=rt0[g];R.repairs[g].t1=rt1[g];
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
static mxArray* result_struct(const PlanResult&R,double t0,double tm,double t1,const std::vector<Obstacle>&obs,const mxArray* packet){
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
        "compiled_correction","compiled_native_exact","compiled_native_ms","compiled_native_revision"};
    mxArray*S=mxCreateStructMatrix(1,1,sizeof(fn)/sizeof(fn[0]),fn);
    auto set=[&](const char*n,mxArray*v){mxSetField(S,0,n,v);};
    set("success",logicalScalar(R.success));set("reason_code",dscalar(R.reason));
    set("t0",dscalar(t0));set("tm",dscalar(tm));set("hold_end",dscalar(R.hold_end));set("t1",dscalar(t1));
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
    mxArray* compiled=nullptr; bool compiledExact=false; double compiledMs=0.0;
    const mxArray* base=optfld(packet,"r28_compiled_base"); const mxArray* tv=optfld(packet,"r28_target_version");
    if(R.success && base && tv){
        const auto tc=std::chrono::steady_clock::now();
        const double targetVersion=scalar(tv,"r28_target_version");
        compiled=compiled_packet_struct(base,R.cand.seg0,R.cand.seg1,R.cand.cp,t0,t1,targetVersion);
        compiledMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tc).count();
        compiledExact=true;
    }
    if(!compiled) compiled=mxCreateStructMatrix(0,0,0,nullptr);
    set("compiled_correction",compiled);set("compiled_native_exact",logicalScalar(compiledExact));
    set("compiled_native_ms",dscalar(compiledMs));set("compiled_native_revision",mxCreateString(R28_REV));
    return S;
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
        if(op==1){
            if(nrhs!=2) mexErrMsgIdAndTxt("AF7R34:Arity","plan op: op, packet.");
            const mxArray*P=prhs[1];
            std::string sig=str(fld(P,"nominal_signature"),"nominal_signature");
            if(!G_compiled||sig!=G_nominal.signature) mexErrMsgIdAndTxt("AF7R34:Reference","compiled nominal signature mismatch.");
            ActiveReference R=parse_ref(P);Live L=parse_live(P);SpeedCert B=parse_speed(P);auto O=parse_obs(P);
            double t0=scalar(fld(P,"t0"),"t0"),tm=scalar(fld(P,"tm"),"tm"),t1=scalar(fld(P,"t1"),"t1");
            PlanResult Q=plan(R,L,B,O,t0,tm,t1);plhs[0]=result_struct(Q,t0,tm,t1,O,P);return;
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
