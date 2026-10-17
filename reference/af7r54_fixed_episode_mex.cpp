
#include "mex.h"
#include "af7r54_fixed_episode_core.hpp"
#include <string>
#include <vector>
#include <stdexcept>
#include <cstring>

using namespace af7r34;

static NominalExpr G_nominal;
static bool G_compiled=false;

static const mxArray* fld(const mxArray* s,const char* name){
    if(!mxIsStruct(s)) mexErrMsgIdAndTxt("AF7R54:Packet","packet must be struct.");
    const mxArray* v=mxGetField(s,0,name);
    if(!v) mexErrMsgIdAndTxt("AF7R54:Packet","missing field %s.",name);
    return v;
}
static double scalar(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x)||mxGetNumberOfElements(x)!=1)
        mexErrMsgIdAndTxt("AF7R54:Input","%s must be real double scalar.",name);
    double v=mxGetScalar(x);
    if(!std::isfinite(v)) mexErrMsgIdAndTxt("AF7R54:Input","%s must be finite.",name);
    return v;
}
static std::vector<double> dvec(const mxArray* x,const char* name){
    if(!mxIsDouble(x)||mxIsComplex(x))
        mexErrMsgIdAndTxt("AF7R54:Input","%s must be real double.",name);
    size_t n=mxGetNumberOfElements(x);const double* p=mxGetPr(x);
    return std::vector<double>(p,p+n);
}
static std::string str(const mxArray* x,const char* name){
    if(!mxIsChar(x)) mexErrMsgIdAndTxt("AF7R54:Input","%s must be char.",name);
    char* c=mxArrayToString(x);if(!c)mexErrMsgIdAndTxt("AF7R54:Input","cannot read %s.",name);
    std::string s(c);mxFree(c);return s;
}
static mxArray* logicalScalar(bool v){return mxCreateLogicalScalar(v);}
static mxArray* dscalar(double v){return mxCreateDoubleScalar(v);}

static ActiveReference parse_ref(const mxArray* P){
    if(!G_compiled) mexErrMsgIdAndTxt("AF7R54:Reference","nominal AST is not compiled.");
    ActiveReference R;R.nominal=&G_nominal;
    R.flight_t0=scalar(fld(P,"flight_t0"),"flight_t0");

    auto offs=dvec(fld(P,"repair_offsets"),"repair_offsets");
    auto rt0=dvec(fld(P,"repair_t0"),"repair_t0");
    auto rt1=dvec(fld(P,"repair_t1"),"repair_t1");
    auto s0=dvec(fld(P,"repair_seg0"),"repair_seg0");
    auto s1=dvec(fld(P,"repair_seg1"),"repair_seg1");
    const mxArray* cpA=fld(P,"repair_cp");
    if(!mxIsDouble(cpA)||mxIsComplex(cpA)||mxGetM(cpA)!=48)
        mexErrMsgIdAndTxt("AF7R54:Packet","repair_cp must be 48xS.");
    size_t S=mxGetN(cpA);const double* cp=mxGetPr(cpA);
    if(s0.size()!=S||s1.size()!=S) mexErrMsgIdAndTxt("AF7R54:Packet","repair segment size mismatch.");
    size_t G=rt0.size();
    if(rt1.size()!=G||offs.size()!=G+1) mexErrMsgIdAndTxt("AF7R54:Packet","repair group size mismatch.");
    R.repairs.resize(G);
    for(size_t g=0;g<G;++g){
        long a=(long)std::llround(offs[g]),b=(long)std::llround(offs[g+1]);
        if(a<0||b<a||b>(long)S) mexErrMsgIdAndTxt("AF7R54:Packet","invalid repair offsets.");
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
    if(!mxIsDouble(C)||mxIsComplex(C)||mxGetM(C)!=3) mexErrMsgIdAndTxt("AF7R54:Packet","centers must be 3xM.");
    size_t M=mxGetN(C);
    if(!mxIsDouble(R)||mxIsComplex(R)||mxGetM(R)!=9||mxGetN(R)!=M) mexErrMsgIdAndTxt("AF7R54:Packet","rotations_flat must be 9xM.");
    if(!mxIsDouble(A)||mxIsComplex(A)||mxGetM(A)!=3||mxGetN(A)!=M) mexErrMsgIdAndTxt("AF7R54:Packet","base_axes must be 3xM.");
    if(!mxIsDouble(I)||mxIsComplex(I)||mxGetNumberOfElements(I)!=M) mexErrMsgIdAndTxt("AF7R54:Packet","ids must have M entries.");
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

static mxArray* row_matrix2(const std::vector<std::vector<double>>&rows){
    if(rows.empty()) return mxCreateDoubleMatrix(0,0,mxREAL);
    size_t n=rows.front().size();
    mxArray* A=mxCreateDoubleMatrix(rows.size(),n,mxREAL);
    double*p=mxGetPr(A);
    for(size_t i=0;i<rows.size();++i){
        if(rows[i].size()!=n) mexErrMsgIdAndTxt("AF7R54:Rows","row width mismatch");
        for(size_t j=0;j<n;++j)p[i+rows.size()*j]=rows[i][j];
    }
    return A;
}
static mxArray* row_b2(const std::vector<double>&b){
    mxArray*x=mxCreateDoubleMatrix(b.size(),1,mxREAL);double*p=mxGetPr(x);
    for(size_t i=0;i<b.size();++i)p[i]=b[i];return x;
}
static mxArray* cell_signs(const std::vector<CandidateDiag>&D){
    mxArray*c=mxCreateCellMatrix(1,D.size());
    for(size_t i=0;i<D.size();++i)mxSetCell(c,i,ivecrow(D[i].cand.signs));return c;
}
static mxArray* cell_active(const std::vector<CandidateDiag>&D){
    mxArray*c=mxCreateCellMatrix(1,D.size());
    for(size_t i=0;i<D.size();++i)mxSetCell(c,i,ivecrow(D[i].cand.active_set));return c;
}
static mxArray* cell_lambda(const std::vector<CandidateDiag>&D){
    mxArray*c=mxCreateCellMatrix(1,D.size());
    for(size_t i=0;i<D.size();++i)mxSetCell(c,i,vecrow(D[i].cand.lambda));return c;
}
static mxArray* cell_rowsA(const std::vector<CandidateDiag>&D){
    mxArray*c=mxCreateCellMatrix(1,D.size());
    for(size_t i=0;i<D.size();++i)mxSetCell(c,i,row_matrix2(D[i].rowA));return c;
}
static mxArray* cell_rowsb(const std::vector<CandidateDiag>&D){
    mxArray*c=mxCreateCellMatrix(1,D.size());
    for(size_t i=0;i<D.size();++i)mxSetCell(c,i,row_b2(D[i].rowb));return c;
}
static mxArray* diag_vector(const std::vector<CandidateDiag>&D,int which){
    mxArray*x=mxCreateDoubleMatrix(1,D.size(),mxREAL);double*p=mxGetPr(x);
    for(size_t i=0;i<D.size();++i){
        switch(which){
            case 0:p[i]=D[i].cand.cost;break;
            case 1:p[i]=D[i].cand.minSlack;break;
            case 2:p[i]=D[i].speed.VQ;break;
            case 3:p[i]=D[i].witness_max;break;
            case 4:p[i]=D[i].geometry_margin;break;
            default:p[i]=NAN;
        }
    }return x;
}
static mxArray* diag_logical(const std::vector<CandidateDiag>&D,int which){
    mxArray*x=mxCreateLogicalMatrix(1,D.size());mxLogical*p=mxGetLogicals(x);
    for(size_t i=0;i<D.size();++i){
        bool v=false;
        switch(which){
            case 0:v=D[i].speed_certified;break;
            case 1:v=D[i].finite_witness;break;
            case 2:v=D[i].repr;break;
            case 3:v=D[i].rejoin_pass;break;
            case 4:v=D[i].geometry_pass;break;
        }
        p[i]=v;
    }return x;
}
static mxArray* result_struct(const PlanResult&R,double t0,double tm,double t1,const std::vector<Obstacle>&obs){
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
        "qp_active_set","qp_lambda","qp_min_slack",
        "diagnostic_first_candidate_valid","diagnostic_corridor_signs",
        "diagnostic_qp_active_set","diagnostic_qp_lambda","diagnostic_qp_cost","diagnostic_qp_min_slack",
        "diagnostic_speed_certified","diagnostic_finite_witness","diagnostic_repr","diagnostic_witness_max",
        "diagnostic_speed_payload","diagnostic_speed_accel","diagnostic_speed_jerk","diagnostic_speed_sigma",
        "diagnostic_speed_qdot","diagnostic_speed_uav","diagnostic_speed_stageA",
        "all_candidate_count","all_candidate_signs","all_candidate_active_set","all_candidate_lambda",
        "all_candidate_cost","all_candidate_min_slack","all_candidate_rows_A","all_candidate_rows_b",
        "all_candidate_speed_certified","all_candidate_finite_witness","all_candidate_repr",
        "all_candidate_speed_uav","all_candidate_witness_max","all_candidate_rejoin_pass",
        "all_candidate_geometry_pass","all_candidate_geometry_margin",
        "episode_ms","hessian_ms","corridor_ms","qp_ms","candidate_build_ms","synthesis_ms","candidate_ms","total_ms"};
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
    set("qp_active_set",ivecrow(R.cand.active_set));set("qp_lambda",vecrow(R.cand.lambda));set("qp_min_slack",dscalar(R.cand.minSlack));
    set("diagnostic_first_candidate_valid",logicalScalar(R.first_cand_valid));
    set("diagnostic_corridor_signs",ivecrow(R.first_cand.signs));
    set("diagnostic_qp_active_set",ivecrow(R.first_cand.active_set));
    set("diagnostic_qp_lambda",vecrow(R.first_cand.lambda));
    set("diagnostic_qp_cost",dscalar(R.first_cand.cost));
    set("diagnostic_qp_min_slack",dscalar(R.first_cand.minSlack));
    set("diagnostic_speed_certified",logicalScalar(R.diag_speed_certified));
    set("diagnostic_finite_witness",logicalScalar(R.diag_finite_witness));
    set("diagnostic_repr",logicalScalar(R.diag_repr));
    set("diagnostic_witness_max",dscalar(R.diag_witness_max));
    set("diagnostic_speed_payload",dscalar(R.diag_speed.VL));set("diagnostic_speed_accel",dscalar(R.diag_speed.A));
    set("diagnostic_speed_jerk",dscalar(R.diag_speed.J));set("diagnostic_speed_sigma",dscalar(R.diag_speed.sigma));
    set("diagnostic_speed_qdot",dscalar(R.diag_speed.qdot));set("diagnostic_speed_uav",dscalar(R.diag_speed.VQ));
    set("diagnostic_speed_stageA",logicalScalar(R.diag_speed.stageA));
    set("all_candidate_count",dscalar((double)R.all_diag.size()));
    set("all_candidate_signs",cell_signs(R.all_diag));
    set("all_candidate_active_set",cell_active(R.all_diag));
    set("all_candidate_lambda",cell_lambda(R.all_diag));
    set("all_candidate_cost",diag_vector(R.all_diag,0));
    set("all_candidate_min_slack",diag_vector(R.all_diag,1));
    set("all_candidate_rows_A",cell_rowsA(R.all_diag));
    set("all_candidate_rows_b",cell_rowsb(R.all_diag));
    set("all_candidate_speed_certified",diag_logical(R.all_diag,0));
    set("all_candidate_finite_witness",diag_logical(R.all_diag,1));
    set("all_candidate_repr",diag_logical(R.all_diag,2));
    set("all_candidate_speed_uav",diag_vector(R.all_diag,2));
    set("all_candidate_witness_max",diag_vector(R.all_diag,3));
    set("all_candidate_rejoin_pass",diag_logical(R.all_diag,3));
    set("all_candidate_geometry_pass",diag_logical(R.all_diag,4));
    set("all_candidate_geometry_margin",diag_vector(R.all_diag,4));
    set("episode_ms",dscalar(R.tim.episode));set("hessian_ms",dscalar(R.tim.hessian));
    set("corridor_ms",dscalar(R.tim.corridor));set("qp_ms",dscalar(R.tim.qp));
    set("candidate_build_ms",dscalar(R.tim.candidate_build));set("synthesis_ms",dscalar(R.tim.synthesis));
    set("candidate_ms",dscalar(R.tim.candidate));set("total_ms",dscalar(R.tim.total));
    return S;
}
static mxArray* eval_matrix(const ActiveReference&R,const std::vector<double>&t){
    mxArray*Y=mxCreateDoubleMatrix(21,t.size(),mxREAL);double*p=mxGetPr(Y);
    for(size_t k=0;k<t.size();++k){auto y=R.eval(t[k]);for(int r=0;r<7;++r){p[(3*r+0)+21*k]=y[r].x;p[(3*r+1)+21*k]=y[r].y;p[(3*r+2)+21*k]=y[r].z;}}
    return Y;
}


struct TailSafeCert{
    bool pass=false;
    int leaves=0,accepted=0,splits=0,repr_fail=0;
    double min_margin=std::numeric_limits<double>::infinity();
};
static inline TailSafeCert certify_tail_safe(const ActiveReference&ref,const Live&L,
                                             const SpeedCert&base,const std::vector<Obstacle>&obs,
                                             double ta,double tb){
    TailSafeCert C;
    if(!(std::isfinite(ta)&&std::isfinite(tb)&&tb>ta&&!obs.empty()))return C;
    std::vector<std::pair<double,double>> st{{ta,tb}};
    double VC=std::max(base.VL,base.VQ);
    while(!st.empty()){
        auto f=st.back();st.pop_back();
        double a=f.first,b=f.second,tc=0.5*(a+b),h=0.5*(b-a);
        ++C.leaves;
        bool split=std::isfinite(tc)&&tc>a&&tc<b;
        bool safe=false;
        double mm=std::numeric_limits<double>::infinity();
        if(split){
            auto x=ref.eval(tc);
            Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};
            double ns=norm(sf);
            if(std::isfinite(ns)&&ns>0){
                Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;
                safe=true;
                for(const auto&o:obs){
                    double mL,mQ,mC;
                    bool sL=point_margin(pL,o,L.rL+L.dmargin+L.tracking+base.VL*h,mL);
                    bool sQ=point_margin(pQ,o,L.rQ+L.dmargin+L.tracking+base.VQ*h,mQ);
                    bool sC=segment_margin(pQ,pL,o,L.rC+L.dmargin+L.tracking+VC*h,mC);
                    if(!(sL&&sQ&&sC)){safe=false;break;}
                    mm=std::min(mm,std::min({mL,mQ,mC}));
                }
            }
        }
        if(safe){
            ++C.accepted;C.min_margin=std::min(C.min_margin,mm);continue;
        }
        if(!split){++C.repr_fail;return C;}
        st.push_back({tc,b});st.push_back({a,tc});++C.splits;
    }
    C.pass=(C.accepted>0&&C.repr_fail==0&&std::isfinite(C.min_margin)&&C.min_margin>0);
    return C;
}
static mxArray* fixed_episode_struct(const ActiveReference&R,const Live&L,const SpeedCert&B,
                                     const std::vector<Obstacle>&O,double t0,double tm,double tU){
    EpisodeCert E=build_episode(R,L,B,O,t0,tU);
    if(!E.ok||E.comp.empty()||E.ep.empty())
        mexErrMsgIdAndTxt("AF7R54:Episode","fixed-domain episode unavailable.");
    double lastEnd=E.comp.back().end;
    double tailStart=std::nextafter(lastEnd,std::numeric_limits<double>::infinity());
    if(!(tailStart<tU))
        mexErrMsgIdAndTxt("AF7R54:Episode","no tail interval after last unsafe component.");
    TailSafeCert T=certify_tail_safe(R,L,B,O,tailStart,tU);
    PlanResult P=plan(R,L,B,O,t0,tm,tU);

    const char*fn[]={"plan","episode_ok","tail_safe","tail_start","last_component_end",
        "tail_leaves","tail_accepted","tail_splits","tail_repr_fail","tail_min_margin",
        "t_upper"};
    mxArray*S=mxCreateStructMatrix(1,1,sizeof(fn)/sizeof(fn[0]),fn);
    mxSetField(S,0,"plan",result_struct(P,t0,tm,tU,O));
    mxSetField(S,0,"episode_ok",logicalScalar(E.ok));
    mxSetField(S,0,"tail_safe",logicalScalar(T.pass));
    mxSetField(S,0,"tail_start",dscalar(tailStart));
    mxSetField(S,0,"last_component_end",dscalar(lastEnd));
    mxSetField(S,0,"tail_leaves",dscalar(T.leaves));
    mxSetField(S,0,"tail_accepted",dscalar(T.accepted));
    mxSetField(S,0,"tail_splits",dscalar(T.splits));
    mxSetField(S,0,"tail_repr_fail",dscalar(T.repr_fail));
    mxSetField(S,0,"tail_min_margin",dscalar(T.min_margin));
    mxSetField(S,0,"t_upper",dscalar(tU));
    return S;
}

void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    try{
        if(nrhs<1||!mxIsDouble(prhs[0])||mxGetNumberOfElements(prhs[0])!=1)
            mexErrMsgIdAndTxt("AF7R54:Input","first input must be op scalar.");
        int op=(int)mxGetScalar(prhs[0]);
        if(op==0){
            if(nrhs!=3) mexErrMsgIdAndTxt("AF7R54:Arity","compile op: op, exprCell, signature.");
            if(!mxIsCell(prhs[1])||mxGetNumberOfElements(prhs[1])!=21) mexErrMsgIdAndTxt("AF7R54:Input","exprCell must contain 21 cells.");
            std::vector<std::string> ex(21);
            for(int i=0;i<21;++i)ex[i]=str(mxGetCell(prhs[1],i),"expression");
            std::string sig=str(prhs[2],"signature");G_nominal.compile(ex,sig);G_compiled=true;
            if(nlhs>0)plhs[0]=logicalScalar(true);return;
        }
        if(op==2){
            if(nrhs!=3) mexErrMsgIdAndTxt("AF7R54:Arity","eval op: op, packet, times.");
            ActiveReference R=parse_ref(prhs[1]);auto t=dvec(prhs[2],"times");plhs[0]=eval_matrix(R,t);return;
        }
        if(op==1){
            if(nrhs!=2) mexErrMsgIdAndTxt("AF7R54:Arity","plan op: op, packet.");
            const mxArray*P=prhs[1];
            std::string sig=str(fld(P,"nominal_signature"),"nominal_signature");
            if(!G_compiled||sig!=G_nominal.signature) mexErrMsgIdAndTxt("AF7R54:Reference","compiled nominal signature mismatch.");
            ActiveReference R=parse_ref(P);Live L=parse_live(P);SpeedCert B=parse_speed(P);auto O=parse_obs(P);
            double t0=scalar(fld(P,"t0"),"t0"),tm=scalar(fld(P,"tm"),"tm"),t1=scalar(fld(P,"t1"),"t1");
            PlanResult Q=plan(R,L,B,O,t0,tm,t1);plhs[0]=result_struct(Q,t0,tm,t1,O);return;
        }
        if(op==3){
            if(nrhs!=3) mexErrMsgIdAndTxt("AF7R54:Arity","fixed episode op: op, packet, tUpper.");
            const mxArray*P=prhs[1];
            std::string sig=str(fld(P,"nominal_signature"),"nominal_signature");
            if(!G_compiled||sig!=G_nominal.signature) mexErrMsgIdAndTxt("AF7R54:Reference","compiled nominal signature mismatch.");
            ActiveReference R=parse_ref(P);Live L=parse_live(P);SpeedCert B=parse_speed(P);auto O=parse_obs(P);
            double t0=scalar(fld(P,"t0"),"t0"),tm=scalar(fld(P,"tm"),"tm"),tU=scalar(prhs[2],"tUpper");
            plhs[0]=fixed_episode_struct(R,L,B,O,t0,tm,tU);return;
        }
        mexErrMsgIdAndTxt("AF7R54:Op","unknown op.");
    }catch(const std::exception&e){
        mexErrMsgIdAndTxt("AF7R54:Exception","%s",e.what());
    }
}
