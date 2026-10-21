
#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace af7r34 {

static inline double epsd(){ return std::numeric_limits<double>::epsilon(); }
static inline bool fin(double x){ return std::isfinite(x); }

struct Vec3 {
    double x=0,y=0,z=0;
    Vec3()=default;
    Vec3(double X,double Y,double Z):x(X),y(Y),z(Z){}
};
static inline Vec3 operator+(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static inline Vec3 operator-(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static inline Vec3 operator*(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
static inline Vec3 operator*(double s,Vec3 a){return a*s;}
static inline Vec3 operator/(Vec3 a,double s){return {a.x/s,a.y/s,a.z/s};}
static inline Vec3& operator+=(Vec3& a,Vec3 b){a=a+b;return a;}
static inline double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static inline Vec3 cross(Vec3 a,Vec3 b){
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
static inline double norm2(Vec3 a){return dot(a,a);}
static inline double norm(Vec3 a){return std::sqrt(norm2(a));}
static inline bool finite(Vec3 a){return fin(a.x)&&fin(a.y)&&fin(a.z);}
static inline Vec3 normalized(Vec3 a){
    double n=norm(a); if(!(fin(n)&&n>0)) throw std::runtime_error("zero Vec3 normalization");
    return a/n;
}
static inline double maxabs(Vec3 a){return std::max({std::fabs(a.x),std::fabs(a.y),std::fabs(a.z)});}

struct Mat3 {
    // row-major
    double a[9]{};
    double& operator()(int r,int c){return a[3*r+c];}
    double operator()(int r,int c) const{return a[3*r+c];}
    static Mat3 I(){ Mat3 M; M(0,0)=M(1,1)=M(2,2)=1; return M; }
};
static inline Vec3 mul(const Mat3& M,Vec3 v){
    return {M(0,0)*v.x+M(0,1)*v.y+M(0,2)*v.z,
            M(1,0)*v.x+M(1,1)*v.y+M(1,2)*v.z,
            M(2,0)*v.x+M(2,1)*v.y+M(2,2)*v.z};
}
static inline Vec3 tmul(const Mat3& M,Vec3 v){
    return {M(0,0)*v.x+M(1,0)*v.y+M(2,0)*v.z,
            M(0,1)*v.x+M(1,1)*v.y+M(2,1)*v.z,
            M(0,2)*v.x+M(1,2)*v.y+M(2,2)*v.z};
}
static inline Mat3 transpose(const Mat3& M){
    Mat3 T; for(int r=0;r<3;++r)for(int c=0;c<3;++c)T(r,c)=M(c,r); return T;
}
static inline Mat3 operator+(const Mat3&A,const Mat3&B){
    Mat3 C;for(int i=0;i<9;++i)C.a[i]=A.a[i]+B.a[i];return C;
}
static inline Mat3 operator-(const Mat3&A,const Mat3&B){
    Mat3 C;for(int i=0;i<9;++i)C.a[i]=A.a[i]-B.a[i];return C;
}
static inline Mat3 operator*(const Mat3&A,double s){
    Mat3 C;for(int i=0;i<9;++i)C.a[i]=A.a[i]*s;return C;
}
static inline Mat3 mmul(const Mat3&A,const Mat3&B){
    Mat3 C;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){
        double s=0;for(int k=0;k<3;++k)s+=A(i,k)*B(k,j); C(i,j)=s;
    }
    return C;
}
static inline double frob(const Mat3&M){double s=0;for(double v:M.a)s+=v*v;return std::sqrt(s);}

struct SymEig3 {
    std::array<double,3> val{};
    Mat3 V=Mat3::I(); // columns eigenvectors
};
static inline SymEig3 jacobi_sym3(Mat3 A){
    SymEig3 out;
    Mat3 V=Mat3::I();
    for(int it=0;it<40;++it){
        int p=0,q=1;
        double m=std::fabs(A(0,1));
        if(std::fabs(A(0,2))>m){m=std::fabs(A(0,2));p=0;q=2;}
        if(std::fabs(A(1,2))>m){m=std::fabs(A(1,2));p=1;q=2;}
        double scale=std::max({1.0,std::fabs(A(0,0)),std::fabs(A(1,1)),std::fabs(A(2,2))});
        if(m<=64*epsd()*scale) break;
        double app=A(p,p), aqq=A(q,q), apq=A(p,q);
        double phi=0.5*std::atan2(2*apq,aqq-app);
        double c=std::cos(phi),s=std::sin(phi);
        for(int k=0;k<3;++k){
            double aik=A(k,p), aiq=A(k,q);
            A(k,p)=c*aik-s*aiq;
            A(k,q)=s*aik+c*aiq;
        }
        for(int k=0;k<3;++k){
            double apk=A(p,k), aqk=A(q,k);
            A(p,k)=c*apk-s*aqk;
            A(q,k)=s*apk+c*aqk;
        }
        A(p,q)=A(q,p)=0;
        for(int k=0;k<3;++k){
            double vip=V(k,p),viq=V(k,q);
            V(k,p)=c*vip-s*viq;
            V(k,q)=s*vip+c*viq;
        }
    }
    out.val={A(0,0),A(1,1),A(2,2)};
    out.V=V;
    return out;
}
static inline Vec3 largest_eigvec(Mat3 Q){
    auto E=jacobi_sym3(Q);
    int ix=0; if(E.val[1]>E.val[ix])ix=1; if(E.val[2]>E.val[ix])ix=2;
    Vec3 v{E.V(0,ix),E.V(1,ix),E.V(2,ix)};
    return normalized(v);
}

// ------------------------- expression evaluator ----------------------------
enum class NType {CONST,VAR,ADD,SUB,MUL,DIV,POW,NEG,FUNC1,FUNC2};
struct Node {
    NType type=NType::CONST;
    double c=0;
    std::string fn;
    std::unique_ptr<Node> l,r;
    double eval(double t) const {
        switch(type){
            case NType::CONST:return c;
            case NType::VAR:return t;
            case NType::ADD:return l->eval(t)+r->eval(t);
            case NType::SUB:return l->eval(t)-r->eval(t);
            case NType::MUL:return l->eval(t)*r->eval(t);
            case NType::DIV:return l->eval(t)/r->eval(t);
            case NType::POW:return std::pow(l->eval(t),r->eval(t));
            case NType::NEG:return -l->eval(t);
            case NType::FUNC1:{
                double x=l->eval(t);
                if(fn=="sin")return std::sin(x); if(fn=="cos")return std::cos(x);
                if(fn=="tan")return std::tan(x); if(fn=="asin")return std::asin(x);
                if(fn=="acos")return std::acos(x); if(fn=="atan")return std::atan(x);
                if(fn=="sinh")return std::sinh(x); if(fn=="cosh")return std::cosh(x);
                if(fn=="tanh")return std::tanh(x); if(fn=="exp")return std::exp(x);
                if(fn=="log")return std::log(x); if(fn=="sqrt")return std::sqrt(x);
                if(fn=="abs")return std::fabs(x); if(fn=="floor")return std::floor(x);
                if(fn=="ceil")return std::ceil(x); if(fn=="sign")return (x>0)-(x<0);
                throw std::runtime_error("unsupported unary function "+fn);
            }
            case NType::FUNC2:{
                double a=l->eval(t),b=r->eval(t);
                if(fn=="atan2")return std::atan2(a,b);
                if(fn=="min")return std::min(a,b); if(fn=="max")return std::max(a,b);
                if(fn=="pow")return std::pow(a,b);
                throw std::runtime_error("unsupported binary function "+fn);
            }
        }
        return std::numeric_limits<double>::quiet_NaN();
    }
};
class Parser {
    std::string s; size_t p=0;
    void ws(){while(p<s.size()&&std::isspace((unsigned char)s[p]))++p;}
    bool eat(char c){ws();if(p<s.size()&&s[p]==c){++p;return true;}return false;}
    std::string ident(){
        ws();size_t b=p;
        if(p<s.size()&&(std::isalpha((unsigned char)s[p])||s[p]=='_'))++p; else return "";
        while(p<s.size()&&(std::isalnum((unsigned char)s[p])||s[p]=='_'))++p;
        return s.substr(b,p-b);
    }
    std::unique_ptr<Node> expr(){
        auto n=term();
        while(true){ if(eat('+')){auto q=std::make_unique<Node>();q->type=NType::ADD;q->l=std::move(n);q->r=term();n=std::move(q);}
            else if(eat('-')){auto q=std::make_unique<Node>();q->type=NType::SUB;q->l=std::move(n);q->r=term();n=std::move(q);} else break;}
        return n;
    }
    std::unique_ptr<Node> term(){
        auto n=power();
        while(true){ if(eat('*')){auto q=std::make_unique<Node>();q->type=NType::MUL;q->l=std::move(n);q->r=power();n=std::move(q);}
            else if(eat('/')){auto q=std::make_unique<Node>();q->type=NType::DIV;q->l=std::move(n);q->r=power();n=std::move(q);} else break;}
        return n;
    }
    std::unique_ptr<Node> power(){
        auto n=unary();
        if(eat('^')){auto q=std::make_unique<Node>();q->type=NType::POW;q->l=std::move(n);q->r=power();return q;}
        return n;
    }
    std::unique_ptr<Node> unary(){
        if(eat('+')) return unary();
        if(eat('-')){auto q=std::make_unique<Node>();q->type=NType::NEG;q->l=unary();return q;}
        return primary();
    }
    std::unique_ptr<Node> primary(){
        ws();
        if(eat('(')){auto n=expr();if(!eat(')'))throw std::runtime_error("missing )");return n;}
        ws();
        if(p<s.size()&&(std::isdigit((unsigned char)s[p])||s[p]=='.')){
            char* end=nullptr; const char* b=s.c_str()+p; double v=std::strtod(b,&end);
            if(end==b)throw std::runtime_error("bad number");
            p=(size_t)(end-s.c_str()); auto n=std::make_unique<Node>();n->type=NType::CONST;n->c=v;return n;
        }
        std::string id=ident();
        if(id.empty())throw std::runtime_error("bad token at "+std::to_string(p));
        if(id=="pi"||id=="PI"){auto n=std::make_unique<Node>();n->type=NType::CONST;n->c=3.141592653589793238462643383279502884;return n;}
        if(id=="Inf"||id=="inf"){auto n=std::make_unique<Node>();n->type=NType::CONST;n->c=std::numeric_limits<double>::infinity();return n;}
        ws();
        if(eat('(')){
            auto a=expr();
            if(eat(',')){auto b=expr();if(!eat(')'))throw std::runtime_error("missing )");
                auto n=std::make_unique<Node>();n->type=NType::FUNC2;n->fn=id;n->l=std::move(a);n->r=std::move(b);return n;}
            if(!eat(')'))throw std::runtime_error("missing )");
            auto n=std::make_unique<Node>();n->type=NType::FUNC1;n->fn=id;n->l=std::move(a);return n;
        }
        // normalized builder is required to leave only one free variable.
        auto n=std::make_unique<Node>();n->type=NType::VAR;return n;
    }
public:
    explicit Parser(std::string x):s(std::move(x)){}
    std::unique_ptr<Node> parse(){
        auto n=expr();ws(); if(p!=s.size())throw std::runtime_error("trailing expression text: "+s.substr(p));
        return n;
    }
};

struct NominalExpr {
    std::array<std::unique_ptr<Node>,21> f;
    std::string signature;
    void compile(const std::vector<std::string>& ex,const std::string& sig){
        if(ex.size()!=21)throw std::runtime_error("need 21 nominal scalar expressions");
        for(size_t i=0;i<21;++i) f[i]=Parser(ex[i]).parse();
        signature=sig;
    }
    std::array<Vec3,7> eval(double tau) const{
        std::array<Vec3,7> y;
        for(int r=0;r<7;++r){
            y[r]={f[3*r+0]->eval(tau),f[3*r+1]->eval(tau),f[3*r+2]->eval(tau)};
            if(!finite(y[r]))throw std::runtime_error("nonfinite nominal expression");
        }
        return y;
    }
};

// ----------------------------- Bezier ---------------------------------------
using Poly=std::vector<Vec3>;
static inline Vec3 decasteljau(Poly C,double u){
    while(C.size()>1){
        Poly D(C.size()-1);
        for(size_t i=0;i+1<C.size();++i)D[i]=C[i]*(1-u)+C[i+1]*u;
        C.swap(D);
    }
    return C.front();
}
static inline Poly deriv_poly(const Poly&C,double T,int order=1){
    Poly D=C;
    int deg=(int)D.size()-1;
    for(int r=0;r<order;++r){
        if(!(T>0)||D.size()<2)return {};
        Poly E(D.size()-1);
        for(size_t i=0;i+1<D.size();++i)E[i]=(D[i+1]-D[i])*(double(deg)/T);
        D.swap(E); --deg;
    }
    return D;
}
static inline std::pair<Poly,Poly> split_half(const Poly&C){
    size_t m=C.size(); Poly L(m),R(m),T=C;
    L[0]=T[0];R[m-1]=T[m-1];
    for(size_t k=1;k<m;++k){
        Poly U(T.size()-1);
        for(size_t i=0;i+1<T.size();++i)U[i]=(T[i]+T[i+1])*0.5;
        T.swap(U);L[k]=T.front();R[m-1-k]=T.back();
    }
    return {L,R};
}
static inline double poly_norm_ub(const Poly&C){
    double u=0;for(auto v:C)u=std::max(u,norm(v));return u;
}
static inline std::vector<double> scalar_control_from_jets(const std::array<double,8>&j0,
                                                           const std::array<double,8>&j1,double T){
    std::array<double,8> ds{},de{};ds[0]=j0[0];de[0]=j1[0];
    double ff=1;
    for(int r=1;r<=7;++r){ff*=16-r;ds[r]=j0[r]*std::pow(T,r)/ff;de[r]=j1[r]*std::pow(T,r)/ff;}
    std::vector<double>P(16,0);
    auto choose=[](int n,int k)->double{
        if(k<0||k>n)return 0; double v=1;for(int i=1;i<=k;++i)v*=double(n-k+i)/double(i);return v;
    };
    for(int k=0;k<=7;++k){
        double v0=0,v1=0;
        for(int r=0;r<=k;++r){
            double bc=choose(k,r);v0+=bc*ds[r];v1+=((r&1)?-1.0:1.0)*bc*de[r];
        }
        P[k]=v0;P[15-k]=v1;
    }
    return P;
}
static inline Poly vector_control_from_jets(const std::array<Vec3,8>&j0,
                                             const std::array<Vec3,8>&j1,double T){
    std::array<double,8>x0{},x1{},y0{},y1{},z0{},z1{};
    for(int r=0;r<8;++r){x0[r]=j0[r].x;x1[r]=j1[r].x;y0[r]=j0[r].y;y1[r]=j1[r].y;z0[r]=j0[r].z;z1[r]=j1[r].z;}
    auto X=scalar_control_from_jets(x0,x1,T),Y=scalar_control_from_jets(y0,y1,T),Z=scalar_control_from_jets(z0,z1,T);
    Poly P(16);for(int i=0;i<16;++i)P[i]={X[i],Y[i],Z[i]};return P;
}

struct CorrSegment { double t0=0,t1=0; Poly cp; };
struct Correction { double t0=0,t1=0; std::vector<CorrSegment> seg; };

struct ActiveReference {
    const NominalExpr* nominal=nullptr;
    double flight_t0=0;
    std::vector<Correction> repairs;
    std::array<Vec3,7> eval(double t) const{
        double tau=std::max(0.0,t-flight_t0);
        auto y=nominal->eval(tau);
        for(const auto& C:repairs){
            if(t<=C.t0)continue;

            // AF7R34: a nonzero final correction is a persistent schedule
            // delay for an affine nominal reference. Historical repairs end
            // at zero, so this branch is exactly a no-op for them.
            if(t>=C.t1){
                if(!C.seg.empty()) y[0]+=C.seg.back().cp.back();
                continue;
            }

            const CorrSegment* sp=nullptr;
            for(const auto& s:C.seg){if(t<=s.t1){sp=&s;break;}}
            if(!sp)continue;
            if(t<sp->t0)continue;
            double T=sp->t1-sp->t0,u=(t-sp->t0)/T;u=std::max(0.0,std::min(1.0,u));
            Poly D=sp->cp;
            y[0]+=decasteljau(D,u);
            int deg=15;
            for(int r=1;r<=6;++r){
                Poly E(D.size()-1);
                for(size_t i=0;i+1<D.size();++i)E[i]=(D[i+1]-D[i])*(double(deg)/T);
                D.swap(E);--deg;y[r]+=decasteljau(D,u);
            }
        }
        return y;
    }
};

// ------------------------ geometry support ---------------------------------
struct Obstacle { Vec3 c; Mat3 R; Vec3 axes; double id=0; };

static inline bool point_margin(Vec3 x,const Obstacle&o,double rho,double&best){
    Vec3 d=x-o.c;Vec3 y=tmul(o.R,d);
    Vec3 y2{y.x/(o.axes.x*o.axes.x),y.y/(o.axes.y*o.axes.y),y.z/(o.axes.z*o.axes.z)};
    Vec3 n0=mul(o.R,y2),n1=d; Vec3 N[2]={n0,n1};
    best=-std::numeric_limits<double>::infinity();bool ok=false;
    for(auto n:N){
        double nn=norm(n);if(!(fin(nn)&&nn>0))continue;
        Vec3 rn=tmul(o.R,n);Vec3 ar{o.axes.x*rn.x,o.axes.y*rn.y,o.axes.z*rn.z};
        double hs=dot(n,o.c)+norm(ar);
        double m=(dot(n,x)-hs-rho*nn)/nn;
        if(fin(m)){best=std::max(best,m);ok=true;}
    }
    return ok&&fin(best)&&best>0;
}
static inline bool segment_margin(Vec3 p0,Vec3 p1,const Obstacle&o,double rho,double&best){
    Vec3 d0=p0-o.c,d1=p1-o.c;
    Vec3 z0=tmul(o.R,d0),z1=tmul(o.R,d1);
    z0={z0.x/o.axes.x,z0.y/o.axes.y,z0.z/o.axes.z};
    z1={z1.x/o.axes.x,z1.y/o.axes.y,z1.z/o.axes.z};
    Vec3 sv=z1-z0;double ss=norm2(sv),lam=0;
    if(ss>0){lam=-dot(z0,sv)/ss;lam=std::max(0.0,std::min(1.0,lam));}
    Vec3 xs=p0+(p1-p0)*lam;Vec3 Y[3]={xs,p0,p1};
    best=-std::numeric_limits<double>::infinity();bool ok=false;
    for(auto x:Y){
        Vec3 d=x-o.c,yr=tmul(o.R,d);
        Vec3 y2{yr.x/(o.axes.x*o.axes.x),yr.y/(o.axes.y*o.axes.y),yr.z/(o.axes.z*o.axes.z)};
        Vec3 N[2]={mul(o.R,y2),d};
        for(auto n:N){
            double nn=norm(n);if(!(fin(nn)&&nn>0))continue;
            Vec3 rn=tmul(o.R,n);Vec3 ar{o.axes.x*rn.x,o.axes.y*rn.y,o.axes.z*rn.z};
            double hs=dot(n,o.c)+norm(ar);
            double proj=std::min(dot(n,p0),dot(n,p1));
            double m=(proj-hs-rho*nn)/nn;
            if(fin(m)){best=std::max(best,m);ok=true;}
        }
    }
    return ok&&fin(best)&&best>0;
}

// ------------------------------- planner data -------------------------------
struct SpeedCert {
    bool certified=false;
    double VL=0,A=0,J=0,sigma=0,qdot=0,VQ=0;
    int leaf_count=0,split_count=0;
    bool stageA=false;
};
struct Live {
    double dt=0,g=9.81,L=0,rL=0,rQ=0,rC=0,dmargin=0,sensor_range=0,horizon=0,tracking=0;
};
struct Episode { int obs=0,comp=0; double start=0,end=0; };
struct Component { double start=0,end=0; std::vector<int> episodes; };
struct EpisodeCert {
    bool ok=false;std::vector<Episode> ep;std::vector<Component> comp;
    int leaves=0,safe_leaves=0,unsafe_leaves=0,splits=0,repr_fail=0;
};
struct GeometryCert {
    bool pass=false;int reason=0,leaf_count=0,accepted=0,splits=0,max_depth=0;
    double minL=std::numeric_limits<double>::quiet_NaN();
    double minQ=std::numeric_limits<double>::quiet_NaN();
    double minC=std::numeric_limits<double>::quiet_NaN();
    double minMargin=std::numeric_limits<double>::quiet_NaN();
};
struct Candidate {
    double cost=0,minSlack=0;
    // family: 1 = exact minimum integrated squared deviation affine family,
    //         2 = historical minimum-snap affine fallback,
    //         3 = towing natural-time C3 quadratic family,
    //         4 = towing minimum-norm constant family,
    //         5 = towing speed-optimized exact-C7 split family,
    //         6 = curved exact-C7 zero-jet bell migration family,
    //         7 = affine-base progress-retimed exact-C7 primary.
    int family=0;
    bool constant_translation=false;
    bool exact_c7=false;
    std::vector<int> signs;
    std::vector<double> seg0,seg1;
    std::vector<Poly> cp;
    std::vector<Vec3> pos,vel;
    std::array<double,6> dub{};
    double max_ctrl=0;
    bool progress_retimed=false;
    double progress_scale=1.0;
    double retime_entry_T=0.0;
    double retime_flow_T=0.0;
    double retime_exit_T=0.0;

    // AF7R34 delayed-schedule rejoin.
    // After terminal_hold_start the correction is a constant position offset.
    // For an exactly affine nominal base this equals p_nom(t-delay)-p_nom(t),
    // so the vehicle rejoins the nominal SPATIAL path without ever having to
    // catch back up to the old clock schedule.
    bool terminal_hold=false;
    double terminal_hold_start=0.0;
    Vec3 terminal_offset{};
    double schedule_delay=0.0;

    // AF7R35 path-tangent yaw theorem metadata.
    // If heading_c6 is true, the actual horizontal velocity stays in the
    // open half-plane centered on heading_anchor_rad with certified
    // projection >= heading_gamma > 0.  Since position is exact C7, yaw =
    // atan2(pdot_y,pdot_x) is therefore globally single-branch C6.
    bool heading_c6=false;
    double heading_gamma=std::numeric_limits<double>::quiet_NaN();
    double heading_anchor_rad=std::numeric_limits<double>::quiet_NaN();
    // AF7R45R3: exact native authority vector used by the yaw-C6 theorem.
    // MATLAB must consume this same vector; it must not re-estimate it.
    Vec3 heading_base_velocity{};

    // AF7R37 trigger-to-free-rejoin spatial path.
    bool trigger_to_free_rejoin=false;
    bool identity_time_law=true;  // phase-A path/time contract: s(t)=t
    double actual_rejoin_time=std::numeric_limits<double>::quiet_NaN();
    int spatial_knot_count=0;

    // AF7R38 exact-real C7 proof metadata for trigger-to-free-rejoin family 7.
    bool exact_c7_constructive=false;
    bool legacy_float_endpoint_checker_pass=false;
};
struct PlanResult {
    bool success=false;int reason=0;
    EpisodeCert episode;
    Candidate cand;
    SpeedCert speed;
    GeometryCert geom;
    int speed_pass=0,rejoin_pass=0,geom_tests=0;

    // Branch-local synthesis/certificate diagnostics. These never grant
    // acceptance; they only explain which unchanged hard gate rejected the
    // primary minimum-deviation family before the old fallback was used.
    int primary_candidate_count=0;
    int primary_speed_pass=0;
    int primary_rejoin_pass=0;
    int primary_geom_tests=0;
    int fallback_candidate_count=0;
    int fallback_speed_pass=0;
    int fallback_rejoin_pass=0;
    int fallback_geom_tests=0;

    // AF7R24E_FAMILY_DIAGNOSTICS -- diagnostic only.
    std::array<int,8> r24e_family_seen{};
    std::array<int,8> r24e_family_exact{};
    std::array<int,8> r24e_family_exact_reject{};
    std::array<int,8> r24e_family_speed_pass{};
    std::array<int,8> r24e_family_speed_fail{};
    int r24e_split_representability_reject=0;
    int towing_candidate_count=0;
    int towing_speed_pass=0;
    int towing_rejoin_pass=0;
    int towing_geom_tests=0;

    // AF7R28 flowing-family pre-certificate diagnostics. Diagnostic only:
    // none of these counters can grant acceptance.
    int r28_flow_side_assignments=0;
    int r28_flow_qp_solutions=0;
    int r28_flow_built=0;
    int r28_flow_row_reject=0;
    int r28_flow_grid_reject=0;
    int r28_flow_support_reject=0;
    double r28_flow_step_s=std::numeric_limits<double>::quiet_NaN();
    double r28_flow_entry_end_s=std::numeric_limits<double>::quiet_NaN();
    double r28_flow_exit_start_s=std::numeric_limits<double>::quiet_NaN();
    double r28_flow_exit_end_s=std::numeric_limits<double>::quiet_NaN();
    double r30_flow_support_start_s=std::numeric_limits<double>::quiet_NaN();
    double r30_flow_wait_s=std::numeric_limits<double>::quiet_NaN();
    double r30_first_unsafe_s=std::numeric_limits<double>::quiet_NaN();
    double r30_last_unsafe_s=std::numeric_limits<double>::quiet_NaN();
    double r30_reference_end_s=std::numeric_limits<double>::quiet_NaN();
    double r30_lead_s=std::numeric_limits<double>::quiet_NaN();
    double r30_tail_s=std::numeric_limits<double>::quiet_NaN();
    double r31_entry_duration_s=std::numeric_limits<double>::quiet_NaN();
    double r31_flow_segment_duration_s=std::numeric_limits<double>::quiet_NaN();
    double r31_exit_duration_s=std::numeric_limits<double>::quiet_NaN();
    double r31_best_correction_speed_ub=std::numeric_limits<double>::infinity();
    double r31_best_stageA_uav_speed_ub=std::numeric_limits<double>::infinity();
    int r31_exact_join_reject=0;
    int r32_affine_base_detected=0;
    int r32_timing_trial_count=0;
    int r32_exact_candidate_count=0;
    int r32_monotone_reject_count=0;
    double r32_nominal_progress_speed=std::numeric_limits<double>::quiet_NaN();
    double r32_best_pre_speed_ub=std::numeric_limits<double>::infinity();
    double r32_best_progress_scale=std::numeric_limits<double>::quiet_NaN();
    double r32_best_entry_T=std::numeric_limits<double>::quiet_NaN();
    double r32_best_flow_T=std::numeric_limits<double>::quiet_NaN();
    double r32_best_exit_T=std::numeric_limits<double>::quiet_NaN();

    int r33_exact_timing_candidate_count=0;
    int r33_pareto_candidate_count=0;
    int r33_quantized_progress_reject_count=0;
    double r33_time_quantum_s=std::numeric_limits<double>::quiet_NaN();
    double r33_support_start_s=std::numeric_limits<double>::quiet_NaN();
    double r33_support_end_s=std::numeric_limits<double>::quiet_NaN();
    double r33_best_accel_ub=std::numeric_limits<double>::infinity();
    double r33_best_jerk_ub=std::numeric_limits<double>::infinity();

    // AF7R34 delay-preserving retimer diagnostics.
    int r34_delay_candidate_count=0;
    int r34_terminal_hold_exact_count=0;
    int r34_monotone_progress_count=0;
    double r34_selected_delay_s=std::numeric_limits<double>::quiet_NaN();
    double r34_selected_hold_start_s=std::numeric_limits<double>::quiet_NaN();
    double r34_selected_alpha=std::numeric_limits<double>::quiet_NaN();
    double r34_selected_velocity_ub=std::numeric_limits<double>::infinity();
    double r34_selected_accel_ub=std::numeric_limits<double>::infinity();
    double r34_selected_jerk_ub=std::numeric_limits<double>::infinity();

    // AF7R35 path-tangent yaw diagnostics.
    int r35_heading_candidate_count=0;
    int r35_heading_pass_count=0;
    int r35_heading_reject_count=0;
    double r35_best_heading_gamma_m_s=std::numeric_limits<double>::quiet_NaN();
    double r35_selected_heading_gamma_m_s=std::numeric_limits<double>::quiet_NaN();
    double r35_selected_heading_anchor_rad=std::numeric_limits<double>::quiet_NaN();

    int r36_spatial_qp_solutions=0;
    int r36_spatial_row_rejects=0;
    int r36_timing_trials=0;
    int r36_exact_curved_candidates=0;
    int r36_curved_heading_pass=0;
    double r36_mid_progress_s=std::numeric_limits<double>::quiet_NaN();
    double r36_best_spatial_path_speed_ub=std::numeric_limits<double>::infinity();
    double r36_best_total_velocity_ub=std::numeric_limits<double>::infinity();
    double r36_best_accel_ub=std::numeric_limits<double>::infinity();
    double r36_best_jerk_ub=std::numeric_limits<double>::infinity();

    // AF7R37 full-support C7 spatial-path diagnostics.
    int r37_side_assignments=0;
    int r37_spatial_qp_solutions=0;
    int r37_exact_c7_candidates=0;
    int r37_heading_pass_count=0;
    int r37_knot_count=0;
    double r37_support_start_s=std::numeric_limits<double>::quiet_NaN();
    double r37_actual_rejoin_s=std::numeric_limits<double>::quiet_NaN();
    double r37_best_total_velocity_ub=std::numeric_limits<double>::infinity();
    double r37_best_accel_ub=std::numeric_limits<double>::infinity();
    double r37_best_jerk_ub=std::numeric_limits<double>::infinity();

    int r38_constructive_c7_count=0;
    int r38_legacy_float_checker_pass_count=0;
    int r38_legacy_float_false_reject_count=0;

    // AF7R45 tangent-support corridor diagnostics. Diagnostic only.
    // The R45 corridor rotates the exact ellipsoid support plane inside the
    // nominal-progress / selected-lateral plane and minimizes the required
    // lateral translation on each dt cell. These fields never grant safety.
    int r45_tangent_interval_count=0;
    int r45_zero_offset_interval_count=0;
    int r45_convex_search_fallback_count=0;
    double r45_best_fixed_peak_req_m=std::numeric_limits<double>::infinity();
    double r45_best_tangent_peak_req_m=std::numeric_limits<double>::infinity();
    double r45_max_peak_relief_m=0.0;

    // AF7_TOWING_DIAG_V1 -- diagnostic-only; never grants acceptance.
    double finite_sensing_limit=std::numeric_limits<double>::quiet_NaN();
    double primary_min_stageA_vq=std::numeric_limits<double>::infinity();
    double primary_min_stageA_sigma=-std::numeric_limits<double>::infinity();
    std::array<double,3> primary_min_stageA_U{{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()}};
    double primary_min_envelope=std::numeric_limits<double>::infinity();
    int primary_stageA_certified_count=0;
    int primary_stageB_certified_count=0;
    int primary_stageB_finite_witness_count=0;
    int primary_stageB_repr_fail_count=0;

    // R35 adaptive rejoin diagnostics. `search_end` is only the finite
    // source-derived validity bound; `selected_t1` is the actual certified
    // end of the current correction group.
    double hold_end=0;
    double search_end=std::numeric_limits<double>::quiet_NaN();
    double selected_t1=std::numeric_limits<double>::quiet_NaN();
    double rejoin_seed_duration=std::numeric_limits<double>::quiet_NaN();
    double rejoin_selected_duration=std::numeric_limits<double>::quiet_NaN();
    double rejoin_next_gap=std::numeric_limits<double>::quiet_NaN();
    int rejoin_attempts=0;
    int rejoin_safe_trials=0;
    int rejoin_scope_components=0;
    int rejoin_merged_components=0;
    int search_speed_pass_total=0;
    int search_rejoin_pass_total=0;
    int search_geom_tests_total=0;
    bool spatial_envelope_priority=true;

    struct Tim {
        double episode=0,hessian=0,corridor=0,qp=0,candidate_build=0,
               synthesis=0,candidate=0,rejoin_search=0,total=0;
    } tim;
};

// ------------------------ episode cover ------------------------------------
static inline bool nominal_cell_safe(const ActiveReference&ref,const Live&L,
                                     const SpeedCert&base,const std::vector<Obstacle>&obs,
                                     double ta,double tb){
    double tc=0.5*(ta+tb),h=0.5*(tb-ta);
    if(!(fin(tc)&&fin(h)&&h>0))return false;
    auto x=ref.eval(tc);Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};double ns=norm(sf);
    if(!(fin(ns)&&ns>0))return false;
    Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;
    double VC=std::max(base.VL,base.VQ);
    double rhoL=L.rL+L.dmargin+base.VL*h;
    double rhoQ=L.rQ+L.dmargin+base.VQ*h;
    double rhoC=L.rC+L.dmargin+VC*h;
    for(const auto&o:obs){
        double m; if(!point_margin(pL,o,rhoL,m))return false;
        if(!point_margin(pQ,o,rhoQ,m))return false;
        if(!segment_margin(pQ,pL,o,rhoC,m))return false;
    }
    return true;
}
static inline EpisodeCert build_episode(const ActiveReference&ref,const Live&L,const SpeedCert&base,
                                        const std::vector<Obstacle>&obs,double t0,double searchEnd){
    EpisodeCert E;
    if(!(fin(t0)&&fin(searchEnd)&&searchEnd>t0&&L.dt>0&&!obs.empty()))return E;
    double tolMerge=64*epsd()*std::max({1.0,std::fabs(t0),std::fabs(searchEnd),std::fabs(L.dt)});
    std::vector<Episode> raw;
    for(size_t oi=0;oi<obs.size();++oi){
        std::vector<std::pair<double,double>> stack{{t0,searchEnd}},unsafe;
        while(!stack.empty()){
            auto [ta,tb]=stack.back();stack.pop_back();
            if(!(fin(ta)&&fin(tb)&&tb>ta))continue;
            double tc=0.5*(ta+tb),h=0.5*(tb-ta);
            bool splittable=fin(tc)&&tc>ta&&tc<tb;
            ++E.leaves;
            bool safe=false;
            if(splittable){
                auto x=ref.eval(tc);Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};double ns=norm(sf);
                if(fin(ns)&&ns>0){
                    Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;
                    double VC=std::max(base.VL,base.VQ);
                    double mL,mQ,mC;
                    bool a=point_margin(pL,obs[oi],L.rL+L.dmargin+L.tracking+base.VL*h,mL);
                    bool b=point_margin(pQ,obs[oi],L.rQ+L.dmargin+L.tracking+base.VQ*h,mQ);
                    bool c=segment_margin(pQ,pL,obs[oi],L.rC+L.dmargin+L.tracking+VC*h,mC);
                    safe=a&&b&&c;
                }
            }
            if(safe){++E.safe_leaves;continue;}
            if((tb-ta)<=L.dt+tolMerge || !splittable){
                unsafe.push_back({ta,tb});++E.unsafe_leaves;if(!splittable)++E.repr_fail;continue;
            }
            stack.push_back({tc,tb});stack.push_back({ta,tc});++E.splits;
        }
        if(unsafe.empty())continue;
        std::sort(unsafe.begin(),unsafe.end());
        double s=unsafe[0].first,e=unsafe[0].second;
        for(size_t k=1;k<unsafe.size();++k){
            if(unsafe[k].first<=e+tolMerge)e=std::max(e,unsafe[k].second);
            else{raw.push_back({(int)oi,0,s,e});s=unsafe[k].first;e=unsafe[k].second;}
        }
        raw.push_back({(int)oi,0,s,e});
    }
    if(raw.empty()){E.ok=true;return E;}
    std::sort(raw.begin(),raw.end(),[](const Episode&a,const Episode&b){return a.start<b.start;});
    double tolT=64*epsd()*std::max({1.0,std::fabs(t0),std::fabs(searchEnd)});
    for(size_t i=0;i<raw.size();++i){
        if(E.comp.empty()||raw[i].start>E.comp.back().end+tolT){
            Component c;c.start=raw[i].start;c.end=raw[i].end;c.episodes.push_back((int)i);E.comp.push_back(c);
        }else{
            E.comp.back().end=std::max(E.comp.back().end,raw[i].end);E.comp.back().episodes.push_back((int)i);
        }
        raw[i].comp=(int)E.comp.size()-1;
    }
    E.ep=std::move(raw);E.ok=true;return E;
}

// ----------------------- corridor normal/hull -------------------------------
static inline Vec3 canonical_sign(Vec3 n){
    double sc=std::max(1.0,maxabs(n));
    double g=128*epsd()*sc;
    double v[3]={n.x,n.y,n.z};
    for(int k=0;k<3;++k){
        if(std::fabs(v[k])>g){
            if(v[k]<0)n=n*(-1.0);
            return n;
        }
    }
    return n;
}
static inline Vec3 lateral_normal(const Mat3&R,Vec3 axes,Vec3 progress){
    // AF7R36 canonical separator.
    //
    // For nonzero progress v, solve the constrained principal-eigenvector
    // problem in a deterministic orthonormal basis of v^perp.  The 2x2
    // symmetric eigenproblem is analytic.  If its top eigenvalue is
    // binary64-indistinguishable from the second eigenvalue, the metric
    // objective has no unique lateral direction; select b1, a deterministic
    // frame tie-break.  This changes synthesis selection only.  The
    // independent continuous whole-body certificate remains the authority.
    Mat3 D{};D(0,0)=1/(axes.x*axes.x);D(1,1)=1/(axes.y*axes.y);D(2,2)=1/(axes.z*axes.z);
    Mat3 Q=mmul(mmul(R,D),transpose(R));Q=(Q+transpose(Q))*0.5;
    double np=norm(progress);
    if(finite(progress)&&np>128*epsd()*std::max(1.0,np)){
        Vec3 v=progress/np;
        double av[3]={std::fabs(v.x),std::fabs(v.y),std::fabs(v.z)};
        int k=0;if(av[1]<av[k])k=1;if(av[2]<av[k])k=2;
        Vec3 e = k==0?Vec3{1,0,0}:(k==1?Vec3{0,1,0}:Vec3{0,0,1});
        Vec3 b1=e-v*dot(v,e);
        double nb1=norm(b1);
        if(!(fin(nb1)&&nb1>0))throw std::runtime_error("canonical lateral basis failed");
        b1=b1/nb1;
        Vec3 b2=cross(v,b1);
        double nb2=norm(b2);
        if(!(fin(nb2)&&nb2>0))throw std::runtime_error("canonical lateral basis cross failed");
        b2=b2/nb2;

        double a11=dot(b1,mul(Q,b1));
        double a12=dot(b1,mul(Q,b2));
        double a22=dot(b2,mul(Q,b2));
        double delta=0.5*(a11-a22);
        double rad=std::hypot(delta,a12);
        double scale=std::max({1.0,std::fabs(a11),std::fabs(a12),std::fabs(a22),frob(Q)});
        double guard=4096*epsd()*scale;

        Vec3 n;
        if(rad<=guard){
            n=b1;
        }else{
            double theta=0.5*std::atan2(a12,delta);
            n=b1*std::cos(theta)+b2*std::sin(theta);
            n=normalized(n);
        }
        return canonical_sign(n);
    }

    // Zero-progress case: Q eigenvalues are exactly the ellipsoid metric
    // axis values 1/a_i^2 with eigenvectors R(:,i).  Build the numerically
    // top eigenspace projector from the supplied obstacle representation and
    // select the first canonical world axis with nonzero projection.
    double lam[3]={1/(axes.x*axes.x),1/(axes.y*axes.y),1/(axes.z*axes.z)};
    double lmax=std::max({lam[0],lam[1],lam[2]});
    double scale=std::max({1.0,std::fabs(lam[0]),std::fabs(lam[1]),std::fabs(lam[2])});
    double guard=4096*epsd()*scale;
    Mat3 E{};
    for(int j=0;j<3;++j){
        if(std::fabs(lam[j]-lmax)<=guard){
            Vec3 r{R(0,j),R(1,j),R(2,j)};
            E(0,0)+=r.x*r.x;E(0,1)+=r.x*r.y;E(0,2)+=r.x*r.z;
            E(1,0)+=r.y*r.x;E(1,1)+=r.y*r.y;E(1,2)+=r.y*r.z;
            E(2,0)+=r.z*r.x;E(2,1)+=r.z*r.y;E(2,2)+=r.z*r.z;
        }
    }
    Vec3 axesWorld[3]={{1,0,0},{0,1,0},{0,0,1}};
    for(auto e:axesWorld){
        Vec3 n=mul(E,e);
        double nn=norm(n);
        if(fin(nn)&&nn>128*epsd()){
            return canonical_sign(n/nn);
        }
    }
    throw std::runtime_error("zero-progress canonical top eigenspace selection failed");
}
struct Row {std::vector<double>a;double b=0;};
static inline std::vector<std::pair<double,double>> upper_hull(std::vector<std::pair<double,double>> sb){
    std::sort(sb.begin(),sb.end(),[](auto&a,auto&b){return a.first<b.first;});
    std::vector<std::pair<double,double>> u;
    for(size_t k=0;k<sb.size();){
        double s=sb[k].first,b=sb[k].second;size_t j=k+1;
        while(j<sb.size()&&sb[j].first==s){b=std::max(b,sb[j].second);++j;}
        u.push_back({s,b});k=j;
    }
    std::vector<std::pair<double,double>> h;
    for(auto p:u){
        while(h.size()>=2){
            auto p1=h[h.size()-2],p2=h[h.size()-1],p3=p;
            double dx1=p2.first-p1.first,dy1=p2.second-p1.second;
            double dx2=p3.first-p2.first,dy2=p3.second-p2.second;
            double t1=dx1*dy2,t2=dy1*dx2,cr=t1-t2;
            double sc=std::max({1.0,std::fabs(t1),std::fabs(t2),std::fabs(dx1),std::fabs(dx2),std::fabs(dy1),std::fabs(dy2)});
            double guard=128*epsd()*sc;
            if(cr>guard||cr==0)h.pop_back();else break;
        }
        h.push_back(p);
    }
    return h;
}
static inline std::vector<Row> corridor_rows(const ActiveReference&ref,const Live&L,const SpeedCert&base,
                                             const std::vector<Obstacle>&obs,const EpisodeCert&E,
                                             int ei,Vec3 n,int nvar){
    const auto& ep=E.ep[ei];int jj=ep.comp;double tComp=E.comp[jj].start;
    n=normalized(n);const auto&o=obs[ep.obs];
    Vec3 rn=tmul(o.R,n), ar{o.axes.x*rn.x,o.axes.y*rn.y,o.axes.z*rn.z};
    double hObs=dot(n,o.c)+norm(ar),VC=std::max(base.VL,base.VQ);
    std::vector<std::pair<double,double>> sb;
    double ta=ep.start,te=ep.end;
    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        double tb=std::min(te,ta+L.dt),tc=0.5*(ta+tb),half=0.5*(tb-ta);
        auto x=ref.eval(tc);Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};double ns=norm(sf);
        if(!(fin(ns)&&ns>0))throw std::runtime_error("corridor flatness singular");
        Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;
        double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        double rhoC=L.rC+L.dmargin+L.tracking+VC*half;
        double req=std::max({hObs+rhoL-dot(n,pL),hObs+rhoQ-dot(n,pQ),
                            hObs+rhoC-std::min(dot(n,pQ),dot(n,pL))});
        req+=64*epsd()*std::max(1.0,std::fabs(req));
        sb.push_back({ta-tComp,req});sb.push_back({tb-tComp,req});ta=tb;
    }
    auto h=upper_hull(sb);std::vector<Row> rows;
    int pslot=2*jj,vslot=2*jj+1;
    for(auto [s,b]:h){
        Row R;R.a.assign(nvar,0);R.b=b;
        R.a[3*pslot+0]=n.x;R.a[3*pslot+1]=n.y;R.a[3*pslot+2]=n.z;
        R.a[3*vslot+0]=s*n.x;R.a[3*vslot+1]=s*n.y;R.a[3*vslot+2]=s*n.z;
        rows.push_back(std::move(R));
    }
    return rows;
}

// ------------------------ minimum snap Hessian ------------------------------
static inline double choose_int(int n,int k){
    if(k<0||k>n)return 0;if(k>n-k)k=n-k;double v=1;for(int i=1;i<=k;++i)v*=double(n-k+i)/i;return v;
}
static inline double gram(int n,int i,int j){
    return choose_int(n,i)*choose_int(n,j)/((2*n+1)*choose_int(2*n,i+j));
}
static inline std::vector<double> diff_scalar(std::vector<double> p,int r){
    for(int q=0;q<r;++q){std::vector<double>d(p.size()-1);for(size_t i=0;i+1<p.size();++i)d[i]=p[i+1]-p[i];p.swap(d);}return p;
}
static inline void scalar_endpoint_maps(const std::vector<Component>&comp,int j0,int j1,int mode,int nslot,
                                        std::vector<std::array<double,8>>&J0,std::vector<std::array<double,8>>&J1){
    J0.assign(nslot,{});J1.assign(nslot,{});
    if(mode==0){ // start->entry j1 (1-based component)
        int ip=2*j1,iv=2*j1+1;J1[ip][0]=1;J1[iv][1]=1;
    }else if(mode==1){
        double D0=comp[j0].end-comp[j0].start;int ip0=2*j0,iv0=2*j0+1,ip1=2*j1,iv1=2*j1+1;
        J0[ip0][0]=1;J0[iv0][0]=D0;J0[iv0][1]=1;J1[ip1][0]=1;J1[iv1][1]=1;
    }else{
        double D0=comp[j0].end-comp[j0].start;int ip=2*j0,iv=2*j0+1;
        J0[ip][0]=1;J0[iv][0]=D0;J0[iv][1]=1;
    }
}
static inline void add_transition_hessian(std::vector<double>&Hs,int nslot,
                                          const std::vector<std::array<double,8>>&J0,
                                          const std::vector<std::array<double,8>>&J1,double T){
    if(!(fin(T)&&T>0))throw std::runtime_error("nonpositive transition time");
    std::vector<std::vector<double>> B(nslot);
    double ff=15.0*14*13*12;
    for(int k=0;k<nslot;++k){
        auto P=scalar_control_from_jets(J0[k],J1[k],T);
        auto d=diff_scalar(P,4);for(double&v:d)v*=ff/std::pow(T,4);B[k]=std::move(d);
    }
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j){
        double v=0;for(int a=0;a<12;++a)for(int b=0;b<12;++b)v+=B[i][a]*gram(11,a,b)*B[j][b];
        Hs[i*nslot+j]+=T*v;
    }
}
static inline std::vector<double> minimum_snap_hessian(const std::vector<Component>&comp,double t0,double t1){
    int C=(int)comp.size(),nslot=2*C;std::vector<double>Hs(nslot*nslot,0);
    std::vector<std::array<double,8>>J0,J1;
    scalar_endpoint_maps(comp,0,0,0,nslot,J0,J1);add_transition_hessian(Hs,nslot,J0,J1,comp[0].start-t0);
    for(int j=0;j<C-1;++j){scalar_endpoint_maps(comp,j,j+1,1,nslot,J0,J1);add_transition_hessian(Hs,nslot,J0,J1,comp[j+1].start-comp[j].end);}
    scalar_endpoint_maps(comp,C-1,0,2,nslot,J0,J1);add_transition_hessian(Hs,nslot,J0,J1,t1-comp.back().end);
    int n=3*nslot;std::vector<double>H(n*n,0);
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j)for(int d=0;d<3;++d)H[(3*i+d)*n+(3*j+d)]=Hs[i*nslot+j];
    return H;
}

// ---------------- exact nominal-deviation Hessian --------------------------
// Primary spatial objective used by R36. For the complete correction d(t),
//
//      J_dev = integral_{t0}^{t1} ||d(t)||^2 dt,
//
// is computed EXACTLY for every degree-15 transition and every affine unsafe
// component.  Since all control points / affine coefficients are linear in z,
// J_dev is a positive quadratic form.  No smoothness/deviation weighting is
// introduced: C7 continuity is structural and speed/accel/jerk + whole-body
// safety remain unchanged hard certificates.  This objective only decides
// which safe-side synthesis is attempted first.
static inline void add_transition_deviation_hessian(std::vector<double>&Hs,int nslot,
                                          const std::vector<std::array<double,8>>&J0,
                                          const std::vector<std::array<double,8>>&J1,double T){
    if(!(fin(T)&&T>0))throw std::runtime_error("nonpositive transition time");
    std::vector<std::vector<double>> P(nslot);
    for(int k=0;k<nslot;++k) P[k]=scalar_control_from_jets(J0[k],J1[k],T);
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j){
        double v=0;
        for(int a=0;a<16;++a)for(int b=0;b<16;++b)
            v+=P[i][a]*gram(15,a,b)*P[j][b];
        // solve_qp minimizes 0.5*z'*H*z. Using H=T*Gram gives exactly
        // one-half of the physical L2 integral, an irrelevant common scale.
        Hs[i*nslot+j]+=T*v;
    }
}
static inline std::vector<double> minimum_deviation_hessian(const std::vector<Component>&comp,double t0,double t1){
    int C=(int)comp.size(),nslot=2*C;
    std::vector<double>Hs(nslot*nslot,0.0);
    std::vector<std::array<double,8>>J0,J1;

    // Exact degree-15 safe-transition deviation integrals.
    scalar_endpoint_maps(comp,0,0,0,nslot,J0,J1);
    add_transition_deviation_hessian(Hs,nslot,J0,J1,comp[0].start-t0);
    for(int j=0;j<C-1;++j){
        scalar_endpoint_maps(comp,j,j+1,1,nslot,J0,J1);
        add_transition_deviation_hessian(Hs,nslot,J0,J1,comp[j+1].start-comp[j].end);
    }
    scalar_endpoint_maps(comp,C-1,0,2,nslot,J0,J1);
    add_transition_deviation_hessian(Hs,nslot,J0,J1,t1-comp.back().end);

    // Exact unsafe affine integral for d(s)=a+v*s, s in [0,D]:
    // integral d^2 ds = D*a^2 + D^2*a*v + D^3/3*v^2.
    // H below represents one-half of that integral under 0.5*z'*H*z.
    for(int j=0;j<C;++j){
        double D=comp[j].end-comp[j].start;
        if(!(fin(D)&&D>0))throw std::runtime_error("nonpositive unsafe component duration");
        int ip=2*j,iv=2*j+1;
        Hs[ip*nslot+ip]+=D;
        Hs[ip*nslot+iv]+=0.5*D*D;
        Hs[iv*nslot+ip]+=0.5*D*D;
        Hs[iv*nslot+iv]+=(D*D*D)/3.0;
    }

    int n=3*nslot;std::vector<double>H(n*n,0.0);
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j)for(int d=0;d<3;++d)
        H[(3*i+d)*n+(3*j+d)]=Hs[i*nslot+j];
    return H;
}

// AF7_TOWING_NATURAL_TIME_C3_R5
// Exact derivative-energy Hessian for the complete correction.
static inline void add_transition_derivative_hessian(std::vector<double>&Hs,int nslot,
                                          const std::vector<std::array<double,8>>&J0,
                                          const std::vector<std::array<double,8>>&J1,
                                          double T,int r,double weight){
    if(!(fin(T)&&T>0&&fin(weight)&&weight>=0&&r>=1&&r<=6))
        throw std::runtime_error("invalid towing derivative Hessian request");
    std::vector<std::vector<double>> B(nslot);
    double ff=1.0;for(int k=0;k<r;++k)ff*=double(15-k);
    for(int k=0;k<nslot;++k){
        auto P=scalar_control_from_jets(J0[k],J1[k],T);
        auto d=diff_scalar(P,r);
        double sc=ff/std::pow(T,r);
        for(double&v:d)v*=sc;
        B[k]=std::move(d);
    }
    const int deg=15-r;
    const int ncp=16-r;
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j){
        double v=0.0;
        for(int a=0;a<ncp;++a)for(int b=0;b<ncp;++b)
            v+=B[i][a]*gram(deg,a,b)*B[j][b];
        Hs[i*nslot+j]+=weight*T*v;
    }
}

static inline std::vector<double> towing_natural_time_c3_hessian(
        const std::vector<Component>&comp,double t0,double t1,double cableL,double gravity){
    if(!(fin(cableL)&&cableL>0&&fin(gravity)&&gravity>0))
        throw std::runtime_error("invalid towing natural-time parameters");
    int C=(int)comp.size(),nslot=2*C;
    std::vector<double>Hs(nslot*nslot,0.0);
    std::vector<std::array<double,8>>J0,J1;
    const double tau2=cableL/gravity;
    const double w0=1.0/tau2;
    const double w1=1.0;
    const double w2=tau2;
    const double w3=tau2*tau2;

    auto add_all=[&](double T){
        std::vector<double> Hp(nslot*nslot,0.0);
        add_transition_deviation_hessian(Hp,nslot,J0,J1,T);
        for(int i=0;i<nslot*nslot;++i)Hs[i]+=w0*Hp[i];
        add_transition_derivative_hessian(Hs,nslot,J0,J1,T,1,w1);
        add_transition_derivative_hessian(Hs,nslot,J0,J1,T,2,w2);
        add_transition_derivative_hessian(Hs,nslot,J0,J1,T,3,w3);
    };

    scalar_endpoint_maps(comp,0,0,0,nslot,J0,J1);
    add_all(comp[0].start-t0);
    for(int j=0;j<C-1;++j){
        scalar_endpoint_maps(comp,j,j+1,1,nslot,J0,J1);
        add_all(comp[j+1].start-comp[j].end);
    }
    scalar_endpoint_maps(comp,C-1,0,2,nslot,J0,J1);
    add_all(t1-comp.back().end);

    // Unsafe affine span d(s)=a+v*s:
    // position term is exact; velocity term is D*||v||^2;
    // acceleration and jerk are identically zero on the affine span.
    for(int j=0;j<C;++j){
        double D=comp[j].end-comp[j].start;
        if(!(fin(D)&&D>0))throw std::runtime_error("nonpositive unsafe component duration");
        int ip=2*j,iv=2*j+1;
        Hs[ip*nslot+ip]+=w0*D;
        Hs[ip*nslot+iv]+=w0*0.5*D*D;
        Hs[iv*nslot+ip]+=w0*0.5*D*D;
        Hs[iv*nslot+iv]+=w0*(D*D*D)/3.0;
        Hs[iv*nslot+iv]+=w1*D;
    }

    int n=3*nslot;std::vector<double>H(n*n,0.0);
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j)for(int d=0;d<3;++d)
        H[(3*i+d)*n+(3*j+d)]=Hs[i*nslot+j];
    return H;
}

// ------------------------- exact convex QP ----------------------------------
static inline bool chol(const std::vector<double>&H,int n,std::vector<double>&L){
    L.assign(n*n,0);double sc=1;for(double v:H)sc=std::max(sc,std::fabs(v));double tol=1024*epsd()*sc;
    for(int i=0;i<n;++i)for(int j=0;j<=i;++j){
        double s=H[i*n+j];for(int k=0;k<j;++k)s-=L[i*n+k]*L[j*n+k];
        if(i==j){if(!(fin(s)&&s>tol))return false;L[i*n+j]=std::sqrt(s);}
        else L[i*n+j]=s/L[j*n+j];
    }return true;
}
static inline std::vector<double> cholsolve(const std::vector<double>&L,int n,const std::vector<double>&b){
    std::vector<double>y(n),x(n);
    for(int i=0;i<n;++i){double s=b[i];for(int k=0;k<i;++k)s-=L[i*n+k]*y[k];y[i]=s/L[i*n+i];}
    for(int i=n-1;i>=0;--i){double s=y[i];for(int k=i+1;k<n;++k)s-=L[k*n+i]*x[k];x[i]=s/L[i*n+i];}
    return x;
}
static inline bool dense_solve(std::vector<double>M,std::vector<double>b,int n,std::vector<double>&x){
    double sc=1;for(double v:M)sc=std::max(sc,std::fabs(v));double tol=2048*epsd()*sc;
    for(int k=0;k<n;++k){int p=k;double pv=std::fabs(M[k*n+k]);for(int i=k+1;i<n;++i)if(std::fabs(M[i*n+k])>pv){pv=std::fabs(M[i*n+k]);p=i;}
        if(!(fin(pv)&&pv>tol))return false;if(p!=k){for(int j=k;j<n;++j)std::swap(M[k*n+j],M[p*n+j]);std::swap(b[k],b[p]);}
        for(int i=k+1;i<n;++i){double f=M[i*n+k]/M[k*n+k];for(int j=k+1;j<n;++j)M[i*n+j]-=f*M[k*n+j];b[i]-=f*b[k];}
    }
    x.assign(n,0);for(int i=n-1;i>=0;--i){double s=b[i];for(int j=i+1;j<n;++j)s-=M[i*n+j]*x[j];x[i]=s/M[i*n+i];}
    for(double v:x)if(!fin(v))return false;return true;
}
struct QPSol {bool ok=false;std::vector<double>z;double cost=0,slack=0;};
static inline QPSol solve_qp(const std::vector<double>&H,int n,const std::vector<Row>&rows){
    QPSol best;int m=(int)rows.size();if(m<1)return best;std::vector<double>Lc;if(!chol(H,n,Lc))return best;
    auto feasible=[&](const std::vector<double>&z,double&mins){
        mins=std::numeric_limits<double>::infinity();
        for(const auto&r:rows){double az=0;for(int j=0;j<n;++j)az+=r.a[j]*z[j];double sl=az-r.b;
            double tol=512*epsd()*std::max({1.0,std::fabs(az),std::fabs(r.b)});if(sl<-tol)return false;mins=std::min(mins,sl);}return true;};
    std::vector<double>z0(n,0);double sl0;if(feasible(z0,sl0)){best={true,z0,0,sl0};return best;}
    std::vector<std::vector<double>>X(m,std::vector<double>(n));std::vector<double>G(m*m,0);
    for(int i=0;i<m;++i)X[i]=cholsolve(Lc,n,rows[i].a);
    for(int i=0;i<m;++i)for(int j=0;j<m;++j){double s=0;for(int k=0;k<n;++k)s+=rows[i].a[k]*X[j][k];G[i*m+j]=s;}
    std::vector<int>S;
    std::function<void(int,int,int)> rec=[&](int start,int need,int kt){
        if(need==0){
            int k=kt;std::vector<double>M(k*k),b(k);
            for(int i=0;i<k;++i){b[i]=rows[S[i]].b;for(int j=0;j<k;++j)M[i*k+j]=G[S[i]*m+S[j]];}
            std::vector<double>lam;if(!dense_solve(M,b,k,lam))return;
            double lsc=1;for(double v:lam)lsc=std::max(lsc,std::fabs(v));for(double v:b)lsc=std::max(lsc,std::fabs(v));
            double tl=512*epsd()*lsc;for(double v:lam)if(v<-tl)return;
            std::vector<double>z(n,0);for(int q=0;q<k;++q)for(int j=0;j<n;++j)z[j]+=X[S[q]][j]*lam[q];
            double mins;if(!feasible(z,mins))return;
            double c=0;for(int i=0;i<n;++i)for(int j=0;j<n;++j)c+=0.5*z[i]*H[i*n+j]*z[j];
            if(!fin(c))return;double to=512*epsd()*std::max({1.0,std::fabs(c),best.ok?std::fabs(best.cost):1.0});
            if(!best.ok||c<best.cost-to)best={true,z,c,mins};
            return;
        }
        for(int i=start;i<=m-need;++i){S.push_back(i);rec(i+1,need-1,kt);S.pop_back();}
    };
    for(int k=1;k<=std::min(n,m);++k)rec(0,k,k);
    return best;
}


// AF7R45R2: scalable active-set exchange solver for the R45 corridor QP.
// Same strictly-convex primal problem as solve_qp():
//      min 0.5 z'Hz  s.t. A z >= b.
// The historical solve_qp enumerates all active subsets and is appropriate
// only when the corridor has very few rows.  A smooth tangent-support
// corridor can contain O(T/dt) rows, making exhaustive subset enumeration
// combinatorial.  This routine keeps at most n independent working rows,
// enforces KKT sign conditions, and verifies ALL original inequalities before
// returning ok.  Failure is non-authoritative and the caller may fall back to
// the historical fixed corridor.
static inline QPSol solve_qp_active_exchange(
        const std::vector<double>&H,int n,const std::vector<Row>&rows){
    QPSol out;
    const int m=(int)rows.size();
    if(n<=0||m<1)return out;
    for(const auto&r:rows)if((int)r.a.size()!=n||!fin(r.b))return out;

    std::vector<double>Lc;
    if(!chol(H,n,Lc))return out;

    // X_i = H^{-1} a_i.  Cached once; m*n is small even when m is a few
    // hundred, while avoiding repeated triangular solves in the exchange loop.
    std::vector<std::vector<double>>X(m,std::vector<double>(n));
    for(int i=0;i<m;++i){
        X[i]=cholsolve(Lc,n,rows[i].a);
        for(double v:X[i])if(!fin(v))return out;
    }

    std::vector<int>W;
    std::vector<unsigned char>inW(m,0);
    std::vector<double>z(n,0.0),lam;

    auto solveW=[&]()->bool{
        const int k=(int)W.size();
        if(k==0){z.assign(n,0.0);lam.clear();return true;}
        if(k>n)return false;
        std::vector<double>M(k*k,0.0),b(k,0.0);
        for(int i=0;i<k;++i){
            b[i]=rows[W[i]].b;
            for(int j=0;j<k;++j){
                double q=0.0;
                for(int d=0;d<n;++d)q+=rows[W[i]].a[d]*X[W[j]][d];
                M[i*k+j]=q;
            }
        }
        if(!dense_solve(M,b,k,lam))return false;
        z.assign(n,0.0);
        for(int j=0;j<k;++j)
            for(int d=0;d<n;++d)z[d]+=X[W[j]][d]*lam[j];
        for(double v:z)if(!fin(v))return false;
        for(double v:lam)if(!fin(v))return false;
        return true;
    };

    // A finite algorithmic guard only.  Reaching it never grants a candidate;
    // the caller falls back conservatively.  It is not a maneuver parameter.
    const int iterMax=8*m+16*n+64;
    for(int iter=0;iter<iterMax;++iter){
        if(!solveW())return out;

        // Equality-constrained solution is primal-optimal for this working
        // set only when every active multiplier is nonnegative.
        int rem=-1;
        double worstLam=0.0;
        double lscale=1.0;
        for(double v:lam)lscale=std::max(lscale,std::fabs(v));
        const double ltol=2048*epsd()*lscale;
        for(int j=0;j<(int)lam.size();++j){
            if(lam[j]<-ltol && (rem<0||lam[j]<worstLam)){
                rem=j;worstLam=lam[j];
            }
        }
        if(rem>=0){
            inW[W[rem]]=0;
            W.erase(W.begin()+rem);
            continue;
        }

        int add=-1;
        double worstSlack=0.0;
        double minSlack=std::numeric_limits<double>::infinity();
        for(int i=0;i<m;++i){
            double az=0.0,asc=1.0;
            for(int d=0;d<n;++d){az+=rows[i].a[d]*z[d];asc=std::max(asc,std::fabs(rows[i].a[d]*z[d]));}
            const double sl=az-rows[i].b;
            if(!fin(sl))return out;
            minSlack=std::min(minSlack,sl);
            const double ftol=2048*epsd()*std::max({asc,std::fabs(az),std::fabs(rows[i].b),1.0});
            if(sl<-ftol && (add<0||sl<worstSlack)){
                add=i;worstSlack=sl;
            }
        }

        if(add<0){
            double c=0.0;
            for(int i=0;i<n;++i)for(int j=0;j<n;++j)c+=0.5*z[i]*H[i*n+j]*z[j];
            if(!fin(c)||!fin(minSlack))return out;
            out.ok=true;out.z=z;out.cost=c;out.slack=minSlack;
            return out;
        }

        if(inW[add])return out; // numerical inconsistency: fail closed.
        W.push_back(add);inW[add]=1;
        if((int)W.size()>n)return out;
    }
    return out;
}


// AF7_TOWING_C7_CONDENSED_C3_R7B
// Two-piece C7 transition with analytically eliminated midpoint jets.
struct C7R7MidGain {
    double T=0;
    std::vector<double> K; // 8 x nslot, midpoint jet = K*zslot
};
struct C7R7Model {
    std::vector<double> H; // expanded 3D reduced Hessian
    std::vector<C7R7MidGain> gain; // entry, inter..., exit
};

static inline double c7r7_ff(int r){
    double f=1.0;for(int k=0;k<r;++k)f*=double(15-k);return f;
}
static inline std::vector<double> c7r7_derivative_basis(
        const std::array<double,8>&j0,const std::array<double,8>&j1,double T,int r){
    auto P=scalar_control_from_jets(j0,j1,T);
    auto D=diff_scalar(P,r);
    double sc=c7r7_ff(r)/std::pow(T,r);
    for(double&v:D)v*=sc;
    return D;
}

static inline void c7r7_add_segment_quadratic(
        std::vector<double>&Q,int nv,
        const std::vector<std::array<double,8>>&A,
        const std::vector<std::array<double,8>>&B,
        double T,double tau2){
    if(!(fin(T)&&T>0&&fin(tau2)&&tau2>0))
        throw std::runtime_error("R7B invalid transition scale");
    const double w[4]={1.0/tau2,1.0,tau2,tau2*tau2};
    for(int r=0;r<=3;++r){
        const int deg=15-r,ncp=16-r;
        std::vector<std::vector<double>> D(nv);
        for(int k=0;k<nv;++k)D[k]=c7r7_derivative_basis(A[k],B[k],T,r);
        for(int i=0;i<nv;++i)for(int j=0;j<nv;++j){
            double v=0;
            for(int a=0;a<ncp;++a)for(int b=0;b<ncp;++b)
                v+=D[i][a]*gram(deg,a,b)*D[j][b];
            Q[i*nv+j]+=w[r]*T*v;
        }
    }
}

static inline C7R7MidGain c7r7_condense_transition(
        std::vector<double>&Hs,int nslot,
        const std::vector<std::array<double,8>>&J0,
        const std::vector<std::array<double,8>>&J1,
        double T,double tau2){
    if(!(fin(T)&&T>0))throw std::runtime_error("R7B nonpositive transition");
    const int nm=8,nv=nslot+nm;
    std::vector<double>Q(nv*nv,0.0);
    std::vector<std::array<double,8>> A(nv),M(nv),B(nv);
    for(int k=0;k<nslot;++k){A[k]=J0[k];B[k]=J1[k];}
    for(int r=0;r<8;++r){M[nslot+r][r]=1.0;}
    const double Th=0.5*T;
    c7r7_add_segment_quadratic(Q,nv,A,M,Th,tau2);
    c7r7_add_segment_quadratic(Q,nv,M,B,Th,tau2);

    std::vector<double>C(nm*nm,0.0);
    for(int i=0;i<nm;++i)for(int j=0;j<nm;++j)
        C[i*nm+j]=Q[(nslot+i)*nv+(nslot+j)];

    C7R7MidGain G;G.T=T;G.K.assign(nm*nslot,0.0);
    for(int j=0;j<nslot;++j){
        std::vector<double>rhs(nm),x;
        for(int i=0;i<nm;++i)rhs[i]=Q[(nslot+i)*nv+j];
        if(!dense_solve(C,rhs,nm,x))throw std::runtime_error("R7B midpoint Schur solve failed");
        for(int i=0;i<nm;++i)G.K[i*nslot+j]=-x[i];
    }

    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j){
        double v=Q[i*nv+j];
        for(int r=0;r<nm;++r)v+=Q[i*nv+(nslot+r)]*G.K[r*nslot+j];
        Hs[i*nslot+j]+=v;
    }
    return G;
}

static inline C7R7Model c7r7_model(
        const std::vector<Component>&comp,double t0,double t1,double cableL,double gravity){
    if(!(fin(cableL)&&cableL>0&&fin(gravity)&&gravity>0))
        throw std::runtime_error("R7B invalid natural time");
    const double tau2=cableL/gravity;
    int Cn=(int)comp.size(),nslot=2*Cn;
    std::vector<double>Hs(nslot*nslot,0.0);
    std::vector<std::array<double,8>>J0,J1;
    C7R7Model R;

    scalar_endpoint_maps(comp,0,0,0,nslot,J0,J1);
    R.gain.push_back(c7r7_condense_transition(Hs,nslot,J0,J1,comp[0].start-t0,tau2));
    for(int j=0;j<Cn-1;++j){
        scalar_endpoint_maps(comp,j,j+1,1,nslot,J0,J1);
        R.gain.push_back(c7r7_condense_transition(Hs,nslot,J0,J1,comp[j+1].start-comp[j].end,tau2));
    }
    scalar_endpoint_maps(comp,Cn-1,0,2,nslot,J0,J1);
    R.gain.push_back(c7r7_condense_transition(Hs,nslot,J0,J1,t1-comp.back().end,tau2));

    // Unsafe affine span remains exactly the existing proven structure.
    const double w0=1.0/tau2,w1=1.0;
    for(int j=0;j<Cn;++j){
        double D=comp[j].end-comp[j].start;
        if(!(fin(D)&&D>0))throw std::runtime_error("R7B invalid component duration");
        int ip=2*j,iv=2*j+1;
        Hs[ip*nslot+ip]+=w0*D;
        Hs[ip*nslot+iv]+=w0*0.5*D*D;
        Hs[iv*nslot+ip]+=w0*0.5*D*D;
        Hs[iv*nslot+iv]+=w0*(D*D*D)/3.0+w1*D;
    }

    int n=3*nslot;R.H.assign(n*n,0.0);
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j)for(int d=0;d<3;++d)
        R.H[(3*i+d)*n+(3*j+d)]=Hs[i*nslot+j];
    return R;
}

static inline std::array<Vec3,8> c7r7_midjets(
        const C7R7MidGain&G,const std::vector<double>&z,int nslot){
    std::array<Vec3,8>M{};
    for(int r=0;r<8;++r)for(int k=0;k<nslot;++k){
        double a=G.K[r*nslot+k];
        M[r].x+=a*z[3*k+0];M[r].y+=a*z[3*k+1];M[r].z+=a*z[3*k+2];
    }
    return M;
}

static inline void c7r7_append_split_transition(
        Candidate&C,double ta,double tb,
        const std::array<Vec3,8>&A,const std::array<Vec3,8>&B,
        const C7R7MidGain&G,const std::vector<double>&z,int nslot){
    double tm=0.5*(ta+tb),Th=0.5*(tb-ta);
    if(!(fin(Th)&&Th>0))throw std::runtime_error("R7B split transition duration");
    auto M=c7r7_midjets(G,z,nslot);
    C.seg0.push_back(ta);C.seg1.push_back(tm);C.cp.push_back(vector_control_from_jets(A,M,Th));
    C.seg0.push_back(tm);C.seg1.push_back(tb);C.cp.push_back(vector_control_from_jets(M,B,Th));
}

static inline Candidate build_candidate_c7r7(
        const std::vector<Component>&comp,double t0,double t1,
        const std::vector<double>&z,double cost,const std::vector<int>&signs,double minSlack,
        const C7R7Model&Mdl,int family=3){
    Candidate C;C.cost=cost;C.signs=signs;C.minSlack=minSlack;C.family=family;C.constant_translation=false;
    int nc=(int)comp.size(),nslot=2*nc;
    if((int)z.size()!=3*nslot||(int)Mdl.gain.size()!=nc+1)throw std::runtime_error("R7B candidate size");
    C.pos.resize(nc);C.vel.resize(nc);
    for(int j=0;j<nc;++j){
        C.pos[j]={z[3*(2*j)],z[3*(2*j)+1],z[3*(2*j)+2]};
        C.vel[j]={z[3*(2*j+1)],z[3*(2*j+1)+1],z[3*(2*j+1)+2]};
    }
    auto zerojets=[](){std::array<Vec3,8>a{};return a;};
    int gi=0;
    {
        auto A=zerojets(),B=zerojets();B[0]=C.pos[0];B[1]=C.vel[0];
        c7r7_append_split_transition(C,t0,comp[0].start,A,B,Mdl.gain[gi++],z,nslot);
    }
    for(int j=0;j<nc;++j){
        double T=comp[j].end-comp[j].start;Poly p(16);
        for(int k=0;k<16;++k)p[k]=C.pos[j]+C.vel[j]*(T*(double(k)/15.0));
        C.seg0.push_back(comp[j].start);C.seg1.push_back(comp[j].end);C.cp.push_back(std::move(p));
        if(j<nc-1){
            auto A=zerojets(),B=zerojets();
            A[0]=C.pos[j]+C.vel[j]*(comp[j].end-comp[j].start);A[1]=C.vel[j];
            B[0]=C.pos[j+1];B[1]=C.vel[j+1];
            c7r7_append_split_transition(C,comp[j].end,comp[j+1].start,A,B,Mdl.gain[gi++],z,nslot);
        }
    }
    {
        int j=nc-1;auto A=zerojets(),B=zerojets();
        A[0]=C.pos[j]+C.vel[j]*(comp[j].end-comp[j].start);A[1]=C.vel[j];
        c7r7_append_split_transition(C,comp[j].end,t1,A,B,Mdl.gain[gi++],z,nslot);
    }
    C.dub.fill(0);C.max_ctrl=0;
    for(size_t s=0;s<C.cp.size();++s){
        double T=C.seg1[s]-C.seg0[s];Poly D=C.cp[s];int deg=15;
        for(auto v:D)C.max_ctrl=std::max(C.max_ctrl,norm(v));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);for(size_t i=0;i+1<D.size();++i)E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }
    return C;
}

// AF7R38_CONSTRUCTIVE_EXACT_REAL_C7
// Family 6: one compact curved zero-jet bell per disconnected unsafe component.
//
// For component j, W_j=[l_j,p_j,r_j] and amplitude A_j:
//   [l_j,p_j] : degree-15 Bezier [0 x8, A_j x8]
//   [p_j,r_j] : degree-15 Bezier [A_j x8, 0 x8]
//
// Hence the represented binary64 control polygon has exactly identical first/last
// eight control points at every outer/peak join. Interpreted as exact reals,
// derivatives 1..7 are exactly zero at each side, so the correction is C7 with
// ZERO continuity tolerance and NO equal-half-duration requirement.
// The corridor below is synthesis-only. Unchanged Stage-A/B speed, rejoin and
// continuous whole-body certificates remain the sole acceptance authorities.
struct R25BellWin{double l=0,p=0,r=0;};

static inline std::vector<R25BellWin> r25_windows(
        const std::vector<Component>&comp,double t0,double t1){
    const int C=(int)comp.size();
    std::vector<R25BellWin> W(C);
    if(C<1)return W;
    std::vector<double> cut(C+1);
    cut[0]=t0;cut[C]=t1;
    for(int j=1;j<C;++j){
        const double a=comp[j-1].end,b=comp[j].start;
        cut[j]=a+0.5*(b-a);
        if(!(fin(cut[j])&&a<cut[j]&&cut[j]<b))
            throw std::runtime_error("R25F inter-component bell cut not representable");
    }
    for(int j=0;j<C;++j){
        W[j].l=cut[j];
        W[j].p=comp[j].start+0.5*(comp[j].end-comp[j].start);
        W[j].r=cut[j+1];
        if(!(fin(W[j].l)&&fin(W[j].p)&&fin(W[j].r)&&
             W[j].l<comp[j].start&&comp[j].start<W[j].p&&
             W[j].p<comp[j].end&&comp[j].end<W[j].r))
            throw std::runtime_error("R25F invalid/degenerate bell support");
    }
    return W;
}

static inline double r25_casteljau_scalar(std::array<double,16>a,double u){
    u=std::max(0.0,std::min(1.0,u));
    for(int n=15;n>0;--n)
        for(int i=0;i<n;++i)a[i]=(1.0-u)*a[i]+u*a[i+1];
    return a[0];
}

static inline double r25_beta(double t,const R25BellWin&w){
    if(t<=w.l||t>=w.r)return 0.0;
    if(t==w.p)return 1.0;
    std::array<double,16>b{};
    if(t<w.p){
        for(int k=8;k<16;++k)b[k]=1.0;
        return r25_casteljau_scalar(b,(t-w.l)/(w.p-w.l));
    }
    for(int k=0;k<8;++k)b[k]=1.0;
    return r25_casteljau_scalar(b,(t-w.p)/(w.r-w.p));
}

static inline std::vector<Row> r25_bell_rows(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,const EpisodeCert&E,
        const std::vector<R25BellWin>&W,int ei,Vec3 n,int nvar){
    // AF7R25G exact row reduction.
    //
    // Every cell of one episode has IDENTICAL left-hand side n' A_j and only
    // a different scalar right-hand side b_i. Therefore
    //
    //        n' A_j >= b_i  for every cell i
    //
    // is exactly equivalent (not conservative, not approximate) to
    //
    //        n' A_j >= max_i b_i.
    //
    // This removes the previous O(number-of-25ms-cells) redundant QP rows
    // without changing the bell-family feasible set.
    const auto&ep=E.ep[ei];
    const int jj=ep.comp;
    n=normalized(n);
    const auto&o=obs[ep.obs];
    Vec3 rn=tmul(o.R,n),ar{o.axes.x*rn.x,o.axes.y*rn.y,o.axes.z*rn.z};
    const double hObs=dot(n,o.c)+norm(ar);
    const double VC=std::max(base.VL,base.VQ);

    double bmax=-std::numeric_limits<double>::infinity();
    bool any=false;
    double ta=ep.start,te=ep.end;
    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        double tb=std::min(te,ta+L.dt);
        if(ta<W[jj].p&&tb>W[jj].p)tb=W[jj].p;
        if(!(tb>ta))throw std::runtime_error("R25F bell cell not representable");

        const double tc=ta+0.5*(tb-ta),half=0.5*(tb-ta);
        auto x=ref.eval(tc);
        Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};
        double ns=norm(sf);
        if(!(fin(ns)&&ns>0))
            throw std::runtime_error("R25F corridor flatness singular");

        Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;
        const double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        const double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        const double rhoC=L.rC+L.dmargin+L.tracking+VC*half;

        double req=std::max({hObs+rhoL-dot(n,pL),
                             hObs+rhoQ-dot(n,pQ),
                             hObs+rhoC-std::min(dot(n,pQ),dot(n,pL))});
        req+=64*epsd()*std::max(1.0,std::fabs(req));

        double bl=std::min(r25_beta(ta,W[jj]),r25_beta(tb,W[jj]));
        bl-=128*epsd()*std::max(1.0,std::fabs(bl));
        if(!(fin(bl)&&bl>0))
            throw std::runtime_error("R25F bell beta lower bound is not positive");

        double b=req/bl;
        b+=64*epsd()*std::max(1.0,std::fabs(b));
        if(!(fin(b)))throw std::runtime_error("R25F bell row nonfinite");
        bmax=std::max(bmax,b);
        any=true;
        ta=tb;
    }

    if(!any||!fin(bmax))throw std::runtime_error("R25F bell episode row unavailable");

    Row R;
    R.a.assign(nvar,0.0);
    R.b=bmax;
    R.a[3*jj+0]=n.x;
    R.a[3*jj+1]=n.y;
    R.a[3*jj+2]=n.z;
    return std::vector<Row>{std::move(R)};
}

static inline std::vector<double> r25_bell_hessian(
        const std::vector<R25BellWin>&W){
    const int C=(int)W.size(),n=3*C;
    std::vector<double>H(n*n,0.0);
    // Integral_0^1 beta_up(u)^2 du for [0 x8, 1 x8].
    double I=0.0;
    for(int i=8;i<16;++i)
        for(int j=8;j<16;++j)I+=gram(15,i,j);
    if(!(fin(I)&&I>0))
        throw std::runtime_error("R25F bell L2 Gram invalid");
    for(int j=0;j<C;++j){
        // Down half has the same integral by symmetry, so total is (r-l)*I.
        const double w=(W[j].r-W[j].l)*I;
        if(!(fin(w)&&w>0))
            throw std::runtime_error("R25F bell L2 weight invalid");
        for(int d=0;d<3;++d)H[(3*j+d)*n+(3*j+d)]=w;
    }
    return H;
}

static inline Poly r25_bell_segment(Vec3 A,bool up){
    Poly P(16);
    if(up){
        for(int k=0;k<8;++k)P[k]={0,0,0};
        for(int k=8;k<16;++k)P[k]=A;
    }else{
        for(int k=0;k<8;++k)P[k]=A;
        for(int k=8;k<16;++k)P[k]={0,0,0};
    }
    return P;
}

static inline Candidate r25_build_bell(
        const std::vector<Component>&comp,const std::vector<R25BellWin>&W,
        const std::vector<double>&z,double cost,
        const std::vector<int>&signs,double minSlack){
    const int Cn=(int)comp.size();
    if((int)W.size()!=Cn||(int)z.size()!=3*Cn)
        throw std::runtime_error("R25F bell candidate size");
    Candidate C;
    C.cost=cost;C.signs=signs;C.minSlack=minSlack;
    C.family=6;
    C.constant_translation=false;
    C.exact_c7=true;
    C.pos.resize(Cn);
    C.vel.assign(Cn,Vec3{0,0,0});
    for(int j=0;j<Cn;++j){
        Vec3 A{z[3*j],z[3*j+1],z[3*j+2]};
        if(!finite(A))throw std::runtime_error("R25F nonfinite bell amplitude");
        C.pos[j]=A;
        C.seg0.push_back(W[j].l);
        C.seg1.push_back(W[j].p);
        C.cp.push_back(r25_bell_segment(A,true));
        C.seg0.push_back(W[j].p);
        C.seg1.push_back(W[j].r);
        C.cp.push_back(r25_bell_segment(A,false));
    }
    C.dub.fill(0);
    C.max_ctrl=0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        if(!(fin(T)&&T>0))
            throw std::runtime_error("R25F nonpositive bell segment duration");
        Poly D=C.cp[s];
        int deg=(int)D.size()-1;
        for(auto v:D)C.max_ctrl=std::max(C.max_ctrl,norm(v));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);
            --deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }
    return C;
}


// ===========================================================================
// AF7R28_FLOWING_AFFINE_EXACT_C7
//
// Purpose
// -------
// The zero-jet bell reaches a large lateral offset with zero correction
// velocity at its internal peak.  That can require unnecessarily high peak
// speed when the fixed-H trigger gives only ~4 s before nominal collision.
//
// R27 keeps the trigger unchanged.  Instead it preserves the affine
// correction freedom already present in the minimum-deviation QP:
//
//      d(t) = a + v (t - t_component_start)
//
// throughout the nominally unsafe component.  Entry and exit are represented
// by degree-15 Bezier transitions whose internal join has NONZERO first
// derivative and zero derivatives 2..7.
//
// For one connected unsafe component, every segment in the R27 stored
// representation has the SAME binary64 duration h = component.start - t0.
// A common dyadic arithmetic-progression stencil therefore gives exact-real
// equality of stored derivatives 0..7 at every internal join:
//
//      d_left^(r) == d_right^(r),  r=0..7.
//
// No tolerance is used to claim C7.  The stored polygon is the authority.
// The candidate is still independently accepted only by the unchanged
// exact-C7 gate, finite-sensing speed, rejoin, and continuous whole-body
// certificates.
// ===========================================================================

struct R27AxisLattice{
    int e=0;
    long long p=0;
    long long d=0;
};

static inline R27AxisLattice r27_axis_lattice(
        double p,double d,long long kmin,long long kmax){
    if(!(fin(p)&&fin(d)&&kmin<=0&&kmax>=0))
        throw std::runtime_error("R27 invalid lattice request");

    const double reach=std::max(std::fabs((double)kmin*d),
                                std::fabs((double)kmax*d));
    const double scale=std::max({1.0,std::fabs(p),reach});
    const int es=std::ilogb(scale);

    // Keep several integer guard bits for k*d while retaining substantially
    // more precision than is needed by the geometric certificate.
    const int e=es-46;
    const double ps=std::ldexp(p,-e);
    const double ds=std::ldexp(d,-e);
    if(!(fin(ps)&&fin(ds)&&
         std::fabs(ps)<4503599627370496.0&&
         std::fabs(ds)<4503599627370496.0))
        throw std::runtime_error("R27 lattice integer range");

    const long long ip=std::llround(ps);
    const long long id=std::llround(ds);

    const long double q0=(long double)ip+(long double)kmin*(long double)id;
    const long double q1=(long double)ip+(long double)kmax*(long double)id;
    const long double lim=4503599627370496.0L;
    if(std::fabs(q0)>=lim||std::fabs(q1)>=lim)
        throw std::runtime_error("R27 lattice reach overflow");

    return {e,ip,id};
}

static inline double r27_grid(long long q,int e){
    if(q<=-4503599627370496LL||q>=4503599627370496LL)
        throw std::runtime_error("R27 grid overflow");
    const double x=std::ldexp((double)q,e);
    if(!fin(x))throw std::runtime_error("R27 grid nonfinite");
    return x;
}

static inline Vec3 r27_grid_vec(
        const R27AxisLattice&x,const R27AxisLattice&y,
        const R27AxisLattice&z,long long k){
    auto q=[&](const R27AxisLattice&a)->long long{
        const long double qq=(long double)a.p+(long double)k*(long double)a.d;
        if(std::fabs(qq)>=4503599627370496.0L)
            throw std::runtime_error("R27 grid index overflow");
        return a.p+k*a.d;
    };
    return {r27_grid(q(x),x.e),r27_grid(q(y),y.e),r27_grid(q(z),z.e)};
}

static inline std::vector<double> r27_flow_hessian_scalar(int ncoast,double h){
    if(!(ncoast>=1&&fin(h)&&h>0))
        throw std::runtime_error("R27 invalid flow Hessian request");

    // Variables are [P, V], where P is correction position at the unsafe
    // component start and V is the constant correction velocity through the
    // flowing section.  The following control maps are EXACTLY those used by
    // r27_build_flow_single before dyadic quantization.
    double H00=0.0,H01=0.0,H11=0.0;

    auto add_segment=[&](const std::array<double,16>&bp,
                         const std::array<double,16>&bv){
        double a=0,b=0,c=0;
        for(int i=0;i<16;++i)for(int j=0;j<16;++j){
            const double G=gram(15,i,j);
            a+=bp[i]*bp[j]*G;
            b+=bp[i]*bv[j]*G;
            c+=bv[i]*bv[j]*G;
        }
        H00+=h*a;H01+=h*b;H11+=h*c;
    };

    std::array<double,16>bp{},bv{};

    // Entry: first eight points zero; last eight approach P with terminal V.
    bp.fill(0);bv.fill(0);
    for(int k=8;k<16;++k){
        const int idx=k-15; // -7 ... 0
        bp[k]=1.0;
        bv[k]=h*double(idx)/15.0;
    }
    add_segment(bp,bv);

    // Constant-velocity flowing segments.
    for(int m=0;m<ncoast;++m){
        for(int k=0;k<16;++k){
            bp[k]=1.0;
            bv[k]=h*(double(m)+double(k)/15.0);
        }
        add_segment(bp,bv);
    }

    // Exit: preserve incoming V at the start, then return to zero with
    // derivatives 0..7 equal to zero at the outer boundary.
    bp.fill(0);bv.fill(0);
    for(int k=0;k<8;++k){
        bp[k]=1.0;
        bv[k]=h*(double(ncoast)+double(k)/15.0);
    }
    add_segment(bp,bv);

    return {H00,H01,H01,H11};
}

static inline std::vector<double> r27_flow_hessian_3d(int ncoast,double h){
    auto S=r27_flow_hessian_scalar(ncoast,h);
    std::vector<double>H(36,0.0);
    for(int d=0;d<3;++d){
        H[d*6+d]=S[0];
        H[d*6+(3+d)]=S[1];
        H[(3+d)*6+d]=S[2];
        H[(3+d)*6+(3+d)]=S[3];
    }
    return H;
}

static inline std::vector<Row> r27_transform_rows_single(
        const std::vector<Row>&src,double delta){
    std::vector<Row>out;
    out.reserve(src.size());
    for(const auto&r:src){
        if(r.a.size()!=6)throw std::runtime_error("R27 row size");
        Row q;q.a.assign(6,0.0);q.b=r.b;
        for(int d=0;d<3;++d){
            // a = P - V*delta
            q.a[d]=r.a[d];
            q.a[3+d]=r.a[3+d]-delta*r.a[d];
        }
        out.push_back(std::move(q));
    }
    return out;
}

static inline Candidate r27_build_flow_single(
        const Component&comp,double t0,double t1,
        const std::vector<double>&z,double cost,
        const std::vector<int>&signs,double minSlack){
    if(z.size()!=6)throw std::runtime_error("R27 z size");

    // Entry duration is exactly the fixed-H trigger lead realized by the
    // current component start.  The unsafe affine section therefore begins
    // exactly where the corridor variable a+v*s is defined.
    const double h=comp.start-t0;
    if(!(fin(h)&&h>0))throw std::runtime_error("R27 nonpositive entry duration");

    // R27 is intentionally the fixed-H flowing family.  Large deviations from
    // the live H mean this is not the intended trigger episode.
    const double htol=256*epsd()*std::max({1.0,std::fabs(h),4.0});
    if(std::fabs(h-4.0)>std::max(0.25,htol))
        throw std::runtime_error("R27 component start is not a fixed-H episode");

    const double D=comp.end-comp.start;
    if(!(fin(D)&&D>0))throw std::runtime_error("R27 invalid unsafe duration");
    int ncoast=std::max(1,(int)std::ceil(D/h-64*epsd()*std::max(1.0,D/h)));

    // Build the exact-equal-duration time grid.
    std::vector<double>tt;
    tt.reserve((size_t)ncoast+3);
    tt.push_back(t0);
    tt.push_back(comp.start); // exact authoritative first interval
    for(int k=0;k<ncoast+1;++k){
        double nx=tt.back()+h;
        if(!(fin(nx)&&nx>tt.back()&&nx-tt.back()==h))
            throw std::runtime_error("R27 equal-duration binary64 grid unavailable");
        tt.push_back(nx);
    }
    // tt: [t0, comp.start, coast ends..., exit end]
    if((int)tt.size()!=ncoast+3)
        throw std::runtime_error("R27 time-grid size");
    const double supportEnd=tt.back();
    if(!(supportEnd<t1-8*epsd()*std::max(1.0,std::fabs(t1))))
        throw std::runtime_error("R27 insufficient post-obstacle validity");

    Vec3 P{z[0],z[1],z[2]};
    Vec3 V{z[3],z[4],z[5]};
    Vec3 di=V*(h/15.0);

    const long long kmin=-7;
    const long long kmax=15LL*ncoast+7;
    const auto ax=r27_axis_lattice(P.x,di.x,kmin,kmax);
    const auto ay=r27_axis_lattice(P.y,di.y,kmin,kmax);
    const auto az=r27_axis_lattice(P.z,di.z,kmin,kmax);

    Candidate C;
    C.family=7;
    C.constant_translation=false;
    C.exact_c7=true;
    C.cost=cost;
    C.signs=signs;
    C.minSlack=minSlack;

    // Entry.
    Poly entry(16,Vec3{});
    for(int k=8;k<16;++k)entry[k]=r27_grid_vec(ax,ay,az,k-15);
    C.seg0.push_back(tt[0]);C.seg1.push_back(tt[1]);C.cp.push_back(std::move(entry));

    // Flowing affine section.  Each segment has the same exact duration h and
    // continues the same arithmetic-progression control stencil.
    for(int m=0;m<ncoast;++m){
        Poly p(16);
        const long long base=15LL*m;
        for(int k=0;k<16;++k)p[k]=r27_grid_vec(ax,ay,az,base+k);
        C.seg0.push_back(tt[1+m]);
        C.seg1.push_back(tt[2+m]);
        C.cp.push_back(std::move(p));
    }

    // Exit.
    Poly exit(16,Vec3{});
    const long long base=15LL*ncoast;
    for(int k=0;k<8;++k)exit[k]=r27_grid_vec(ax,ay,az,base+k);
    C.seg0.push_back(tt[1+ncoast]);
    C.seg1.push_back(tt[2+ncoast]);
    C.cp.push_back(std::move(exit));

    const Vec3 Pq=r27_grid_vec(ax,ay,az,0);
    const Vec3 dq=r27_grid_vec(
        R27AxisLattice{ax.e,0,ax.d},
        R27AxisLattice{ay.e,0,ay.d},
        R27AxisLattice{az.e,0,az.d},1);
    const Vec3 Vq=dq*(15.0/h);
    C.pos={Pq};C.vel={Vq};

    C.dub.fill(0);C.max_ctrl=0.0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        if(!(fin(T)&&T==h))
            throw std::runtime_error("R27 stored duration mismatch");
        Poly D=C.cp[s];
        int deg=(int)D.size()-1;
        for(auto v:D)C.max_ctrl=std::max(C.max_ctrl,norm(v));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }
    return C;
}



// ===========================================================================
// AF7R28_FLOW_GENERAL_EXACT_C7
//
// R27 family-7 could silently generate zero candidates because it imposed
// several REPRESENTATION restrictions unrelated to the safety theorem:
//   * exactly one merged unsafe component,
//   * component.start - t0 approximately equal to 4 s,
//   * unsafe component boundaries themselves had to lie on the common
//     equal-duration binary64 grid.
//
// R28 removes those restrictions.
//
// A common exact binary64 time step h is selected so that:
//   entry     : [t0, t0+h]
//   flow      : one or more h-length degree-15 affine Bezier segments
//   exit      : one h-length degree-15 segment
//
// The flow starts no later than the first unsafe episode and ends no earlier
// than the last unsafe episode.  Corridor rows are expressed directly in the
// common stored variables [P0,V], where P0 is the correction at flow start.
// Thus arbitrary episode/component boundaries need NOT coincide with a Bezier
// join.  All joins of the stored candidate still have exactly the same h.
//
// Obstacle orientation remains fully general.  Each episode separator is built
// from lateral_normal(R,axes,progress), and exact ellipsoid support uses R.
// There is no world-axis special case.
// ===========================================================================

struct R28FlowGrid{
    bool ok=false;
    int reason=0;
    double base_h=0.0;
    double support_start=0.0;
    double entry_T=0.0;
    double flow_start=0.0;
    double flow_T=0.0;
    double flow_end=0.0;
    double exit_T=0.0;
    double exit_end=0.0;
    double wait=0.0;
    long long Ke=0,Kf=0,Kx=0;
    int nflow=0;
};

static inline double r31_align_up(double x,double h){
    if(!(fin(x)&&fin(h)&&h>0))return std::numeric_limits<double>::quiet_NaN();
    const double q=x/h;
    if(!(fin(q)&&std::fabs(q)<4503599627370496.0))
        return std::numeric_limits<double>::quiet_NaN();
    double k=std::ceil(q),y=k*h;
    if(y<x)y=(k+1.0)*h;
    return y;
}
static inline long long r31_largest_pow2_le(long long n){
    if(n<1)return 0;
    long long p=1;
    while(p<=n/2)p*=2;
    return p;
}
static inline bool r31_exact_time(double a,double T,double&b){
    b=a+T;
    return fin(b)&&b>a&&(b-a)==T;
}

static inline R28FlowGrid r28_select_flow_grid(
        double t0,double firstUnsafe,double lastUnsafe,double t1){
    R28FlowGrid G;
    if(!(fin(t0)&&fin(firstUnsafe)&&fin(lastUnsafe)&&fin(t1)&&
         firstUnsafe>t0&&lastUnsafe>=firstUnsafe&&t1>lastUnsafe)){
        G.reason=1;return G;
    }

    // Exact binary64 representation quantum only; it is NOT a maneuver
    // duration and contains no obstacle-size threshold.
    const double base_h=std::ldexp(1.0,-4); // 0.0625 s
    const double start=r31_align_up(t0,base_h);
    if(!(fin(start)&&start>=t0)){G.reason=2;return G;}

    // Use the largest power-of-two multiple of base_h that fits before the
    // first unsafe instant.  This exploits essentially the full fixed-H lead
    // instead of R30's one-second entry.
    const double entryAvail=firstUnsafe-start;
    const long long eu=(long long)std::floor(
        entryAvail/base_h-
        128*epsd()*std::max(1.0,std::fabs(entryAvail/base_h)));
    const long long Ke=r31_largest_pow2_le(eu);
    if(Ke<1){G.reason=3;return G;}
    const double entryT=double(Ke)*base_h;

    double flowStart=0.0;
    if(!r31_exact_time(start,entryT,flowStart)){G.reason=2;return G;}
    const double tol=256*epsd()*std::max(
        {1.0,std::fabs(flowStart),std::fabs(firstUnsafe),std::fabs(t1)});
    if(flowStart>firstUnsafe-tol){G.reason=3;return G;}

    // One-second affine flow segments are only a storage segmentation.
    long long Kf=16;
    double flowT=double(Kf)*base_h;
    if(!(flowT>0)){G.reason=2;return G;}

    const double raw=(lastUnsafe-flowStart)/flowT;
    int nflow=std::max(1,(int)std::ceil(
        raw-128*epsd()*std::max(1.0,std::fabs(raw))));
    double flowEnd=flowStart;
    for(int m=0;m<nflow;++m){
        double nx=0.0;
        if(!r31_exact_time(flowEnd,flowT,nx)){G.reason=2;return G;}
        flowEnd=nx;
    }
    if(flowEnd<lastUnsafe-tol){G.reason=2;return G;}

    // Use the available post-obstacle tail instead of forcing a one-second
    // return.  This is what prevents the exit segment from demanding a large
    // artificial catch-up speed.
    const double exitAvail=t1-flowEnd;
    const long long xu=(long long)std::floor(
        exitAvail/base_h-
        128*epsd()*std::max(1.0,std::fabs(exitAvail/base_h)));
    const long long Kx=r31_largest_pow2_le(xu);
    if(Kx<1){G.reason=3;return G;}
    const double exitT=double(Kx)*base_h;
    double exitEnd=0.0;
    if(!r31_exact_time(flowEnd,exitT,exitEnd)){G.reason=2;return G;}
    if(!(exitEnd<t1-tol)){G.reason=3;return G;}

    G.ok=true;G.reason=0;G.base_h=base_h;
    G.support_start=start;G.entry_T=entryT;G.flow_start=flowStart;
    G.flow_T=flowT;G.flow_end=flowEnd;G.exit_T=exitT;G.exit_end=exitEnd;
    G.wait=start-t0;G.Ke=Ke;G.Kf=Kf;G.Kx=Kx;G.nflow=nflow;
    return G;
}

static inline std::vector<Row> r28_flow_corridor_rows(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,const EpisodeCert&E,
        int ei,Vec3 n,double flowStart){
    const auto&ep=E.ep[ei];
    n=normalized(n);
    const auto&o=obs[ep.obs];

    Vec3 rn=tmul(o.R,n);
    Vec3 ar{o.axes.x*rn.x,o.axes.y*rn.y,o.axes.z*rn.z};
    const double hObs=dot(n,o.c)+norm(ar);
    const double VC=std::max(base.VL,base.VQ);

    std::vector<std::pair<double,double>> sb;
    double ta=ep.start,te=ep.end;
    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        const double tb=std::min(te,ta+L.dt);
        const double tc=0.5*(ta+tb);
        const double half=0.5*(tb-ta);
        auto x=ref.eval(tc);
        Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};
        const double ns=norm(sf);
        if(!(fin(ns)&&ns>0))
            throw std::runtime_error("R28 corridor flatness singular");
        Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;

        const double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        const double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        const double rhoC=L.rC+L.dmargin+L.tracking+VC*half;

        double req=std::max({
            hObs+rhoL-dot(n,pL),
            hObs+rhoQ-dot(n,pQ),
            hObs+rhoC-std::min(dot(n,pQ),dot(n,pL))
        });
        req+=64*epsd()*std::max(1.0,std::fabs(req));

        // Stored affine correction during the flow:
        //   d(t) = P0 + V*(t-flowStart).
        sb.push_back({ta-flowStart,req});
        sb.push_back({tb-flowStart,req});
        ta=tb;
    }

    auto hull=upper_hull(sb);
    std::vector<Row> rows;
    rows.reserve(hull.size());
    for(auto [s,b]:hull){
        Row R;R.a.assign(6,0.0);R.b=b;
        R.a[0]=n.x;R.a[1]=n.y;R.a[2]=n.z;
        R.a[3]=s*n.x;R.a[4]=s*n.y;R.a[5]=s*n.z;
        rows.push_back(std::move(R));
    }
    return rows;
}


static inline bool r31_vec_exact(Vec3 a,Vec3 b){
    return a.x==b.x&&a.y==b.y&&a.z==b.z;
}
static inline Vec3 r31_endpoint_derivative(
        const Poly&C,double T,int r,bool right){
    if(r==0)return right?C.back():C.front();
    Poly D=deriv_poly(C,T,r);
    if(D.empty())return {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    return right?D.back():D.front();
}
static inline bool r31_exact_c7_stored(const Candidate&C){
    if(C.cp.empty()||C.cp.size()!=C.seg0.size()||C.cp.size()!=C.seg1.size())
        return false;
    Vec3 z{};
    for(size_t s=0;s<C.cp.size();++s)
        if(C.cp[s].size()!=16||!(C.seg1[s]>C.seg0[s]))return false;

    for(int r=0;r<=7;++r){
        if(!r31_vec_exact(r31_endpoint_derivative(
                C.cp.front(),C.seg1.front()-C.seg0.front(),r,false),z))
            return false;
        if(!r31_vec_exact(r31_endpoint_derivative(
                C.cp.back(),C.seg1.back()-C.seg0.back(),r,true),z))
            return false;
    }
    for(size_t s=0;s+1<C.cp.size();++s){
        if(C.seg1[s]!=C.seg0[s+1])return false;
        const double TL=C.seg1[s]-C.seg0[s];
        const double TR=C.seg1[s+1]-C.seg0[s+1];
        for(int r=0;r<=7;++r)
            if(!r31_vec_exact(
                r31_endpoint_derivative(C.cp[s],TL,r,true),
                r31_endpoint_derivative(C.cp[s+1],TR,r,false)))
                return false;
    }
    return true;
}

static inline std::vector<double> r31_flow_hessian_3d(
        const R28FlowGrid&G){
    if(!G.ok||G.nflow<1)return {};
    double H00=0,H01=0,H11=0;
    auto add=[&](const std::array<double,16>&bp,
                 const std::array<double,16>&bv,double T){
        double a=0,b=0,c=0;
        for(int i=0;i<16;++i)for(int j=0;j<16;++j){
            const double Q=gram(15,i,j);
            a+=bp[i]*bp[j]*Q;
            b+=bp[i]*bv[j]*Q;
            c+=bv[i]*bv[j]*Q;
        }
        H00+=T*a;H01+=T*b;H11+=T*c;
    };
    std::array<double,16>bp{},bv{};
    for(int k=8;k<16;++k){
        bp[k]=1.0;bv[k]=G.entry_T*double(k-15)/15.0;
    }
    add(bp,bv,G.entry_T);

    for(int m=0;m<G.nflow;++m){
        for(int k=0;k<16;++k){
            bp[k]=1.0;
            bv[k]=double(m)*G.flow_T+G.flow_T*double(k)/15.0;
        }
        add(bp,bv,G.flow_T);
    }

    bp.fill(0);bv.fill(0);
    const double flowDur=double(G.nflow)*G.flow_T;
    for(int k=0;k<8;++k){
        bp[k]=1.0;
        bv[k]=flowDur+G.exit_T*double(k)/15.0;
    }
    add(bp,bv,G.exit_T);

    std::vector<double>HH(36,0.0);
    for(int d=0;d<3;++d){
        HH[d*6+d]=H00;
        HH[d*6+(3+d)]=H01;
        HH[(3+d)*6+d]=H01;
        HH[(3+d)*6+(3+d)]=H11;
    }
    return HH;
}

static inline Candidate r28_build_flow_candidate(
        const R28FlowGrid&G,const std::vector<double>&z,double cost,
        const std::vector<int>&signs,double minSlack){
    if(!G.ok||z.size()!=6)throw std::runtime_error("R31 invalid build");

    Vec3 P0{z[0],z[1],z[2]},V{z[3],z[4],z[5]};
    Vec3 di=V*(G.base_h/15.0);

    const long long kmin=-7LL*G.Ke;
    const long long flowBase=15LL*G.nflow*G.Kf;
    const long long kmax=flowBase+7LL*G.Kx;
    const auto ax=r27_axis_lattice(P0.x,di.x,kmin,kmax);
    const auto ay=r27_axis_lattice(P0.y,di.y,kmin,kmax);
    const auto az=r27_axis_lattice(P0.z,di.z,kmin,kmax);

    Candidate C;
    C.family=7;C.constant_translation=false;
    C.cost=cost;C.signs=signs;C.minSlack=minSlack;

    Poly entry(16,Vec3{});
    for(int k=8;k<16;++k)
        entry[k]=r27_grid_vec(ax,ay,az,(long long)(k-15)*G.Ke);
    C.seg0.push_back(G.support_start);C.seg1.push_back(G.flow_start);
    C.cp.push_back(std::move(entry));

    double ta=G.flow_start;
    for(int m=0;m<G.nflow;++m){
        Poly p(16);
        const long long base=15LL*m*G.Kf;
        for(int k=0;k<16;++k)
            p[k]=r27_grid_vec(ax,ay,az,base+(long long)k*G.Kf);
        double tb=0.0;
        if(!r31_exact_time(ta,G.flow_T,tb))
            throw std::runtime_error("R31 flow time");
        C.seg0.push_back(ta);C.seg1.push_back(tb);C.cp.push_back(std::move(p));
        ta=tb;
    }
    if(ta!=G.flow_end)throw std::runtime_error("R31 flow end");

    Poly exit(16,Vec3{});
    for(int k=0;k<8;++k)
        exit[k]=r27_grid_vec(ax,ay,az,flowBase+(long long)k*G.Kx);
    C.seg0.push_back(G.flow_end);C.seg1.push_back(G.exit_end);
    C.cp.push_back(std::move(exit));

    const Vec3 Pq=r27_grid_vec(ax,ay,az,0);
    const Vec3 dq=r27_grid_vec(
        R27AxisLattice{ax.e,0,ax.d},
        R27AxisLattice{ay.e,0,ay.d},
        R27AxisLattice{az.e,0,az.d},1);
    C.pos={Pq};
    C.vel={dq*(15.0/G.base_h)};
    C.exact_c7=r31_exact_c7_stored(C);

    C.dub.fill(0);C.max_ctrl=0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        Poly D=C.cp[s];int deg=(int)D.size()-1;
        for(auto v:D)C.max_ctrl=std::max(C.max_ctrl,norm(v));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }
    return C;
}




// ===========================================================================
// AF7R32 AFFINE-NOMINAL PROGRESS RETIMING
//
// For an exactly affine active base reference p_b(t)=p0+v_b t, scalar
// progress retiming is exactly representable inside the existing additive
// correction packet by d_parallel(t)=v_b(s(t)-t).  Hence no runtime packet
// schema change is required.
//
// The spatial corridor variables [P,Vpath] are solved in nominal progress
// coordinates.  Actual entry / flow / exit durations are search variables;
// no obstacle-size-dependent duration constant is used.
// ===========================================================================
static inline bool r32_vec_zero_exact(Vec3 a){return a.x==0.0&&a.y==0.0&&a.z==0.0;}
static inline bool r32_vec_equal_exact(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
static inline bool r32_affine_base_exact(const ActiveReference&ref,double t0,double a,double b,double t1,Vec3&v){
    const double T[4]={t0,a,b,t1}; std::array<Vec3,7> X[4];
    for(int i=0;i<4;++i)X[i]=ref.eval(T[i]);
    v=X[0][1]; if(!(finite(v)&&norm(v)>0))return false;
    for(int i=0;i<4;++i){
        if(!r32_vec_equal_exact(X[i][1],v))return false;
        for(int r=2;r<=6;++r)if(!r32_vec_zero_exact(X[i][r]))return false;
    }
    return true;
}
static inline void r32_add_lateral_equalities(std::vector<Row>&rows,Vec3 e){
    e=normalized(e);
    Row r;r.a.assign(6,0.0);r.b=0.0;r.a[0]=e.x;r.a[1]=e.y;r.a[2]=e.z;rows.push_back(r);
    for(double&x:r.a)x=-x;rows.push_back(r);
    Row v;v.a.assign(6,0.0);v.b=0.0;v.a[3]=e.x;v.a[4]=e.y;v.a[5]=e.z;rows.push_back(v);
    for(double&x:v.a)x=-x;rows.push_back(v);
}
static inline bool r32_progress_monotone_exact(Vec3 vbase,const Candidate&C){
    const double nv=norm(vbase); if(!(fin(nv)&&nv>0))return false; Vec3 e=vbase/nv;
    const double guard=256*epsd()*std::max(1.0,nv);
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s]; if(!(fin(T)&&T>0))return false;
        Poly D=deriv_poly(C.cp[s],T,1);
        for(const auto&dc:D){double vp=dot(e,vbase+dc);if(!(fin(vp)&&vp>=-guard))return false;}
    }
    return true;
}
static inline double r32_affine_velocity_control_ub(Vec3 vbase,const Candidate&C){
    double ub=0.0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s]; Poly D=deriv_poly(C.cp[s],T,1);
        for(const auto&dc:D)ub=std::max(ub,norm(vbase+dc));
    }
    return ub;
}
static inline Candidate r32_build_retimed_candidate(double t0,double t1,double sA,double sB,
        Vec3 vbase,Vec3 P,Vec3 Vpath,double alpha,double Te,double cost,
        const std::vector<int>&signs,double minSlack){
    Candidate C; C.family=7; C.constant_translation=false; C.cost=cost; C.signs=signs; C.minSlack=minSlack;
    C.progress_retimed=true; C.progress_scale=alpha;
    const double Ds=sB-sA;
    if(!(fin(alpha)&&alpha>0&&alpha<=1&&fin(Te)&&Te>0&&fin(Ds)&&Ds>0))throw std::runtime_error("R32 invalid timing");
    const double Tf=Ds/alpha, Tx=t1-t0-Te-Tf;
    if(!(fin(Tf)&&Tf>0&&fin(Tx)&&Tx>0))throw std::runtime_error("R32 no exit time");
    const double tA=t0+Te,tB=tA+Tf; if(!(t0<tA&&tA<tB&&tB<t1))throw std::runtime_error("R32 timing order");

    Vec3 C0=P+vbase*(sA-tA);
    Vec3 Vc=Vpath*alpha+vbase*(alpha-1.0);
    Vec3 C1=C0+Vc*Tf;
    std::array<Vec3,8>Z{},JA{},JB{}; JA[0]=C0;JA[1]=Vc;JB[0]=C1;JB[1]=Vc;
    C.seg0={t0,tA,tB}; C.seg1={tA,tB,t1};
    C.cp.push_back(vector_control_from_jets(Z,JA,Te));
    C.cp.push_back(vector_control_from_jets(JA,JB,Tf));
    C.cp.push_back(vector_control_from_jets(JB,Z,Tx));
    C.exact_c7=r31_exact_c7_stored(C); C.pos={P};C.vel={Vpath};
    C.retime_entry_T=Te;C.retime_flow_T=Tf;C.retime_exit_T=Tx;
    C.dub.fill(0);C.max_ctrl=0.0;
    for(size_t s=0;s<C.cp.size();++s){
        double T=C.seg1[s]-C.seg0[s];Poly D=C.cp[s];int deg=(int)D.size()-1;
        for(auto q:D)C.max_ctrl=std::max(C.max_ctrl,norm(q));
        for(int r=1;r<=6;++r){Poly E(D.size()-1);for(size_t i=0;i+1<D.size();++i)E[i]=(D[i+1]-D[i])*(double(deg)/T);D.swap(E);--deg;C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));}
    }
    return C;
}


// AF7R33 exact adaptive timing helpers.
// Physical maneuver durations are not fixed.  The only fixed quantity here is
// a binary64 representation quantum derived from live control dt.
static inline double r33_time_quantum(double dt){
    if(!(fin(dt)&&dt>0))throw std::runtime_error("R33 invalid dt");
    return std::ldexp(1.0,std::ilogb(dt)-4);
}
static inline double r33_align_up(double x,double h){
    if(!(fin(x)&&fin(h)&&h>0))return std::numeric_limits<double>::quiet_NaN();
    const double q=x/h;
    if(!(fin(q)&&std::fabs(q)<4503599627370496.0))
        return std::numeric_limits<double>::quiet_NaN();
    double k=std::ceil(q),y=k*h;
    if(y<x)y=(k+1.0)*h;
    return y;
}
static inline double r33_time_at_units(double start,double h,long long u){
    const double q0=start/h;
    if(!(fin(q0)&&q0==std::floor(q0)&&std::fabs(q0)<4503599627370496.0))
        return std::numeric_limits<double>::quiet_NaN();
    const double q=q0+(double)u;
    if(!(fin(q)&&std::fabs(q)<4503599627370496.0))
        return std::numeric_limits<double>::quiet_NaN();
    return q*h;
}
static inline std::vector<long long> r33_power_units(long long maxu){
    std::vector<long long> out;
    for(long long k=1;k>0&&k<=maxu;){
        out.push_back(k);
        if(k>maxu/2)break;
        k*=2;
    }
    return out;
}
static inline std::vector<long long> r33_binary_chunks(long long units){
    std::vector<long long> c;
    if(units<=0)return c;
    long long p=1;
    while(p<=units/2)p*=2;
    while(p>0){
        if(units>=p){c.push_back(p);units-=p;}
        p/=2;
    }
    return c;
}
static inline bool r33_dominates(
        double va,double aa,double ja,double vb,double ab,double jb){
    return va<=vb&&aa<=ab&&ja<=jb&&(va<vb||aa<ab||ja<jb);
}

static inline Candidate r33_build_retimed_candidate(
        double supportStart,double h,long long Ke,long long Kflow,long long Kx,
        double sA,double sB,Vec3 vbase,Vec3 P,Vec3 Vpath,double cost,
        const std::vector<int>&signs,double minSlack){
    if(!(fin(supportStart)&&fin(h)&&h>0&&Ke>0&&Kflow>0&&Kx>0))
        throw std::runtime_error("R33 invalid timing units");

    const double Te=(double)Ke*h;
    const double Tf=(double)Kflow*h;
    const double Tx=(double)Kx*h;
    const double Ds=sB-sA;
    if(!(fin(Ds)&&Ds>0&&Tf>=Ds))
        throw std::runtime_error("R33 invalid progress duration");
    const double alpha=Ds/Tf;
    if(!(fin(alpha)&&alpha>0&&alpha<=1.0))
        throw std::runtime_error("R33 invalid alpha");

    const double tA=r33_time_at_units(supportStart,h,Ke);
    const double tB=r33_time_at_units(supportStart,h,Ke+Kflow);
    const double tE=r33_time_at_units(supportStart,h,Ke+Kflow+Kx);
    if(!(fin(tA)&&fin(tB)&&fin(tE)&&supportStart<tA&&tA<tB&&tB<tE))
        throw std::runtime_error("R33 invalid timing order");

    Vec3 C0=P+vbase*(sA-tA);
    Vec3 Vc=Vpath*alpha+vbase*(alpha-1.0);

    Vec3 di=Vc*(h/15.0);
    const long long kmin=-7LL*Ke;
    const long long kmax=15LL*Kflow+7LL*Kx;
    const auto ax=r27_axis_lattice(C0.x,di.x,kmin,kmax);
    const auto ay=r27_axis_lattice(C0.y,di.y,kmin,kmax);
    const auto az=r27_axis_lattice(C0.z,di.z,kmin,kmax);

    Candidate C;
    C.family=7;C.constant_translation=false;
    C.cost=cost;C.signs=signs;C.minSlack=minSlack;
    C.progress_retimed=true;

    Poly entry(16,Vec3{});
    for(int k=8;k<16;++k)
        entry[k]=r27_grid_vec(ax,ay,az,(long long)(k-15)*Ke);
    C.seg0.push_back(supportStart);C.seg1.push_back(tA);
    C.cp.push_back(std::move(entry));

    auto chunks=r33_binary_chunks(Kflow);
    if(chunks.empty())throw std::runtime_error("R33 empty flow");
    long long uoff=Ke,ioff=0;
    for(long long K:chunks){
        const double ta=r33_time_at_units(supportStart,h,uoff);
        const double tb=r33_time_at_units(supportStart,h,uoff+K);
        if(!(fin(ta)&&fin(tb)&&tb>ta&&(tb-ta)==(double)K*h))
            throw std::runtime_error("R33 flow time exactness");

        Poly p(16);
        const long long base=15LL*ioff;
        for(int k=0;k<16;++k)
            p[k]=r27_grid_vec(ax,ay,az,base+(long long)k*K);
        C.seg0.push_back(ta);C.seg1.push_back(tb);C.cp.push_back(std::move(p));
        uoff+=K;ioff+=K;
    }

    Poly exit(16,Vec3{});
    const long long base=15LL*Kflow;
    for(int k=0;k<8;++k)
        exit[k]=r27_grid_vec(ax,ay,az,base+(long long)k*Kx);
    C.seg0.push_back(tB);C.seg1.push_back(tE);C.cp.push_back(std::move(exit));

    C.exact_c7=r31_exact_c7_stored(C);
    if(!C.exact_c7)return C;

    const Vec3 C0q=r27_grid_vec(ax,ay,az,0);
    const Vec3 dq=r27_grid_vec(
        R27AxisLattice{ax.e,0,ax.d},
        R27AxisLattice{ay.e,0,ay.d},
        R27AxisLattice{az.e,0,az.d},1);
    const Vec3 Vcq=dq*(15.0/h);

    const double vv=dot(vbase,vbase);
    const double alphaQ=1.0+dot(vbase,Vcq)/vv;
    if(!(fin(alphaQ)&&alphaQ>0&&alphaQ<=1.0+
         4096*epsd()*std::max(1.0,std::fabs(alphaQ))))
        throw std::runtime_error("R33 quantized progress invalid");

    const Vec3 Pq=C0q-vbase*(sA-tA);
    const Vec3 VpathQ=(Vcq-vbase*(alphaQ-1.0))/alphaQ;
    C.pos={Pq};C.vel={VpathQ};
    C.progress_scale=alphaQ;
    C.retime_entry_T=Te;C.retime_flow_T=Tf;C.retime_exit_T=Tx;

    C.dub.fill(0);C.max_ctrl=0.0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        Poly D=C.cp[s];int deg=(int)D.size()-1;
        for(auto q:D)C.max_ctrl=std::max(C.max_ctrl,norm(q));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }
    return C;
}



// ===========================================================================
// AF7R34 DELAY-PRESERVING RETIMING
//
// Key theorem for the current exact-affine nominal base:
//
//   p_nom(t) = p0 + v_b t.
//
// If the maneuver finishes at t_h with schedule delay Delta >= 0 and then
// keeps the constant correction
//
//   d_hold = -v_b Delta,
//
// then for every t >= t_h
//
//   p_nom(t) + d_hold = p_nom(t-Delta).
//
// Therefore the repaired reference has rejoined the SAME NOMINAL SPATIAL PATH
// and continues with nominal speed, but it does not need the physically
// harmful catch-up sdot>1 that R32/R33 imposed.
//
// The trigger horizon H=4 s remains trigger-only.
// ===========================================================================

static inline bool r34_exact_c7_terminal_hold(const Candidate&C){
    if(C.cp.empty()||C.cp.size()!=C.seg0.size()||C.cp.size()!=C.seg1.size())
        return false;

    Vec3 zero{};
    // Initial correction and jets must be identically zero.
    for(int r=0;r<=7;++r){
        if(!r31_vec_exact(
                r31_endpoint_derivative(
                    C.cp.front(),C.seg1.front()-C.seg0.front(),r,false),
                zero))
            return false;
    }

    // Every internal stored join is exact through order 7.
    for(size_t s=0;s+1<C.cp.size();++s){
        if(C.seg1[s]!=C.seg0[s+1])return false;
        const double TL=C.seg1[s]-C.seg0[s];
        const double TR=C.seg1[s+1]-C.seg0[s+1];
        for(int r=0;r<=7;++r){
            if(!r31_vec_exact(
                    r31_endpoint_derivative(C.cp[s],TL,r,true),
                    r31_endpoint_derivative(C.cp[s+1],TR,r,false)))
                return false;
        }
    }

    // The terminal POSITION may be nonzero; terminal derivatives 1..7 must
    // be exactly zero because the correction is held forever afterward.
    const double T=C.seg1.back()-C.seg0.back();
    for(int r=1;r<=7;++r){
        if(!r31_vec_exact(
                r31_endpoint_derivative(C.cp.back(),T,r,true),zero))
            return false;
    }
    return r31_vec_exact(C.cp.back().back(),C.terminal_offset);
}

struct R34TimingTrial{
    long long Ke=0,Kflow=0,Kx=0;
    double alpha=1.0;
    double preV=std::numeric_limits<double>::infinity();
    double acc=std::numeric_limits<double>::infinity();
    double jerk=std::numeric_limits<double>::infinity();
};

static inline bool r34_trial_dominates(
        const R34TimingTrial&a,const R34TimingTrial&b){
    return a.preV<=b.preV && a.acc<=b.acc && a.jerk<=b.jerk &&
           (a.preV<b.preV || a.acc<b.acc || a.jerk<b.jerk);
}

static inline R34TimingTrial r34_timing_prebound(
        double supportStart,double h,long long Ke,long long Kflow,long long Kx,
        double sA,double sB,Vec3 vbase,Vec3 P,Vec3 Vpath){
    R34TimingTrial R;
    R.Ke=Ke;R.Kflow=Kflow;R.Kx=Kx;

    const double Te=(double)Ke*h;
    const double Tf=(double)Kflow*h;
    const double Tx=(double)Kx*h;
    const double Ds=sB-sA;
    if(!(Te>0&&Tf>=Ds&&Tx>0))return R;

    const double alpha=Ds/Tf;
    if(!(fin(alpha)&&alpha>0&&alpha<=1))return R;
    R.alpha=alpha;

    const double tA=supportStart+Te;
    const double tB=tA+Tf;
    const double tE=tB+Tx;

    Vec3 C0=P+vbase*(sA-tA);
    Vec3 Vc=Vpath*alpha+vbase*(alpha-1.0);
    Vec3 C1=C0+Vc*Tf;

    // Symmetric C7 speed transition alpha -> 1:
    // the average progress rate is (alpha+1)/2, so the final nominal-path
    // progress is determined, not tuned.
    const double sE=sB+0.5*(alpha+1.0)*Tx;
    const double delay=tE-sE;
    if(!(fin(delay)&&delay>=0))return R;
    Vec3 H=vbase*(-delay);

    std::array<Vec3,8>Z{},JA{},JB{},JH{};
    JA[0]=C0;JA[1]=Vc;
    JB[0]=C1;JB[1]=Vc;
    JH[0]=H;

    Poly A=vector_control_from_jets(Z,JA,Te);
    Poly B=vector_control_from_jets(JA,JB,Tf);
    Poly C=vector_control_from_jets(JB,JH,Tx);

    double vub=0,aub=0,jub=0;
    Poly S[3]={A,B,C};
    double TT[3]={Te,Tf,Tx};
    for(int is=0;is<3;++is){
        Poly D1=deriv_poly(S[is],TT[is],1);
        Poly D2=deriv_poly(S[is],TT[is],2);
        Poly D3=deriv_poly(S[is],TT[is],3);
        for(auto d:D1)vub=std::max(vub,norm(vbase+d));
        aub=std::max(aub,poly_norm_ub(D2));
        jub=std::max(jub,poly_norm_ub(D3));
    }
    R.preV=vub;R.acc=aub;R.jerk=jub;
    return R;
}

static inline Candidate r34_build_delay_candidate(
        double supportStart,double searchEnd,double h,
        long long Ke,long long Kflow,long long Kx,
        double sA,double sB,Vec3 vbase,Vec3 P,Vec3 Vpath,
        double cost,const std::vector<int>&signs,double minSlack){
    Candidate C;
    C.family=7;C.constant_translation=false;
    C.cost=cost;C.signs=signs;C.minSlack=minSlack;
    C.progress_retimed=true;
    C.terminal_hold=true;

    const double Te=(double)Ke*h;
    const double Tf=(double)Kflow*h;
    const double Tx=(double)Kx*h;
    const double Ds=sB-sA;
    if(!(Te>0&&Tf>=Ds&&Tx>0&&searchEnd>supportStart+Te+Tf+Tx))
        throw std::runtime_error("R34 invalid timing");

    const double alpha=Ds/Tf;
    if(!(fin(alpha)&&alpha>0&&alpha<=1))
        throw std::runtime_error("R34 invalid alpha");

    const double tA=r33_time_at_units(supportStart,h,Ke);
    const double tB=r33_time_at_units(supportStart,h,Ke+Kflow);
    const double tE=r33_time_at_units(supportStart,h,Ke+Kflow+Kx);
    if(!(fin(tA)&&fin(tB)&&fin(tE)&&supportStart<tA&&tA<tB&&tB<tE&&tE<searchEnd))
        throw std::runtime_error("R34 invalid exact timing");

    // Unquantized desired entry state.
    Vec3 C0=P+vbase*(sA-tA);
    Vec3 Vc=Vpath*alpha+vbase*(alpha-1.0);

    // One common dyadic velocity lattice for entry -> flow -> exit-start.
    Vec3 di=Vc*(h/15.0);
    const long long kmin=-7LL*Ke;
    const long long kmax=15LL*Kflow+7LL*Kx;
    const auto ax=r27_axis_lattice(C0.x,di.x,kmin,kmax);
    const auto ay=r27_axis_lattice(C0.y,di.y,kmin,kmax);
    const auto az=r27_axis_lattice(C0.z,di.z,kmin,kmax);

    const Vec3 C0q=r27_grid_vec(ax,ay,az,0);
    const Vec3 dq=r27_grid_vec(
        R27AxisLattice{ax.e,0,ax.d},
        R27AxisLattice{ay.e,0,ay.d},
        R27AxisLattice{az.e,0,az.d},1);
    const Vec3 Vcq=dq*(15.0/h);

    // Recover the ACTUAL represented progress law after quantization.
    const double vv=dot(vbase,vbase);
    if(!(fin(vv)&&vv>0))throw std::runtime_error("R34 invalid base speed");
    const double sAq=tA+dot(vbase,C0q)/vv;
    const double alphaQ=1.0+dot(vbase,Vcq)/vv;
    if(!(fin(alphaQ)&&alphaQ>0&&alphaQ<=1.0+
         4096*epsd()*std::max(1.0,std::fabs(alphaQ))))
        throw std::runtime_error("R34 nonmonotone quantized progress");
    const double sBq=sAq+alphaQ*Tf;

    // Delay-preserving terminal progress.  A symmetric C7 alpha->1 speed
    // transition has mean progress rate (alpha+1)/2.  No catch-up is imposed.
    const double sEq=sBq+0.5*(alphaQ+1.0)*Tx;
    const double delay=tE-sEq;
    if(!(fin(delay)&&delay>=-4096*epsd()*std::max(1.0,std::fabs(tE))))
        throw std::runtime_error("R34 negative delay");
    const double delayQ=std::max(0.0,delay);
    const Vec3 H=vbase*(-delayQ);

    // Entry: zero jet -> C0q,Vcq.
    Poly entry(16,Vec3{});
    for(int k=8;k<16;++k)
        entry[k]=r27_grid_vec(ax,ay,az,(long long)(k-15)*Ke);

    // One exact affine-flow segment.  No binary segmentation is needed:
    // duration Kflow*h and control step Kflow*dq give exactly the same Vcq.
    Poly flow(16);
    for(int k=0;k<16;++k)
        flow[k]=r27_grid_vec(ax,ay,az,(long long)k*Kflow);

    // Exit start shares the same position/velocity lattice exactly.  The last
    // eight controls repeat H, so terminal derivatives 1..7 are exactly zero.
    Poly exit(16,H);
    const long long base=15LL*Kflow;
    for(int k=0;k<8;++k)
        exit[k]=r27_grid_vec(ax,ay,az,base+(long long)k*Kx);

    // Constant hold to finite certificate end; persisted after searchEnd by
    // the active-reference evaluator.
    Poly hold(16,H);

    C.seg0={supportStart,tA,tB,tE};
    C.seg1={tA,tB,tE,searchEnd};
    C.cp={std::move(entry),std::move(flow),std::move(exit),std::move(hold)};

    C.terminal_hold_start=tE;
    C.terminal_offset=H;
    C.schedule_delay=delayQ;
    C.progress_scale=alphaQ;
    C.retime_entry_T=Te;
    C.retime_flow_T=Tf;
    C.retime_exit_T=Tx;

    C.exact_c7=r34_exact_c7_terminal_hold(C);

    C.dub.fill(0);C.max_ctrl=0.0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        if(!(fin(T)&&T>0))throw std::runtime_error("R34 invalid segment time");
        Poly D=C.cp[s];int deg=(int)D.size()-1;
        for(auto q:D)C.max_ctrl=std::max(C.max_ctrl,norm(q));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }

    // Stored diagnostic path variables reconstructed from represented jets.
    const Vec3 Pq=C0q-vbase*(sA-tA);
    const Vec3 VpathQ=(Vcq-vbase*(alphaQ-1.0))/alphaQ;
    C.pos={Pq};C.vel={VpathQ};
    return C;
}



// ===========================================================================
// AF7R35 PATH-TANGENT YAW C6 THEOREM
//
// For an exact-affine base p_b(t)=p0+v_b t and an exact-C7 correction d(t),
// define the actual horizontal velocity
//
//      v_xy(t) = [v_bx + d'_x(t), v_by + d'_y(t)].
//
// Let e_h = v_b,xy / ||v_b,xy||.  On each Bezier segment, e_h^T v_xy(t)
// is a scalar Bernstein polynomial.  Therefore
//
//      e_h^T v_xy(t) >= min_i e_h^T V_i = gamma.
//
// If gamma>0, the whole trajectory lies in one open horizontal half-plane,
// hence ||v_xy||>=gamma and atan2(v_y,v_x) has a unique continuous branch
// within +/-pi/2 of the nominal heading.  Because p(t) is C7, v_xy is C6,
// and the path-tangent yaw is C6.
//
// This certificate is independent of obstacle geometry and does not weaken
// any existing speed/rejoin/whole-body certificate.
// ===========================================================================
static inline double r35_heading_projection_lb(Vec3 vbase,const Candidate&C){
    const double nh=std::hypot(vbase.x,vbase.y);
    if(!(fin(nh)&&nh>0))return std::numeric_limits<double>::quiet_NaN();
    const double ex=vbase.x/nh, ey=vbase.y/nh;
    double gamma=nh;

    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        if(!(fin(T)&&T>0))return std::numeric_limits<double>::quiet_NaN();
        Poly D=deriv_poly(C.cp[s],T,1);
        for(const auto&dc:D){
            const double proj=ex*(vbase.x+dc.x)+ey*(vbase.y+dc.y);
            if(!fin(proj))return std::numeric_limits<double>::quiet_NaN();
            gamma=std::min(gamma,proj);
        }
    }

    // After a delay-preserving terminal hold the correction derivative is
    // identically zero, so the forward projection is exactly ||v_b,xy||.
    if(C.terminal_hold)gamma=std::min(gamma,nh);
    return gamma;
}

static inline bool r35_apply_heading_certificate(
        Vec3 vbase,Candidate&C,double&gammaOut,double&anchorOut){
    const double nh=std::hypot(vbase.x,vbase.y);
    if(!(fin(nh)&&nh>0))return false;
    const double gamma=r35_heading_projection_lb(vbase,C);
    if(!(fin(gamma)&&gamma>0))return false;

    C.heading_c6=true;
    C.heading_gamma=gamma;
    C.heading_anchor_rad=std::atan2(vbase.y,vbase.x);
    C.heading_base_velocity=vbase;
    gammaOut=gamma;
    anchorOut=C.heading_anchor_rad;
    return true;
}



// ===========================================================================
// AF7R36 CURVED SPATIAL C7 PRIMARY
//
// Family 7 no longer forces one affine P+V(s-sA) correction over the unsafe
// span.  It uses two degree-15 spatial Hermite pieces:
//     (P0,V0) -> (PM,VM) -> (P1,V1)
// with spatial derivatives 2..7 exactly zero at all three nodes.
//
// The six vector node states are restricted to the plane perpendicular to the
// exact affine nominal progress direction.  R34 delay preservation and R35
// path-tangent yaw remain unchanged.
// ===========================================================================
struct R36Plane{Vec3 b1{},b2{};};

static inline R36Plane r36_plane(Vec3 progress){
    const double np=norm(progress);
    if(!(finite(progress)&&fin(np)&&np>0))
        throw std::runtime_error("R36 zero progress");
    Vec3 v=progress/np;
    double av[3]={std::fabs(v.x),std::fabs(v.y),std::fabs(v.z)};
    int k=0;if(av[1]<av[k])k=1;if(av[2]<av[k])k=2;
    Vec3 e=k==0?Vec3{1,0,0}:(k==1?Vec3{0,1,0}:Vec3{0,0,1});
    Vec3 b1=e-v*dot(v,e);
    double n1=norm(b1);
    if(!(fin(n1)&&n1>0))throw std::runtime_error("R36 plane basis");
    b1=b1/n1;
    Vec3 b2=cross(v,b1);
    double n2=norm(b2);
    if(!(fin(n2)&&n2>0))throw std::runtime_error("R36 plane cross");
    b2=b2/n2;
    return {b1,b2};
}

static inline double r36_scalar_bezier_eval(std::vector<double>P,double u){
    if(!(fin(u)&&u>=0&&u<=1)||P.empty())
        return std::numeric_limits<double>::quiet_NaN();
    while(P.size()>1){
        std::vector<double>Q(P.size()-1);
        for(size_t i=0;i+1<P.size();++i)
            Q[i]=P[i]*(1.0-u)+P[i+1]*u;
        P.swap(Q);
    }
    return P.front();
}

// scalar slot order: P0,V0, PM,VM, P1,V1.
static inline std::array<double,6> r36_spatial_basis(
        double s,double sA,double sB){
    if(!(fin(s)&&fin(sA)&&fin(sB)&&sB>sA))
        throw std::runtime_error("R36 spatial basis range");
    const double sM=sA+0.5*(sB-sA);
    const double D=0.5*(sB-sA);
    const int seg=(s<=sM)?0:1;
    const double a=(seg==0)?sA:sM;
    double u=(s-a)/D;
    u=std::max(0.0,std::min(1.0,u));

    std::array<double,6>w{};
    for(int slot=0;slot<6;++slot){
        std::array<double,8>J0{},J1{};
        if(seg==0){
            if(slot==0)J0[0]=1.0;
            else if(slot==1)J0[1]=1.0;
            else if(slot==2)J1[0]=1.0;
            else if(slot==3)J1[1]=1.0;
        }else{
            if(slot==2)J0[0]=1.0;
            else if(slot==3)J0[1]=1.0;
            else if(slot==4)J1[0]=1.0;
            else if(slot==5)J1[1]=1.0;
        }
        w[slot]=r36_scalar_bezier_eval(
            scalar_control_from_jets(J0,J1,D),u);
    }
    return w;
}

static inline std::vector<double> r36_spatial_hessian(
        double sA,double sB,double cableL,double gravity){
    const double D=0.5*(sB-sA);
    if(!(fin(D)&&D>0&&fin(cableL)&&cableL>0&&fin(gravity)&&gravity>0))
        throw std::runtime_error("R36 spatial Hessian request");

    const double tau2=cableL/gravity;
    const double wgt[4]={1.0/tau2,1.0,tau2,tau2*tau2};
    const int ns=6;
    std::vector<double>Hs(ns*ns,0.0);

    for(int seg=0;seg<2;++seg){
        std::vector<std::vector<double>>CP(ns,std::vector<double>(16,0.0));
        for(int slot=0;slot<ns;++slot){
            std::array<double,8>J0{},J1{};
            if(seg==0){
                if(slot==0)J0[0]=1.0;
                else if(slot==1)J0[1]=1.0;
                else if(slot==2)J1[0]=1.0;
                else if(slot==3)J1[1]=1.0;
            }else{
                if(slot==2)J0[0]=1.0;
                else if(slot==3)J0[1]=1.0;
                else if(slot==4)J1[0]=1.0;
                else if(slot==5)J1[1]=1.0;
            }
            CP[slot]=scalar_control_from_jets(J0,J1,D);
        }

        for(int r=0;r<=3;++r){
            const int deg=15-r,ncp=16-r;
            double ff=1.0;
            for(int q=0;q<r;++q)ff*=double(15-q);
            std::vector<std::vector<double>>B(ns);
            for(int i=0;i<ns;++i){
                B[i]=diff_scalar(CP[i],r);
                const double sc=(r==0)?1.0:ff/std::pow(D,r);
                for(double&v:B[i])v*=sc;
            }
            for(int i=0;i<ns;++i)for(int j=0;j<ns;++j){
                double q=0.0;
                for(int aa=0;aa<ncp;++aa)for(int bb=0;bb<ncp;++bb)
                    q+=B[i][aa]*gram(deg,aa,bb)*B[j][bb];
                Hs[i*ns+j]+=wgt[r]*D*q;
            }
        }
    }

    std::vector<double>H(12*12,0.0);
    for(int i=0;i<6;++i)for(int j=0;j<6;++j){
        H[(2*i+0)*12+(2*j+0)]=Hs[i*6+j];
        H[(2*i+1)*12+(2*j+1)]=Hs[i*6+j];
    }
    return H;
}

static inline std::vector<Row> r36_curved_corridor_rows(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,const EpisodeCert&E,int ei,Vec3 n,
        double sA,double sB,const R36Plane&B){
    const auto&ep=E.ep[ei];
    n=normalized(n);
    const auto&o=obs[ep.obs];

    Vec3 rn=tmul(o.R,n);
    Vec3 ar{o.axes.x*rn.x,o.axes.y*rn.y,o.axes.z*rn.z};
    const double hObs=dot(n,o.c)+norm(ar);
    const double VC=std::max(base.VL,base.VQ);

    std::vector<std::pair<double,double>>sb;
    double ta=ep.start,te=ep.end;
    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        const double tb=std::min(te,ta+L.dt);
        const double tc=0.5*(ta+tb),half=0.5*(tb-ta);
        auto x=ref.eval(tc);
        Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};
        const double ns=norm(sf);
        if(!(fin(ns)&&ns>0))
            throw std::runtime_error("R36 corridor flatness singular");
        Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;

        const double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        const double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        const double rhoC=L.rC+L.dmargin+L.tracking+VC*half;
        double req=std::max({
            hObs+rhoL-dot(n,pL),
            hObs+rhoQ-dot(n,pQ),
            hObs+rhoC-std::min(dot(n,pQ),dot(n,pL))
        });
        req+=64*epsd()*std::max(1.0,std::fabs(req));
        sb.push_back({ta,req});sb.push_back({tb,req});
        ta=tb;
    }

    // Synthesis only. Continuous whole-body certification remains the
    // acceptance authority.
    auto hull=upper_hull(sb);
    const double nb1=dot(n,B.b1),nb2=dot(n,B.b2);
    std::vector<Row>rows;
    for(auto [s,b]:hull){
        auto w=r36_spatial_basis(s,sA,sB);
        Row R;R.a.assign(12,0.0);R.b=b;
        for(int slot=0;slot<6;++slot){
            R.a[2*slot+0]=w[slot]*nb1;
            R.a[2*slot+1]=w[slot]*nb2;
        }
        rows.push_back(std::move(R));
    }
    return rows;
}

struct R36Path{
    bool ok=false;
    Vec3 P0{},V0{},PM{},VM{},P1{},V1{};
    double cost=0.0,slack=0.0;
    std::vector<int>signs;
};

static inline R36Path r36_decode_path(
        const QPSol&q,const R36Plane&B,const std::vector<int>&signs){
    R36Path P;
    if(!q.ok||q.z.size()!=12)return P;
    auto v=[&](int slot){
        return B.b1*q.z[2*slot]+B.b2*q.z[2*slot+1];
    };
    P.P0=v(0);P.V0=v(1);P.PM=v(2);P.VM=v(3);P.P1=v(4);P.V1=v(5);
    P.cost=q.cost;P.slack=q.slack;P.signs=signs;P.ok=true;
    return P;
}

static inline std::array<Poly,2> r36_spatial_controls(
        const R36Path&P,double sA,double sB){
    const double D=0.5*(sB-sA);
    std::array<Vec3,8>A{},M{},B{};
    A[0]=P.P0;A[1]=P.V0;
    M[0]=P.PM;M[1]=P.VM;
    B[0]=P.P1;B[1]=P.V1;
    return {vector_control_from_jets(A,M,D),
            vector_control_from_jets(M,B,D)};
}

static inline double r36_spatial_total_speed_ub(
        const R36Path&P,double sA,double sB,Vec3 vbase){
    auto C=r36_spatial_controls(P,sA,sB);
    const double D=0.5*(sB-sA);
    double ub=0.0;
    for(int j=0;j<2;++j){
        Poly V=deriv_poly(C[j],D,1);
        for(const auto&v:V)ub=std::max(ub,norm(vbase+v));
    }
    return ub;
}

static inline R34TimingTrial r36_timing_prebound(
        double supportStart,double h,long long Ke,long long Kflow,long long Kx,
        double sA,double sB,Vec3 vbase,const R36Path&P){
    R34TimingTrial R;R.Ke=Ke;R.Kflow=Kflow;R.Kx=Kx;
    if(Kflow<2||(Kflow&1LL))return R;

    const double Te=(double)Ke*h,Tf=(double)Kflow*h,Tx=(double)Kx*h;
    const double Ds=sB-sA;
    if(!(Te>0&&Tf>=Ds&&Tx>0))return R;
    const double alpha=Ds/Tf;
    if(!(fin(alpha)&&alpha>0&&alpha<=1))return R;
    R.alpha=alpha;

    const double tA=supportStart+Te,tB=tA+Tf,tE=tB+Tx;
    Vec3 C0=P.P0+vbase*(sA-tA);
    Vec3 W0=P.V0*alpha+vbase*(alpha-1.0);
    Vec3 C1=P.P1+vbase*(sB-tB);
    Vec3 W1=P.V1*alpha+vbase*(alpha-1.0);

    const double sE=sB+0.5*(alpha+1.0)*Tx;
    const double delay=tE-sE;
    if(!(fin(delay)&&delay>=0))return R;
    Vec3 H=vbase*(-delay);

    std::array<Vec3,8>Z{},J0{},J1{},JH{};
    J0[0]=C0;J0[1]=W0;J1[0]=C1;J1[1]=W1;JH[0]=H;
    Poly entry=vector_control_from_jets(Z,J0,Te);
    Poly exit=vector_control_from_jets(J1,JH,Tx);

    double vub=0.0,aub=0.0,jub=0.0;
    auto consume=[&](const Poly&Q,double T){
        Poly D1=deriv_poly(Q,T,1);
        Poly D2=deriv_poly(Q,T,2);
        Poly D3=deriv_poly(Q,T,3);
        for(auto d:D1)vub=std::max(vub,norm(vbase+d));
        aub=std::max(aub,poly_norm_ub(D2));
        jub=std::max(jub,poly_norm_ub(D3));
    };
    consume(entry,Te);consume(exit,Tx);

    auto SP=r36_spatial_controls(P,sA,sB);
    const double D=0.5*Ds;
    for(int seg=0;seg<2;++seg){
        Poly D1=deriv_poly(SP[seg],D,1);
        Poly D2=deriv_poly(SP[seg],D,2);
        Poly D3=deriv_poly(SP[seg],D,3);
        for(auto d:D1)vub=std::max(vub,alpha*norm(vbase+d));
        aub=std::max(aub,alpha*alpha*poly_norm_ub(D2));
        jub=std::max(jub,alpha*alpha*alpha*poly_norm_ub(D3));
    }
    R.preV=vub;R.acc=aub;R.jerk=jub;
    return R;
}

static inline Candidate r36_build_curved_delay_candidate(
        double supportStart,double searchEnd,double h,
        long long Ke,long long Kflow,long long Kx,
        double sA,double sB,Vec3 vbase,const R36Path&P){
    if(Kflow<2||(Kflow&1LL))
        throw std::runtime_error("R36 flow units must be even");

    Candidate C;
    C.family=7;C.constant_translation=false;
    C.cost=P.cost;C.signs=P.signs;C.minSlack=P.slack;
    C.progress_retimed=true;C.terminal_hold=true;

    const long long Ks=Kflow/2;
    const double Te=(double)Ke*h,Tf=(double)Kflow*h,Tx=(double)Kx*h;
    const double Ds=sB-sA,alpha=Ds/Tf;
    if(!(Te>0&&Tf>=Ds&&Tx>0&&alpha>0&&alpha<=1))
        throw std::runtime_error("R36 invalid timing");

    const double tA=r33_time_at_units(supportStart,h,Ke);
    const double tM=r33_time_at_units(supportStart,h,Ke+Ks);
    const double tB=r33_time_at_units(supportStart,h,Ke+Kflow);
    const double tE=r33_time_at_units(supportStart,h,Ke+Kflow+Kx);
    if(!(fin(tA)&&fin(tM)&&fin(tB)&&fin(tE)&&
         supportStart<tA&&tA<tM&&tM<tB&&tB<tE&&tE<searchEnd))
        throw std::runtime_error("R36 exact time grid");

    const double sM=sA+0.5*Ds;
    Vec3 PN[3]={P.P0,P.PM,P.P1};
    Vec3 VN[3]={P.V0,P.VM,P.V1};
    double SN[3]={sA,sM,sB};
    double TN[3]={tA,tM,tB};

    struct NL{R27AxisLattice x,y,z;};
    NL Ls[3];
    Vec3 Cq[3],Wq[3];

    for(int j=0;j<3;++j){
        Vec3 Ct=PN[j]+vbase*(SN[j]-TN[j]);
        Vec3 Wt=VN[j]*alpha+vbase*(alpha-1.0);
        Vec3 di=Wt*(h/15.0);
        long long kmn=-7LL*Ks,kmx=7LL*Ks;
        if(j==0)kmn=std::min(kmn,-7LL*Ke);
        if(j==2)kmx=std::max(kmx,7LL*Kx);

        Ls[j]={
            r27_axis_lattice(Ct.x,di.x,kmn,kmx),
            r27_axis_lattice(Ct.y,di.y,kmn,kmx),
            r27_axis_lattice(Ct.z,di.z,kmn,kmx)
        };
        Cq[j]=r27_grid_vec(Ls[j].x,Ls[j].y,Ls[j].z,0);
        Vec3 dq=r27_grid_vec(
            R27AxisLattice{Ls[j].x.e,0,Ls[j].x.d},
            R27AxisLattice{Ls[j].y.e,0,Ls[j].y.d},
            R27AxisLattice{Ls[j].z.e,0,Ls[j].z.d},1);
        Wq[j]=dq*(15.0/h);
    }

    const double vv=dot(vbase,vbase);
    const double sBq=tB+dot(vbase,Cq[2])/vv;
    const double alphaB=1.0+dot(vbase,Wq[2])/vv;
    if(!(fin(alphaB)&&alphaB>0))
        throw std::runtime_error("R36 final progress not monotone");

    const double sEq=sBq+0.5*(alphaB+1.0)*Tx;
    const double delay=tE-sEq;
    if(!(fin(delay)&&delay>=-4096*epsd()*std::max(1.0,std::fabs(tE))))
        throw std::runtime_error("R36 negative terminal delay");
    const double delayQ=std::max(0.0,delay);
    const Vec3 H=vbase*(-delayQ);

    Poly entry(16,Vec3{});
    for(int k=8;k<16;++k)
        entry[k]=r27_grid_vec(
            Ls[0].x,Ls[0].y,Ls[0].z,(long long)(k-15)*Ke);

    Poly f0(16);
    for(int k=0;k<8;++k)
        f0[k]=r27_grid_vec(Ls[0].x,Ls[0].y,Ls[0].z,(long long)k*Ks);
    for(int k=8;k<16;++k)
        f0[k]=r27_grid_vec(Ls[1].x,Ls[1].y,Ls[1].z,(long long)(k-15)*Ks);

    Poly f1(16);
    for(int k=0;k<8;++k)
        f1[k]=r27_grid_vec(Ls[1].x,Ls[1].y,Ls[1].z,(long long)k*Ks);
    for(int k=8;k<16;++k)
        f1[k]=r27_grid_vec(Ls[2].x,Ls[2].y,Ls[2].z,(long long)(k-15)*Ks);

    Poly exit(16,H);
    for(int k=0;k<8;++k)
        exit[k]=r27_grid_vec(
            Ls[2].x,Ls[2].y,Ls[2].z,(long long)k*Kx);
    Poly hold(16,H);

    C.seg0={supportStart,tA,tM,tB,tE};
    C.seg1={tA,tM,tB,tE,searchEnd};
    C.cp={std::move(entry),std::move(f0),std::move(f1),
          std::move(exit),std::move(hold)};

    C.terminal_hold_start=tE;
    C.terminal_offset=H;
    C.schedule_delay=delayQ;
    C.progress_scale=alpha;
    C.retime_entry_T=Te;C.retime_flow_T=Tf;C.retime_exit_T=Tx;
    C.exact_c7=r34_exact_c7_terminal_hold(C);

    C.dub.fill(0);C.max_ctrl=0.0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        Poly Dp=C.cp[s];int deg=(int)Dp.size()-1;
        for(auto q:Dp)C.max_ctrl=std::max(C.max_ctrl,norm(q));
        for(int r=1;r<=6;++r){
            Poly E2(Dp.size()-1);
            for(size_t i=0;i+1<Dp.size();++i)
                E2[i]=(Dp[i+1]-Dp[i])*(double(deg)/T);
            Dp.swap(E2);--deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(Dp));
        }
    }
    C.pos={P.P0,P.PM,P.P1};
    C.vel={P.V0,P.VM,P.V1};
    return C;
}



// ===========================================================================
// AF7R37 TRIGGER -> FREE-REJOIN EXACT-C7 SPATIAL SPLINE
//
// This phase removes the R36 "bend mainly inside the unsafe span" restriction.
//
// For a fixed rejoin deadline t1, the spatial correction is synthesized over
// the complete interval:
//
//      trigger/supportStart  ->  all unsafe components  ->  actual rejoin.
//
// Knots are generated from the exact dyadic representation grid and the
// component start/end geometry.  At every interior knot, correction position
// P_j and first derivative V_j are free.  Derivatives 2..7 are zero.  The
// endpoints are exactly zero through derivative 7.  Therefore the stored
// piecewise degree-15 correction is exact C7.
//
// This is phase-A path/time separation with identity time law s(t)=t.
// It intentionally removes R32-R36 retiming while the spatial geometry is
// being closed.  There is no old-clock catch-up.  A nontrivial C6 time law is
// attached only after this spatial primary closes the hard certificates.
// ===========================================================================

static inline double r37_align_down(double x,double h){
    if(!(fin(x)&&fin(h)&&h>0))
        return std::numeric_limits<double>::quiet_NaN();
    const double q=x/h;
    if(!(fin(q)&&std::fabs(q)<4503599627370496.0))
        return std::numeric_limits<double>::quiet_NaN();
    double k=std::floor(q);
    double y=k*h;
    if(y>x)y=(k-1.0)*h;
    return y;
}

static inline std::vector<double> r37_make_knots(
        const EpisodeCert&E,double t0,double t1,double h){
    std::vector<double>K;
    const double a=r33_align_up(t0,h);
    const double b=r37_align_down(t1,h);
    if(!(fin(a)&&fin(b)&&b>a))return K;
    K.push_back(a);

    for(const auto&c:E.comp){
        double ks=r37_align_down(c.start,h);
        double ke=r33_align_up(c.end,h);
        ks=std::max(ks,a+h);
        ke=std::min(ke,b-h);
        if(ks>a && ks<b)K.push_back(ks);
        if(ke>a && ke<b)K.push_back(ke);
    }
    K.push_back(b);

    std::sort(K.begin(),K.end());
    std::vector<double>U;
    const double tol=32*epsd()*std::max(1.0,std::max(std::fabs(a),std::fabs(b)));
    for(double x:K){
        if(U.empty()||x>U.back()+tol)U.push_back(x);
    }
    if(U.size()<2||!(U.back()>U.front()))U.clear();
    return U;
}

// Number of scalar node dofs: [P_j,V_j] for interior nodes j=1..N-2.
static inline int r37_scalar_dofs(const std::vector<double>&K){
    return K.size()>=3 ? 2*((int)K.size()-2) : 0;
}
static inline int r37_scalar_index(int node,int pv,int N){
    if(node<=0||node>=N-1)return -1;
    return 2*(node-1)+pv; // pv=0 position, pv=1 first derivative
}

static inline int r37_find_segment(
        const std::vector<double>&K,double t){
    if(K.size()<2)return -1;
    if(t<=K.front())return 0;
    if(t>=K.back())return (int)K.size()-2;
    auto it=std::upper_bound(K.begin(),K.end(),t);
    int s=(int)(it-K.begin())-1;
    return std::max(0,std::min((int)K.size()-2,s));
}

static inline std::vector<double> r37_scalar_basis(
        const std::vector<double>&K,double t){
    const int N=(int)K.size(),ns=r37_scalar_dofs(K);
    std::vector<double>w(ns,0.0);
    if(N<2||ns<=0)return w;

    const int s=r37_find_segment(K,t);
    if(s<0)return w;
    const double T=K[s+1]-K[s];
    if(!(fin(T)&&T>0))return w;
    double u=(t-K[s])/T;
    u=std::max(0.0,std::min(1.0,u));

    const int nodes[2]={s,s+1};
    for(int a=0;a<2;++a){
        const int node=nodes[a];
        for(int pv=0;pv<2;++pv){
            const int q=r37_scalar_index(node,pv,N);
            if(q<0)continue;
            std::array<double,8>J0{},J1{};
            if(a==0)J0[pv]=1.0;
            else J1[pv]=1.0;
            w[q]=r36_scalar_bezier_eval(
                scalar_control_from_jets(J0,J1,T),u);
        }
    }
    return w;
}

static inline std::vector<double> r37_spatial_hessian(
        const std::vector<double>&K,double cableL,double gravity){
    const int N=(int)K.size(),ns=r37_scalar_dofs(K);
    if(N<3||ns<=0||!(fin(cableL)&&cableL>0&&fin(gravity)&&gravity>0))
        throw std::runtime_error("R37 invalid Hessian request");

    // Dimensionally grounded towing metric; same natural time used by the
    // earlier towing primary. No empirical smoothing weights are introduced.
    const double tau2=cableL/gravity;
    const double wr[4]={1.0/tau2,1.0,tau2,tau2*tau2};
    std::vector<double>Hs(ns*ns,0.0);

    for(int s=0;s<N-1;++s){
        const double T=K[s+1]-K[s];
        if(!(fin(T)&&T>0))throw std::runtime_error("R37 nonpositive segment");

        std::vector<int>qv;
        std::vector<std::vector<double>>CP;
        for(int a=0;a<2;++a){
            int node=(a==0)?s:s+1;
            for(int pv=0;pv<2;++pv){
                int q=r37_scalar_index(node,pv,N);
                if(q<0)continue;
                std::array<double,8>J0{},J1{};
                if(a==0)J0[pv]=1.0;else J1[pv]=1.0;
                qv.push_back(q);
                CP.push_back(scalar_control_from_jets(J0,J1,T));
            }
        }

        for(int r=0;r<=3;++r){
            const int deg=15-r,ncp=16-r;
            std::vector<std::vector<double>>D(CP.size());
            double ff=1.0;
            for(int q=0;q<r;++q)ff*=double(15-q);
            for(size_t i=0;i<CP.size();++i){
                D[i]=diff_scalar(CP[i],r);
                const double sc=(r==0)?1.0:ff/std::pow(T,r);
                for(double&v:D[i])v*=sc;
            }
            for(size_t i=0;i<D.size();++i)for(size_t j=0;j<D.size();++j){
                double q=0.0;
                for(int aa=0;aa<ncp;++aa)for(int bb=0;bb<ncp;++bb)
                    q+=D[i][aa]*gram(deg,aa,bb)*D[j][bb];
                Hs[qv[i]*ns+qv[j]]+=wr[r]*T*q;
            }
        }
    }

    // Two coordinates in the progress-perpendicular plane.
    const int n=2*ns;
    std::vector<double>H(n*n,0.0);
    for(int i=0;i<ns;++i)for(int j=0;j<ns;++j){
        H[(2*i+0)*n+(2*j+0)]=Hs[i*ns+j];
        H[(2*i+1)*n+(2*j+1)]=Hs[i*ns+j];
    }
    return H;
}


// ===========================================================================
// AF7R45 TANGENT-SUPPORT CORRIDOR
//
// The historical R37 synthesis used one fixed lateral supporting normal for
// the complete unsafe episode.  For an affine nominal line this makes the
// required correction nearly constant through the whole episode, producing
// an artificial "translate sideways -> parallel pass -> translate back"
// shape.
//
// R45 keeps the SAME selected bypass side e_lat, but allows the separating
// support plane to rotate in the plane spanned by nominal progress e_prog and
// e_lat:
//
//      m(u) = e_lat + u e_prog.
//
// R37 correction lives exactly in e_prog^perp, hence
//
//      m(u)' d(t) = e_lat' d(t)
//
// for every R37 correction.  Therefore u may be optimized solely on the
// obstacle/support side without adding any progress correction.
//
// For each dt cell we minimize the exact-support sufficient translation
//
//   b(u) = max{
//      h_O(m)+rho_L ||m|| - m'p_L,
//      h_O(m)+rho_Q ||m|| - m'p_Q,
//      h_O(m)+rho_C ||m|| - m'p_Q,
//      h_O(m)+rho_C ||m|| - m'p_L }
//
// where h_O is the exact support function of the oriented ellipsoid and the
// radii include the same tracking/dmargin plus half-cell sweep used by the
// historical corridor.  b(u) is convex in the scalar u (maximum of convex
// support functions).  A derivative-subgradient bracket/bisection computes
// its minimum.  If numerical bracketing is inconclusive, synthesis falls back
// to u=0 (the historical fixed normal); this can only increase conservatism.
//
// This corridor is synthesis guidance only.  Final authority remains the
// unchanged exact-C7 -> yaw-C6 -> Stage-A/B finite-sensing -> rejoin ->
// continuous PAYLOAD/UAV/CABLE whole-body certificate stack.
// ===========================================================================
struct R45CorridorDiag {
    int intervals=0;
    int zero_offset_intervals=0;
    int search_fallbacks=0;
    double fixed_peak=0.0;
    double tangent_peak=0.0;
};

struct R45ConvexEval {
    bool ok=false;
    double value=std::numeric_limits<double>::infinity();
    double dmin=std::numeric_limits<double>::quiet_NaN();
    double dmax=std::numeric_limits<double>::quiet_NaN();
};

static inline R45ConvexEval r45_corridor_eval_u(
        const Vec3&pL,const Vec3&pQ,const Obstacle&o,
        double rhoL,double rhoQ,double rhoC,
        Vec3 eprog,Vec3 elat,double u){
    R45ConvexEval R;
    if(!(finite(pL)&&finite(pQ)&&finite(eprog)&&finite(elat)&&
            fin(rhoL)&&fin(rhoQ)&&fin(rhoC)&&
            rhoL>=0&&rhoQ>=0&&rhoC>=0&&fin(u))) return R;

    Vec3 m=elat+eprog*u;
    const double nm=norm(m);
    if(!(fin(nm)&&nm>0)) return R;

    Vec3 rm=tmul(o.R,m);
    Vec3 rp=tmul(o.R,eprog);
    Vec3 am{o.axes.x*rm.x,o.axes.y*rm.y,o.axes.z*rm.z};
    Vec3 ap{o.axes.x*rp.x,o.axes.y*rp.y,o.axes.z*rp.z};
    const double na=norm(am);
    if(!(fin(na)&&na>0)) return R;

    const double hObs=dot(m,o.c)+na;
    const double dhObs=dot(eprog,o.c)+dot(am,ap)/na;
    const double dnm=dot(m,eprog)/nm;

    const double commonL=hObs+rhoL*nm;
    const double commonQ=hObs+rhoQ*nm;
    const double commonC=hObs+rhoC*nm;
    const double dcommonL=dhObs+rhoL*dnm;
    const double dcommonQ=dhObs+rhoQ*dnm;
    const double dcommonC=dhObs+rhoC*dnm;

    const double v[4]={
        commonL-dot(m,pL),
        commonQ-dot(m,pQ),
        commonC-dot(m,pQ),
        commonC-dot(m,pL)
    };
    const double d[4]={
        dcommonL-dot(eprog,pL),
        dcommonQ-dot(eprog,pQ),
        dcommonC-dot(eprog,pQ),
        dcommonC-dot(eprog,pL)
    };

    double vmax=v[0];
    for(int k=1;k<4;++k)vmax=std::max(vmax,v[k]);
    if(!fin(vmax)) return R;

    const double sc=std::max({1.0,std::fabs(v[0]),std::fabs(v[1]),
        std::fabs(v[2]),std::fabs(v[3]),std::fabs(vmax)});
    const double guard=512*epsd()*sc;
    double dlo=std::numeric_limits<double>::infinity();
    double dhi=-std::numeric_limits<double>::infinity();
    for(int k=0;k<4;++k){
        if(v[k]>=vmax-guard){
            dlo=std::min(dlo,d[k]);
            dhi=std::max(dhi,d[k]);
        }
    }
    if(!(fin(dlo)&&fin(dhi))) return R;
    R.ok=true;R.value=vmax;R.dmin=dlo;R.dmax=dhi;
    return R;
}

static inline double r45_min_lateral_requirement(
        const Vec3&pL,const Vec3&pQ,const Obstacle&o,
        double rhoL,double rhoQ,double rhoC,
        Vec3 eprog,Vec3 elat,bool&fallback){
    fallback=false;
    auto F=[&](double u){
        return r45_corridor_eval_u(
            pL,pQ,o,rhoL,rhoQ,rhoC,eprog,elat,u);
    };

    R45ConvexEval E0=F(0.0);
    if(!E0.ok){fallback=true;return std::numeric_limits<double>::infinity();}
    if(E0.value<=0.0)return 0.0;
    if(E0.dmin<=0.0 && E0.dmax>=0.0)return E0.value;

    const int dir=(E0.dmax<0.0)?+1:-1;
    double uprev=0.0;
    R45ConvexEval Eprev=E0;
    double step=1.0;
    double ulo=std::numeric_limits<double>::quiet_NaN();
    double uhi=std::numeric_limits<double>::quiet_NaN();
    bool bracket=false;

    // Purely numerical bracketing.  The iteration cap protects binary64
    // overflow and is not a physical/trajectory tuning parameter.
    for(int it=0;it<60;++it){
        const double ucur=double(dir)*step;
        R45ConvexEval Ecur=F(ucur);
        if(!Ecur.ok){break;}
        if(Ecur.value<=0.0)return 0.0;

        if(dir>0){
            // At u=0 the subgradient is strictly negative.  Once the right
            // endpoint has a nonnegative subgradient, convexity brackets the
            // global minimizer.
            if(Ecur.dmax>=0.0){
                ulo=uprev;uhi=ucur;bracket=true;break;
            }
        }else{
            // At u=0 the subgradient is strictly positive.  Moving left until
            // a nonpositive subgradient brackets the global minimizer.
            if(Ecur.dmin<=0.0){
                ulo=ucur;uhi=uprev;bracket=true;break;
            }
        }

        uprev=ucur;Eprev=Ecur;
        step*=2.0;
        if(!fin(step))break;
    }

    if(!bracket){
        // Fail conservative: retain historical fixed-lateral requirement.
        fallback=true;
        return E0.value;
    }

    double best=std::min(E0.value,Eprev.value);
    for(int it=0;it<80;++it){
        const double um=0.5*(ulo+uhi);
        if(!(fin(um)&&um>ulo&&um<uhi))break;
        R45ConvexEval Em=F(um);
        if(!Em.ok){fallback=true;return E0.value;}
        if(Em.value<=0.0)return 0.0;
        best=std::min(best,Em.value);

        if(Em.dmin<=0.0 && Em.dmax>=0.0){
            best=std::min(best,Em.value);
            break;
        }
        if(Em.dmax<0.0) ulo=um;
        else if(Em.dmin>0.0) uhi=um;
        else break;

        const double tol=256*epsd()*
            std::max({1.0,std::fabs(ulo),std::fabs(uhi)});
        if((uhi-ulo)<=tol)break;
    }
    if(!fin(best)){fallback=true;return E0.value;}
    return std::max(0.0,best);
}


// Historical R42 fixed-separator corridor retained only as a conservative
// synthesis fallback if the scalable R45 QP exchange solver is numerically
// inconclusive.  It has no independent safety authority.
static inline std::vector<Row> r45_fixed_corridor_rows(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,const EpisodeCert&E,int ei,Vec3 n,
        const std::vector<double>&K,const R36Plane&B){
    const int ns=r37_scalar_dofs(K),nvar=2*ns;
    const auto&ep=E.ep[ei];
    n=normalized(n);
    const auto&o=obs[ep.obs];

    Vec3 rn=tmul(o.R,n);
    Vec3 ar{o.axes.x*rn.x,o.axes.y*rn.y,o.axes.z*rn.z};
    const double hObs=dot(n,o.c)+norm(ar);
    const double VC=std::max(base.VL,base.VQ);

    std::vector<std::pair<double,double>>sb;
    double ta=ep.start,te=ep.end;
    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        const double tb=std::min(te,ta+L.dt);
        const double tc=0.5*(ta+tb),half=0.5*(tb-ta);
        auto x=ref.eval(tc);
        Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};
        const double nf=norm(sf);
        if(!(fin(nf)&&nf>0))throw std::runtime_error("R45R2 fixed fallback flatness singular");
        Vec3 q=sf*(-1.0/nf),pQ=pL-q*L.L;
        const double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        const double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        const double rhoC=L.rC+L.dmargin+L.tracking+VC*half;
        double req=std::max({
            hObs+rhoL-dot(n,pL),
            hObs+rhoQ-dot(n,pQ),
            hObs+rhoC-std::min(dot(n,pQ),dot(n,pL))
        });
        req+=64*epsd()*std::max(1.0,std::fabs(req));
        sb.push_back({ta,req});sb.push_back({tb,req});ta=tb;
    }
    auto hull=upper_hull(sb);
    const double nb1=dot(n,B.b1),nb2=dot(n,B.b2);
    std::vector<Row>rows;
    for(auto [t,b]:hull){
        auto w=r37_scalar_basis(K,t);
        Row R;R.a.assign(nvar,0.0);R.b=b;
        for(int q=0;q<ns;++q){
            R.a[2*q+0]=w[q]*nb1;
            R.a[2*q+1]=w[q]*nb2;
        }
        rows.push_back(std::move(R));
    }
    return rows;
}

static inline std::vector<Row> r37_corridor_rows(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,const EpisodeCert&E,int ei,Vec3 n,
        const std::vector<double>&K,const R36Plane&B,Vec3 vbase,
        R45CorridorDiag*diag=nullptr){
    const int ns=r37_scalar_dofs(K),nvar=2*ns;
    const auto&ep=E.ep[ei];
    const auto&o=obs[ep.obs];

    Vec3 eprog=normalized(vbase);
    // Ensure the selected bypass direction lies exactly in the R37 correction
    // plane.  lateral_normal() is already constrained to progress^perp; this
    // projection only removes binary64 residue.
    Vec3 elat=n-eprog*dot(n,eprog);
    const double nl=norm(elat);
    if(!(fin(nl)&&nl>0))
        throw std::runtime_error("R45 degenerate lateral direction");
    elat=elat/nl;

    const double VC=std::max(base.VL,base.VQ);
    std::vector<std::pair<double,double>>sb;
    double ta=ep.start,te=ep.end;
    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        const double tb=std::min(te,ta+L.dt);
        const double tc=0.5*(ta+tb),half=0.5*(tb-ta);
        auto x=ref.eval(tc);
        Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};
        const double nf=norm(sf);
        if(!(fin(nf)&&nf>0))
            throw std::runtime_error("R45 flatness singular");
        Vec3 q=sf*(-1.0/nf),pQ=pL-q*L.L;

        const double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        const double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        const double rhoC=L.rC+L.dmargin+L.tracking+VC*half;

        R45ConvexEval F0=r45_corridor_eval_u(
            pL,pQ,o,rhoL,rhoQ,rhoC,eprog,elat,0.0);
        if(!F0.ok)
            throw std::runtime_error("R45 fixed support evaluation failed");

        bool fb=false;
        double req=r45_min_lateral_requirement(
            pL,pQ,o,rhoL,rhoQ,rhoC,eprog,elat,fb);
        if(!fin(req))
            throw std::runtime_error("R45 tangent support minimization failed");

        const double fixedReq=std::max(0.0,F0.value);
        if(diag){
            ++diag->intervals;
            if(req<=0.0)++diag->zero_offset_intervals;
            if(fb)++diag->search_fallbacks;
            diag->fixed_peak=std::max(diag->fixed_peak,fixedReq);
            diag->tangent_peak=std::max(diag->tangent_peak,req);
        }

        if(req>0.0)
            req+=64*epsd()*std::max(1.0,std::fabs(req));

        // Same lateral coordinate for every rotated tangent plane because the
        // R37 correction has no eprog component.  Therefore the scalar
        // upper-hull compression remains valid even though the obstacle-side
        // support plane rotates with time.
        sb.push_back({ta,req});
        sb.push_back({tb,req});
        ta=tb;
    }

    auto hull=upper_hull(sb);
    const double nb1=dot(elat,B.b1),nb2=dot(elat,B.b2);
    std::vector<Row>rows;
    for(auto [t,b]:hull){
        auto w=r37_scalar_basis(K,t);
        Row R;R.a.assign(nvar,0.0);R.b=b;
        for(int q=0;q<ns;++q){
            R.a[2*q+0]=w[q]*nb1;
            R.a[2*q+1]=w[q]*nb2;
        }
        rows.push_back(std::move(R));
    }
    return rows;
}

struct R37Path{
    bool ok=false;
    std::vector<double>knots;
    std::vector<Vec3>P,V; // includes fixed zero endpoints
    double cost=0.0,slack=0.0;
    std::vector<int>signs;
};

static inline R37Path r37_decode_path(
        const QPSol&q,const std::vector<double>&K,
        const R36Plane&B,const std::vector<int>&signs){
    R37Path P;
    const int N=(int)K.size(),ns=r37_scalar_dofs(K);
    if(!q.ok||N<3||(int)q.z.size()!=2*ns)return P;
    P.knots=K;P.P.assign(N,Vec3{});P.V.assign(N,Vec3{});
    for(int node=1;node<N-1;++node){
        int qp=r37_scalar_index(node,0,N);
        int qv=r37_scalar_index(node,1,N);
        P.P[node]=B.b1*q.z[2*qp]+B.b2*q.z[2*qp+1];
        P.V[node]=B.b1*q.z[2*qv]+B.b2*q.z[2*qv+1];
    }
    P.cost=q.cost;P.slack=q.slack;P.signs=signs;P.ok=true;
    return P;
}

// AF7R38 constructive exact-real C7 proof
//
// R37's historical r31_exact_c7_stored() checker differentiated each segment
// in binary64 and compared the rounded endpoint derivatives with ==.
// For unequal exact durations K_L*h and K_R*h this can false-reject:
//
//   15*(K_L*d*2^e)/(K_L*h) = 15*d*2^e/h
//   15*(K_R*d*2^e)/(K_R*h) = 15*d*2^e/h
//
// as exact real numbers, while the two floating divisions can round
// differently.
//
// The actual stored R37 control points have a stronger constructive proof:
// - h is an exact power of two;
// - every segment duration is verified exactly as integer K*h;
// - one dyadic node lattice (p,d,e) is shared by both adjacent segments;
// - r27_grid() maps every |integer|<2^52 times 2^e exactly to binary64;
// - the endpoint control stencils are exact arithmetic progressions.
//
// Hence position is identical at each join, first derivatives are exactly
// equal by cancellation of K, derivatives 2..7 are exactly zero, and the
// outer 8-point zero stencils make derivatives 0..7 exactly zero at support
// endpoints. This is a theorem about the stored polynomials, not a tolerance.
//
static inline Candidate r37_build_candidate(
        const R37Path&P,double h,Vec3 vbase){
    Candidate C;
    C.family=7;C.constant_translation=false;
    C.cost=P.cost;C.signs=P.signs;C.minSlack=P.slack;
    C.trigger_to_free_rejoin=true;
    C.identity_time_law=true;
    C.progress_retimed=false;
    C.progress_scale=1.0;
    C.schedule_delay=0.0;
    C.terminal_hold=false;
    C.actual_rejoin_time=P.knots.back();
    C.spatial_knot_count=(int)P.knots.size();

    const int N=(int)P.knots.size();
    if(N<3)throw std::runtime_error("R37 insufficient knots");

    struct NL{R27AxisLattice x,y,z;};
    std::vector<NL>Latt(N);
    for(int node=1;node<N-1;++node){
        const double Tm=P.knots[node]-P.knots[node-1];
        const double Tp=P.knots[node+1]-P.knots[node];
        long long Km=(long long)std::llround(Tm/h);
        long long Kp=(long long)std::llround(Tp/h);
        if(Km<1||Kp<1||Tm!=(double)Km*h||Tp!=(double)Kp*h)
            throw std::runtime_error("R37 knot not on exact grid");

        Vec3 di=P.V[node]*(h/15.0);
        Latt[node]={
            r27_axis_lattice(P.P[node].x,di.x,-7LL*Km,7LL*Kp),
            r27_axis_lattice(P.P[node].y,di.y,-7LL*Km,7LL*Kp),
            r27_axis_lattice(P.P[node].z,di.z,-7LL*Km,7LL*Kp)
        };
    }

    C.seg0.clear();C.seg1.clear();C.cp.clear();
    C.pos=P.P;C.vel=P.V;
    for(int s=0;s<N-1;++s){
        const double T=P.knots[s+1]-P.knots[s];
        const long long K=(long long)std::llround(T/h);
        if(K<1||T!=(double)K*h)
            throw std::runtime_error("R37 segment grid mismatch");

        Poly cp(16,Vec3{});
        if(s>0){
            for(int k=0;k<8;++k)
                cp[k]=r27_grid_vec(
                    Latt[s].x,Latt[s].y,Latt[s].z,(long long)k*K);
        }
        if(s+1<N-1){
            for(int k=8;k<16;++k)
                cp[k]=r27_grid_vec(
                    Latt[s+1].x,Latt[s+1].y,Latt[s+1].z,
                    (long long)(k-15)*K);
        }

        C.seg0.push_back(P.knots[s]);
        C.seg1.push_back(P.knots[s+1]);
        C.cp.push_back(std::move(cp));
    }

    // Constructive proof authority for the R37/R38 representation.
    // All proof preconditions are checked by the exact-grid/lattice
    // construction above; any violation throws before this point.
    C.exact_c7_constructive=true;
    C.exact_c7=true;

    // Retain the historical floating endpoint checker only as a diagnostic
    // so false rejections remain visible during migration.
    C.legacy_float_endpoint_checker_pass=r31_exact_c7_stored(C);

    C.dub.fill(0);C.max_ctrl=0.0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        Poly D=C.cp[s];int deg=15;
        for(auto q:D)C.max_ctrl=std::max(C.max_ctrl,norm(q));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }

    // Path-tangent yaw theorem is applied by the ordinary family-7 gate.
    (void)vbase;
    return C;
}


// -------------------- build candidate control polygons ---------------------
static inline Candidate build_candidate(const std::vector<Component>&comp,double t0,double t1,
                                        const std::vector<double>&z,double cost,const std::vector<int>&signs,double minSlack,
                                        int family=0){
    Candidate C;C.cost=cost;C.signs=signs;C.minSlack=minSlack;C.family=family;C.constant_translation=false;
    // AF7R42: build_candidate is structurally exact C7. Each degree-15
    // transition is fixed by endpoint jets 0..7, while every unsafe affine
    // span has identically zero derivatives of order 2..7. This flag records
    // that construction fact; all downstream hard certificates are unchanged.
    C.exact_c7=true;
    int nc=(int)comp.size(),nslot=2*nc;if((int)z.size()!=3*nslot)throw std::runtime_error("candidate z size");
    C.pos.resize(nc);C.vel.resize(nc);
    for(int j=0;j<nc;++j){C.pos[j]={z[3*(2*j)],z[3*(2*j)+1],z[3*(2*j)+2]};
                           C.vel[j]={z[3*(2*j+1)],z[3*(2*j+1)+1],z[3*(2*j+1)+2]};}
    auto zerojets=[](){std::array<Vec3,8>a{};return a;};
    // start transition
    {
        std::array<Vec3,8>a=zerojets(),b=zerojets();b[0]=C.pos[0];b[1]=C.vel[0];
        C.seg0.push_back(t0);C.seg1.push_back(comp[0].start);C.cp.push_back(vector_control_from_jets(a,b,comp[0].start-t0));
    }
    for(int j=0;j<nc;++j){
        double T=comp[j].end-comp[j].start;Poly p(16);
        for(int k=0;k<16;++k)p[k]=C.pos[j]+C.vel[j]*(T*(double(k)/15.0));
        C.seg0.push_back(comp[j].start);C.seg1.push_back(comp[j].end);C.cp.push_back(std::move(p));
        if(j<nc-1){
            std::array<Vec3,8>a=zerojets(),b=zerojets();
            a[0]=C.pos[j]+C.vel[j]*(comp[j].end-comp[j].start);a[1]=C.vel[j];
            b[0]=C.pos[j+1];b[1]=C.vel[j+1];
            double Tt=comp[j+1].start-comp[j].end;
            C.seg0.push_back(comp[j].end);C.seg1.push_back(comp[j+1].start);C.cp.push_back(vector_control_from_jets(a,b,Tt));
        }
    }
    {
        int j=nc-1;std::array<Vec3,8>a=zerojets(),b=zerojets();
        a[0]=C.pos[j]+C.vel[j]*(comp[j].end-comp[j].start);a[1]=C.vel[j];
        C.seg0.push_back(comp[j].end);C.seg1.push_back(t1);C.cp.push_back(vector_control_from_jets(a,b,t1-comp[j].end));
    }
    C.dub.fill(0);C.max_ctrl=0;
    for(size_t s=0;s<C.cp.size();++s){
        double T=C.seg1[s]-C.seg0[s];Poly D=C.cp[s];int deg=15;
        for(auto v:D)C.max_ctrl=std::max(C.max_ctrl,norm(v));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);for(size_t i=0;i+1<D.size();++i)E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }
    return C;
}

// ---------------------------------------------------------------------------
// AF7_TOWING_C7_SPEED_OPT_SPLIT_R17
// ---------------------------------------------------------------------------
// AF7_EXACT_C7_DYADIC_LATTICE_R22C
struct R22AxisLattice{int e=0;long long p=0,d=0;};
static inline R22AxisLattice r22_axis_lattice(double pm,double di){
    if(!(fin(pm)&&fin(di)))throw std::runtime_error("R22C nonfinite lattice target");
    const double scale=std::max({1.0,std::fabs(pm),7.0*std::fabs(di)});
    const int es=std::ilogb(scale);
    const int e=es-50;
    const double ps=std::ldexp(pm,-e),ds=std::ldexp(di,-e);
    if(!(fin(ps)&&fin(ds)&&std::fabs(ps)<4503599627370496.0&&std::fabs(ds)<4503599627370496.0))
        throw std::runtime_error("R22C dyadic lattice integer range");
    const long long ip=std::llround(ps),id=std::llround(ds);
    const long long ap=ip<0?-ip:ip,ad=id<0?-id:id;
    if(ap+7*ad>=4503599627370496LL)
        throw std::runtime_error("R22C dyadic lattice exactness range");
    return {e,ip,id};
}
static inline double r22_grid(long long q,int e){
    if(q<=-4503599627370496LL||q>=4503599627370496LL)
        throw std::runtime_error("R22C dyadic grid overflow");
    const double x=std::ldexp((double)q,e);
    if(!fin(x))throw std::runtime_error("R22C dyadic grid nonfinite");
    return x;
}
static inline Vec3 r22_grid_vec(const R22AxisLattice&x,const R22AxisLattice&y,
                              const R22AxisLattice&z,int k){
    return {r22_grid(x.p+(long long)k*x.d,x.e),
            r22_grid(y.p+(long long)k*y.d,y.e),
            r22_grid(z.p+(long long)k*z.d,z.e)};
}
static inline void r17_append_split_transition(Candidate&C,Vec3 p0,Vec3 p1,
                                               double ta,double tb,double alpha){
    const double T=tb-ta;
    if(!(fin(T)&&T>0&&fin(alpha)&&alpha>=15.0/8.0&&alpha<=2.0))
        throw std::runtime_error("R17 invalid split transition");
    double tm=ta+0.5*(tb-ta);
    bool splitExact=false;double hL=0,hR=0;
    if(tm>ta&&tm<tb){hL=tm-ta;hR=tb-tm;splitExact=(hL==hR&&fin(hL)&&hL>0);}
    if(!splitExact){
        double lo=tm,hi=tm;
        for(int it=0;it<64&&!splitExact;++it){
            lo=std::nextafter(lo,ta);
            if(lo>ta&&lo<tb){hL=lo-ta;hR=tb-lo;if(hL==hR&&fin(hL)&&hL>0){tm=lo;splitExact=true;break;}}
            hi=std::nextafter(hi,tb);
            if(hi>ta&&hi<tb){hL=hi-ta;hR=tb-hi;if(hL==hR&&fin(hL)&&hL>0){tm=hi;splitExact=true;break;}}
        }
    }
    if(!splitExact)throw std::runtime_error("R22C split halves not exactly representable");
    const Vec3 pm=(p0+p1)*0.5;
    // Ideal midpoint Bezier increment is v_mid*h/15 = alpha*(p1-p0)/30.
    // It is quantized only to an exact dyadic representability lattice;
    // the resulting stored polygon itself is what the hard gates certify.
    const Vec3 di=(p1-p0)*(alpha/30.0);
    const auto ax=r22_axis_lattice(pm.x,di.x);
    const auto ay=r22_axis_lattice(pm.y,di.y);
    const auto az=r22_axis_lattice(pm.z,di.z);
    Poly Lp(16),Rp(16);
    for(int k=0;k<8;++k){Lp[k]=p0;Rp[15-k]=p1;}
    for(int k=0;k<8;++k){
        Lp[15-k]=r22_grid_vec(ax,ay,az,-k);
        Rp[k]=r22_grid_vec(ax,ay,az,k);
    }
    C.seg0.push_back(ta);C.seg1.push_back(tm);C.cp.push_back(std::move(Lp));
    C.seg0.push_back(tm);C.seg1.push_back(tb);C.cp.push_back(std::move(Rp));
}

static inline void r17_finalize_candidate_bounds(Candidate&C){
    C.dub.fill(0);C.max_ctrl=0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        if(!(fin(T)&&T>0))throw std::runtime_error("R17 nonpositive segment time");
        Poly D=C.cp[s];int deg=15;
        for(auto v:D)C.max_ctrl=std::max(C.max_ctrl,norm(v));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }
}

static inline Candidate r17_build_split_constant(const std::vector<Component>&comp,
        double t0,double t1,const std::vector<double>&z,double cost,
        const std::vector<int>&signs,double minSlack,double alpha){
    Candidate C;C.cost=cost;C.signs=signs;C.minSlack=minSlack;
    C.family=5;C.constant_translation=true;C.exact_c7=true;
    const int nc=(int)comp.size(),nslot=2*nc;
    if((int)z.size()!=3*nslot||nc<1)throw std::runtime_error("R17 candidate z size");
    C.pos.resize(nc);C.vel.assign(nc,Vec3{});
    for(int j=0;j<nc;++j){
        C.pos[j]={z[3*(2*j)],z[3*(2*j)+1],z[3*(2*j)+2]};
        const Vec3 vz{z[3*(2*j+1)],z[3*(2*j+1)+1],z[3*(2*j+1)+2]};
        const double tv=4096*epsd()*std::max(1.0,norm(C.pos[j]));
        if(norm(vz)>tv)throw std::runtime_error("R17 requires zero unsafe-span velocity");
    }
    r17_append_split_transition(C,Vec3{},C.pos[0],t0,comp[0].start,alpha);
    for(int j=0;j<nc;++j){
        Poly p(16,C.pos[j]);
        C.seg0.push_back(comp[j].start);C.seg1.push_back(comp[j].end);
        C.cp.push_back(std::move(p));
        if(j<nc-1)
            r17_append_split_transition(C,C.pos[j],C.pos[j+1],
                                        comp[j].end,comp[j+1].start,alpha);
    }
    r17_append_split_transition(C,C.pos.back(),Vec3{},comp.back().end,t1,alpha);
    r17_finalize_candidate_bounds(C);
    return C;
}

static inline double r17_speed_opt_alpha(const std::vector<Component>&comp,
        double t0,double t1,const std::vector<Vec3>&p,const SpeedCert&base,const Live&L){
    double A1=0,B2=0,C3=0;
    auto add=[&](Vec3 d,double T){
        if(!(fin(T)&&T>0))throw std::runtime_error("R17 invalid transition duration");
        const double D=norm(d);
        A1=std::max(A1,D/T);
        B2=std::max(B2,D/(T*T));
        C3=std::max(C3,D/(T*T*T));
    };
    add(p[0],comp[0].start-t0);
    for(size_t j=0;j+1<p.size();++j)
        add(p[j+1]-p[j],comp[j+1].start-comp[j].end);
    add(p.back()*(-1.0),t1-comp.back().end);
    if(!(fin(A1)&&fin(B2)&&fin(C3))||A1<=0||B2<=0)return 2.0;
    const double rad=7.0*L.L*(728.0*C3*base.sigma+28.0*B2*base.J)/A1;
    if(!(fin(rad)&&rad>=0))return 2.0;
    const double target=std::sqrt(rad);
    double h=(base.sigma-target)/(28.0*B2);
    h=std::max(1.0,std::min(15.0/8.0,h));
    double alpha=(15.0-h)/7.0;
    alpha=std::max(15.0/8.0,std::min(2.0,alpha));
    return alpha;
}

// ---------------------------- Stage A ---------------------------------------
struct DLeaf {Poly C;double u0=0,u1=1,ub=0,lb=0;};
static inline DLeaf make_dleaf(const Poly&C,double u0,double u1){
    DLeaf l;l.C=C;l.u0=u0;l.u1=u1;l.ub=poly_norm_ub(C);
    l.lb=std::max({norm(C.front()),norm(C.back()),norm(decasteljau(C,0.5))});return l;
}
struct TightRes {bool certified=false,impossible=false,repr=false;int splits=0;std::array<double,3>U{},Lw{};};
static inline TightRes tighten_stageA(const Candidate&C,const SpeedCert&base,double cableL,double VQlimit){
    TightRes R;std::array<std::vector<DLeaf>,3> leaves;
    for(int r=1;r<=3;++r){
        for(size_t s=0;s<C.cp.size();++s){
            double T=C.seg1[s]-C.seg0[s];Poly D=deriv_poly(C.cp[s],T,r);if(D.empty())return R;
            leaves[r-1].push_back(make_dleaf(D,0,1));
        }
    }
    while(true){
        for(int r=0;r<3;++r){R.U[r]=0;R.Lw[r]=0;for(auto&l:leaves[r]){R.U[r]=std::max(R.U[r],l.ub);R.Lw[r]=std::max(R.Lw[r],l.lb);}}
        double sig=base.sigma-R.U[1];
        double vq=sig>0?base.VL+R.U[0]+cableL*(base.J+R.U[2])/sig:std::numeric_limits<double>::infinity();
        if(fin(vq)&&vq<VQlimit){R.certified=true;return R;}
        if(R.Lw[1]>=base.sigma){R.impossible=true;return R;}
        double sigp=base.sigma-R.Lw[1];
        double lo=base.VL+R.Lw[0]+cableL*(base.J+R.Lw[2])/sigp;
        if(fin(lo)&&lo>=VQlimit){R.impossible=true;return R;}
        bool any=false;
        for(int r=0;r<3;++r){
            double ur=0;for(auto&l:leaves[r])ur=std::max(ur,l.ub);double tol=64*epsd()*std::max(1.0,std::fabs(ur));
            std::vector<DLeaf> next;
            for(auto&l:leaves[r]){
                if(l.ub>=ur-tol){
                    double um=0.5*(l.u0+l.u1);if(um>l.u0&&um<l.u1){
                        auto sp=split_half(l.C);next.push_back(make_dleaf(sp.first,l.u0,um));next.push_back(make_dleaf(sp.second,um,l.u1));++R.splits;any=true;continue;
                    }
                }
                next.push_back(l);
            }
            leaves[r].swap(next);
        }
        if(!any){R.repr=true;return R;}
    }
}
static inline SpeedCert stageA_speed(const TightRes&T,const SpeedCert&base,double L){
    SpeedCert s;if(!T.certified)return s;
    s.certified=true;s.stageA=true;s.VL=base.VL+T.U[0];s.A=base.A+T.U[1];s.J=base.J+T.U[2];
    s.sigma=base.sigma-T.U[1];if(!(s.sigma>0)){s.certified=false;return s;}
    s.qdot=s.J/s.sigma;s.VQ=s.VL+L*s.qdot;return s;
}

// ---------------------------- Stage B ---------------------------------------
// AF7_TOWMAP_R4C_CORE_WITNESS -- rejection-only diagnostics.
// class: 1 ENTRY_C7, 2 UNSAFE_AFFINE, 3 INTER_C7, 4 EXIT_REJOIN_C7.
struct TowMapWitnessSummary {
    int count=0;
    double min_vq=std::numeric_limits<double>::infinity();
    double min_t=std::numeric_limits<double>::quiet_NaN();
    int min_seg=-1,min_class=0;
    double max_vq=-std::numeric_limits<double>::infinity();
    double max_t=std::numeric_limits<double>::quiet_NaN();
    int max_seg=-1,max_class=0;
};
static TowMapWitnessSummary G_towmap_primary_witness;
static TowMapWitnessSummary G_towmap_towing_witness;
static inline void towmap_core_reset_witness(){
    G_towmap_primary_witness=TowMapWitnessSummary{};
    G_towmap_towing_witness=TowMapWitnessSummary{};
}
static inline void towmap_core_record_witness(int family,int seg,int nseg,double t,double vq){
    if(!(fin(t)&&fin(vq))||nseg<1)return;
    TowMapWitnessSummary* W=nullptr;
    if(family==1)W=&G_towmap_primary_witness;
    else if(family==3)W=&G_towmap_towing_witness;
    else return;
    int cls=0;
    // AF7_PRIMARY_HEADING_R7_MAP_R9B: family-3 uses split C7 transitions.
    if(family==3 && nseg>=5 && ((nseg-2)%3)==0){
        int nc=(nseg-2)/3;
        if(seg<=1)cls=1;                           // ENTRY_A / ENTRY_B
        else if(seg>=3*nc)cls=4;                  // EXIT_A / EXIT_B
        else if(((seg-2)%3)==0)cls=2;             // true UNSAFE_AFFINE
        else cls=3;                               // INTER_A / INTER_B
    }else{
        if(seg==0)cls=1;
        else if(seg==nseg-1)cls=4;
        else if((seg&1)==1)cls=2;
        else cls=3;
    }
    ++W->count;
    if(vq<W->min_vq){W->min_vq=vq;W->min_t=t;W->min_seg=seg;W->min_class=cls;}
    if(vq>W->max_vq){W->max_vq=vq;W->max_t=t;W->max_seg=seg;W->max_class=cls;}
}
struct VLeaf {
    int seg=0;double ta=0,tb=0;Poly C0;double vl=0,al=0,jl=0,sigma=0,qdot=0,vq=0,wit=std::numeric_limits<double>::quiet_NaN();
};
static inline VLeaf make_vleaf(const ActiveReference&ref,const Live&L,const SpeedCert&base,
                               const Poly&C0,double ta,double tb,int seg){
    VLeaf f;f.seg=seg;f.ta=ta;f.tb=tb;f.C0=C0;double T=tb-ta;
    if(!(fin(T)&&T>0&&C0.size()==16)){f.vq=std::numeric_limits<double>::infinity();return f;}
    Poly C1=deriv_poly(C0,T,1),C2=deriv_poly(C0,T,2),C3=deriv_poly(C0,T,3);
    double tc=0.5*(ta+tb),h=0.5*T;auto x=ref.eval(tc);Vec3 v0=x[1],a0=x[2],j0=x[3];
    f.vl=0;for(auto v:C1)f.vl=std::max(f.vl,norm(v+v0));f.vl+=base.A*h;
    f.al=0;for(auto a:C2)f.al=std::max(f.al,norm(a+a0));f.al+=base.J*h;
    Vec3 d1=decasteljau(C1,0.5),d2=decasteljau(C2,0.5),d3=decasteljau(C3,0.5);
    f.jl=base.J+poly_norm_ub(C3);
    Vec3 sm=a0+d2+Vec3{0,0,L.g};f.sigma=norm(sm)-f.jl*h;
    if(!(fin(f.sigma)&&f.sigma>0)){f.qdot=f.vq=std::numeric_limits<double>::infinity();}
    else{f.qdot=f.jl/f.sigma;f.vq=f.vl+L.L*f.qdot;}
    Vec3 vm=v0+d1,am=a0+d2,jm=j0+d3,s=am+Vec3{0,0,L.g};double ns=norm(s);
    if(fin(ns)&&ns>0){Vec3 q=s*(-1.0/ns);Vec3 proj=jm-q*dot(q,jm);Vec3 qd=proj*(-1.0/ns);f.wit=norm(vm-qd*L.L);}
    return f;
}
static inline SpeedCert stageB_speed(const ActiveReference&ref,const Live&L,const SpeedCert&base,
                                     const Candidate&C,double lim,bool&finiteWitness,bool&repr){
    std::vector<VLeaf> leaves;for(size_t s=0;s<C.cp.size();++s)leaves.push_back(make_vleaf(ref,L,base,C.cp[s],C.seg0[s],C.seg1[s],(int)s));
    int splits=0;
    while(true){
        bool all=true;for(auto&f:leaves)if(!(fin(f.vq)&&f.vq<lim)){all=false;break;}
        if(all)break;
        for(auto&f:leaves)if(fin(f.wit)&&f.wit>=lim){finiteWitness=true;
            towmap_core_record_witness(C.family,f.seg,(int)C.cp.size(),0.5*(f.ta+f.tb),f.wit);return {};}
        int bi=-1;double bs=-std::numeric_limits<double>::infinity();
        for(int i=0;i<(int)leaves.size();++i)if(!fin(leaves[i].vq)||leaves[i].vq>=lim){
            double sc=fin(leaves[i].vq)?leaves[i].vq-lim:std::numeric_limits<double>::infinity();
            if(sc>bs){bs=sc;bi=i;}
        }
        if(bi<0)break;auto old=leaves[bi];double tm=0.5*(old.ta+old.tb);
        if(!(fin(tm)&&tm>old.ta&&tm<old.tb)){repr=true;return {};}
        auto sp=split_half(old.C0);
        leaves[bi]=make_vleaf(ref,L,base,sp.first,old.ta,tm,old.seg);
        leaves.push_back(make_vleaf(ref,L,base,sp.second,tm,old.tb,old.seg));++splits;
    }
    SpeedCert s;s.certified=true;s.stageA=false;s.leaf_count=(int)leaves.size();s.split_count=splits;
    s.VQ=0;s.VL=0;s.A=0;s.J=0;s.sigma=std::numeric_limits<double>::infinity();s.qdot=0;
    for(auto&f:leaves){if(!fin(f.vq)||!fin(f.sigma)||f.sigma<=0){s.certified=false;return s;}
        s.VQ=std::max(s.VQ,f.vq);s.VL=std::max(s.VL,f.vl);s.A=std::max(s.A,f.al);s.J=std::max(s.J,f.jl);
        s.sigma=std::min(s.sigma,f.sigma);s.qdot=std::max(s.qdot,f.qdot);}
    if(!(s.VQ<lim&&s.sigma>0))s.certified=false;return s;
}

// -------------------------- corrected candidate eval ------------------------
static inline std::array<Vec3,7> candidate_eval(const ActiveReference&ref,const Candidate&C,double t){
    auto y=ref.eval(t);
    if(C.cp.empty()||t<=C.seg0.front())return y;
    if(t>=C.seg1.back()){
        if(C.terminal_hold)y[0]+=C.terminal_offset;
        return y;
    }
    int s=-1;for(int i=0;i<(int)C.cp.size();++i){if(t<=C.seg1[i]){s=i;break;}}
    if(s<0||t<C.seg0[s])return y;
    double T=C.seg1[s]-C.seg0[s],u=std::max(0.0,std::min(1.0,(t-C.seg0[s])/T));Poly D=C.cp[s];
    y[0]+=decasteljau(D,u);int deg=15;
    for(int r=1;r<=6;++r){Poly E(D.size()-1);for(size_t i=0;i+1<D.size();++i)E[i]=(D[i+1]-D[i])*(double(deg)/T);D.swap(E);--deg;y[r]+=decasteljau(D,u);}
    return y;
}
static inline bool rejoin_safe(const ActiveReference&ref,const Live&L,const std::vector<Obstacle>&obs,
                               double t1,const Candidate&C,const SpeedCert&s){
    // AF7R34: "rejoin" means rejoin to the nominal spatial path, possibly
    // with a persistent schedule delay. Evaluate the actual candidate, not
    // the old-clock nominal reference.
    auto x=candidate_eval(ref,C,t1);Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};double ns=norm(sf);if(!(fin(ns)&&ns>0))return false;
    Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;double h=0.5*L.dt,VC=std::max(s.VL,s.VQ);
    for(auto&o:obs){double m;if(!point_margin(pL,o,L.rL+L.dmargin+L.tracking+s.VL*h,m))return false;
        if(!point_margin(pQ,o,L.rQ+L.dmargin+L.tracking+s.VQ*h,m))return false;
        if(!segment_margin(pQ,pL,o,L.rC+L.dmargin+L.tracking+VC*h,m))return false;}
    return true;
}
static inline GeometryCert wholebody(const ActiveReference&ref,const Live&L,const std::vector<Obstacle>&obs,
                                     double t0,double tm,double hold,double t1,const Candidate&C,const SpeedCert&s){
    GeometryCert G;struct Leaf{double a,b;int d;};std::vector<Leaf> st;
    if(t1>hold)st.push_back({hold,t1,0});if(hold>tm)st.push_back({tm,hold,0});if(tm>t0)st.push_back({t0,tm,0});
    double minL=std::numeric_limits<double>::infinity(),minQ=minL,minC=minL,VC=std::max(s.VL,s.VQ);
    while(!st.empty()){
        Leaf f=st.back();st.pop_back();double tc=0.5*(f.a+f.b),h=0.5*(f.b-f.a);
        if(!(tc>f.a&&tc<f.b)){G.reason=112;return G;}
        auto x=candidate_eval(ref,C,tc);Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};double ns=norm(sf);
        if(!(fin(ns)&&ns>=s.sigma)){G.reason=108;return G;}
        Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;double smL=std::numeric_limits<double>::infinity(),smQ=smL,smC=smL;
        for(auto&o:obs){double a,b,c;if(!point_margin(pL,o,L.rL+L.dmargin+L.tracking,a)||
                !point_margin(pQ,o,L.rQ+L.dmargin+L.tracking,b)||
                !segment_margin(pQ,pL,o,L.rC+L.dmargin+L.tracking,c)){G.reason=111;return G;}
            smL=std::min(smL,a);smQ=std::min(smQ,b);smC=std::min(smC,c);}
        ++G.leaf_count;G.max_depth=std::max(G.max_depth,f.d);
        double rL=s.VL==0?std::numeric_limits<double>::infinity():smL/s.VL;
        double rQ=s.VQ==0?std::numeric_limits<double>::infinity():smQ/s.VQ;
        double rC=VC==0?std::numeric_limits<double>::infinity():smC/VC;
        double rc=std::min({rL,rQ,rC});
        if(!fin(rc)||h<rc){
            double a=smL-s.VL*h,b=smQ-s.VQ*h,c=smC-VC*h;if(std::min({a,b,c})<=0){G.reason=110;return G;}
            ++G.accepted;minL=std::min(minL,a);minQ=std::min(minQ,b);minC=std::min(minC,c);continue;
        }
        st.push_back({tc,f.b,f.d+1});st.push_back({f.a,tc,f.d+1});++G.splits;
    }
    G.minL=minL;G.minQ=minQ;G.minC=minC;G.minMargin=std::min({minL,minQ,minC});
    if(G.accepted>0&&fin(G.minMargin)&&G.minMargin>0){G.pass=true;G.reason=0;}else G.reason=105;
    return G;
}

// ---------------------------- top-level -------------------------------------
// R36 fixed-terminal certificate primitive + minimum-deviation synthesis
// -----------------------
// The packet field `t1` / `search_end` is a finite source-derived reference
// validity bound only. It is NOT used as the correction end any more.
//
// The current maneuver is built over the nominal-unsafe component(s) that
// cannot be separated by a certified return to the active base reference
// before the next unsafe component begins. Separated future components remain
// eligible for the existing recursive trigger/replanning architecture. Within
// the selected scope, the rejoin duration is chosen automatically by a
// finite source-domain search with no fixed maneuver-duration cap. Every
// accepted trial still passes exactly the same:
//   * Stage-A/B finite-sensing speed certificate,
//   * continuous rejoin-cell certificate,
//   * continuous all-obstacle whole-body certificate.
// A failed/inconclusive trial never grants execution.
//
// Nominal-deviation minimization is a synthesis preference only. The primary family
// explicitly minimizes correction from the nominal/reference, while C7
// continuity is imposed structurally.  Candidate ordering uses the continuous
// Bezier correction-envelope upper bound max_i ||P_i|| before the secondary
// synthesis cost.  Safety acceptance remains independent of this ordering.

struct OrientedRows {std::vector<Row> plus,minus;};

static inline EpisodeCert episode_prefix(const EpisodeCert&full,int lastComp){
    EpisodeCert E;
    if(!full.ok || lastComp<0 || lastComp>=(int)full.comp.size()) return E;
    E.ok=true;
    E.leaves=full.leaves; E.safe_leaves=full.safe_leaves;
    E.unsafe_leaves=full.unsafe_leaves; E.splits=full.splits;
    E.repr_fail=full.repr_fail;
    E.comp.assign(full.comp.begin(),full.comp.begin()+lastComp+1);
    for(const auto&e:full.ep) if(e.comp<=lastComp) E.ep.push_back(e);
    return E;
}

static inline bool candidate_envelope_less(const Candidate&a,const Candidate&b){
    double sc=std::max({1.0,std::fabs(a.max_ctrl),std::fabs(b.max_ctrl)});
    double g=65536*epsd()*sc;
    if(a.max_ctrl < b.max_ctrl-g) return true;
    if(b.max_ctrl < a.max_ctrl-g) return false;

    sc=std::max({1.0,std::fabs(a.cost),std::fabs(b.cost)});
    g=65536*epsd()*sc;
    if(a.cost < b.cost-g) return true;
    if(b.cost < a.cost-g) return false;

    size_t n=std::min(a.signs.size(),b.signs.size());
    for(size_t i=0;i<n;++i){
        if(a.signs[i]!=b.signs[i]) return a.signs[i] < b.signs[i];
    }
    return a.signs.size()<b.signs.size();
}

// Legacy affine fallback ordering is intentionally kept identical to the
// previously flown planner: minimum-snap QP cost first, then deterministic
// sign tie-break.  Compact spatial ordering is used ONLY for the new
// minimum-deviation affine family. This preserves the old certified numerical
// fallback rather than silently changing its candidate order.
static inline bool candidate_affine_less(const Candidate&a,const Candidate&b){
    double sc=std::max({1.0,std::fabs(a.cost),std::fabs(b.cost)});
    double g=65536*epsd()*sc;
    if(a.cost < b.cost-g) return true;
    if(b.cost < a.cost-g) return false;
    size_t n=std::min(a.signs.size(),b.signs.size());
    for(size_t i=0;i<n;++i){
        if(a.signs[i]!=b.signs[i]) return a.signs[i] < b.signs[i];
    }
    return a.signs.size()<b.signs.size();
}

// AF7_TOWING_C7_CONDENSED_C3_R7B heading theorem gate.
static inline bool c7r7_heading_regular(const ActiveReference&ref,const Candidate&C,
                                        const SpeedCert&s,double ta,double tb){
    if(!(s.certified&&fin(s.A)&&s.A>=0&&fin(ta)&&fin(tb)&&tb>ta))return false;
    struct Leaf{double a,b;int d;};std::vector<Leaf>st;st.push_back({ta,tb,0});
    const int maxDepth=52;
    while(!st.empty()){
        Leaf f=st.back();st.pop_back();double tc=0.5*(f.a+f.b),h=0.5*(f.b-f.a);
        if(!(tc>f.a&&tc<f.b))return false;
        auto x=candidate_eval(ref,C,tc);double vxy=std::hypot(x[1].x,x[1].y);
        if(fin(vxy)&&vxy-s.A*h>0)continue;
        if(f.d>=maxDepth)return false;
        st.push_back({tc,f.b,f.d+1});st.push_back({f.a,tc,f.d+1});
    }
    return true;
}

// One exact hard-certificate attempt for a fixed maneuver scope and fixed t1.
// No sampled acceptance is introduced here.
static inline PlanResult plan_fixed_scope(const ActiveReference&ref,const Live&L,const SpeedCert&base,
                                          const std::vector<Obstacle>&obs,double t0,double tm,double t1,
                                          const EpisodeCert&E){
    using clk=std::chrono::steady_clock;
    auto T0=clk::now();
    PlanResult R;
    R.episode=E;
    R.selected_t1=t1;
    R.search_end=t1;

    if(!E.ok||E.comp.empty()||E.ep.empty()){
        R.reason=22;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
    }
    if(E.comp.front().start<=t0+8*epsd()*std::max(1.0,std::fabs(t0))){
        R.reason=14;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
    }
    if(E.comp.back().end>=t1-8*epsd()*std::max(1.0,std::fabs(t1))){
        R.reason=17;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
    }
    R.hold_end=E.comp.back().end;

    auto a=clk::now();
    int Cn=(int)E.comp.size(),Ne=(int)E.ep.size(),nvar=6*Cn;
    std::vector<OrientedRows> OR(Ne);

    auto ts=clk::now();
    for(int e=0;e<Ne;++e){
        auto ep=E.ep[e];
        auto xa=ref.eval(ep.start),xb=ref.eval(ep.end);
        Vec3 prog=xb[0]-xa[0];
        if(norm(prog)<=128*epsd()*std::max(1.0,norm(xa[0]))) prog=ref.eval(0.5*(ep.start+ep.end))[1];
        Vec3 n=lateral_normal(obs[ep.obs].R,obs[ep.obs].axes,prog);
        OR[e].plus=corridor_rows(ref,L,base,obs,E,e,n,nvar);
        OR[e].minus=corridor_rows(ref,L,base,obs,E,e,n*(-1),nvar);
    }
    R.tim.corridor=std::chrono::duration<double,std::milli>(clk::now()-ts).count();

    // The affine minimum-snap Hessian is built lazily only if the primary
    // minimum-deviation family cannot close the hard certificates.
    std::vector<double> Haff;
    R.tim.hessian=0.0;

    double rhoBody=std::max({L.rQ,L.L+L.rL,L.L+L.rC});
    double lim=(L.sensor_range-rhoBody)/(L.horizon+L.dt);
    if(!(fin(lim)&&lim>0)){
        R.reason=16;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
    }
    R.finite_sensing_limit=lim; // AF7_TOWING_DIAG_V1

    struct Base{std::vector<double>z;double c=0,s=0;std::vector<int>sg;};

    auto certify_candidates=[&](std::vector<Candidate>& candidates,bool envelopeOrder,int branch)->bool{
        if(branch==1) R.primary_candidate_count+=(int)candidates.size();
        else if(branch==2) R.fallback_candidate_count+=(int)candidates.size();
        else if(branch==3) R.towing_candidate_count+=(int)candidates.size();

        // Across the discrete obstacle-side choices, prefer the smaller
        // continuous Bezier control-polygon envelope.  Each primary candidate
        // itself is already the exact minimum-L2-deviation solution for that
        // side assignment.  No candidate is accepted by this preference.
        if(envelopeOrder)
            std::stable_sort(candidates.begin(),candidates.end(),candidate_envelope_less);
        else
            std::stable_sort(candidates.begin(),candidates.end(),candidate_affine_less);
        for(auto cand:candidates){
        const int r24cfi=(cand.family>=0&&cand.family<8)?cand.family:0;
        ++R.r24e_family_seen[r24cfi];
        if(cand.exact_c7)++R.r24e_family_exact[r24cfi];
        else{
            ++R.r24e_family_exact_reject[r24cfi];
            continue;
        }

        // AF7R38: this primary is accepted as C7 only through its
        // constructive exact-real dyadic-lattice proof.
        if(cand.family==7 && cand.trigger_to_free_rejoin &&
                !cand.exact_c7_constructive){
            ++R.r24e_family_exact_reject[r24cfi];
            continue;
        }

        // AF7R35: current primary family must carry a constructive
        // path-tangent C6 yaw regularity certificate before any speed/safety
        // certificate is attempted.
        if(cand.family==7 &&
                !(cand.heading_c6&&fin(cand.heading_gamma)&&cand.heading_gamma>0))
            continue;

            SpeedCert sc;
            if(branch==1) R.primary_min_envelope=std::min(R.primary_min_envelope,cand.max_ctrl);
            auto tr=tighten_stageA(cand,base,L.L,lim);
            if(cand.family==7){
                R.r31_best_correction_speed_ub=
                    std::min(R.r31_best_correction_speed_ub,cand.dub[0]);
                const double dsig31=base.sigma-tr.U[1];
                const double dvq31=(dsig31>0.0)?
                    base.VL+tr.U[0]+L.L*(base.J+tr.U[2])/dsig31:
                    std::numeric_limits<double>::infinity();
                if(fin(dvq31))
                    R.r31_best_stageA_uav_speed_ub=
                        std::min(R.r31_best_stageA_uav_speed_ub,dvq31);
            }
            if(branch==1){
                const double dsig=base.sigma-tr.U[1];
                const double dvq=(dsig>0.0)?
                    base.VL+tr.U[0]+L.L*(base.J+tr.U[2])/dsig:
                    std::numeric_limits<double>::infinity();
                if(fin(dvq) && dvq<R.primary_min_stageA_vq){
                    R.primary_min_stageA_vq=dvq;
                    R.primary_min_stageA_sigma=dsig;
                    R.primary_min_stageA_U=tr.U;
                }
                if(tr.certified) ++R.primary_stageA_certified_count;
            }
            if(tr.certified){
                sc=stageA_speed(tr,base,L.L);
                if(sc.certified&&sc.VQ<lim){
                    cand.dub[0]=tr.U[0];cand.dub[1]=tr.U[1];cand.dub[2]=tr.U[2];
                }else sc.certified=false;
            }
            if(!sc.certified){
                bool wit=false,rp=false;
                sc=stageB_speed(ref,L,base,cand,lim,wit,rp);
                if(branch==1){
                    if(wit) ++R.primary_stageB_finite_witness_count;
                    if(rp) ++R.primary_stageB_repr_fail_count;
                    if(sc.certified) ++R.primary_stageB_certified_count;
                }
            }
            if(!sc.certified){
                ++R.r24e_family_speed_fail[r24cfi];
                continue;
            }
            ++R.r24e_family_speed_pass[r24cfi];
            ++R.speed_pass;
            if(branch==1) ++R.primary_speed_pass; else if(branch==2) ++R.fallback_speed_pass; else if(branch==3) ++R.towing_speed_pass;
            if(!rejoin_safe(ref,L,obs,t1,cand,sc)) continue;
            ++R.rejoin_pass;++R.geom_tests;
            if(branch==1){++R.primary_rejoin_pass;++R.primary_geom_tests;}
            else if(branch==2){++R.fallback_rejoin_pass;++R.fallback_geom_tests;}
            else if(branch==3){++R.towing_rejoin_pass;++R.towing_geom_tests;}
            auto G=wholebody(ref,L,obs,t0,tm,R.hold_end,t1,cand,sc);
            if(G.pass){
                if(branch==3 && !c7r7_heading_regular(ref,cand,sc,t0,t1))continue;
                R.success=true;R.reason=0;R.cand=std::move(cand);R.speed=sc;R.geom=G;return true;
            }
        }
        return false;
    };

    // ------------------------------------------------------------------
    // PRIMARY R37/R45: trigger -> free-rejoin generic exact-C7 spatial spline
    // with tangent-support corridor relief.
    //
    // For this fixed-t1 certificate primitive, t1 is the rejoin deadline.
    // The actual exact-grid rejoin is the largest representable time <= t1.
    // MATLAB may independently search shorter deadlines; every accepted trial
    // is rebuilt and passed through these same hard certificates.
    // ------------------------------------------------------------------
    ts=clk::now();
    {
        const double sA=E.comp.front().start;
        const double sB=E.comp.back().end;
        Vec3 vbase{};
        const bool affineBase=r32_affine_base_exact(ref,t0,sA,sB,t1,vbase);
        R.r32_affine_base_detected=affineBase?1:0;

        if(affineBase){
            R.r32_nominal_progress_speed=norm(vbase);
            const R36Plane B37=r36_plane(vbase);

            // Exact representation grid only; this is a numerical storage
            // quantum, not a maneuver-duration rule.
            const double h37=std::ldexp(1.0,std::ilogb(L.dt));
            auto K=r37_make_knots(E,t0,t1,h37);

            if(K.size()>=3 && K.front()<E.comp.front().start &&
                    K.back()>E.comp.back().end){
                R.r37_support_start_s=K.front();
                R.r37_actual_rejoin_s=K.back();
                R.r37_knot_count=(int)K.size();

                std::vector<double>H37;
                try{
                    H37=r37_spatial_hessian(K,L.L,L.g);
                }catch(const std::runtime_error&){
                    H37.clear();
                }

                if(!H37.empty()){
                    std::vector<Candidate>cands;
                    std::vector<int>sg37(Ne,1);

                    std::function<void(int)>rec37=[&](int k){
                        if(k<Ne){
                            sg37[k]=1;rec37(k+1);
                            sg37[k]=-1;rec37(k+1);
                            return;
                        }
                        ++R.r37_side_assignments;

                        std::vector<Row>rows;
                        try{
                            for(int ei=0;ei<Ne;++ei){
                                auto ep=E.ep[ei];
                                auto xa=ref.eval(ep.start),xb=ref.eval(ep.end);
                                Vec3 prog=xb[0]-xa[0];
                                if(norm(prog)<=128*epsd()*
                                        std::max(1.0,norm(xa[0])))
                                    prog=ref.eval(0.5*(ep.start+ep.end))[1];

                                Vec3 n0=lateral_normal(
                                    obs[ep.obs].R,obs[ep.obs].axes,prog);
                                Vec3 n=(sg37[ei]>0)?n0:n0*(-1.0);
                                R45CorridorDiag D45;
                                auto rr=r37_corridor_rows(
                                    ref,L,base,obs,E,ei,n,K,B37,vbase,&D45);
                                rows.insert(rows.end(),rr.begin(),rr.end());

                                R.r45_tangent_interval_count+=D45.intervals;
                                R.r45_zero_offset_interval_count+=D45.zero_offset_intervals;
                                R.r45_convex_search_fallback_count+=D45.search_fallbacks;
                                R.r45_best_fixed_peak_req_m=
                                    std::min(R.r45_best_fixed_peak_req_m,D45.fixed_peak);
                                R.r45_best_tangent_peak_req_m=
                                    std::min(R.r45_best_tangent_peak_req_m,D45.tangent_peak);
                                R.r45_max_peak_relief_m=
                                    std::max(R.r45_max_peak_relief_m,
                                        D45.fixed_peak-D45.tangent_peak);
                            }
                        }catch(const std::runtime_error&){
                            ++R.r28_flow_grid_reject;
                            return;
                        }

                        const int nvar=2*r37_scalar_dofs(K);
                        QPSol q=solve_qp_active_exchange(H37,nvar,rows);

                        // Numerical failure of the scalable exchange solver
                        // never relaxes the corridor. Rebuild the historical
                        // fixed-separator rows and use the original exact
                        // subset enumerator, whose row count is small for that
                        // conservative corridor.
                        if(!q.ok){
                            std::vector<Row>fixedRows;
                            try{
                                for(int ei=0;ei<Ne;++ei){
                                    auto ep=E.ep[ei];
                                    auto xa=ref.eval(ep.start),xb=ref.eval(ep.end);
                                    Vec3 prog=xb[0]-xa[0];
                                    if(norm(prog)<=128*epsd()*std::max(1.0,norm(xa[0])))
                                        prog=ref.eval(0.5*(ep.start+ep.end))[1];
                                    Vec3 n0=lateral_normal(obs[ep.obs].R,obs[ep.obs].axes,prog);
                                    Vec3 n=(sg37[ei]>0)?n0:n0*(-1.0);
                                    auto rr=r45_fixed_corridor_rows(ref,L,base,obs,E,ei,n,K,B37);
                                    fixedRows.insert(fixedRows.end(),rr.begin(),rr.end());
                                }
                            }catch(const std::runtime_error&){
                                return;
                            }
                            q=solve_qp(H37,nvar,fixedRows);
                        }
                        if(!q.ok)return;
                        ++R.r37_spatial_qp_solutions;

                        R37Path P=r37_decode_path(q,K,B37,sg37);
                        if(!P.ok)return;

                        try{
                            Candidate C=r37_build_candidate(P,h37,vbase);
                            if(!C.exact_c7 || !C.exact_c7_constructive){
                                ++R.r31_exact_join_reject;
                                return;
                            }
                            ++R.r37_exact_c7_candidates;
                            ++R.r38_constructive_c7_count;
                            if(C.legacy_float_endpoint_checker_pass)
                                ++R.r38_legacy_float_checker_pass_count;
                            else
                                ++R.r38_legacy_float_false_reject_count;

                            ++R.r35_heading_candidate_count;
                            double gamma=std::numeric_limits<double>::quiet_NaN();
                            double anchor=std::numeric_limits<double>::quiet_NaN();
                            if(!r35_apply_heading_certificate(
                                    vbase,C,gamma,anchor)){
                                ++R.r35_heading_reject_count;
                                return;
                            }
                            ++R.r35_heading_pass_count;
                            ++R.r37_heading_pass_count;
                            if(!fin(R.r35_best_heading_gamma_m_s)||
                                    gamma>R.r35_best_heading_gamma_m_s)
                                R.r35_best_heading_gamma_m_s=gamma;

                            const double vu=r32_affine_velocity_control_ub(vbase,C);
                            if(vu<R.r37_best_total_velocity_ub){
                                R.r37_best_total_velocity_ub=vu;
                                R.r37_best_accel_ub=C.dub[1];
                                R.r37_best_jerk_ub=C.dub[2];
                                R.r32_best_pre_speed_ub=vu;
                                R.r32_best_progress_scale=1.0;
                                R.r32_best_entry_T=E.comp.front().start-K.front();
                                R.r32_best_flow_T=E.comp.back().end-E.comp.front().start;
                                R.r32_best_exit_T=K.back()-E.comp.back().end;
                            }
                            cands.push_back(std::move(C));
                        }catch(const std::runtime_error&){
                            ++R.r28_flow_grid_reject;
                        }
                    };
                    rec37(0);

                    // Quality only. Safety remains in certify_candidates().
                    // For the same rejoin deadline, prefer lower cable-driving
                    // jerk bound, then lower acceleration, then smaller
                    // deviation envelope. No arbitrary weighted sum.
                    std::stable_sort(cands.begin(),cands.end(),
                        [](const Candidate&a,const Candidate&b){
                            if(a.dub[2]<b.dub[2])return true;
                            if(b.dub[2]<a.dub[2])return false;
                            if(a.dub[1]<b.dub[1])return true;
                            if(b.dub[1]<a.dub[1])return false;
                            return candidate_envelope_less(a,b);
                        });

                    R.tim.qp+=std::chrono::duration<double,std::milli>(
                        clk::now()-ts).count();
                    auto tc37=clk::now();

                    // Preserve the quality order above. envelopeOrder=false
                    // would normally invoke the affine cost order, so certify
                    // one candidate at a time in the already-established
                    // lexicographic towing-quality order.
                    for(size_t i=0;i<cands.size();++i){
                        std::vector<Candidate>one;
                        one.push_back(cands[i]);
                        if(certify_candidates(one,true,4)){
                            R.r35_selected_heading_gamma_m_s=R.cand.heading_gamma;
                            R.r35_selected_heading_anchor_rad=R.cand.heading_anchor_rad;
                            R.r37_actual_rejoin_s=R.cand.actual_rejoin_time;
                            R.tim.candidate_build+=
                                std::chrono::duration<double,std::milli>(
                                    clk::now()-tc37).count();
                            R.tim.synthesis=
                                std::chrono::duration<double,std::milli>(
                                    clk::now()-a).count();
                            R.tim.candidate=R.tim.synthesis;
                            R.tim.total=
                                std::chrono::duration<double,std::milli>(
                                    clk::now()-T0).count();
                            return R;
                        }
                    }
                    R.tim.candidate_build+=
                        std::chrono::duration<double,std::milli>(
                            clk::now()-tc37).count();
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // PRIMARY R25F: CURVED_C7_ZERO_JET_BELL (family 6).
    //
    // There is no internal constant plateau. Each disconnected unsafe
    // component gets one two-piece compact bell 0 -> A_j -> 0. Exact C7 is
    // structural from repeated endpoint/peak control points, not a tolerance
    // test and not an equal-duration construction.
    //
    // Half-space rows are synthesis-only. Acceptance below uses the SAME
    // exact-C7 hard gate followed by the unchanged finite-sensing speed,
    // rejoin and continuous whole-body certificate stack.
    // ------------------------------------------------------------------
    ts=clk::now();
    {
        std::vector<R25BellWin> W25;
        try{
            W25=r25_windows(E.comp,t0,t1);
        }catch(const std::runtime_error&){
            W25.clear(); // family 6 unavailable for this representability case
        }
        const int nv25=3*Cn;
        std::vector<Base>B25;
        if(!W25.empty()){
            auto H25=r25_bell_hessian(W25);
            std::vector<int>sg25(Ne,1);
            std::function<void(int)> rec25=[&](int k){
                if(k<Ne){
                    sg25[k]=1;rec25(k+1);
                    sg25[k]=-1;rec25(k+1);
                    return;
                }
                // AF7R25G exact block separation:
                // family-6 objective and constraints are independent per
                // disconnected unsafe component. Solve Cn small 3D QPs
                // instead of one generic high-dimensional active-set QP.
                std::vector<std::vector<Row>> rowsByComp(Cn);
                try{
                    for(int e=0;e<Ne;++e){
                        const auto&ep25=E.ep[e];
                        auto xa25=ref.eval(ep25.start),xb25=ref.eval(ep25.end);
                        Vec3 prog25=xb25[0]-xa25[0];
                        if(norm(prog25)<=128*epsd()*std::max(1.0,norm(xa25[0])))
                            prog25=ref.eval(ep25.start+0.5*(ep25.end-ep25.start))[1];
                        Vec3 n25=lateral_normal(obs[ep25.obs].R,obs[ep25.obs].axes,prog25);
                        Vec3 n=(sg25[e]>0)?n25:n25*(-1.0);
                        auto gv=r25_bell_rows(ref,L,base,obs,E,W25,e,n,nv25);
                        if(gv.size()!=1)throw std::runtime_error("R25F reduced row count");
                        Row lr;lr.a.assign(3,0.0);lr.b=gv[0].b;
                        const int j=ep25.comp;
                        lr.a[0]=gv[0].a[3*j+0];
                        lr.a[1]=gv[0].a[3*j+1];
                        lr.a[2]=gv[0].a[3*j+2];
                        rowsByComp[j].push_back(std::move(lr));
                    }
                }catch(const std::runtime_error&){
                    return;
                }

                std::vector<double> z25(nv25,0.0);
                double c25=0.0,s25=std::numeric_limits<double>::infinity();
                bool ok25=true;
                for(int j=0;j<Cn;++j){
                    if(rowsByComp[j].empty()){ok25=false;break;}
                    const double w=H25[(3*j)*nv25+(3*j)];
                    if(!(fin(w)&&w>0)){ok25=false;break;}
                    std::vector<double>H3(9,0.0);
                    H3[0]=H3[4]=H3[8]=w;
                    QPSol qj=solve_qp(H3,3,rowsByComp[j]);
                    if(!qj.ok||qj.z.size()!=3){ok25=false;break;}
                    z25[3*j+0]=qj.z[0];
                    z25[3*j+1]=qj.z[1];
                    z25[3*j+2]=qj.z[2];
                    c25+=qj.cost;
                    s25=std::min(s25,qj.slack);
                }
                if(ok25)B25.push_back({z25,c25,s25,sg25});
            };
            rec25(0);
        }

        std::vector<Candidate>bell;
        bell.reserve(B25.size());
        for(auto&b:B25)
            bell.push_back(r25_build_bell(E.comp,W25,b.z,b.c,b.sg,b.s));

        R.tim.qp+=std::chrono::duration<double,std::milli>(clk::now()-ts).count();
        auto tc25=clk::now();
        // branch=4 is intentionally distinct from the legacy branch-3
        // C7R7 heading gate. Independent C6 yaw remains outside this family.
        if(certify_candidates(bell,true,4)){
            R.tim.candidate_build+=
                std::chrono::duration<double,std::milli>(clk::now()-tc25).count();
            R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();
            R.tim.candidate=R.tim.synthesis;
            R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
            return R;
        }
        R.tim.candidate_build+=
            std::chrono::duration<double,std::milli>(clk::now()-tc25).count();
    }

    // ------------------------------------------------------------------
    // PRIMARY R36 branch: exact minimum integrated squared deviation.
    //
    // Unsafe components retain the proven affine towing structure
    //      d_j(s)=a_j+v_j s,
    // so d_j''=0 exactly and the existing suspended-load common-translation
    // theorem remains unchanged.  Unlike the old minimum-snap-only QP, the
    // optimization objective is the exact complete-trajectory quantity
    //      integral ||d(t)||^2 dt.
    // This directly penalizes unnecessary departure from the nominal
    // reference while retaining v_j freedom needed when a constant offset
    // would violate finite-sensing / derivative feasibility.
    // ------------------------------------------------------------------
    ts=clk::now();
    {
        auto Hdev=minimum_deviation_hessian(E.comp,t0,t1);
        std::vector<Base>B;
        std::vector<int>sg(Ne,1);
        std::function<void(int)> rec=[&](int k){
            if(k<Ne){sg[k]=1;rec(k+1);sg[k]=-1;rec(k+1);return;}
            std::vector<Row> rows;
            for(int e=0;e<Ne;++e){
                auto&v=sg[e]>0?OR[e].plus:OR[e].minus;
                rows.insert(rows.end(),v.begin(),v.end());
            }
            QPSol q=solve_qp(Hdev,nvar,rows);
            if(q.ok)B.push_back({q.z,q.cost,q.slack,sg});
        };
        rec(0);
        std::vector<Candidate> primary;
        primary.reserve(B.size());
        for(auto&b:B)
            primary.push_back(build_candidate(E.comp,t0,t1,b.z,b.c,b.sg,b.s,1));
        R.tim.qp+=std::chrono::duration<double,std::milli>(clk::now()-ts).count();
        auto tc=clk::now();
        if(certify_candidates(primary,true,1)){
            R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-tc).count();
            R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();
            R.tim.candidate=R.tim.synthesis;
            R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
            return R;
        }
        R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-tc).count();
    }

    // ------------------------------------------------------------------
    // AF7_TOWING_C7_CONDENSED_C3_R7B
    // NEW TOWING FAMILY: reduced two-piece C7 transition spline.
    //
    // The midpoint 0..7 jets of every safe transition are optimized
    // analytically and condensed from the QP. The unsafe affine span
    // and corridor inequalities stay unchanged.
    // ------------------------------------------------------------------
    ts=clk::now();
    {
        // AF7_R7_FAILCLOSED_FALLTHROUGH_R15B
        // R7 is optional. Failure to construct its condensed midpoint
        // model means R7 is unavailable; it does not invalidate the
        // independently certified fallback family.
        C7R7Model MR;
        try{
            MR=c7r7_model(E.comp,t0,t1,L.L,L.g);
        }catch(const std::runtime_error&e){
            if(std::string(e.what())=="R7B midpoint Schur solve failed"){
                goto AF7_R15B_FALLBACK;
            }
            throw;
        }
        std::vector<Base> BT;
        std::vector<int>sg(Ne,1);
        std::function<void(int)> rec=[&](int k){
            if(k<Ne){sg[k]=1;rec(k+1);sg[k]=-1;rec(k+1);return;}
            std::vector<Row>rows;
            for(int e=0;e<Ne;++e){const auto&src=sg[e]>0?OR[e].plus:OR[e].minus;rows.insert(rows.end(),src.begin(),src.end());}
            QPSol q=solve_qp(MR.H,nvar,rows);
            if(q.ok)BT.push_back({q.z,q.cost,q.slack,sg});
        };
        rec(0);
        std::vector<Candidate> towing;
        towing.reserve(BT.size());
        for(auto&b:BT)towing.push_back(build_candidate_c7r7(E.comp,t0,t1,b.z,b.c,b.sg,b.s,MR,3));
        R.tim.qp+=std::chrono::duration<double,std::milli>(clk::now()-ts).count();
        auto tc=clk::now();
        if(certify_candidates(towing,true,3)){
            R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-tc).count();
            R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();
            R.tim.candidate=R.tim.synthesis;
            R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
            return R;
        }
        R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-tc).count();
    }

    // ------------------------------------------------------------------
AF7_R15B_FALLBACK:
    // ------------------------------------------------------------------
    // AF7_TOWING_MINNORM_CONSTANT_R16
    // NEW TOWING FALLBACK: componentwise minimum-norm constant offset.
    //
    // Restriction on every unsafe component j:
    //     d_j(t)=a_j,  v_j=0.
    // Therefore correction-induced derivatives of order >=1 vanish
    // identically on the unsafe span.  The corridor rows reduce exactly
    // from n'a_j+s n'v_j>=b to n'a_j>=b.
    //
    // Each component solves min 0.5||a_j||^2 with H=I3.  No arbitrary
    // weights, no Schur complement and no minimum-snap Hessian are used.
    // Candidate acceptance remains the unchanged hard certificate stack.
    // ------------------------------------------------------------------
    {
        const auto tfb=clk::now();
        std::vector<Candidate> calm;
        std::vector<int> sg(Ne,1);
        std::function<void(int)> rec16=[&](int k){
            if(k<Ne){
                sg[k]=1;rec16(k+1);
                sg[k]=-1;rec16(k+1);
                return;
            }
            std::vector<double> z(nvar,0.0);
            double totalCost=0.0;
            double minSlack=std::numeric_limits<double>::infinity();
            bool ok=true;
            for(int j=0;j<Cn && ok;++j){
                std::vector<Row> r3;
                for(int e=0;e<Ne;++e){
                    if(E.ep[e].comp!=j)continue;
                    const auto&src=(sg[e]>0)?OR[e].plus:OR[e].minus;
                    for(const auto&rf:src){
                        Row rr;rr.a.assign(3,0.0);rr.b=rf.b;
                        const int pslot=2*j;
                        rr.a[0]=rf.a[3*pslot+0];
                        rr.a[1]=rf.a[3*pslot+1];
                        rr.a[2]=rf.a[3*pslot+2];
                        r3.push_back(std::move(rr));
                    }
                }
                if(r3.empty()){ok=false;break;}
                std::vector<double> H3(9,0.0);H3[0]=H3[4]=H3[8]=1.0;
                QPSol q=solve_qp(H3,3,r3);
                if(!q.ok||q.z.size()!=3){ok=false;break;}
                const int pslot=2*j;
                z[3*pslot+0]=q.z[0];
                z[3*pslot+1]=q.z[1];
                z[3*pslot+2]=q.z[2];
                // velocity slot stays exactly zero.
                totalCost+=q.cost;
                minSlack=std::min(minSlack,q.slack);
            }
            if(!ok)return;
            Candidate C=build_candidate(E.comp,t0,t1,z,totalCost,sg,minSlack,4);
            C.constant_translation=true;
            C.exact_c7=true; // zero-jet transitions + constant unsafe spans
            const double aopt=r17_speed_opt_alpha(E.comp,t0,t1,C.pos,base,L);
            auto pushR17=[&](double av){
                try{
                    Candidate S=r17_build_split_constant(E.comp,t0,t1,z,totalCost,sg,minSlack,av);
                    calm.push_back(std::move(S));
                }catch(const std::runtime_error&e){
                    const std::string m=e.what();
                    if(m.rfind("R22C ",0)==0){
                        ++R.r24e_split_representability_reject;
                        return;
                    }
                    throw;
                }
            };
            pushR17(aopt);
            if(std::fabs(aopt-15.0/8.0)>128*epsd())pushR17(15.0/8.0);
            if(std::fabs(aopt-2.0)>128*epsd())pushR17(2.0);
            calm.push_back(std::move(C));
        };
        rec16(0);
        R.tim.qp+=std::chrono::duration<double,std::milli>(clk::now()-tfb).count();
        const auto tc16=clk::now();
        if(certify_candidates(calm,true,4)){
            R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-tc16).count();
            R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();
            R.tim.candidate=R.tim.synthesis;
            R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
            return R;
        }
        R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-tc16).count();
    }

    // ------------------------------------------------------------------
    // FALLBACK: exact previously flown affine/minimum-snap family.
    // Only reached when no exact minimum-deviation affine candidate closes
    // all unchanged hard certificates. Thus the new spatial objective cannot
    // reduce the old certified solution set.
    // ------------------------------------------------------------------
    ts=clk::now();
    Haff=minimum_snap_hessian(E.comp,t0,t1);
    R.tim.hessian+=std::chrono::duration<double,std::milli>(clk::now()-ts).count();

    ts=clk::now();
    std::vector<Base>B;
    std::vector<int>sg(Ne,1);
    std::function<void(int)> rec=[&](int k){
        if(k<Ne){sg[k]=1;rec(k+1);sg[k]=-1;rec(k+1);return;}
        std::vector<Row> rows;
        for(int e=0;e<Ne;++e){auto&v=sg[e]>0?OR[e].plus:OR[e].minus;rows.insert(rows.end(),v.begin(),v.end());}
        QPSol q=solve_qp(Haff,nvar,rows);
        if(q.ok)B.push_back({q.z,q.cost,q.slack,sg});
    };
    rec(0);
    R.tim.qp+=std::chrono::duration<double,std::milli>(clk::now()-ts).count();

    ts=clk::now();
    std::vector<Candidate> affine;
    affine.reserve(B.size());
    for(auto&b:B)affine.push_back(build_candidate(E.comp,t0,t1,b.z,b.c,b.sg,b.s,2));
    R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-ts).count();
    R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();

    a=clk::now();
    certify_candidates(affine,false,2);
    R.tim.candidate+=std::chrono::duration<double,std::milli>(clk::now()-a).count();
    if(!R.success)R.reason=R.speed_pass==0?24:25;
    R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
    return R;
}
// AF7_C7_HOMOTOPY_FRONTIER_R6B
// Diagnostic-only continuation frontier. Never used for production selection.
struct C7FrontRow {
    double alpha=0;
    int sign_code=0;
    bool generated=false,speed=false,rejoin=false,geometry=false,heading=false;
    double envelope=std::numeric_limits<double>::quiet_NaN();
    double VL=std::numeric_limits<double>::quiet_NaN();
    double A=std::numeric_limits<double>::quiet_NaN();
    double J=std::numeric_limits<double>::quiet_NaN();
    double sigma=std::numeric_limits<double>::quiet_NaN();
    double qdot=std::numeric_limits<double>::quiet_NaN();
    double VQ=std::numeric_limits<double>::quiet_NaN();
    double heading_lb=std::numeric_limits<double>::quiet_NaN();
    int heading_splits=0;
};
static std::vector<C7FrontRow> G_c7front_rows;

struct HeadingRegularityCert {
    bool pass=false;
    double min_lb=std::numeric_limits<double>::infinity();
    int splits=0;
};
static inline HeadingRegularityCert c7front_heading_regular(
        const ActiveReference&ref,const Candidate&C,const SpeedCert&s,double ta,double tb){
    HeadingRegularityCert R;
    struct Leaf{double a,b;int d;};
    std::vector<Leaf> st;
    if(!(fin(ta)&&fin(tb)&&tb>ta&&s.certified&&fin(s.A)&&s.A>=0))return R;
    st.push_back({ta,tb,0});
    const int maxDepth=52;
    while(!st.empty()){
        Leaf f=st.back();st.pop_back();
        double tc=0.5*(f.a+f.b),h=0.5*(f.b-f.a);
        if(!(tc>f.a&&tc<f.b))return R;
        auto x=candidate_eval(ref,C,tc);
        double vxy=std::hypot(x[1].x,x[1].y);
        double lb=vxy-s.A*h;
        if(fin(lb)&&lb>0){R.min_lb=std::min(R.min_lb,lb);continue;}
        if(f.d>=maxDepth)return R;
        st.push_back({tc,f.b,f.d+1});
        st.push_back({f.a,tc,f.d+1});
        ++R.splits;
    }
    if(fin(R.min_lb)&&R.min_lb>0)R.pass=true;
    return R;
}

static inline C7FrontRow c7front_certify_one(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,double t0,double tm,double hold,double t1,
        Candidate cand,double lim,double alpha,int signCode){
    C7FrontRow D;D.alpha=alpha;D.sign_code=signCode;D.generated=true;D.envelope=cand.max_ctrl;
    SpeedCert sc;
    auto tr=tighten_stageA(cand,base,L.L,lim);
    if(tr.certified){
        sc=stageA_speed(tr,base,L.L);
        if(sc.certified&&sc.VQ<lim){
            cand.dub[0]=tr.U[0];cand.dub[1]=tr.U[1];cand.dub[2]=tr.U[2];
        }else sc.certified=false;
    }
    if(!sc.certified){bool wit=false,rp=false;sc=stageB_speed(ref,L,base,cand,lim,wit,rp);}
    if(!sc.certified)return D;
    D.speed=true;D.VL=sc.VL;D.A=sc.A;D.J=sc.J;D.sigma=sc.sigma;D.qdot=sc.qdot;D.VQ=sc.VQ;
    if(!rejoin_safe(ref,L,obs,t1,cand,sc))return D;
    D.rejoin=true;
    auto G=wholebody(ref,L,obs,t0,tm,hold,t1,cand,sc);
    if(!G.pass)return D;
    D.geometry=true;
    auto H=c7front_heading_regular(ref,cand,sc,t0,t1);
    D.heading=H.pass;D.heading_lb=H.pass?H.min_lb:std::numeric_limits<double>::quiet_NaN();D.heading_splits=H.splits;
    return D;
}

static inline void c7front_run(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,double t0,double tm,double t1,const EpisodeCert&E){
    G_c7front_rows.clear();
    if(!E.ok||E.comp.empty()||E.ep.empty())return;
    int Cn=(int)E.comp.size(),Ne=(int)E.ep.size(),nvar=6*Cn;
    std::vector<OrientedRows> OR(Ne);
    for(int e=0;e<Ne;++e){
        auto ep=E.ep[e];auto xa=ref.eval(ep.start),xb=ref.eval(ep.end);
        Vec3 prog=xb[0]-xa[0];
        if(norm(prog)<=128*epsd()*std::max(1.0,norm(xa[0])))prog=ref.eval(0.5*(ep.start+ep.end))[1];
        Vec3 n=lateral_normal(obs[ep.obs].R,obs[ep.obs].axes,prog);
        OR[e].plus=corridor_rows(ref,L,base,obs,E,e,n,nvar);
        OR[e].minus=corridor_rows(ref,L,base,obs,E,e,n*(-1),nvar);
    }
    double rhoBody=std::max({L.rQ,L.L+L.rL,L.L+L.rC});
    double lim=(L.sensor_range-rhoBody)/(L.horizon+L.dt);
    if(!(fin(lim)&&lim>0))return;
    auto Hc3=towing_natural_time_c3_hessian(E.comp,t0,t1,L.L,L.g);
    auto Hsn=minimum_snap_hessian(E.comp,t0,t1);
    std::vector<int>sg(Ne,1);int signCode=0;
    std::function<void(int)> rec=[&](int k){
        if(k<Ne){sg[k]=1;rec(k+1);sg[k]=-1;rec(k+1);return;}
        std::vector<Row> rows;
        for(int e=0;e<Ne;++e){const auto&v=sg[e]>0?OR[e].plus:OR[e].minus;rows.insert(rows.end(),v.begin(),v.end());}
        QPSol qc=solve_qp(Hc3,nvar,rows),qs=solve_qp(Hsn,nvar,rows);
        ++signCode;
        if(!(qc.ok&&qs.ok)||qc.z.size()!=qs.z.size())return;
        for(int kk=0;kk<=16;++kk){
            double alpha=double(kk)/16.0;
            std::vector<double>z(qc.z.size());
            for(size_t i=0;i<z.size();++i)z[i]=(1.0-alpha)*qc.z[i]+alpha*qs.z[i];
            Candidate C=build_candidate(E.comp,t0,t1,z,alpha,sg,0.0,4);
            G_c7front_rows.push_back(c7front_certify_one(ref,L,base,obs,t0,tm,E.comp.back().end,t1,C,lim,alpha,signCode));
        }
    };
    rec(0);
}

// AF7_R7_SHADOW_AUTHORITY_R8B
struct C7R8ShadowResult {
    bool success=false;
    int reason=0,candidate_count=0,speed_pass=0,rejoin_pass=0,geom_pass=0,heading_pass=0;
    Candidate cand;
    SpeedCert speed;
    double total_ms=0;
};

static inline C7R8ShadowResult c7r8_shadow_plan(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,double t0,double tm,double t1){
    using clk=std::chrono::steady_clock;const auto T0=clk::now();C7R8ShadowResult R;
    if(!(fin(t0)&&fin(tm)&&fin(t1)&&t1>tm&&tm>t0&&L.dt>0)){R.reason=17;return R;}
    EpisodeCert E=build_episode(ref,L,base,obs,t0,t1);
    if(!E.ok||E.comp.empty()||E.ep.empty()){R.reason=22;return R;}
    if(E.comp.front().start<=t0+8*epsd()*std::max(1.0,std::fabs(t0))){R.reason=14;return R;}
    if(E.comp.back().end>=t1-8*epsd()*std::max(1.0,std::fabs(t1))){R.reason=17;return R;}

    int Cn=(int)E.comp.size(),Ne=(int)E.ep.size(),nvar=6*Cn;
    std::vector<OrientedRows> OR(Ne);
    for(int e=0;e<Ne;++e){
        auto ep=E.ep[e];auto xa=ref.eval(ep.start),xb=ref.eval(ep.end);Vec3 prog=xb[0]-xa[0];
        if(norm(prog)<=128*epsd()*std::max(1.0,norm(xa[0])))prog=ref.eval(0.5*(ep.start+ep.end))[1];
        Vec3 n=lateral_normal(obs[ep.obs].R,obs[ep.obs].axes,prog);
        OR[e].plus=corridor_rows(ref,L,base,obs,E,e,n,nvar);
        OR[e].minus=corridor_rows(ref,L,base,obs,E,e,n*(-1),nvar);
    }
    double rhoBody=std::max({L.rQ,L.L+L.rL,L.L+L.rC});
    double lim=(L.sensor_range-rhoBody)/(L.horizon+L.dt);
    if(!(fin(lim)&&lim>0)){R.reason=16;return R;}

    struct BaseR8{std::vector<double>z;double c=0,s=0;std::vector<int>sg;};
    auto MR=c7r7_model(E.comp,t0,t1,L.L,L.g);
    std::vector<BaseR8>B;std::vector<int>sg(Ne,1);
    std::function<void(int)> rec=[&](int k){
        if(k<Ne){sg[k]=1;rec(k+1);sg[k]=-1;rec(k+1);return;}
        std::vector<Row>rows;
        for(int e=0;e<Ne;++e){const auto&src=sg[e]>0?OR[e].plus:OR[e].minus;rows.insert(rows.end(),src.begin(),src.end());}
        QPSol q=solve_qp(MR.H,nvar,rows);
        if(q.ok)B.push_back({q.z,q.cost,q.slack,sg});
    };rec(0);

    std::vector<Candidate>C;
    C.reserve(B.size());for(auto&b:B)C.push_back(build_candidate_c7r7(E.comp,t0,t1,b.z,b.c,b.sg,b.s,MR,3));
    R.candidate_count=(int)C.size();
    std::stable_sort(C.begin(),C.end(),candidate_envelope_less);

    for(auto cand:C){
        SpeedCert sc;auto tr=tighten_stageA(cand,base,L.L,lim);
        if(tr.certified){
            sc=stageA_speed(tr,base,L.L);
            if(sc.certified&&sc.VQ<lim){cand.dub[0]=tr.U[0];cand.dub[1]=tr.U[1];cand.dub[2]=tr.U[2];}
            else sc.certified=false;
        }
        if(!sc.certified){bool wit=false,rp=false;sc=stageB_speed(ref,L,base,cand,lim,wit,rp);}
        if(!sc.certified)continue;
        ++R.speed_pass;
        if(!rejoin_safe(ref,L,obs,t1,cand,sc))continue;
        ++R.rejoin_pass;
        auto G=wholebody(ref,L,obs,t0,tm,E.comp.back().end,t1,cand,sc);
        if(!G.pass)continue;
        ++R.geom_pass;
        if(!c7r7_heading_regular(ref,cand,sc,t0,t1))continue;
        ++R.heading_pass;
        R.success=true;R.reason=0;R.cand=std::move(cand);R.speed=sc;
        R.total_ms=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
    }
    if(R.candidate_count==0)R.reason=23;
    else if(R.speed_pass==0)R.reason=24;
    else if(R.rejoin_pass==0)R.reason=25;
    else if(R.geom_pass==0)R.reason=26;
    else R.reason=27;
    R.total_ms=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
}

// AF7_HEADING_TOPOLOGY_R10
struct HeadingTopoR10 {
    bool strict_pass=false;
    int location=0; // 0 pass, 1 t0, 2 t1, 3 interior, 4 both endpoints
    double fail_t=NAN,fail_width=NAN,vxy=NAN,v0=NAN,v1=NAN;
    std::array<double,6> hder{};
    int unresolved=0,max_depth_hit=0;
};

static inline HeadingTopoR10 heading_topology_r10(
        const ActiveReference&ref,const Candidate&C,const SpeedCert&s,double ta,double tb){
    HeadingTopoR10 R;
    if(!(s.certified&&fin(s.A)&&s.A>=0&&fin(ta)&&fin(tb)&&tb>ta))return R;
    auto xa=candidate_eval(ref,C,ta),xb=candidate_eval(ref,C,tb);
    R.v0=std::hypot(xa[1].x,xa[1].y);R.v1=std::hypot(xb[1].x,xb[1].y);
    struct Leaf{double a,b;int d;bool lt,rt;};
    std::vector<Leaf>st;st.push_back({ta,tb,0,true,true});
    const int maxDepth=52;bool hitL=false,hitR=false,hitI=false;
    double bestW=std::numeric_limits<double>::infinity();
    while(!st.empty()){
        Leaf f=st.back();st.pop_back();double tc=0.5*(f.a+f.b),h=0.5*(f.b-f.a);
        if(!(tc>f.a&&tc<f.b)){++R.unresolved;if(f.lt)hitL=true;if(f.rt)hitR=true;if(!f.lt&&!f.rt)hitI=true;continue;}
        auto x=candidate_eval(ref,C,tc);double v=std::hypot(x[1].x,x[1].y);
        double lb=v-s.A*h;
        if(fin(lb)&&lb>0)continue;
        if(f.d>=maxDepth){
            ++R.unresolved;++R.max_depth_hit;
            if(f.lt)hitL=true;if(f.rt)hitR=true;if(!f.lt&&!f.rt)hitI=true;
            double w=f.b-f.a;if(w<bestW){
                bestW=w;R.fail_t=tc;R.fail_width=w;R.vxy=v;
                for(int k=1;k<=6;++k)R.hder[k-1]=std::hypot(x[k].x,x[k].y);
            }
            continue;
        }
        st.push_back({tc,f.b,f.d+1,false,f.rt});
        st.push_back({f.a,tc,f.d+1,f.lt,false});
    }
    if(R.unresolved==0){R.strict_pass=true;R.location=0;return R;}
    if(hitI)R.location=3;
    else if(hitL&&hitR)R.location=4;
    else if(hitL)R.location=1;
    else if(hitR)R.location=2;
    else R.location=3;
    return R;
}

// AF7_HEADING_TOPOLOGY_R10B
struct HeadingTopoR10B {
    int cls=0; // 1 strict, 2 endpoint-only proven, 3 interior/core fail, 4 endpoint departure open
    bool strict=false,core=false,left_proven=false,right_proven=false;
    double v0=NAN,v1=NAN,a0=NAN,a1=NAN,J=NAN,dl=0,dr=0,core0=NAN,core1=NAN;
};

static inline HeadingTopoR10B heading_topology_r10b(
        const ActiveReference&ref,const Candidate&C,const SpeedCert&s,double ta,double tb){
    HeadingTopoR10B R;
    if(!(s.certified&&fin(s.J)&&s.J>=0&&fin(ta)&&fin(tb)&&tb>ta)){R.cls=4;return R;}
    R.J=s.J;
    if(c7r7_heading_regular(ref,C,s,ta,tb)){R.strict=true;R.core=true;R.cls=1;return R;}

    auto A=candidate_eval(ref,C,ta),B=candidate_eval(ref,C,tb);
    R.v0=std::hypot(A[1].x,A[1].y);R.v1=std::hypot(B[1].x,B[1].y);
    R.a0=std::hypot(A[2].x,A[2].y);R.a1=std::hypot(B[2].x,B[2].y);
    double scale=std::max({1.0,std::fabs(s.VL),std::fabs(s.VQ)});
    double zguard=512*epsd()*scale;
    bool z0=fin(R.v0)&&R.v0<=zguard,z1=fin(R.v1)&&R.v1<=zguard;
    R.left_proven=!z0;R.right_proven=!z1;

    const double T=tb-ta;
    if(z0){
        if(fin(R.a0)&&R.a0>zguard){
            R.dl=(s.J>0)?std::min(T,R.a0/s.J):T;
            R.left_proven=R.dl>0;
        }
    }
    if(z1){
        if(fin(R.a1)&&R.a1>zguard){
            R.dr=(s.J>0)?std::min(T,R.a1/s.J):T;
            R.right_proven=R.dr>0;
        }
    }

    if(!R.left_proven||!R.right_proven){R.cls=4;return R;}

    // Use half of each theorem-valid neighborhood for the closed core
    // boundary. This leaves a strict positive analytical margin:
    // ||v|| >= (3/8)*||a_xy||*delta when J>0.
    double trimL=z0?0.5*R.dl:0.0;
    double trimR=z1?0.5*R.dr:0.0;
    R.core0=ta+trimL;R.core1=tb-trimR;
    if(!(R.core1>R.core0)){
        // Endpoint theorem neighborhoods overlap and cover the whole interval.
        R.core=true;R.cls=(z0||z1)?2:1;return R;
    }
    R.core=c7r7_heading_regular(ref,C,s,R.core0,R.core1);
    if(!R.core){R.cls=3;return R;}
    R.cls=(z0||z1)?2:1;return R;
}

static inline PlanResult plan(const ActiveReference&ref,const Live&L,const SpeedCert&base,
                              const std::vector<Obstacle>&obs,double t0,double tm,double t1){
    // R37 layering contract:
    //   This native entry point certifies ONE FIXED terminal time t1.
    //   It does not perform adaptive time search internally.
    //
    // The MATLAB REPLANNING_SYSTEM owns adaptive rejoin selection by calling
    // this fixed-t1 primitive repeatedly.  Keeping the layers separate gives
    // two critical properties:
    //   (1) the full-domain previously flown t1 remains an exact fail-safe
    //       witness, so a failed compact/adaptive search cannot suppress
    //       obstacle avoidance; and
    //   (2) every shorter candidate is independently passed through the same
    //       finite-sensing, rejoin and continuous whole-body certificates.
    //
    // Spatial synthesis hierarchy inside plan_fixed_scope():
    //   primary  = exact minimum integrated squared deviation with affine unsafe spans;
    //   fallback = unchanged affine minimum-snap family in legacy cost order.
    // Safety is never an objective term; it remains a hard certificate.
    using clk=std::chrono::steady_clock;
    const auto Tall=clk::now();
    PlanResult OUT;
    OUT.search_end=t1;
    OUT.selected_t1=t1;

    if(!(fin(t0)&&fin(tm)&&fin(t1)&&t1>tm&&tm>t0&&L.dt>0)){
        OUT.reason=17;
        OUT.tim.total=std::chrono::duration<double,std::milli>(clk::now()-Tall).count();
        return OUT;
    }

    const auto tepi=clk::now();
    EpisodeCert E=build_episode(ref,L,base,obs,t0,t1);
    const double episodeMs=std::chrono::duration<double,std::milli>(clk::now()-tepi).count();

    if(!E.ok||E.comp.empty()||E.ep.empty()){
        OUT.episode=E;
        OUT.reason=22;
        OUT.tim.episode=episodeMs;
        OUT.tim.total=std::chrono::duration<double,std::milli>(clk::now()-Tall).count();
        return OUT;
    }

    OUT=plan_fixed_scope(ref,L,base,obs,t0,tm,t1,E);
    G_c7front_rows.clear();
    /* R8: obsolete R6B homotopy execution disabled; production result unchanged. */
    OUT.search_end=t1;
    OUT.selected_t1=t1;
    OUT.rejoin_attempts=1;
    OUT.rejoin_safe_trials=OUT.success?1:0;
    OUT.rejoin_scope_components=(int)E.comp.size();
    OUT.rejoin_merged_components=std::max(0,(int)E.comp.size()-1);
    OUT.search_speed_pass_total=OUT.speed_pass;
    OUT.search_rejoin_pass_total=OUT.rejoin_pass;
    OUT.search_geom_tests_total=OUT.geom_tests;
    OUT.spatial_envelope_priority=true;
    if(!E.comp.empty()){
        OUT.rejoin_selected_duration=t1-E.comp.back().end;
        OUT.rejoin_seed_duration=std::max(L.dt,E.comp.front().start-t0);
    }
    OUT.rejoin_next_gap=std::numeric_limits<double>::infinity();
    OUT.tim.episode=episodeMs;
    OUT.tim.rejoin_search=0.0;
    OUT.tim.total=std::chrono::duration<double,std::milli>(clk::now()-Tall).count();
    return OUT;
}

} // namespace af7r34
