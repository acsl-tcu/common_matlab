
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
            if(!(t>C.t0&&t<C.t1))continue;
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
    int objective_code=0; // 1=min correction energy, 2=min-snap recoverability
    double correction_energy=std::numeric_limits<double>::quiet_NaN();
    std::vector<int> signs;
    std::vector<double> seg0,seg1;
    std::vector<Poly> cp;
    std::vector<Vec3> pos,vel;
    std::array<double,6> dub{};
    double max_ctrl=0;
};
struct PlanResult {
    bool success=false;int reason=0;
    EpisodeCert episode;
    Candidate cand;
    SpeedCert speed;
    GeometryCert geom;
    int speed_pass=0,rejoin_pass=0,geom_tests=0;
    double hold_end=0;
    struct Tim {double episode=0,hessian=0,corridor=0,qp=0,candidate_build=0,synthesis=0,candidate=0,total=0;} tim;
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


// ---------------- minimum correction-energy Hessian ------------------------
// Formal performance objective used by AF7R49:
//
//      J_d = integral_{t0}^{t1} ||d(t)||^2 dt
//
// d(t) is the C7 correction already used by the unchanged safety proof.
// For fixed episode/corridor signs d(t) is linear in the QP variables z, so
// J_d = z' H_d z is a convex quadratic.  No tuning weight is introduced.
// The feasible set, C7 construction, R9 speed certificate, rejoin test and
// continuous whole-body certificate are unchanged.
static inline void add_transition_position_hessian(std::vector<double>&Hs,int nslot,
                                          const std::vector<std::array<double,8>>&J0,
                                          const std::vector<std::array<double,8>>&J1,double T){
    if(!(fin(T)&&T>0))throw std::runtime_error("nonpositive transition time");
    std::vector<std::vector<double>> B(nslot);
    for(int k=0;k<nslot;++k)B[k]=scalar_control_from_jets(J0[k],J1[k],T);
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j){
        double v=0;for(int a=0;a<16;++a)for(int b=0;b<16;++b)v+=B[i][a]*gram(15,a,b)*B[j][b];
        Hs[i*nslot+j]+=T*v;
    }
}
static inline void add_affine_position_hessian(std::vector<double>&Hs,int nslot,int compIndex,double T){
    if(!(fin(T)&&T>0))throw std::runtime_error("nonpositive affine component time");
    int ip=2*compIndex,iv=2*compIndex+1;
    std::array<double,16> bp{},bv{};
    for(int k=0;k<16;++k){bp[k]=1.0;bv[k]=T*(double(k)/15.0);}
    double hpp=0,hpv=0,hvv=0;
    for(int a=0;a<16;++a)for(int b=0;b<16;++b){
        double g=gram(15,a,b);
        hpp+=bp[a]*g*bp[b]; hpv+=bp[a]*g*bv[b]; hvv+=bv[a]*g*bv[b];
    }
    Hs[ip*nslot+ip]+=T*hpp;
    Hs[ip*nslot+iv]+=T*hpv;
    Hs[iv*nslot+ip]+=T*hpv;
    Hs[iv*nslot+iv]+=T*hvv;
}
static inline std::vector<double> minimum_correction_energy_hessian(const std::vector<Component>&comp,double t0,double t1){
    int C=(int)comp.size(),nslot=2*C;std::vector<double>Hs(nslot*nslot,0);
    std::vector<std::array<double,8>>J0,J1;
    scalar_endpoint_maps(comp,0,0,0,nslot,J0,J1);
    add_transition_position_hessian(Hs,nslot,J0,J1,comp[0].start-t0);
    for(int j=0;j<C;++j){
        add_affine_position_hessian(Hs,nslot,j,comp[j].end-comp[j].start);
        if(j<C-1){
            scalar_endpoint_maps(comp,j,j+1,1,nslot,J0,J1);
            add_transition_position_hessian(Hs,nslot,J0,J1,comp[j+1].start-comp[j].end);
        }
    }
    scalar_endpoint_maps(comp,C-1,0,2,nslot,J0,J1);
    add_transition_position_hessian(Hs,nslot,J0,J1,t1-comp.back().end);
    int n=3*nslot;std::vector<double>H(n*n,0);
    for(int i=0;i<nslot;++i)for(int j=0;j<nslot;++j)for(int d=0;d<3;++d)H[(3*i+d)*n+(3*j+d)]=Hs[i*nslot+j];
    return H;
}

static inline double candidate_correction_energy(const Candidate&C){
    double E=0;
    for(size_t s=0;s<C.cp.size();++s){
        double T=C.seg1[s]-C.seg0[s];
        if(!(fin(T)&&T>0&&C.cp[s].size()==16))return std::numeric_limits<double>::quiet_NaN();
        double q=0;
        for(int a=0;a<16;++a)for(int b=0;b<16;++b)q+=dot(C.cp[s][a],C.cp[s][b])*gram(15,a,b);
        E+=T*q;
    }
    return E;
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

// -------------------- build candidate control polygons ---------------------
static inline Candidate build_candidate(const std::vector<Component>&comp,double t0,double t1,
                                        const std::vector<double>&z,double cost,const std::vector<int>&signs,double minSlack,int objectiveCode){
    Candidate C;C.cost=cost;C.signs=signs;C.minSlack=minSlack;C.objective_code=objectiveCode;
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
        for(auto&f:leaves)if(fin(f.wit)&&f.wit>=lim){finiteWitness=true;return {};}
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
    if(C.cp.empty()||t<=C.seg0.front()||t>=C.seg1.back())return y;
    int s=-1;for(int i=0;i<(int)C.cp.size();++i){if(t<=C.seg1[i]){s=i;break;}}
    if(s<0||t<C.seg0[s])return y;
    double T=C.seg1[s]-C.seg0[s],u=std::max(0.0,std::min(1.0,(t-C.seg0[s])/T));Poly D=C.cp[s];
    y[0]+=decasteljau(D,u);int deg=15;
    for(int r=1;r<=6;++r){Poly E(D.size()-1);for(size_t i=0;i+1<D.size();++i)E[i]=(D[i+1]-D[i])*(double(deg)/T);D.swap(E);--deg;y[r]+=decasteljau(D,u);}
    return y;
}
static inline bool rejoin_safe(const ActiveReference&ref,const Live&L,const std::vector<Obstacle>&obs,
                               double t1,const SpeedCert&s){
    auto x=ref.eval(t1);Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};double ns=norm(sf);if(!(fin(ns)&&ns>0))return false;
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
struct OrientedRows {std::vector<Row> plus,minus;};
static inline PlanResult plan(const ActiveReference&ref,const Live&L,const SpeedCert&base,
                              const std::vector<Obstacle>&obs,double t0,double tm,double t1){
    using clk=std::chrono::steady_clock;auto T0=clk::now();PlanResult R;
    auto a=clk::now();R.episode=build_episode(ref,L,base,obs,t0,t1);R.tim.episode=std::chrono::duration<double,std::milli>(clk::now()-a).count();
    if(!R.episode.ok||R.episode.comp.empty()||R.episode.ep.empty()){R.reason=22;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;}
    if(R.episode.comp.front().start<=t0+8*epsd()*std::max(1.0,std::fabs(t0))){
        R.reason=14;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
    }
    if(R.episode.comp.back().end>=t1-8*epsd()*std::max(1.0,std::fabs(t1))){
        R.reason=17;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
    }
    R.hold_end=R.episode.comp.back().end;
    a=clk::now();
    int Cn=(int)R.episode.comp.size(),E=(int)R.episode.ep.size(),nvar=6*Cn;
    std::vector<OrientedRows> OR(E);
    auto ts=clk::now();
    for(int e=0;e<E;++e){
        auto ep=R.episode.ep[e];auto xa=ref.eval(ep.start),xb=ref.eval(ep.end);Vec3 prog=xb[0]-xa[0];
        if(norm(prog)<=128*epsd()*std::max(1.0,norm(xa[0])))prog=ref.eval(0.5*(ep.start+ep.end))[1];
        Vec3 n=lateral_normal(obs[ep.obs].R,obs[ep.obs].axes,prog);
        OR[e].plus=corridor_rows(ref,L,base,obs,R.episode,e,n,nvar);
        OR[e].minus=corridor_rows(ref,L,base,obs,R.episode,e,n*(-1),nvar);
    }
    R.tim.corridor=std::chrono::duration<double,std::milli>(clk::now()-ts).count();
    ts=clk::now();
    auto Hcorr=minimum_correction_energy_hessian(R.episode.comp,t0,t1);
    auto Hsnap=minimum_snap_hessian(R.episode.comp,t0,t1);
    R.tim.hessian=std::chrono::duration<double,std::milli>(clk::now()-ts).count();
    struct Base{std::vector<double>z;double c=0,s=0;std::vector<int>sg;int objective=0;};std::vector<Base>B;
    std::vector<int>sg(E,1);
    ts=clk::now();
    std::function<void(int)> rec=[&](int k){
        if(k<E){sg[k]=1;rec(k+1);sg[k]=-1;rec(k+1);return;}
        std::vector<Row> rows;for(int e=0;e<E;++e){auto&v=sg[e]>0?OR[e].plus:OR[e].minus;rows.insert(rows.end(),v.begin(),v.end());}
        QPSol qc=solve_qp(Hcorr,nvar,rows);if(qc.ok)B.push_back({qc.z,qc.cost,qc.slack,sg,1});
        // Existing minimum-snap synthesis is retained strictly as a
        // recoverability family. It does not weaken any acceptance test.
        QPSol qs=solve_qp(Hsnap,nvar,rows);if(qs.ok)B.push_back({qs.z,qs.cost,qs.slack,sg,2});
    };rec(0);
    R.tim.qp=std::chrono::duration<double,std::milli>(clk::now()-ts).count();
    std::stable_sort(B.begin(),B.end(),[](const auto&a,const auto&b){
        if(a.objective!=b.objective)return a.objective<b.objective; // correction-energy first
        double sc=std::max({1.0,std::fabs(a.c),std::fabs(b.c)});
        double g=65536*epsd()*sc;
        if(a.c < b.c-g) return true;
        if(b.c < a.c-g) return false;
        size_t n=std::min(a.sg.size(),b.sg.size());
        for(size_t i=0;i<n;++i){
            if(a.sg[i]!=b.sg[i]) return a.sg[i] < b.sg[i]; // -1 before +1
        }
        return a.sg.size()<b.sg.size();
    });
    ts=clk::now();
    std::vector<Candidate> candidates;
    for(auto&b:B){
        Candidate c=build_candidate(R.episode.comp,t0,t1,b.z,b.c,b.sg,b.s,b.objective);
        c.correction_energy=candidate_correction_energy(c);
        candidates.push_back(std::move(c));
    }
    R.tim.candidate_build=std::chrono::duration<double,std::milli>(clk::now()-ts).count();
    R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();
    if(candidates.empty()){R.reason=23;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;}
    double rhoBody=std::max({L.rQ,L.L+L.rL,L.L+L.rC});
    double lim=(L.sensor_range-rhoBody)/(L.horizon+L.dt);
    if(!(fin(lim)&&lim>0)){R.reason=16;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;}
    a=clk::now();
    for(auto cand:candidates){
        SpeedCert sc;auto tr=tighten_stageA(cand,base,L.L,lim);
        if(tr.certified){
            sc=stageA_speed(tr,base,L.L);
            if(sc.certified&&sc.VQ<lim){cand.dub[0]=tr.U[0];cand.dub[1]=tr.U[1];cand.dub[2]=tr.U[2];}
            else sc.certified=false;
        }
        if(!sc.certified){bool wit=false,rp=false;sc=stageB_speed(ref,L,base,cand,lim,wit,rp);}
        if(!sc.certified)continue;
        ++R.speed_pass;if(!rejoin_safe(ref,L,obs,t1,sc))continue;
        ++R.rejoin_pass;++R.geom_tests;auto G=wholebody(ref,L,obs,t0,tm,R.hold_end,t1,cand,sc);
        if(G.pass){R.success=true;R.reason=0;R.cand=std::move(cand);R.speed=sc;R.geom=G;break;}
    }
    R.tim.candidate=std::chrono::duration<double,std::milli>(clk::now()-a).count();
    if(!R.success)R.reason=R.speed_pass==0?24:25;
    R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
}

} // namespace af7r34
