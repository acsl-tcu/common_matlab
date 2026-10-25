
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
#include <queue>
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
// AF7R58 clean arithmetic: allocation-free degree-15 Bernstein evaluation.
// Same left-to-right de Casteljau order and Vec3 arithmetic as decasteljau().
// Strictly used with degree <=15; dynamically sized paths keep legacy code.
static inline Vec3 r58_decasteljau_fixed16(const Vec3* cp,int count,double u){
    if(count<1||count>16||cp==nullptr)
        throw std::runtime_error("R58 fixed Bernstein invalid degree");
    std::array<Vec3,16> a{};
    for(int i=0;i<count;++i)a[i]=cp[i];
    for(int k=count;k>1;--k){
        for(int i=0;i<k-1;++i)
            a[i]=a[i]*(1-u)+a[i+1]*u;
    }
    return a[0];
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
static inline const std::array<std::array<double,8>,8>& r58r23_binomial_table();

static inline std::vector<double> scalar_control_from_jets(const std::array<double,8>&j0,
                                                           const std::array<double,8>&j1,double T){
    std::array<double,8> ds{},de{};ds[0]=j0[0];de[0]=j1[0];
    double ff=1;
    for(int r=1;r<=7;++r){ff*=16-r;ds[r]=j0[r]*std::pow(T,r)/ff;de[r]=j1[r]*std::pow(T,r)/ff;}
    std::vector<double>P(16,0);
    const auto& chooseTable=r58r23_binomial_table();
    for(int k=0;k<=7;++k){
        double v0=0,v1=0;
        for(int r=0;r<=k;++r){
            double bc=chooseTable[k][r];v0+=bc*ds[r];v1+=((r&1)?-1.0:1.0)*bc*de[r];
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

// ===========================================================================
// AF7R51 GENERAL C7 PHASE REPARAMETERIZATION
//
// The phase law is independent of the spatial shape of the base reference.
// It maps physical time t to source/base-reference time s(t), with
//
//      0 < ds/dt <= 1
//
// continuously.  Therefore the vehicle may slow its progress through the
// original parameterized spatial path, but never catches up to a later source
// phase.  After the exit transition, s(t)=t-Delta and the delay Delta is held.
//
// This representation is valid for any C7 active reference: straight,
// saddle/curved, or an already-repaired C7 path.
// ===========================================================================
struct R51PhaseSpec{
    bool enabled=false;
    double t0=0.0;     // phase transition support start
    double tA=0.0;     // end 1 -> alpha transition
    double tB=0.0;     // end constant-alpha flow
    double tE=0.0;     // end alpha -> 1 transition
    double sA=0.0;     // source phase at tA
    double sB=0.0;     // source phase at tB
    double alpha=1.0;  // constant flow ds/dt
    double delay=0.0;  // persistent phase delay after tE
};

static inline double r51_scalar_eval(std::vector<double>P,double u){
    while(P.size()>1){
        std::vector<double>Q(P.size()-1);
        for(size_t i=0;i+1<P.size();++i)Q[i]=(1.0-u)*P[i]+u*P[i+1];
        P.swap(Q);
    }
    return P.empty()?0.0:P[0];
}
static inline std::vector<double> r51_diff_scalar(
        std::vector<double>P,int r){
    for(int k=0;k<r;++k){
        if(P.size()<2)return {};
        std::vector<double>Q(P.size()-1);
        for(size_t i=0;i+1<P.size();++i)Q[i]=P[i+1]-P[i];
        P.swap(Q);
    }
    return P;
}


static inline bool r51_phase_valid(const R51PhaseSpec&P){
    if(!P.enabled)return true;
    return fin(P.t0)&&fin(P.tA)&&fin(P.tB)&&fin(P.tE)&&
           fin(P.sA)&&fin(P.sB)&&fin(P.alpha)&&fin(P.delay)&&
           P.t0<P.tA&&P.tA<P.tB&&P.tB<P.tE&&
           P.sB>P.sA&&P.alpha>0.0&&P.alpha<=1.0&&P.delay>=0.0;
}

static inline std::array<double,7> r51_delay_jet(
        const R51PhaseSpec&P,double t){
    std::array<double,7>D{};
    if(!P.enabled||t<=P.t0)return D;
    if(t>=P.tE){D[0]=P.delay;return D;}

    const double beta=1.0-P.alpha;
    std::vector<double> C;
    double ta=0.0,tb=0.0;
    if(t<P.tA){
        ta=P.t0;tb=P.tA;
        std::array<double,8>J0{},J1{};
        J0[0]=0.0;J0[1]=0.0;
        J1[0]=P.tA-P.sA;J1[1]=beta;
        C=scalar_control_from_jets(J0,J1,tb-ta);
    }else if(t<=P.tB){
        D[0]=(P.tA-P.sA)+beta*(t-P.tA);
        D[1]=beta;
        return D;
    }else{
        ta=P.tB;tb=P.tE;
        std::array<double,8>J0{},J1{};
        J0[0]=P.tB-P.sB;J0[1]=beta;
        J1[0]=P.delay;J1[1]=0.0;
        C=scalar_control_from_jets(J0,J1,tb-ta);
    }

    const double T=tb-ta;
    if(!(fin(T)&&T>0.0))return D;
    double u=(t-ta)/T;
    u=std::max(0.0,std::min(1.0,u));
    std::vector<double> Q=C;
    int deg=15;
    D[0]=r51_scalar_eval(Q,u);
    for(int r=1;r<=6;++r){
        Q=r51_diff_scalar(Q,1);
        const double sc=double(deg)/T;
        for(double&x:Q)x*=sc;
        --deg;
        D[r]=r51_scalar_eval(Q,u);
    }
    return D;
}

static inline std::array<double,7> r51_phase_jet(
        const R51PhaseSpec&P,double t){
    auto D=r51_delay_jet(P,t);
    std::array<double,7>S{};
    S[0]=t-D[0];
    S[1]=1.0-D[1];
    for(int r=2;r<=6;++r)S[r]=-D[r];
    return S;
}

static inline std::array<Vec3,7> r51_compose_vec_jet(
        const std::array<Vec3,7>&F,const std::array<double,7>&S){
    // Taylor-series composition to derivative order 6.
    static const double fact[7]={1,1,2,6,24,120,720};
    std::array<Vec3,7> coef{};
    std::array<double,7> q{};
    q[0]=0.0;
    for(int r=1;r<=6;++r)q[r]=S[r]/fact[r];

    std::array<double,7> pw{};pw[0]=1.0;
    for(int k=0;k<=6;++k){
        const double fk=1.0/fact[k];
        for(int n=0;n<=6;++n)
            coef[n]+=F[k]*(fk*pw[n]);
        if(k==6)break;
        std::array<double,7> nx{};
        for(int i=0;i<=6;++i)if(pw[i]!=0.0)
            for(int j=1;i+j<=6;++j)if(q[j]!=0.0)
                nx[i+j]+=pw[i]*q[j];
        pw=nx;
    }
    std::array<Vec3,7> out{};
    for(int n=0;n<=6;++n)out[n]=coef[n]*fact[n];
    return out;
}

static inline bool r51_phase_derivative_bounds(
        const R51PhaseSpec&P,double&smin,double&smax,double&S2,double&S3){
    if(!P.enabled){smin=smax=1.0;S2=S3=0.0;return true;}
    if(!r51_phase_valid(P))return false;
    smin=std::numeric_limits<double>::infinity();
    smax=-std::numeric_limits<double>::infinity();
    S2=S3=0.0;

    auto consume=[&](const std::vector<double>&D0,double T)->bool{
        if(!(fin(T)&&T>0.0&&D0.size()==16))return false;
        auto D1=r51_diff_scalar(D0,1);
        auto D2=r51_diff_scalar(D0,2);
        auto D3=r51_diff_scalar(D0,3);
        double f1=15.0/T;
        double f2=15.0*14.0/(T*T);
        double f3=15.0*14.0*13.0/(T*T*T);
        for(double x:D1){double sp=1.0-f1*x;smin=std::min(smin,sp);smax=std::max(smax,sp);}
        for(double x:D2)S2=std::max(S2,std::fabs(f2*x));
        for(double x:D3)S3=std::max(S3,std::fabs(f3*x));
        return true;
    };

    const double beta=1.0-P.alpha;
    {
        std::array<double,8>J0{},J1{};
        J1[0]=P.tA-P.sA;J1[1]=beta;
        if(!consume(scalar_control_from_jets(J0,J1,P.tA-P.t0),P.tA-P.t0))
            return false;
    }
    smin=std::min(smin,P.alpha);smax=std::max(smax,P.alpha);
    {
        std::array<double,8>J0{},J1{};
        J0[0]=P.tB-P.sB;J0[1]=beta;
        J1[0]=P.delay;
        if(!consume(scalar_control_from_jets(J0,J1,P.tE-P.tB),P.tE-P.tB))
            return false;
    }
    const double tol=4096*epsd()*std::max(1.0,std::max(std::fabs(smin),std::fabs(smax)));
    return fin(smin)&&fin(smax)&&fin(S2)&&fin(S3)&&
           smin>0.0&&smax<=1.0+tol;
}

struct Correction { double t0=0,t1=0; std::vector<CorrSegment> seg; R51PhaseSpec phase; };

struct ActiveReference {
    const NominalExpr* nominal=nullptr;
    double flight_t0=0;
    std::vector<Correction> repairs;

    std::array<Vec3,7> eval(double t) const{
        const size_t G=repairs.size();

        // Bounded nonrecursive evaluation of sequential reference layers.
        // Each layer may first reparameterize the previous layer by its C7
        // phase map, then add its ordinary degree-15 spatial correction.
        std::vector<double> T(G+1,t);
        std::vector<std::array<double,7>> SJ(G);
        for(size_t kk=G;kk>0;--kk){
            const size_t g=kk-1;
            SJ[g]=r51_phase_jet(repairs[g].phase,T[g+1]);
            T[g]=SJ[g][0];
        }

        const double tau=std::max(0.0,T[0]-flight_t0);
        auto y=nominal->eval(tau);

        for(size_t g=0;g<G;++g){
            const auto&C=repairs[g];
            y=r51_compose_vec_jet(y,SJ[g]);
            const double tg=T[g+1];

            if(tg<=C.t0)continue;

            if(!C.phase.enabled && tg>=C.t1){
                // Historical constant-hold semantics.
                if(!C.seg.empty()) y[0]+=C.seg.back().cp.back();
                continue;
            }

            const CorrSegment* sp=nullptr;
            for(const auto& q:C.seg){if(tg<=q.t1){sp=&q;break;}}
            if(!sp||tg<sp->t0)continue;

            const double TT=sp->t1-sp->t0;
            if(!(fin(TT)&&TT>0.0))continue;
            double u=(tg-sp->t0)/TT;
            u=std::max(0.0,std::min(1.0,u));
            if(sp->cp.size()==16){
                // Same floating-point operations as the previous vector
                // implementation, but without 100+ heap allocations per
                // active-reference evaluation (position and derivatives 1..6).
                std::array<Vec3,16> D{};
                for(int i=0;i<16;++i)D[i]=sp->cp[i];
                int n=16,deg=15;
                y[0]+=r58_decasteljau_fixed16(D.data(),n,u);
                for(int r=1;r<=6;++r){
                    for(int i=0;i<n-1;++i)
                        D[i]=(D[i+1]-D[i])*(double(deg)/TT);
                    --n;--deg;
                    y[r]+=r58_decasteljau_fixed16(D.data(),n,u);
                }
            }else{
                // Non-degree-15 representation is never converted or approximated.
                Poly D=sp->cp;
                y[0]+=decasteljau(D,u);
                int deg=15;
                for(int r=1;r<=6;++r){
                    Poly E(D.size()-1);
                    for(size_t i=0;i+1<D.size();++i)
                        E[i]=(D[i+1]-D[i])*(double(deg)/TT);
                    D.swap(E);--deg;
                    y[r]+=decasteljau(D,u);
                }
            }
        }
        return y;
    }
};


// R78: Native derivative order 7 of the *same serialized correction packet*
// consumed by ActiveReference::eval(order 0..6).  No phase case only:
// for a general retiming, p^(7) needs the entire seventh-order chain rule,
// and MUST NOT be silently evaluated with the identity-time formula.
// This routine DOES NOT authorize a trajectory or relax any hard certificate.
static inline Vec3 r78_correction_jet7_identity(const ActiveReference&ref,double t){
    Vec3 total{};
    if(!fin(t))throw std::runtime_error("R78 nonfinite time");
    for(const auto&C:ref.repairs){
        if(C.phase.enabled)
            throw std::runtime_error("R78 phase layer requires full seventh-order composition");
        if(t<=C.t0 || t>=C.t1)continue; // same side conventions as ref.eval
        const CorrSegment* sp=nullptr;
        for(const auto&sg:C.seg){if(t<=sg.t1){sp=&sg;break;}}
        if(!sp || t<sp->t0)continue;
        const double T=sp->t1-sp->t0;
        if(!(fin(T)&&T>0.0) || sp->cp.size()!=16)
            throw std::runtime_error("R78 invalid degree-15 correction");
        const double u=std::max(0.0,std::min(1.0,(t-sp->t0)/T));
        std::array<Vec3,16> D{};
        for(int i=0;i<16;++i)D[i]=sp->cp[i];
        int n=16,deg=15;
        for(int r=1;r<=7;++r){
            for(int i=0;i<n-1;++i)
                D[i]=(D[i+1]-D[i])*(double(deg)/T);
            --n;--deg;
        }
        total+=r58_decasteljau_fixed16(D.data(),n,u);
    }
    return total;
}


static inline std::array<Vec3,7> r51_phase_reference_eval(
        const ActiveReference&ref,const R51PhaseSpec&P,double t){
    if(!P.enabled)return ref.eval(t);
    const auto S=r51_phase_jet(P,t);
    auto b=ref.eval(S[0]);
    return r51_compose_vec_jet(b,S);
}

static inline double r51_phase_inverse_flow(
        const R51PhaseSpec&P,double sourceTime){
    if(!P.enabled||!(P.alpha>0.0))return sourceTime;
    return P.tA+(sourceTime-P.sA)/P.alpha;
}

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
    // R13 diagnostic-only: no effects on hard acceptance or fallback.
    bool r58_detail_diagnostic_enabled=false; // R58 pure diagnostics only; NEVER controls bridge or proof
    bool r58_shadow_diagnostic_enabled=false;
    bool r58r14_lowjet_seed_only=false;
    bool r58r16_reference_bridge=false; // opt-in, reference-only proposal; normal commit guards stay active
};

// AF7R52D2 diagnostic-only gate trace.
// It never grants acceptance and does not modify any planner parameter,
// trajectory family, hard gate, runtime mass authority, or fallback semantics.
struct R52StageBTrace {
    int attempted=0;
    int result_code=0; // 1 certified, 2 finite witness, 3 representability, 4 terminal failure
    int leaf_count=0,split_count=0;
    int worst_seg=-1;
    double worst_t=std::numeric_limits<double>::quiet_NaN();
    double worst_vq=-std::numeric_limits<double>::infinity();
    double worst_vl=std::numeric_limits<double>::quiet_NaN();
    double worst_sigma=std::numeric_limits<double>::quiet_NaN();
    double worst_qdot=std::numeric_limits<double>::quiet_NaN();
    double worst_witness=std::numeric_limits<double>::quiet_NaN();
};

struct R52HeadingTrace {
    int attempted=0;
    int reason=0; // 0 pass; 1 input; 2 Lipschitz; 3 midpoint/representation; 4 terminal gamma; 5 anchor
    int leaves=0,splits=0;
    double min_g0=std::numeric_limits<double>::infinity();
    double min_lb=std::numeric_limits<double>::infinity();
    double worst_a=std::numeric_limits<double>::quiet_NaN();
    double worst_b=std::numeric_limits<double>::quiet_NaN();
};

struct R52CandidateGateDiag {
    double seq=0;
    int family=0;
    int exact_c7=0;
    double max_ctrl=std::numeric_limits<double>::quiet_NaN();
    double du=std::numeric_limits<double>::quiet_NaN();
    double da=std::numeric_limits<double>::quiet_NaN();
    double dj=std::numeric_limits<double>::quiet_NaN();
    double phase_alpha=std::numeric_limits<double>::quiet_NaN();
    double phase_delay=std::numeric_limits<double>::quiet_NaN();
    double actual_rejoin=std::numeric_limits<double>::quiet_NaN();

    double limit=std::numeric_limits<double>::quiet_NaN();
    double base_VL=std::numeric_limits<double>::quiet_NaN();
    double base_A=std::numeric_limits<double>::quiet_NaN();
    double base_J=std::numeric_limits<double>::quiet_NaN();
    double base_sigma=std::numeric_limits<double>::quiet_NaN();

    double A_Uv=std::numeric_limits<double>::quiet_NaN();
    double A_Ua=std::numeric_limits<double>::quiet_NaN();
    double A_Uj=std::numeric_limits<double>::quiet_NaN();
    double A_sigma=std::numeric_limits<double>::quiet_NaN();
    double A_VQ=std::numeric_limits<double>::quiet_NaN();
    int A_cert=0,A_impossible=0,A_repr=0;

    R52StageBTrace B;
    int speed_pass=0;

    R52HeadingTrace H;
    int heading_pass=0;
    double heading_gamma=std::numeric_limits<double>::quiet_NaN();

    int rejoin_attempt=0,rejoin_pass=0;
    int geom_attempt=0,geom_pass=0;
    double geom_min_margin=std::numeric_limits<double>::quiet_NaN();
};

static std::vector<R52CandidateGateDiag> G_r52_gate_diag;
static double G_r52_gate_diag_seq=0.0;

struct R53KnotDiagRow{
    double build_id=0,side=0,rejoin=0,corr_v=0,corr_a=0,corr_j=0,node=0,knot=0;
    Vec3 P{},V{};
    double Pnorm=0,Vnorm=0,prevT=0,nextT=0;
};
struct R53SegmentDiagRow{
    double build_id=0,side=0,rejoin=0,corr_v=0,corr_a=0,corr_j=0;
    double seg=0,ta=0,tb=0,T=0,cp_norm=0,vel_ub=0,acc_ub=0,jerk_ub=0;
};
static std::vector<R53KnotDiagRow> G_r53_knot_diag;
static std::vector<R53SegmentDiagRow> G_r53_segment_diag;
static double G_r53_build_seq=0.0;
static inline void r53_shape_diag_reset(){
    G_r53_knot_diag.clear();
    G_r53_segment_diag.clear();
    G_r53_build_seq=0.0;
}
static inline void r52_gate_diag_reset(){
    G_r52_gate_diag.clear();
    G_r52_gate_diag_seq=0.0;
}
static inline void r52_gate_diag_push(R52CandidateGateDiag d){
    d.seq=++G_r52_gate_diag_seq;
    if(G_r52_gate_diag.size()>=256)G_r52_gate_diag.erase(G_r52_gate_diag.begin());
    G_r52_gate_diag.push_back(std::move(d));
}
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
    //         7 = active-reference additive exact-C7 tangent-corridor primary.
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
    // AF7R51 general source-phase reparameterization.
    bool phase_reparameterized=false;
    R51PhaseSpec phase;

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
    // R13: one row per R58 full-branch speed assessment (35 columns),
    // constructed ONLY when explicitly running the non-committable R58 shadow.
    std::vector<std::array<double,38>> r58_speed_gate_rows;
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
    // AF7R47 active-reference replan / yaw-C6 diagnostics.
    int r47_active_base_replan_count=0;
    int r47_nonaffine_base_count=0;
    int r47_heading_leaf_count=0;
    int r47_heading_split_count=0;
    // AF7R48 automatic natural-rejoin / no-overshoot diagnostics.
    int r48_natural_rejoin_trial_count=0;
    int r48_natural_rejoin_selected_count=0;
    int r48_exit_monotone_row_count=0;
    // AF7R49 constructive no-catch-up phase timing diagnostics.
    int r49_phase_timing_trial_count=0;
    int r49_phase_gate_pass_count=0;
    int r49_hard_certificate_trial_count=0;
    long long r49_selected_Ke=0;
    long long r49_selected_Kflow=0;
    long long r49_selected_Kx=0;
    // AF7R50 phase/lateral decoupling diagnostics.
    int r50_spatial_side_solution_count=0;
    int r50_spatial_lateral_constraint_count=0;
    int r50_entry_candidate_count=0;
    int r50_exit_candidate_count=0;
    double r50_best_spatial_velocity_norm=std::numeric_limits<double>::infinity();
    // AF7R51 general-C7 phase/tangent primary diagnostics.
    int r51_phase_seed_count=0;
    int r51_phase_seed_reject_count=0;
    int r51_tangent_candidate_count=0;
    int r51_tangent_certified_count=0;
    double r51_phase_alpha=std::numeric_limits<double>::quiet_NaN();
    double r51_phase_delay_s=std::numeric_limits<double>::quiet_NaN();
    double r51_phase_exit_time_s=std::numeric_limits<double>::quiet_NaN();
    // AF7R52 general-C7 world-coordinate corridor diagnostics.
    int r52_phase_episode_count=0;
    int r52_phase_episode_rebuild_fail_count=0;
    int r52_corridor_row_count=0;
    int r52_support_plane_fail_count=0;
    int r52_spatial_qp_solution_count=0;
    int r52_rejoin_time_trial_count=0;
    double r52_selected_spatial_rejoin_s=std::numeric_limits<double>::quiet_NaN();
    int r53_normal_bundle_node_count=0;
    int r53_normal_bundle_row_count=0;
    int r53_normal_bundle_singular_count=0;
    int r53_affine_reduction_count=0;
    // AF7R54 horizon-decoupled adaptive phase continuation diagnostics.
    int r54_phase_candidate_count=0;
    int r54_phase_trial_count=0;
    int r54_phase_base_certified_count=0;
    int r54_phase_episode_pass_count=0;
    int r54_phase_episode_empty_count=0;
    long long r54_selected_tau_multiple=0;
    double r54_selected_alpha=std::numeric_limits<double>::quiet_NaN();
    double r54_selected_delay_s=std::numeric_limits<double>::quiet_NaN();
    double r54_selected_first_unsafe_lead_s=std::numeric_limits<double>::quiet_NaN();
    double r54_max_first_unsafe_lead_s=-std::numeric_limits<double>::infinity();
    int r54_maneuver_horizon_coupling_count=0; // must remain exactly zero
    // AF7R55 balanced time allocation + mixed oriented-ellipsoid corridor.
    int r55_phase_build_count=0;
    int r55_phase_build_fail_count=0;
    int r55_phase_episode_pass_count=0;
    int r55_phase_episode_empty_count=0;
    int r55_mixed_side_episode_count=0;
    int r55_side_plus_count=0;
    int r55_side_minus_count=0;
    int r55_side_onebranch_count=0;
    int r55_side_both_failed_count=0;
    int r55_corridor_row_count=0;
    int r55_obstacle_count=0;
    int r55_old_clock_binding_count=0;
    int r55_maneuver_horizon_coupling_count=0;
    double r55_alpha=std::numeric_limits<double>::quiet_NaN();
    double r55_delay_s=std::numeric_limits<double>::quiet_NaN();
    double r55_source_unsafe_start_s=std::numeric_limits<double>::quiet_NaN();
    double r55_source_unsafe_end_s=std::numeric_limits<double>::quiet_NaN();
    double r55_predicted_first_unsafe_t_s=std::numeric_limits<double>::quiet_NaN();
    double r55_predicted_last_unsafe_t_s=std::numeric_limits<double>::quiet_NaN();
    double r55_pre_unsafe_slack_s=std::numeric_limits<double>::quiet_NaN();
    double r55_post_unsafe_slack_s=std::numeric_limits<double>::quiet_NaN();
    double r55_min_temporal_slack_s=std::numeric_limits<double>::quiet_NaN();
    double r55_phase_entry_transition_s=std::numeric_limits<double>::quiet_NaN();
    double r55_phase_exit_transition_s=std::numeric_limits<double>::quiet_NaN();
    double r55_source_phase_at_rejoin_s=std::numeric_limits<double>::quiet_NaN();
    double r55_rejoin_clock_delay_s=std::numeric_limits<double>::quiet_NaN();
    double r55_min_ellipsoid_axis_m=std::numeric_limits<double>::infinity();
    double r55_max_ellipsoid_axis_m=0.0;
    double r55_max_rotation_orth_error=0.0;
    // AF7R56 THEORY/SHADOW: source-phase load path + independent time law.
    // Diagnostic only; none of these fields may grant production acceptance.
    int r56_shadow_ran=0;
    int r56_source_qp_ok=0;
    int r56_source_exact_c7=0;
    int r56_source_knot_count=0;
    int r56_source_episode_count=0;
    int r56_source_corridor_row_count=0;
    int r56_source_normal_row_count=0;
    int r56_source_side_plus_count=0;
    int r56_source_side_minus_count=0;
    int r56_source_side_fail_count=0;
    int r56_const_sample_count=0;
    int r56_const_dynamic_pass=0;
    int r56_const_geometry_point_pass=0;
    int r56_entry_bestcase_pass=0;
    int r56_exit_bestcase_pass=0;
    int r56_yaw_source_regular=0;
    int r56_old_clock_binding_count=0;          // must remain zero
    int r56_horizon_duration_coupling_count=0; // must remain zero
    double r56_source_support_start_s=std::numeric_limits<double>::quiet_NaN();
    double r56_source_rejoin_s=std::numeric_limits<double>::quiet_NaN();
    double r56_source_corr_v_ub=std::numeric_limits<double>::quiet_NaN();
    double r56_source_corr_a_ub=std::numeric_limits<double>::quiet_NaN();
    double r56_source_corr_j_ub=std::numeric_limits<double>::quiet_NaN();
    double r56_source_min_hxy=std::numeric_limits<double>::quiet_NaN();
    double r56_alpha_max_pointwise=std::numeric_limits<double>::quiet_NaN();
    double r56_const_max_vl=std::numeric_limits<double>::quiet_NaN();
    double r56_const_max_vq=std::numeric_limits<double>::quiet_NaN();
    double r56_const_min_sigma=std::numeric_limits<double>::quiet_NaN();
    double r56_const_max_qdot=std::numeric_limits<double>::quiet_NaN();
    double r56_const_min_geom_margin=std::numeric_limits<double>::quiet_NaN();
    double r56_entry_Tmax_s=std::numeric_limits<double>::quiet_NaN();
    double r56_entry_max_vq=std::numeric_limits<double>::quiet_NaN();
    double r56_entry_min_sigma=std::numeric_limits<double>::quiet_NaN();
    double r56_entry_max_qdot=std::numeric_limits<double>::quiet_NaN();
    double r56_exit_Tmax_s=std::numeric_limits<double>::quiet_NaN();
    double r56_exit_max_vq=std::numeric_limits<double>::quiet_NaN();
    double r56_exit_min_sigma=std::numeric_limits<double>::quiet_NaN();
    double r56_exit_max_qdot=std::numeric_limits<double>::quiet_NaN();
    double r56_chain_rule_max_error=0.0;
    double r56_shadow_ms=0.0;
    // AF7R57 diagnostic-only continuous source-path probe.
    // Never commits a candidate and never selects a production rejoin time.
    int r57_probe_ran=0;
    int r57_source_qp_ok=0;
    int r57_source_exact_c7=0;
    int r57_speed_attempt=0;
    int r57_speed_pass=0;
    int r57_speed_stageA_pass=0;
    int r57_speed_stageB_attempt=0;
    int r57_speed_stageB_finite_witness=0;
    int r57_speed_stageB_repr_fail=0;
    int r57_heading_attempt=0;
    int r57_heading_pass=0;
    int r57_heading_leaf_count=0;
    int r57_heading_split_count=0;
    int r57_geometry_attempt=0;
    int r57_geometry_pass=0;
    int r57_commit_attempted=0;               // must remain zero
    int r57_rejoin_selected=0;                // must remain zero
    int r57_old_clock_commit_binding_count=0; // must remain zero
    int r57_horizon_duration_coupling_count=0;// must remain zero
    double r57_speed_VL=std::numeric_limits<double>::quiet_NaN();
    double r57_speed_A=std::numeric_limits<double>::quiet_NaN();
    double r57_speed_J=std::numeric_limits<double>::quiet_NaN();
    double r57_speed_sigma=std::numeric_limits<double>::quiet_NaN();
    double r57_speed_qdot=std::numeric_limits<double>::quiet_NaN();
    double r57_speed_VQ=std::numeric_limits<double>::quiet_NaN();
    double r57_heading_gamma=std::numeric_limits<double>::quiet_NaN();
    double r57_geometry_min_margin=std::numeric_limits<double>::quiet_NaN();
    double r57_source_support_start_s=std::numeric_limits<double>::quiet_NaN();
    double r57_source_support_end_s=std::numeric_limits<double>::quiet_NaN();
    double r57_source_corr_v_ub=std::numeric_limits<double>::quiet_NaN();
    double r57_source_corr_a_ub=std::numeric_limits<double>::quiet_NaN();
    double r57_source_corr_j_ub=std::numeric_limits<double>::quiet_NaN();
    double r57_source_build_ms=0.0;
    double r57_speed_ms=0.0;
    double r57_heading_ms=0.0;
    double r57_geometry_ms=0.0;
    double r57_probe_ms=0.0;
    // AF7R58 free-C7 / global-homotopy reference-core shadow.
    int r58_shadow_ran=0;
    int r58_reference_only=1;
    int r58_fixed_rejoin_constraint_count=0;
    int r58_same_clock_binding_count=0;
    int r58_horizon_duration_coupling_count=0;
    int r58_free_c7_constructive=0;
    int r58_terminal_offset_hold=0;
    int r58_terminal_zero_derivative_order=0;
    int r58_source_knot_count=0;
    int r58_scalar_dof_count=0;
    int r58r14_lowjet_seed_only=0;
    int r58r14_active_reduced_dof=0;
    int r58r14_full_reduced_dof=0;
    int r58r16_connected_candidate=0;
    int r58_spatial_dof_count=0;
    int r58_episode_count=0;
    int r58_component_count=0;
    int r58_side_cache_plus_valid=0;
    int r58_side_cache_minus_valid=0;
    int r58_side_cache_fail=0;
    int r58_bnb_nodes_popped=0;
    int r58_bnb_qp_solves=0;
    int r58_bnb_qp_infeasible=0;
    int r58_bnb_full_assignments=0;
    int r58_bnb_speed_fail=0;
    int r58_bnb_heading_fail=0;
    int r58_bnb_geometry_fail=0;
    int r58_bnb_feasible_full=0;
    int r58_bnb_max_queue=0;
    int r58_selected_side_plus=0;
    int r58_selected_side_minus=0;
    int r58_speed_pass=0;
    int r58_heading_pass=0;
    int r58_geometry_pass=0;
    int r58_needs_time_law=0;
    int r58_commit_attempted=0;
    int r58_rejoin_selected=0;
    double r58_reference_tracking_inflation_m=std::numeric_limits<double>::quiet_NaN();
    double r58_support_start_s=std::numeric_limits<double>::quiet_NaN();
    double r58_support_end_s=std::numeric_limits<double>::quiet_NaN();
    double r58_terminal_offset_norm_m=std::numeric_limits<double>::quiet_NaN();
    double r58_quantized_jet_max_abs_error=0.0;
    double r58_legacy_join_max_abs_mismatch=0.0;
    double r58_best_qp_cost=std::numeric_limits<double>::infinity();
    double r58_speed_VL=std::numeric_limits<double>::quiet_NaN();
    double r58_speed_A=std::numeric_limits<double>::quiet_NaN();
    double r58_speed_J=std::numeric_limits<double>::quiet_NaN();
    double r58_speed_sigma=std::numeric_limits<double>::quiet_NaN();
    double r58_speed_qdot=std::numeric_limits<double>::quiet_NaN();
    double r58_speed_VQ=std::numeric_limits<double>::quiet_NaN();
    double r58_heading_gamma=std::numeric_limits<double>::quiet_NaN();
    double r58_geometry_min_margin=std::numeric_limits<double>::quiet_NaN();
    double r58_build_ms=0.0;
    double r58_homotopy_ms=0.0;
    double r58_certificate_ms=0.0;
    double r58_shadow_ms=0.0;
    // AF7R58R2 numerically equivalent conditioning diagnostics.
    int r58r2_node_scaled_variables=0;
    int r58r2_row_equilibrated=0;
    int r58r2_root_qp_elided=0;
    double r58r2_node_scale_min_s=std::numeric_limits<double>::infinity();
    double r58r2_node_scale_max_s=0.0;
    double r58r2_row_scale_min=std::numeric_limits<double>::infinity();
    double r58r2_row_scale_max=0.0;
    // AF7R58R3 exact-coordinate Hessian diagonal equilibration.
    int r58r3_hessian_diag_equilibrated=0;
    int r58r3_equilibrated_cholesky_pass=0;
    double r58r3_raw_hdiag_min=std::numeric_limits<double>::infinity();
    double r58r3_raw_hdiag_max=0.0;
    double r58r3_raw_hdiag_ratio=std::numeric_limits<double>::infinity();
    double r58r3_eq_hdiag_min=std::numeric_limits<double>::infinity();
    double r58r3_eq_hdiag_max=0.0;
    // AF7R58R4 analytical elimination of moving-normal equalities.
    int r58r4_normal_equalities_eliminated=0;
    int r58r4_reduced_spatial_dof_count=0;
    int r58r4_elimination_build_fail=0;
    double r58r4_elimination_max_residual=0.0;
    // AF7R58R5 H-metric Dykstra solver diagnostics.
    int r58r5_dykstra_used=0;
    int r58r5_dykstra_converged_feasible=0;
    int r58r5_dykstra_resource_fail=0;
    int r58r5_dykstra_degenerate_row_fail=0;
    int r58r5_dykstra_max_sweeps_used=0;
    int r58r5_dykstra_projection_count=0;
    int r58r5_dykstra_optimal_kkt_count=0;
    double r58r5_dykstra_max_violation=std::numeric_limits<double>::infinity();
    double r58r5_dykstra_duality_gap=std::numeric_limits<double>::infinity();
    double r58r5_dykstra_stationarity_residual=std::numeric_limits<double>::infinity();
    double r58r5_dykstra_ms=0.0;
    // AF7R58R6 KKT-converged Hildreth dual coordinate-ascent diagnostics.
    int r58r6_hildreth_used=0;
    int r58r6_kkt_converged_count=0;
    int r58r6_resource_fail_count=0;
    int r58r6_max_sweeps_used=0;
    int r58r6_coordinate_updates=0;
    double r58r6_max_primal_violation=std::numeric_limits<double>::infinity();
    double r58r6_max_complementarity=std::numeric_limits<double>::infinity();
    double r58r6_duality_gap=std::numeric_limits<double>::infinity();
    double r58r6_stationarity_residual=std::numeric_limits<double>::infinity();
    double r58r6_max_lambda=0.0;
    double r58r6_solver_ms=0.0;
    double r58r6_prequant_scaled_jet_max_abs=0.0;
    double r58r6_quantized_scaled_jet_rel_error=0.0;
    double r58r6_quantized_physical_jet_max_abs_error=0.0;
    double r58r6_failed_corr_v_ub=0.0;
    double r58r6_failed_corr_a_ub=0.0;
    double r58r6_failed_corr_j_ub=0.0;
    double r58r6_failed_speed_worst_vq=-std::numeric_limits<double>::infinity();
    double r58r6_failed_speed_worst_vl=std::numeric_limits<double>::quiet_NaN();
    double r58r6_failed_speed_worst_sigma=std::numeric_limits<double>::quiet_NaN();
    double r58r6_failed_speed_worst_qdot=std::numeric_limits<double>::quiet_NaN();
    double r58r6_failed_speed_witness=std::numeric_limits<double>::quiet_NaN();
    // AF7R58R7 exact constraint-generation QP diagnostics.
    int r58r7_constraint_generation_used=0;
    int r58r7_full_row_count_max=0;
    int r58r7_outer_iterations_max=0;
    int r58r7_full_scans=0;
    int r58r7_rows_added=0;
    int r58r7_active_rows_max=0;
    int r58r7_subset_kkt_solves=0;
    int r58r7_subset_kkt_converged=0;
    int r58r7_subset_resource_fail=0;
    int r58r7_duplicate_add_fail=0;
    int r58r7_full_optimality_closed=0;
    int r58r7_subset_sweeps_max=0;
    int r58r7_coordinate_updates=0;
    double r58r7_initial_worst_violation=0.0;
    double r58r7_final_max_violation=std::numeric_limits<double>::infinity();
    double r58r7_final_duality_gap=std::numeric_limits<double>::infinity();
    double r58r7_final_complementarity=std::numeric_limits<double>::infinity();
    double r58r7_final_stationarity=std::numeric_limits<double>::infinity();
    double r58r7_solver_ms=0.0;
    // AF7R58R8 primal-dual Mehrotra subset-QP diagnostics.
    int r58r8_ipm_used=0;
    int r58r8_ipm_converged=0;
    int r58r8_ipm_resource_fail=0;
    int r58r8_ipm_factor_fail=0;
    int r58r8_ipm_max_iterations=0;
    int r58r8_ipm_newton_solves=0;
    double r58r8_ipm_primal_residual=std::numeric_limits<double>::infinity();
    double r58r8_ipm_dual_residual=std::numeric_limits<double>::infinity();
    double r58r8_ipm_complementarity=std::numeric_limits<double>::infinity();
    double r58r8_ipm_duality_gap=std::numeric_limits<double>::infinity();
    double r58r8_ipm_min_slack=std::numeric_limits<double>::quiet_NaN();
    double r58r8_ipm_min_lambda=std::numeric_limits<double>::quiet_NaN();
    double r58r8_ipm_ms=0.0;
    int r58r8_newton_diag_equilibrated=0;
    int r58r8_pcg_used=0;
    int r58r8_pcg_resource_fail=0;
    int r58r8_pcg_max_iterations=0;
    double r58r8_pcg_final_rel_residual=std::numeric_limits<double>::infinity();
    // AF7R58R9 solver root-cause diagnostics only; no theory/acceptance change.
    int r58r9_failure_count=0;
    int r58r9_failure_depth=0;
    int r58r9_failure_last_side=0;
    int r58r9_failure_outer_iteration=0;
    int r58r9_failure_active_rows=0;
    int r58r9_failure_added_row=-1;
    int r58r9_failure_ipm_iteration=0;
    int r58r9_failure_stage=0; // 1 predictor-PCG, 2 corrector-PCG, 3 IPM iter limit
    int r58r9_failure_pcg_iterations=0;
    double r58r9_failure_pcg_final_rel_residual=std::numeric_limits<double>::infinity();
    double r58r9_failure_pcg_rhs_inf=std::numeric_limits<double>::quiet_NaN();
    double r58r9_failure_D_min=std::numeric_limits<double>::quiet_NaN();
    double r58r9_failure_D_max=std::numeric_limits<double>::quiet_NaN();
    double r58r9_failure_D_ratio=std::numeric_limits<double>::quiet_NaN();
    double r58r9_failure_min_slack=std::numeric_limits<double>::quiet_NaN();
    double r58r9_failure_min_lambda=std::numeric_limits<double>::quiet_NaN();
    double r58r9_failure_metric_max_abs_corr=0.0;
    double r58r9_failure_metric_one_minus_corr=1.0;
    double r58r9_full_scan_ms=0.0;
    double r58r9_subset_solver_ms=0.0;
    double r58r9_pcg_ms=0.0;

    int r58r9_plus_failed=0;
    int r58r9_minus_failed=0;
    int r58r9_plus_failure_stage=0;
    int r58r9_minus_failure_stage=0;
    int r58r9_plus_failure_active_rows=0;
    int r58r9_minus_failure_active_rows=0;
    double r58r9_plus_failure_pcg_relres=std::numeric_limits<double>::quiet_NaN();
    double r58r9_minus_failure_pcg_relres=std::numeric_limits<double>::quiet_NaN();
    double r58r9_plus_metric_max_abs_corr=std::numeric_limits<double>::quiet_NaN();
    double r58r9_minus_metric_max_abs_corr=std::numeric_limits<double>::quiet_NaN();
    double r58r9_plus_cg_ms=0.0;
    double r58r9_minus_cg_ms=0.0;
    // AF7R58R10 branch-specific central-path diagnostics only.
    double r58r10_plus_primal_residual=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_dual_residual=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_mu=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_duality_gap=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_kkt_tol=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_gap_tol=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_last_sigma=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_last_alpha_primal=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_last_alpha_dual=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_min_alpha_primal=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_min_alpha_dual=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_mu_ratio=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_D_min=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_D_max=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_D_ratio=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_min_slack=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_min_lambda=std::numeric_limits<double>::quiet_NaN();
    int r58r10_plus_predictor_pcg_iterations=0;
    int r58r10_plus_corrector_pcg_iterations=0;
    double r58r10_plus_predictor_pcg_relres=std::numeric_limits<double>::quiet_NaN();
    double r58r10_plus_corrector_pcg_relres=std::numeric_limits<double>::quiet_NaN();

    double r58r10_minus_primal_residual=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_dual_residual=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_mu=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_duality_gap=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_kkt_tol=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_gap_tol=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_last_sigma=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_last_alpha_primal=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_last_alpha_dual=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_min_alpha_primal=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_min_alpha_dual=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_mu_ratio=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_D_min=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_D_max=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_D_ratio=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_min_slack=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_min_lambda=std::numeric_limits<double>::quiet_NaN();
    int r58r10_minus_predictor_pcg_iterations=0;
    int r58r10_minus_corrector_pcg_iterations=0;
    double r58r10_minus_predictor_pcg_relres=std::numeric_limits<double>::quiet_NaN();
    double r58r10_minus_corrector_pcg_relres=std::numeric_limits<double>::quiet_NaN();
    // AF7R58R11 augmented-KKT complete-pivot Newton + common-step diagnostics.
    int r58r11_augmented_kkt_used=0;
    int r58r11_augmented_kkt_solve_fail=0;
    int r58r11_common_step_used=0;
    int r58r11_augmented_kkt_max_dim=0;
    int r58r11_newton_solves=0;
    double r58r11_newton_max_rel_residual=0.0;
    double r58r11_common_step_min=1.0;
    double r58r11_common_step_last=std::numeric_limits<double>::quiet_NaN();
    double r58r11_solver_ms=0.0;
    double r48_natural_rejoin_seed_t1=std::numeric_limits<double>::quiet_NaN();

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

// AF7R47: synthesis guide for a general active C7 base reference.
// This vector has NO safety authority. It only defines the 2-D correction
// plane used by the compact R37 spatial QP. Safety remains in the unchanged
// speed/rejoin/whole-body certificates.
//
// Prefer the actual active-reference velocity near the unsafe component.
// If a local stationary point occurs, fall back to the spatial chord. No
// maneuver-duration or obstacle-size tuning constant is introduced.
static inline bool r47_active_progress_guide(
        const ActiveReference&ref,double a,double b,Vec3&g){
    if(!(fin(a)&&fin(b)&&b>a))return false;
    const double T[3]={0.5*(a+b),a,b};
    for(double t:T){
        auto x=ref.eval(t);
        const double nv=norm(x[1]);
        if(finite(x[1])&&fin(nv)&&nv>0.0){
            g=x[1];
            return true;
        }
    }
    auto xa=ref.eval(a),xb=ref.eval(b);
    Vec3 d=xb[0]-xa[0];
    const double nd=norm(d);
    if(finite(d)&&fin(nd)&&nd>0.0){
        g=d;
        return true;
    }
    return false;
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
// AF7R48R1 exact no-catch-up progress gate for the delay-preserving affine-base family.
// For e=vbase/||vbase||, every Bernstein control point of the total
// longitudinal velocity must lie in [0, ||vbase||]. Because a Bezier curve
// stays in the convex hull of its control values, this proves continuously
//
//      0 <= ds/dt <= 1
//
// relative to the original reference clock. Thus the avoidance trajectory
// may fall behind but can never recover delay by running ahead of the source
// reference. Binary64 guards are representability guards only.
static inline bool r48_no_catchup_progress_exact(Vec3 vbase,const Candidate&C){
    const double nv=norm(vbase);
    if(!(fin(nv)&&nv>0))return false;
    const Vec3 e=vbase/nv;
    const double guard=256*epsd()*std::max(1.0,nv);
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        if(!(fin(T)&&T>0))return false;
        Poly D=deriv_poly(C.cp[s],T,1);
        for(const auto&dc:D){
            const double vp=dot(e,vbase+dc);
            if(!(fin(vp)&&vp>=-guard&&vp<=nv+guard))return false;
        }
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
static inline SpeedCert r51_phase_base_certificate(
        const SpeedCert&B,const Live&L,const R51PhaseSpec&P){
    if(!P.enabled)return B;
    SpeedCert S;
    double smin=0,smax=0,S2=0,S3=0;
    if(!r51_phase_derivative_bounds(P,smin,smax,S2,S3))return S;
    smin=std::max(0.0,smin);
    smax=std::min(1.0,std::max(0.0,smax));

    S.VL=B.VL*smax;
    S.A=B.A*smax*smax+B.VL*S2;
    S.J=B.J*smax*smax*smax+3.0*B.A*smax*S2+B.VL*S3;

    // Compare phased specific force against the certified old-reference
    // specific force.  ||a_s-a|| <= A(1-sdot^2)+V|sddot|.
    const double dA=B.A*std::max(0.0,1.0-smin*smin)+B.VL*S2;
    S.sigma=B.sigma-dA;
    if(!(fin(S.sigma)&&S.sigma>0.0&&fin(S.J)&&S.J>=0.0))return SpeedCert{};
    S.qdot=S.J/S.sigma;
    S.VQ=S.VL+L.L*S.qdot;
    S.certified=fin(S.VQ)&&fin(S.VL)&&fin(S.A)&&fin(S.J)&&
                fin(S.qdot)&&S.sigma>0.0;
    S.stageA=false;
    return S;
}


static inline EpisodeCert r51_warp_episode(
        const EpisodeCert&E,const R51PhaseSpec&P){
    EpisodeCert W=E;
    if(!P.enabled)return W;
    for(auto&e:W.ep){
        e.start=r51_phase_inverse_flow(P,e.start);
        e.end=r51_phase_inverse_flow(P,e.end);
    }
    for(auto&c:W.comp){
        c.start=r51_phase_inverse_flow(P,c.start);
        c.end=r51_phase_inverse_flow(P,c.end);
    }
    return W;
}

// AF7R52: rebuild unsafe episodes on the ACTUAL phase-composed reference.
// No time-warp approximation of a previously built episode is used.
static inline EpisodeCert r52_build_phase_episode(
        const ActiveReference&ref,const R51PhaseSpec&phase,
        const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,double t0,double searchEnd){
    EpisodeCert E;
    if(!(fin(t0)&&fin(searchEnd)&&searchEnd>t0&&L.dt>0&&!obs.empty()))return E;
    const double tolMerge=64*epsd()*std::max({1.0,std::fabs(t0),std::fabs(searchEnd),std::fabs(L.dt)});
    std::vector<Episode> raw;
    for(size_t oi=0;oi<obs.size();++oi){
        std::vector<std::pair<double,double>> stack{{t0,searchEnd}},unsafe;
        while(!stack.empty()){
            auto [ta,tb]=stack.back();stack.pop_back();
            if(!(fin(ta)&&fin(tb)&&tb>ta))continue;
            const double tc=0.5*(ta+tb),h=0.5*(tb-ta);
            const bool splittable=fin(tc)&&tc>ta&&tc<tb;
            ++E.leaves;
            bool safe=false;
            if(splittable){
                auto x=r51_phase_reference_eval(ref,phase,tc);
                Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};
                const double ns=norm(sf);
                if(fin(ns)&&ns>0){
                    Vec3 q=sf*(-1.0/ns),pQ=pL-q*L.L;
                    const double VC=std::max(base.VL,base.VQ);
                    double mL,mQ,mC;
                    const bool a=point_margin(pL,obs[oi],L.rL+L.dmargin+L.tracking+base.VL*h,mL);
                    const bool b=point_margin(pQ,obs[oi],L.rQ+L.dmargin+L.tracking+base.VQ*h,mQ);
                    const bool c=segment_margin(pQ,pL,obs[oi],L.rC+L.dmargin+L.tracking+VC*h,mC);
                    safe=a&&b&&c;
                }
            }
            if(safe){++E.safe_leaves;continue;}
            if((tb-ta)<=L.dt+tolMerge || !splittable){
                unsafe.push_back({ta,tb});++E.unsafe_leaves;
                if(!splittable)++E.repr_fail;
                continue;
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
    const double tolT=64*epsd()*std::max({1.0,std::fabs(t0),std::fabs(searchEnd)});
    for(size_t i=0;i<raw.size();++i){
        if(E.comp.empty()||raw[i].start>E.comp.back().end+tolT){
            Component c;c.start=raw[i].start;c.end=raw[i].end;c.episodes.push_back((int)i);E.comp.push_back(c);
        }else{
            E.comp.back().end=std::max(E.comp.back().end,raw[i].end);
            E.comp.back().episodes.push_back((int)i);
        }
        raw[i].comp=(int)E.comp.size()-1;
    }
    E.ep=std::move(raw);E.ok=true;return E;
}

static inline bool r51_build_phase_seed(
        const Live&L,const EpisodeCert&E,double t0,double searchEnd,
        R51PhaseSpec&P){
    P=R51PhaseSpec{};
    if(E.comp.empty()||!(fin(t0)&&fin(searchEnd)&&searchEnd>t0))
        return false;

    const double h=r33_time_quantum(L.dt);
    const double ts=r33_align_up(t0,h);
    const double sA=E.comp.front().start;
    const double sB=E.comp.back().end;
    const double Ds=sB-sA;
    const double tau=std::sqrt(L.L/L.g);
    if(!(fin(h)&&h>0.0&&fin(ts)&&ts<sA&&fin(Ds)&&Ds>0.0&&
         fin(tau)&&tau>0.0))
        return false;

    // Stretch the source-phase traversal by one suspended-load natural time.
    // This is a physics-derived quality seed, not a maneuver-duration cap.
    long long Kflow=(long long)std::ceil((Ds+tau)/h);
    Kflow=std::max(1LL,Kflow);
    const double Tf=(double)Kflow*h;
    const double alpha=Ds/Tf;
    if(!(fin(alpha)&&alpha>0.0&&alpha<=1.0))return false;
    const double beta=1.0-alpha;
    const double De=sA-ts;

    // Exact no-catch-up entry interval inherited from the R49 Bernstein proof.
    const double denLo=1.0-7.0*beta/15.0;
    const double denHi=14.0-7.0*beta;
    if(!(denLo>0.0&&denHi>0.0))return false;
    const double TeLo=De/denLo;
    const double TeHi=15.0*De/denHi;
    if(!(fin(TeLo)&&fin(TeHi)&&TeHi>=TeLo&&TeLo>0.0))return false;

    const double ratioMid=(1.0+14.0*beta)/30.0;
    const double TeMid=De/(1.0-ratioMid);
    long long Ke=(long long)std::llround(TeMid/h);
    long long KeMin=(long long)std::ceil(
        TeLo/h-64*epsd()*std::max(1.0,std::fabs(TeLo/h)));
    long long KeMax=(long long)std::floor(
        TeHi/h+64*epsd()*std::max(1.0,std::fabs(TeHi/h)));
    KeMin=std::max(1LL,KeMin);
    if(Ke<KeMin)Ke=KeMin;
    if(Ke>KeMax)Ke=KeMax;
    if(Ke<1||KeMax<KeMin)return false;

    const double Te=(double)Ke*h;
    long long Ktau=(long long)std::ceil(
        tau/h-64*epsd()*std::max(1.0,std::fabs(tau/h)));
    Ktau=std::max(1LL,Ktau);
    long long Kx=std::max(Ke,Ktau);
    double Tx=(double)Kx*h;

    double tA=ts+Te;
    double tB=tA+Tf;
    double tE=tB+Tx;

    if(!(tE<searchEnd-64*epsd()*std::max(1.0,std::fabs(searchEnd))))
        return false;

    const double sE=sB+0.5*(alpha+1.0)*Tx;
    const double delay=tE-sE;
    if(!(fin(delay)&&delay>0.0))return false;

    P.enabled=true;
    P.t0=ts;P.tA=tA;P.tB=tB;P.tE=tE;
    P.sA=sA;P.sB=sB;P.alpha=alpha;P.delay=delay;
    if(!r51_phase_valid(P))return false;

    double smin=0,smax=0,S2=0,S3=0;
    if(!r51_phase_derivative_bounds(P,smin,smax,S2,S3))
        return false;
    const double tol=4096*epsd()*std::max(1.0,std::max(std::fabs(smin),std::fabs(smax)));
    return smin>0.0 && smax<=1.0+tol;
}


// ===========================================================================
// AF7R54 HORIZON-DECOUPLED ADAPTIVE PHASE CONTINUATION
//
// H is NOT a maneuver duration.  The trigger horizon and the finite-sensing
// theorem retain L.horizon elsewhere; this construction intentionally does not
// read L.horizon at all.
//
// A phase candidate is indexed by an integer natural-time stretch multiple m:
//
//     Tf >= Ds + m*sqrt(L/g),      m = 1,2,4,...
//
// on the exact dyadic time grid.  For each Tf, the R49 Bernstein theorem is
// used unchanged to choose an exact-C7 entry duration satisfying continuously
//
//     0 < ds/dt <= 1.
//
// The continuation has no arbitrary physical duration cap.  The only terminal
// bound is the finite active-reference domain searchEnd.  Stronger slowdown
// candidates are generated only while that domain can contain the complete
// phase transition.  Every phase candidate rebuilds its own continuous unsafe
// episode before any spatial corridor is synthesized.
// ===========================================================================
struct R54PhaseTrial{
    R51PhaseSpec phase;
    long long tau_multiple=0;
};

static inline bool r54_build_phase_from_tau_multiple(
        const Live&L,const EpisodeCert&E,double t0,double searchEnd,
        long long tauMultiple,R51PhaseSpec&P){
    P=R51PhaseSpec{};
    if(tauMultiple<1||E.comp.empty()||
       !(fin(t0)&&fin(searchEnd)&&searchEnd>t0))return false;

    const double h=r33_time_quantum(L.dt);
    const double ts=r33_align_up(t0,h);
    const double sA=E.comp.front().start;
    const double sB=E.comp.back().end;
    const double Ds=sB-sA;
    const double tau=std::sqrt(L.L/L.g);
    if(!(fin(h)&&h>0.0&&fin(ts)&&ts<sA&&fin(Ds)&&Ds>0.0&&
         fin(tau)&&tau>0.0))return false;

    const double targetFlow=Ds+(double)tauMultiple*tau;
    if(!(fin(targetFlow)&&targetFlow>Ds))return false;
    long long Kflow=(long long)std::ceil(
        targetFlow/h-64*epsd()*std::max(1.0,std::fabs(targetFlow/h)));
    Kflow=std::max(1LL,Kflow);
    const double Tf=(double)Kflow*h;
    if(!(fin(Tf)&&Tf>=Ds))return false;

    const double alpha=Ds/Tf;
    if(!(fin(alpha)&&alpha>0.0&&alpha<1.0))return false;
    const double beta=1.0-alpha;
    const double De=sA-ts;

    // Exact R49 no-catch-up entry interval.
    const double denLo=1.0-7.0*beta/15.0;
    const double denHi=14.0-7.0*beta;
    if(!(denLo>0.0&&denHi>0.0))return false;
    const double TeLo=De/denLo;
    const double TeHi=15.0*De/denHi;
    if(!(fin(TeLo)&&fin(TeHi)&&TeHi>=TeLo&&TeLo>0.0))return false;

    // Midpoint of the exact Bernstein-feasible interval maximizes distance
    // from both longitudinal velocity faces ds/dt=0 and ds/dt=1.
    const double ratioMid=(1.0+14.0*beta)/30.0;
    const double TeMid=De/(1.0-ratioMid);
    long long Ke=(long long)std::llround(TeMid/h);
    long long KeMin=(long long)std::ceil(
        TeLo/h-64*epsd()*std::max(1.0,std::fabs(TeLo/h)));
    long long KeMax=(long long)std::floor(
        TeHi/h+64*epsd()*std::max(1.0,std::fabs(TeHi/h)));
    KeMin=std::max(1LL,KeMin);
    if(Ke<KeMin)Ke=KeMin;
    if(Ke>KeMax)Ke=KeMax;
    if(Ke<1||KeMax<KeMin)return false;

    const double Te=(double)Ke*h;
    long long Ktau=(long long)std::ceil(
        tau/h-64*epsd()*std::max(1.0,std::fabs(tau/h)));
    Ktau=std::max(1LL,Ktau);
    const long long Kx=std::max(Ke,Ktau);
    const double Tx=(double)Kx*h;

    const double tA=ts+Te;
    const double tB=tA+Tf;
    const double tE=tB+Tx;
    if(!(fin(tE)&&tE<searchEnd-64*epsd()*std::max(1.0,std::fabs(searchEnd))))
        return false;

    const double sE=sB+0.5*(alpha+1.0)*Tx;
    const double delay=tE-sE;
    if(!(fin(delay)&&delay>0.0))return false;

    P.enabled=true;
    P.t0=ts;P.tA=tA;P.tB=tB;P.tE=tE;
    P.sA=sA;P.sB=sB;P.alpha=alpha;P.delay=delay;
    if(!r51_phase_valid(P))return false;

    double smin=0.0,smax=0.0,S2=0.0,S3=0.0;
    if(!r51_phase_derivative_bounds(P,smin,smax,S2,S3))return false;
    const double tol=4096*epsd()*std::max(1.0,
        std::max(std::fabs(smin),std::fabs(smax)));
    return smin>0.0&&smax<=1.0+tol;
}

static inline std::vector<R54PhaseTrial> r54_phase_candidates(
        const Live&L,const EpisodeCert&E,double t0,double searchEnd){
    std::vector<R54PhaseTrial> out;
    if(E.comp.empty())return out;

    const double h=r33_time_quantum(L.dt);
    const double ts=r33_align_up(t0,h);
    const double Ds=E.comp.back().end-E.comp.front().start;
    const double tau=std::sqrt(L.L/L.g);
    if(!(fin(h)&&h>0.0&&fin(ts)&&fin(Ds)&&Ds>0.0&&fin(tau)&&tau>0.0&&
         fin(searchEnd)&&searchEnd>ts))return out;

    // Source-domain-derived upper bound only.  It is deliberately loose;
    // r54_build_phase_from_tau_multiple applies the exact tE<searchEnd gate.
    const double room=searchEnd-ts-Ds;
    if(!(fin(room)&&room>tau))return out;
    long long mDomain=(long long)std::floor(room/tau);
    mDomain=std::max(1LL,mDomain);

    auto add=[&](long long m){
        if(m<1)return;
        R51PhaseSpec P;
        if(!r54_build_phase_from_tau_multiple(L,E,t0,searchEnd,m,P))return;
        for(const auto&q:out)
            if(q.tau_multiple==m)return;
        out.push_back({P,m});
    };

    // Least slowdown first, then dyadic continuation.  These are search
    // coordinates, not physical caps or tuning weights.
    long long m=1;
    while(m<=mDomain){
        add(m);
        if(m>mDomain/2)break;
        m*=2;
    }
    // Also test the finite-domain edge if it is representable; this prevents
    // the dyadic continuation from leaving a large unexamined tail.
    add(mDomain);

    std::sort(out.begin(),out.end(),[](const R54PhaseTrial&a,const R54PhaseTrial&b){
        return a.tau_multiple<b.tau_multiple;
    });
    return out;
}


// ===========================================================================
// AF7R55 BALANCED SOURCE-PHASE / REAL-TIME ALLOCATION
//
// The source path coordinate and wall-clock time are independent variables.
// No equality p_new(t_r)=p_old(t_r) is imposed, and L.horizon is not read.
//
// For the current active reference let [s0,s1] be the complete unsafe
// source-phase envelope.  Within the finite real-time support [t0,tE], choose
// the constant-flow phase rate alpha so that the unsafe source midpoint maps
// to the physical support midpoint.  For affine flow this exactly maximizes
//
//      min( t(s0)-t0, tE-t(s1) )
//
// without a scalar tuning weight.  Large/small obstacles change [s0,s1]
// directly, so the timing changes from geometry rather than a size class.
//
// The C7 entry/exit phase transitions are symmetric degree-15 Hermite delay
// transitions.  Their derivative bounds are certified afterwards by the
// existing Bernstein authority r51_phase_derivative_bounds, proving
//
//      0 < ds/dt <= 1
//
// continuously.
// ===========================================================================
struct R55PhaseDiag{
    double s0=0,s1=0,tFirst=0,tLast=0,preSlack=0,postSlack=0,minSlack=0;
    double Te=0,Tx=0;
};

static inline double r55_align_down(double x,double h){
    if(!(fin(x)&&fin(h)&&h>0.0))
        return std::numeric_limits<double>::quiet_NaN();
    const double q=x/h;
    if(!(fin(q)&&std::fabs(q)<4503599627370496.0))
        return std::numeric_limits<double>::quiet_NaN();
    double k=std::floor(q),y=k*h;
    if(y>x)y=(k-1.0)*h;
    return y;
}

static inline bool r55_build_balanced_phase(
        const Live&L,const EpisodeCert&E,double t0,double searchEnd,
        R51PhaseSpec&P,R55PhaseDiag&D){
    P=R51PhaseSpec{};D=R55PhaseDiag{};
    if(E.comp.empty()||!(fin(t0)&&fin(searchEnd)&&searchEnd>t0))
        return false;

    const double h=r33_time_quantum(L.dt);
    const double ts=r33_align_up(t0,h);
    const double tE=r55_align_down(searchEnd,h);
    const double s0=E.comp.front().start;
    const double s1=E.comp.back().end;
    const double tau=std::sqrt(L.L/L.g);
    if(!(fin(h)&&h>0.0&&fin(ts)&&fin(tE)&&tE>ts&&
         fin(s0)&&fin(s1)&&s1>s0&&fin(tau)&&tau>0.0&&ts<s0))
        return false;

    long long Ktau=(long long)std::ceil(
        tau/h-64*epsd()*std::max(1.0,std::fabs(tau/h)));
    Ktau=std::max(1LL,Ktau);
    const double Ttau=(double)Ktau*h;

    // Complete the 1 -> alpha transition before the first unsafe source phase.
    const double preRoom=s0-ts;
    const double TeMax=r55_align_down(preRoom-h,h);
    if(!(fin(TeMax)&&TeMax>=h))return false;
    const double Te=std::min(Ttau,TeMax);
    const double tA=ts+Te;
    if(!(tA<s0&&tA<tE))return false;

    const double sMid=0.5*(s0+s1);
    const double tMid=0.5*(ts+tE);

    // With symmetric entry delay dA=(1-alpha)Te/2:
    // sA=tA-dA=tA-Te/2+alpha*Te/2.
    // Enforce s(tMid)=sMid and solve alpha analytically.
    const double den=(tMid-tA)+0.5*Te;
    const double num=sMid-(tA-0.5*Te);
    if(!(fin(den)&&den>0.0&&fin(num)&&num>0.0))return false;
    const double alpha=num/den;
    if(!(fin(alpha)&&alpha>0.0&&alpha<=1.0))return false;

    const double beta=1.0-alpha;
    const double dA=0.5*beta*Te;
    const double sA=tA-dA;
    if(!(fin(sA)&&sA<s0))return false;

    const double tFirst=tA+(s0-sA)/alpha;
    const double tLast=tA+(s1-sA)/alpha;
    if(!(fin(tFirst)&&fin(tLast)&&tA<tFirst&&tFirst<tLast&&tLast<tE))
        return false;

    // Reserve a C7 phase exit after the last unsafe source phase.
    const double postRoom=tE-tLast;
    const double TxMax=r55_align_down(postRoom-h,h);
    if(!(fin(TxMax)&&TxMax>=h))return false;
    const double Tx=std::min(Ttau,TxMax);
    const double tB=tE-Tx;
    if(!(tLast<tB&&tA<tB))return false;

    const double sB=sA+alpha*(tB-tA);
    const double sEnd=sB+0.5*(alpha+1.0)*Tx;
    const double delay=tE-sEnd;
    if(!(fin(sB)&&sB>s1&&fin(delay)&&delay>=0.0))return false;

    P.enabled=true;
    P.t0=ts;P.tA=tA;P.tB=tB;P.tE=tE;
    P.sA=sA;P.sB=sB;P.alpha=alpha;P.delay=delay;
    if(!r51_phase_valid(P))return false;

    double smin=0.0,smax=0.0,S2=0.0,S3=0.0;
    if(!r51_phase_derivative_bounds(P,smin,smax,S2,S3))return false;
    const double tol=4096*epsd()*std::max(1.0,
        std::max(std::fabs(smin),std::fabs(smax)));
    if(!(smin>0.0&&smax<=1.0+tol))return false;

    D.s0=s0;D.s1=s1;D.tFirst=tFirst;D.tLast=tLast;
    D.preSlack=tFirst-ts;D.postSlack=tE-tLast;
    D.minSlack=std::min(D.preSlack,D.postSlack);
    D.Te=Te;D.Tx=Tx;
    return fin(D.minSlack)&&D.minSlack>0.0;
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
// AF7R37 HISTORICAL SAME-CLOCK FREE-REJOIN EXACT-C7 SPATIAL SPLINE (R48R1 production-unreachable)
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


static inline std::vector<Row> r51_phase_corridor_rows(
        const ActiveReference&ref,const R51PhaseSpec&phase,
        const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,const EpisodeCert&E,int ei,Vec3 n,
        const std::vector<double>&K,const R36Plane&B,Vec3 progressGuide,
        R45CorridorDiag*diag=nullptr){
    const int ns=r37_scalar_dofs(K),nvar=2*ns;
    const auto&ep=E.ep[ei];
    const auto&o=obs[ep.obs];

    Vec3 eprog=normalized(progressGuide);
    Vec3 elat=n-eprog*dot(n,eprog);
    const double nl=norm(elat);
    if(!(fin(nl)&&nl>0))
        throw std::runtime_error("R51 degenerate local lateral direction");
    elat=elat/nl;

    const double VC=std::max(base.VL,base.VQ);
    std::vector<std::pair<double,double>>sb;
    double ta=ep.start,te=ep.end;
    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        const double tb=std::min(te,ta+L.dt);
        const double tc=0.5*(ta+tb),half=0.5*(tb-ta);
        auto x=r51_phase_reference_eval(ref,phase,tc);
        Vec3 pL=x[0],sf=x[2]+Vec3{0,0,L.g};
        const double nf=norm(sf);
        if(!(fin(nf)&&nf>0))
            throw std::runtime_error("R51 phased-base flatness singular");
        Vec3 q=sf*(-1.0/nf),pQ=pL-q*L.L;

        const double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        const double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        const double rhoC=L.rC+L.dmargin+L.tracking+VC*half;

        R45ConvexEval F0=r45_corridor_eval_u(
            pL,pQ,o,rhoL,rhoQ,rhoC,eprog,elat,0.0);
        if(!F0.ok)
            throw std::runtime_error("R51 support evaluation failed");

        bool fb=false;
        double req=r45_min_lateral_requirement(
            pL,pQ,o,rhoL,rhoQ,rhoC,eprog,elat,fb);
        if(!fin(req))
            throw std::runtime_error("R51 tangent support minimization failed");

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

// AF7R48 monotone exit-to-nominal sufficient conditions.
//
// For the final C7 exit segment, the correction is represented by a degree-15
// Bezier polynomial from the last unsafe-component node to the exact zero jet
// endpoint.  For every selected separating direction belonging to the last
// unsafe component we constrain the scalar Bezier control polygon to be
// nonnegative and nonincreasing.  By the Bezier convex-hull and derivative
// properties this proves, continuously over the complete exit segment,
//
//   s(t) >= 0,   ds/dt <= 0,   s(t1)=0.
//
// Hence the correction cannot cross the nominal path and come back in any of
// the selected certified bypass directions.  These are synthesis-quality
// constraints only; the unchanged whole-body certificate remains safety
// authority.
static inline int r48_add_exit_monotone_rows(
        std::vector<Row>&rows,const std::vector<double>&K,
        const R36Plane&B,const std::vector<Vec3>&signedDirs){
    const int N=(int)K.size(),ns=r37_scalar_dofs(K);
    if(N<3||ns<=0||signedDirs.empty())return 0;
    const int node=N-2;
    const int qp=r37_scalar_index(node,0,N);
    const int qv=r37_scalar_index(node,1,N);
    if(qp<0||qv<0)return 0;
    const double T=K.back()-K[N-2];
    if(!(fin(T)&&T>0))return 0;

    std::array<double,8>J0P{},J0V{},J1{};
    J0P[0]=1.0;
    J0V[1]=1.0;
    const auto cP=scalar_control_from_jets(J0P,J1,T);
    const auto cV=scalar_control_from_jets(J0V,J1,T);
    const int nvar=2*ns;
    int added=0;

    auto push=[&](double ap,double av,double nb1,double nb2){
        Row R;R.a.assign(nvar,0.0);R.b=0.0;
        R.a[2*qp+0]=ap*nb1;
        R.a[2*qp+1]=ap*nb2;
        R.a[2*qv+0]=av*nb1;
        R.a[2*qv+1]=av*nb2;
        double ma=0.0;for(double x:R.a)ma=std::max(ma,std::fabs(x));
        if(ma>0.0){rows.push_back(std::move(R));++added;}
    };

    for(Vec3 d:signedDirs){
        double nb1=dot(d,B.b1),nb2=dot(d,B.b2);
        const double nn=std::hypot(nb1,nb2);
        if(!(fin(nn)&&nn>0))continue;
        nb1/=nn;nb2/=nn;

        // Scalar correction remains on the selected bypass side.
        for(int k=0;k<15;++k)
            push(cP[k],cV[k],nb1,nb2);

        // Scalar control polygon is nonincreasing toward exact zero.
        for(int k=0;k<15;++k)
            push(cP[k]-cP[k+1],cV[k]-cV[k+1],nb1,nb2);
    }
    return added;
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
// AF7R52: isotropic world-coordinate version of the R37 towing Hessian.
static inline std::vector<double> r52_spatial_hessian3(
        const std::vector<double>&K,double cableL,double gravity){
    const int ns=r37_scalar_dofs(K);
    if(ns<=0)return {};
    const auto H2=r37_spatial_hessian(K,cableL,gravity);
    const int n2=2*ns,n3=3*ns;
    if((int)H2.size()!=n2*n2)return {};
    std::vector<double>H3(n3*n3,0.0);
    for(int i=0;i<ns;++i)for(int j=0;j<ns;++j){
        const double hij=H2[(2*i)*n2+(2*j)];
        for(int a=0;a<3;++a)H3[(3*i+a)*n3+(3*j+a)]=hij;
    }
    return H3;
}

static inline R37Path r52_decode_path3(
        const QPSol&q,const std::vector<double>&K,const std::vector<int>&signs){
    R37Path P;
    const int N=(int)K.size(),ns=r37_scalar_dofs(K);
    if(!q.ok||N<3||(int)q.z.size()!=3*ns)return P;
    P.knots=K;P.P.assign(N,Vec3{});P.V.assign(N,Vec3{});
    for(int node=1;node<N-1;++node){
        const int qp=r37_scalar_index(node,0,N);
        const int qv=r37_scalar_index(node,1,N);
        P.P[node]={q.z[3*qp+0],q.z[3*qp+1],q.z[3*qp+2]};
        P.V[node]={q.z[3*qv+0],q.z[3*qv+1],q.z[3*qv+2]};
    }
    P.cost=q.cost;P.slack=q.slack;P.signs=signs;P.ok=true;
    return P;
}


// AF7R53D3 diagnostic-only shape recorder.
static inline void r53_record_shape_diag(
        const R37Path&P,const Candidate&C,int side){
    const double id=++G_r53_build_seq;
    const int N=(int)P.knots.size();

    for(int j=0;j<N;++j){
        R53KnotDiagRow R;
        R.build_id=id;R.side=(double)side;R.rejoin=C.actual_rejoin_time;
        R.corr_v=C.dub[0];R.corr_a=C.dub[1];R.corr_j=C.dub[2];
        R.node=(double)j;R.knot=P.knots[j];
        R.P=P.P[j];R.V=P.V[j];
        R.Pnorm=norm(R.P);R.Vnorm=norm(R.V);
        R.prevT=(j>0)?P.knots[j]-P.knots[j-1]:std::numeric_limits<double>::quiet_NaN();
        R.nextT=(j+1<N)?P.knots[j+1]-P.knots[j]:std::numeric_limits<double>::quiet_NaN();
        G_r53_knot_diag.push_back(R);
    }

    for(size_t s=0;s<C.cp.size();++s){
        R53SegmentDiagRow R;
        R.build_id=id;R.side=(double)side;R.rejoin=C.actual_rejoin_time;
        R.corr_v=C.dub[0];R.corr_a=C.dub[1];R.corr_j=C.dub[2];
        R.seg=(double)s;R.ta=C.seg0[s];R.tb=C.seg1[s];R.T=R.tb-R.ta;
        if(!(fin(R.T)&&R.T>0.0))continue;

        Poly D=C.cp[s];int deg=15;
        for(const auto&q:D)R.cp_norm=std::max(R.cp_norm,norm(q));
        for(int r=1;r<=3;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/R.T);
            D.swap(E);--deg;
            const double ub=poly_norm_ub(D);
            if(r==1)R.vel_ub=ub;
            else if(r==2)R.acc_ub=ub;
            else R.jerk_ub=ub;
        }
        G_r53_segment_diag.push_back(R);
    }
}

struct R52SupportPlane{
    bool ok=false;
    Vec3 m{};
    double req=std::numeric_limits<double>::infinity();
    double u=0.0;
};

// Same convex exact-support minimization as R45, but returns the actual
// separating plane. No fixed-normal fallback is permitted: numerical
// inconclusiveness fails this candidate closed.
static inline R52SupportPlane r52_best_support_plane(
        const Vec3&pL,const Vec3&pQ,const Obstacle&o,
        double rhoL,double rhoQ,double rhoC,Vec3 eprog,Vec3 elat){
    R52SupportPlane O;
    auto F=[&](double u){return r45_corridor_eval_u(
        pL,pQ,o,rhoL,rhoQ,rhoC,eprog,elat,u);};
    auto E0=F(0.0);
    if(!E0.ok)return O;
    double best=E0.value,bestu=0.0;
    if(E0.value<=0.0){O.ok=true;O.m=elat;O.req=0.0;return O;}
    if(E0.dmin<=0.0&&E0.dmax>=0.0){
        O.ok=true;O.m=elat;O.req=std::max(0.0,E0.value);return O;
    }
    const int dir=(E0.dmax<0.0)?+1:-1;
    double uprev=0.0,step=1.0,ulo=0.0,uhi=0.0;
    R45ConvexEval Eprev=E0;
    bool bracket=false;
    for(int it=0;it<60;++it){
        const double ucur=double(dir)*step;
        auto Ec=F(ucur);
        if(!Ec.ok)return O;
        if(Ec.value<best){best=Ec.value;bestu=ucur;}
        if(Ec.value<=0.0){best=0.0;bestu=ucur;break;}
        if((dir>0&&Ec.dmax>=0.0)||(dir<0&&Ec.dmin<=0.0)){
            if(dir>0){ulo=uprev;uhi=ucur;}else{ulo=ucur;uhi=uprev;}
            bracket=true;break;
        }
        uprev=ucur;Eprev=Ec;step*=2.0;
        if(!fin(step))return O;
    }
    if(best>0.0){
        if(!bracket)return O;
        for(int it=0;it<80;++it){
            const double um=0.5*(ulo+uhi);
            if(!(fin(um)&&um>ulo&&um<uhi))break;
            auto Em=F(um);if(!Em.ok)return O;
            if(Em.value<best){best=Em.value;bestu=um;}
            if(Em.dmin<=0.0&&Em.dmax>=0.0)break;
            if(Em.dmax<0.0)ulo=um;else if(Em.dmin>0.0)uhi=um;else break;
            const double tol=256*epsd()*std::max({1.0,std::fabs(ulo),std::fabs(uhi)});
            if((uhi-ulo)<=tol)break;
        }
    }
    Vec3 m=elat+eprog*bestu;
    if(!(finite(m)&&fin(norm(m))&&norm(m)>0.0&&fin(best)))return O;
    O.ok=true;O.m=m;O.req=std::max(0.0,best);O.u=bestu;return O;
}


// AF7R53 GENERAL-C7 MOVING NORMAL-BUNDLE CONSTRAINT.
//
// Phase is the sole longitudinal/progress variable.  The world-3D avoidance
// correction is constrained at every interior exact-C7 knot to the moving
// normal bundle of the phase-composed active reference:
//
//   e^T P = 0,
//   e^T V + e_dot^T P = 0,
//   e = v_b/||v_b||,
//   e_dot = (I-ee^T)a_b/||v_b||.
//
// No straight/fixed-plane/saddle/circle special case is used.
// For an affine base, e_dot=0 and e is constant, so the construction reduces
// exactly to the successful historical R45/R47 e^perp correction subspace.
//
// These are synthesis constraints only.  Safety acceptance remains the
// unchanged exact-C7 / finite-sensing-speed / yaw-C6 / rejoin / whole-body
// certificate chain.  Failure is fail-closed; no alternative family fallback
// is introduced.
static inline bool r53_add_moving_normal_bundle_rows3(
        std::vector<Row>&rows,const std::vector<double>&K,
        const ActiveReference&ref,const R51PhaseSpec&phase,
        int&nodeCount,int&rowCount,int&singularCount){
    const int N=(int)K.size(),ns=r37_scalar_dofs(K),nvar=3*ns;
    if(N<3||ns<=0)return false;

    for(int node=1;node<N-1;++node){
        const double t=K[node];
        auto x=r51_phase_reference_eval(ref,phase,t);
        const Vec3 v=x[1],a=x[2];
        const double nv=norm(v);
        if(!(finite(v)&&finite(a)&&fin(nv)&&nv>0.0)){
            ++singularCount;
            return false;
        }
        const Vec3 e=v/nv;
        const Vec3 edot=(a-e*dot(e,a))/nv;
        if(!(finite(e)&&finite(edot))){
            ++singularCount;
            return false;
        }

        const int qp=r37_scalar_index(node,0,N);
        const int qv=r37_scalar_index(node,1,N);
        if(qp<0||qv<0){
            ++singularCount;
            return false;
        }

        auto push_eq=[&](Vec3 cp,Vec3 cv){
            Row r;r.a.assign(nvar,0.0);r.b=0.0;
            r.a[3*qp+0]=cp.x;r.a[3*qp+1]=cp.y;r.a[3*qp+2]=cp.z;
            r.a[3*qv+0]=cv.x;r.a[3*qv+1]=cv.y;r.a[3*qv+2]=cv.z;
            rows.push_back(r);
            for(double&z:r.a)z=-z;
            rows.push_back(std::move(r));
            rowCount+=2;
        };

        push_eq(e,Vec3{});   // e^T P = 0
        push_eq(edot,e);     // e^T V + e_dot^T P = 0
        ++nodeCount;
    }
    return true;
}

static inline std::vector<Row> r52_phase_corridor_rows3(
        const ActiveReference&ref,const R51PhaseSpec&phase,
        const Live&L,const SpeedCert&base,const std::vector<Obstacle>&obs,
        const EpisodeCert&E,int ei,int side,const std::vector<double>&K,
        R45CorridorDiag*diag=nullptr){
    const int ns=r37_scalar_dofs(K),nvar=3*ns;
    const auto&ep=E.ep[ei];const auto&o=obs[ep.obs];
    const double VC=std::max(base.VL,base.VQ);
    std::vector<Row> rows;
    double ta=ep.start,te=ep.end;
    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        const double tb=std::min(te,ta+L.dt);
        const double tc=0.5*(ta+tb),half=0.5*(tb-ta);
        auto x=r51_phase_reference_eval(ref,phase,tc);
        Vec3 pL=x[0],v=x[1],sf=x[2]+Vec3{0,0,L.g};
        const double nv=norm(v),nf=norm(sf);
        if(!(fin(nv)&&nv>0&&fin(nf)&&nf>0))
            throw std::runtime_error("R52 phased local frame singular");
        Vec3 eprog=v/nv;
        Vec3 q=sf*(-1.0/nf),pQ=pL-q*L.L;
        Vec3 n0=lateral_normal(o.R,o.axes,eprog);
        Vec3 elat=(side>=0)?n0:n0*(-1.0);
        elat=elat-eprog*dot(elat,eprog);
        const double nl=norm(elat);
        if(!(fin(nl)&&nl>0))throw std::runtime_error("R52 lateral projection singular");
        elat=elat/nl;
        const double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        const double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        const double rhoC=L.rC+L.dmargin+L.tracking+VC*half;
        auto SP=r52_best_support_plane(pL,pQ,o,rhoL,rhoQ,rhoC,eprog,elat);
        if(!SP.ok)throw std::runtime_error("R52 support plane optimization failed");
        if(diag){
            ++diag->intervals;
            if(SP.req<=0.0)++diag->zero_offset_intervals;
            diag->tangent_peak=std::max(diag->tangent_peak,SP.req);
        }
        if(SP.req>0.0){
            const double req=SP.req+64*epsd()*std::max(1.0,std::fabs(SP.req));
            auto w=r37_scalar_basis(K,tc);
            Row R;R.a.assign(nvar,0.0);R.b=req;
            for(int qq=0;qq<ns;++qq){
                R.a[3*qq+0]=w[qq]*SP.m.x;
                R.a[3*qq+1]=w[qq]*SP.m.y;
                R.a[3*qq+2]=w[qq]*SP.m.z;
            }
            rows.push_back(std::move(R));
        }
        ta=tb;
    }
    return rows;
}


// AF7R55 episode-local bypass side selection.
//
// Every row still uses the exact support function of the full oriented
// ellipsoid (center, rotation, and all three semiaxes).  Both local homotopy
// sides are evaluated for each episode; different obstacle episodes may pick
// different sides.  This removes the old "one global side for every obstacle"
// restriction without 2^N global enumeration.
struct R55EpisodeRows{
    bool ok=false;
    int sign=0;
    bool plus_ok=false,minus_ok=false;
    double plus_peak=std::numeric_limits<double>::infinity();
    double minus_peak=std::numeric_limits<double>::infinity();
    double plus_sum=std::numeric_limits<double>::infinity();
    double minus_sum=std::numeric_limits<double>::infinity();
    R45CorridorDiag plus_diag{},minus_diag{};
    std::vector<Row> rows;
};

static inline double r55_positive_row_sum(const std::vector<Row>&R){
    double s=0.0;
    for(const auto&r:R)if(fin(r.b)&&r.b>0.0)s+=r.b;
    return s;
}

static inline R55EpisodeRows r55_choose_episode_rows(
        const ActiveReference&ref,const R51PhaseSpec&phase,
        const Live&L,const SpeedCert&base,const std::vector<Obstacle>&obs,
        const EpisodeCert&E,int ei,const std::vector<double>&K){
    R55EpisodeRows O;
    std::vector<Row> plus,minus;
    try{
        plus=r52_phase_corridor_rows3(
            ref,phase,L,base,obs,E,ei,+1,K,&O.plus_diag);
        O.plus_ok=!plus.empty();
    }catch(const std::runtime_error&){O.plus_ok=false;}
    try{
        minus=r52_phase_corridor_rows3(
            ref,phase,L,base,obs,E,ei,-1,K,&O.minus_diag);
        O.minus_ok=!minus.empty();
    }catch(const std::runtime_error&){O.minus_ok=false;}

    if(!O.plus_ok&&!O.minus_ok)return O;

    if(O.plus_ok){
        O.plus_peak=O.plus_diag.tangent_peak;
        O.plus_sum=r55_positive_row_sum(plus);
    }
    if(O.minus_ok){
        O.minus_peak=O.minus_diag.tangent_peak;
        O.minus_sum=r55_positive_row_sum(minus);
    }

    if(O.plus_ok&&!O.minus_ok){
        O.sign=+1;O.rows=std::move(plus);
    }else if(!O.plus_ok&&O.minus_ok){
        O.sign=-1;O.rows=std::move(minus);
    }else{
        // Lexicographic, not weighted: first minimize peak exact-support
        // displacement requirement, then total positive row requirement.
        const double sc=std::max({1.0,std::fabs(O.plus_peak),std::fabs(O.minus_peak)});
        const double g=65536*epsd()*sc;
        if(O.plus_peak<O.minus_peak-g){
            O.sign=+1;O.rows=std::move(plus);
        }else if(O.minus_peak<O.plus_peak-g){
            O.sign=-1;O.rows=std::move(minus);
        }else{
            const double ss=std::max({1.0,std::fabs(O.plus_sum),std::fabs(O.minus_sum)});
            const double gs=65536*epsd()*ss;
            if(O.minus_sum<O.plus_sum-gs){
                O.sign=-1;O.rows=std::move(minus);
            }else{
                O.sign=+1;O.rows=std::move(plus);
            }
        }
    }
    O.ok=!O.rows.empty();
    return O;
}

static inline void r55_ellipsoid_audit(
        const std::vector<Obstacle>&obs,int&count,double&amin,double&amax,double&orthErr){
    count=(int)obs.size();
    amin=std::numeric_limits<double>::infinity();
    amax=0.0;orthErr=0.0;
    for(const auto&o:obs){
        const double ax[3]={o.axes.x,o.axes.y,o.axes.z};
        for(double x:ax){
            if(fin(x)){amin=std::min(amin,x);amax=std::max(amax,x);}
        }
        for(int i=0;i<3;++i)for(int j=0;j<3;++j){
            double z=0.0;
            for(int k=0;k<3;++k)z+=o.R(k,i)*o.R(k,j);
            const double target=(i==j)?1.0:0.0;
            if(fin(z))orthErr=std::max(orthErr,std::fabs(z-target));
        }
    }
}


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
static inline std::array<Vec3,7> r51_candidate_base_eval(
        const ActiveReference&ref,const Candidate&C,double t);

static inline VLeaf make_vleaf(const ActiveReference&ref,const Live&L,const SpeedCert&base,
                               const Candidate&Cand,const Poly&C0,double ta,double tb,int seg){
    VLeaf f;f.seg=seg;f.ta=ta;f.tb=tb;f.C0=C0;double T=tb-ta;
    if(!(fin(T)&&T>0&&C0.size()==16)){f.vq=std::numeric_limits<double>::infinity();return f;}
    Poly C1=deriv_poly(C0,T,1),C2=deriv_poly(C0,T,2),C3=deriv_poly(C0,T,3);
    double tc=0.5*(ta+tb),h=0.5*T;auto x=r51_candidate_base_eval(ref,Cand,tc);Vec3 v0=x[1],a0=x[2],j0=x[3];
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
                                     const Candidate&C,double lim,bool&finiteWitness,bool&repr,
                                     R52StageBTrace* dbg=nullptr){
    if(dbg){*dbg=R52StageBTrace{};dbg->attempted=1;}
    std::vector<VLeaf> leaves;for(size_t s=0;s<C.cp.size();++s)leaves.push_back(make_vleaf(ref,L,base,C,C.cp[s],C.seg0[s],C.seg1[s],(int)s));
    int splits=0;
    auto update_dbg=[&](){
        if(!dbg)return;
        dbg->leaf_count=(int)leaves.size();dbg->split_count=splits;
        for(const auto&f:leaves){
            const double vv=fin(f.vq)?f.vq:std::numeric_limits<double>::infinity();
            if(vv>dbg->worst_vq){
                dbg->worst_vq=vv;dbg->worst_t=0.5*(f.ta+f.tb);dbg->worst_seg=f.seg;
                dbg->worst_vl=f.vl;dbg->worst_sigma=f.sigma;dbg->worst_qdot=f.qdot;dbg->worst_witness=f.wit;
            }
        }
    };
    update_dbg();
    while(true){
        bool all=true;for(auto&f:leaves)if(!(fin(f.vq)&&f.vq<lim)){all=false;break;}
        if(all)break;
        for(auto&f:leaves)if(fin(f.wit)&&f.wit>=lim){finiteWitness=true;
            if(dbg){update_dbg();dbg->result_code=2;dbg->worst_witness=f.wit;dbg->worst_t=0.5*(f.ta+f.tb);dbg->worst_seg=f.seg;}
            towmap_core_record_witness(C.family,f.seg,(int)C.cp.size(),0.5*(f.ta+f.tb),f.wit);return {};}
        int bi=-1;double bs=-std::numeric_limits<double>::infinity();
        for(int i=0;i<(int)leaves.size();++i)if(!fin(leaves[i].vq)||leaves[i].vq>=lim){
            double sc=fin(leaves[i].vq)?leaves[i].vq-lim:std::numeric_limits<double>::infinity();
            if(sc>bs){bs=sc;bi=i;}
        }
        if(bi<0)break;auto old=leaves[bi];double tm=0.5*(old.ta+old.tb);
        if(!(fin(tm)&&tm>old.ta&&tm<old.tb)){repr=true;if(dbg){update_dbg();dbg->result_code=3;}return {};}
        auto sp=split_half(old.C0);
        leaves[bi]=make_vleaf(ref,L,base,C,sp.first,old.ta,tm,old.seg);
        leaves.push_back(make_vleaf(ref,L,base,C,sp.second,tm,old.tb,old.seg));++splits;
        update_dbg();
    }
    SpeedCert s;s.certified=true;s.stageA=false;s.leaf_count=(int)leaves.size();s.split_count=splits;
    s.VQ=0;s.VL=0;s.A=0;s.J=0;s.sigma=std::numeric_limits<double>::infinity();s.qdot=0;
    for(auto&f:leaves){if(!fin(f.vq)||!fin(f.sigma)||f.sigma<=0){s.certified=false;return s;}
        s.VQ=std::max(s.VQ,f.vq);s.VL=std::max(s.VL,f.vl);s.A=std::max(s.A,f.al);s.J=std::max(s.J,f.jl);
        s.sigma=std::min(s.sigma,f.sigma);s.qdot=std::max(s.qdot,f.qdot);}
    if(!(s.VQ<lim&&s.sigma>0))s.certified=false;
    if(dbg){update_dbg();dbg->result_code=s.certified?1:4;}
    return s;
}

// -------------------------- corrected candidate eval ------------------------
static inline std::array<Vec3,7> r51_candidate_base_eval(
        const ActiveReference&ref,const Candidate&C,double t){
    if(!C.phase_reparameterized||!C.phase.enabled)
        return ref.eval(t);
    const auto S=r51_phase_jet(C.phase,t);
    auto b=ref.eval(S[0]);
    return r51_compose_vec_jet(b,S);
}

static inline std::array<Vec3,7> candidate_eval(
        const ActiveReference&ref,const Candidate&C,double t){
    auto y=r51_candidate_base_eval(ref,C,t);
    if(C.cp.empty()||t<=C.seg0.front())return y;

    if(t>=C.seg1.back()){
        if(!C.phase_reparameterized && C.terminal_hold)
            y[0]+=C.terminal_offset;
        return y;
    }

    int s=-1;
    for(int i=0;i<(int)C.cp.size();++i){
        if(t<=C.seg1[i]){s=i;break;}
    }
    if(s<0||t<C.seg0[s])return y;

    const double T=C.seg1[s]-C.seg0[s];
    double u=std::max(0.0,std::min(1.0,(t-C.seg0[s])/T));
    Poly D=C.cp[s];
    y[0]+=decasteljau(D,u);
    int deg=15;
    for(int r=1;r<=6;++r){
        Poly E(D.size()-1);
        for(size_t i=0;i+1<D.size();++i)
            E[i]=(D[i+1]-D[i])*(double(deg)/T);
        D.swap(E);--deg;
        y[r]+=decasteljau(D,u);
    }
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
// The packet field `t1` / `search_end` is a finite source-derived active
// reference-validity boundary. It is NOT a fixed physical avoidance-duration
// parameter.
//
// AF7R47 currently certifies one fixed-t1 candidate per native call. The
// historical repeated full-planner rejoin-shortening search was removed from
// the hard cycle because it was quality-only and caused large online spikes.
// Automatic shorter rejoin remains a later quality layer; when reintroduced
// it must reuse the already-built episode/corridor rather than weaken or
// duplicate the hard certificates.
//
// Every accepted candidate still passes exactly the same:
//   * exact-C7 structural gate,
//   * continuous path-tangent yaw-C6 regularity gate,
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


// AF7R47 ACTIVE-BASE PATH-TANGENT YAW C6 CERTIFICATE
//
// Let vb(t) be the horizontal velocity of the currently active certified
// reference, and vn(t) the horizontal velocity after adding the new exact-C7
// correction.  We certify
//
//      g(t) = vb_xy(t)' vn_xy(t) > 0
//
// continuously on [ta,tb].  Hence both horizontal velocities are nonzero and
// the new tangent direction stays strictly within +/-pi/2 of the old tangent
// direction at every time.  This provides a unique local branch transfer from
// the already continuous old path-tangent yaw to the new path-tangent yaw.
//
// For one leaf I with midpoint tc and half-width h,
//
//   |g'(t)| <= A_b V_n + V_b A_n = Lg,
//
// where (V_b,A_b) are the certified active-reference bounds and (V_n,A_n)
// are the newly certified candidate bounds. Therefore
//
//   g(t) >= g(tc) - Lg h.
//
// Subdivision terminates only when that continuous lower bound is positive.
// If binary64 cannot form a strict midpoint first, certification fails closed.
// There is no sampled acceptance and no fixed physical angle threshold.
static inline bool r47_apply_heading_certificate(
        const ActiveReference&ref,Candidate&C,
        const SpeedCert&base,const SpeedCert&now,
        double ta,double tb,double&gammaOut,double&anchorOut,
        int&leafCount,int&splitCount,R52HeadingTrace* dbg=nullptr){
    gammaOut=std::numeric_limits<double>::quiet_NaN();
    anchorOut=std::numeric_limits<double>::quiet_NaN();
    leafCount=0;splitCount=0;
    if(dbg){*dbg=R52HeadingTrace{};dbg->attempted=1;}

    if(!(base.certified&&now.certified&&
            fin(base.VL)&&base.VL>0&&fin(base.A)&&base.A>=0&&
            fin(now.VL)&&now.VL>0&&fin(now.A)&&now.A>=0&&
            fin(ta)&&fin(tb)&&tb>ta)){
        if(dbg)dbg->reason=1;
        return false;
    }

    const double Lg=base.A*now.VL + base.VL*now.A;
    if(!(fin(Lg)&&Lg>=0)){if(dbg)dbg->reason=2;return false;}

    struct Leaf{double a,b;};
    std::vector<Leaf> st;
    st.push_back({ta,tb});
    double minDot=std::numeric_limits<double>::infinity();

    while(!st.empty()){
        Leaf f=st.back();st.pop_back();
        const double tc=0.5*(f.a+f.b);
        const double h=0.5*(f.b-f.a);
        if(!(fin(tc)&&fin(h)&&h>0&&tc>f.a&&tc<f.b)){
            if(dbg){dbg->reason=3;dbg->worst_a=f.a;dbg->worst_b=f.b;}
            return false;
        }

        const auto xb=r51_candidate_base_eval(ref,C,tc);
        const auto xn=candidate_eval(ref,C,tc);
        const double g0=xb[1].x*xn[1].x + xb[1].y*xn[1].y;
        if(!fin(g0)){if(dbg)dbg->reason=3;return false;}

        const double lb=g0-Lg*h;
        if(dbg){
            dbg->min_g0=std::min(dbg->min_g0,g0);
            dbg->min_lb=std::min(dbg->min_lb,lb);
            if(lb<=dbg->min_lb){dbg->worst_a=f.a;dbg->worst_b=f.b;}
        }
        ++leafCount;
        if(fin(lb)&&lb>0.0){
            minDot=std::min(minDot,lb);
            continue;
        }

        st.push_back({tc,f.b});
        st.push_back({f.a,tc});
        ++splitCount;
    }

    if(dbg){dbg->leaves=leafCount;dbg->splits=splitCount;}
    if(!(fin(minDot)&&minDot>0.0&&base.VL>0.0)){
        if(dbg)dbg->reason=4;
        return false;
    }

    // g <= ||vb_xy|| ||vn_xy|| <= Vb ||vn_xy||.
    const double gamma=minDot/base.VL;
    if(!(fin(gamma)&&gamma>0.0)){if(dbg)dbg->reason=4;return false;}

    const auto x0=r51_candidate_base_eval(ref,C,ta);
    const double nh=std::hypot(x0[1].x,x0[1].y);
    if(!(fin(nh)&&nh>0.0)){if(dbg)dbg->reason=5;return false;}

    C.heading_c6=true;
    C.heading_gamma=gamma;
    C.heading_anchor_rad=std::atan2(x0[1].y,x0[1].x);
    C.heading_base_velocity=x0[1];
    gammaOut=gamma;
    anchorOut=C.heading_anchor_rad;
    if(dbg){dbg->reason=0;dbg->leaves=leafCount;dbg->splits=splitCount;}
    return true;
}


// ===========================================================================
// AF7R57 GENERAL PATH-TANGENT YAW-C6 CERTIFICATE
//
// Exact-C7 position => horizontal velocity v_xy is C6.
// If inf ||v_xy|| > 0 on a connected time interval, the normalized tangent
// is C6 and has a global continuous C6 angle lift.  Certify nonvanishing by
//
//      ||v_xy(t)|| >= ||v_xy(tc)|| - A*h
//
// using the already certified total acceleration bound A.  No old-heading
// half-plane, no physical angle threshold, and no fixed subdivision depth.
// Binary64 midpoint exhaustion fails closed.
// ===========================================================================
static inline bool r57_apply_heading_certificate(
        const ActiveReference&ref,Candidate&C,const SpeedCert&now,
        double ta,double tb,double&gammaOut,double&anchorOut,
        int&leafCount,int&splitCount){
    gammaOut=std::numeric_limits<double>::quiet_NaN();
    anchorOut=std::numeric_limits<double>::quiet_NaN();
    leafCount=0;splitCount=0;

    if(!(now.certified&&fin(now.A)&&now.A>=0.0&&
         fin(ta)&&fin(tb)&&tb>ta))
        return false;

    struct Leaf{double a,b;};
    std::vector<Leaf> st;
    st.push_back({ta,tb});
    double gamma=std::numeric_limits<double>::infinity();

    while(!st.empty()){
        const Leaf f=st.back();st.pop_back();
        const double tc=0.5*(f.a+f.b);
        const double h=0.5*(f.b-f.a);
        if(!(fin(tc)&&fin(h)&&h>0.0&&tc>f.a&&tc<f.b))
            return false;

        const auto x=candidate_eval(ref,C,tc);
        const double vxy=std::hypot(x[1].x,x[1].y);
        const double lb=vxy-now.A*h;
        ++leafCount;

        if(fin(lb)&&lb>0.0){
            gamma=std::min(gamma,lb);
            continue;
        }

        st.push_back({tc,f.b});
        st.push_back({f.a,tc});
        ++splitCount;
    }

    if(!(fin(gamma)&&gamma>0.0))
        return false;

    const auto x0=candidate_eval(ref,C,ta);
    const double h0=std::hypot(x0[1].x,x0[1].y);
    if(!(fin(h0)&&h0>0.0))
        return false;

    C.heading_c6=true;
    C.heading_gamma=gamma;
    C.heading_anchor_rad=std::atan2(x0[1].y,x0[1].x);
    C.heading_base_velocity=x0[1];
    gammaOut=gamma;
    anchorOut=C.heading_anchor_rad;
    return true;
}



// ===========================================================================
// AF7R58 FREE-C7 SOURCE-SPACE REFERENCE CORE
//
// At every interior source knot, correction jets d^(0)...d^(7) are free.
// Adjacent degree-15 Bezier segments share the same node jet. The start jet
// is zero. At the finite source-validity boundary only derivatives 1...7 are
// zero; position is free and continues as a constant offset. No prescribed
// zero-position rejoin or same-clock catch-up exists.
//
// QP variable z_r = d^(r) lambda_j^r with a symmetric local node scale
// lambda_j=sqrt(T_- T_+) (terminal: T_-), so each variable has units
// of length. This is an invertible coordinate transform only; physical
// objective, constraints, and feasible trajectories are unchanged.
// The moving-normal equalities are eliminated analytically before solving.
// Each interior (P,V) pair is parameterized by four normal-bundle coordinates,
// so no +/- equality-row pair reaches the active-set solver. A second
// diagonal Hessian equilibration y_i=sqrt(H_ii) z_i is then applied
// before the active-set solve; rows are transformed consistently and the
// physical z is recovered exactly by the inverse coordinate map. Before storing a candidate, the shared node jets are projected
// onto a common dyadic lattice in normalized Bernstein-difference
// coordinates. Checked int64 endpoint-difference identities then provide a
// constructive exact-real C7 proof even for unequal segment durations.
//
// Each unsafe episode caches both exact-support +/- corridor alternatives.
// Best-first side-graph search uses the convex projection cost only as an
// ordering key; NO branch is pruned by cost. Thus safety and side-graph
// coverage do not rely on objective optimality. The finite numerical resource
// guard may still fail closed before an existing feasible branch is found.
// No local greedy side choice is used.
//
// Reference-only shadow: execution tracking/observer/HLC tubes are outside
// this stage by design.
//
// R58R7 QP implementation uses exact constraint generation: each subset QP
// is solved to KKT closure, then ALL omitted rows are checked. A violated row
// is added and the process repeats. Once the subset optimum satisfies all
// full rows, it is also the full-QP optimum by feasible-set inclusion. No row
// is permanently discarded and no sampled/partial-row result can grant
// acceptance.

//
// R58R8 solves each small generated subset with an infeasible-start
// primal-dual Mehrotra method.  The Newton matrix is
// H + A' diag(lambda./s) A and is SPD whenever H is SPD and s,lambda>0.
// Thus no active-row independence assumption is required.  Full-row
// optimality is still established only by the unchanged R58R7 all-row scan.


//
// R58R11 replaces the ill-conditioned normal-equation PCG solve by the
// algebraically equivalent augmented Newton system in (dz,dlambda):
// [H,-A'; Lambda*A,S]. Positive row/column equilibration and complete pivoting
// are invertible numerical operations; the original Newton residual is checked
// before the direction is accepted. No lambda/s normal equation is formed.
//
// R58R11 also uses one common primal/dual step.  Because the primal and dual
// KKT residual equations are linear in the Newton direction, this guarantees
// r_p^+=(1-alpha)r_p and r_d^+=(1-alpha)r_d in exact arithmetic.  Positivity
// is enforced with the largest common step, moved by one binary64 value toward
// zero only when the boundary is active.  No empirical step fraction is used.
// ===========================================================================

static inline double r58_ff(int r){
    double f=1.0;
    for(int k=0;k<r;++k)f*=double(15-k);
    return f;
}
static inline double r58_tau_pow(double tau,int r){
    double p=1.0;
    for(int k=0;k<r;++k)p*=tau;
    return p;
}

static inline double r58_node_scale(
        const std::vector<double>&K,int node){
    const int N=(int)K.size();
    if(node<=0||node>=N)
        return std::numeric_limits<double>::quiet_NaN();
    const double Tm=K[node]-K[node-1];
    if(!(fin(Tm)&&Tm>0.0))
        return std::numeric_limits<double>::quiet_NaN();
    if(node==N-1)return Tm;
    const double Tp=K[node+1]-K[node];
    if(!(fin(Tp)&&Tp>0.0))
        return std::numeric_limits<double>::quiet_NaN();

    // Symmetric minimax-log conditioning scale:
    // lambda = sqrt(T_- T_+) minimizes
    // max{|log(T_-/lambda)|, |log(T_+/lambda)|}.
    return std::sqrt(Tm*Tp);
}

static inline int r58_scalar_dofs(const std::vector<double>&K){
    const int N=(int)K.size();
    return N>=2 ? 8*std::max(0,N-2)+1 : 0;
}
static inline int r58_scalar_index(int node,int r,int N){
    if(node<=0||node>=N||r<0||r>7)return -1;
    if(node==N-1)return r==0 ? 8*(N-2) : -1;
    return 8*(node-1)+r;
}

static inline std::vector<double> r58_scalar_basis(
        const std::vector<double>&K,double t){
    const int N=(int)K.size(),ns=r58_scalar_dofs(K);
    std::vector<double>w(ns,0.0);
    if(N<2||ns<=0)return w;
    const int s=r37_find_segment(K,t);
    if(s<0)return w;
    const double T=K[s+1]-K[s];
    if(!(fin(T)&&T>0.0))return w;
    double u=(t-K[s])/T;
    u=std::max(0.0,std::min(1.0,u));

    for(int a=0;a<2;++a){
        const int node=(a==0)?s:s+1;
        const double lambda=(node>0)?r58_node_scale(K,node):1.0;
        if(node>0&&!(fin(lambda)&&lambda>0.0))return std::vector<double>(ns,0.0);
        for(int r=0;r<=7;++r){
            const int q=r58_scalar_index(node,r,N);
            if(q<0)continue;
            std::array<double,8>J0{},J1{};
            const double jr=(node>0)?1.0/std::pow(lambda,r):1.0;
            if(a==0)J0[r]=jr;else J1[r]=jr;
            const auto cp=scalar_control_from_jets(J0,J1,T);
            w[q]=r36_scalar_bezier_eval(cp,u);
        }
    }
    return w;
}

// R23: coefficients are independent of sensed obstacles, reference version,
// payload estimate and current trajectory. Prewarm these one-time tables before
// Flight; all runtime-dependent Hessian terms are STILL computed per candidate.
static inline const std::array<std::array<double,256>,4>& r58r23_gram_table(){
    static const auto G=[](){
        std::array<std::array<double,256>,4> T{};
        for(int d=0;d<=3;++d){
            const int deg=15-d;
            for(int a=0;a<=deg;++a)for(int b=0;b<=deg;++b)
                T[d][16*a+b]=gram(deg,a,b);
        }
        return T;
    }();
    return G;
}

static inline const std::array<std::array<double,8>,8>& r58r23_binomial_table(){
    static const auto B=[](){
        std::array<std::array<double,8>,8> T{};
        for(int n=0;n<=7;++n)for(int k=0;k<=n;++k){
            double v=1.0;
            for(int i=1;i<=k;++i)v*=double(n-k+i)/double(i);
            T[n][k]=v;
        }
        return T;
    }();
    return B;
}

static inline std::vector<double> r58r22_scalar_hessian(
        const std::vector<double>&K,double cableL,double gravity){
    const int N=(int)K.size(),ns=r58_scalar_dofs(K);
    if(N<2||ns<=0||!(fin(cableL)&&cableL>0&&fin(gravity)&&gravity>0))
        return {};
    const double tau=std::sqrt(cableL/gravity);
    if(!(fin(tau)&&tau>0.0))return {};

    const double tau2=tau*tau;
    const double wr[4]={1.0/tau2,1.0,tau2,tau2*tau2};
    // R23: same exact Gram values, now warmed from the setup-only opcode.
    const auto& grams=r58r23_gram_table();
    std::vector<double>Hs(ns*ns,0.0);

    for(int s=0;s<N-1;++s){
        const double T=K[s+1]-K[s];
        if(!(fin(T)&&T>0.0))return {};

        std::vector<int>qv;
        std::vector<std::vector<double>>CP;
        for(int a=0;a<2;++a){
            const int node=(a==0)?s:s+1;
            for(int r=0;r<=7;++r){
                const int q=r58_scalar_index(node,r,N);
                if(q<0)continue;
                std::array<double,8>J0{},J1{};
                const double lambda=r58_node_scale(K,node);
                if(!(fin(lambda)&&lambda>0.0))return {};
                const double jr=1.0/std::pow(lambda,r);
                if(a==0)J0[r]=jr;else J1[r]=jr;
                qv.push_back(q);
                CP.push_back(scalar_control_from_jets(J0,J1,T));
            }
        }

        for(int d=0;d<=3;++d){
            const int deg=15-d,ncp=16-d;
            std::vector<std::vector<double>>D(CP.size());
            double ff=1.0;
            for(int q=0;q<d;++q)ff*=double(15-q);
            for(size_t i=0;i<CP.size();++i){
                D[i]=diff_scalar(CP[i],d);
                const double sc=(d==0)?1.0:ff/std::pow(T,d);
                for(double&v:D[i])v*=sc;
            }
            for(size_t i=0;i<D.size();++i)for(size_t j=0;j<D.size();++j){
                double q=0.0;
                for(int aa=0;aa<ncp;++aa)for(int bb=0;bb<ncp;++bb)
                    q+=D[i][aa]*grams[d][16*aa+bb]*D[j][bb];
                Hs[qv[i]*ns+qv[j]]+=wr[d]*T*q;
            }
        }
    }

    return Hs;
}

static inline std::vector<double> r58_spatial_hessian3(
        const std::vector<double>&K,double cableL,double gravity){
    const int ns=r58_scalar_dofs(K);
    const auto Hs=r58r22_scalar_hessian(K,cableL,gravity);
    if((int)Hs.size()!=ns*ns||ns<=0)return {};
    const int n=3*ns;
    std::vector<double>H(n*n,0.0);
    for(int i=0;i<ns;++i)for(int j=0;j<ns;++j){
        const double hij=Hs[i*ns+j];
        for(int a=0;a<3;++a)
            H[(3*i+a)*n+(3*j+a)]=hij;
    }
    return H;
}

static inline bool r58_add_moving_normal_rows(
        std::vector<Row>&rows,const std::vector<double>&K,
        const ActiveReference&ref,
        int&nodeCount,int&rowCount,int&singularCount){
    const int N=(int)K.size(),ns=r58_scalar_dofs(K),nvar=3*ns;
    if(N<3||ns<=0)return false;

    for(int node=1;node<N-1;++node){
        const auto x=ref.eval(K[node]);
        const Vec3 v=x[1],a=x[2];
        const double nv=norm(v);
        if(!(finite(v)&&finite(a)&&fin(nv)&&nv>0.0)){
            ++singularCount;return false;
        }
        const Vec3 e=v/nv;
        const Vec3 edot=(a-e*dot(e,a))/nv;
        const int qp=r58_scalar_index(node,0,N);
        const int qv=r58_scalar_index(node,1,N);
        if(qp<0||qv<0){++singularCount;return false;}

        auto push_eq=[&](Vec3 cp,Vec3 cv){
            Row R;R.a.assign(nvar,0.0);R.b=0.0;
            R.a[3*qp+0]=cp.x;R.a[3*qp+1]=cp.y;R.a[3*qp+2]=cp.z;
            R.a[3*qv+0]=cv.x;R.a[3*qv+1]=cv.y;R.a[3*qv+2]=cv.z;
            rows.push_back(R);
            for(double&z:R.a)z=-z;
            rows.push_back(std::move(R));
            rowCount+=2;
        };
        push_eq(e,Vec3{});
        const double lambda=r58_node_scale(K,node);
        if(!(fin(lambda)&&lambda>0.0)){++singularCount;return false;}
        push_eq(edot,e*(1.0/lambda));
        ++nodeCount;
    }
    return true;
}

struct R58SideRows{
    bool valid=false;
    int sign=0;
    double peak=0.0;
    std::vector<Row>rows;
};

static inline R58SideRows r58_episode_rows(
        const ActiveReference&ref,const Live&L,const SpeedCert&base,
        const std::vector<Obstacle>&obs,const EpisodeCert&E,int ei,int side,
        const std::vector<double>&K){
    R58SideRows O;O.sign=side;
    const int ns=r58_scalar_dofs(K),nvar=3*ns;
    if(ns<=0)return O;
    const auto&ep=E.ep[ei];const auto&o=obs[ep.obs];
    const double VC=std::max(base.VL,base.VQ);
    double ta=ep.start,te=ep.end;

    while(ta<te-8*epsd()*std::max(1.0,std::fabs(te))){
        const double tb=std::min(te,ta+L.dt);
        const double tc=0.5*(ta+tb),half=0.5*(tb-ta);
        auto x=ref.eval(tc);
        Vec3 pL=x[0],v=x[1],sf=x[2]+Vec3{0,0,L.g};
        const double nv=norm(v),nf=norm(sf);
        if(!(fin(nv)&&nv>0.0&&fin(nf)&&nf>0.0))return O;

        Vec3 eprog=v/nv;
        Vec3 q=sf*(-1.0/nf),pQ=pL-q*L.L;
        Vec3 n0=lateral_normal(o.R,o.axes,eprog);
        Vec3 elat=(side>=0)?n0:n0*(-1.0);
        elat=elat-eprog*dot(elat,eprog);
        const double nl=norm(elat);
        if(!(fin(nl)&&nl>0.0))return O;
        elat=elat/nl;

        const double rhoL=L.rL+L.dmargin+L.tracking+base.VL*half;
        const double rhoQ=L.rQ+L.dmargin+L.tracking+base.VQ*half;
        const double rhoC=L.rC+L.dmargin+L.tracking+VC*half;
        auto SP=r52_best_support_plane(
            pL,pQ,o,rhoL,rhoQ,rhoC,eprog,elat);
        if(!SP.ok)return O;
        O.peak=std::max(O.peak,SP.req);

        if(SP.req>0.0){
            const double req=SP.req+
                64*epsd()*std::max(1.0,std::fabs(SP.req));
            auto w=r58_scalar_basis(K,tc);
            Row R;R.a.assign(nvar,0.0);R.b=req;
            for(int qq=0;qq<ns;++qq){
                R.a[3*qq+0]=w[qq]*SP.m.x;
                R.a[3*qq+1]=w[qq]*SP.m.y;
                R.a[3*qq+2]=w[qq]*SP.m.z;
            }
            O.rows.push_back(std::move(R));
        }
        ta=tb;
    }
    O.valid=true;
    return O;
}

struct R58NodeLattice{
    int e[3]{0,0,0};
    long long q[3][8]{};
};

// MSVC-portable exact integer arithmetic for the dyadic C7 lattice.
// No floating approximation is used in the endpoint-difference identities.
// If a platform-int64 operation would overflow, construction fails closed.
static_assert(std::numeric_limits<long long>::digits>=63,
              "R58 requires at least signed 64-bit long long");

static inline long long r58_checked_add_ll(long long a,long long b){
    const long long hi=std::numeric_limits<long long>::max();
    const long long lo=std::numeric_limits<long long>::min();
    if((b>0&&a>hi-b)||(b<0&&a<lo-b))
        throw std::runtime_error("R58 int64 add overflow");
    return a+b;
}
static inline long long r58_checked_mul_ll(long long a,long long b){
    const long long hi=std::numeric_limits<long long>::max();
    const long long lo=std::numeric_limits<long long>::min();
    if(a==0||b==0)return 0;
    if(a==-1){
        if(b==lo)throw std::runtime_error("R58 int64 mul overflow");
        return -b;
    }
    if(b==-1){
        if(a==lo)throw std::runtime_error("R58 int64 mul overflow");
        return -a;
    }
    if(a>0){
        if(b>0){
            if(a>hi/b)throw std::runtime_error("R58 int64 mul overflow");
        }else{
            if(b<lo/a)throw std::runtime_error("R58 int64 mul overflow");
        }
    }else{
        if(b>0){
            if(a<lo/b)throw std::runtime_error("R58 int64 mul overflow");
        }else{
            // a<0,b<0.  Division by a is well-defined here because a!=-1.
            if(a<hi/b)throw std::runtime_error("R58 int64 mul overflow");
        }
    }
    return a*b;
}
static inline long long r58_binom_ll(int n,int k){
    if(k<0||k>n)return 0;
    long long v=1;
    for(int i=1;i<=k;++i)v=v*(n-k+i)/i;
    return v;
}

static inline R58NodeLattice r58_make_node_lattice(
        const std::array<Vec3,8>&Z,double lambda,double h,
        long long Km,long long Kp,bool terminal,
        double&maxJetErr,double&maxPhysicalJetErr){
    R58NodeLattice L;
    if(!(fin(lambda)&&lambda>0&&fin(h)&&h>0&&Km>=1&&Kp>=0))
        throw std::runtime_error("R58 invalid lattice request");

    for(int ax=0;ax<3;++ax){
        double b[8]{};
        for(int r=0;r<=7;++r){
            if(terminal&&r>0){b[r]=0.0;continue;}
            const double z=(ax==0)?Z[r].x:((ax==1)?Z[r].y:Z[r].z);
            const double jr=z/std::pow(lambda,r);
            b[r]=jr*std::pow(h,r)/r58_ff(r);
            if(!fin(b[r]))throw std::runtime_error("R58 nonfinite normalized jet");
        }

        double M=0.0;
        const long long KK[2]={Km,Kp};
        for(int side=0;side<2;++side){
            const long long K=KK[side];
            if(K<1)continue;
            for(int k=0;k<=7;++k){
                long double kp=1.0L;
                for(int r=0;r<=k;++r){
                    if(r>0)kp*=(long double)K;
                    const long double v=
                        std::fabs((long double)b[r])*kp*
                        (long double)r58_binom_ll(k,r);
                    if(!std::isfinite((double)v))
                        throw std::runtime_error("R58 lattice magnitude overflow");
                    M=std::max(M,(double)v);
                }
            }
        }

        int e=(M>0.0)?std::ilogb(M)-44:-1070;
        e=std::max(-1070,std::min(1000,e));
        L.e[ax]=e;

        for(int r=0;r<=7;++r){
            if(terminal&&r>0){L.q[ax][r]=0;continue;}
            const double scaled=std::ldexp(b[r],-e);
            if(!(fin(scaled)&&std::fabs(scaled)<4503599627370496.0))
                throw std::runtime_error("R58 lattice quantization range");
            L.q[ax][r]=std::llround(scaled);

            const double bq=std::ldexp((double)L.q[ax][r],e);
            const double jq=bq*r58_ff(r)/std::pow(h,r);
            const double zq=jq*std::pow(lambda,r);
            const double z=(ax==0)?Z[r].x:((ax==1)?Z[r].y:Z[r].z);
            const double jr0=z/std::pow(lambda,r);
            maxJetErr=std::max(maxJetErr,std::fabs(zq-z));
            maxPhysicalJetErr=std::max(
                maxPhysicalJetErr,std::fabs(jq-jr0));
        }
    }
    return L;
}

static inline double r58_lattice_control(
        const R58NodeLattice&L,int ax,long long K,int k,bool right){
    long long sum=0;
    for(int r=0;r<=k;++r){
        long long term=L.q[ax][r];
        for(int j=0;j<r;++j)
            term=r58_checked_mul_ll(term,K);
        term=r58_checked_mul_ll(term,r58_binom_ll(k,r));
        if(right&&(r&1)){
            if(term==std::numeric_limits<long long>::min())
                throw std::runtime_error("R58 int64 sign overflow");
            term=-term;
        }
        sum=r58_checked_add_ll(sum,term);
    }

    // Every integer up to 2^53 is exactly representable in binary64.
    // Keep one bit of headroom for subsequent dyadic scaling/composition.
    const long long exactLimit=(1LL<<52);
    if(sum<=-exactLimit||sum>=exactLimit)
        throw std::runtime_error("R58 control integer exactness range");

    return std::ldexp((double)sum,L.e[ax]);
}

static inline Candidate r58_build_candidate(
        const QPSol&q,const std::vector<double>&K,double h,
        double cableL,double gravity,const std::vector<int>&signs,
        double&maxJetErr,double&maxPhysicalJetErr,double&legacyMismatch,
        std::array<double,3>&rawQPderivativeUB,bool collectRawQPDiag){
    Candidate C;
    const int N=(int)K.size(),ns=r58_scalar_dofs(K);
    if(!q.ok||N<2||(int)q.z.size()!=3*ns)
        throw std::runtime_error("R58 invalid QP solution");
    if(!(fin(cableL)&&cableL>0.0&&fin(gravity)&&gravity>0.0))
        throw std::runtime_error("R58 invalid physical scale");

    std::vector<std::array<Vec3,8>>Z(N);
    for(int node=1;node<N;++node){
        for(int r=0;r<=7;++r){
            const int qi=r58_scalar_index(node,r,N);
            if(qi<0)continue;
            Z[node][r]={
                q.z[3*qi+0],q.z[3*qi+1],q.z[3*qi+2]};
        }
    }

    // R13 diagnostic only: reconstruct the floating point QP-jet candidate
    // before exact lattice projection and measure its Bernstein derivative UBs.
    // NEVER use this non-lattice candidate for C7 proof or acceptance.
    rawQPderivativeUB.fill(std::numeric_limits<double>::quiet_NaN());
    if(collectRawQPDiag){
    rawQPderivativeUB.fill(0.0);
    for(int s=0;s<N-1;++s){
        const double T=K[s+1]-K[s];
        std::array<Vec3,8> j0{},j1{};
        for(int r=0;r<=7;++r){
            if(s>0){
                const double lam=r58_node_scale(K,s);
                j0[r]=Z[s][r]/std::pow(lam,r);
            }
            const double lam=r58_node_scale(K,s+1);
            j1[r]=Z[s+1][r]/std::pow(lam,r);
        }
        const Poly pre=vector_control_from_jets(j0,j1,T);
        for(int r=1;r<=3;++r){
            const Poly der=deriv_poly(pre,T,r);
            rawQPderivativeUB[r-1]=std::max(rawQPderivativeUB[r-1],
                                             poly_norm_ub(der));
        }
    }
    }
    std::vector<R58NodeLattice>L(N);
    maxJetErr=0.0;maxPhysicalJetErr=0.0;
    for(int node=1;node<N;++node){
        const double Tm=K[node]-K[node-1];
        const long long Km=(long long)std::llround(Tm/h);
        long long Kp=0;
        if(node+1<N){
            const double Tp=K[node+1]-K[node];
            Kp=(long long)std::llround(Tp/h);
            if(!(Kp>=1&&Tp==(double)Kp*h))
                throw std::runtime_error("R58 right knot off exact grid");
        }
        if(!(Km>=1&&Tm==(double)Km*h))
            throw std::runtime_error("R58 left knot off exact grid");
        const double lambda=r58_node_scale(K,node);
        if(!(fin(lambda)&&lambda>0.0))
            throw std::runtime_error("R58 invalid node scale");
        L[node]=r58_make_node_lattice(
            Z[node],lambda,h,Km,Kp,node==N-1,
            maxJetErr,maxPhysicalJetErr);
    }

    C.family=7;C.cost=q.cost;C.minSlack=q.slack;C.signs=signs;
    C.constant_translation=false;
    C.identity_time_law=true;
    C.progress_retimed=false;
    C.progress_scale=1.0;
    C.schedule_delay=0.0;
    C.phase_reparameterized=false;
    C.trigger_to_free_rejoin=false;
    C.actual_rejoin_time=std::numeric_limits<double>::quiet_NaN();
    C.spatial_knot_count=N;
    C.terminal_hold=true;
    C.terminal_hold_start=K.back();

    for(int s=0;s<N-1;++s){
        const double T=K[s+1]-K[s];
        const long long KK=(long long)std::llround(T/h);
        if(!(KK>=1&&T==(double)KK*h))
            throw std::runtime_error("R58 segment off exact grid");

        Poly cp(16,Vec3{});
        if(s>0){
            for(int k=0;k<8;++k){
                cp[k]={
                    r58_lattice_control(L[s],0,KK,k,false),
                    r58_lattice_control(L[s],1,KK,k,false),
                    r58_lattice_control(L[s],2,KK,k,false)};
            }
        }
        for(int k=0;k<8;++k){
            cp[15-k]={
                r58_lattice_control(L[s+1],0,KK,k,true),
                r58_lattice_control(L[s+1],1,KK,k,true),
                r58_lattice_control(L[s+1],2,KK,k,true)};
        }

        C.seg0.push_back(K[s]);
        C.seg1.push_back(K[s+1]);
        C.cp.push_back(std::move(cp));
    }

    C.terminal_offset=C.cp.back().back();
    C.exact_c7=true;
    C.exact_c7_constructive=true;
    C.legacy_float_endpoint_checker_pass=r31_exact_c7_stored(C);

    C.dub.fill(0.0);C.max_ctrl=0.0;
    for(size_t s=0;s<C.cp.size();++s){
        const double T=C.seg1[s]-C.seg0[s];
        Poly D=C.cp[s];int deg=15;
        for(const auto&v:D)C.max_ctrl=std::max(C.max_ctrl,norm(v));
        for(int r=1;r<=6;++r){
            Poly E(D.size()-1);
            for(size_t i=0;i+1<D.size();++i)
                E[i]=(D[i+1]-D[i])*(double(deg)/T);
            D.swap(E);--deg;
            C.dub[r-1]=std::max(C.dub[r-1],poly_norm_ub(D));
        }
    }

    legacyMismatch=0.0;
    for(size_t s=0;s+1<C.cp.size();++s){
        const double TL=C.seg1[s]-C.seg0[s];
        const double TR=C.seg1[s+1]-C.seg0[s+1];
        for(int r=0;r<=7;++r){
            const Vec3 a=r31_endpoint_derivative(C.cp[s],TL,r,true);
            const Vec3 b=r31_endpoint_derivative(C.cp[s+1],TR,r,false);
            legacyMismatch=std::max(legacyMismatch,norm(a-b));
        }
    }
    return C;
}




struct R58NormalElim{
    bool ok=false;
    int nphys=0;
    int nred=0;
    std::vector<double>B; // z_phys = B * y_red, row-major [nphys x nred]
    double maxResidual=0.0;
};

static inline bool r58_normal_plane_basis(
        Vec3 e,Vec3&n1,Vec3&n2){
    if(!(finite(e)&&fin(norm(e))&&norm(e)>0.0))return false;
    e=e/norm(e);
    // Deterministic axis least aligned with e.
    const double ax=std::fabs(e.x),ay=std::fabs(e.y),az=std::fabs(e.z);
    Vec3 a=(ax<=ay&&ax<=az)?Vec3{1,0,0}:((ay<=az)?Vec3{0,1,0}:Vec3{0,0,1});
    n1=cross(e,a);
    const double n=norm(n1);
    if(!(fin(n)&&n>0.0))return false;
    n1=n1/n;
    n2=cross(e,n1);
    const double n_2=norm(n2);
    if(!(fin(n_2)&&n_2>0.0))return false;
    n2=n2/n_2;
    return true;
}

static inline R58NormalElim r58_build_normal_elimination(
        const std::vector<double>&K,const ActiveReference&ref){
    R58NormalElim O;
    const int N=(int)K.size();
    const int ns=r58_scalar_dofs(K);
    const int nphys=3*ns;
    if(N<2||ns<=0)return O;

    const int nInterior=std::max(0,N-2);
    const int nred=22*nInterior+3; // (P,V constrained =>4) + 6*3 higher jets; terminal P=3
    if(nred<=0||nred>nphys)return O;

    O.nphys=nphys;O.nred=nred;
    O.B.assign(nphys*nred,0.0);

    auto setB=[&](int prow,int col,double v){
        O.B[prow*nred+col]+=v;
    };
    int col=0;

    for(int node=1;node<N-1;++node){
        const auto x=ref.eval(K[node]);
        Vec3 v=x[1],a=x[2];
        const double nv=norm(v);
        if(!(finite(v)&&finite(a)&&fin(nv)&&nv>0.0))return R58NormalElim{};
        const Vec3 e=v/nv;
        const Vec3 edot=(a-e*dot(e,a))/nv;
        Vec3 n1{},n2{};
        if(!r58_normal_plane_basis(e,n1,n2))return R58NormalElim{};

        const double lambda=r58_node_scale(K,node);
        if(!(fin(lambda)&&lambda>0.0))return R58NormalElim{};

        const int qp=r58_scalar_index(node,0,N);
        const int qv=r58_scalar_index(node,1,N);
        if(qp<0||qv<0)return R58NormalElim{};

        auto add_vec=[&](int q,int c,Vec3 w){
            setB(3*q+0,c,w.x);setB(3*q+1,c,w.y);setB(3*q+2,c,w.z);
        };

        // p1: P=n1*p1 and zV has the dependent longitudinal component.
        add_vec(qp,col,n1);
        add_vec(qv,col,e*(-lambda*dot(edot,n1)));
        ++col;

        // p2
        add_vec(qp,col,n2);
        add_vec(qv,col,e*(-lambda*dot(edot,n2)));
        ++col;

        // v1,v2 are the two free normal-plane components of scaled zV=lambda*V.
        add_vec(qv,col,n1);++col;
        add_vec(qv,col,n2);++col;

        // Higher jets 2..7 are unconstrained 3-vectors.
        for(int r=2;r<=7;++r){
            const int q=r58_scalar_index(node,r,N);
            if(q<0)return R58NormalElim{};
            for(int ax=0;ax<3;++ax){
                setB(3*q+ax,col,1.0);
                ++col;
            }
        }

        // Algebraic residual of E*B for this node over all columns constructed so far.
        for(int c=0;c<col;++c){
            Vec3 P{
                O.B[(3*qp+0)*nred+c],
                O.B[(3*qp+1)*nred+c],
                O.B[(3*qp+2)*nred+c]};
            Vec3 zV{
                O.B[(3*qv+0)*nred+c],
                O.B[(3*qv+1)*nred+c],
                O.B[(3*qv+2)*nred+c]};
            const double r0=dot(e,P);
            const double r1=dot(edot,P)+dot(e,zV)/lambda;
            O.maxResidual=std::max(O.maxResidual,std::fabs(r0));
            O.maxResidual=std::max(O.maxResidual,std::fabs(r1));
        }
    }

    // Terminal free position offset only.
    const int qt=r58_scalar_index(N-1,0,N);
    if(qt<0)return R58NormalElim{};
    for(int ax=0;ax<3;++ax){
        setB(3*qt+ax,col,1.0);
        ++col;
    }

    if(col!=nred)return R58NormalElim{};
    O.ok=true;
    return O;
}

// R14: Construct a NESTED, optional low-jet C7 candidate space.
// This is a candidate FAMILY, NOT a new hard constraint on the full planner.
// Interior P and V normal-bundle coordinates remain free; jets 2..7 of the
// correction are zero at interior nodes.  The moving-normal equalities are
// still represented by the original, exact normal-elimination map B.
// No approximation or sampled acceptance is introduced.
static inline std::vector<int> r58r14_lowjet_indices(int knotCount){
    std::vector<int> I;
    if(knotCount<2)return I;
    for(int j=0;j<knotCount-2;++j){
        for(int q=0;q<4;++q)I.push_back(22*j+q);
    }
    const int tail=22*(knotCount-2);
    for(int q=0;q<3;++q)I.push_back(tail+q);
    return I;
}
static inline std::vector<double> r58r14_principal_hessian(
        const std::vector<double>&H,int fullN,
        const std::vector<int>&I){
    const int n=(int)I.size();
    if(fullN<1||(int)H.size()!=fullN*fullN||n<1)return {};
    std::vector<double>R(n*n,0.0);
    for(int a=0;a<n;++a){
        if(I[a]<0||I[a]>=fullN)return {};
        for(int b=0;b<n;++b){
            if(I[b]<0||I[b]>=fullN)return {};
            R[a*n+b]=H[I[a]*fullN+I[b]];
        }
    }
    return R;
}
static inline bool r58r14_project_rows(
        std::vector<Row>&rows,int fullN,const std::vector<int>&I){
    for(auto&row:rows){
        if((int)row.a.size()!=fullN)return false;
        std::vector<double>a(I.size());
        for(size_t j=0;j<I.size();++j){
            if(I[j]<0||I[j]>=fullN)return false;
            a[j]=row.a[I[j]];
            if(!fin(a[j]))return false;
        }
        row.a.swap(a);
    }
    return true;
}
static inline QPSol r58r14_lift_solution(
        const QPSol&q,int fullN,const std::vector<int>&I){
    if(!q.ok||fullN<1||q.z.size()!=I.size())return QPSol{};
    QPSol out=q;
    out.z.assign(fullN,0.0);
    for(size_t i=0;i<I.size();++i){
        if(I[i]<0||I[i]>=fullN||!fin(q.z[i]))return QPSol{};
        out.z[I[i]]=q.z[i];
    }
    return out;
}

static inline std::vector<double> r58_reduce_hessian(
        const std::vector<double>&H,const R58NormalElim&E){
    if(!E.ok||(int)H.size()!=E.nphys*E.nphys)return {};
    std::vector<double>T(E.nphys*E.nred,0.0);
    for(int i=0;i<E.nphys;++i)
        for(int c=0;c<E.nred;++c){
            double s=0.0;
            for(int j=0;j<E.nphys;++j)
                s+=H[i*E.nphys+j]*E.B[j*E.nred+c];
            T[i*E.nred+c]=s;
        }

    std::vector<double>R(E.nred*E.nred,0.0);
    for(int a=0;a<E.nred;++a)
        for(int b=0;b<E.nred;++b){
            double s=0.0;
            for(int i=0;i<E.nphys;++i)
                s+=E.B[i*E.nred+a]*T[i*E.nred+b];
            R[a*E.nred+b]=s;
        }
    return R;
}

// R21: project straight to the R14 nested low-jet coordinates.
// Algebraic identity: B_I^T H B_I == (B^T H B)[I,I].
// The accumulation order for each retained term matches the old full
// reduction, so no extra physical/mathematical assumption is introduced.
static inline std::vector<double> r58_reduce_hessian_selected(
        const std::vector<double>&H,const R58NormalElim&E,
        const std::vector<int>&I){
    if(!E.ok||(int)H.size()!=E.nphys*E.nphys||I.empty())return {};
    const int n=(int)I.size();
    for(int c:I)if(c<0||c>=E.nred)return {};
    std::vector<double>T(E.nphys*n,0.0);
    for(int i=0;i<E.nphys;++i)for(int c=0;c<n;++c){
        double sum=0.0;
        for(int j=0;j<E.nphys;++j)
            sum+=H[i*E.nphys+j]*E.B[j*E.nred+I[c]];
        T[i*n+c]=sum;
    }
    std::vector<double>R(n*n,0.0);
    for(int a=0;a<n;++a)for(int b=0;b<n;++b){
        double sum=0.0;
        for(int i=0;i<E.nphys;++i)
            sum+=E.B[i*E.nred+I[a]]*T[i*n+b];
        R[a*n+b]=sum;
    }
    return R;
}

// R21: direct projection of original physical corridor rows.  Identical
// constraints in the same nested variable space; no rows are discarded.
// R22: form the EXACT same selected normal-elimination Hessian without
// allocating the all-axis physical Hessian. H_phys[(3i+a),(3j+b)] is
// H_scalar[i,j] when a==b and zero otherwise. Reuse precisely the R21
// accumulation ordering (inner scalar j, then outer physical i).
// This is a computing-representation change only; no objective/constraint
// coefficient or safety-acceptance formula is intentionally changed.
static inline std::vector<double> r58r22_lowjet_hessian_direct(
        const std::vector<double>&K,double cableL,double gravity,
        const R58NormalElim&E,const std::vector<int>&I){
    const int ns=r58_scalar_dofs(K),n=3*ns,nsel=(int)I.size();
    if(!E.ok||E.nphys!=n||ns<=0||I.empty())return {};
    const auto Hs=r58r22_scalar_hessian(K,cableL,gravity);
    if((int)Hs.size()!=ns*ns)return {};
    for(int c:I)if(c<0||c>=E.nred)return {};
    std::vector<double>T(n*nsel,0.0);
    for(int i=0;i<n;++i){
        const int qi=i/3,axis=i%3;
        for(int c=0;c<nsel;++c){
            double sum=0.0;
            for(int j=0;j<ns;++j)
                sum+=Hs[qi*ns+j]*E.B[(3*j+axis)*E.nred+I[c]];
            T[i*nsel+c]=sum;
        }
    }
    std::vector<double>R(nsel*nsel,0.0);
    for(int a=0;a<nsel;++a)for(int b=0;b<nsel;++b){
        double sum=0.0;
        for(int i=0;i<n;++i)
            sum+=E.B[i*E.nred+I[a]]*T[i*nsel+b];
        R[a*nsel+b]=sum;
    }
    return R;
}

static inline bool r58_reduce_rows_selected(
        std::vector<Row>&rows,const R58NormalElim&E,
        const std::vector<int>&I){
    if(!E.ok||I.empty())return false;
    for(int c:I)if(c<0||c>=E.nred)return false;
    for(auto&row:rows){
        if((int)row.a.size()!=E.nphys)return false;
        std::vector<double>a(I.size(),0.0);
        for(size_t c=0;c<I.size();++c){
            double sum=0.0;
            for(int i=0;i<E.nphys;++i)
                sum+=row.a[i]*E.B[i*E.nred+I[c]];
            if(!fin(sum))return false;
            a[c]=sum;
        }
        row.a.swap(a);
    }
    return true;
}

static inline bool r58_reduce_rows(
        std::vector<Row>&rows,const R58NormalElim&E){
    if(!E.ok)return false;
    for(auto&R:rows){
        if((int)R.a.size()!=E.nphys)return false;
        std::vector<double>a(E.nred,0.0);
        for(int c=0;c<E.nred;++c){
            double s=0.0;
            for(int i=0;i<E.nphys;++i)
                s+=R.a[i]*E.B[i*E.nred+c];
            if(!fin(s))return false;
            a[c]=s;
        }
        R.a.swap(a);
    }
    return true;
}

static inline QPSol r58_expand_solution(
        const QPSol&q,const R58NormalElim&E){
    if(!q.ok||!E.ok||(int)q.z.size()!=E.nred)return QPSol{};
    QPSol p=q;
    p.z.assign(E.nphys,0.0);
    for(int i=0;i<E.nphys;++i){
        double s=0.0;
        for(int c=0;c<E.nred;++c)
            s+=E.B[i*E.nred+c]*q.z[c];
        if(!fin(s))return QPSol{};
        p.z[i]=s;
    }
    return p;
}

static inline bool r58_equilibrate_hessian(
        const std::vector<double>&H,int n,
        std::vector<double>&Heq,std::vector<double>&scale,
        double&rawMin,double&rawMax,double&eqMin,double&eqMax){
    if(n<=0||(int)H.size()!=n*n)return false;
    scale.assign(n,0.0);
    rawMin=std::numeric_limits<double>::infinity();
    rawMax=0.0;
    for(int i=0;i<n;++i){
        const double d=H[i*n+i];
        if(!(fin(d)&&d>0.0))return false;
        rawMin=std::min(rawMin,d);
        rawMax=std::max(rawMax,d);
        scale[i]=std::sqrt(d);
        if(!(fin(scale[i])&&scale[i]>0.0))return false;
    }

    Heq.assign(n*n,0.0);
    eqMin=std::numeric_limits<double>::infinity();
    eqMax=0.0;
    for(int i=0;i<n;++i){
        for(int j=0;j<n;++j){
            const double v=H[i*n+j]/(scale[i]*scale[j]);
            if(!fin(v))return false;
            Heq[i*n+j]=v;
        }
        eqMin=std::min(eqMin,Heq[i*n+i]);
        eqMax=std::max(eqMax,Heq[i*n+i]);
    }
    return true;
}

static inline bool r58_apply_variable_scaling_to_rows(
        std::vector<Row>&rows,const std::vector<double>&scale){
    for(auto&R:rows){
        if(R.a.size()!=scale.size())return false;
        for(size_t i=0;i<R.a.size();++i){
            if(!(fin(scale[i])&&scale[i]>0.0))return false;
            R.a[i]/=scale[i];
            if(!fin(R.a[i]))return false;
        }
    }
    return true;
}

static inline QPSol r58_unscale_solution(
        const QPSol&q,const std::vector<double>&scale){
    if(!q.ok||q.z.size()!=scale.size())return QPSol{};
    QPSol u=q;
    for(size_t i=0;i<u.z.size();++i){
        if(!(fin(scale[i])&&scale[i]>0.0))return QPSol{};
        u.z[i]/=scale[i];
        if(!fin(u.z[i]))return QPSol{};
    }
    return u;
}

static inline bool r58_equilibrate_rows(
        std::vector<Row>&rows,double&minScale,double&maxScale){
    minScale=std::numeric_limits<double>::infinity();
    maxScale=0.0;
    for(auto&R:rows){
        double s=std::fabs(R.b);
        for(double a:R.a)s=std::max(s,std::fabs(a));
        if(!(fin(s)&&s>0.0)){
            // Zero >= 0 is redundant. Any zero row with positive b is infeasible.
            if(R.b>0.0)return false;
            s=1.0;
        }
        const double inv=1.0/s;
        for(double&a:R.a)a*=inv;
        R.b*=inv;
        minScale=std::min(minScale,s);
        maxScale=std::max(maxScale,s);
    }
    return true;
}


struct R58DykstraDiag{
    bool feasible=false;
    bool kktOptimal=false;
    bool resourceFail=false;
    bool degenerateRowFail=false;
    int sweeps=0;
    int projections=0;
    double maxViolation=std::numeric_limits<double>::infinity();
    double dualityGap=std::numeric_limits<double>::infinity();
    double stationarityResidual=std::numeric_limits<double>::infinity();
};

// Same strictly-convex primal problem:
//      min 0.5 z' H z   s.t. a_i' z >= b_i.
//
// In the H-metric, projection onto one halfspace is
//
//      z <- z + alpha H^{-1}a_i,
//      alpha=(b_i-a_i'z)/(a_i'H^{-1}a_i).
//
// Dykstra correction for each halfspace is a scalar because each correction
// remains parallel to H^{-1}a_i.  No working-set Gram matrix is formed, so
// duplicate / nearly parallel corridor rows cannot create a singular active
// set.
//
// Safety authority is ALL-row primal feasibility. If the finite computational
// resource guard is reached first, fail closed. KKT/duality-gap closure is
// diagnostic quality information only.
static inline QPSol r58_solve_qp_dykstra(
        const std::vector<double>&H,int n,const std::vector<Row>&rows,
        R58DykstraDiag&diag){
    QPSol out;
    const int m=(int)rows.size();
    if(n<=0||m<1)return out;
    for(const auto&r:rows)
        if((int)r.a.size()!=n||!fin(r.b))return out;

    std::vector<double>Lc;
    if(!chol(H,n,Lc))return out;

    std::vector<std::vector<double>>X(m,std::vector<double>(n));
    std::vector<double>gii(m,0.0);
    for(int i=0;i<m;++i){
        X[i]=cholsolve(Lc,n,rows[i].a);
        double g=0.0,an=0.0;
        for(int d=0;d<n;++d){
            if(!fin(X[i][d]))return out;
            g+=rows[i].a[d]*X[i][d];
            an=std::max(an,std::fabs(rows[i].a[d]));
        }
        const double gtol=4096*epsd()*std::max(1.0,an*an);
        if(!(fin(g)&&g>gtol)){
            if(rows[i].b>4096*epsd()){
                diag.degenerateRowFail=true;
                return out;
            }
            gii[i]=0.0;
        }else{
            gii[i]=g;
        }
    }

    std::vector<double>z(n,0.0);
    std::vector<double>rho(m,0.0); // lambda_i = -rho_i >= 0 at convergence

    auto verify=[&](double&maxViol,double&minSlack)->bool{
        maxViol=0.0;
        minSlack=std::numeric_limits<double>::infinity();
        for(int i=0;i<m;++i){
            double az=0.0,asc=1.0;
            for(int d=0;d<n;++d){
                az+=rows[i].a[d]*z[d];
                asc=std::max(asc,std::fabs(rows[i].a[d]*z[d]));
            }
            if(!fin(az))return false;
            const double sl=az-rows[i].b;
            minSlack=std::min(minSlack,sl);
            const double tol=8192*epsd()*
                std::max({1.0,asc,std::fabs(az),std::fabs(rows[i].b)});
            if(sl<-tol)maxViol=std::max(maxViol,-sl);
        }
        return true;
    };

    // Computational resource guard only. It never grants acceptance.
    const int sweepMax=std::max(64,8*m+16*n+64);

    for(int sweep=1;sweep<=sweepMax;++sweep){
        int projectedThisSweep=0;

        for(int i=0;i<m;++i){
            if(gii[i]==0.0)continue;

            // x = z + rho_i H^{-1}a_i
            if(rho[i]!=0.0){
                for(int d=0;d<n;++d)z[d]+=rho[i]*X[i][d];
            }

            double az=0.0;
            for(int d=0;d<n;++d)az+=rows[i].a[d]*z[d];
            if(!fin(az))return QPSol{};

            if(az>=rows[i].b){
                rho[i]=0.0;
            }else{
                const double alpha=(rows[i].b-az)/gii[i];
                if(!(fin(alpha)&&alpha>=0.0))return QPSol{};
                for(int d=0;d<n;++d)z[d]+=alpha*X[i][d];
                rho[i]=-alpha;
                ++projectedThisSweep;
                ++diag.projections;
            }
        }

        diag.sweeps=sweep;

        double maxViol=0.0,minSlack=0.0;
        if(!verify(maxViol,minSlack))return QPSol{};
        diag.maxViolation=maxViol;

        if(maxViol==0.0){
            double qform=0.0;
            for(int i=0;i<n;++i)
                for(int j=0;j<n;++j)
                    qform+=z[i]*H[i*n+j]*z[j];

            double bLam=0.0;
            for(int i=0;i<m;++i){
                const double lam=-rho[i];
                if(lam>0.0)bLam+=lam*rows[i].b;
            }
            const double gap=qform-bLam;
            diag.dualityGap=gap;

            double sres=0.0;
            for(int d=0;d<n;++d){
                double hz=0.0,atl=0.0;
                for(int j=0;j<n;++j)hz+=H[d*n+j]*z[j];
                for(int i=0;i<m;++i){
                    const double lam=-rho[i];
                    if(lam>0.0)atl+=rows[i].a[d]*lam;
                }
                sres=std::max(sres,std::fabs(hz-atl));
            }
            diag.stationarityResidual=sres;

            const double gtol=16384*epsd()*
                std::max({1.0,std::fabs(qform),std::fabs(bLam)});
            const double stol=16384*epsd()*
                std::max(1.0,std::sqrt(std::max(0.0,qform)));
            diag.kktOptimal=
                fin(gap)&&std::fabs(gap)<=gtol&&
                fin(sres)&&sres<=stol;

            const double c=0.5*qform;
            if(!fin(c)||!fin(minSlack))return QPSol{};
            out.ok=true;
            out.z=z;
            out.cost=c;
            out.slack=minSlack;
            diag.feasible=true;
            return out;
        }

        if(projectedThisSweep==0){
            diag.resourceFail=true;
            return QPSol{};
        }
    }

    diag.resourceFail=true;
    return QPSol{};
}


struct R58HildrethDiag{
    bool kktConverged=false;
    bool resourceFail=false;
    int sweeps=0;
    int updates=0;
    double maxPrimalViolation=std::numeric_limits<double>::infinity();
    double maxComplementarity=std::numeric_limits<double>::infinity();
    double dualityGap=std::numeric_limits<double>::infinity();
    double stationarityResidual=std::numeric_limits<double>::infinity();
    double maxLambda=0.0;
};

static inline double r58_gamma_k(int k){
    const double u=epsd();
    const double ku=double(std::max(1,k))*u;
    if(!(fin(ku)&&ku<0.25))
        return std::numeric_limits<double>::infinity();
    return ku/(1.0-ku);
}

// Hildreth dual coordinate ascent for exactly the same strictly-convex QP:
//
//   min 0.5 z'H z,  a_i'z >= b_i.
//
// Dual:
//   max_{lambda>=0} b'lambda - 0.5 lambda'G lambda,
//   G=A H^{-1} A'.
//
// Coordinate update:
//   lambda_i^+ = max(0, lambda_i + (b_i-a_i'z)/G_ii),
//   z^+ = z + (lambda_i^+-lambda_i) H^{-1}a_i.
//
// Duplicate/parallel rows are harmless. R58R6 does NOT stop at mere primal
// feasibility; it returns only after numerical KKT closure. Resource
// exhaustion is fail-closed.
static inline QPSol r58_solve_qp_hildreth(
        const std::vector<double>&H,int n,const std::vector<Row>&rows,
        R58HildrethDiag&diag){
    QPSol out;
    const int m=(int)rows.size();
    if(n<=0||m<1)return out;
    for(const auto&r:rows)
        if((int)r.a.size()!=n||!fin(r.b))return out;

    std::vector<double>Lc;
    if(!chol(H,n,Lc))return out;

    std::vector<std::vector<double>>X(m,std::vector<double>(n));
    std::vector<double>gii(m,0.0);
    for(int i=0;i<m;++i){
        X[i]=cholsolve(Lc,n,rows[i].a);
        double g=0.0,an=0.0;
        for(int d=0;d<n;++d){
            if(!fin(X[i][d]))return out;
            g+=rows[i].a[d]*X[i][d];
            an=std::max(an,std::fabs(rows[i].a[d]));
        }
        const double gtol=
            16.0*r58_gamma_k(4*n+8)*std::max(1.0,an*an);
        if(!(fin(g)&&g>gtol)){
            if(rows[i].b>
               16.0*r58_gamma_k(8)*std::max(1.0,std::fabs(rows[i].b)))
                return out;
            gii[i]=0.0;
        }else gii[i]=g;
    }

    std::vector<double>z(n,0.0),lam(m,0.0);

    auto kkt_check=[&](double&minSlack,double&cost)->bool{
        diag.maxPrimalViolation=0.0;
        diag.maxComplementarity=0.0;
        diag.dualityGap=0.0;
        diag.stationarityResidual=0.0;
        diag.maxLambda=0.0;
        minSlack=std::numeric_limits<double>::infinity();

        double bLam=0.0;
        for(int i=0;i<m;++i){
            double az=0.0,absDot=0.0;
            for(int d=0;d<n;++d){
                az+=rows[i].a[d]*z[d];
                absDot+=std::fabs(rows[i].a[d]*z[d]);
            }
            if(!fin(az))return false;
            const double s=az-rows[i].b;
            minSlack=std::min(minSlack,s);

            const double ptol=
                16.0*r58_gamma_k(2*n+8)*
                std::max({1.0,absDot,std::fabs(az),std::fabs(rows[i].b)});
            if(s<-ptol)
                diag.maxPrimalViolation=
                    std::max(diag.maxPrimalViolation,-s);

            diag.maxComplementarity=
                std::max(diag.maxComplementarity,lam[i]*std::fabs(s));
            bLam+=lam[i]*rows[i].b;
            diag.maxLambda=std::max(diag.maxLambda,lam[i]);
        }

        double qform=0.0;
        for(int i=0;i<n;++i)
            for(int j=0;j<n;++j)
                qform+=z[i]*H[i*n+j]*z[j];
        if(!fin(qform)||!fin(bLam))return false;
        cost=0.5*qform;
        diag.dualityGap=qform-bLam;

        double statScale=1.0;
        for(int d=0;d<n;++d){
            double hz=0.0,atl=0.0;
            for(int j=0;j<n;++j)hz+=H[d*n+j]*z[j];
            for(int i=0;i<m;++i)atl+=rows[i].a[d]*lam[i];
            diag.stationarityResidual=
                std::max(diag.stationarityResidual,std::fabs(hz-atl));
            statScale=std::max(statScale,std::fabs(hz)+std::fabs(atl));
        }

        const double gapScale=
            std::max({1.0,std::fabs(qform),std::fabs(bLam)});
        const double gapTol=
            32.0*r58_gamma_k(4*n*n+4*m*n+16)*gapScale;
        const double compTol=
            32.0*r58_gamma_k(4*n+16)*
            std::max(1.0,diag.maxLambda)*
            std::max(1.0,std::sqrt(std::max(0.0,qform)));
        const double statTol=
            32.0*r58_gamma_k(4*m+4*n+16)*statScale;

        return
            diag.maxPrimalViolation==0.0 &&
            fin(diag.dualityGap) &&
            std::fabs(diag.dualityGap)<=gapTol &&
            diag.maxComplementarity<=compTol &&
            diag.stationarityResidual<=statTol;
    };

    // Computational guard only; reaching it never grants acceptance.
    const int sweepMax=std::max(256,2*m+8*n+64);

    for(int sweep=1;sweep<=sweepMax;++sweep){
        int updatesThisSweep=0;

        for(int i=0;i<m;++i){
            if(gii[i]==0.0)continue;

            double az=0.0;
            for(int d=0;d<n;++d)az+=rows[i].a[d]*z[d];
            if(!fin(az))return QPSol{};

            const double trial=lam[i]+(rows[i].b-az)/gii[i];
            if(!fin(trial))return QPSol{};
            const double next=std::max(0.0,trial);
            const double delta=next-lam[i];

            if(delta!=0.0){
                for(int d=0;d<n;++d)z[d]+=delta*X[i][d];
                lam[i]=next;
                ++updatesThisSweep;
                ++diag.updates;
            }
        }

        diag.sweeps=sweep;
        double minSlack=0.0,cost=0.0;
        if(kkt_check(minSlack,cost)){
            out.ok=true;
            out.z=z;
            out.cost=cost;
            out.slack=minSlack;
            diag.kktConverged=true;
            return out;
        }

        if(updatesThisSweep==0){
            diag.resourceFail=true;
            return QPSol{};
        }
    }

    diag.resourceFail=true;
    return QPSol{};
}



struct R58IPMDiag{
    bool converged=false;
    bool resourceFail=false;
    bool factorFail=false;
    bool pcgResourceFail=false;
    int iterations=0;
    int newtonSolves=0;
    int pcgMaxIterations=0;
    int failureStage=0;
    int failurePCGIterations=0;
    double pcgFinalRelResidual=std::numeric_limits<double>::infinity();
    double failurePCGFinalRelResidual=std::numeric_limits<double>::infinity();
    double failurePCGRhsInf=std::numeric_limits<double>::quiet_NaN();
    double failureDMin=std::numeric_limits<double>::quiet_NaN();
    double failureDMax=std::numeric_limits<double>::quiet_NaN();
    double failureDRatio=std::numeric_limits<double>::quiet_NaN();
    double pcgMs=0.0;

    int predictorPCGIterations=0;
    int correctorPCGIterations=0;
    double predictorPCGRelResidual=std::numeric_limits<double>::quiet_NaN();
    double correctorPCGRelResidual=std::numeric_limits<double>::quiet_NaN();

    double lastSigma=std::numeric_limits<double>::quiet_NaN();
    double lastAlphaPrimal=std::numeric_limits<double>::quiet_NaN();
    double lastAlphaDual=std::numeric_limits<double>::quiet_NaN();
    double minAlphaPrimal=1.0;
    double minAlphaDual=1.0;
    double lastMuRatio=std::numeric_limits<double>::quiet_NaN();
    double kktTolerance=std::numeric_limits<double>::quiet_NaN();
    double gapTolerance=std::numeric_limits<double>::quiet_NaN();
    double currentDMin=std::numeric_limits<double>::quiet_NaN();
    double currentDMax=std::numeric_limits<double>::quiet_NaN();
    double currentDRatio=std::numeric_limits<double>::quiet_NaN();

    bool augmentedKKTSolveFail=false;
    int augmentedKKTMaxDim=0;
    int augmentedKKTNewtonSolves=0;
    double augmentedKKTMaxRelResidual=0.0;
    double commonStepMin=1.0;
    double commonStepLast=std::numeric_limits<double>::quiet_NaN();

    double primalResidual=std::numeric_limits<double>::infinity();
    double dualResidual=std::numeric_limits<double>::infinity();
    double complementarity=std::numeric_limits<double>::infinity();
    double dualityGap=std::numeric_limits<double>::infinity();
    double minSlack=std::numeric_limits<double>::quiet_NaN();
    double minLambda=std::numeric_limits<double>::quiet_NaN();
};

static inline double r58_inf_norm(const std::vector<double>&x){
    double v=0.0;
    for(double a:x){
        if(!fin(a))return std::numeric_limits<double>::infinity();
        v=std::max(v,std::fabs(a));
    }
    return v;
}

static inline double r58_max_step_positive(
        const std::vector<double>&x,const std::vector<double>&dx){
    double a=1.0;
    for(size_t i=0;i<x.size();++i){
        if(dx[i]<0.0){
            const double ai=-x[i]/dx[i];
            if(fin(ai))a=std::min(a,ai);
        }
    }
    return std::max(0.0,std::min(1.0,a));
}

// Infeasible-start primal-dual Mehrotra method for
//
//   min 0.5 z' H z
//   s.t. A z - s = b,  s >= 0.
//
// KKT:
//   H z - A'lambda = 0,
//   A z - s - b    = 0,
//   S lambda       = 0,
//   s,lambda >= 0.
//
// Eliminating ds and dlambda yields the SPD Newton matrix
//
//   M = H + A' diag(lambda./s) A,
//
// so linearly dependent / nearly parallel rows cannot create a singular
// working-set Gram matrix.  Predictor-corrector centering uses Mehrotra's
// sigma=(mu_aff/mu)^3.  The fraction-to-boundary factor is derived only from
// binary64 precision: eta=1-sqrt(eps).  Acceptance requires backward-error
// KKT closure; resource exhaustion fails closed.

static inline bool r58_mehrotra_pcg(
        const std::vector<double>&H,const std::vector<double>&Lh,int n,
        const std::vector<Row>&rows,const std::vector<double>&D,
        const std::vector<double>&rhs,std::vector<double>&x,
        int&iters,double&relResidual){
    const int m=(int)rows.size();
    if(n<=0||(int)H.size()!=n*n||(int)Lh.size()!=n*n||
       (int)D.size()!=m||(int)rhs.size()!=n)return false;

    auto matvec=[&](const std::vector<double>&v,std::vector<double>&y)->bool{
        y.assign(n,0.0);
        for(int i=0;i<n;++i){
            double s=0.0;
            for(int j=0;j<n;++j)s+=H[i*n+j]*v[j];
            y[i]=s;
        }
        for(int k=0;k<m;++k){
            if(!(fin(D[k])&&D[k]>=0.0))return false;
            double av=0.0;
            for(int d=0;d<n;++d)av+=rows[k].a[d]*v[d];
            const double w=D[k]*av;
            if(!fin(w))return false;
            for(int d=0;d<n;++d)y[d]+=rows[k].a[d]*w;
        }
        for(double q:y)if(!fin(q))return false;
        return true;
    };

    auto dotv=[&](const std::vector<double>&a,const std::vector<double>&b){
        double s=0.0;
        for(int i=0;i<n;++i)s+=a[i]*b[i];
        return s;
    };

    x.assign(n,0.0);
    std::vector<double>r=rhs;
    std::vector<double>z=cholsolve(Lh,n,r);
    if((int)z.size()!=n)return false;
    std::vector<double>p=z;
    double rz=dotv(r,z);
    if(!(fin(rz)&&rz>=0.0))return false;

    const double rhsn=std::max(1.0,r58_inf_norm(rhs));
    relResidual=r58_inf_norm(r)/rhsn;
    const double tol=
        64.0*r58_gamma_k(8*n*n+8*m*n+64);
    if(relResidual<=tol){iters=0;return true;}

    // In exact arithmetic SPD-CG terminates in <= n iterations.  The factor
    // four is a floating-point resource allowance only; it never grants
    // acceptance because residual closure is checked explicitly.
    const int maxIt=std::max(8,4*n);

    for(int k=1;k<=maxIt;++k){
        std::vector<double>Mp;
        if(!matvec(p,Mp))return false;
        const double den=dotv(p,Mp);
        if(!(fin(den)&&den>0.0))return false;
        const double alpha=rz/den;
        if(!fin(alpha))return false;

        for(int i=0;i<n;++i){
            x[i]+=alpha*p[i];
            r[i]-=alpha*Mp[i];
        }
        relResidual=r58_inf_norm(r)/
            std::max(1.0,rhsn+r58_inf_norm(x));
        iters=k;
        if(relResidual<=tol)return true;

        z=cholsolve(Lh,n,r);
        if((int)z.size()!=n)return false;
        const double rzNew=dotv(r,z);
        if(!(fin(rzNew)&&rzNew>=0.0))return false;
        if(rz==0.0)return false;
        const double beta=rzNew/rz;
        if(!fin(beta))return false;
        for(int i=0;i<n;++i)p[i]=z[i]+beta*p[i];
        rz=rzNew;
    }
    return false;
}



static inline bool r58_dense_complete_pivot_solve(
        const std::vector<double>&A0,const std::vector<double>&b0,int n,
        std::vector<double>&x,double&relResidual){
    if(n<=0||(int)A0.size()!=n*n||(int)b0.size()!=n)return false;
    std::vector<double>A=A0,b=b0;
    std::vector<double>cscale(n,1.0);
    std::vector<int>perm(n);
    for(int j=0;j<n;++j)perm[j]=j;

    // Positive row scaling.
    for(int i=0;i<n;++i){
        double sc=std::fabs(b[i]);
        for(int j=0;j<n;++j)sc=std::max(sc,std::fabs(A[i*n+j]));
        sc=std::max(1.0,sc);
        for(int j=0;j<n;++j)A[i*n+j]/=sc;
        b[i]/=sc;
    }
    // Invertible column-variable scaling: x_j=y_j/c_j.
    for(int j=0;j<n;++j){
        double sc=0.0;
        for(int i=0;i<n;++i)sc=std::max(sc,std::fabs(A[i*n+j]));
        if(!(fin(sc)&&sc>0.0))return false;
        cscale[j]=sc;
        for(int i=0;i<n;++i)A[i*n+j]/=sc;
    }

    const double ptol=64.0*r58_gamma_k(8*n*n*n+64);

    for(int k=0;k<n;++k){
        int pi=k,pj=k;
        double pv=0.0;
        for(int i=k;i<n;++i)for(int j=k;j<n;++j){
            double v=std::fabs(A[i*n+j]);
            if(v>pv){pv=v;pi=i;pj=j;}
        }
        if(!(fin(pv)&&pv>ptol))return false;
        if(pi!=k){
            for(int j=0;j<n;++j)std::swap(A[k*n+j],A[pi*n+j]);
            std::swap(b[k],b[pi]);
        }
        if(pj!=k){
            for(int i=0;i<n;++i)std::swap(A[i*n+k],A[i*n+pj]);
            std::swap(perm[k],perm[pj]);
        }
        const double piv=A[k*n+k];
        if(!(fin(piv)&&std::fabs(piv)>ptol))return false;
        for(int i=k+1;i<n;++i){
            const double f=A[i*n+k]/piv;
            A[i*n+k]=0.0;
            for(int j=k+1;j<n;++j)A[i*n+j]-=f*A[k*n+j];
            b[i]-=f*b[k];
            if(!fin(b[i]))return false;
        }
    }

    std::vector<double>yp(n,0.0);
    for(int i=n-1;i>=0;--i){
        double s=b[i];
        for(int j=i+1;j<n;++j)s-=A[i*n+j]*yp[j];
        const double piv=A[i*n+i];
        if(!(fin(piv)&&std::fabs(piv)>ptol))return false;
        yp[i]=s/piv;
        if(!fin(yp[i]))return false;
    }
    std::vector<double>y(n,0.0);
    for(int posi=0;posi<n;++posi)y[perm[posi]]=yp[posi];
    x.assign(n,0.0);
    for(int j=0;j<n;++j){
        x[j]=y[j]/cscale[j];
        if(!fin(x[j]))return false;
    }

    double rn=0.0,bn=0.0,xn=0.0;
    for(int i=0;i<n;++i){
        double ax=0.0;
        for(int j=0;j<n;++j)ax+=A0[i*n+j]*x[j];
        rn=std::max(rn,std::fabs(ax-b0[i]));
        bn=std::max(bn,std::fabs(b0[i]));
    }
    for(double v:x)xn=std::max(xn,std::fabs(v));
    relResidual=rn/std::max(1.0,bn+xn);
    return fin(relResidual) &&
        relResidual<=256.0*r58_gamma_k(12*n*n*n+128);
}

static inline QPSol r58_solve_qp_mehrotra(
        const std::vector<double>&H,int n,const std::vector<Row>&rows,
        R58IPMDiag&diag){
    QPSol out;
    const int m=(int)rows.size();
    if(n<=0||m<1||(int)H.size()!=n*n)return out;
    for(const auto&r:rows)
        if((int)r.a.size()!=n||!fin(r.b))return out;

    std::vector<double>Lh;
    if(!chol(H,n,Lh)){
        diag.factorFail=true;
        return out;
    }

    std::vector<double>z(n,0.0),s(m,1.0),lam(m,1.0);
    // Scale-free positive initialization after row equilibration.
    for(int i=0;i<m;++i)
        s[i]=std::max(1.0,std::fabs(rows[i].b));

    auto residuals=[&](
            std::vector<double>&rd,std::vector<double>&rp,
            double&mu,double&pr,double&dr,double&gap,
            double&minS,double&minL,double&cost)->bool{
        rd.assign(n,0.0);
        rp.assign(m,0.0);

        std::vector<double>hz(n,0.0),atl(n,0.0);
        for(int i=0;i<n;++i){
            double v=0.0;
            for(int j=0;j<n;++j)v+=H[i*n+j]*z[j];
            hz[i]=v;
        }
        for(int i=0;i<m;++i){
            for(int d=0;d<n;++d)
                atl[d]+=rows[i].a[d]*lam[i];
        }
        for(int d=0;d<n;++d)rd[d]=hz[d]-atl[d];

        double pscale=1.0;
        for(int i=0;i<m;++i){
            double az=0.0,absDot=0.0;
            for(int d=0;d<n;++d){
                az+=rows[i].a[d]*z[d];
                absDot+=std::fabs(rows[i].a[d]*z[d]);
            }
            rp[i]=az-s[i]-rows[i].b;
            pscale=std::max(
                pscale,std::max({absDot,std::fabs(az),
                                 std::fabs(s[i]),std::fabs(rows[i].b)}));
        }

        const double dscale=
            std::max(1.0,std::max(r58_inf_norm(hz),r58_inf_norm(atl)));
        pr=r58_inf_norm(rp)/pscale;
        dr=r58_inf_norm(rd)/dscale;

        mu=0.0;minS=std::numeric_limits<double>::infinity();
        minL=std::numeric_limits<double>::infinity();
        for(int i=0;i<m;++i){
            if(!(fin(s[i])&&fin(lam[i])&&s[i]>0.0&&lam[i]>0.0))
                return false;
            mu+=s[i]*lam[i];
            minS=std::min(minS,s[i]);
            minL=std::min(minL,lam[i]);
        }
        mu/=double(m);

        double q=0.0,bLam=0.0;
        for(int i=0;i<n;++i)q+=z[i]*hz[i];
        for(int i=0;i<m;++i)bLam+=rows[i].b*lam[i];
        cost=0.5*q;
        gap=std::fabs(q-bLam);

        return fin(pr)&&fin(dr)&&fin(mu)&&fin(gap)&&fin(cost);
    };

    auto newton=[&](
            const std::vector<double>&rd,
            const std::vector<double>&rp,
            const std::vector<double>&rc,
            std::vector<double>&dz,
            std::vector<double>&ds,
            std::vector<double>&dl,
            int stage)->bool{
        const int N=n+m;
        std::vector<double>K(N*N,0.0),br(N,0.0);

        for(int i=0;i<n;++i){
            br[i]=-rd[i];
            for(int j=0;j<n;++j)K[i*N+j]=H[i*n+j];
            for(int q=0;q<m;++q)K[i*N+(n+q)]=-rows[q].a[i];
        }

        double Dmin=std::numeric_limits<double>::infinity();
        double Dmax=0.0;
        for(int q=0;q<m;++q){
            if(!(s[q]>0.0&&lam[q]>0.0&&fin(rc[q])&&fin(rp[q])))
                return false;
            for(int j=0;j<n;++j)
                K[(n+q)*N+j]=lam[q]*rows[q].a[j];
            K[(n+q)*N+(n+q)]=s[q];
            br[n+q]=-rc[q]-lam[q]*rp[q];

            const double Di=lam[q]/s[q];
            if(Di>0.0)Dmin=std::min(Dmin,Di);
            Dmax=std::max(Dmax,Di);
        }
        diag.currentDMin=fin(Dmin)?Dmin:0.0;
        diag.currentDMax=Dmax;
        diag.currentDRatio=(fin(Dmin)&&Dmin>0.0)?
            Dmax/Dmin:std::numeric_limits<double>::infinity();

        std::vector<double>sol;
        double kres=std::numeric_limits<double>::infinity();
        if(!r58_dense_complete_pivot_solve(K,br,N,sol,kres)){
            diag.augmentedKKTSolveFail=true;
            diag.failureStage=stage;
            diag.failurePCGFinalRelResidual=kres;
            diag.failurePCGRhsInf=r58_inf_norm(br);
            diag.failureDMin=diag.currentDMin;
            diag.failureDMax=diag.currentDMax;
            diag.failureDRatio=diag.currentDRatio;
            return false;
        }
        ++diag.augmentedKKTNewtonSolves;
        diag.augmentedKKTMaxRelResidual=
            std::max(diag.augmentedKKTMaxRelResidual,kres);
        diag.augmentedKKTMaxDim=std::max(diag.augmentedKKTMaxDim,m);

        dz.assign(sol.begin(),sol.begin()+n);
        dl.assign(sol.begin()+n,sol.end());
        ds.assign(m,0.0);
        for(int q=0;q<m;++q){
            double adz=0.0;
            for(int d=0;d<n;++d)adz+=rows[q].a[d]*dz[d];
            ds[q]=adz+rp[q];
            if(!fin(ds[q])||!fin(dl[q]))return false;
        }
        if(stage==1){
            diag.predictorPCGIterations=0;
            diag.predictorPCGRelResidual=kres;
        }else{
            diag.correctorPCGIterations=0;
            diag.correctorPCGRelResidual=kres;
        }
        ++diag.newtonSolves;
        return true;
    };

    const int iterMax=std::max(64,2*(n+m)+16);

    std::vector<double>rd,rp;
    double mu=0.0,pr=0.0,dr=0.0,gap=0.0,minS=0.0,minL=0.0,cost=0.0;
    double prevMu=std::numeric_limits<double>::quiet_NaN();

    for(int it=1;it<=iterMax;++it){
        diag.iterations=it;
        if(!residuals(rd,rp,mu,pr,dr,gap,minS,minL,cost))
            return QPSol{};

        diag.primalResidual=pr;
        diag.dualResidual=dr;
        diag.complementarity=mu;
        diag.dualityGap=gap;
        diag.minSlack=minS;
        diag.minLambda=minL;

        const double tol=
            64.0*r58_gamma_k(8*n*n+8*m*n+16*m+64);
        const double gapScale=
            std::max(1.0,2.0*std::fabs(cost)+gap);
        diag.kktTolerance=tol;
        diag.gapTolerance=tol*gapScale;
        if(fin(prevMu)&&prevMu>0.0)
            diag.lastMuRatio=mu/prevMu;
        prevMu=mu;

        if(pr<=tol && dr<=tol && mu<=tol*gapScale && gap<=tol*gapScale){
            out.ok=true;
            out.z=z;
            out.cost=cost;

            double minSlack=std::numeric_limits<double>::infinity();
            for(int i=0;i<m;++i){
                double az=0.0;
                for(int d=0;d<n;++d)az+=rows[i].a[d]*z[d];
                minSlack=std::min(minSlack,az-rows[i].b);
            }
            out.slack=minSlack;
            diag.converged=true;
            return out;
        }

        // Affine predictor: rc = S*lambda.
        std::vector<double>rcAff(m,0.0);
        for(int i=0;i<m;++i)rcAff[i]=s[i]*lam[i];

        std::vector<double>dzAff,dsAff,dlAff;
        if(!newton(rd,rp,rcAff,dzAff,dsAff,dlAff,1))
            return QPSol{};

        const double apAff=r58_max_step_positive(s,dsAff);
        const double adAff=r58_max_step_positive(lam,dlAff);

        double muAff=0.0;
        for(int i=0;i<m;++i)
            muAff+=(s[i]+apAff*dsAff[i])*
                   (lam[i]+adAff*dlAff[i]);
        muAff/=double(m);

        double sigma=0.0;
        if(mu>0.0&&fin(muAff)){
            const double r=std::max(0.0,std::min(1.0,muAff/mu));
            sigma=r*r*r;
        }
        diag.lastSigma=sigma;

        // Predictor-corrector with second-order affine term.
        std::vector<double>rc(m,0.0);
        for(int i=0;i<m;++i)
            rc[i]=s[i]*lam[i]+dsAff[i]*dlAff[i]-sigma*mu;

        std::vector<double>dz,ds,dl;
        if(!newton(rd,rp,rc,dz,ds,dl,2))
            return QPSol{};

        // Largest representable common step that remains strictly inside
        // every positivity boundary. Each limiting ratio is moved by exactly
        // one binary64 value toward zero before the minimum is taken.
        double alpha=1.0;
        for(int i=0;i<m;++i){
            if(ds[i]<0.0){
                const double r=-s[i]/ds[i];
                if(!(fin(r)&&r>0.0))return QPSol{};
                alpha=std::min(alpha,std::nextafter(r,0.0));
            }
            if(dl[i]<0.0){
                const double r=-lam[i]/dl[i];
                if(!(fin(r)&&r>0.0))return QPSol{};
                alpha=std::min(alpha,std::nextafter(r,0.0));
            }
        }
        if(!(fin(alpha)&&alpha>0.0)){
            diag.resourceFail=true;
            return QPSol{};
        }

        diag.lastAlphaPrimal=alpha;
        diag.lastAlphaDual=alpha;
        diag.minAlphaPrimal=std::min(diag.minAlphaPrimal,alpha);
        diag.minAlphaDual=std::min(diag.minAlphaDual,alpha);
        diag.commonStepLast=alpha;
        diag.commonStepMin=std::min(diag.commonStepMin,alpha);

        for(int d=0;d<n;++d)z[d]=std::fma(alpha,dz[d],z[d]);
        for(int i=0;i<m;++i){
            s[i]=std::fma(alpha,ds[i],s[i]);
            lam[i]=std::fma(alpha,dl[i],lam[i]);
            if(!(s[i]>0.0&&lam[i]>0.0&&fin(s[i])&&fin(lam[i])))
                return QPSol{};
        }
    }

    diag.resourceFail=true;
    if(diag.failureStage==0)diag.failureStage=3;
    double DminEnd=std::numeric_limits<double>::infinity();
    double DmaxEnd=0.0;
    for(int i=0;i<m;++i){
        if(s[i]>0.0&&lam[i]>=0.0&&fin(s[i])&&fin(lam[i])){
            const double Di=lam[i]/s[i];
            if(Di>0.0)DminEnd=std::min(DminEnd,Di);
            DmaxEnd=std::max(DmaxEnd,Di);
        }
    }
    diag.currentDMin=fin(DminEnd)?DminEnd:0.0;
    diag.currentDMax=DmaxEnd;
    diag.currentDRatio=
        (fin(DminEnd)&&DminEnd>0.0)?DmaxEnd/DminEnd:
        std::numeric_limits<double>::infinity();
    return QPSol{};
}

struct R58CGDiag{
    bool fullOptimalityClosed=false;
    bool subsetResourceFail=false;
    bool duplicateAddFail=false;
    int fullRowCount=0;
    int outerIterations=0;
    int fullScans=0;
    int rowsAdded=0;
    int activeRowsMax=0;
    int subsetKKTSolves=0;
    int subsetKKTConverged=0;
    int subsetSweepsMax=0;
    int coordinateUpdates=0;
    int ipmConverged=0;
    int ipmResourceFail=0;
    int ipmFactorFail=0;
    int ipmIterationsMax=0;
    int ipmNewtonSolves=0;
    int ipmPCGResourceFail=0;
    int ipmPCGMaxIterations=0;
    double ipmPCGFinalRelResidual=std::numeric_limits<double>::infinity();
    double ipmPrimalResidual=std::numeric_limits<double>::infinity();
    double ipmDualResidual=std::numeric_limits<double>::infinity();
    double ipmComplementarity=std::numeric_limits<double>::infinity();
    double ipmDualityGap=std::numeric_limits<double>::infinity();
    double ipmMinSlack=std::numeric_limits<double>::quiet_NaN();
    double ipmMinLambda=std::numeric_limits<double>::quiet_NaN();

    int failureOuterIteration=0;
    int failureActiveRows=0;
    int failureAddedRow=-1;
    int failureIPMIteration=0;
    int failureStage=0;
    int failurePCGIterations=0;
    double failurePCGFinalRelResidual=std::numeric_limits<double>::infinity();
    double failurePCGRhsInf=std::numeric_limits<double>::quiet_NaN();
    double failureDMin=std::numeric_limits<double>::quiet_NaN();
    double failureDMax=std::numeric_limits<double>::quiet_NaN();
    double failureDRatio=std::numeric_limits<double>::quiet_NaN();
    double failureMinSlack=std::numeric_limits<double>::quiet_NaN();
    double failureMinLambda=std::numeric_limits<double>::quiet_NaN();
    double failureMetricMaxAbsCorr=0.0;
    double failurePrimalResidual=std::numeric_limits<double>::quiet_NaN();
    double failureDualResidual=std::numeric_limits<double>::quiet_NaN();
    double failureMu=std::numeric_limits<double>::quiet_NaN();
    double failureDualityGap=std::numeric_limits<double>::quiet_NaN();
    double failureKKTTol=std::numeric_limits<double>::quiet_NaN();
    double failureGapTol=std::numeric_limits<double>::quiet_NaN();
    double failureLastSigma=std::numeric_limits<double>::quiet_NaN();
    double failureLastAlphaPrimal=std::numeric_limits<double>::quiet_NaN();
    double failureLastAlphaDual=std::numeric_limits<double>::quiet_NaN();
    double failureMinAlphaPrimal=std::numeric_limits<double>::quiet_NaN();
    double failureMinAlphaDual=std::numeric_limits<double>::quiet_NaN();
    double failureMuRatio=std::numeric_limits<double>::quiet_NaN();
    int failurePredictorPCGIterations=0;
    int failureCorrectorPCGIterations=0;
    double failurePredictorPCGRelResidual=std::numeric_limits<double>::quiet_NaN();
    double failureCorrectorPCGRelResidual=std::numeric_limits<double>::quiet_NaN();

    double fullScanMs=0.0;
    double subsetSolverMs=0.0;
    double pcgMs=0.0;

    int augmentedKKTSolveFail=0;
    int augmentedKKTMaxDim=0;
    int augmentedKKTNewtonSolves=0;
    double augmentedKKTMaxRelResidual=0.0;
    double commonStepMin=1.0;
    double commonStepLast=std::numeric_limits<double>::quiet_NaN();

    double initialWorstViolation=0.0;
    double finalMaxViolation=std::numeric_limits<double>::infinity();
    double finalDualityGap=std::numeric_limits<double>::infinity();
    double finalComplementarity=std::numeric_limits<double>::infinity();
    double finalStationarity=std::numeric_limits<double>::infinity();
};

static inline QPSol r58_solve_qp_constraint_generation(
        const std::vector<double>&H,int n,const std::vector<Row>&rows,
        R58CGDiag&diag){
    QPSol out;
    const int m=(int)rows.size();
    diag.fullRowCount=m;
    if(n<=0||m<1||(int)H.size()!=n*n)return out;
    for(const auto&r:rows)
        if((int)r.a.size()!=n||!fin(r.b))return out;

    std::vector<double>Lc;
    if(!chol(H,n,Lc))return out;

    // Precompute H^{-1}a_i and metric diagonal once for the full row set.
    std::vector<std::vector<double>>X(m,std::vector<double>(n));
    std::vector<double>gii(m,0.0);
    for(int i=0;i<m;++i){
        X[i]=cholsolve(Lc,n,rows[i].a);
        double g=0.0,an=0.0;
        for(int d=0;d<n;++d){
            if(!fin(X[i][d]))return out;
            g+=rows[i].a[d]*X[i][d];
            an=std::max(an,std::fabs(rows[i].a[d]));
        }
        const double gtol=
            16.0*r58_gamma_k(4*n+8)*std::max(1.0,an*an);
        if(!(fin(g)&&g>gtol)){
            // SPD H implies a nonzero row has positive H^{-1}-metric norm.
            // Numerically zero row: positive rhs is genuinely infeasible,
            // otherwise it is redundant.
            if(rows[i].b>
               16.0*r58_gamma_k(8)*std::max(1.0,std::fabs(rows[i].b)))
                return out;
            gii[i]=0.0;
        }else gii[i]=g;
    }

    std::vector<double>z(n,0.0),lambda(m,0.0);
    std::vector<unsigned char>active(m,0);
    std::vector<int>activeList;
    activeList.reserve(std::min(m,n+8));

    auto active_metric_max_abs_corr=[&]()->double{
        double cmax=0.0;
        const int ma=(int)activeList.size();
        for(int p=0;p<ma;++p){
            const int i=activeList[p];
            if(!(gii[i]>0.0))continue;
            for(int q=p+1;q<ma;++q){
                const int j=activeList[q];
                if(!(gii[j]>0.0))continue;
                double gij=0.0;
                for(int d=0;d<n;++d)
                    gij+=rows[i].a[d]*X[j][d];
                const double den=std::sqrt(gii[i]*gii[j]);
                if(!(fin(den)&&den>0.0))continue;
                const double rho=std::fabs(gij/den);
                if(fin(rho))cmax=std::max(cmax,std::min(1.0,rho));
            }
        }
        return cmax;
    };

    auto full_scan=[&](int&worstRow,double&maxViolation,
                       double&minSlack)->bool{
        const auto Tscan=std::chrono::steady_clock::now();
        ++diag.fullScans;
        worstRow=-1;
        maxViolation=0.0;
        minSlack=std::numeric_limits<double>::infinity();
        double bestExcess=0.0;

        for(int i=0;i<m;++i){
            double az=0.0,absDot=0.0;
            for(int d=0;d<n;++d){
                az+=rows[i].a[d]*z[d];
                absDot+=std::fabs(rows[i].a[d]*z[d]);
            }
            if(!fin(az)){
                diag.fullScanMs+=std::chrono::duration<double,std::milli>(
                    std::chrono::steady_clock::now()-Tscan).count();
                return false;
            }

            const double slack=az-rows[i].b;
            minSlack=std::min(minSlack,slack);

            const double tol=
                16.0*r58_gamma_k(2*n+8)*
                std::max({1.0,absDot,std::fabs(az),std::fabs(rows[i].b)});
            const double excess=-slack-tol;
            if(excess>0.0){
                maxViolation=std::max(maxViolation,-slack);
                if(worstRow<0||excess>bestExcess){
                    worstRow=i;
                    bestExcess=excess;
                }
            }
        }
        diag.fullScanMs+=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-Tscan).count();
        return true;
    };

    auto subset_kkt=[&](double&minSlack,double&cost,
                        double&gap,double&comp,double&stat)->bool{
        ++diag.subsetKKTSolves;
        const int ma=(int)activeList.size();
        if(ma<1)return false;

        std::vector<Row>sub;
        sub.reserve(ma);
        for(int idx:activeList)sub.push_back(rows[idx]);

        R58IPMDiag ID;
        const auto Tsub=std::chrono::steady_clock::now();
        QPSol q=r58_solve_qp_mehrotra(H,n,sub,ID);
        diag.subsetSolverMs+=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-Tsub).count();
        diag.pcgMs+=ID.pcgMs;

        if(ID.augmentedKKTSolveFail)++diag.augmentedKKTSolveFail;
        diag.augmentedKKTMaxDim=
            std::max(diag.augmentedKKTMaxDim,ID.augmentedKKTMaxDim);
        diag.augmentedKKTNewtonSolves+=ID.augmentedKKTNewtonSolves;
        diag.augmentedKKTMaxRelResidual=
            std::max(diag.augmentedKKTMaxRelResidual,ID.augmentedKKTMaxRelResidual);
        diag.commonStepMin=std::min(diag.commonStepMin,ID.commonStepMin);
        if(fin(ID.commonStepLast))diag.commonStepLast=ID.commonStepLast;

        if(ID.converged)++diag.ipmConverged;
        if(ID.resourceFail)++diag.ipmResourceFail;
        if(ID.factorFail)++diag.ipmFactorFail;
        diag.ipmIterationsMax=std::max(diag.ipmIterationsMax,ID.iterations);
        diag.ipmNewtonSolves+=ID.newtonSolves;
        if(ID.pcgResourceFail)++diag.ipmPCGResourceFail;
        diag.ipmPCGMaxIterations=
            std::max(diag.ipmPCGMaxIterations,ID.pcgMaxIterations);
        if(fin(ID.pcgFinalRelResidual))
            diag.ipmPCGFinalRelResidual=
                std::min(diag.ipmPCGFinalRelResidual,ID.pcgFinalRelResidual);
        if(fin(ID.primalResidual))
            diag.ipmPrimalResidual=std::min(diag.ipmPrimalResidual,ID.primalResidual);
        if(fin(ID.dualResidual))
            diag.ipmDualResidual=std::min(diag.ipmDualResidual,ID.dualResidual);
        if(fin(ID.complementarity))
            diag.ipmComplementarity=std::min(diag.ipmComplementarity,ID.complementarity);
        if(fin(ID.dualityGap))
            diag.ipmDualityGap=std::min(diag.ipmDualityGap,ID.dualityGap);
        if(fin(ID.minSlack))diag.ipmMinSlack=ID.minSlack;
        if(fin(ID.minLambda))diag.ipmMinLambda=ID.minLambda;

        if(!q.ok||!ID.converged){
            diag.failureOuterIteration=diag.outerIterations;
            diag.failureActiveRows=ma;
            diag.failureAddedRow=activeList.empty()?-1:activeList.back();
            diag.failureIPMIteration=ID.iterations;
            diag.failureStage=ID.failureStage;
            diag.failurePCGIterations=ID.failurePCGIterations;
            diag.failurePCGFinalRelResidual=ID.failurePCGFinalRelResidual;
            diag.failurePCGRhsInf=ID.failurePCGRhsInf;
            diag.failureDMin=ID.failureDMin;
            diag.failureDMax=ID.failureDMax;
            diag.failureDRatio=ID.failureDRatio;
            diag.failureMinSlack=ID.minSlack;
            diag.failureMinLambda=ID.minLambda;
            diag.failureMetricMaxAbsCorr=active_metric_max_abs_corr();
            diag.failurePrimalResidual=ID.primalResidual;
            diag.failureDualResidual=ID.dualResidual;
            diag.failureMu=ID.complementarity;
            diag.failureDualityGap=ID.dualityGap;
            diag.failureKKTTol=ID.kktTolerance;
            diag.failureGapTol=ID.gapTolerance;
            diag.failureLastSigma=ID.lastSigma;
            diag.failureLastAlphaPrimal=ID.lastAlphaPrimal;
            diag.failureLastAlphaDual=ID.lastAlphaDual;
            diag.failureMinAlphaPrimal=ID.minAlphaPrimal;
            diag.failureMinAlphaDual=ID.minAlphaDual;
            diag.failureMuRatio=ID.lastMuRatio;
            diag.failurePredictorPCGIterations=ID.predictorPCGIterations;
            diag.failureCorrectorPCGIterations=ID.correctorPCGIterations;
            diag.failurePredictorPCGRelResidual=ID.predictorPCGRelResidual;
            diag.failureCorrectorPCGRelResidual=ID.correctorPCGRelResidual;
            if(ID.failureStage==3){
                diag.failureDMin=ID.currentDMin;
                diag.failureDMax=ID.currentDMax;
                diag.failureDRatio=ID.currentDRatio;
            }
            diag.subsetResourceFail=true;
            return false;
        }

        ++diag.subsetKKTConverged;
        z=q.z;
        minSlack=q.slack;
        cost=q.cost;
        gap=ID.dualityGap;
        comp=ID.complementarity;
        stat=std::max(ID.primalResidual,ID.dualResidual);
        std::fill(lambda.begin(),lambda.end(),0.0);
        return true;
    };

    // First full scan from z=0.
    int worst=-1;
    double maxViol=0.0,minSlack=0.0;
    if(!full_scan(worst,maxViol,minSlack))return out;
    diag.initialWorstViolation=maxViol;

    // If z=0 already satisfies all rows, it is trivially the full optimum.
    if(worst<0){
        out.ok=true;
        out.z=z;
        out.cost=0.0;
        out.slack=minSlack;
        diag.fullOptimalityClosed=true;
        diag.finalMaxViolation=0.0;
        diag.finalDualityGap=0.0;
        diag.finalComplementarity=0.0;
        diag.finalStationarity=0.0;
        return out;
    }

    // Outer loop is finite by construction: each failed full scan adds one
    // previously inactive violated row, so there can be at most m additions.
    for(int outer=1;outer<=m;++outer){
        diag.outerIterations=outer;

        if(worst<0||worst>=m)return QPSol{};
        if(active[worst]){
            diag.duplicateAddFail=true;
            return QPSol{};
        }
        active[worst]=1;
        activeList.push_back(worst);
        ++diag.rowsAdded;
        diag.activeRowsMax=
            std::max(diag.activeRowsMax,(int)activeList.size());

        double subsetMinSlack=0.0,cost=0.0,gap=0.0,comp=0.0,stat=0.0;
        if(!subset_kkt(subsetMinSlack,cost,gap,comp,stat)){
            if(!diag.subsetResourceFail)
                diag.subsetResourceFail=true;
            return QPSol{};
        }

        // Exact constraint-generation authority: scan ALL original rows.
        if(!full_scan(worst,maxViol,minSlack))return QPSol{};
        diag.finalMaxViolation=maxViol;
        diag.finalDualityGap=std::fabs(gap);
        diag.finalComplementarity=comp;
        diag.finalStationarity=stat;

        if(worst<0){
            // The subset optimum is feasible for the full feasible set.
            // Since F_full subset F_subset:
            //   J_subset* <= J_full* <= J(z_subset*) = J_subset*.
            // Therefore this is also the full-QP optimum.
            out.ok=true;
            out.z=z;
            out.cost=cost;
            out.slack=minSlack;
            diag.fullOptimalityClosed=true;
            return out;
        }
    }

    // Reaching here would require more than m distinct violated rows, which is
    // impossible without a logic/numerical inconsistency. Fail closed.
    diag.duplicateAddFail=true;
    return QPSol{};
}

struct R58SearchNode{
    int depth=0;
    std::vector<int>signs;
    QPSol q;
    double lb=std::numeric_limits<double>::infinity();
    unsigned long long seq=0;
};
struct R58SearchGreater{
    bool operator()(const R58SearchNode&a,const R58SearchNode&b)const{
        if(a.lb!=b.lb)return a.lb>b.lb;
        return a.seq>b.seq;
    }
};



// ===========================================================================
// AF7R56 THEORY/SHADOW
//
// Purpose:
//   1) construct the LOAD spatial path in SOURCE PHASE
//
//          r(s) = p_active(s) + delta(s)
//
//      with exact-C7 delta(s), exact-support oriented-ellipsoid corridor rows,
//      and moving-normal synthesis constraints;
//
//   2) evaluate an INDEPENDENT time law s(t) through exact chain-rule
//      composition.  The shadow never changes the R55 production candidate,
//      never grants acceptance, and never relaxes a hard certificate.
//
// Important suspended-load coupling:
//   although the load path r(s) is a purely spatial/source-phase object, the
//   cable direction q=-f_s/||f_s|| and UAV point p_Q=p_L-Lq depend on the
//   chosen time law through r_ss*sdot^2+r_s*sddot.  Therefore R56 explicitly
//   measures this residual geometry/time coupling instead of pretending that
//   whole-body geometry is fully separable.
//
// No trigger horizon is used as a maneuver duration.
// No equality to the old same-clock reference position is imposed.
// ===========================================================================

struct R56SourceSample{
    double s=0.0;
    Vec3 p{},v{},a{},j{};
    double hxy=0.0;
};

struct R56DynDiag{
    bool finite=true;
    bool dynamic_pass=false;
    bool geometry_point_pass=false;
    int samples=0;
    double maxVL=0.0,maxVQ=0.0,minSigma=std::numeric_limits<double>::infinity();
    double maxQdot=0.0,minGeom=std::numeric_limits<double>::infinity();
    double chainErr=0.0;
};

static inline double r56_vec_err(Vec3 a,Vec3 b){
    return norm(a-b);
}

static inline std::array<double,7> r56_scalar_jet_from_control(
        std::vector<double>C,double T,double tau){
    std::array<double,7>S{};
    if(!(fin(T)&&T>0.0&&C.size()==16&&fin(tau)))return S;
    double u=std::max(0.0,std::min(1.0,tau/T));
    std::vector<double>Q=C;
    int deg=15;
    S[0]=r51_scalar_eval(Q,u);
    for(int r=1;r<=6;++r){
        Q=r51_diff_scalar(Q,1);
        const double sc=double(deg)/T;
        for(double&x:Q)x*=sc;
        --deg;
        S[r]=r51_scalar_eval(Q,u);
    }
    return S;
}

static inline std::array<double,7> r56_entry_phase_jet(
        double source0,double alpha,double T,double tau){
    std::array<double,8>J0{},J1{};
    J0[0]=source0;J0[1]=1.0;
    J1[0]=source0+0.5*(1.0+alpha)*T;
    J1[1]=alpha;
    auto C=scalar_control_from_jets(J0,J1,T);
    return r56_scalar_jet_from_control(C,T,tau);
}

static inline std::array<double,7> r56_exit_phase_jet(
        double source0,double alpha,double T,double tau){
    std::array<double,8>J0{},J1{};
    J0[0]=source0;J0[1]=alpha;
    J1[0]=source0+0.5*(1.0+alpha)*T;
    J1[1]=1.0;
    auto C=scalar_control_from_jets(J0,J1,T);
    return r56_scalar_jet_from_control(C,T,tau);
}

static inline bool r56_towing_state_from_jet(
        const std::array<Vec3,7>&Y,const Live&L,
        double&VL,double&VQ,double&sigma,double&qdot,Vec3&pQ){
    const Vec3 pL=Y[0],v=Y[1],a=Y[2],j=Y[3];
    Vec3 sf=a+Vec3{0,0,L.g};
    sigma=norm(sf);
    if(!(fin(sigma)&&sigma>0.0&&finite(v)&&finite(j)))return false;
    Vec3 q=sf*(-1.0/sigma);
    Vec3 proj=j-q*dot(q,j);
    Vec3 qd=proj*(-1.0/sigma);
    pQ=pL-q*L.L;
    VL=norm(v);qdot=norm(qd);VQ=norm(v-qd*L.L);
    return fin(VL)&&fin(VQ)&&fin(qdot)&&finite(pQ);
}

static inline void r56_update_point_geometry(
        const std::array<Vec3,7>&Y,const Live&L,
        const std::vector<Obstacle>&obs,double&minGeom,bool&finiteAll){
    double VL=0,VQ=0,sigma=0,qd=0;Vec3 pQ{};
    if(!r56_towing_state_from_jet(Y,L,VL,VQ,sigma,qd,pQ)){
        finiteAll=false;return;
    }
    const Vec3 pL=Y[0];
    const double rhoL=L.rL+L.dmargin+L.tracking;
    const double rhoQ=L.rQ+L.dmargin+L.tracking;
    const double rhoC=L.rC+L.dmargin+L.tracking;
    for(const auto&o:obs){
        double mL=-std::numeric_limits<double>::infinity();
        double mQ=-std::numeric_limits<double>::infinity();
        double mC=-std::numeric_limits<double>::infinity();
        (void)point_margin(pL,o,rhoL,mL);
        (void)point_margin(pQ,o,rhoQ,mQ);
        (void)segment_margin(pQ,pL,o,rhoC,mC);
        if(!(fin(mL)&&fin(mQ)&&fin(mC))){
            finiteAll=false;continue;
        }
        minGeom=std::min(minGeom,std::min(mL,std::min(mQ,mC)));
    }
}

static inline R56DynDiag r56_eval_const_alpha_samples(
        const std::vector<R56SourceSample>&S,const Live&L,
        const std::vector<Obstacle>&obs,double alpha,double lim){
    R56DynDiag D;
    if(!(fin(alpha)&&alpha>0.0&&alpha<=1.0&&fin(lim)&&lim>0.0)||S.empty()){
        D.finite=false;return D;
    }
    D.samples=(int)S.size();
    const std::array<double,7>PJ={0.0,alpha,0.0,0.0,0.0,0.0,0.0};

    for(const auto&Q:S){
        std::array<Vec3,7>X{};
        X[0]=Q.p;X[1]=Q.v;X[2]=Q.a;X[3]=Q.j;
        // Higher derivatives are not required for the towing state here.
        auto Y=r51_compose_vec_jet(X,PJ);

        const Vec3 ve=Q.v*alpha;
        const Vec3 ae=Q.a*(alpha*alpha);
        const Vec3 je=Q.j*(alpha*alpha*alpha);
        D.chainErr=std::max(D.chainErr,r56_vec_err(Y[1],ve));
        D.chainErr=std::max(D.chainErr,r56_vec_err(Y[2],ae));
        D.chainErr=std::max(D.chainErr,r56_vec_err(Y[3],je));

        double VL=0,VQ=0,sigma=0,qd=0;Vec3 pQ{};
        if(!r56_towing_state_from_jet(Y,L,VL,VQ,sigma,qd,pQ)){
            D.finite=false;continue;
        }
        D.maxVL=std::max(D.maxVL,VL);
        D.maxVQ=std::max(D.maxVQ,VQ);
        D.minSigma=std::min(D.minSigma,sigma);
        D.maxQdot=std::max(D.maxQdot,qd);
        r56_update_point_geometry(Y,L,obs,D.minGeom,D.finite);
    }

    D.dynamic_pass=D.finite&&fin(D.minSigma)&&D.minSigma>0.0&&
                   fin(D.maxVQ)&&D.maxVQ<lim;
    D.geometry_point_pass=D.finite&&fin(D.minGeom)&&D.minGeom>0.0;
    return D;
}

static inline R56DynDiag r56_eval_transition(
        const ActiveReference&ref,const Candidate&Cs,const Live&L,
        const std::vector<Obstacle>&obs,double lim,
        double source0,double alpha,double T,bool entry,double h){
    R56DynDiag D;
    if(!(fin(T)&&T>0.0&&fin(h)&&h>0.0&&fin(alpha)&&alpha>0.0&&alpha<=1.0)){
        D.finite=false;return D;
    }
    long long N=(long long)std::ceil(T/h);
    N=std::max(1LL,N);
    for(long long k=0;k<=N;++k){
        const double tau=T*double(k)/double(N);
        const auto SJ=entry?
            r56_entry_phase_jet(source0,alpha,T,tau):
            r56_exit_phase_jet(source0,alpha,T,tau);
        if(!(fin(SJ[0])&&fin(SJ[1])&&SJ[1]>0.0&&SJ[1]<=1.0+
             4096*epsd()*std::max(1.0,std::fabs(SJ[1])))){
            D.finite=false;continue;
        }
        auto X=candidate_eval(ref,Cs,SJ[0]);
        auto Y=r51_compose_vec_jet(X,SJ);

        const Vec3 ve=X[1]*SJ[1];
        const Vec3 ae=X[2]*(SJ[1]*SJ[1])+X[1]*SJ[2];
        const Vec3 je=X[3]*(SJ[1]*SJ[1]*SJ[1])+
                      X[2]*(3.0*SJ[1]*SJ[2])+X[1]*SJ[3];
        D.chainErr=std::max(D.chainErr,r56_vec_err(Y[1],ve));
        D.chainErr=std::max(D.chainErr,r56_vec_err(Y[2],ae));
        D.chainErr=std::max(D.chainErr,r56_vec_err(Y[3],je));

        double VL=0,VQ=0,sigma=0,qd=0;Vec3 pQ{};
        if(!r56_towing_state_from_jet(Y,L,VL,VQ,sigma,qd,pQ)){
            D.finite=false;continue;
        }
        ++D.samples;
        D.maxVL=std::max(D.maxVL,VL);
        D.maxVQ=std::max(D.maxVQ,VQ);
        D.minSigma=std::min(D.minSigma,sigma);
        D.maxQdot=std::max(D.maxQdot,qd);
        r56_update_point_geometry(Y,L,obs,D.minGeom,D.finite);
    }
    D.dynamic_pass=D.finite&&D.samples>0&&fin(D.minSigma)&&D.minSigma>0.0&&
                   fin(D.maxVQ)&&D.maxVQ<lim;
    D.geometry_point_pass=D.finite&&D.samples>0&&fin(D.minGeom)&&D.minGeom>0.0;
    return D;
}

static inline double r56_align_down_positive(double x,double h){
    if(!(fin(x)&&fin(h)&&h>0.0&&x>0.0))return 0.0;
    double y=r37_align_down(x,h);
    return (fin(y)&&y>0.0)?y:0.0;
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

    // AF7R47: legacy affine/towing corridor construction is lazy. The R37/R45
    // primary has its own exact-support corridor, so eagerly building the old
    // fallback corridor before trying the primary was pure online overhead.
    // If the primary does not certify, the historical rows are constructed
    // exactly as before and all fallbacks remain available.
    std::vector<OrientedRows> OR;
    bool legacyCorridorReady=false;
    auto ensure_legacy_corridor=[&]()->bool{
        if(legacyCorridorReady)return true;
        const auto tcorr=clk::now();
        try{
            OR.assign(Ne,OrientedRows{});
            for(int e=0;e<Ne;++e){
                auto ep=E.ep[e];
                auto xa=ref.eval(ep.start),xb=ref.eval(ep.end);
                Vec3 prog=xb[0]-xa[0];
                if(norm(prog)<=128*epsd()*std::max(1.0,norm(xa[0])))
                    prog=ref.eval(0.5*(ep.start+ep.end))[1];
                Vec3 n=lateral_normal(obs[ep.obs].R,obs[ep.obs].axes,prog);
                OR[e].plus=corridor_rows(ref,L,base,obs,E,e,n,nvar);
                OR[e].minus=corridor_rows(ref,L,base,obs,E,e,n*(-1),nvar);
            }
        }catch(const std::runtime_error&){
            OR.clear();
            R.tim.corridor+=std::chrono::duration<double,std::milli>(
                clk::now()-tcorr).count();
            return false;
        }
        R.tim.corridor+=std::chrono::duration<double,std::milli>(
            clk::now()-tcorr).count();
        legacyCorridorReady=true;
        return true;
    };

    // The affine minimum-snap Hessian is built lazily only if the primary
    // minimum-deviation family cannot close the hard certificates.
    std::vector<double> Haff;
    R.tim.hessian=0.0;
    auto ts=clk::now();

    double rhoBody=std::max({L.rQ,L.L+L.rL,L.L+L.rC});
    double lim=(L.sensor_range-rhoBody)/(L.horizon+L.dt);
    if(!(fin(lim)&&lim>0)){
        R.reason=16;R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;
    }

    // ------------------------------------------------------------------
    // AF7R58 FREE-C7 / GLOBAL-HOMOTOPY REFERENCE-CORE SHADOW
    //
    // The finite source boundary is a validity boundary, NOT a rejoin time.
    // Correction position is free there; derivatives 1..7 are zero and the
    // offset is held. No same-clock nominal position is targeted.
    //
    // Reference-level only: tracking inflation must be zero.
    // ------------------------------------------------------------------
    if(L.r58_shadow_diagnostic_enabled){
        const auto Tr58=clk::now();
        // Only the low-jet C7 family may be bridged. The bridge does NOT
        // bypass execution-aware certification or change any acceptance test.
        const bool bridge16=L.r58r16_reference_bridge&&L.r58r14_lowjet_seed_only&&L.tracking==0.0;
        R.r58_shadow_ran=1;
        R.r58_reference_tracking_inflation_m=L.tracking;
        R.r58_episode_count=(int)E.ep.size();
        R.r58_component_count=(int)E.comp.size();

        const double h58=std::ldexp(1.0,std::ilogb(L.dt));
        auto K58=r37_make_knots(E,t0,t1,h58);

        if(K58.size()>=2&&L.tracking==0.0){
            R.r58_support_start_s=K58.front();
            R.r58_support_end_s=K58.back();
            R.r58_source_knot_count=(int)K58.size();
            R.r58_scalar_dof_count=r58_scalar_dofs(K58);
            R.r58_spatial_dof_count=3*R.r58_scalar_dof_count;
            R.r58r2_node_scaled_variables=1;
            for(int node=1;node<(int)K58.size();++node){
                const double lam=r58_node_scale(K58,node);
                if(fin(lam)&&lam>0.0){
                    R.r58r2_node_scale_min_s=
                        std::min(R.r58r2_node_scale_min_s,lam);
                    R.r58r2_node_scale_max_s=
                        std::max(R.r58r2_node_scale_max_s,lam);
                }
            }

            const auto Tb=clk::now();
            auto EL58=r58_build_normal_elimination(K58,ref);
            if(!EL58.ok){
                ++R.r58r4_elimination_build_fail;
            }else{
                R.r58r4_normal_equalities_eliminated=1;
                R.r58r4_reduced_spatial_dof_count=EL58.nred;
                R.r58r4_elimination_max_residual=EL58.maxResidual;
            }

            const bool lowJet58=L.r58r14_lowjet_seed_only;
            const auto lowJetIndices58=lowJet58 ?
                r58r14_lowjet_indices((int)K58.size()) : std::vector<int>{};
            // Direct R22 low-jet Hessian: identical quadratic functional,
            // but no intermediate 3*ns x 3*ns spatial matrix is built.
            auto H58=lowJet58 ?
                r58r22_lowjet_hessian_direct(K58,L.L,L.g,EL58,lowJetIndices58) :
                r58_reduce_hessian(r58_spatial_hessian3(K58,L.L,L.g),EL58);
            std::vector<double>H58eq,varScale58;
            double hdmin=0.0,hdmax=0.0,hemin=0.0,hemax=0.0;
            const int nv58=lowJet58?(int)lowJetIndices58.size():EL58.nred;
            R.r58r14_lowjet_seed_only=lowJet58?1:0;
            R.r58r14_active_reduced_dof=nv58;
            R.r58r14_full_reduced_dof=EL58.nred;
            const bool heqOK=EL58.ok&&!H58.empty()&&
                r58_equilibrate_hessian(
                    H58,nv58,H58eq,varScale58,
                    hdmin,hdmax,hemin,hemax);
            if(heqOK){
                R.r58r3_hessian_diag_equilibrated=1;
                R.r58r3_raw_hdiag_min=hdmin;
                R.r58r3_raw_hdiag_max=hdmax;
                R.r58r3_raw_hdiag_ratio=hdmax/hdmin;
                R.r58r3_eq_hdiag_min=hemin;
                R.r58r3_eq_hdiag_max=hemax;
                std::vector<double>Ltest;
                R.r58r3_equilibrated_cholesky_pass=
                    chol(H58eq,nv58,Ltest)?1:0;
            }

            std::vector<std::array<R58SideRows,2>>cache(E.ep.size());
            // R56: row-wise scaling/equilibration commutes with concatenation.
            // Precondition each immutable episode/side row ONCE rather than
            // inside each repeated branch-and-bound partial QP solve.
            // This changes no QP objective/feasible set or acceptance gate.
            std::vector<std::array<std::array<double,2>,2>>
                r56RowScales(E.ep.size());
            bool cacheOK=heqOK&&R.r58r3_equilibrated_cholesky_pass==1;
            if(cacheOK){
                for(int ei=0;ei<(int)E.ep.size();++ei){
                    cache[ei][0]=r58_episode_rows(
                        ref,L,base,obs,E,ei,+1,K58);
                    cache[ei][1]=r58_episode_rows(
                        ref,L,base,obs,E,ei,-1,K58);
                    if(cache[ei][0].valid){
                        cache[ei][0].valid=lowJet58 ?
                            r58_reduce_rows_selected(cache[ei][0].rows,EL58,lowJetIndices58) :
                            r58_reduce_rows(cache[ei][0].rows,EL58);
                    }
                    if(cache[ei][1].valid){
                        cache[ei][1].valid=lowJet58 ?
                            r58_reduce_rows_selected(cache[ei][1].rows,EL58,lowJetIndices58) :
                            r58_reduce_rows(cache[ei][1].rows,EL58);
                    }
                    // Every row's scale depends ONLY on that row; the
                    // original solve_partial() did these two operations
                    // AFTER concatenation, yielding the same row values.
                    for(int si=0;si<2;++si){
                        if(!cache[ei][si].valid)continue;
                        auto& rows56=cache[ei][si].rows;
                        double rmin56=0.0,rmax56=0.0;
                        cache[ei][si].valid=
                            r58_apply_variable_scaling_to_rows(rows56,varScale58) &&
                            r58_equilibrate_rows(rows56,rmin56,rmax56);
                        if(cache[ei][si].valid){
                            r56RowScales[ei][si]={{rmin56,rmax56}};
                        }
                    }
                    if(cache[ei][0].valid)++R.r58_side_cache_plus_valid;
                    if(cache[ei][1].valid)++R.r58_side_cache_minus_valid;
                    if(!cache[ei][0].valid&&!cache[ei][1].valid){
                        ++R.r58_side_cache_fail;cacheOK=false;break;
                    }
                }
            }
            R.r58_build_ms=
                std::chrono::duration<double,std::milli>(clk::now()-Tb).count();

            if(cacheOK){
                const auto Th=clk::now();
                const int M=(int)E.ep.size();
                unsigned long long seq=0;

                auto solve_partial=[&](const std::vector<int>&sg,int depth)->QPSol{
                    std::vector<Row>rows;
                    double rsmin=std::numeric_limits<double>::infinity();
                    double rsmax=0.0;
                    for(int i=0;i<depth;++i){
                        const int si=sg[i]>0?0:1;
                        const auto&SR=cache[i][si];
                        if(!SR.valid)return QPSol{};
                        rows.insert(rows.end(),SR.rows.begin(),SR.rows.end());
                        rsmin=std::min(rsmin,r56RowScales[i][si][0]);
                        rsmax=std::max(rsmax,r56RowScales[i][si][1]);
                    }
                    R.r58r2_row_equilibrated=1;
                    R.r58r2_row_scale_min=
                        std::min(R.r58r2_row_scale_min,rsmin);
                    R.r58r2_row_scale_max=
                        std::max(R.r58r2_row_scale_max,rsmax);
                    ++R.r58_bnb_qp_solves;
                    R.r58r7_constraint_generation_used=1;
                    R58CGDiag CG;
                    const auto Tcg=clk::now();
                    QPSol q=r58_solve_qp_constraint_generation(
                        H58eq,nv58,rows,CG);
                    const double cgms=
                        std::chrono::duration<double,std::milli>(
                            clk::now()-Tcg).count();
                    R.r58r7_solver_ms+=cgms;
                    R.r58r8_ipm_ms+=cgms;

                    R.r58r7_full_row_count_max=
                        std::max(R.r58r7_full_row_count_max,CG.fullRowCount);
                    R.r58r7_outer_iterations_max=
                        std::max(R.r58r7_outer_iterations_max,CG.outerIterations);
                    R.r58r7_full_scans+=CG.fullScans;
                    R.r58r7_rows_added+=CG.rowsAdded;
                    R.r58r7_active_rows_max=
                        std::max(R.r58r7_active_rows_max,CG.activeRowsMax);
                    R.r58r7_subset_kkt_solves+=CG.subsetKKTSolves;
                    R.r58r7_subset_kkt_converged+=CG.subsetKKTConverged;
                    if(CG.subsetResourceFail)++R.r58r7_subset_resource_fail;
                    if(CG.duplicateAddFail)++R.r58r7_duplicate_add_fail;
                    if(CG.fullOptimalityClosed)++R.r58r7_full_optimality_closed;
                    R.r58r7_subset_sweeps_max=
                        std::max(R.r58r7_subset_sweeps_max,CG.subsetSweepsMax);
                    R.r58r7_coordinate_updates+=CG.coordinateUpdates;
                    R.r58r7_initial_worst_violation=
                        std::max(R.r58r7_initial_worst_violation,
                                 CG.initialWorstViolation);
                    if(fin(CG.finalMaxViolation))
                        R.r58r7_final_max_violation=
                            std::min(R.r58r7_final_max_violation,
                                     CG.finalMaxViolation);
                    if(fin(CG.finalDualityGap))
                        R.r58r7_final_duality_gap=
                            std::min(R.r58r7_final_duality_gap,
                                     CG.finalDualityGap);
                    if(fin(CG.finalComplementarity))
                        R.r58r7_final_complementarity=
                            std::min(R.r58r7_final_complementarity,
                                     CG.finalComplementarity);
                    if(fin(CG.finalStationarity))
                        R.r58r7_final_stationarity=
                            std::min(R.r58r7_final_stationarity,
                                     CG.finalStationarity);

                    R.r58r8_ipm_used=1;
                    R.r58r8_newton_diag_equilibrated=0;
                    R.r58r8_ipm_converged+=CG.ipmConverged;
                    R.r58r8_ipm_resource_fail+=CG.ipmResourceFail;
                    R.r58r8_ipm_factor_fail+=CG.ipmFactorFail;
                    R.r58r8_ipm_max_iterations=
                        std::max(R.r58r8_ipm_max_iterations,CG.ipmIterationsMax);
                    R.r58r8_ipm_newton_solves+=CG.ipmNewtonSolves;
                    R.r58r8_pcg_used=1;
                    R.r58r8_pcg_resource_fail+=CG.ipmPCGResourceFail;
                    R.r58r8_pcg_max_iterations=
                        std::max(R.r58r8_pcg_max_iterations,CG.ipmPCGMaxIterations);
                    if(fin(CG.ipmPCGFinalRelResidual))
                        R.r58r8_pcg_final_rel_residual=
                            std::min(R.r58r8_pcg_final_rel_residual,
                                     CG.ipmPCGFinalRelResidual);

                    R.r58r9_full_scan_ms+=CG.fullScanMs;
                    R.r58r9_subset_solver_ms+=CG.subsetSolverMs;
                    R.r58r9_pcg_ms+=CG.pcgMs;

                    R.r58r11_augmented_kkt_used=1;
                    R.r58r11_augmented_kkt_solve_fail+=CG.augmentedKKTSolveFail;
                    R.r58r11_common_step_used=1;
                    R.r58r11_augmented_kkt_max_dim=
                        std::max(R.r58r11_augmented_kkt_max_dim,
                                 CG.augmentedKKTMaxDim);
                    R.r58r11_newton_solves+=CG.augmentedKKTNewtonSolves;
                    R.r58r11_newton_max_rel_residual=
                        std::max(R.r58r11_newton_max_rel_residual,
                                 CG.augmentedKKTMaxRelResidual);
                    R.r58r11_common_step_min=
                        std::min(R.r58r11_common_step_min,CG.commonStepMin);
                    if(fin(CG.commonStepLast))
                        R.r58r11_common_step_last=CG.commonStepLast;
                    R.r58r11_solver_ms+=cgms;

                    if(!q.ok){
                        ++R.r58r9_failure_count;
                        R.r58r9_failure_depth=depth;
                        R.r58r9_failure_last_side=sg.empty()?0:sg.back();
                        R.r58r9_failure_outer_iteration=CG.failureOuterIteration;
                        R.r58r9_failure_active_rows=CG.failureActiveRows;
                        R.r58r9_failure_added_row=CG.failureAddedRow;
                        R.r58r9_failure_ipm_iteration=CG.failureIPMIteration;
                        R.r58r9_failure_stage=CG.failureStage;
                        R.r58r9_failure_pcg_iterations=CG.failurePCGIterations;
                        R.r58r9_failure_pcg_final_rel_residual=
                            CG.failurePCGFinalRelResidual;
                        R.r58r9_failure_pcg_rhs_inf=CG.failurePCGRhsInf;
                        R.r58r9_failure_D_min=CG.failureDMin;
                        R.r58r9_failure_D_max=CG.failureDMax;
                        R.r58r9_failure_D_ratio=CG.failureDRatio;
                        R.r58r9_failure_min_slack=CG.failureMinSlack;
                        R.r58r9_failure_min_lambda=CG.failureMinLambda;
                        R.r58r9_failure_metric_max_abs_corr=
                            CG.failureMetricMaxAbsCorr;
                        R.r58r9_failure_metric_one_minus_corr=
                            1.0-CG.failureMetricMaxAbsCorr;

                        if(depth==1&&!sg.empty()&&sg.back()>0){
                            R.r58r9_plus_failed=1;
                            R.r58r9_plus_failure_stage=CG.failureStage;
                            R.r58r9_plus_failure_active_rows=CG.failureActiveRows;
                            R.r58r9_plus_failure_pcg_relres=
                                CG.failurePCGFinalRelResidual;
                            R.r58r9_plus_metric_max_abs_corr=
                                CG.failureMetricMaxAbsCorr;
                            R.r58r9_plus_cg_ms=cgms;

                            R.r58r10_plus_primal_residual=CG.failurePrimalResidual;
                            R.r58r10_plus_dual_residual=CG.failureDualResidual;
                            R.r58r10_plus_mu=CG.failureMu;
                            R.r58r10_plus_duality_gap=CG.failureDualityGap;
                            R.r58r10_plus_kkt_tol=CG.failureKKTTol;
                            R.r58r10_plus_gap_tol=CG.failureGapTol;
                            R.r58r10_plus_last_sigma=CG.failureLastSigma;
                            R.r58r10_plus_last_alpha_primal=CG.failureLastAlphaPrimal;
                            R.r58r10_plus_last_alpha_dual=CG.failureLastAlphaDual;
                            R.r58r10_plus_min_alpha_primal=CG.failureMinAlphaPrimal;
                            R.r58r10_plus_min_alpha_dual=CG.failureMinAlphaDual;
                            R.r58r10_plus_mu_ratio=CG.failureMuRatio;
                            R.r58r10_plus_D_min=CG.failureDMin;
                            R.r58r10_plus_D_max=CG.failureDMax;
                            R.r58r10_plus_D_ratio=CG.failureDRatio;
                            R.r58r10_plus_min_slack=CG.failureMinSlack;
                            R.r58r10_plus_min_lambda=CG.failureMinLambda;
                            R.r58r10_plus_predictor_pcg_iterations=
                                CG.failurePredictorPCGIterations;
                            R.r58r10_plus_corrector_pcg_iterations=
                                CG.failureCorrectorPCGIterations;
                            R.r58r10_plus_predictor_pcg_relres=
                                CG.failurePredictorPCGRelResidual;
                            R.r58r10_plus_corrector_pcg_relres=
                                CG.failureCorrectorPCGRelResidual;
                        }else if(depth==1&&!sg.empty()&&sg.back()<0){
                            R.r58r9_minus_failed=1;
                            R.r58r9_minus_failure_stage=CG.failureStage;
                            R.r58r9_minus_failure_active_rows=CG.failureActiveRows;
                            R.r58r9_minus_failure_pcg_relres=
                                CG.failurePCGFinalRelResidual;
                            R.r58r9_minus_metric_max_abs_corr=
                                CG.failureMetricMaxAbsCorr;
                            R.r58r9_minus_cg_ms=cgms;

                            R.r58r10_minus_primal_residual=CG.failurePrimalResidual;
                            R.r58r10_minus_dual_residual=CG.failureDualResidual;
                            R.r58r10_minus_mu=CG.failureMu;
                            R.r58r10_minus_duality_gap=CG.failureDualityGap;
                            R.r58r10_minus_kkt_tol=CG.failureKKTTol;
                            R.r58r10_minus_gap_tol=CG.failureGapTol;
                            R.r58r10_minus_last_sigma=CG.failureLastSigma;
                            R.r58r10_minus_last_alpha_primal=CG.failureLastAlphaPrimal;
                            R.r58r10_minus_last_alpha_dual=CG.failureLastAlphaDual;
                            R.r58r10_minus_min_alpha_primal=CG.failureMinAlphaPrimal;
                            R.r58r10_minus_min_alpha_dual=CG.failureMinAlphaDual;
                            R.r58r10_minus_mu_ratio=CG.failureMuRatio;
                            R.r58r10_minus_D_min=CG.failureDMin;
                            R.r58r10_minus_D_max=CG.failureDMax;
                            R.r58r10_minus_D_ratio=CG.failureDRatio;
                            R.r58r10_minus_min_slack=CG.failureMinSlack;
                            R.r58r10_minus_min_lambda=CG.failureMinLambda;
                            R.r58r10_minus_predictor_pcg_iterations=
                                CG.failurePredictorPCGIterations;
                            R.r58r10_minus_corrector_pcg_iterations=
                                CG.failureCorrectorPCGIterations;
                            R.r58r10_minus_predictor_pcg_relres=
                                CG.failurePredictorPCGRelResidual;
                            R.r58r10_minus_corrector_pcg_relres=
                                CG.failureCorrectorPCGRelResidual;
                        }
                    }else if(depth==1&&!sg.empty()&&sg.back()>0){
                        R.r58r9_plus_cg_ms=cgms;
                    }else if(depth==1&&!sg.empty()&&sg.back()<0){
                        R.r58r9_minus_cg_ms=cgms;
                    }
                    if(fin(CG.ipmPrimalResidual))
                        R.r58r8_ipm_primal_residual=
                            std::min(R.r58r8_ipm_primal_residual,CG.ipmPrimalResidual);
                    if(fin(CG.ipmDualResidual))
                        R.r58r8_ipm_dual_residual=
                            std::min(R.r58r8_ipm_dual_residual,CG.ipmDualResidual);
                    if(fin(CG.ipmComplementarity))
                        R.r58r8_ipm_complementarity=
                            std::min(R.r58r8_ipm_complementarity,CG.ipmComplementarity);
                    if(fin(CG.ipmDualityGap))
                        R.r58r8_ipm_duality_gap=
                            std::min(R.r58r8_ipm_duality_gap,CG.ipmDualityGap);
                    if(fin(CG.ipmMinSlack))R.r58r8_ipm_min_slack=CG.ipmMinSlack;
                    if(fin(CG.ipmMinLambda))R.r58r8_ipm_min_lambda=CG.ipmMinLambda;

                    if(!q.ok)++R.r58_bnb_qp_infeasible;
                    return q;
                };

                std::priority_queue<
                    R58SearchNode,std::vector<R58SearchNode>,R58SearchGreater>pq;

                R58SearchNode root58;
                root58.depth=0;root58.seq=++seq;
                // Base rows are homogeneous equalities and z=0 is feasible.
                // H is positive definite after Cholesky authority, so the
                // exact root lower bound is zero; no QP solve is necessary.
                root58.q.ok=true;
                root58.q.cost=0.0;
                root58.q.slack=0.0;
                root58.q.z.assign(nv58,0.0);
                root58.lb=0.0;
                R.r58r2_root_qp_elided=1;
                pq.push(std::move(root58));

                while(!pq.empty()&&!R.r58_bnb_feasible_full){
                    R.r58_bnb_max_queue=
                        std::max(R.r58_bnb_max_queue,(int)pq.size());
                    auto N58=pq.top();pq.pop();
                    ++R.r58_bnb_nodes_popped;

                    if(N58.depth<M){
                        const int ei=N58.depth;
                        for(int si=0;si<2;++si){
                            const int side=(si==0)?+1:-1;
                            if(!cache[ei][si].valid)continue;

                            R58SearchNode Cn;
                            Cn.depth=N58.depth+1;
                            Cn.signs=N58.signs;
                            Cn.signs.push_back(side);
                            Cn.seq=++seq;
                            Cn.q=solve_partial(Cn.signs,Cn.depth);
                            if(!Cn.q.ok)continue;
                            Cn.lb=Cn.q.cost;
                            pq.push(std::move(Cn));
                        }
                        continue;
                    }

                    ++R.r58_bnb_full_assignments;
                    double qerr=0.0,qphys=0.0,joinmis=0.0;
                    std::array<double,3> rawQPderivativeUB{};
                    Candidate C58;
                    try{
                        QPSol qReduced=r58_unscale_solution(
                            N58.q,varScale58);
                        if(!qReduced.ok)continue;
                        if(lowJet58){
                            qReduced=r58r14_lift_solution(
                                qReduced,EL58.nred,lowJetIndices58);
                            if(!qReduced.ok)continue;
                        }
                        QPSol qPhysical=r58_expand_solution(qReduced,EL58);
                        if(!qPhysical.ok)continue;
                        for(double zv:qPhysical.z)
                            R.r58r6_prequant_scaled_jet_max_abs=
                                std::max(R.r58r6_prequant_scaled_jet_max_abs,
                                         std::fabs(zv));
                        C58=r58_build_candidate(
                            qPhysical,K58,h58,L.L,L.g,N58.signs,
                            qerr,qphys,joinmis,rawQPderivativeUB,
                            L.r58_detail_diagnostic_enabled);
                    }catch(const std::runtime_error&){
                        continue;
                    }

                    R.r58_quantized_jet_max_abs_error=
                        std::max(R.r58_quantized_jet_max_abs_error,qerr);
                    R.r58r6_quantized_physical_jet_max_abs_error=
                        std::max(R.r58r6_quantized_physical_jet_max_abs_error,
                                 qphys);
                    if(R.r58r6_prequant_scaled_jet_max_abs>0.0)
                        R.r58r6_quantized_scaled_jet_rel_error=
                            std::max(
                                R.r58r6_quantized_scaled_jet_rel_error,
                                qerr/R.r58r6_prequant_scaled_jet_max_abs);
                    R.r58_legacy_join_max_abs_mismatch=
                        std::max(R.r58_legacy_join_max_abs_mismatch,joinmis);

                    if(!(C58.exact_c7&&C58.exact_c7_constructive))
                        continue;

                    const auto Tc=clk::now();
                    SpeedCert sc58;
                    auto tr58=tighten_stageA(C58,base,L.L,lim);
                    if(tr58.certified){
                        sc58=stageA_speed(tr58,base,L.L);
                        if(!(sc58.certified&&sc58.VQ<lim))
                            sc58=SpeedCert{};
                    }
                    R52StageBTrace SB58;
                    bool finiteWitness58=false,repr58=false;
                    if(!sc58.certified){
                        sc58=stageB_speed(
                            ref,L,base,C58,lim,
                            finiteWitness58,repr58,&SB58);
                    }
                    // R58: 38-column gate trace is diagnostic ONLY. Do not spend
                    // Flight time on it in clean production mode.
                    if(L.r58_detail_diagnostic_enabled){
                    const double sigmaA=base.sigma-tr58.U[1];
                    const double vqA=sigmaA>0.0 ?
                        base.VL+tr58.U[0]+L.L*(base.J+tr58.U[2])/sigmaA :
                        std::numeric_limits<double>::infinity();
                    const double qnan=std::numeric_limits<double>::quiet_NaN();
                    std::array<double,38> r13row{{
                        (double)(R.r58_bnb_full_assignments),
                        N58.signs.empty()?0.0:(double)N58.signs.back(),
                        (double)N58.signs.size(),
                        (double)C58.cp.size(),N58.q.cost,lim,
                        qerr,qphys,C58.dub[0],C58.dub[1],C58.dub[2],
                        tr58.certified?1.0:0.0,
                        tr58.impossible?1.0:0.0,
                        tr58.repr?1.0:0.0,(double)tr58.splits,
                        tr58.U[0],tr58.U[1],tr58.U[2],
                        tr58.Lw[0],tr58.Lw[1],tr58.Lw[2],
                        (double)SB58.attempted,(double)SB58.result_code,
                        (double)SB58.leaf_count,(double)SB58.split_count,
                        (double)SB58.worst_seg,SB58.worst_t,
                        SB58.attempted?SB58.worst_vq:qnan,
                        SB58.worst_witness,SB58.worst_sigma,SB58.worst_qdot,
                        vqA,sigmaA,
                        finiteWitness58?1.0:0.0,
                        repr58?1.0:0.0,
                        rawQPderivativeUB[0],
                        rawQPderivativeUB[1],
                        rawQPderivativeUB[2]
                    }};
                    R.r58_speed_gate_rows.push_back(r13row);
                    }
                    if(!sc58.certified){
                        ++R.r58_bnb_speed_fail;
                        R.r58_needs_time_law=1;
                        R.r58r6_failed_corr_v_ub=
                            std::max(R.r58r6_failed_corr_v_ub,C58.dub[0]);
                        R.r58r6_failed_corr_a_ub=
                            std::max(R.r58r6_failed_corr_a_ub,C58.dub[1]);
                        R.r58r6_failed_corr_j_ub=
                            std::max(R.r58r6_failed_corr_j_ub,C58.dub[2]);
                        if(fin(SB58.worst_vq)&&
                           SB58.worst_vq>R.r58r6_failed_speed_worst_vq){
                            R.r58r6_failed_speed_worst_vq=SB58.worst_vq;
                            R.r58r6_failed_speed_worst_vl=SB58.worst_vl;
                            R.r58r6_failed_speed_worst_sigma=SB58.worst_sigma;
                            R.r58r6_failed_speed_worst_qdot=SB58.worst_qdot;
                            R.r58r6_failed_speed_witness=SB58.worst_witness;
                        }
                        R.r58_certificate_ms+=
                            std::chrono::duration<double,std::milli>(
                                clk::now()-Tc).count();
                        continue;
                    }

                    double gamma=std::numeric_limits<double>::quiet_NaN();
                    double anchor=std::numeric_limits<double>::quiet_NaN();
                    int hl=0,hs=0;
                    if(!r57_apply_heading_certificate(
                            ref,C58,sc58,t0,t1,gamma,anchor,hl,hs)){
                        ++R.r58_bnb_heading_fail;
                        R.r58_certificate_ms+=
                            std::chrono::duration<double,std::milli>(
                                clk::now()-Tc).count();
                        continue;
                    }

                    auto G58=wholebody(
                        ref,L,obs,t0,tm,R.hold_end,t1,C58,sc58);
                    R.r58_certificate_ms+=
                        std::chrono::duration<double,std::milli>(
                            clk::now()-Tc).count();
                    if(!G58.pass){
                        ++R.r58_bnb_geometry_fail;
                        continue;
                    }

                    // The normal hard candidate pipeline also tests
                    // endpoint whole-body separation. Do NOT confuse an
                    // arbitrary terminal offset with a spatial rejoin.
                    if(bridge16 && !rejoin_safe(ref,L,obs,t1,C58,sc58))
                        continue;
                    R.r58_bnb_feasible_full=1;
                    R.r58_free_c7_constructive=1;
                    R.r58_terminal_offset_hold=1;
                    R.r58_terminal_zero_derivative_order=7;
                    R.r58_best_qp_cost=N58.q.cost;
                    R.r58_terminal_offset_norm_m=norm(C58.terminal_offset);
                    for(int sgn:N58.signs){
                        if(sgn>0)++R.r58_selected_side_plus;
                        else ++R.r58_selected_side_minus;
                    }

                    R.r58_speed_pass=1;
                    R.r58_heading_pass=1;
                    R.r58_geometry_pass=1;
                    R.r58_speed_VL=sc58.VL;
                    R.r58_speed_A=sc58.A;
                    R.r58_speed_J=sc58.J;
                    R.r58_speed_sigma=sc58.sigma;
                    R.r58_speed_qdot=sc58.qdot;
                    R.r58_speed_VQ=sc58.VQ;
                    R.r58_heading_gamma=gamma;
                    R.r58_geometry_min_margin=G58.minMargin;
                    if(bridge16){
                        // Promote the EXACT already-certified C58 bytes.
                        // No reoptimization, sampled validation, or alternative
                        // polynomial representation is allowed here.
                        if(!(C58.exact_c7&&C58.exact_c7_constructive&&
                             C58.heading_c6&&C58.terminal_hold&&
                             !C58.phase_reparameterized&&sc58.certified&&
                             G58.pass&&C58.cp.size()==C58.seg0.size()&&
                             !C58.cp.empty()&&
                             fin(C58.terminal_hold_start)&&
                             C58.terminal_hold_start<=t1&&
                             finite(C58.terminal_offset)))continue;
                        R.cand=std::move(C58);
                        R.speed=sc58;
                        R.geom=G58;
                        R.speed_pass=1;
                        R.rejoin_pass=1; // endpoint safety, not old-clock rejoin
                        R.geom_tests=1;
                        R.r58r16_connected_candidate=1;
                    }
                }

                R.r58_homotopy_ms=
                    std::chrono::duration<double,std::milli>(
                        clk::now()-Th).count();
            }
        }

        R.r58_fixed_rejoin_constraint_count=0;
        R.r58_same_clock_binding_count=0;
        R.r58_horizon_duration_coupling_count=0;
        R.r58_commit_attempted=0;
        R.r58_rejoin_selected=0;
        R.r58_reference_only=1;
        R.r58_shadow_ms=
            std::chrono::duration<double,std::milli>(clk::now()-Tr58).count();
    }
    if(R.r58r16_connected_candidate){
        // Return the certified reference-level native proposal to the SAME
        // MATLAB stage1/2/3 pipeline. Stage deadlines, freshness gates,
        // and commit policy remain completely unchanged.
        R.success=true;
        R.reason=0;
        R.r58_reference_only=1;
        R.r58_commit_attempted=0;
        R.selected_t1=t1; // finite certificate boundary, not forced rejoin
        R.finite_sensing_limit=lim;
        R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
        return R;
    }
    if(L.r58r16_reference_bridge){
        // No unintended family fallback in explicit bridge mode.
        R.reason=58;
        R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
        return R;
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
        R52CandidateGateDiag DG;
        const bool trace52=(cand.family==7);
        if(trace52){
            DG.family=cand.family;DG.exact_c7=cand.exact_c7?1:0;DG.max_ctrl=cand.max_ctrl;
            DG.du=cand.dub[0];DG.da=cand.dub[1];DG.dj=cand.dub[2];
            DG.phase_alpha=(cand.phase_reparameterized&&cand.phase.enabled)?cand.phase.alpha:
                std::numeric_limits<double>::quiet_NaN();
            DG.phase_delay=(cand.phase_reparameterized&&cand.phase.enabled)?cand.phase.delay:
                std::numeric_limits<double>::quiet_NaN();
            DG.actual_rejoin=cand.actual_rejoin_time;DG.limit=lim;
        }
        auto push52=[&](){if(trace52)r52_gate_diag_push(DG);};
        const int r24cfi=(cand.family>=0&&cand.family<8)?cand.family:0;
        ++R.r24e_family_seen[r24cfi];
        if(cand.exact_c7)++R.r24e_family_exact[r24cfi];
        else{
            ++R.r24e_family_exact_reject[r24cfi];
            push52();
            continue;
        }

        // AF7R38: this primary is accepted as C7 only through its
        // constructive exact-real dyadic-lattice proof.
        if(cand.family==7 && cand.trigger_to_free_rejoin &&
                !cand.exact_c7_constructive){
            ++R.r24e_family_exact_reject[r24cfi];
            push52();
            continue;
        }

            SpeedCert sc;
            const SpeedCert bcert=(cand.phase_reparameterized&&cand.phase.enabled)?
                r51_phase_base_certificate(base,L,cand.phase):base;
            if(trace52){
                DG.base_VL=bcert.VL;DG.base_A=bcert.A;DG.base_J=bcert.J;DG.base_sigma=bcert.sigma;
            }
            if(!bcert.certified){
                ++R.r24e_family_speed_fail[r24cfi];
                push52();
                continue;
            }
            if(branch==1) R.primary_min_envelope=std::min(R.primary_min_envelope,cand.max_ctrl);
            auto tr=tighten_stageA(cand,bcert,L.L,lim);
            if(trace52){
                DG.A_Uv=tr.U[0];DG.A_Ua=tr.U[1];DG.A_Uj=tr.U[2];
                DG.A_sigma=bcert.sigma-tr.U[1];
                DG.A_VQ=(DG.A_sigma>0.0)?
                    bcert.VL+tr.U[0]+L.L*(bcert.J+tr.U[2])/DG.A_sigma:
                    std::numeric_limits<double>::infinity();
                DG.A_cert=tr.certified?1:0;DG.A_impossible=tr.impossible?1:0;DG.A_repr=tr.repr?1:0;
            }
            if(cand.family==7){
                R.r31_best_correction_speed_ub=
                    std::min(R.r31_best_correction_speed_ub,cand.dub[0]);
                const double dsig31=bcert.sigma-tr.U[1];
                const double dvq31=(dsig31>0.0)?
                    bcert.VL+tr.U[0]+L.L*(bcert.J+tr.U[2])/dsig31:
                    std::numeric_limits<double>::infinity();
                if(fin(dvq31))
                    R.r31_best_stageA_uav_speed_ub=
                        std::min(R.r31_best_stageA_uav_speed_ub,dvq31);
            }
            if(branch==1){
                const double dsig=bcert.sigma-tr.U[1];
                const double dvq=(dsig>0.0)?
                    bcert.VL+tr.U[0]+L.L*(bcert.J+tr.U[2])/dsig:
                    std::numeric_limits<double>::infinity();
                if(fin(dvq) && dvq<R.primary_min_stageA_vq){
                    R.primary_min_stageA_vq=dvq;
                    R.primary_min_stageA_sigma=dsig;
                    R.primary_min_stageA_U=tr.U;
                }
                if(tr.certified) ++R.primary_stageA_certified_count;
            }
            if(tr.certified){
                sc=stageA_speed(tr,bcert,L.L);
                if(sc.certified&&sc.VQ<lim){
                    cand.dub[0]=tr.U[0];cand.dub[1]=tr.U[1];cand.dub[2]=tr.U[2];
                }else sc.certified=false;
            }
            if(!sc.certified){
                bool wit=false,rp=false;
                R52StageBTrace Bdbg;
                sc=stageB_speed(ref,L,bcert,cand,lim,wit,rp,trace52?&Bdbg:nullptr);
                if(trace52)DG.B=Bdbg;
                if(branch==1){
                    if(wit) ++R.primary_stageB_finite_witness_count;
                    if(rp) ++R.primary_stageB_repr_fail_count;
                    if(sc.certified) ++R.primary_stageB_certified_count;
                }
            }
            if(!sc.certified){
                ++R.r24e_family_speed_fail[r24cfi];
                push52();
                continue;
            }
            if(trace52)DG.speed_pass=1;
            ++R.r24e_family_speed_pass[r24cfi];
            ++R.speed_pass;
            if(branch==1) ++R.primary_speed_pass; else if(branch==2) ++R.fallback_speed_pass; else if(branch==3) ++R.towing_speed_pass;

            // AF7R47: yaw regularity is certified on the TOTAL active-base
            // plus candidate trajectory, after the hard speed certificate has
            // supplied continuous V/A bounds. This supports replanning from
            // an already-curved C7 active reference.
            // AF7R47 absolute smoothness contract:
            // Every exact-C7 candidate, including historical fallbacks, must
            // certify the TOTAL path-tangent yaw as C6 before it can reach
            // rejoin/geometry acceptance. No fallback can silently revert to
            // a non-path-tangent yaw convention.
            ++R.r35_heading_candidate_count;
            double gamma=std::numeric_limits<double>::quiet_NaN();
            double anchor=std::numeric_limits<double>::quiet_NaN();
            int hLeaves=0,hSplits=0;
            R52HeadingTrace Hdbg;
            if(trace52)DG.H.attempted=1;
            if(!r47_apply_heading_certificate(
                    ref,cand,bcert,sc,t0,t1,
                    gamma,anchor,hLeaves,hSplits,trace52?&Hdbg:nullptr)){
                if(trace52)DG.H=Hdbg;
                ++R.r35_heading_reject_count;
                push52();
                continue;
            }
            if(trace52){DG.H=Hdbg;DG.heading_pass=1;DG.heading_gamma=gamma;}
            ++R.r35_heading_pass_count;
            if(cand.family==7 && cand.trigger_to_free_rejoin)
                ++R.r37_heading_pass_count;
            R.r47_heading_leaf_count+=hLeaves;
            R.r47_heading_split_count+=hSplits;
            if(!fin(R.r35_best_heading_gamma_m_s)||
                    gamma>R.r35_best_heading_gamma_m_s)
                R.r35_best_heading_gamma_m_s=gamma;

            if(trace52)DG.rejoin_attempt=1;
            if(!rejoin_safe(ref,L,obs,t1,cand,sc)){push52();continue;}
            if(trace52)DG.rejoin_pass=1;
            ++R.rejoin_pass;++R.geom_tests;
            if(branch==1){++R.primary_rejoin_pass;++R.primary_geom_tests;}
            else if(branch==2){++R.fallback_rejoin_pass;++R.fallback_geom_tests;}
            else if(branch==3){++R.towing_rejoin_pass;++R.towing_geom_tests;}
            if(trace52)DG.geom_attempt=1;
            auto G=wholebody(ref,L,obs,t0,tm,R.hold_end,t1,cand,sc);
            if(trace52){DG.geom_pass=G.pass?1:0;DG.geom_min_margin=G.minMargin;}
            if(G.pass){
                push52();
                R.success=true;R.reason=0;R.cand=std::move(cand);R.speed=sc;R.geom=G;return true;
            }
            push52();
        }
        return false;
    };

    // ------------------------------------------------------------------
    // AF7R55 PRIMARY: BALANCED SOURCE-PHASE / REAL-TIME ALLOCATION
    //                 + EPISODE-LOCAL ORIENTED-ELLIPSOID SIDE SELECTION
    //                 + WORLD-3D MOVING-NORMAL EXACT-C7 CORRECTION.
    //
    // H is not a maneuver duration.
    // No same-clock old-reference position is imposed at rejoin.
    // No phase x rejoin Cartesian-product enumeration is performed.
    // ------------------------------------------------------------------
    ts=clk::now();
    {
        ++R.r51_phase_seed_count;
        if(!ref.repairs.empty())++R.r47_active_base_replan_count;

        Vec3 av{};
        const bool ab=r32_affine_base_exact(
            ref,t0,E.comp.front().start,E.comp.back().end,t1,av);
        R.r32_affine_base_detected=ab?1:0;
        if(!ab)++R.r47_nonaffine_base_count;

        r55_ellipsoid_audit(
            obs,R.r55_obstacle_count,R.r55_min_ellipsoid_axis_m,
            R.r55_max_ellipsoid_axis_m,R.r55_max_rotation_orth_error);

        ++R.r55_phase_build_count;
        R51PhaseSpec phase;
        R55PhaseDiag PD;

        if(!r55_build_balanced_phase(L,E,t0,t1,phase,PD)){
            ++R.r55_phase_build_fail_count;
        }else{
            R.r55_alpha=phase.alpha;
            R.r55_delay_s=phase.delay;
            R.r55_source_unsafe_start_s=PD.s0;
            R.r55_source_unsafe_end_s=PD.s1;
            R.r55_predicted_first_unsafe_t_s=PD.tFirst;
            R.r55_predicted_last_unsafe_t_s=PD.tLast;
            R.r55_pre_unsafe_slack_s=PD.preSlack;
            R.r55_post_unsafe_slack_s=PD.postSlack;
            R.r55_min_temporal_slack_s=PD.minSlack;
            R.r55_phase_entry_transition_s=PD.Te;
            R.r55_phase_exit_transition_s=PD.Tx;

            R.r51_phase_alpha=phase.alpha;
            R.r51_phase_delay_s=phase.delay;
            R.r51_phase_exit_time_s=phase.tE;
            R.r34_selected_alpha=phase.alpha;
            R.r34_selected_delay_s=phase.delay;
            R.r34_selected_hold_start_s=phase.tE;

            const auto SR=r51_phase_jet(phase,phase.tE);
            R.r55_source_phase_at_rejoin_s=SR[0];
            R.r55_rejoin_clock_delay_s=phase.tE-SR[0];

            const SpeedCert phaseBase=r51_phase_base_certificate(base,L,phase);
            if(phaseBase.certified){
                EpisodeCert Ep=r52_build_phase_episode(
                    ref,phase,L,phaseBase,obs,phase.t0,phase.tE);
                ++R.r52_phase_episode_count;

                if(!Ep.ok){
                    ++R.r52_phase_episode_rebuild_fail_count;
                }else if(Ep.comp.empty()||Ep.ep.empty()){
                    ++R.r55_phase_episode_empty_count;

                    Candidate C;
                    C.family=7;C.exact_c7=true;C.exact_c7_constructive=true;
                    C.phase_reparameterized=true;C.phase=phase;
                    C.progress_retimed=true;C.identity_time_law=false;
                    C.progress_scale=phase.alpha;C.schedule_delay=phase.delay;
                    C.retime_entry_T=phase.tA-phase.t0;
                    C.retime_flow_T=phase.tB-phase.tA;
                    C.retime_exit_T=phase.tE-phase.tB;
                    C.terminal_hold=false;C.terminal_hold_start=phase.tE;
                    C.actual_rejoin_time=phase.tE;C.trigger_to_free_rejoin=true;
                    C.seg0={phase.t0};C.seg1={phase.tE};
                    C.cp={Poly(16,Vec3{})};C.dub.fill(0.0);C.max_ctrl=0.0;

                    std::vector<Candidate>one{C};
                    if(certify_candidates(one,true,1)){
                        R.r52_selected_spatial_rejoin_s=R.cand.actual_rejoin_time;
                        R.r37_actual_rejoin_s=R.cand.actual_rejoin_time;
                        R.r35_selected_heading_gamma_m_s=R.cand.heading_gamma;
                        R.r35_selected_heading_anchor_rad=R.cand.heading_anchor_rad;
                        R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();
                        R.tim.candidate=R.tim.synthesis;
                        R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
                        return R;
                    }
                }else{
                    ++R.r55_phase_episode_pass_count;

                    const double h55=std::ldexp(1.0,std::ilogb(L.dt));
                    const double firstUnsafe=Ep.comp.front().start;
                    const double lastUnsafe=Ep.comp.back().end;
                    const double tR=phase.tE;

                    auto K=r37_make_knots(Ep,phase.t0,tR,h55);
                    if(K.size()>=3&&K.front()<firstUnsafe&&K.back()>lastUnsafe){
                        R.r37_support_start_s=K.front();
                        R.r37_actual_rejoin_s=K.back();
                        R.r37_knot_count=(int)K.size();

                        std::vector<double>H55;
                        const auto th55=clk::now();
                        try{H55=r52_spatial_hessian3(K,L.L,L.g);}
                        catch(const std::runtime_error&){H55.clear();}
                        R.tim.hessian+=std::chrono::duration<double,std::milli>(
                            clk::now()-th55).count();

                        if(!H55.empty()){
                            std::vector<Row>rows;
                            std::vector<int>signs;
                            bool rowsOK=true;

                            const auto tcorr=clk::now();
                            for(int ei=0;ei<(int)Ep.ep.size();++ei){
                                auto ER=r55_choose_episode_rows(
                                    ref,phase,L,phaseBase,obs,Ep,ei,K);
                                if(!ER.ok){
                                    rowsOK=false;
                                    ++R.r55_side_both_failed_count;
                                    break;
                                }

                                ++R.r55_mixed_side_episode_count;
                                if(ER.sign>0)++R.r55_side_plus_count;
                                else ++R.r55_side_minus_count;
                                if(ER.plus_ok!=ER.minus_ok)
                                    ++R.r55_side_onebranch_count;

                                const auto&D=(ER.sign>0)?ER.plus_diag:ER.minus_diag;
                                R.r45_tangent_interval_count+=D.intervals;
                                R.r45_zero_offset_interval_count+=D.zero_offset_intervals;
                                R.r45_best_tangent_peak_req_m=
                                    std::min(R.r45_best_tangent_peak_req_m,D.tangent_peak);

                                signs.push_back(ER.sign);
                                R.r55_corridor_row_count+=(int)ER.rows.size();
                                rows.insert(rows.end(),ER.rows.begin(),ER.rows.end());
                            }
                            R.tim.corridor+=std::chrono::duration<double,std::milli>(
                                clk::now()-tcorr).count();

                            if(rowsOK&&!rows.empty()){
                                int nc=0,rc=0,sc=0;
                                if(r53_add_moving_normal_bundle_rows3(
                                        rows,K,ref,phase,nc,rc,sc)){
                                    R.r53_normal_bundle_node_count+=nc;
                                    R.r53_normal_bundle_row_count+=rc;
                                    R.r53_normal_bundle_singular_count+=sc;
                                    if(ab)++R.r53_affine_reduction_count;

                                    const int nv=3*r37_scalar_dofs(K);
                                    const auto tq=clk::now();
                                    QPSol q=solve_qp_active_exchange(H55,nv,rows);
                                    R.tim.qp+=std::chrono::duration<double,std::milli>(
                                        clk::now()-tq).count();

                                    if(q.ok){
                                        ++R.r37_spatial_qp_solutions;
                                        ++R.r52_spatial_qp_solution_count;
                                        R37Path P55=r52_decode_path3(q,K,signs);

                                        if(P55.ok){
                                            try{
                                                Candidate C=r37_build_candidate(P55,h55,Vec3{});
                                                if(C.exact_c7&&C.exact_c7_constructive){
                                                    C.phase_reparameterized=true;
                                                    C.phase=phase;
                                                    C.progress_retimed=true;
                                                    C.identity_time_law=false;
                                                    C.progress_scale=phase.alpha;
                                                    C.schedule_delay=phase.delay;
                                                    C.retime_entry_T=firstUnsafe-phase.t0;
                                                    C.retime_flow_T=lastUnsafe-firstUnsafe;
                                                    C.retime_exit_T=tR-lastUnsafe;
                                                    C.terminal_hold=false;
                                                    C.terminal_hold_start=tR;
                                                    C.actual_rejoin_time=tR;
                                                    C.trigger_to_free_rejoin=true;

                                                    r53_record_shape_diag(P55,C,0);
                                                    ++R.r37_exact_c7_candidates;
                                                    ++R.r38_constructive_c7_count;
                                                    ++R.r51_tangent_candidate_count;
                                                    if(C.legacy_float_endpoint_checker_pass)
                                                        ++R.r38_legacy_float_checker_pass_count;
                                                    else
                                                        ++R.r38_legacy_float_false_reject_count;

                                                    const double vu=phaseBase.VL+C.dub[0];
                                                    if(vu<R.r37_best_total_velocity_ub){
                                                        R.r37_best_total_velocity_ub=vu;
                                                        R.r37_best_accel_ub=C.dub[1];
                                                        R.r37_best_jerk_ub=C.dub[2];
                                                        R.r32_best_pre_speed_ub=vu;
                                                        R.r32_best_progress_scale=phase.alpha;
                                                        R.r32_best_entry_T=C.retime_entry_T;
                                                        R.r32_best_flow_T=C.retime_flow_T;
                                                        R.r32_best_exit_T=C.retime_exit_T;
                                                    }

                                                    std::vector<Candidate>one{C};
                                                    if(certify_candidates(one,true,1)){
                                                        ++R.r51_tangent_certified_count;
                                                        R.r52_selected_spatial_rejoin_s=
                                                            R.cand.actual_rejoin_time;
                                                        R.r37_actual_rejoin_s=
                                                            R.cand.actual_rejoin_time;
                                                        R.r35_selected_heading_gamma_m_s=
                                                            R.cand.heading_gamma;
                                                        R.r35_selected_heading_anchor_rad=
                                                            R.cand.heading_anchor_rad;
                                                        R.r34_selected_delay_s=
                                                            R.cand.schedule_delay;
                                                        R.r34_selected_hold_start_s=
                                                            R.cand.actual_rejoin_time;
                                                        R.r34_selected_alpha=
                                                            R.cand.progress_scale;
                                                        R.r34_selected_velocity_ub=R.speed.VL;
                                                        R.r34_selected_accel_ub=R.speed.A;
                                                        R.r34_selected_jerk_ub=R.speed.J;
                                                        R.tim.synthesis=
                                                            std::chrono::duration<double,std::milli>(
                                                                clk::now()-a).count();
                                                        R.tim.candidate=R.tim.synthesis;
                                                        R.tim.total=
                                                            std::chrono::duration<double,std::milli>(
                                                                clk::now()-T0).count();
                                                        return R;
                                                    }
                                                }else{
                                                    ++R.r31_exact_join_reject;
                                                }
                                            }catch(const std::runtime_error&){}
                                        }
                                    }
                                }else{
                                    R.r53_normal_bundle_singular_count+=std::max(1,sc);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Single production theory: no alternative trajectory-family fallback.
    R.reason=55;
    R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();
    R.tim.candidate=R.tim.synthesis;
    R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
    return R;

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

    // AF7R47: only now, after the active-reference R37/R45 primary and
    // the independent bell family have failed, construct the historical
    // fallback corridor. This changes timing only, never safety semantics.
    if(!ensure_legacy_corridor()){
        R.reason=23;
        R.tim.synthesis=std::chrono::duration<double,std::milli>(clk::now()-a).count();
        R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();
        return R;
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
    // AF7R48 automatic natural rejoin.
    //
    // `t1` is only the finite source/reference-validity boundary.  After the
    // unsafe episode is built once, we form one physics-derived shorter
    // quality trial
    //
    //   T_exit = max(T_entry, T_unsafe_scope) + sqrt(L/g),
    //
    // where sqrt(L/g) is the suspended-load natural time scale.  There is no
    // fixed physical maneuver-duration constant and no arbitrary weighted
    // objective.  The shorter trial is accepted ONLY if the complete unchanged
    // exact-C7 -> yaw-C6 -> speed -> rejoin -> whole-body hard certificate
    // stack closes.  Otherwise the original full source-domain deadline is
    // certified as a fail-safe fallback.
    //
    // Spatial synthesis hierarchy inside plan_fixed_scope():
    //   primary  = exact minimum integrated squared deviation with affine unsafe spans;
    //   fallback = unchanged affine minimum-snap family in legacy cost order.
    // Safety is never an objective term; it remains a hard certificate.
    using clk=std::chrono::steady_clock;
    const auto Tall=clk::now();
    r52_gate_diag_reset();
    r53_shape_diag_reset();
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

    // AF7R51: one source-domain call.  The general phase/tangent primary
    // selects its physical phase exit internally from the actual unsafe
    // episode and suspended-load natural time. Re-running the complete planner
    // at alternate same-clock deadlines is neither needed nor allowed.
    PlanResult Q=plan_fixed_scope(ref,L,base,obs,t0,tm,t1,E);
    OUT=std::move(Q);
    G_c7front_rows.clear();
    OUT.search_end=t1;
    OUT.selected_t1=(OUT.success && fin(OUT.cand.actual_rejoin_time))?
        OUT.cand.actual_rejoin_time:t1;
    OUT.rejoin_attempts=1;
    OUT.rejoin_safe_trials=OUT.success?1:0;
    OUT.rejoin_scope_components=(int)E.comp.size();
    OUT.rejoin_merged_components=std::max(0,(int)E.comp.size()-1);
    OUT.search_speed_pass_total=OUT.speed_pass;
    OUT.search_rejoin_pass_total=OUT.rejoin_pass;
    OUT.search_geom_tests_total=OUT.geom_tests;
    OUT.spatial_envelope_priority=true;
    if(!E.comp.empty())
        OUT.rejoin_selected_duration=OUT.selected_t1-E.comp.back().end;
    OUT.rejoin_seed_duration=OUT.rejoin_selected_duration;
    OUT.rejoin_next_gap=std::numeric_limits<double>::infinity();
    OUT.tim.episode=episodeMs;
    OUT.tim.rejoin_search=0.0;
    OUT.tim.total=std::chrono::duration<double,std::milli>(clk::now()-Tall).count();
    return OUT;
}

} // namespace af7r34
