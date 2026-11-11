// One-invocation REAL-ARITHMETIC sufficient C9 taut-load certificate.
// Important: not a directed-rounding proof; returns proof_numerically_directed=false.
// It NEVER reports physical / actual HLC execution safety.
// The MATLAB wrapper validates all 12 observed shapes and real authority.
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>
#include <string>
#include <stdexcept>
#include <limits>
#include <chrono>

struct V {double x,y,z;};
static V add(V a,V b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static V sub(V a,V b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static V mul(V a,double t){return {a.x*t,a.y*t,a.z*t};}
static double dot(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static double norm(V a){return std::sqrt(dot(a,a));}
static V unit(V a){return mul(a,1.0/norm(a));}
static bool finite(V a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
static V lerp(V a,V b){return mul(add(a,b),0.5);}
struct Shape {
    std::string kind;
    double id=std::numeric_limits<double>::quiet_NaN();
    V c;
    std::array<double,9> R;
    std::array<double,3> dim{};
    std::vector<V> vertices;
    double outer=0;
};
static V local_dir(const Shape&s,V n){
    // MATLAB column-major R, local = R' * n
    return {s.R[0]*n.x+s.R[1]*n.y+s.R[2]*n.z,
            s.R[3]*n.x+s.R[4]*n.y+s.R[5]*n.z,
            s.R[6]*n.x+s.R[7]*n.y+s.R[8]*n.z};
}
static double support(const Shape&s,V n) {
    V w=local_dir(s,n);
    if(!s.vertices.empty()){
        double h=-std::numeric_limits<double>::infinity();
        for(V p:s.vertices)h=std::max(h,dot(p,w));
        return dot(s.c,n)+h;
    }
    const double r=std::hypot(w.x,w.y), L=norm(w);
    const double a=s.dim[0], b=s.dim[1], c=s.dim[2];
    double val=0;
    if(s.kind=="sphere")val=a*L;
    else if(s.kind=="ellipsoid")val=norm({a*w.x,b*w.y,c*w.z});
    else if(s.kind=="box")val=(a*std::abs(w.x)+b*std::abs(w.y)+c*std::abs(w.z))*0.5;
    else if(s.kind=="cylinder")val=a*r+b*std::abs(w.z)*0.5;
    else if(s.kind=="capsule")val=a*L+b*std::abs(w.z)*0.5;
    else if(s.kind=="cone")val=std::max(a*r-b*w.z*0.5,b*w.z*0.5);
    else if(s.kind=="frustum")val=std::max(a*r-c*w.z*0.5,b*r+c*w.z*0.5);
    else if(s.kind=="torus")val=a*r+b*L;
    else throw std::runtime_error("UNSUPPORTED_SHAPE_KIND");
    return dot(s.c,n)+val;
}
// Canonical support-subgradient of all twelve *prepared* convex-hull families.
// A point returned here attains the TRUE shape support in direction n.
// Generic 3-D separation below is a candidate-normal search, NOT a proof
// by itself: test_axis() rechecks every proposed direction against h_O(n).
static double sign0(double x){return (x>0)-(x<0);}
static V world_point(const Shape&s,V a){
    return add(s.c,{s.R[0]*a.x+s.R[3]*a.y+s.R[6]*a.z,
                    s.R[1]*a.x+s.R[4]*a.y+s.R[7]*a.z,
                    s.R[2]*a.x+s.R[5]*a.y+s.R[8]*a.z});
}
static V support_point(const Shape&s,V n){
    const V w=local_dir(s,n);
    const double rho=std::hypot(w.x,w.y), len=norm(w);
    const V xy=rho>0?V{w.x/rho,w.y/rho,0}:V{0,0,0};
    const double a=s.dim[0],b=s.dim[1],c=s.dim[2];
    V u{0,0,0};
    if(!s.vertices.empty()){
        double v=-std::numeric_limits<double>::infinity();
        for(const V&x:s.vertices){
            const double t=dot(x,w);
            if(t>v){v=t;u=x;}
        }
        return world_point(s,u);
    }
    if(s.kind=="sphere"){
        if(len>0)u=mul(w,a/len);
    }else if(s.kind=="ellipsoid"){
        const double v=norm({a*w.x,b*w.y,c*w.z});
        if(v>0)u={a*a*w.x/v,b*b*w.y/v,c*c*w.z/v};
    }else if(s.kind=="box"){
        u={.5*a*sign0(w.x),.5*b*sign0(w.y),.5*c*sign0(w.z)};
    }else if(s.kind=="cylinder"){
        u={a*xy.x,a*xy.y,.5*b*sign0(w.z)};
    }else if(s.kind=="capsule"){
        if(len>0)u=mul(w,a/len);
        u.z+=.5*b*sign0(w.z);
    }else if(s.kind=="cone"){
        u=(a*rho-.5*b*w.z>=.5*b*w.z)?
          V{a*xy.x,a*xy.y,-.5*b}:V{0,0,.5*b};
    }else if(s.kind=="frustum"){
        u=(a*rho-.5*c*w.z>=b*rho+.5*c*w.z)?
          V{a*xy.x,a*xy.y,-.5*c}:V{b*xy.x,b*xy.y,.5*c};
    }else if(s.kind=="torus"){
        if(len>0)u=mul(w,b/len);
        u.x+=a*xy.x;u.y+=a*xy.y;
    }else throw std::runtime_error("UNSUPPORTED_SUPPORT_POINT");
    return world_point(s,u);
}
// Matches the canonical MATLAB Gilbert/Frank-Wolfe direction generator:
// min_{h in H} n.h - support(shape,n) > margin is the only acceptance.
// The normals are derived solely from the current 3-D convex geometry.
// Existing proof node count governs the search, not an invented time cap.
static bool support_axis(const std::vector<V>&H,const Shape&s,
                         V a,double margin,int&calls){
    const double d=norm(a);
    if(!(std::isfinite(d)&&d>0))return false;
    const V n=mul(a,1./d);
    const double h=support(s,n);++calls;
    double v=std::numeric_limits<double>::infinity();
    for(V p:H)v=std::min(v,dot(p,n));
    return std::isfinite(v)&&std::isfinite(h)&&v>h+margin;
}
static bool generic_separator(const std::vector<V>&H,const Shape&s,
                              V reference,double margin,int maxiter,
                              int&calls,int&iterations){
    V mean{0,0,0};
    for(V p:H)mean=add(mean,p);
    mean=mul(mean,1./H.size());
    V a=sub(mean,s.c), b=sub(reference,s.c);
    if(support_axis(H,s,a,margin,calls)||support_axis(H,s,b,margin,calls))return true;
    if(norm(a)==0)a=b;
    if(norm(a)==0){
        double radius=-1;
        for(V p:H){const V d=sub(p,mean);const double d2=dot(d,d);
            if(d2>radius){radius=d2;a=sub(p,s.c);}}
    }
    if(!(std::isfinite(norm(a))&&norm(a)>0))return false;
    size_t idx=0;
    double t=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<H.size();++i){const double d=dot(a,H[i]);if(d<t){t=d;idx=i;}}
    V q=sub(H[idx],support_point(s,a));
    for(int k=0;k<maxiter;++k){
        ++iterations;
        if(support_axis(H,s,q,margin,calls))return true;
        const double qnorm=norm(q);
        if(!(std::isfinite(qnorm)&&qnorm>0))return false;
        // q is a convex combination of Minkowski difference vertices
        // H_i - support_point(shape,n). Therefore norm(q)<=margin
        // witnesses a pair of points in the two convex hulls closer than
        // the required separation margin. No single supporting plane can
        // certify this WHOLE LEAF.  This is NOT a physical collision claim;
        // subdivision/another candidate may still succeed.
        if(qnorm<=margin)return false;
        idx=0;t=std::numeric_limits<double>::infinity();
        for(size_t i=0;i<H.size();++i){const double d=dot(q,H[i]);if(d<t){t=d;idx=i;}}
        const V d=sub(sub(H[idx],support_point(s,q)),q);
        const double d2=dot(d,d);
        if(!(std::isfinite(d2)&&d2>0))return false;
        const double step=std::min(1.,std::max(0.,-dot(q,d)/d2));
        if(!(std::isfinite(step)&&step>0))return false;
        q=add(q,mul(d,step));
    }
    return support_axis(H,s,q,margin,calls);
}
static double cap_max(V c,double rho,V n){
    const double nc=norm(c),ln=norm(n);
    if(!(nc>rho&&ln>0))return std::numeric_limits<double>::quiet_NaN();
    const double a=std::max(-1.,std::min(1.,dot(c,n)/(nc*ln)));
    const double ss=rho/nc, cs=std::sqrt(std::max(0.,1.-ss*ss));
    if(a>=cs)return 1.;
    return std::min(1.,std::max(-1.,a*cs+std::sqrt(std::max(0.,1.-a*a))*ss));
}
static double maxnorm(const std::vector<V>&v){double m=0;for(V x:v)m=std::max(m,norm(x));return m;}
static std::vector<V> derivative(const std::vector<V>&P,int degree,double T){
    std::vector<V>D;D.reserve(P.size()-1);
    const double scale=static_cast<double>(degree)/T;
    for(size_t j=0;j+1<P.size();++j)D.push_back(mul(sub(P[j+1],P[j]),scale));
    return D;
}
static void split_half(const std::vector<V>&P,std::vector<V>&L,std::vector<V>&R){
    int n=static_cast<int>(P.size());
    L.resize(n);R.resize(n);
    std::vector<V> tmp=P;
    for(int k=0;k<n;++k){
        L[k]=tmp[0];R[n-1-k]=tmp[n-1-k];
        for(int j=0;j<n-1-k;++j)tmp[j]=lerp(tmp[j],tmp[j+1]);
    }
}
struct Options{
    double mU,mL,L,minw,thrustlo,thrusthi,fguard,ru,rc,rp,tol;
    int maxdepth,maxnodes;bool ground;
    V g;
};
struct Stats{
    bool certified=false;
    std::string status="UNKNOWN_UNPROVEN";
    int nodes=0,leaves=0,supports=0,outer=0,near=0;
    int generic_separator_calls=0,generic_separator_positives=0,generic_separator_iterations=0;
    double tension=std::numeric_limits<double>::infinity();
    double thrustlo=std::numeric_limits<double>::infinity();
    double thrusthi=0;
    double pad=0;
    double groundclear=std::numeric_limits<double>::infinity();
    // A failed proof is UNKNOWN.  These data locate where a *sufficient*
    // enclosure did not separate the physical bodies.  They never prove
    // collision, and are never sufficient for an acceptance certificate.
    bool witness=false;
    double phase0=std::numeric_limits<double>::quiet_NaN();
    double phase1=std::numeric_limits<double>::quiet_NaN();
    double unresolved_id=std::numeric_limits<double>::quiet_NaN();
    std::array<bool,3> unresolved{{false,false,false}};
    int witness_node_depth=-1;
};
static void record_witness(Stats &s,double a,double b,int depth,
                           double id,const std::array<bool,3>&parts){
    if(s.witness)return; // deterministic first unresolved subtree
    if(!(a>=0&&b<=1&&b>a))return;
    s.witness=true;s.phase0=a;s.phase1=b;s.witness_node_depth=depth;
    s.unresolved_id=id;s.unresolved=parts;
}

static bool test(const std::vector<V>&P,const std::vector<V>&W,
                 const std::vector<V>&W1,const std::vector<V>&W2,
                 double T,int depth,double phase0,double phase1,const std::vector<Shape>&sh,
                 const Options&o,Stats&s){
    if(s.nodes>=o.maxnodes){
        s.status="UNKNOWN_NODE_BUDGET";
        record_witness(s,phase0,phase1,depth,
            std::numeric_limits<double>::quiet_NaN(),{{false,false,false}});
        return false;
    }
    ++s.nodes;
    V c{0,0,0},m{0,0,0};
    for(V x:W)c=add(c,x);
    c=mul(c,1./W.size());
    double rho=0;
    for(V x:W)rho=std::max(rho,norm(sub(x,c)));
    const double nc=norm(c),low=nc-rho;
    bool taut=std::isfinite(low)&&nc>rho&&low>o.minw;
    double flo=-1,fhi=-1;
    bool force=false,geom=false;
    double blocked_id=std::numeric_limits<double>::quiet_NaN();
    std::array<bool,3> blocked_parts{{false,false,false}};
    if(taut){
        const double D1=maxnorm(W1),D2=maxnorm(W2);
        const double qdd=D2/low+3*(D1/low)*(D1/low);
        const double fdelta=o.mU*o.L*qdd;
        flo=std::max(0.,(o.mU+o.mL)*low-fdelta);
        fhi=(o.mU+o.mL)*(nc+rho)+fdelta;
        force=std::isfinite(flo)&&std::isfinite(fhi)&&flo>=o.thrustlo+o.fguard&&fhi<=o.thrusthi-o.fguard;
        if(force){
            for(V x:P)m=add(m,x);
            m=mul(m,1./P.size());
            const V q0=unit(c);
            double minz=std::numeric_limits<double>::infinity();
            double Rload=0.;
            for(V x:P){minz=std::min(minz,x.z);Rload=std::max(Rload,norm(sub(x,m)));}
            const double rt=rho/nc;
            const double chord=rt*std::sqrt(2./(1.+std::sqrt(std::max(0.,1.-rt*rt))));
            s.pad=std::max(s.pad,o.L*chord);
            bool groundpass=true;
            if(o.ground){
                const double cap=cap_max(c,rho,{0,0,1});
                const double gapU=minz-o.L*cap-o.ru-o.tol;
                const double gapC=minz-o.L*std::max(0.,cap)-o.rc-o.tol;
                const double gapP=minz-o.rp-o.tol;
                s.groundclear=std::min(s.groundclear,std::min({gapU,gapC,gapP}));
                groundpass=std::isfinite(gapU)&&std::isfinite(gapC)&&std::isfinite(gapP)&&gapU>0&&gapC>0&&gapP>0;
            }
            geom=groundpass;
            if(geom){
                const double broad=Rload+std::max({o.ru+o.L,o.rc+o.L,o.rp})+o.tol;
                for(const Shape&shape:sh){
                    const double distance=norm(sub(shape.c,m));
                    // Excess numerical guard only in the DISCARD direction.
                    const double guard=64*std::numeric_limits<double>::epsilon()*std::max(1.,std::abs(distance)+std::abs(broad)+std::abs(shape.outer));
                    if(distance>broad+shape.outer+guard){++s.outer;continue;}
                    ++s.near;
                    std::array<bool,3> separated{{false,false,false}};
                    const V normals[3]={sub(m,shape.c),sub(sub(m,mul(q0,o.L)),shape.c),sub(sub(m,mul(q0,o.L/2)),shape.c)};
                    for(const V& raw:normals){
                        const double length=norm(raw);
                        if(!(std::isfinite(length)&&length>0))continue;
                        V n=mul(raw,1./length);
                        const double h=support(shape,n);++s.supports;
                        const double cap=cap_max(c,rho,n);
                        double mn=std::numeric_limits<double>::infinity();
                        for(V p:P)mn=std::min(mn,dot(n,p));
                        if(std::isfinite(cap)&&std::isfinite(h)){
                            separated[0]|=mn-o.L*cap>h+o.ru+o.tol;
                            separated[1]|=mn-o.L*std::max(0.,cap)>h+o.rc+o.tol;
                            separated[2]|=mn>h+o.rp+o.tol;
                        }
                        if(separated[0]&&separated[1]&&separated[2])break;
                    }
                    if(!(separated[0]&&separated[1]&&separated[2])){
                        // The three inexpensive cap normals are sufficient
                        // only in simple scenes.  Before declaring this
                        // leaf unresolved, run the SAME general 3-D support
                        // separator as the canonical MATLAB full-time oracle.
                        // Geometry supports are rechecked at every proposal.
                        const double rt2=rho/nc;
                        const double chord2=rt2*std::sqrt(2./(1.+std::sqrt(std::max(0.,1.-rt2*rt2))));
                        const double pad=o.L*chord2;
                        for(int body=0;body<3;++body){
                            if(separated[body])continue;
                            std::vector<V> H;
                            H.reserve(body==1?2*P.size():P.size());
                            if(body==0){for(V x:P)H.push_back(sub(x,mul(q0,o.L)));}
                            else if(body==1){for(V x:P)H.push_back(x);
                                for(V x:P)H.push_back(sub(x,mul(q0,o.L)));}
                            else H=P;
                            const double margin=(body==0?o.ru:body==1?o.rc:o.rp)+
                                (body==2?0.:pad)+o.tol;
                            int calls=0,iters=0;
                            ++s.generic_separator_calls;
                            const bool passed=generic_separator(H,shape,m,margin,
                                                                o.maxnodes,calls,iters);
                            s.supports+=calls;
                            s.generic_separator_iterations+=iters;
                            if(passed){separated[body]=true;++s.generic_separator_positives;}
                        }
                    }
                    if(!(separated[0]&&separated[1]&&separated[2])){
                        blocked_id=shape.id;
                        for(int k=0;k<3;++k)blocked_parts[k]=!separated[k];
                        geom=false;break;
                    }
                }
            }
        }
    }
    if(taut&&force&&geom){
        ++s.leaves;
        s.tension=std::min(s.tension,o.mL*low);
        s.thrustlo=std::min(s.thrustlo,flo);
        s.thrusthi=std::max(s.thrusthi,fhi);
        return true;
    }
    s.status=!taut?"UNKNOWN_TENSION":!force?"UNKNOWN_THRUST":"UNKNOWN_GEOMETRY";
    if(depth>=o.maxdepth){
        record_witness(s,phase0,phase1,depth,blocked_id,blocked_parts);
        return false;
    }
    std::vector<V>pa,pb,wa,wb,da,db,ea,eb;
    split_half(P,pa,pb);split_half(W,wa,wb);
    split_half(W1,da,db);split_half(W2,ea,eb);
    const double mid=0.5*(phase0+phase1);
    return test(pa,wa,da,ea,T/2,depth+1,phase0,mid,sh,o,s) &&
           test(pb,wb,db,eb,T/2,depth+1,mid,phase1,sh,o,s);
}
static Stats verify(const std::vector<V>&P,double T,const std::vector<Shape>&sh,const Options&o){
    Stats s;
    if(!(T>0&&P.size()>=8&&std::isfinite(T)))return s;
    auto a2=derivative(derivative(P,P.size()-1,T),P.size()-2,T);
    auto a3=derivative(a2,P.size()-3,T);
    auto a4=derivative(a3,P.size()-4,T);
    std::vector<V>W(a2.size()),W1(a3.size()),W2(a4.size());
    for(size_t i=0;i<a2.size();++i)W[i]=sub(o.g,a2[i]);
    for(size_t i=0;i<a3.size();++i)W1[i]=mul(a3[i],-1.);
    for(size_t i=0;i<a4.size();++i)W2[i]=mul(a4[i],-1.);
    s.certified=test(P,W,W1,W2,T,0,0.,1.,sh,o,s);
    if(s.certified)s.status="CERTIFIED_NATIVE_REAL_ARITHMETIC_GEOMETRY_FORCE_TENSION__NO_HLC";
    return s;
}
#ifndef C9_STANDALONE
#include "mex.h"
static const mxArray* field(const mxArray*a,mwIndex i,const char*n){
    const mxArray*f=mxGetField(a,i,n);
    if(!f)throw std::runtime_error(std::string("MISSING_FIELD_")+n);
    return f;
}
static std::vector<double> num(const mxArray*a){
    if(!mxIsDouble(a)||mxIsComplex(a)||mxIsSparse(a))throw std::runtime_error("NOT_REAL_DOUBLE");
    const double*p=mxGetDoubles(a);
    return std::vector<double>(p,p+mxGetNumberOfElements(a));
}
static Shape shape_at(const mxArray*src,mwIndex j){
    Shape S;
    const mxArray*k=field(src,j,"kind");
    char*kindText=mxArrayToString(k);
    if(!kindText)throw std::runtime_error("INVALID_SHAPE_KIND");
    S.kind=kindText;mxFree(kindText);
    auto idv=num(field(src,j,"id"));
    if(idv.size()!=1||!std::isfinite(idv[0]))
        throw std::runtime_error("INVALID_SHAPE_ID");
    S.id=idv[0];
    auto center=num(field(src,j,"center"));
    auto R=num(field(src,j,"R"));
    if(center.size()!=3||R.size()!=9)throw std::runtime_error("INVALID_PREPARED_POSE");
    S.c={center[0],center[1],center[2]};
    std::copy(R.begin(),R.end(),S.R.begin());
    if(!finite(S.c))throw std::runtime_error("NONFINITE_SHAPE_CENTER");
    // The MATLAB canonical prepare validator requires SO(3).  Reject a
    // direct MEX invocation that bypasses that validator, rather than
    // silently producing a spurious support function and false positive.
    for(double x:S.R)if(!std::isfinite(x))throw std::runtime_error("NONFINITE_ROTATION");
    const V c0{S.R[0],S.R[1],S.R[2]};
    const V c1{S.R[3],S.R[4],S.R[5]};
    const V c2{S.R[6],S.R[7],S.R[8]};
    const double det=c0.x*(c1.y*c2.z-c1.z*c2.y)-
                     c0.y*(c1.x*c2.z-c1.z*c2.x)+
                     c0.z*(c1.x*c2.y-c1.y*c2.x);
    if(std::abs(dot(c0,c0)-1.)>1e-8 ||
       std::abs(dot(c1,c1)-1.)>1e-8 ||
       std::abs(dot(c2,c2)-1.)>1e-8 ||
       std::abs(dot(c0,c1))>1e-8 ||
       std::abs(dot(c0,c2))>1e-8 ||
       std::abs(dot(c1,c2))>1e-8 ||
       std::abs(det-1.)>1e-8)
       throw std::runtime_error("INVALID_SO3_ROTATION");
    auto verts=field(src,j,"local_vertices");
    if(mxGetNumberOfElements(verts)>0){
        if(mxGetN(verts)!=3)throw std::runtime_error("INVALID_LOCAL_VERTICES");
        auto a=num(verts);
        size_t n=mxGetM(verts);
        for(size_t i=0;i<n;++i){
            const V pt{a[i],a[i+n],a[i+2*n]};
            if(!finite(pt))throw std::runtime_error("NONFINITE_SHAPE_VERTEX");
            S.vertices.push_back(pt);
        }
    }
    if(!S.vertices.empty()){
        for(V v:S.vertices)S.outer=std::max(S.outer,norm(v));
        return S;
    }
    const mxArray*d=field(src,j,"dim");
    if(!mxIsNumeric(d)||mxIsComplex(d))throw std::runtime_error("INVALID_SHAPE_DIMENSIONS");
    auto v=num(d);
    for(size_t i=0;i<std::min<size_t>(3,v.size());++i)S.dim[i]=v[i];
    const double a=S.dim[0],b=S.dim[1],c=S.dim[2];
    if(!(std::isfinite(a)&&std::isfinite(b)&&std::isfinite(c)&&
         a>=0&&b>=0&&c>=0))throw std::runtime_error("NONFINITE_OR_NEGATIVE_DIMENSIONS");
    if(S.kind=="sphere")S.outer=a;
    else if(S.kind=="ellipsoid")S.outer=std::max({a,b,c});
    else if(S.kind=="box")S.outer=norm({a,b,c})/2;
    else if(S.kind=="cylinder"||S.kind=="cone")S.outer=std::hypot(a,b/2);
    else if(S.kind=="capsule")S.outer=a+b/2;
    else if(S.kind=="frustum")S.outer=std::max(std::hypot(a,c/2),std::hypot(b,c/2));
    else if(S.kind=="torus")S.outer=a+b;
    else throw std::runtime_error("UNSUPPORTED_PREPARED_SHAPE");
    return S;
}
static mxArray* scalar(double x){return mxCreateDoubleScalar(x);}
static mxArray* result(const Stats&s,double elapsed){
    const char*names[]={"certified","status","checked_node_count","checked_leaf_count", "shape_support_calls","outer_rejections","near_obstacle_leaf_checks","minimum_tension_lower_bound_N","thrust_lower_bound_N","thrust_upper_bound_N","ground_minimum_clearance_lower_bound_m","elapsed_ms","numerically_directed_rounding_proven","actual_HLC_tracking_proven","safe_to_commit",
        "unresolved_phase_interval","unresolved_obstacle_ids",
        "unresolved_body_indices","witness_depth",
        "generic_separator_calls","generic_separator_positives",
        "generic_separator_iterations"};
    mxArray*out=mxCreateStructMatrix(1,1,sizeof(names)/sizeof(names[0]),names);
    const double nums[]={double(s.certified),0,double(s.nodes),double(s.leaves),double(s.supports),double(s.outer),double(s.near),s.tension,s.thrustlo,s.thrusthi,s.groundclear,elapsed,0,0,0};
    for(int i=0;i<15;++i)
      mxSetField(out,0,names[i],i==1?mxCreateString(s.status.c_str()):scalar(nums[i]));
    mxArray *interval=mxCreateDoubleMatrix(1,2,mxREAL);
    double *p=mxGetDoubles(interval);
    p[0]=s.phase0;p[1]=s.phase1;
    mxSetField(out,0,"unresolved_phase_interval",interval);
    mxArray *ids=mxCreateDoubleMatrix(1,std::isfinite(s.unresolved_id)?1:0,mxREAL);
    if(std::isfinite(s.unresolved_id))mxGetDoubles(ids)[0]=s.unresolved_id;
    mxSetField(out,0,"unresolved_obstacle_ids",ids);
    int n=0;for(bool x:s.unresolved)if(x)++n;
    mxArray *bodies=mxCreateDoubleMatrix(1,n,mxREAL);
    int j=0;for(int i=0;i<3;++i)if(s.unresolved[i])mxGetDoubles(bodies)[j++]=i+1;
    mxSetField(out,0,"unresolved_body_indices",bodies);
    mxSetField(out,0,"witness_depth",scalar(s.witness_node_depth));
    mxSetField(out,0,"generic_separator_calls",scalar(s.generic_separator_calls));
    mxSetField(out,0,"generic_separator_positives",scalar(s.generic_separator_positives));
    mxSetField(out,0,"generic_separator_iterations",scalar(s.generic_separator_iterations));
    return out;
}
void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    if(nrhs==0&&nlhs==1){plhs[0]=mxCreateString("C9_FUSED_V3_GENERAL_SEPARATOR_20261111");return;}
    if(nrhs!=6||nlhs!=1)mexErrMsgIdAndTxt("C9Fused:Arity","Need P,T,g,parameters,preparedShapes,signature.");
    try{
        if(!mxIsChar(prhs[5]))throw std::runtime_error("PIN_REQUIRED");
        char*pin=mxArrayToString(prhs[5]);
        bool matched=pin&&std::string(pin)=="C9_FUSED_V3_GENERAL_SEPARATOR_20261111";
        if(pin)mxFree(pin);
        if(!matched)throw std::runtime_error("PIN_MISMATCH");
        if(mxGetM(prhs[0])!=3||!(mxGetN(prhs[0])==24||mxGetN(prhs[0])==28))throw std::runtime_error("CONTROL_SIZE");
        auto data=num(prhs[0]);
        auto Tv=num(prhs[1]);auto gg=num(prhs[2]);auto par=num(prhs[3]);
        if(Tv.size()!=1||gg.size()!=3||par.size()!=14||!mxIsStruct(prhs[4]))throw std::runtime_error("INPUT_SIZE");
        for(double x:data)if(!std::isfinite(x))throw std::runtime_error("NONFINITE_CONTROL");
        for(double x:par)if(!std::isfinite(x))throw std::runtime_error("NONFINITE_PARAMETERS");
        auto start=std::chrono::steady_clock::now();
        std::vector<V>P;
        for(size_t j=0;j<data.size()/3;++j)P.push_back({data[3*j],data[3*j+1],data[3*j+2]});
        std::vector<Shape>sh;
        sh.reserve(mxGetNumberOfElements(prhs[4]));
        for(mwIndex i=0;i<mxGetNumberOfElements(prhs[4]);++i)sh.push_back(shape_at(prhs[4],i));
        Options o{par[0],par[1],par[2],par[3],par[4],par[5],par[6],par[7],par[8],par[9],par[10],int(par[11]),int(par[12]),par[13]!=0,{gg[0],gg[1],gg[2]}};
        if(!(o.mU>0&&o.mL>0&&o.L>0&&o.minw>=0&&o.thrustlo>=0&&o.thrusthi>=o.thrustlo&&o.fguard>=0&&o.ru>=0&&o.rc>=0&&o.rp>=0&&o.tol>=0&&o.maxdepth>=0&&o.maxnodes>0&&std::isfinite(Tv[0])))throw std::runtime_error("INVALID_PARAMETERS");
        Stats st=verify(P,Tv[0],sh,o);
        double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        plhs[0]=result(st,ms);
    }catch(const std::exception& e){
        mexErrMsgIdAndTxt("C9Fused:Input","%s",e.what());
    }
}
#else
#include <iostream>
#include <random>
int main(){
    Options o{1.1,.14,2.,0.,0.,20.,0.,.3,.01,.2,0.,9,1024,true,{0,0,-9.80665}};
    std::vector<V>P(28);
    for(int j=0;j<28;++j)P[j]={0,3.+27.*j/27.,3};
    Shape s;s.kind="sphere";s.id=913.;s.c={50,50,50};s.dim={1,0,0};s.outer=1;s.R={1,0,0,0,1,0,0,0,1};
    auto p=verify(P,27,{s},o);
    if(!p.certified||p.nodes!=1||p.leaves!=1){std::cerr<<p.status<<"\n";return 1;}
    s.c={0.,13.,3.};
    auto near=verify(P,27,{s},o);
    if(near.certified||!near.witness||!(near.phase1>near.phase0) ||
       !std::isfinite(near.phase0)||near.unresolved_id!=913.||
       !(near.unresolved[0]||near.unresolved[1]||near.unresolved[2])){
       std::cerr<<"Missing native witness for near sphere\n";return 2;
    }
    std::mt19937 gen(25);
    std::uniform_real_distribution<double> ud(-15,15);
    int n_ok=0;
    for(int i=0;i<2000;++i){
      s.c={ud(gen),ud(gen),ud(gen)+15};
      auto x=verify(P,27,{s},o);
      if(x.certified)++n_ok;
    }
    std::cout<<"PASS FUSED_C9_SOURCE_BOUND_NATIVE_START: clear=1,near_unknown=1,random_certified="<<n_ok<<"/2000\n";
    return 0;
}
#endif
