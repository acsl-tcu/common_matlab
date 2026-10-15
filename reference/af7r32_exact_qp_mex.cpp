
#include "mex.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <functional>

static inline double epsd(){ return std::numeric_limits<double>::epsilon(); }

static bool chol_lower(const std::vector<double>& H,int n,std::vector<double>& L){
    L.assign(n*n,0.0);
    double scale=1.0;
    for(double v:H) scale=std::max(scale,std::fabs(v));
    const double tol=1024.0*epsd()*scale;
    for(int i=0;i<n;++i){
        for(int j=0;j<=i;++j){
            double s=H[i+n*j];
            for(int k=0;k<j;++k) s-=L[i+n*k]*L[j+n*k];
            if(i==j){
                if(!(std::isfinite(s)&&s>tol)) return false;
                L[i+n*j]=std::sqrt(s);
            }else{
                L[i+n*j]=s/L[j+n*j];
            }
        }
    }
    return true;
}

static void chol_solve(const std::vector<double>& L,int n,
                       const std::vector<double>& b,std::vector<double>& x){
    std::vector<double> y(n,0.0);
    for(int i=0;i<n;++i){
        double s=b[i];
        for(int k=0;k<i;++k) s-=L[i+n*k]*y[k];
        y[i]=s/L[i+n*i];
    }
    x.assign(n,0.0);
    for(int i=n-1;i>=0;--i){
        double s=y[i];
        for(int k=i+1;k<n;++k) s-=L[k+n*i]*x[k];
        x[i]=s/L[i+n*i];
    }
}

static bool dense_solve(std::vector<double> M,std::vector<double> b,int n,
                        std::vector<double>& x){
    double scale=1.0;
    for(double v:M) scale=std::max(scale,std::fabs(v));
    const double tol=2048.0*epsd()*scale;
    for(int k=0;k<n;++k){
        int piv=k; double pv=std::fabs(M[k+n*k]);
        for(int i=k+1;i<n;++i){
            double q=std::fabs(M[i+n*k]);
            if(q>pv){pv=q;piv=i;}
        }
        if(!(std::isfinite(pv)&&pv>tol)) return false;
        if(piv!=k){
            for(int j=k;j<n;++j) std::swap(M[k+n*j],M[piv+n*j]);
            std::swap(b[k],b[piv]);
        }
        double d=M[k+n*k];
        for(int i=k+1;i<n;++i){
            double f=M[i+n*k]/d;
            M[i+n*k]=0.0;
            for(int j=k+1;j<n;++j) M[i+n*j]-=f*M[k+n*j];
            b[i]-=f*b[k];
        }
    }
    x.assign(n,0.0);
    for(int i=n-1;i>=0;--i){
        double s=b[i];
        for(int j=i+1;j<n;++j) s-=M[i+n*j]*x[j];
        x[i]=s/M[i+n*i];
    }
    for(double v:x) if(!std::isfinite(v)) return false;
    return true;
}

static double quad_cost(const std::vector<double>& H,const std::vector<double>& z,int n){
    double s=0.0;
    for(int j=0;j<n;++j)
        for(int i=0;i<n;++i)
            s += 0.5*z[i]*H[i+n*j]*z[j];
    return s;
}

void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    if(nrhs!=3) mexErrMsgIdAndTxt("AF7R32:Arity","Expected H,A,b.");
    if(!mxIsDouble(prhs[0])||mxIsComplex(prhs[0])||
       !mxIsDouble(prhs[1])||mxIsComplex(prhs[1])||
       !mxIsDouble(prhs[2])||mxIsComplex(prhs[2]))
        mexErrMsgIdAndTxt("AF7R32:Input","Inputs must be real double.");

    const int n=(int)mxGetM(prhs[0]);
    if((int)mxGetN(prhs[0])!=n || n<1)
        mexErrMsgIdAndTxt("AF7R32:Input","H must be square.");
    const int m=(int)mxGetM(prhs[1]);
    if((int)mxGetN(prhs[1])!=n || (int)mxGetNumberOfElements(prhs[2])!=m || m<1)
        mexErrMsgIdAndTxt("AF7R32:Input","A/b dimensions invalid.");

    const double* Hp=mxGetPr(prhs[0]);
    const double* Ap=mxGetPr(prhs[1]);
    const double* bp=mxGetPr(prhs[2]);

    std::vector<double> H(n*n), A(m*n), b(m);
    for(int j=0;j<n;++j) for(int i=0;i<n;++i){
        double v=0.5*(Hp[i+n*j]+Hp[j+n*i]);
        if(!std::isfinite(v)) mexErrMsgIdAndTxt("AF7R32:Input","H nonfinite.");
        H[i+n*j]=v;
    }
    for(int j=0;j<n;++j) for(int i=0;i<m;++i){
        double v=Ap[i+m*j];
        if(!std::isfinite(v)) mexErrMsgIdAndTxt("AF7R32:Input","A nonfinite.");
        A[i+m*j]=v;
    }
    for(int i=0;i<m;++i){
        b[i]=bp[i];
        if(!std::isfinite(b[i])) mexErrMsgIdAndTxt("AF7R32:Input","b nonfinite.");
    }

    std::vector<double> L;
    bool spd=chol_lower(H,n,L);

    std::vector<double> bestZ(n,std::numeric_limits<double>::quiet_NaN());
    double bestCost=std::numeric_limits<double>::infinity();
    double bestSlack=std::numeric_limits<double>::quiet_NaN();
    bool ok=false;
    unsigned long long tested=0, feasible=0;

    if(spd){
        // z=0 is the exact optimum if feasible.
        bool z0=true;
        for(int i=0;i<m;++i){
            double scale=std::max(1.0,std::fabs(b[i]));
            double tol=512.0*epsd()*scale;
            if(0.0 < b[i]-tol){ z0=false; break; }
        }
        ++tested;
        if(z0){
            ok=true; ++feasible;
            std::fill(bestZ.begin(),bestZ.end(),0.0);
            bestCost=0.0;
            bestSlack=std::numeric_limits<double>::infinity();
            for(int i=0;i<m;++i) bestSlack=std::min(bestSlack,-b[i]);
        }else{
            // X(:,i)=H^{-1} a_i^T, and G=A H^{-1} A^T.
            std::vector<double> X(n*m,0.0);
            for(int row=0;row<m;++row){
                std::vector<double> ar(n), xr;
                for(int j=0;j<n;++j) ar[j]=A[row+m*j];
                chol_solve(L,n,ar,xr);
                for(int j=0;j<n;++j) X[j+n*row]=xr[j];
            }
            std::vector<double> G(m*m,0.0);
            for(int col=0;col<m;++col){
                for(int row=0;row<m;++row){
                    double s=0.0;
                    for(int j=0;j<n;++j) s+=A[row+m*j]*X[j+n*col];
                    G[row+m*col]=s;
                }
            }

            const int maxK=std::min(n,m);
            std::vector<int> ids;
            std::function<void(int,int,int)> rec;
            rec = [&](int start,int need,int kTarget){
                if(need==0){
                    ++tested;
                    const int k=kTarget;
                    std::vector<double> GS(k*k), bs(k);
                    for(int cj=0;cj<k;++cj){
                        bs[cj]=b[ids[cj]];
                        for(int ri=0;ri<k;++ri)
                            GS[ri+k*cj]=G[ids[ri]+m*ids[cj]];
                    }
                    std::vector<double> lam;
                    if(!dense_solve(GS,bs,k,lam)) return;
                    double lscale=1.0;
                    for(double v:lam) lscale=std::max(lscale,std::fabs(v));
                    for(double v:bs) lscale=std::max(lscale,std::fabs(v));
                    const double tolLam=512.0*epsd()*lscale;
                    for(double v:lam) if(v < -tolLam) return;

                    std::vector<double> z(n,0.0);
                    for(int q=0;q<k;++q)
                        for(int j=0;j<n;++j)
                            z[j]+=X[j+n*ids[q]]*lam[q];
                    for(double v:z) if(!std::isfinite(v)) return;

                    double minSlack=std::numeric_limits<double>::infinity();
                    for(int i=0;i<m;++i){
                        double az=0.0;
                        for(int j=0;j<n;++j) az+=A[i+m*j]*z[j];
                        double slack=az-b[i];
                        double scale=std::max({1.0,std::fabs(az),std::fabs(b[i])});
                        double tol=512.0*epsd()*scale;
                        if(slack < -tol) return;
                        minSlack=std::min(minSlack,slack);
                    }
                    ++feasible;
                    double cost=quad_cost(H,z,n);
                    if(!std::isfinite(cost)) return;
                    double tolObj=512.0*epsd()*std::max({1.0,std::fabs(cost),std::fabs(bestCost)});
                    if(!ok || cost < bestCost-tolObj){
                        ok=true; bestCost=cost; bestZ=z; bestSlack=minSlack;
                    }
                    return;
                }
                for(int i=start;i<=m-need;++i){
                    ids.push_back(i);
                    rec(i+1,need-1,kTarget);
                    ids.pop_back();
                }
            };
            for(int k=1;k<=maxK;++k) rec(0,k,k);
        }
    }

    plhs[0]=mxCreateDoubleMatrix(n,1,mxREAL);
    double* zOut=mxGetPr(plhs[0]);
    for(int i=0;i<n;++i) zOut[i]=bestZ[i];
    if(nlhs>1) plhs[1]=mxCreateDoubleScalar(bestCost);
    if(nlhs>2) plhs[2]=mxCreateDoubleScalar(bestSlack);
    if(nlhs>3) plhs[3]=mxCreateLogicalScalar(ok);
    if(nlhs>4) plhs[4]=mxCreateDoubleScalar((double)tested);
    if(nlhs>5) plhs[5]=mxCreateDoubleScalar((double)feasible);
    if(nlhs>6) plhs[6]=mxCreateLogicalScalar(spd);
}
