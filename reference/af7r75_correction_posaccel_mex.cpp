#include "mex.h"
#include <vector>
#include <cmath>
#include <algorithm>

// AF7R58ZZY-R19 correction-only exact batch kernel.
// Inputs:
//   tvec        real double query times
//   corrections scalar/vector struct with fields
//               t0,t1,segment_t0,segment_t1,control_points
// Outputs:
//   D0   3 x N x G position-correction contributions per group
//   D2   3 x N x G acceleration-correction contributions per group
//   mask G x N logical; true exactly where that group contributes
//
// The arithmetic intentionally mirrors af7r73_posaccel_core_R17:
//   Q1=(15/T)*diff(Q0,1,2); Q2=(14/T)*diff(Q1,1,2)
// followed by the same separate-multiply/add de Casteljau recursion used by
// af7r71_decasteljau_batch_mex.  No fast-math/FMA is permitted by installer.

static void fail(const char* id, const char* msg) {
    mexErrMsgIdAndTxt(id, "%s", msg);
}
static void require(bool ok, const char* msg) {
    if (!ok) fail("AF7R75:MEX", msg);
}
static const mxArray* field(const mxArray* s, mwIndex i, const char* name) {
    const mxArray* f = mxGetField(s, i, name);
    if (!f) mexErrMsgIdAndTxt("AF7R75:Field", "Missing field: %s", name);
    return f;
}
static double scalar(const mxArray* a, const char* name) {
    if (!(mxIsDouble(a) && !mxIsComplex(a) && mxGetNumberOfElements(a)==1))
        mexErrMsgIdAndTxt("AF7R75:Scalar", "%s must be a real double scalar.", name);
    double x = mxGetScalar(a);
    if (!std::isfinite(x)) mexErrMsgIdAndTxt("AF7R75:Scalar", "%s must be finite.", name);
    return x;
}
static std::vector<double> dvec(const mxArray* a, const char* name) {
    if (!(mxIsDouble(a) && !mxIsComplex(a)))
        mexErrMsgIdAndTxt("AF7R75:Vector", "%s must be real double.", name);
    mwSize n = mxGetNumberOfElements(a);
    std::vector<double> v(n);
    const double* p = mxGetDoubles(a);
    for (mwSize k=0;k<n;++k) {
        if (!std::isfinite(p[k])) mexErrMsgIdAndTxt("AF7R75:Vector", "%s contains nonfinite values.", name);
        v[k]=p[k];
    }
    return v;
}

static void decasteljau3(const double* Q, mwSize cols, double u, double out[3]) {
    double W[48];
    require(cols>=1 && cols<=16, "Unsupported control-point column count.");
    const mwSize count = 3*cols;
    for (mwSize k=0;k<count;++k) W[k]=Q[k];
    const double omu = 1.0-u;
    mwSize c=cols;
    while (c>1) {
        const mwSize nc=c-1;
        for (mwSize j=0;j<nc;++j) {
            for (mwSize i=0;i<3;++i) {
                const volatile double a = omu * W[i+3*j];
                const volatile double b = u * W[i+3*(j+1)];
                const volatile double z = a+b;
                W[i+3*j]=z;
            }
        }
        c=nc;
    }
    out[0]=W[0]; out[1]=W[1]; out[2]=W[2];
}

static void derivative2_exact(const double* Q0, double T, double Q2[42]) {
    // Q0: 3x16, Q1: 3x15, Q2: 3x14.
    double Q1[45];
    const volatile double f1 = 15.0/T;
    const volatile double f2 = 14.0/T;
    for (mwSize j=0;j<15;++j) {
        for (mwSize i=0;i<3;++i) {
            const volatile double d = Q0[i+3*(j+1)] - Q0[i+3*j];
            const volatile double z = f1*d;
            Q1[i+3*j]=z;
        }
    }
    for (mwSize j=0;j<14;++j) {
        for (mwSize i=0;i<3;++i) {
            const volatile double d = Q1[i+3*(j+1)] - Q1[i+3*j];
            const volatile double z = f2*d;
            Q2[i+3*j]=z;
        }
    }
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    require(nrhs==2, "Inputs required: tvec, corrections.");
    require(nlhs==3, "Outputs required: D0, D2, mask.");
    require(mxIsDouble(prhs[0]) && !mxIsComplex(prhs[0]), "tvec must be real double.");
    require(mxIsStruct(prhs[1]), "corrections must be a struct array.");

    const mwSize N = mxGetNumberOfElements(prhs[0]);
    require(N>=1, "tvec must be nonempty.");
    const double* t = mxGetDoubles(prhs[0]);
    for (mwSize n=0;n<N;++n) require(std::isfinite(t[n]), "tvec contains nonfinite values.");
    const mxArray* C = prhs[1];
    const mwSize G = mxGetNumberOfElements(C);

    mwSize dims[3] = {3,N,G};
    plhs[0]=mxCreateNumericArray(3,dims,mxDOUBLE_CLASS,mxREAL);
    plhs[1]=mxCreateNumericArray(3,dims,mxDOUBLE_CLASS,mxREAL);
    plhs[2]=mxCreateLogicalMatrix(G,N);
    double* D0=mxGetDoubles(plhs[0]);
    double* D2=mxGetDoubles(plhs[1]);
    mxLogical* M=mxGetLogicals(plhs[2]);

    for (mwIndex g=0; g<G; ++g) {
        const double ct0=scalar(field(C,g,"t0"),"t0");
        const double ct1=scalar(field(C,g,"t1"),"t1");
        require(ct1>ct0, "Correction support must satisfy t1>t0.");
        std::vector<double> s0=dvec(field(C,g,"segment_t0"),"segment_t0");
        std::vector<double> s1=dvec(field(C,g,"segment_t1"),"segment_t1");
        require(!s0.empty() && s0.size()==s1.size(), "Correction segment vectors are invalid.");
        const mxArray* CP=field(C,g,"control_points");
        require(mxIsCell(CP) && mxGetNumberOfElements(CP)==s0.size(), "control_points cell count mismatch.");

        const mwSize S=s0.size();
        std::vector<double> Q0all(48*S);
        std::vector<double> Q2all(42*S);
        for (mwSize s=0;s<S;++s) {
            const double dur=s1[s]-s0[s];
            require(std::isfinite(dur) && dur>0.0, "Correction segment duration invalid.");
            const mxArray* q=mxGetCell(CP,s);
            require(q && mxIsDouble(q) && !mxIsComplex(q) && mxGetM(q)==3 && mxGetN(q)==16,
                    "Each control-point cell must be real double 3x16.");
            const double* qp=mxGetDoubles(q);
            for (mwSize k=0;k<48;++k) {
                require(std::isfinite(qp[k]), "Control points contain nonfinite values.");
                Q0all[48*s+k]=qp[k];
            }
            derivative2_exact(&Q0all[48*s],dur,&Q2all[42*s]);
        }

        for (mwSize n=0;n<N;++n) {
            const double tt=t[n];
            if (!(tt>ct0 && tt<ct1)) continue;

            mwSize is=0;
            while (is<S && !(tt<=s1[is])) ++is;
            require(is<S, "Active correction time outside segment authority.");
            if (tt<s0[is]) {
                if (is==0) continue;
                --is;
            }
            const double dur=s1[is]-s0[is];
            volatile double u=(tt-s0[is])/dur;
            if (u<0.0) u=0.0;
            if (u>1.0) u=1.0;
            double y0[3], y2[3];
            decasteljau3(&Q0all[48*is],16,(double)u,y0);
            decasteljau3(&Q2all[42*is],14,(double)u,y2);
            const mwSize base=3*n + 3*N*g;
            D0[base+0]=y0[0]; D0[base+1]=y0[1]; D0[base+2]=y0[2];
            D2[base+0]=y2[0]; D2[base+1]=y2[1]; D2[base+2]=y2[2];
            M[g + G*n]=1;
        }
    }
}
