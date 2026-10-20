#include "mex.h"
#include <cmath>
#include <cstddef>

static void require(bool cond, const char* id, const char* msg) {
    if (!cond) mexErrMsgIdAndTxt(id, "%s", msg);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    require(nrhs == 2, "AF7R77:Args", "Expected cp0 and duration.");
    require(nlhs == 1, "AF7R77:Args", "Exactly one output is required.");

    const mxArray* cpA = prhs[0];
    const mxArray* tA  = prhs[1];
    require(mxIsDouble(cpA) && !mxIsComplex(cpA), "AF7R77:Type", "cp0 must be real double.");
    require(mxIsDouble(tA)  && !mxIsComplex(tA),  "AF7R77:Type", "duration must be real double.");

    const mwSize nd = mxGetNumberOfDimensions(cpA);
    const mwSize* dims = mxGetDimensions(cpA);
    require(nd == 2 || nd == 3, "AF7R77:Shape", "cp0 must be 3x16xS.");
    require(dims[0] == 3 && dims[1] == 16, "AF7R77:Shape", "cp0 must be 3x16xS.");
    const mwSize S = (nd >= 3) ? dims[2] : 1;
    require(mxGetNumberOfElements(tA) == S, "AF7R77:Shape", "duration length must equal S.");

    const double* cp0 = mxGetPr(cpA);
    const double* dur = mxGetPr(tA);

    mwSize odims[4] = {3,16,7,S};
    plhs[0] = mxCreateNumericArray(4, odims, mxDOUBLE_CLASS, mxREAL);
    double* out = mxGetPr(plhs[0]);

    // Local buffers use MATLAB column-major 3x16 layout.
    double q[3*16];
    double qnext[3*16];

    for (mwSize s=0; s<S; ++s) {
        const double T = dur[s];
        require(mxIsFinite(T) && T > 0.0, "AF7R77:Duration", "duration must be finite and positive.");

        const double* src = cp0 + (3*16)*s;
        for (mwSize i=0; i<3*16; ++i) {
            const double v = src[i];
            require(mxIsFinite(v), "AF7R77:Finite", "cp0 contains a non-finite value.");
            q[i] = v;
        }

        // r=0: exact copy, including all 16 columns.
        double* dst0 = out + (3*16)*(0 + 7*s);
        for (mwSize i=0; i<3*16; ++i) dst0[i] = q[i];

        mwSize ncols = 16;
        for (mwSize r=1; r<=6; ++r) {
            const double factor = static_cast<double>(16-r) / T;
            const mwSize newcols = ncols - 1;

            // MATLAB recurrence: Q=(deg/T)*diff(Q,1,2).
            // diff is subtraction first; scaling is a separate multiplication.
            for (mwSize j=0; j<newcols; ++j) {
                for (mwSize k=0; k<3; ++k) {
                    volatile double d = q[k + 3*(j+1)] - q[k + 3*j];
                    qnext[k + 3*j] = factor * d;
                }
            }

            double* dst = out + (3*16)*(r + 7*s);
            for (mwSize j=0; j<newcols; ++j) {
                for (mwSize k=0; k<3; ++k) {
                    dst[k + 3*j] = qnext[k + 3*j];
                }
            }

            for (mwSize i=0; i<3*newcols; ++i) q[i] = qnext[i];
            ncols = newcols;
        }
    }
}
