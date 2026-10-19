#include "mex.h"
#include <vector>
#include <cmath>

// AF7R63 exact-recursion candidate for MATLAB local decasteljau_batch(P,u).
// It intentionally uses separate multiply/add operations and never changes
// derivative order or mathematical acceptance. Runtime bitwise equivalence
// against the exact extracted MATLAB local function is mandatory before use.

static void require(bool ok, const char* msg) {
    if (!ok) mexErrMsgIdAndTxt("AF7R63:MEX", "%s", msg);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    require(nrhs == 2, "Two inputs required: control matrix and parameter vector.");
    require(nlhs == 1, "Exactly one output is required.");
    require(mxIsDouble(prhs[0]) && !mxIsComplex(prhs[0]), "First input must be real double.");
    require(mxIsDouble(prhs[1]) && !mxIsComplex(prhs[1]), "Second input must be real double.");
    require(mxGetNumberOfDimensions(prhs[0]) == 2, "First input must be 2-D.");

    const mwSize m = mxGetM(prhs[0]);
    const mwSize k = mxGetN(prhs[0]);
    const mwSize n = mxGetNumberOfElements(prhs[1]);
    require(m >= 1 && k >= 1 && n >= 1, "Inputs must be nonempty.");

    const double* P = mxGetDoubles(prhs[0]);
    const double* U = mxGetDoubles(prhs[1]);
    plhs[0] = mxCreateDoubleMatrix(m, n, mxREAL);
    double* Y = mxGetDoubles(plhs[0]);

    std::vector<double> W(m * k);
    for (mwSize s = 0; s < n; ++s) {
        const double u = U[s];
        const double omu = 1.0 - u;
        for (mwSize j = 0; j < k; ++j) {
            for (mwSize i = 0; i < m; ++i) {
                W[i + m*j] = P[i + m*j];
            }
        }
        mwSize cols = k;
        while (cols > 1) {
            const mwSize nextCols = cols - 1;
            for (mwSize j = 0; j < nextCols; ++j) {
                for (mwSize i = 0; i < m; ++i) {
                    const volatile double a = omu * W[i + m*j];
                    const volatile double b = u * W[i + m*(j+1)];
                    const volatile double c = a + b;
                    W[i + m*j] = c;
                }
            }
            cols = nextCols;
        }
        for (mwSize i = 0; i < m; ++i) {
            Y[i + m*s] = W[i];
        }
    }
}
