/*
 * Optional native MEX for ONE selected C sphere-chain broadphase.
 * Compile/test only; never use as physical certification.
 * Accepts precisely the same 15 numeric arguments as
 * wp_six_trigger_C_chain_numeric_kernel_v8.m.
 * mxArray input is read-only. Logical output is [40,P].
 * This file uses only mex.h and standard C, no MATLAB Coder licence.
 */
#include "mex.h"
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <string.h>

static const double* reqDouble(const mxArray *m, const char *name) {
    if (!mxIsDouble(m) || mxIsComplex(m) || mxIsSparse(m))
        mexErrMsgIdAndTxt("LightTrigger:MEXInput", "%s must be dense real double", name);
    return mxGetDoubles(m);
}
static void checkShape(const mxArray *m, mwSize rows, mwSize cols, const char *name) {
    if (mxGetM(m)!=rows || mxGetN(m)!=cols)
        mexErrMsgIdAndTxt("LightTrigger:MEXShape", "%s dimensions mismatch", name);
}
static double positiveGap(double x) {
    /* MATLAB eps(x) for x>=1: next representable positive floating gap. */
    double y=nextafter(x,INFINITY);
    return y-x;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    mwSize M, P, k, j; int b, axis, N;
    const double *D, *L, *rd, *rl, *ad, *al, *tt, *cc, *hw;
    const mxLogical *mask; mxLogical *out;
    double droneRadius,loadRadius,cableRadius,margin;
    if (nrhs!=15 || nlhs!=1)
        mexErrMsgIdAndTxt("LightTrigger:MEXArity", "Expected 15 inputs and 1 output");
    D=reqDouble(prhs[0],"drone positions");
    L=reqDouble(prhs[1],"payload positions");
    if (mxGetM(prhs[0])!=3 || mxGetN(prhs[0])!=41)
        mexErrMsgIdAndTxt("LightTrigger:MEXShape", "Expected 3x41 drone knots");
    checkShape(prhs[1],3,41,"load knots");
    M=40;
    rd=reqDouble(prhs[2],"rhoDrone"); checkShape(prhs[2],1,41,"rhoDrone");
    rl=reqDouble(prhs[3],"rhoLoad"); checkShape(prhs[3],1,41,"rhoLoad");
    ad=reqDouble(prhs[4],"accDrone"); checkShape(prhs[4],1,40,"accDrone");
    al=reqDouble(prhs[5],"accLoad"); checkShape(prhs[5],1,40,"accLoad");
    tt=reqDouble(prhs[6],"time offsets"); checkShape(prhs[6],1,41,"timeOffsets");
    if (!mxIsUint32(prhs[7]) || mxGetNumberOfElements(prhs[7])!=1)
        mexErrMsgIdAndTxt("LightTrigger:MEXSegments", "segments must be uint32 scalar");
    N=(int)(*((const uint32_T*)mxGetData(prhs[7])));
    if (N<1 || N>1024)
        mexErrMsgIdAndTxt("LightTrigger:MEXSegments", "invalid C segment count");
    reqDouble(prhs[8],"droneRadius");reqDouble(prhs[9],"loadRadius");
    reqDouble(prhs[10],"cableRadius");reqDouble(prhs[11],"safetyMargin");
    for (b=8;b<=11;b++)
        if (mxGetNumberOfElements(prhs[b])!=1)
            mexErrMsgIdAndTxt("LightTrigger:MEXRadius", "expected scalar radius/margin");
    droneRadius=mxGetScalar(prhs[8]);loadRadius=mxGetScalar(prhs[9]);
    cableRadius=mxGetScalar(prhs[10]);margin=mxGetScalar(prhs[11]);
    cc=reqDouble(prhs[12],"obstacle centers");
    hw=reqDouble(prhs[13],"obstacle AABB widths");
    if (mxGetM(prhs[12])!=3)
        mexErrMsgIdAndTxt("LightTrigger:MEXShape", "obstacle centers must be 3xP");
    P=mxGetN(prhs[12]);
    checkShape(prhs[13],3,P,"obstacle AABB widths");
    if (!mxIsLogical(prhs[14]) || mxIsSparse(prhs[14]))
        mexErrMsgIdAndTxt("LightTrigger:MEXMask", "coarsePossible must be logical");
    checkShape(prhs[14],M,P,"coarsePossible");
    mask=mxGetLogicals(prhs[14]);
    plhs[0]=mxCreateLogicalMatrix(M,P);
    out=mxGetLogicals(plhs[0]);
    memcpy(out,mask,M*P*sizeof(mxLogical));
    for (k=0;k<M;k++) {
        double step=tt[k+1]-tt[k];
        double eD=fmax(rd[k],rd[k+1])+ad[k]*step*step/8.0;
        double eL=fmax(rl[k],rl[k+1])+al[k]*step*step/8.0;
        double d0=0.0,d1=0.0,chainRadius;
        for (axis=0;axis<3;axis++) {
            double x0=L[axis+3*k]-D[axis+3*k];
            double x1=L[axis+3*(k+1)]-D[axis+3*(k+1)];
            d0+=x0*x0;d1+=x1*x1;
        }
        chainRadius=hypot((fmax(sqrt(d0),sqrt(d1))+eD+eL)/(2.0*N),cableRadius+margin);
        for (j=0;j<P;j++) {
            int allSeparated=1;
            if (!out[k+M*j]) continue;
            for (b=1;b<=N+3;b++) {
                int oneBodySeparated=0;
                double s=0.0,r;
                if (b==1) r=eD+droneRadius+margin;
                else if (b==2) r=eL+loadRadius+margin;
                else { s=(double)(b-3)/(double)N; r=chainRadius+(1-s)*eD+s*eL; }
                for (axis=0;axis<3;axis++) {
                    double v0,v1,center,half,width,guard,scale;
                    if (b==1) {
                        v0=D[axis+3*k];v1=D[axis+3*(k+1)];
                    } else if (b==2) {
                        v0=L[axis+3*k];v1=L[axis+3*(k+1)];
                    } else {
                        v0=(1-s)*D[axis+3*k]+s*L[axis+3*k];
                        v1=(1-s)*D[axis+3*(k+1)]+s*L[axis+3*(k+1)];
                    }
                    center=(v0+v1)*0.5;
                    half=fabs(v1-v0)*0.5+r;
                    width=half+hw[axis+3*j];
                    scale=fmax(1.0,fabs(center)+fabs(cc[axis+3*j])+width);
                    guard=128.0*positiveGap(scale);
                    if (fabs(center-cc[axis+3*j])>width+guard) {
                        oneBodySeparated=1;break;
                    }
                }
                if (!oneBodySeparated) { allSeparated=0; break; }
            }
            if (allSeparated) out[k+M*j]=0;
        }
    }
}
