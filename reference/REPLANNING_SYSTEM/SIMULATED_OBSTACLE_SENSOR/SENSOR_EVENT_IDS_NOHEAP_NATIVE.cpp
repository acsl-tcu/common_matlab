/* SENSOR_EVENT_IDS_NOHEAP_NATIVE
 * Deterministic ID and entered/left selection with no std::vector allocations.
 * The MATLAB output types, empty shapes, and value order intentionally match
 * the previous SENSOR_EVENT_IDS_NATIVE implementation.
 * Geometry and obstacle structures are not read or modified here.
 */
#include "mex.h"
#include <cstring>
#include <cstddef>

namespace {
void require(bool okay,const char* id,const char* description) {
    if (!okay) mexErrMsgIdAndTxt(id,"%s",description);
}
mxArray* allocate(const mxArray* original,mwSize count) {
    if (count==0) return mxCreateDoubleMatrix(0,0,mxREAL);
    mxArray* out=mxCreateNumericMatrix(1,count,mxGetClassID(original),mxREAL);
    require(out!=nullptr,"SensorNativeIDs:Memory","ID allocation failed");
    return out;
}
}
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {
    require(nrhs==3 && nlhs==3,"SensorNativeIDs:Usage",
        "Usage: [now,entered,left]=SENSOR_EVENT_IDS_NOHEAP_NATIVE(ids,detected,previous)");
    const mxArray* ids=prhs[0];
    const mxArray* detected=prhs[1];
    const mxArray* previous=prhs[2];
    const mwSize n=mxGetNumberOfElements(detected);
    require(mxIsNumeric(ids) && !mxIsComplex(ids) && !mxIsSparse(ids) &&
            !mxIsLogical(ids) && mxGetNumberOfElements(ids)==n,
            "SensorNativeIDs:IDs","Expected N real numeric IDs");
    require(mxIsLogical(detected) && !mxIsSparse(detected) &&
            mxIsLogical(previous) && !mxIsSparse(previous) &&
            mxGetNumberOfElements(previous)==n,
            "SensorNativeIDs:Mask","Expected N-element full logical masks");
    require(n==0 || (mxGetM(ids)==1 && mxGetM(detected)==1 && mxGetM(previous)==1),
            "SensorNativeIDs:Orientation","ID and mask inputs must be row arrays");
    const mxLogical* now=mxGetLogicals(detected);
    const mxLogical* past=mxGetLogicals(previous);
    mwSize n_now=0,n_enter=0,n_left=0;
    for (mwIndex i=0;i<n;++i) {
        if (now[i]) ++n_now;
        if (now[i] && !past[i]) ++n_enter;
        if (!now[i] && past[i]) ++n_left;
    }
    plhs[0]=allocate(ids,n_now);
    plhs[1]=allocate(ids,n_enter);
    plhs[2]=allocate(ids,n_left);
    // No temporary vectors/heap allocations; all outputs have exact final size.
    if (n==0) return;
    const size_t stride=mxGetElementSize(ids);
    const unsigned char* data=static_cast<const unsigned char*>(mxGetData(ids));
    unsigned char* out0=static_cast<unsigned char*>(mxGetData(plhs[0]));
    unsigned char* out1=static_cast<unsigned char*>(mxGetData(plhs[1]));
    unsigned char* out2=static_cast<unsigned char*>(mxGetData(plhs[2]));
    require(data!=nullptr && (n_now==0||out0!=nullptr) &&
            (n_enter==0||out1!=nullptr) && (n_left==0||out2!=nullptr),
            "SensorNativeIDs:Memory","ID memory unavailable");
    mwIndex a=0,b=0,c=0;
    for (mwIndex i=0;i<n;++i) {
        const unsigned char* element=data+static_cast<size_t>(i)*stride;
        if (now[i]) {
            std::memcpy(out0+static_cast<size_t>(a)*stride,element,stride); ++a;
        }
        if (now[i] && !past[i]) {
            std::memcpy(out1+static_cast<size_t>(b)*stride,element,stride); ++b;
        }
        if (!now[i] && past[i]) {
            std::memcpy(out2+static_cast<size_t>(c)*stride,element,stride); ++c;
        }
    }
}
