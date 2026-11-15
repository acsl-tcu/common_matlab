function R=BUILD_AND_BENCH_V8_MEX()
%BUILD_AND_BENCH_V8_MEX Optional and NON-PRODUCTION MATLAB Coder MEX audit.
% Compare numeric kernel MATLAB vs generated C MEX, using 36 obstacles,
% warmup, repeat timing. Compiles into TESTS/MEX_BUILD (not Flight source).
% Does NOT imply the full original HLC/geometry uses MEX. Never install
% compiled code without numerical parity and real-time profiling.
R=struct('status','NOT_RUN','reason','', ...
 'matlabMedianMs',NaN,'mexMedianMs',NaN, ...
 'mexP95Ms',NaN,'mexMaxMs',NaN,'outputsEqual',false);
if exist('codegen','file')~=2
    R.status='SKIPPED_NO_MATLAB_CODER';
    R.reason='MATLAB Coder toolbox unavailable';return
end
out=fullfile(fileparts(mfilename('fullpath')),'MEX_BUILD');
if ~isfolder(out),mkdir(out);end
src=fullfile(fileparts(fileparts(mfilename('fullpath'))),'SIX_WAY');
old=cd(src); guard=onCleanup(@()cd(old));
try
    obsT=coder.typeof(0,[3 128],[false true]);
    maskT=coder.typeof(false,[40 128],[false true]);
    args={zeros(3,41),zeros(3,41),zeros(1,41),zeros(1,41), ...
        zeros(1,40),zeros(1,40),(0:40)*.1,uint32(8), ...
        0.25,0.16,0.015,0.05,obsT,obsT,maskT};
    % Compile in a test-only directory; Coder/MEX compiler required.
    codegen('wp_six_trigger_C_chain_numeric_kernel_v8', ...
        '-config:mex','-args',args,'-o', ...
        'wp_six_trigger_C_chain_numeric_mex_v8','-d',out);
    addpath(out);
    D=repmat([0;0;3],1,41);L=repmat([0;0;2.5],1,41);
    D(1,:)=linspace(0,1,41);L(1,:)=linspace(0,1,41);
    obs=repmat([0;0;2.7],1,36);obs(1,:)=.8+linspace(0,2,36);
    half=repmat([.12;.12;.12],1,36);
    mask=true(40,36);zr=zeros(1,41);za=zeros(1,40);
    q={D,L,zr,zr,za,za,(0:40)*.1,uint32(8), ...
        .25,.16,.015,.05,obs,half,mask};
    a=wp_six_trigger_C_chain_numeric_kernel_v8(q{:});
    b=wp_six_trigger_C_chain_numeric_mex_v8(q{:});
    R.outputsEqual=isequal(a,b);
    if ~R.outputsEqual
        R.status='FAIL_MEX_PARITY';return
    end
    for k=1:8
        wp_six_trigger_C_chain_numeric_kernel_v8(q{:});
        wp_six_trigger_C_chain_numeric_mex_v8(q{:});
    end
    m=zeros(1,60);x=zeros(1,60);
    for k=1:60
        ts=tic;wp_six_trigger_C_chain_numeric_kernel_v8(q{:});m(k)=toc(ts)*1000;
        ts=tic;wp_six_trigger_C_chain_numeric_mex_v8(q{:});x(k)=toc(ts)*1000;
    end
    R.matlabMedianMs=median(m);
    R.mexMedianMs=median(x);
    xs=sort(x);R.mexP95Ms=xs(ceil(.95*numel(xs)));
    R.mexMaxMs=max(x);
    R.status='PASS_NUMERIC_C_CHAIN_MEX_KERNEL_ONLY';
    R.numericParity=TEST_TRIGGER_V8_MEX_NUMERIC_PARITY(250);
    R.fullSelected=BENCH_TRIGGER_V8_FULL_SELECTED_MEX(60);
catch ME
    R.status='MEX_BUILD_OR_RUNTIME_FAILED';
    R.reason=[ME.identifier ': ' ME.message];
end
end
