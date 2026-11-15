function R=TEST_TRIGGER_V8_MEX_NUMERIC_PARITY(n)
%TEST_TRIGGER_V8_MEX_NUMERIC_PARITY Randomized raw preclear parity.
% Validates MATLAB vectorized C, MATLAB scalar numeric, optional MEX.
if nargin<1,n=250;end
assert(exist('wp_six_trigger_C_chain_numeric_mex_v8','file')==3, ...
    'V8:MEX','Compiled native/Coder MEX must be on MATLAB path');
rng(20261113,'twister');
count=0;
for i=1:n
    P=randi([1,36]);N=randi([4,12]);
    pD=cumsum(.02*randn(3,41),2);pD(3,:)=pD(3,:)+3;
    qq=randn(3,41);qq=qq./vecnorm(qq,2,1);
    pL=pD+2*qq;
    rhoD=.02*rand(1,41);rhoL=.02*rand(1,41);
    accD=.05*rand(1,40);accL=.05*rand(1,40);
    center=repmat(pL(:,randi([1 41])),1,P)+randn(3,P);
    half=.02+.25*rand(3,P);
    possible=rand(40,P)>.5;
    time=(0:40)*.1;
    cfg=struct('systemMode','C','cableSegments',N, ...
        'droneRadius',.25,'loadRadius',.16,'cableRadius',.015, ...
        'safetyMargin',.05);
    known=struct('center',center,'orientedAabbHalfWidth',half);
    F=struct('timeOffsets',time,'pDrone',pD,'pLoad',pL, ...
        'rhoDrone',rhoD,'rhoLoad',rhoL, ...
        'accDrone',accD,'accLoad',accL);
    I=1:P;
    [expected,~]=wp_six_trigger_C_chain_aabb_v8( ...
        known,cfg,F,I,possible);
    args={pD,pL,rhoD,rhoL,accD,accL,time,uint32(N), ...
        cfg.droneRadius,cfg.loadRadius,cfg.cableRadius, ...
        cfg.safetyMargin,center,half,possible};
    scalar=wp_six_trigger_C_chain_numeric_kernel_v8(args{:});
    compiled=wp_six_trigger_C_chain_numeric_mex_v8(args{:});
    assert(isequal(expected,scalar) && isequal(expected,compiled), ...
        'V8:MEXParity','Randomized C-chain result differs at trial %d',i);
    count=count+numel(expected);
end
R=struct('status','PASS_VECTOR_SCALAR_MEX_PARITY', ...
    'randomTrials',n,'comparedPairs',count);
end
