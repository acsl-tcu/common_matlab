function V=reference_avoidance_mex_verify()
%REFERENCE_AVOIDANCE_MEX_VERIFY Compare native and MATLAB C-chain kernel.
% Only an exact parity PASS permits use in the official trigger.
% This finite regression is NOT a proof of physical collision avoidance.
name='wp_six_trigger_C_chain_numeric_mex_v8';
V=struct('status','FAIL','tested',0,'mismatches',0,'errorText','');
if exist(name,'file')~=3
    V.errorText='Compiled MEX is not found on the MATLAB path';return
end
try
    % Deterministic fixtures; preserve the user's global RNG state.
    old=rng;cleanup=onCleanup(@()rng(old));rng(4127,'twister'); %#ok<NASGU>
    for caseNo=1:80
        N=uint32(8);
        tt=linspace(0,4,41);
        pD=zeros(3,41);pL=zeros(3,41);
        x=linspace(0,4,41);
        pD(1,:)=0.25*x+0.08*sin(1.4*x+caseNo*.03);
        pD(2,:)=0.1*sin(2*x+caseNo*.07);
        pD(3,:)=3+0.1*cos(x+caseNo*.05);
        pL=pD+repmat([0;0;-2],1,41);
        pL(1,:)=pL(1,:)+0.1*sin(x+caseNo*.02);
        pL(3,:)=pL(3,:)+0.1*(cos(x+caseNo*.02)-1);
        rhoD=0.001+0.01*rand(1,41);rhoL=0.001+0.01*rand(1,41);
        accD=0.02+0.15*rand(1,40);accL=0.02+0.15*rand(1,40);
        P=1+mod(caseNo,13);
        obsC=randn(3,P);obsC(1,:)=1+obsC(1,:)*0.8;
        obsC(3,:)=1.7+obsC(3,:)*0.7;
        obsHalf=0.02+0.35*rand(3,P);
        mask=rand(40,P)>.15;
        args={pD,pL,rhoD,rhoL,accD,accL,tt,N,0.25,0.16,0.015,0.05,obsC,obsHalf,mask};
        a=wp_six_trigger_C_chain_numeric_kernel_v8(args{:});
        b=feval(name,args{:});
        V.tested=V.tested+1;
        if ~isequal(a,b)
            V.mismatches=V.mismatches+1;
            V.errorText=sprintf('Mismatch in deterministic fixture %d',caseNo);
            return
        end
    end
    V.status='PASS_NUMERIC_PARITY';
catch ME
    V.errorText=[ME.identifier ': ' ME.message];
end
end
