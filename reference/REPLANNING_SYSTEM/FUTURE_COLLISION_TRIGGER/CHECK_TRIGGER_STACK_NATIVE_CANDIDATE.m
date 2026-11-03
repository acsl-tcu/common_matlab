function R=CHECK_TRIGGER_STACK_NATIVE_CANDIDATE(logger,agent)
% OFFLINE ONLY. Build separate stack-array candidate MEX and compare it
% with the unchanged pinned production MEX on real recorded Flight states.
% NEVER overwrites TRIGGER_SYMBOLIC_NATIVE_CORE.mexw64 or the parity pin.
% Numerical parity here does NOT certify the original HLC or Flight safety.
R=struct('status','STACK_CANDIDATE_AUDIT_ERROR__NOT_ADOPTED', ...
    'pass',false,'records',0,'max_position_delta_m',Inf, ...
    'max_input_delta',Inf,'max_fused_flag_mismatch',Inf, ...
    'baseline_median_ms',NaN,'candidate_median_ms',NaN, ...
    'candidate_faster',false,'reason','','safe_to_commit',false);
try
    assert(nargin==2&&isobject(logger)&&isobject(agent), ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Inputs', ...
        'Use existing logger and agent from the completed Flight.');
    folder=fileparts(mfilename('fullpath'));
    candidate_dir=fullfile(fileparts(folder),'native_candidates');
    cpp=fullfile(candidate_dir,'TRIGGER_SYMBOLIC_NATIVE_CORE_STACK_CANDIDATE.cpp');
    assert(isfile(cpp), ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Source','Missing original stack candidate cpp.');
    production=which('TRIGGER_SYMBOLIC_NATIVE_CORE');
    assert(~isempty(production) && endsWith(production,['.' mexext]), ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Production', ...
        'Pinned production MEX is not in MATLAB path.');
    test_name='TRIGGER_STACK_CANDIDATE_TEST';
    test_bin=fullfile(candidate_dir,[test_name '.' mexext]);
    do_build=~isfile(test_bin);
    if ~do_build
        src_info=dir(cpp);mex_info=dir(test_bin);
        do_build=mex_info.datenum<src_info.datenum;
    end
    if do_build
        % Compile ONLY the separate output name, outside Flight.
        mex('-R2018a','-O','-outdir',candidate_dir, ...
            '-output',test_name,cpp);
        rehash;
    end
    assert(isfile(test_bin), ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Build','Candidate MEX missing.');
    old_path=path;
    addpath(candidate_dir,'-begin');
    restore_path=onCleanup(@()path(old_path)); %#ok<NASGU>
    resolved=which(test_name);
    assert(strcmpi(char(resolved),char(test_bin)), ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Shadow', ...
        'A different MEX shadows the candidate under test.');
    assert(isequal(feval(test_name,'preload'),true), ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Preload', ...
        'Candidate does not support the original MEX interface.');
    D=logger.Data;
    frames=find(char(D.phase(1:logger.k))=='f');
    assert(~isempty(frames), ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:History', ...
        'No recorded real Flight phase.');
    flight_end_s=double(D.t(frames(end)));
    eligible=frames(double(D.t(frames))+4<=flight_end_s);
    assert(~isempty(eligible), ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Window', ...
        'No full 4-second interval remains within the Flight phase.');
    ix=unique(eligible(round(linspace(1,numel(eligible),min(8,numel(eligible))))));
    P=agent.parameter;
    names=["mass","jx","jy","jz","gravity","loadmass","cableL"];
    hpbase=double(P.get(names));
    modelbase=double(agent.plant.param(:));
    gain=agent.controller.hlc_suspended.param;
    dt=double(gain.dt);
    assert(numel(modelbase)==21 && numel(hpbase)==7 && ...
        all(isfinite(modelbase)) && all(isfinite(hpbase)) && dt>0, ...
        'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Authority', ...
        'Real model/controller parameter order is not supported.');
    ts=(0:floor(4/dt))*dt;
    ts=ts(ts<4);ts=[ts 4];
    nominal=agent.reference.timevarying;
    L=double(P.get("cableL"));
    maxq=0;maxu=0;maxflag=0;
    bench_trials=10;
    baseline_times=zeros(1,numel(ix)*bench_trials);
    candidate_times=zeros(1,numel(ix)*bench_trials);
    report=struct('time_s',{},'q_max_m',{},'l_max_m',{}, ...
        'input_max',{},'flags_mismatch',{},'baseline_ms',{}, ...
        'candidate_ms',{});
    % Actual observed obstacle geometry is NOT needed to compare the
    % baseline and candidate numeric MEX. Fused outer-ball inputs are
    % deterministic throwaway shapes; this is a SOFTWARE parity fixture.
    balls=[1.0 3.0;0.5 -2.0;0.0 1.5;0.7 1.3;1 1];
    radius=[0.11;0.15;0.02];
    for j=1:numel(ix)
        k=ix(j);E=D.agent(1).estimator.result{k}.state;
        ml=double(E.mL);
        assert(isfinite(ml)&&ml>0,'CHECK_TRIGGER_STACK_NATIVE_CANDIDATE:Mass', ...
            'Recorded estimator mass is not valid.');
        keys={'p','q','v','w','pL','vL','pT','wL'};
        x=zeros(24,1);
        for m=1:8
            z=E.(keys{m});
            y=double(z(:));
            assert(isequal(size(y),[3 1])&&all(isfinite(y)));
            x((m-1)*3+(1:3))=y;
        end
        pp=modelbase;pp(20)=ml;
        hp=hpbase(:);
        refs=zeros(28,numel(ts)-1);
        for m=1:numel(ts)-1
            absolute_time=double(D.t(k))+ts(m);
            jet=double(nominal.func(absolute_time-double(nominal.t)));
            jet=jet(:);assert(numel(jet)>=4&&numel(jet)<=28&& ...
                all(isfinite(jet)));
            jet(3)=jet(3)-L;
            refs(1:numel(jet),m)=jet;
        end
        tol=[1e-10;1e-8];
        args={x,ts,refs,pp,hp,gain.F1(:),gain.F2(:),gain.F3(:), ...
            gain.F4(:),dt,tol,balls,radius};
        % Warm BOTH kernels outside the measured region, no Flight effect.
        [q0,l0,u0,st0,f0]=TRIGGER_SYMBOLIC_NATIVE_CORE(args{:});
        [q1,l1,u1,st1,f1]=feval(test_name,args{:});
        q_delta=max(abs(q0(:)-q1(:)));
        l_delta=max(abs(l0(:)-l1(:)));
        u_delta=max(abs(u0(:)-u1(:)));
        flag_delta=nnz(logical(f0(:))~=logical(f1(:)));
        flag_delta=flag_delta+nnz(st0(1:2)~=st1(1:2));
        maxq=max([maxq,q_delta,l_delta]);maxu=max(maxu,u_delta);
        maxflag=max(maxflag,flag_delta);
        base_tick=zeros(1,bench_trials);test_tick=zeros(1,bench_trials);
        for trial=1:bench_trials
            if mod(trial,2)==0
                clock=tic;
                [a,b,c,d,e1]=feval(test_name,args{:}); %#ok<ASGLU>
                test_tick(trial)=1000*toc(clock);
                clock=tic;
                [a,b,c,d,e1]=feval('TRIGGER_SYMBOLIC_NATIVE_CORE',args{:}); %#ok<ASGLU>
                base_tick(trial)=1000*toc(clock);
            else
                clock=tic;
                [a,b,c,d,e1]=feval('TRIGGER_SYMBOLIC_NATIVE_CORE',args{:}); %#ok<ASGLU>
                base_tick(trial)=1000*toc(clock);
                clock=tic;
                [a,b,c,d,e1]=feval(test_name,args{:}); %#ok<ASGLU>
                test_tick(trial)=1000*toc(clock);
            end
        end
        block=(j-1)*bench_trials+(1:bench_trials);
        baseline_times(block)=base_tick;
        candidate_times(block)=test_tick;
        report(j)=struct('time_s',D.t(k),'q_max_m',q_delta, ...
            'l_max_m',l_delta,'input_max',u_delta, ...
            'flags_mismatch',flag_delta, ...
            'baseline_ms',median(base_tick),'candidate_ms',median(test_tick));
    end
    R.records=numel(ix);
    R.max_position_delta_m=maxq;
    R.max_input_delta=maxu;
    R.max_fused_flag_mismatch=maxflag;
    R.baseline_median_ms=median(baseline_times);
    R.candidate_median_ms=median(candidate_times);
    R.candidate_faster=R.candidate_median_ms<R.baseline_median_ms;
    R.parity_pass=maxq<=1e-10 && maxu<=1e-10 && maxflag==0 && ...
        all(isfinite([baseline_times candidate_times]));
    R.pass=R.parity_pass;
    R.details=report;
    if R.pass
        R.status='PASS_STACK_CANDIDATE_VS_PRODUCTION__NOT_ADOPTED';
    else
        R.status='FAIL_STACK_CANDIDATE_PARITY__NOT_ADOPTED';
    end
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_STACK_CANDIDATE_AUDIT_RESULT.mat','R');
fid=fopen('TRIGGER_STACK_CANDIDATE_AUDIT_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nrecords=%d\nposition_delta_m=%.17g\n', ...
        R.status,R.records,R.max_position_delta_m);
    fprintf(fid,'input_delta=%.17g\nflag_mismatch=%g\n', ...
        R.max_input_delta,R.max_fused_flag_mismatch);
    fprintf(fid,'baseline_median_ms=%.8g\ncandidate_median_ms=%.8g\n', ...
        R.baseline_median_ms,R.candidate_median_ms);
    fprintf(fid,'reason=%s\n',R.reason);fclose(fid);
end
end
