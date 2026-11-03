function R=CHECK_TRIGGER_NATIVE_STACK_CANDIDATE(logger,agent)
% OFFLINE ONLY. Builds a *differently named* candidate MEX and compares
% identical supplied inputs with the already installed verified MEX.
% Does not modify/replace the active MEX, parity pin, HLC, EKF or plant.
% No safety certification; developer must still run original HLC+plant audit
% and explicitly re-authorize any new executable revision after parity.
assert(isobject(logger)&&isobject(agent),'CHECK_TRIGGER_NATIVE_STACK_CANDIDATE:Input', ...
    'Pass the completed Flight LOGGER and agent.');
root=fileparts(mfilename('fullpath'));
src=fullfile(root,'native_candidates','TRIGGER_SYMBOLIC_NATIVE_CORE_STACK_CANDIDATE.cpp');
assert(isfile(src),'CHECK_TRIGGER_NATIVE_STACK_CANDIDATE:Source','Candidate source missing.');
assert(~isempty(which('TRIGGER_SYMBOLIC_NATIVE_CORE')), ...
    'CHECK_TRIGGER_NATIVE_STACK_CANDIDATE:Original','Verified original MEX missing.');
mex('-R2018a','-O','-outdir',root,'-output','TRIGGER_STACK_NATIVE_CANDIDATE',src);
rehash;
D=logger.Data;
idx=find(char(D.phase(1:logger.k))=='f');
assert(~isempty(idx),'CHECK_TRIGGER_NATIVE_STACK_CANDIDATE:History','No Flight records.');
idx=idx(unique(round(linspace(1,numel(idx),min(numel(idx),12)))));
name_order=["mass","jx","jy","jz","gravity","loadmass","cableL"];
hp=double(agent.parameter.get(name_order));
pp0=double(agent.plant.param(:));
g=agent.controller.hlc_suspended.param;
dt=double(g.dt);
assert(numel(hp)==7&&numel(pp0)==21&&dt>0, ...
    'CHECK_TRIGGER_NATIVE_STACK_CANDIDATE:Binding','Live model parameters are invalid.');
tol=[1e-10;1e-8]; % For experimental comparative test only.
keys={'p','q','v','w','pL','vL','pT','wL'};
complete=0;full=0;failures={}; max_abs_error=0;
for j=1:numel(idx)
    k=idx(j);
    try
        E=D.agent(1).estimator.result{k}.state;
        ref=D.agent(1).reference.result{k}.state;
        ml=double(E.mL);
        assert(isfinite(ml)&&isscalar(ml)&&ml>0,'Invalid recorded EKF mass.');
        x=zeros(24,1);
        for a=1:8
            v=double(E.(keys{a}));
            assert(isequal(size(v),[3 1])&&all(isfinite(v)),'Invalid recorded state.');
            x(3*a-2:3*a)=v;
        end
        rv=double(ref.xd(:));
        assert(numel(rv)>=4&&numel(rv)<=28&&all(isfinite(rv)),'Invalid reference.');
        jet=zeros(28,1);jet(1:numel(rv))=rv;
        hp_now=hp;hp_now(6)=ml;
        pp=pp0;pp(20)=ml;
        args={x,[0 dt],jet,pp,hp_now(:),g.F1(:),g.F2(:),g.F3(:),g.F4(:),dt,tol};
        [u1,l1,c1,a1]=TRIGGER_SYMBOLIC_NATIVE_CORE(args{:});
        [u2,l2,c2,a2]=TRIGGER_STACK_NATIVE_CANDIDATE(args{:});
        assert(isequaln(u1,u2)&&isequaln(l1,l2)&&isequaln(c1,c2) && ...
            isequaln(a1(1:2),a2(1:2)),'Native output mismatch.');
        max_abs_error=max(max_abs_error,max(abs([u1(:)-u2(:);l1(:)-l2(:);c1(:)-c2(:)])));
        complete=complete+1;
        % An additional 4 s *identical-input A/B* test, NOT a future
        % reference or a physical safety proof. Freeze the recorded jet.
        if full<2
            h=4; n=floor(h/dt); t=(0:n)*dt;t=t(t<h);t=[t,h];
            r=repmat(jet,1,numel(t)-1);
            args4={x,t,r,pp,hp_now(:),g.F1(:),g.F2(:),g.F3(:),g.F4(:),dt,tol};
            [a,b,c,d]=TRIGGER_SYMBOLIC_NATIVE_CORE(args4{:});
            [aa,bb,cc,dd]=TRIGGER_STACK_NATIVE_CANDIDATE(args4{:});
            assert(isequaln(a,aa)&&isequaln(b,bb)&&isequaln(c,cc) && ...
                isequaln(d(1:2),dd(1:2)),'Full-horizon native A/B mismatch.');
            full=full+1;
        end
    catch ME
        failures{end+1}=sprintf('sample %d: %s | %s',k,ME.identifier,ME.message); %#ok<AGROW>
    end
end
R=struct('status','INCOMPLETE_STACK_CANDIDATE_PARITY', ...
    'one_step_bitwise_pass_count',complete,'full_horizon_bitwise_pass_count',full, ...
    'sample_attempt_count',numel(idx),'max_abs_error',max_abs_error, ...
    'failures',{failures},'production_mex_modified',false, ...
    'original_model_parity_verified_for_candidate',false, ...
    'continuous_time_safe',false);
if complete==numel(idx)&&full>=2
    R.status='PASS_NATIVE_TO_NATIVE_BITWISE_ONLY__NOT_ORIGINAL_HLC_SAFETY';
end
save(fullfile(root,'TRIGGER_STACK_CANDIDATE_RESULT.mat'),'R','-v7');
fid=fopen(fullfile(root,'TRIGGER_STACK_CANDIDATE_RESULT.txt'),'wt');
if fid>=0
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\none_step=%d/%d full_horizon=%d max_abs_error=%.17g\n', ...
        R.status,complete,numel(idx),full,max_abs_error);
    for k=1:numel(failures),fprintf(fid,'%s\n',failures{k});end
end
end
