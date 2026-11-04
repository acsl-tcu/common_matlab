function A=CHECK_TRIGGER_EARLIEST_SAMPLED_CONTACT()
% Two independent algorithms: complete per-shape scan versus truncated.
% This checks NOMINAL sampled first hit, NOT continuous flight safety.
A=struct('status','NOT_COMPLETED','cases',0,'failures',0,'reason','');
try
    old_rng=rng;
    restore_rng=onCleanup(@()rng(old_rng)); %#ok<NASGU>
    rng(42,'twister');
    types={'sphere','box','cylinder','ellipsoid'};
    for r=1:80
        n=20+randi(12);
        u=3*randn(3,n);l=u+[zeros(2,n);-2*ones(1,n)];
        grid=(0:n-1)*0.025;
        S=repmat(struct('id',1,'type','sphere','dimensions',1, ...
            'center0',zeros(3,1),'rotation',eye(3), ...
            'bounding_radius_m',1,'is_stationary',true),1,4);
        for j=1:numel(S)
            S(j).id=j;
            S(j).type=types{j};
            switch j
                case 1, D=0.75;
                case 2, D=[1.2;1.8;2.0];
                case 3, D=[0.7;1.5];
                otherwise,D=[0.7;1.0;1.3];
            end
            S(j).dimensions=D;
            S(j).center0=3*randn(3,1);
            S(j).bounding_radius_m=5; % conservative for all test solids
        end
        R=TRIGGER_EARLIEST_SAMPLED_CONTACT(u,l,grid,S,0.2,0.2,0.03,0);
        best=n+1;E=[];
        for j=1:numel(S)
            [a,b,c]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                u,l,S(j),0.2,0.2,0.03,0,true);
            ii=find(a|b|c,1,'first');
            if isempty(ii),continue;end
            if ii<best,best=ii;E=S(j).id;
            elseif ii==best,E(end+1)=S(j).id;end %#ok<AGROW>
        end
        if best<=n
            assert(R.replan_requested && R.first_sample_index==best && ...
                isequal(R.first_obstacle_ids,E) && ...
                isequal(R.first_ttc_s,grid(best)));
        else
            assert(~R.replan_requested && isempty(R.first_obstacle_ids));
        end
        A.cases=A.cases+1;
    end
    A.status='PASS_NOMINAL_EARLIEST_SAMPLE_PARITY__NOT_SAFETY';
catch ME
    A.failures=A.failures+1;
    A.reason=[ME.identifier ': ' ME.message];
end
fid=fopen('TRIGGER_EARLIEST_SAMPLED_CONTACT_RESULT.txt','w');
assert(fid>=0);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'status=%s\ncases=%d\nfailures=%d\nreason=%s\n', ...
    A.status,A.cases,A.failures,A.reason);
end
