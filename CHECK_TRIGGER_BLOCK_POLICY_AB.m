function R=CHECK_TRIGGER_BLOCK_POLICY_AB()
% COMMON_ROOT repeatable parity audit of live-default vs diagnostic block.
% The current branch selects DEFAULT_OFF due to measured paired regression.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_BLOCK_POLICY_AB','cases',0,'failures',0, ...
   'default_block_exclusions',NaN,'experimental_block_exclusions',NaN, ...
   'median_default_ms',NaN,'median_experimental_ms',NaN,'reason','');
old=getenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT');
restore=onCleanup(@()setenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT',old)); %#ok<NASGU>
try
    r=sqrt(0.5);Q=[r -r 0;r r 0;0 0 1];
    s=struct('id',1,'type','ellipsoid','dimensions',[0.5;8;0.6], ...
        'center0',zeros(3,1),'rotation',Q,'velocity',zeros(3,1), ...
        'reference_time_s',0,'bounding_radius_m',8, ...
        'is_stationary',true,'center_norm_m',0, ...
        'outline',zeros(0,2),'vertices',zeros(0,3),'faces',zeros(0,3));
    y=linspace(-6,6,161);
    q=Q*[4+0.05*sin(y);y;0.1*cos(y)];
    l=q+Q*[zeros(1,161);zeros(1,161);-1.5*ones(1,161)];
    setenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT','0');
    [u0,l0,c0,un0,i0]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
        q,l,s,0.2,0.2,0.02,0,true);
    setenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT','1');
    [u1,l1,c1,un1,i1]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
        q,l,s,0.2,0.2,0.02,0,true);
    assert(isequal(u0,u1)&&isequal(l0,l1)&&isequal(c0,c1)&&isequal(un0,un1), ...
        'CHECK_TRIGGER_POLICY:Parity','Default and experiment disagree.');
    assert(i0.sampled_block_support_excluded==0 && ...
       i0.sampled_block_support_probes==0 && ...
       i1.sampled_block_support_excluded>0, ...
        'CHECK_TRIGGER_POLICY:Flags','Wrong block policy or no experiment.');
    R.default_block_exclusions=i0.sampled_block_support_excluded;
    R.experimental_block_exclusions=i1.sampled_block_support_excluded;
    A=zeros(1,60);B=zeros(1,60);
    for k=1:64
        if mod(k,2)==0
            setenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT','1');
            t=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,s,0.2,0.2,0.02,0,true);b=1000*toc(t);
            setenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT','0');
            t=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,s,0.2,0.2,0.02,0,true);a=1000*toc(t);
        else
            setenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT','0');
            t=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,s,0.2,0.2,0.02,0,true);a=1000*toc(t);
            setenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT','1');
            t=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,s,0.2,0.2,0.02,0,true);b=1000*toc(t);
        end
        if k>4,A(k-4)=a;B(k-4)=b;end
    end
    R.median_default_ms=median(A);R.median_experimental_ms=median(B);
    R.cases=161;
    R.status='PASS_SHAPE_AGNOSTIC_BLOCK_DISABLED_BY_DEFAULT';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_BLOCK_POLICY_AB_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_BLOCK_POLICY_AB_RESULT.txt'),'w');
if f>=0
 fprintf(f,'status=%s\ncases=%d\nfailures=%d\n',R.status,R.cases,R.failures);
 fprintf(f,['default_block_exclusions=%g\nexperimental_block_exclusions=%g\n' ...
   'median_default_ms=%.7f\nmedian_experimental_ms=%.7f\nreason=%s\n'], ...
  R.default_block_exclusions,R.experimental_block_exclusions, ...
  R.median_default_ms,R.median_experimental_ms,R.reason);fclose(f);
end
end
