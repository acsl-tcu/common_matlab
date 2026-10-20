function OUT=AF7R58ZZY_R30_SELFTEST(mexName,classSource)
if nargin<1||isempty(mexName), mexName='af7r79_packet_retarget_mex'; end
if nargin<2||isempty(classSource), classSource=which('REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT'); end
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R30 | SINGLE-BUILD EXACT SELFTEST\n');
fprintf('====================================================================================================\n');
OUT=struct(); OUT.closed=false;

rev=feval(mexName,2);
OUT.packet_mex_revision_exact=ischar(rev)&&strcmp(rev,'AF7R58ZZY_R29_PACKET_RETARGET_V1');

types={'sphere','cylinder','ellipsoid','box','prism','cone','pyramid','custom'};
raw={1.25,[0.8 2.4],[1.1 1.7 2.3],[1 2 3],[1.3 1.9 2.6], ...
    [0.9 1.6],[1.2 2.1],[0.7 1.4 2.2]};
M=numel(types); S=repmat(struct('type','','raw_param',[]),1,M);
Araw=zeros(3,M); kxy=sqrt(3/2); kz=sqrt(3);
for i=1:M
    S(i).type=types{i}; S(i).raw_param=raw{i};
    switch types{i}
        case 'sphere'
            Araw(:,i)=double(raw{i}(1))*ones(3,1);
        case 'cylinder'
            r=double(raw{i}(1)); h=double(raw{i}(2));
            Araw(:,i)=[kxy*r;kxy*r;kz*(h/2)];
        otherwise
            q=double(raw{i}(:));
            if numel(q)<3, q=[q;1+numel(q);2+numel(q)]; end
            Araw(:,i)=abs(q(1:3))+0.25;
    end
end
Dp=0.317; Dq=0.529; Dc=0.411;
[Ap0,Aq0,Ac0]=reference_packet(S,Araw,Dp,Dq,Dc);
[Ap1,Aq1,Ac1]=feval(mexName,1,S,Araw,Dp,Dq,Dc,kxy,kz);
OUT.packet_bitwise_exact=bitsame(Ap0,Ap1)&&bitsame(Aq0,Aq1)&&bitsame(Ac0,Ac1);

assert(~isempty(classSource)&&isfile(classSource),'[R30 SELFTEST] class source missing.');
K=fileread(classSource);
OUT.exact_batch_restored=contains(K,'af7r58i_prepare_batch_model(')&& ...
    ~contains(K,'af7r80_fast_batch_append_R29(');
OUT.schedule_unchanged=contains(K,'pipeline_intervals=3')&& ...
    contains(K,'pipeline_delay_s=3*double(obj.prediction_dt)');
OUT.single_build_core=contains(K,'function [ok,reason,B] = r30_prepare_core_bundle_exact')&& ...
    contains(K,'R.r23_compiled_correction=CP;')&& ...
    contains(K,'''r30_single_build_exact'',true');

p0=strfind(K,'function [ok,reason,B] = r30_prepare_core_bundle_exact');
p1=strfind(K,'function r30_validate_native_binding_once');
assert(numel(p0)==1&&numel(p1)==1&&p1>p0,'[R30 SELFTEST] core boundaries invalid.');
core=K(p0:p1-1);
iCP=onepos(core,'R.r23_compiled_correction=CP;');
iB=onepos(core,'            B=struct( ...');
OUT.compiled_attached_before_bundle=iCP<iB;
OUT.no_post_bundle_nested_reference_mutation= ...
    ~contains(core(iB:end),'B.reference.');

p2=strfind(K,'function [ok,reason,B] = prepare_staged_reference_only_bundle');
p3=strfind(K,'function xd = evaluate_active_reference_xd');
assert(numel(p2)==1&&numel(p3)==1&&p3>p2,'[R30 SELFTEST] prepare boundaries invalid.');
prep=K(p2:p3-1);
iCore=onepos(prep,'if obj.r28_core_only_prepare_mode');
iSuper=onepos(prep,'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED');
OUT.flight_core_before_super_fallback=iCore<iSuper;
prefix=prep(1:iSuper-1);
OUT.no_flight_post_return_B_reference_write= ...
    ~contains(prefix,'B.reference.r23_compiled_correction=CP;');

OUT.closed=OUT.packet_mex_revision_exact&&OUT.packet_bitwise_exact&& ...
    OUT.exact_batch_restored&&OUT.schedule_unchanged&&OUT.single_build_core&& ...
    OUT.compiled_attached_before_bundle&&OUT.no_post_bundle_nested_reference_mutation&& ...
    OUT.flight_core_before_super_fallback&&OUT.no_flight_post_return_B_reference_write;

fprintf(' packet MEX revision exact                  = %d\n',OUT.packet_mex_revision_exact);
fprintf(' packet retarget bitwise exact              = %d\n',OUT.packet_bitwise_exact);
fprintf(' exact AF7R58I batch restored               = %d\n',OUT.exact_batch_restored);
fprintf(' measurement schedule unchanged             = %d\n',OUT.schedule_unchanged);
fprintf(' single-build core present                  = %d\n',OUT.single_build_core);
fprintf(' compiled packet attached before B build    = %d\n',OUT.compiled_attached_before_bundle);
fprintf(' no post-B nested reference mutation        = %d\n',OUT.no_post_bundle_nested_reference_mutation);
fprintf(' Flight core precedes superclass fallback   = %d\n',OUT.flight_core_before_super_fallback);
fprintf(' no Flight post-return B.reference write    = %d\n',OUT.no_flight_post_return_B_reference_write);
fprintf(' SELFTEST CLOSED                            = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R30 SELFTEST] closure failed.');
end

function p=onepos(s,x)
q=strfind(s,x); assert(numel(q)==1,'[R30 SELFTEST] anchor nonunique/missing: %s',x); p=q(1);
end
function [Ap,Aq,Ac]=reference_packet(S,Araw,Dp,Dq,Dc)
M=numel(S); Ap=zeros(3,M); Aq=zeros(3,M); Ac=zeros(3,M);
for i=1:M
    o=S(i); base=Araw(:,i); typ=lower(o.type);
    if strcmp(typ,'cylinder')
        r=double(o.raw_param(1)); h=double(o.raw_param(2));
        Ap(:,i)=[sqrt(3/2)*(r+Dp);sqrt(3/2)*(r+Dp);sqrt(3)*(h/2+Dp)];
        Aq(:,i)=[sqrt(3/2)*(r+Dq);sqrt(3/2)*(r+Dq);sqrt(3)*(h/2+Dq)];
        Ac(:,i)=[sqrt(3/2)*(r+Dc);sqrt(3/2)*(r+Dc);sqrt(3)*(h/2+Dc)];
    elseif strcmp(typ,'sphere')
        r=double(o.raw_param(1));
        Ap(:,i)=(r+Dp)*ones(3,1); Aq(:,i)=(r+Dq)*ones(3,1); Ac(:,i)=(r+Dc)*ones(3,1);
    else
        amin=min(base);
        Ap(:,i)=(1+Dp/amin)*base; Aq(:,i)=(1+Dq/amin)*base; Ac(:,i)=(1+Dc/amin)*base;
    end
end
end
function tf=bitsame(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b))&&isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end
