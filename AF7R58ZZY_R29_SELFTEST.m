function OUT=AF7R58ZZY_R29_SELFTEST(mexName)
% Exact local selftest. No Flight, no production source writes.
if nargin<1||isempty(mexName), mexName='af7r79_packet_retarget_mex'; end
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R29 | STAGE3-LEAN EXACT SELFTEST\n');
fprintf('====================================================================================================\n');
OUT=struct(); OUT.closed=false;

rev=feval(mexName,2);
OUT.packet_mex_revision_exact=ischar(rev)&&strcmp(rev,'AF7R58ZZY_R29_PACKET_RETARGET_V1');

% Packet retarget: exercise every production type family.
types={'sphere','cylinder','ellipsoid','box','prism','cone','pyramid','custom'};
raw={1.25,[0.8 2.4],[1.1 1.7 2.3],[1 2 3],[1.3 1.9 2.6], ...
    [0.9 1.6],[1.2 2.1],[0.7 1.4 2.2]};
M=numel(types); S=repmat(struct('type','','raw_param',[]),1,M);
Araw=zeros(3,M);
kxy=sqrt(3/2); kz=sqrt(3);
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
OUT.packet_shape_exact=isequal(size(Ap1),size(Ap0))&&isequal(size(Aq1),size(Aq0))&&isequal(size(Ac1),size(Ac0));
OUT.packet_bitwise_exact=bitsame(Ap0,Ap1)&&bitsame(Aq0,Aq1)&&bitsame(Ac0,Ac1);

% Fast canonical resource append vs the original prune/count/append algebra.
RC=struct('closed',true,'flight_t0',0,'reference_tend',100,'Sgroup_max',100, ...
    'Gmax',100,'Smax',1000);
base=struct('certified',true,'reference_version',7,'schema','AF7R33_BATCH_FLAT_MODEL_V1', ...
    'corrections',repmat(emptycorr(),1,3));
base.corrections(1)=mkcorr(1,2,2,11);
base.corrections(2)=mkcorr(3,8,3,21);
base.corrections(3)=mkcorr(4,9,2,31);
P=struct();
P.representation_code=2; P.t0=5; P.t1=12; P.r28_native_compiled_exact=true;
P.piecewise_segment_t0=[5 7 9]; P.piecewise_segment_t1=[7 9 12];
P.piecewise_control_points={cpseed(41),cpseed(42),cpseed(43)};
targetVersion=8;

REF=reference_append(base,P,targetVersion,P.t0,RC);
CP=af7r77_compile_correction_packet_R25(REF.model,targetVersion);
[ok,reason,METAmodel,META]=af7r80_fast_batch_append_R29(base,P,CP,targetVersion,P.t0,RC);
OUT.fast_append_ok=ok;
OUT.fast_append_reason=reason;
OUT.fast_append_model_exact=ok&&isequaln(METAmodel,REF.model);
OUT.fast_append_counts_exact=ok&& ...
    isequal(double(META.G_before),double(REF.meta.G_before))&& ...
    isequal(double(META.S_before),double(REF.meta.S_before))&& ...
    isequal(double(META.G_after),double(REF.meta.G_after))&& ...
    isequal(double(META.S_after),double(REF.meta.S_after))&& ...
    isequal(double(META.pruned_groups),double(REF.meta.pruned_groups));

OUT.closed=OUT.packet_mex_revision_exact&&OUT.packet_shape_exact&&OUT.packet_bitwise_exact&& ...
    OUT.fast_append_ok&&OUT.fast_append_model_exact&&OUT.fast_append_counts_exact;

fprintf(' packet MEX revision exact              = %d\n',OUT.packet_mex_revision_exact);
fprintf(' packet retarget shape exact            = %d\n',OUT.packet_shape_exact);
fprintf(' packet retarget bitwise exact          = %d\n',OUT.packet_bitwise_exact);
fprintf(' fast canonical append accepted         = %d\n',OUT.fast_append_ok);
fprintf(' fast append model exact                = %d\n',OUT.fast_append_model_exact);
fprintf(' fast append resource counts exact      = %d\n',OUT.fast_append_counts_exact);
fprintf(' SELFTEST CLOSED                        = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R29 SELFTEST] exact-equivalence closure failed.');
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
a=double(a);b=double(b);tf=isequal(size(a),size(b))&&isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end
function C=emptycorr()
C=struct('t0',0,'t1',1,'segment_t0',0,'segment_t1',1,'control_points',{{zeros(3,16)}});
end
function C=mkcorr(t0,t1,S,seed)
e=linspace(t0,t1,S+1); cp=cell(1,S);
for k=1:S, cp{k}=cpseed(seed+k); end
C=struct('t0',t0,'t1',t1,'segment_t0',e(1:end-1),'segment_t1',e(2:end),'control_points',{cp});
end
function P=cpseed(k)
x=reshape((1:48)+k,3,16); P=double(x)/97;
end
function REF=reference_append(base,P,targetVersion,t_commit,RC)
model=base; keep=true(1,numel(model.corrections));
for g=1:numel(model.corrections), keep(g)=double(model.corrections(g).t1)>double(t_commit); end
npruned=sum(~keep); model.corrections=model.corrections(keep);
G0=numel(model.corrections); S0=0;
for g=1:G0, S0=S0+numel(model.corrections(g).control_points); end
C=struct('t0',double(P.t0),'t1',double(P.t1), ...
    'segment_t0',double(P.piecewise_segment_t0(:).'), ...
    'segment_t1',double(P.piecewise_segment_t1(:).'), ...
    'control_points',{P.piecewise_control_points});
Snew=numel(C.control_points); G1=G0+1; S1=S0+Snew;
meta=struct('contract',RC,'pruned_groups',npruned, ...
    'G_before',G0,'S_before',S0,'G_after',G1,'S_after',S1,'candidate_segments',Snew);
model.corrections(end+1)=C; model.reference_version=double(targetVersion);
model.certified=true; model.schema='AF7R33_BATCH_FLAT_MODEL_V1';
model.resource_certificate=RC; model.resource_state=meta;
REF=struct('model',model,'meta',meta);
end
