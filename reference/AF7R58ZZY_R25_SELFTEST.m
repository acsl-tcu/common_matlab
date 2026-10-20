function OUT = AF7R58ZZY_R25_SELFTEST()
% AF7R58ZZY-R25 implementation-equivalence gates.
% No Flight run and no production modification.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R25 | IMPLEMENTATION EQUIVALENCE SELFTEST\n');
fprintf('====================================================================================================\n');

clear af7r77_merge_result_reference_fast_R25;
clear af7r77_compile_correction_packet_R25;
rehash;

assert(exist('af7r77_compile_dcp_mex','file')==3, ...
    '[R25 SELFTEST] af7r77_compile_dcp_mex missing.');
assert(exist('af7r76_compile_correction_packet_R23','file')==2, ...
    '[R25 SELFTEST] R23 reference compiler missing.');

M=synthetic_model();
V=17;
A=af7r76_compile_correction_packet_R23(M,V);
B=af7r77_compile_correction_packet_R25(M,V);

fieldsA=fieldnames(A);
fieldsB=fieldnames(B);
assert(isequal(fieldsA,fieldsB),'[R25 SELFTEST] compiled packet field order/schema differs.');

allExact=true;
for k=1:numel(fieldsA)
    f=fieldsA{k};
    a=A.(f); b=B.(f);
    if isnumeric(a)&&isa(a,'double')&&isnumeric(b)&&isa(b,'double')
        exact=isequal(size(a),size(b)) && ...
            isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
    else
        exact=isequaln(a,b);
    end
    fprintf(' compile %-34s exact = %d\n',f,exact);
    allExact=allExact&&exact;
end
assert(allExact,'[R25 SELFTEST] R25 compiled packet is not bitwise R23-equivalent.');

% Shared-state merge fast path: compare values to original merge_result.
param=struct('state_list',["xd","p"],'num_list',[28,3],'qlist',"");
s=STATE_CLASS(param);
s.set_state('xd',(1:28).','p',[1;2;3]);
r1=struct('state',s,'tag',1,'aux',[4;5]);
r2=struct('state',s,'tag',2,'aux',[7;8]);
legacy=merge_result(r1,r2);
fast=af7r77_merge_result_reference_fast_R25(r1,r2);

assert(isequal(legacy.tag,fast.tag)&&isequal(legacy.aux,fast.aux), ...
    '[R25 SELFTEST] fast merge top-level result differs.');
assert(isequal(legacy.state.list,fast.state.list)&& ...
       isequal(legacy.state.num_list,fast.state.num_list)&& ...
       isequal(legacy.state.type,fast.state.type)&& ...
       isequal(legacy.state.qlist,fast.state.qlist)&& ...
       isequal(legacy.state.xd,fast.state.xd)&& ...
       isequal(legacy.state.p,fast.state.p), ...
    '[R25 SELFTEST] fast merge state values differ from merge_result.');

S=af7r77_merge_result_reference_fast_R25();
assert(S.fast_count>=1&&S.fallback_count==0&&logical(S.last_fast), ...
    '[R25 SELFTEST] shared-state merge fast path did not execute.');

OUT=struct();
OUT.schema='AF7R58ZZY_R25_SELFTEST_V1';
OUT.compile_bitwise_exact=allExact;
OUT.merge_value_exact=true;
OUT.merge_fast_path=true;
OUT.closed=true;

fprintf('----------------------------------------------------------------------------------------------------\n');
fprintf(' compile packet bitwise exact = %d\n',OUT.compile_bitwise_exact);
fprintf(' merge value exact            = %d\n',OUT.merge_value_exact);
fprintf(' merge fast path              = %d\n',OUT.merge_fast_path);
fprintf(' SELFTEST CLOSED              = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
end

function M=synthetic_model()
M=struct();
M.schema='AF7R33_BATCH_FLAT_MODEL_V1';
M.certified=true;
M.root='SYNTHETIC_R25_SELFTEST';
M.nominal_signature='R25_SYNTHETIC';
M.flight_reference_time_origin=5;
M.reference_version=3;
C=repmat(struct('t0',0,'t1',0,'segment_t0',[],'segment_t1',[], ...
    'control_points',{{}}),1,2);

C(1).t0=9.9; C(1).t1=14.2;
C(1).segment_t0=[9.9 10.8 12.1];
C(1).segment_t1=[10.8 12.1 14.2];
C(1).control_points=cell(1,3);
C(2).t0=15.0; C(2).t1=18.0;
C(2).segment_t0=[15.0 16.4];
C(2).segment_t1=[16.4 18.0];
C(2).control_points=cell(1,2);

q=0;
for g=1:2
    for k=1:numel(C(g).control_points)
        q=q+1;
        X=zeros(3,16);
        for j=1:16
            for a=1:3
                % Deterministic finite, non-dyadic data exercises rounding.
                X(a,j)=sin(0.17*q+0.13*j+0.07*a) + ...
                    cos(0.11*q*j+0.03*a)/(1+j+a+q);
            end
        end
        C(g).control_points{k}=X;
    end
end
M.corrections=C;
end
