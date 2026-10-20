function OUT=AF7R58ZZY_R28_SELFTEST(mexName,classPath)
% AF7R58ZZY-R28 exact native representation selftest.
% Uses deterministic values generated inside this function only.
% No Flight, no external MAT/capture/test-data files.
if nargin<1||isempty(mexName), mexName='af7r36_single_core_mex'; end
mexName=char(mexName);
if nargin<2, classPath=''; end

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R28 | NATIVE-ONCE EXACT REPRESENTATION SELFTEST\n');
fprintf('====================================================================================================\n');
OUT=struct(); OUT.schema='AF7R58ZZY_R28_SELFTEST_V1'; OUT.closed=false;

rev=feval(mexName,5);
OUT.revision_exact=ischar(rev)&&strcmp(rev,'AF7R58ZZY_R28_NATIVE_ONCE_V1');
fprintf(' native revision exact                  = %d\n',OUT.revision_exact);
assert(OUT.revision_exact,'[R28 SELFTEST] native revision mismatch.');

%% A. Exact D0..D6 recurrence, bitwise against the already-closed R25 MEX.
assert(exist('af7r77_compile_dcp_mex','file')==3, ...
    '[R28 SELFTEST] R25 derivative reference MEX missing.');
S=4;
v=reshape(1:(3*16*S),3,16,S);
cp0=sin(v*0.137)+cos(v*0.031).*2.^mod(v,5)-0.25;
dur=[0.125 0.375 1.7 3.125];
Dref=double(af7r77_compile_dcp_mex(cp0,dur));
Dnew=double(feval(mexName,3,cp0,dur));
OUT.derivative_shape_exact=isequal(size(Dnew),size(Dref));
OUT.derivative_bitwise_exact=OUT.derivative_shape_exact&& ...
    isequal(typecast(Dnew(:),'uint64'),typecast(Dref(:),'uint64'));
fprintf(' derivative shape exact                 = %d\n',OUT.derivative_shape_exact);
fprintf(' derivative D0..D6 bitwise exact        = %d\n',OUT.derivative_bitwise_exact);
assert(OUT.derivative_bitwise_exact,'[R28 SELFTEST] native derivative recurrence mismatch.');

%% B. Full prune+append packet equivalence against R27 reference implementation.
assert(exist('af7r77_compile_correction_packet_R25','file')==2, ...
    '[R28 SELFTEST] R25 full packet helper missing.');
assert(exist('af7r78_incremental_compile_R27','file')==2, ...
    '[R28 SELFTEST] R27 exact incremental reference helper missing.');

C=repmat(struct('t0',0,'t1',0,'segment_t0',[],'segment_t1',[], ...
    'control_points',{{}}),1,3);
% group 1 expires strictly before commit
C(1)=mkgroup(0.25,1.25,[0.25 0.75],[0.75 1.25],11);
% group 2 ends exactly at commit: exact AF7R58I rule t1<=commit must prune it
C(2)=mkgroup(1.25,2.50,[1.25 2.00],[2.00 2.50],23);
% group 3 survives
C(3)=mkgroup(1.75,4.25,[1.75 2.75 3.50],[2.75 3.50 4.25],37);
M=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
    'corrections',C,'reference_version',7);
base=af7r77_compile_correction_packet_R25(M,7);

P=struct(); P.t0=2.5; P.t1=5.25;
P.piecewise_segment_t0=[2.5 3.0 4.0];
P.piecewise_segment_t1=[3.0 4.0 5.25];
P.piecewise_control_points=cell(1,3);
cpN=zeros(3,16,3);
for k=1:3
    q=mkcp(100+7*k); P.piecewise_control_points{k}=q; cpN(:,:,k)=q;
end
expected=af7r78_incremental_compile_R27(base,P,8,P.t0);
actual=feval(mexName,4,base,P.piecewise_segment_t0,P.piecewise_segment_t1, ...
    cpN,P.t0,P.t1,8);
[eq,detail]=packet_equal_bitwise(expected,actual);
OUT.full_packet_bitwise_exact=eq;
OUT.full_packet_detail=detail;
fprintf(' prune boundary t1==commit exercised    = 1\n');
fprintf(' full prune+append packet bitwise exact = %d\n',eq);
assert(eq,'[R28 SELFTEST] full native prune+append packet mismatch.');

%% C. Structural theorem markers in the exact installed sources.
if isempty(classPath), classPath=which('REPLANNING_SYSTEM_R28_NATIVE_ONCE'); end
assert(~isempty(classPath)&&isfile(classPath),'[R28 SELFTEST] R28 class source missing.');
Sclass=fileread(classPath);
OUT.no_external_r27_compile=~contains(Sclass,'af7r78_incremental_compile_R27(');
OUT.r21_certificate_retained=contains(Sclass, ...
    'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED');
OUT.native_binding_present=contains(Sclass,'r28_validate_native_binding');
OUT.schedule_still_three_intervals=contains(Sclass,'pipeline_intervals=3;');
OUT.native_packet_bound=contains(Sclass,'R28_NATIVE_ONCE_EXACT_R23_SCHEMA');
fprintf(' external R27 runtime compile absent    = %d\n',OUT.no_external_r27_compile);
fprintf(' exact R21 certificate path retained    = %d\n',OUT.r21_certificate_retained);
fprintf(' native binding gate present            = %d\n',OUT.native_binding_present);
fprintf(' measurement schedule unchanged         = %d\n',OUT.schedule_still_three_intervals);
fprintf(' native packet authority split bound    = %d\n',OUT.native_packet_bound);

OUT.closed=OUT.derivative_bitwise_exact&&OUT.full_packet_bitwise_exact&& ...
    OUT.no_external_r27_compile&&OUT.r21_certificate_retained&& ...
    OUT.native_binding_present&&OUT.schedule_still_three_intervals&&OUT.native_packet_bound;
fprintf(' SELFTEST CLOSED                        = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R28 SELFTEST] contract did not close.');
end

function C=mkgroup(t0,t1,s0,s1,seed)
C=struct('t0',t0,'t1',t1,'segment_t0',s0,'segment_t1',s1,'control_points',[]);
C.control_points=cell(1,numel(s0));
for k=1:numel(s0), C.control_points{k}=mkcp(seed+3*k); end
end
function q=mkcp(seed)
v=reshape(1:48,3,16); q=sin((v+seed)*0.071)+cos((2*v-seed)*0.043)+0.01*seed;
end
function [tf,D]=packet_equal_bitwise(A,B)
fields={'schema','reference_version','group_count','segment_count','group_t0','group_t1', ...
    'group_first','group_segments','segment_t0','segment_t1','derivative_control_points', ...
    'max_derivative_order','degree','strict_support','group_addition_order','certified_numeric_representation'};
D=struct(); tf=true;
for k=1:numel(fields)
    f=fields{k}; ok=isfield(A,f)&&isfield(B,f);
    if ok
        a=A.(f); b=B.(f);
        if isnumeric(a)&&isnumeric(b)
            ok=isequal(size(a),size(b))&& ...
                isequal(typecast(double(a(:)),'uint64'),typecast(double(b(:)),'uint64'));
        elseif islogical(a)||islogical(b)
            ok=isequal(logical(a),logical(b));
        else
            ok=isequal(a,b);
        end
    end
    D.(f)=ok; tf=tf&&ok;
end
end
