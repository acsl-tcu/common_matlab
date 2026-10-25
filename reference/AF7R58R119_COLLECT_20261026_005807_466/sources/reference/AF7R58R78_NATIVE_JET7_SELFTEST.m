function T = AF7R58R78_NATIVE_JET7_SELFTEST()
% MATLAB R2025a self-test for NEW native op25; CALL BEFORE SimHL only.
% Does not use/rewrite the live HLC, EKF, plant or any Flight authority.
% The test briefly compiles a zero-nominal AST in the native MEX and clears
% it on exit. Run SimHL afterwards so the correct Flight nominal is compiled.
assert(exist('af7r36_single_core_mex','file')==3, ...
    '[R78] rebuild af7r36_single_core_mex.cpp before this test.');
cleanup = onCleanup(@()clear_native_mex_r78());
E=repmat({'0'},21,1);
ok=af7r36_single_core_mex(0,E,'R78_SELFTEST_ZERO_NOMINAL');
assert(isscalar(ok) && logical(ok),'[R78] nominal prime failed.');
C=zeros(3,16,2);
C(:,9:16,1)=repmat([0.1;0.7;-0.2],1,8);
C(:,9:16,2)=repmat([-0.4;0.2;0.9],1,8);
Q=struct('flight_t0',0, ...
    'repair_t0',[1,4],'repair_t1',[3,6], ...
    'repair_offsets',[0,1,2], ...
    'repair_seg0',[1,4],'repair_seg1',[3,6], ...
    'repair_cp',reshape(C,48,2));
t=[0.8,1,1.25,2.1,3,3.5,4,4.7,5.1,6,6.5];
D=double(af7r36_single_core_mex(25,Q,t));
P=double(af7r36_single_core_mex(2,Q,t));
assert(isequal(size(D),[3,numel(t)]) && ...
       isequal(size(P),[21,numel(t)]),'[R78] native schemas incorrect.');
W7=zeros(3,numel(t));W06=zeros(21,numel(t));
for k=1:numel(t)
    for g=1:2
        if t(k)<=Q.repair_t0(g) || t(k)>=Q.repair_t1(g)
            if t(k)>=Q.repair_t1(g)
                W06(1:3,k)=W06(1:3,k)+C(:,end,g);
            end
            continue;
        end
        v=(t(k)-Q.repair_t0(g))/(Q.repair_t1(g)-Q.repair_t0(g));
        B=double(C(:,:,g));degree=15;
        for r=0:7
            z=B;
            while size(z,2)>1
                z=(1-v)*z(:,1:end-1)+v*z(:,2:end);
            end
            if r==7
                W7(:,k)=W7(:,k)+z(:,1);
            else
                W06(3*r+(1:3),k)=W06(3*r+(1:3),k)+z(:,1);
            end
            if r<7
                B=degree/(Q.repair_t1(g)-Q.repair_t0(g))*diff(B,1,2);
                degree=degree-1;
            end
        end
    end
end
err7=max(abs(D(:)-W7(:)));
err06=max(abs(P(:)-W06(:)));
assert(err7<=1e-9*max(1,max(abs(W7(:)))) && ...
       err06<=1e-9*max(1,max(abs(W06(:)))), ...
       '[R78] native and independently evaluated Bezier jets disagree.');
assert(all(D(:,t<=1)==0,'all') && all(D(:,t>=6)==0,'all'), ...
       '[R78] correction seven-jet escaped support.');
T=struct('status','PASS_R78_NATIVE_SAME_CP_JETS_0_TO_7', ...
    'max_error_order7',err7,'max_error_orders0to6',err06, ...
    'number_of_groups',2,'sample_count',numel(t), ...
    'Flight_trigger_replaced',false, ...
    'C7_yaw6_seam_proved_in_Flight',false);
end

function clear_native_mex_r78()
clear('af7r36_single_core_mex');
end
