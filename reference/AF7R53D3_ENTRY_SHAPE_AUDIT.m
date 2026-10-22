function OUT = AF7R53D3_ENTRY_SHAPE_AUDIT
K=af7r36_single_core_mex(22);
S=af7r36_single_core_mex(23);

knames={'build_id','side','rejoin_s','corr_vel_ub','corr_acc_ub','corr_jerk_ub', ...
        'node','knot_s','Px','Py','Pz','Pnorm','Vx','Vy','Vz','Vnorm','prev_T','next_T'};
snames={'build_id','side','rejoin_s','corr_vel_ub','corr_acc_ub','corr_jerk_ub', ...
        'seg','ta','tb','T','cp_norm','seg_vel_ub','seg_acc_ub','seg_jerk_ub','reserved'};

assert(size(K,2)==numel(knames),'[AF7R53D3] unexpected knot matrix width.');
assert(size(S,2)==numel(snames),'[AF7R53D3] unexpected segment matrix width.');

KT=array2table(K,'VariableNames',knames);
ST=array2table(S,'VariableNames',snames);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R53D3 | ENTRY / KNOT / SEGMENT SHAPE AUDIT (DIAGNOSTIC ONLY)\n');
fprintf('====================================================================================================\n');

fprintf('\n[KNOT TABLE]\n');
disp(KT);

fprintf('\n[SEGMENT TABLE]\n');
disp(ST);

fprintf('\n[FIRST INTERIOR KNOT]\n');
if isempty(KT)
    fprintf('EMPTY\n');
else
    U=KT(KT.node==1,:);
    disp(U(:,{'build_id','side','rejoin_s','knot_s','Pnorm','Vnorm', ...
             'prev_T','next_T','Px','Py','Pz','Vx','Vy','Vz'}));
end

fprintf('\n[ENTRY SEGMENT]\n');
if isempty(ST)
    fprintf('EMPTY\n');
else
    E=ST(ST.seg==0,:);
    disp(E(:,{'build_id','side','rejoin_s','T','cp_norm', ...
             'seg_vel_ub','seg_acc_ub','seg_jerk_ub'}));
end

fprintf('\nInterpretation:\n');
fprintf(' node 1 is the first interior C7 knot after zero correction at trigger/support start.\n');
fprintf(' seg 0 is the exact-C7 entry segment.\n');
fprintf(' large Pnorm with short prev_T => displacement pressure at entry.\n');
fprintf(' large Vnorm => corridor requires substantial through-speed at first unsafe boundary.\n');
fprintf(' seg_vel_ub matching corr_vel_ub => global speed problem is localized to entry.\n');
fprintf(' This is diagnostic only; no hard gate, family, parameter, or fallback is modified.\n');
fprintf('====================================================================================================\n');

OUT=struct('knots',KT,'segments',ST);
assignin('base','R53D3',OUT);
end
