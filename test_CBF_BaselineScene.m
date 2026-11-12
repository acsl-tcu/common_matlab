function qp=test_CBF_BaselineScene(mode,shape,x,s,obstacles)
%TEST_CBF_BASELINESCENE 最適化前の処理を再現する検証・速度比較専用。
% 各障害物について地面/入力を含む全HOCBFを計算し、後から重複行を捨てる。
% 実際のコントローラからは呼び出さない。
assert(iscell(obstacles)&&~isempty(obstacles));
qp=[];
for j=1:numel(obstacles)
    obs=obstacles{j};
    [h,info]=cbf_evaluate_case(mode,shape,obs.type,x,s,obs);
    all=cbf_hocbf_evaluate(mode,shape,obs.type,x,s,obs,h,info);
    if j==1
        qp=all;
        qp.labels=cellfun(@(v)sprintf('obs%d_%s',j,v),all.labels, ...
            'UniformOutput',false);
    else
        keep=~strncmp(all.labels,'ground_',7) & ...
             ~strncmp(all.labels,'thrust_',7) & ...
             ~strncmp(all.labels,'tau_',4);
        qp.A=[qp.A;all.A(keep,:)];qp.b=[qp.b;all.b(keep)];
        qp.labels=[qp.labels;cellfun(@(v)sprintf('obs%d_%s',j,v), ...
            all.labels(keep),'UniformOutput',false)];
        qp.order=[qp.order;all.order(keep)];
        qp.psi=[qp.psi;all.psi(keep)];
    end
end
end
