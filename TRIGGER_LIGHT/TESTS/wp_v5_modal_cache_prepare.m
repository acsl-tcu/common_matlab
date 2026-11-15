function C=wp_v5_modal_cache_prepare(gains,pastRelativeTimes,filterTime)
%WP_V5_MODAL_CACHE_PREPARE Source-controller modal matrices, once per gain
% revision AND history-grid change. Invalidate if HLC F1/F2/F3 or sampling
% period changes. Per-tick use must pass the same exact relative time grid.
t=reshape(pastRelativeTimes,1,[]);
assert(numel(t)>6 && abs(t(end))<1e-10 && ...
    all(diff(t)>0) && isfinite(filterTime) && filterTime>0);
A6=diag(ones(5,1),1);
Ax=A6;Ax(6,:)=-reshape(gains.F2,1,[]);
Ay=A6;Ay(6,:)=-reshape(gains.F3,1,[]);
Az=[0,1;-reshape(gains.F1,1,[])];
A={Ax,Ay,Az};
HP=cell(1,3);Phi=cell(1,3);A2=cell(1,3);
for j=1:3
    HP{j}=zeros(numel(t),size(A{j},1));
    for k=1:numel(t)
        B=expm(A{j}*t(k));
        HP{j}(k,:)=B(1,:);
    end
    Phi{j}=expm(A{j}*.025);
    A2{j}=A{j}*A{j};
end
C=struct('pastRelativeTimes',t,'HP',{HP},'Phi',{Phi},'A2',{A2}, ...
    'governor',wp_six_trigger_governor_fast_coeff(.025,filterTime), ...
    'gainsF1',gains.F1,'gainsF2',gains.F2,'gainsF3',gains.F3, ...
    'computedOutsideTick',true, ...
    'physicalPredictionErrorCertified',false);
end
