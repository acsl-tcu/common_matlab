function qp=cbf_attach_virtual_input_limits(qp,Thlc,tauOffset,tauMap,settings)
%CBF_ATTACH_VIRTUAL_INPUT_LIMITS  既存HLC逆変換のアフィン写像が得られた場合のみ使用。
% tau = tauOffset + tauMap * nuVirtual  （nuVirtualは4x1）
% 複数のHLCで係数が異なるため、tauMapは外部で正確に計算して渡す。
% TはnuVirtualで瞬間的に変更できないので、現在Tの範囲を検査する。
% 【注意】瞬時の推力検査では将来の推力制限は保証できない。
if ~strcmp(qp.mode,'virtual')
    error('CBF:Mode','仮想入力QPにのみ使用できます。');
end
if ~isequal(size(tauMap),[3,4]) || numel(tauOffset)~=3 || ...
   ~isscalar(Thlc) || any(~isfinite(tauMap(:))) || ...
   any(~isfinite(tauOffset(:))) || ~isfinite(Thlc)
    error('CBF:InvalidHLCMap','HLCの推力・トルク変換値が不正です。');
end
lo=settings.input.torqueMin(:);hi=settings.input.torqueMax(:);
qp.A=[qp.A;tauMap;-tauMap];
qp.b=[qp.b;lo-tauOffset(:);tauOffset(:)-hi];
qp.labels=[qp.labels;{'HLC_tau_x_min';'HLC_tau_y_min';'HLC_tau_z_min'; ...
                      'HLC_tau_x_max';'HLC_tau_y_max';'HLC_tau_z_max'}];
qp.order=[qp.order;zeros(6,1)];
qp.psi=[qp.psi;repmat({[]},6,1)];
qp.psiInitiallyNonnegative=[qp.psiInitiallyNonnegative;true(6,1)];
qp.effectNorm=sqrt(sum(qp.A.^2,2));
qp.degenerate=[qp.degenerate;all(tauMap==0,2);all(tauMap==0,2)];
qp.uncontrollableViolation=qp.degenerate & (qp.b>0);
qp.relativeDegreeAtState=[qp.relativeDegreeAtState;NaN(6,1)];
qp.inputLimitIncluded=false; % 将来の推力制限は含まないのでfalseのまま。
qp.torqueLimitsIncluded=true;
qp.currentThrustInBounds=(Thlc>=settings.input.thrustMin && ...
                          Thlc<=settings.input.thrustMax);
qp.note=['実HLCから渡されたtauMapによるトルク上下限制約を追加。' ...
         '瞬間推力範囲は診断のみ、将来の推力制約と逆変換の正則性は別途必要。'];
end
