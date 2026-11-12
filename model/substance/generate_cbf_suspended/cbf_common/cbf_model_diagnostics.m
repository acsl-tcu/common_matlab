function info=cbf_model_diagnostics(mode,x,settings)
%CBF_MODEL_DIAGNOSTICS モデル成立条件を数値チェック（保証ではなく診断）。
% real: ケーブル張力、姿勢正規化、実入力上下限を記録。
% virtual: 吊り荷加速度から復元するケーブル方向の分母を記録。
P=cbf_parameter_vector(settings);
if strcmp(mode,'real')
    if numel(x)~=29,error('CBF:StateLength','29状態が必要です。');end
    q=x(1:4);om=x(5:7);pt=x(14:16);omL=x(17:19);T=x(20);
    E=[-q(2),q(1),-q(4),q(3);-q(3),q(4),q(1),-q(2);-q(4),-q(3),q(2),q(1)];
    Lq=[-q(2),q(1),q(4),-q(3);-q(3),-q(4),q(1),q(2);-q(4),q(3),-q(2),q(1)];
    R=E*Lq.';dpt=cross(omL,pt);
    coefficient=(dot(pt,T*R*[0;0;1])-P(1)*P(3)*dot(dpt,dpt))/(P(1)+P(2));
    info.tension=-P(2)*coefficient;
    info.quaternionNormError=abs(norm(q)-1);
    info.cableDirectionNormError=abs(norm(pt)-1);
    info.cableAngularOrthogonality=abs(dot(pt,omL));
    info.thrustInBounds=(T>=settings.input.thrustMin && T<=settings.input.thrustMax);
    info.torqueInBounds=all(x(24:26)>=settings.input.torqueMin(:) & ...
                             x(24:26)<=settings.input.torqueMax(:));
    info.beta2Checked=false; % HLC Beta2 はコントローラ側の状態で要確認
elseif strcmp(mode,'virtual')
    if numel(x)~=20,error('CBF:StateLength','20状態が必要です。');end
    a=x(7:9)+P(4)*[0;0;1]-[P(14);P(15);0];
    info.flatMapDenominator=norm(a);
    info.tensionUnderTautAssumption=P(2)*norm(a);
    info.flatMapValid=norm(a)>1e-6;
    info.beta2Checked=false;
else
    error('CBF:UnknownMode','real/virtualを指定してください。');
end
end
