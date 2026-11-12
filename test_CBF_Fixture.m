function s = test_CBF_Fixture()
%TEST_CBF_FIXTURE テスト専用の仮の数値。生成コードには一切埋め込まれません。
% このファイルだけに試験数値を置き、一般の生成器からは呼びません。
s = CBF_RuntimeSettings_Template();
s.physical.massDrone = 1.0;
s.physical.massLoad = 0.15;
s.physical.cableLength = 0.46;
s.physical.gravity = 9.81;
s.physical.inertia = [0.012;0.012;0.020];
s.physical.disturbanceXY = [0;0];
s.shape.droneRadius = 0.20;
s.shape.loadRadius = 0.10;
s.shape.cableRadius = 0.005;
s.cable.maxExtraRadius = 0.010;
s.cable.numberOfSpheres = [];
s.cable.maxNumberOfSpheres = 200;
s.cbf.safetyMargin = 0.04;
s.cbf.smoothEpsilon = 1e-5;
s.cbf.hocbfGain = 1.2;
s.cbf.inputGain = 1.2;
s.cbf.degeneracyTolerance = 1e-10;
s.ground.height = 0;
s.input.thrustMin = 0;
s.input.thrustMax = 20;
s.input.torqueMin = [-1;-1;-1];
s.input.torqueMax = [1;1;1];
end
