function objects=scene_gate_square()
% Gate aperture is along local z; roll=90deg rotates tunnel axis to horizontal.
% World gate center: [x y z] = [0 3 2.6].
% outerXY=[outer width outer height] in local gate plane; holeXY=[inner width inner height].
objects=obs_hollow_square_prism('square_gate',[0 3 2.6], ...
   [4.0 3.6],[2.2 2.0],0.40,[90 0 0]);
end
