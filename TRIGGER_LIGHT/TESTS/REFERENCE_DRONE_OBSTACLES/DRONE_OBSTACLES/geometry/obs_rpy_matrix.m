function R=obs_rpy_matrix(rpyDeg)
% Extrinsic xyz / roll-pitch-yaw: R=Rz(yaw)*Ry(pitch)*Rx(roll).
a=double(rpyDeg(:))*pi/180; cr=cos(a(1)); sr=sin(a(1));
cp=cos(a(2)); sp=sin(a(2)); cy=cos(a(3)); sy=sin(a(3));
R=[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr; ...
   sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr; ...
   -sp, cp*sr, cp*cr];
end
