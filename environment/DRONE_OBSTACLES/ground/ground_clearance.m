function d=ground_clearance(g,pts)
% Positive for allowed side, negative for penetration. Not a CBF for bodies.
n=reshape(g.normal,1,3);n=n/norm(n);
d=(pts-reshape(g.point,1,3))*n'-g.safetyMargin;
end
