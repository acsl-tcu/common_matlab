function z = observables_candidate_trig(x)
% A compact candidate lifting used for controlled comparisons.
% The first 12 entries always preserve [p q v w] in the project order.
state = x(1:12,1);
roll = state(4); pitch = state(5); yaw = state(6);
z = [state;
     sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
     -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
     cos(roll)*cos(pitch);
     1];
end
