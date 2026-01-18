function y = sl_sensor_func(self, dt, ~)
% Sensor adapter shared by Sim/Exp SuspendedLoad modes
p = self.sensor.result.state(1).get('p');
q = self.sensor.result.state(1).getq('3');

switch self.cha
    case 't'
        pL = p;
        pL(3) = pL(3) - self.parameter.get("cableL");
        pT = (pL - p);
        pT = pT / norm(pT);
        QmL = 1e-3;
        if self.estimator.ekf.Q(end, end) ~= QmL
            B = blkdiag([0.5 * dt ^ 2 * eye(6); dt * eye(6)], ...
                        [0.5 * dt ^ 2 * eye(3); dt * eye(3)], ...
                        [0.5 * dt ^ 2 * eye(3); dt * eye(3)], 1);
            Q = blkdiag(eye(3) * 1E1, eye(3) * 1E1, eye(3) * 1E1, eye(3) * 1E1, QmL);
            R = blkdiag(eye(3) * 1e-6, eye(3) * 1e-6, eye(3) * 1e-6, eye(3) * 1e-3);
            self.estimator.ekf.B = B;
            self.estimator.ekf.Q = Q;
            self.estimator.ekf.R = R;
            self.estimator.ekf.result.P = eye(25);
        end
    case {'a', 'l'}
        pL = p;
        pL(3) = pL(3) - self.parameter.get("cableL");
        pT = [0; 0; -1];
    otherwise
        pL = self.sensor.result.state(2).get('p');
        pT = (pL - p);
        pT = pT / norm(pT);
        QmL = 1e-3;
        if self.estimator.ekf.Q(end, end) ~= QmL
            B = blkdiag([0.5 * dt ^ 2 * eye(6); dt * eye(6)], ...
                        [0.5 * dt ^ 2 * eye(3); dt * eye(3)], ...
                        [0.5 * dt ^ 2 * eye(3); dt * eye(3)], 1);
            Q = blkdiag(eye(3) * 1E1, eye(3) * 1E1, eye(3) * 1E1, eye(3) * 1E1, QmL);
            R = blkdiag(eye(3) * 1e-6, eye(3) * 1e-6, eye(3) * 1e-6, eye(3) * 1e-6);
            self.estimator.ekf.B = B;
            self.estimator.ekf.Q = Q;
            self.estimator.ekf.R = R;
            self.estimator.ekf.result.P = eye(25);
        end
end

y = [p; q; pL; pT];
end
