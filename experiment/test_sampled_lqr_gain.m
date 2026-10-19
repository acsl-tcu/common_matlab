% スカラー系の解析解と、独立した数値積分・Lyapunov方程式でゲインを検証。
repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repo_root,'assets'),fullfile(repo_root,'mode','HL'));

dt = 0.025;
q = 3;
r = 0.2;
Qd = q*dt;
Nd = q*dt^2/2;
Rd = r*dt+q*dt^3/3;
P = sqrt(Qd*Rd-Nd^2)/dt;
expected_gain = (dt*P+Nd)/(Rd+dt^2*P);
assert(abs(sampled_lqr_gain(0,1,q,r,dt)-expected_gain)<1e-9);

for dt = [0.025,0.1]
    setting = Controller_HL(dt);
    assert(isequal(setting.F2,setting.F3));
    for axis_index = 1:3
        if axis_index == 2
            A = diag(ones(1,3),1);
            B = [0;0;0;1];
            Q = diag([1500,1000,200,1]);
            gain = setting.F2;
        else
            A = [0,1;0,0];
            B = [0;1];
            if axis_index == 1
                Q = diag([100,1]);
                gain = setting.F1;
            else
                Q = diag([300,10]);
                gain = setting.F4;
            end
        end
        n = size(A,1);
        dynamics = [A,B;zeros(1,n+1)];
        flow = expm(dynamics*dt);
        Ad = flow(1:n,1:n);
        Bd = flow(1:n,end);
        % 代替計算のVan Loan公式とは独立に、数値積分で評価関数を求める。
        cost = integral(@(t) expm(dynamics'*t)*blkdiag(Q,0.05)* ...
            expm(dynamics*t),0,dt,'ArrayValued',true);
        closed = Ad-Bd*gain;
        assert(all(abs(eig(closed))<1));
        stage_cost = [eye(n);-gain]'*cost*[eye(n);-gain];
        P = reshape((eye(n*n)-kron(closed',closed'))\stage_cost(:),n,n);
        stationarity = (cost(end,end)+Bd'*P*Bd)*gain ...
            -(Bd'*P*Ad+cost(end,1:n));
        assert(norm(stationarity,'fro')<1e-7*max(1,norm(gain,'fro')));
    end
end
fprintf('sampled_lqr_gain: analytic and HLC optimality checks passed.\n');
