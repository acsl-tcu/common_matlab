function [eigs_a, eigs_b, eigs_c, eigs_d, info] = kalman_mode_eigs(A, B, C, doPlot, tol)


    narginchk(3, 5);
    if nargin < 4 || isempty(doPlot), doPlot = true; end

    [n, n2] = size(A);
    if n ~= n2
        error('A must be square.');
    end
    if size(B,1) ~= n || size(C,2) ~= n
        error('A, B, and C have incompatible dimensions.');
    end
    if nargin < 5 || isempty(tol)
        tol = 100 * max(size(A)) * eps(max(1, norm(A,2)));
    end

    % Controllable subspace R and unobservable subspace N.
    Co = local_ctrb(A, B);
    Ob = local_obsv(A, C);
    QR = local_range(Co, tol);
    QN = null(Ob, tol);
    rR = size(QR,2);
    rN = size(QN,2);

    % R intersection N: singular values equal to one give common vectors.
    if rR == 0 || rN == 0
        Qa = zeros(n,0);
    else
        [U,S,~] = svd(QR' * QN, 'econ');
        s = diag(S);
        common = abs(1-s) <= max(10*tol, sqrt(eps));
        Qa = local_range(QR * U(:,common), tol);
    end

    na = size(Qa,2);
    nb = rR - na;
    nc = rN - na;
    nd = n - na - nb - nc;
    if nd < 0
        error('Numerical rank inconsistency. Try specifying a larger tolerance.');
    end

    % Build an adapted (generally non-orthogonal) Kalman basis:
    % span(Ta,Tb)=R and span(Ta,Tc)=N.
    Ta = Qa;
    Tb = local_extend(Ta, QR, nb, tol);
    Tc = local_extend([Ta Tb], QN, nc, tol);
    Td = local_extend([Ta Tb Tc], eye(n), nd, tol);
    T = [Ta Tb Tc Td];
    if size(T,2) ~= n || rcond(T) < eps
        error('Could not construct a reliable Kalman basis. Adjust tol or scale the model.');
    end

    Abar = T \ (A*T);
    Bbar = T \ B;
    Cbar = C*T;

    ia = 1:na;
    ib = na + (1:nb);
    ic = na + nb + (1:nc);
    id = na + nb + nc + (1:nd);
    eigs_a = eig(Abar(ia,ia));
    eigs_b = eig(Abar(ib,ib));
    eigs_c = eig(Abar(ic,ic));
    eigs_d = eig(Abar(id,id));

    % Whole-system properties. For a discrete-time system, stabilizability
    % means that every uncontrollable mode is strictly inside the unit
    % circle. Detectability is its dual for unobservable modes.
    uncontrollableEigs = [eigs_c; eigs_d];
    unobservableEigs = [eigs_a; eigs_c];
    stabilityTol = max(1e-9, tol);
    badStabilizabilityEigs = ...
        uncontrollableEigs(abs(uncontrollableEigs) >= 1-stabilityTol);
    badDetectabilityEigs = ...
        unobservableEigs(abs(unobservableEigs) >= 1-stabilityTol);

    % Extract the controllable subsystem and design its discrete-time LQR.
    % Qc = I means that every controllable state has weight 1.
    if rR == 0
        error('The system has no controllable state, so dlqr cannot be used.');
    end
    if size(B,2) ~= 4
        error(['Rc = diag([1 0.1 0.1 1]) requires B to have exactly ' ...
               'four input columns.']);
    end

    controllableIdx = 1:rR;
    Ac = Abar(controllableIdx,controllableIdx);
    Bc = Bbar(controllableIdx,:);
    Qc = eye(rR);
    Rc = diag([1; 0.1; 0.1; 1]);

    % dlqr uses u(k) = -Kc*xc(k) in the controllable coordinates.
    if exist('dlqr','file') == 0
        error(['The dlqr function was not found. Install or enable ' ...
               'Control System Toolbox to perform the LQR design.']);
    end
    [Kc, Sc, closedLoopEigsC] = dlqr(Ac, Bc, Qc, Rc);

    % Return the gain to the original x coordinates:
    % x = T*xbar, u = -[Kc 0]*xbar = -K*x.
    Kbar = [Kc zeros(size(B,2),n-rR)];
    K = Kbar / T;
    Aclosed = A - B*K;
    closedLoopEigs = eig(Aclosed);

    if ~isempty(badStabilizabilityEigs)
        warning('kalman_mode_eigs:NotStabilizable', ...
            ['The controllable part was designed by dlqr, but the full ' ...
             'system cannot be stabilized because it has an unstable ' ...
             'uncontrollable mode.']);
    end

    info.T = T;
    info.Abar = Abar;
    info.Bbar = Bbar;
    info.Cbar = Cbar;
    info.counts = struct('a',na, 'b',nb, 'c',nc, 'd',nd);
    info.tol = tol;
    info.stabilityTol = stabilityTol;
    info.isControllable = (rR == n);
    info.isObservable = (rN == 0);
    info.isStabilizable = isempty(badStabilizabilityEigs);
    info.isDetectable = isempty(badDetectabilityEigs);
    info.badStabilizabilityEigs = badStabilizabilityEigs;
    info.badDetectabilityEigs = badDetectabilityEigs;
    info.Ac = Ac;
    info.Bc = Bc;
    info.Qc = Qc;
    info.Rc = Rc;
    info.Kc = Kc;
    info.Sc = Sc;
    info.closedLoopEigsC = closedLoopEigsC;
    info.Kbar = Kbar;
    info.K = K;
    info.Aclosed = Aclosed;
    info.closedLoopEigs = closedLoopEigs;
    info.isClosedLoopStable = all(abs(closedLoopEigs) < 1-stabilityTol);

    fprintf('a: controllable / unobservable   = %d mode(s)\n', na);
    fprintf('b: controllable / observable     = %d mode(s)\n', nb);
    fprintf('c: uncontrollable / unobservable = %d mode(s)\n', nc);
    fprintf('d: uncontrollable / observable   = %d mode(s)\n', nd);

    if info.isControllable
        fprintf('System test: controllable\n');
    else
        fprintf('System test: NOT controllable\n');
    end
    if info.isObservable
        fprintf('System test: observable\n');
    else
        fprintf('System test: NOT observable\n');
    end
    if info.isStabilizable
        fprintf('System test: stabilizable (discrete time)\n');
    else
        fprintf('System test: NOT stabilizable (discrete time)\n');
        fprintf('Uncontrollable unstable/boundary eigenvalue(s):\n');
        disp(info.badStabilizabilityEigs);
    end
    if info.isDetectable
        fprintf('System test: detectable (discrete time)\n');
    else
        fprintf('System test: NOT detectable (discrete time)\n');
        fprintf('Unobservable unstable/boundary eigenvalue(s):\n');
        disp(info.badDetectabilityEigs);
    end
    fprintf('LQR gain Kc for the controllable subsystem:\n');
    disp(info.Kc);
    fprintf('LQR gain K in the original coordinates:\n');
    disp(info.K);
    if info.isClosedLoopStable
        fprintf('Closed-loop test: stable (all poles are inside the unit circle)\n');
    else
        fprintf('Closed-loop test: NOT stable\n');
    end

    if doPlot
        local_plot(eigs_a, eigs_b, eigs_c, eigs_d);
    end
end

function Co = local_ctrb(A,B)
    n = size(A,1);
    Co = zeros(n, n*size(B,2), 'like', A+B*B');
    X = B;
    for k = 1:n
        cols = (k-1)*size(B,2) + (1:size(B,2));
        Co(:,cols) = X;
        X = A*X;
    end
end

function Ob = local_obsv(A,C)
    n = size(A,1);
    Ob = zeros(n*size(C,1), n, 'like', A+C'*C);
    X = C;
    for k = 1:n
        rows = (k-1)*size(C,1) + (1:size(C,1));
        Ob(rows,:) = X;
        X = X*A;
    end
end

function Q = local_range(X,tol)
    if isempty(X), Q = zeros(size(X,1),0); return; end
    [U,S,~] = svd(X,'econ');
    s = diag(S);
    if isempty(s), Q = zeros(size(X,1),0); return; end
    Q = U(:,s > tol*max(1,s(1)));
end

function Xadd = local_extend(X, candidates, wanted, tol)
    Xadd = zeros(size(candidates,1),0, 'like', candidates);
    baseRank = rank(X, tol);
    for k = 1:size(candidates,2)
        if size(Xadd,2) == wanted, break; end
        trial = [X Xadd candidates(:,k)];
        if rank(trial, tol) > baseRank + size(Xadd,2)
            Xadd(:,end+1) = candidates(:,k); %#ok<AGROW>
        end
    end
    if size(Xadd,2) ~= wanted
        error('Basis construction failed. Try a different numerical tolerance.');
    end
end

function local_plot(ea,eb,ec,ed)
    figure('Color','w'); hold on; grid on; box on; axis equal;
    th = linspace(0,2*pi,600);
    plot(cos(th),sin(th),'k--','LineWidth',1.2,'DisplayName','Unit circle');
    plot(real(ea),imag(ea),'ro','MarkerSize',8,'LineWidth',1.6, ...
        'DisplayName','controllable / unobservable');
    plot(real(eb),imag(eb),'b+','MarkerSize',9,'LineWidth',1.8, ...
        'DisplayName','controllable / observable');
    plot(real(ec),imag(ec),'ks','MarkerSize',8,'LineWidth',1.6, ...
        'DisplayName','uncontrollable / unobservable');
    plot(real(ed),imag(ed),'md','MarkerSize',8,'LineWidth',1.6, ...
        'DisplayName','uncontrollable / observable');
    xline(0,'Color',[.7 .7 .7],'HandleVisibility','off');
    yline(0,'Color',[.7 .7 .7],'HandleVisibility','off');
    xlabel('Real part'); ylabel('Imaginary part');
    title('Kalman mode classification');
    legend('Location','best');
end
