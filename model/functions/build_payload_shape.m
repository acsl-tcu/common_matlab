function shape = build_payload_shape(N, type, param)
arguments
    N
    type = "struct"
    param.rho = []
    param.rhoini = []
    param.rhoc = []
    param.pUp = []
    param.pDown = []
    param.G = []
end

shape = struct("rho", [], "rhoini", [], "rhoc", [], "pUp", [], "pDown", [], "G", []);
if ~isempty(param.rho)
    shape.rho = param.rho;
    if ~isempty(param.rhoini)
        shape.rhoini = param.rhoini;
    else
        shape.rhoini = param.rho;
    end
    shape.rhoc = param.rhoc;
    shape.pUp = param.pUp;
    shape.pDown = param.pDown;
    shape.G = param.G;
    return
end

isRegularHexagon = false;
if ~isRegularHexagon
    if isempty(param.pUp) || isempty(param.pDown)
        % Default payload shape (hexagonal prism)
        xUp = [-2 -1.5 0 1.5 1 0];
        yUp = [-1 0.5 1 0.5 -0.5 -1];
        zUp = 0.5*ones(1,6);
        pUp = [xUp;yUp;zUp]*0.4;

        xDown = [-2 -1.5 0 1.5 1 0];
        yDown = [-1 0.5 1 0.5 -0.5 -1];
        zDown = -0.5*ones(1,6);
        pDown = [xDown;yDown;zDown]*0.4;
    else
        pUp = param.pUp;
        pDown = param.pDown;
    end

    polyin = polyshape(pUp(1,:), pUp(2,:));
    [x, y] = centroid(polyin);
    G = [x; y; 0];
    rho = pUp - G;

    polyin = polyshape(pUp(1,1:N), pUp(2,1:N));
    [x, y] = centroid(polyin);
    Gc = [x; y; 0];
    rhoc = pUp - Gc;

    shape.rho = rho(:,1:N);
    shape.rhoini = shape.rho;
    shape.rhoc = rhoc(:,1:N);
    shape.pUp = pUp;
    shape.pDown = pDown;
    shape.G = G;
else
    if contains(type, "zup")
        rho0 = [0;0;1/4];
    else
        rho0 = [0;0;-1/4];
    end
    R = Rodrigues([0;0;1], 2*pi/N);
    shape.rho = rho0 + [[1;0;0], double(cellmatfun(@(A,~) A*[1;0;0], ...
        FoldList(@(A,B) A*B, cellrepmat(R,1,N-1), {eye(3)}, "mat"), "mat"))];
    shape.rhoini = shape.rho;
end
end
