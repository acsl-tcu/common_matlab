function ref = gen_ref_for_rigid_body(xdt, order, opts)
% Generate rigid-body reference with time derivatives up to order.
% xdt:
%   - function handle: returns [x;y;z;yaw] or [x;y;z]
%   - struct with fields:
%       .pYaw (function handle or symbolic), optional
%       .q    (function handle or symbolic), optional
% order: derivative order (default: 6)
% opts.useYawFromPYaw: use pYaw(4) as yaw when q is omitted (default: true)
arguments
    xdt
    order (1,1) double {mustBeNonnegative} = 6
    opts.useYawFromPYaw (1,1) logical = true
end

syms t real

pYaw_expr = resolve_expr(xdt, "pYaw", t);
if isempty(pYaw_expr)
    error("gen_ref_for_rigid_body:MissingPYaw", "pYaw reference is required.");
end
pYaw_expr = pYaw_expr(:);
if numel(pYaw_expr) == 3
    pYaw_expr = [pYaw_expr; 0];
end
if numel(pYaw_expr) ~= 4
    error("gen_ref_for_rigid_body:InvalidPYaw", "pYaw must be size 3 or 4.");
end

q_expr = resolve_expr(xdt, "q", t);
if isempty(q_expr)
    if opts.useYawFromPYaw
        q_expr = [0; 0; pYaw_expr(4)];
    else
        q_expr = [0; 0; 0];
    end
end
q_expr = q_expr(:);
if numel(q_expr) ~= 3
    error("gen_ref_for_rigid_body:InvalidQ", "q must be size 3 (Euler angles).");
end

pYaw_stack = stack_derivatives(pYaw_expr, t, order);
q_stack = stack_derivatives(q_expr, t, order);

R = eul_to_rotmat(q_expr);
R_stack = stack_rotmat_derivatives(R, t, order);

ref.pYaw = matlabFunction(pYaw_stack, "vars", t);
ref.q = matlabFunction(q_stack, "vars", t);
ref.rotms = matlabFunction(q_expr, R_stack, "vars", t);
ref.order = order;
end

function expr = resolve_expr(xdt, field, t)
expr = [];
if isstruct(xdt) && isfield(xdt, field)
    value = xdt.(field);
    if isa(value, "function_handle")
        expr = value(t);
    else
        expr = value;
    end
elseif isa(xdt, "function_handle") && field == "pYaw"
    expr = xdt(t);
end
end

function stack = stack_derivatives(expr, t, order)
stack = [];
for k = 0:order
    if k == 0
        dk = expr;
    else
        dk = diff(expr, t, k);
    end
    stack = [stack; dk(:)];
end
end

function R = eul_to_rotmat(q)
rx = q(1);
ry = q(2);
rz = q(3);
Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
R = Rx * Ry * Rz;
end

function R_stack = stack_rotmat_derivatives(R, t, order)
R_stack = [];
zero13 = zeros(1, 3);
for k = 0:order
    if k == 0
        dR = R;
    else
        dR = diff(R, t, k);
    end
    R_stack = [R_stack; dR; zero13];
end
end
