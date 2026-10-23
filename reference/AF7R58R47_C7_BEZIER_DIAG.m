function A = AF7R58R47_C7_BEZIER_DIAG(segments, signed_axis)
% AF7R58R47_C7_BEZIER_DIAG  Piecewise BEZIER correction C7 diagnostics.
% Each element of segments MUST be a nonoverlapping, time-ordered piece of
% the *already combined* correction, with fields:
%   t0_s, t1_s, cp_m (16x3 for degree 15 Bernstein in physical time).
% The axis is an EXPLICIT measured signed direction (3x1); no side guess.
% This verifies double-precision junction jets and Bernstein coefficient
% hulls. It does not check UAV/cable/payload safety, yaw, or plant dynamics.
assert(isstruct(segments) && ~isempty(segments), ...
    '[R47/C7] Supply actual combined correction Bezier segments.');
axis=double(signed_axis(:));
assert(numel(axis)==3 && all(isfinite(axis)) && norm(axis)>0, ...
    '[R47/C7] Explicit measured signed axis must be a nonzero 3-vector.');
axis=axis/norm(axis);
n=numel(segments);
starts=zeros(n,8,3);
ends=zeros(n,8,3);
v_bounds=zeros(n,1);
signed_lows=zeros(n,1);
signed_highs=zeros(n,1);
t0=zeros(n,1);t1=zeros(n,1);
for j=1:n
    seg=segments(j);
    assert(all(isfield(seg,{'t0_s','t1_s','cp_m'})), ...
        '[R47/C7] Missing t0_s, t1_s or cp_m in segment %d.',j);
    t0(j)=double(seg.t0_s); t1(j)=double(seg.t1_s);
    assert(isscalar(t0(j)) && isscalar(t1(j)) && ...
        isfinite(t0(j)) && isfinite(t1(j)) && t1(j)>t0(j), ...
        '[R47/C7] Bad time interval at segment %d.',j);
    c=double(seg.cp_m);
    assert(isequal(size(c),[16,3]) && all(isfinite(c(:))), ...
        '[R47/C7] Segment %d needs degree-15 16x3 combined CP.',j);
    signed_lows(j)=min(c*axis);
    signed_highs(j)=max(c*axis);
    h=t1(j)-t0(j);
    d=c;
    for r=0:7
        starts(j,r+1,:)=d(1,:);
        ends(j,r+1,:)=d(end,:);
        if r==1
            v_bounds(j)=max(sqrt(sum(d.^2,2)));
        end
        d=(15-r)*(diff(d,1,1))/h;
    end
end
residual=zeros(max(0,n-1),8);
time_gap=zeros(max(0,n-1),1);
for j=1:n-1
    time_gap(j)=t0(j+1)-t1(j);
    for r=0:7
        l=reshape(ends(j,r+1,:),[1 3]);
        rr=reshape(starts(j+1,r+1,:),[1 3]);
        residual(j,r+1)=norm(l-rr);
    end
end
terminal_jet=zeros(8,1);
for r=0:7
    terminal_jet(r+1)=norm(reshape(ends(end,r+1,:),[3 1]));
end
A=struct();
A.schema='AF7R58R47_COMBINED_CORRECTION_C7_NUMERIC_DIAG_V1';
A.segment_count=n;
A.start_time_s=t0(1);A.end_time_s=t1(end);
A.time_gaps_s=time_gap;
A.junction_C7_residual_by_order_m_per_s_r=residual;
A.max_junction_C7_jet_residual=max([0;residual(:)]);
A.terminal_jet_norm_by_order=terminal_jet;
A.max_terminal_jet_norm=max(terminal_jet);
A.signed_axis=axis;
A.min_signed_CP_m=min(signed_lows);
A.max_signed_CP_m=max(signed_highs);
A.segment_velocity_CP_upper_mps=v_bounds;
A.max_velocity_CP_upper_mps=max(v_bounds);
A.exact_real_arithmetic_proof=false;
A.head_tangent_yaw_C6_proved=false;
A.whole_body_obstacle_certificate=false;
A.execution_certificate=false;
A.production_modified=false;
A.status='DOUBLE_PRECISION_SHAPE_DIAGNOSTIC_ONLY';
fprintf('\n[R47/C7] %d combined degree-15 Bezier intervals [%g, %g] s\n',n,t0(1),t1(end));
fprintf('[R47/C7] max time gap=%.12g s | C7 junction=%.12g | terminal jets=%.12g\n', ...
    max([0;abs(time_gap)]),A.max_junction_C7_jet_residual, A.max_terminal_jet_norm);
fprintf('[R47/C7] signed CP range=[%.12g, %.12g] m\n', ...
    A.min_signed_CP_m,A.max_signed_CP_m);
fprintf('[R47/C7] correction-speed CP bound max %.12g m/s\n', ...
    A.max_velocity_CP_upper_mps);
fprintf('[R47/C7] %s\n',A.status);
end
