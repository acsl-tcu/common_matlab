function W=TRIGGER_SHAPE_NEUTRAL_INTERVAL_BOX_WITNESS(qlo,qhi,llo,lhi,c,R,r)
% SHAPE-NEUTRAL CONDITIONAL ALL-TIME INTERVAL GEOMETRY CERTIFICATE.
% Preconditions: q(t) in [qlo,qhi] and l(t) in [llo,lhi] for ALL times,
% cable is straight/taut, and O subset of closed B(c,R) is proved by an
% independent shape preparation certificate. Input doubles represent exact
% endpoints; arithmetic below is rounded outward one ULP per operation.
% For each material part, one coordinate with proven gap > (R+r_part)
% suffices for disjointness. A taut cable's every coordinate is in the
% hull of the two endpoint coordinate intervals. Nothing depends on shape
% family, orientation, convexity, or sampled trajectory points.
% A failure or absent certified source NEVER permits Flight commitment.
W=struct('status','UNKNOWN_INTERVAL_BOX_OUTER', ...
    'q_separated',false,'l_separated',false,'cable_separated',false, ...
    'all_parts_separated',false,'safe_to_commit',false, ...
    'source_outer_ball_verified_here',false,'reason','');
try
    assert(isnumeric(qlo)&&isnumeric(qhi)&&isnumeric(llo)&& ...
        isnumeric(lhi)&&isnumeric(c)&&isnumeric(R)&&isnumeric(r) && ...
        isreal(qlo)&&isreal(qhi)&&isreal(llo)&&isreal(lhi) && ...
        isreal(c)&&isreal(R)&&isreal(r) && ...
        isequal(size(qlo),[3 1])&&isequal(size(qhi),[3 1]) && ...
        isequal(size(llo),[3 1])&&isequal(size(lhi),[3 1]) && ...
        isequal(size(c),[3 1])&&isscalar(R)&&isequal(size(r),[1 3]) && ...
        all(isfinite([qlo;qhi;llo;lhi;c;R;r(:)])) && ...
        all(qlo<=qhi)&&all(llo<=lhi)&&R>=0&&all(r>=0), ...
        'TRIGGER_BOX_WITNESS:Input','Finite ordered 3D intervals and radii required.');
    qb=local_gap(qlo,qhi,c);
    lb=local_gap(llo,lhi,c);
    % Any convex combination of taut cable endpoints lies in this hull.
    cb=local_gap(min(qlo,llo),max(qhi,lhi),c);
    rq=TRIGGER_FLOAT_OUTWARD_ONE_ULP(double(R)+double(r(1)),1);
    rl=TRIGGER_FLOAT_OUTWARD_ONE_ULP(double(R)+double(r(2)),1);
    rc=TRIGGER_FLOAT_OUTWARD_ONE_ULP(double(R)+double(r(3)),1);
    assert(all(isfinite([rq rl rc])),'TRIGGER_BOX_WITNESS:Overflow', ...
        'Cannot establish finite outward upper bounds.');
    W.q_separated=any(qb>rq);
    W.l_separated=any(lb>rl);
    W.cable_separated=any(cb>rc);
    W.all_parts_separated= ...
        W.q_separated && W.l_separated && W.cable_separated;
    if W.all_parts_separated
        W.status='CONDITIONAL_SHAPE_NEUTRAL_ALL_TIME_SEPARATED';
        W.reason=['Outer-ball enclosure, true endpoint tubes, taut ' ...
            'mode and exact source arithmetic must be proved externally.'];
    else
        W.status='UNKNOWN_OUTER_BOX_OVERLAP';
        W.reason='One or more swept parts lack a strict axis gap.';
    end
catch ME
    W.reason=[ME.identifier ' | ' ME.message];
end
end
function gap=local_gap(lo,hi,c)
    % min|x_i-c_i| over each coordinate range: avoid sqrt/libm.
    % The two directed subtractions are exact-real lower enclosures.
    gap=max([zeros(1,3); ...
        TRIGGER_FLOAT_OUTWARD_ONE_ULP((lo-c).',-1); ...
        TRIGGER_FLOAT_OUTWARD_ONE_ULP((c-hi).',-1)],[],1);
end
