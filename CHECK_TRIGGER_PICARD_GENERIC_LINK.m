function R=CHECK_TRIGGER_PICARD_GENERIC_LINK()
% Tests connecting the ORIGINAL 24-state WHOLE-interval tube directly to
% full-length cable clearance, independent of an obstacle-specific route.
R=struct('status','NOT_COMPLETED','reason','', ...
    'intervals',0,'far_clear',false,'unknown_on_near',false, ...
    'full_sphere_when_direction_unknown',false,'noncommit',false);
try
    lo=zeros(24,2);hi=zeros(24,2);
    lo(1:3,:)=-0.01;hi(1:3,:)=0.01;
    lo(19:21,:)=repmat([-0.05;-0.05;-1],1,2);
    hi(19:21,:)=repmat([0.05;0.05;-0.99],1,2);
    [T,S]=TRIGGER_PICARD_TO_GENERIC_TUBE.from_boxes( ...
        lo,hi,[0 1.4;1.4 4],[2 2],[2 2],0.35,0.02,0.25);
    assert(contains(S.status,'CONDITIONAL') && S.intervals==2 && ...
        isequal(size(T.axis),[3 2]) && ~S.safe_to_commit, ...
        'CHECK_TRIGGER_PICARD_GENERIC_LINK:Adapter');
    sh=struct('id',91,'center0',[12;0;0], ...
        'rotation',eye(3),'type','ellipsoid', ...
        'dimensions',[0.5 0.2 0.7], ...
        'is_stationary',true,'bounding_radius_m',0.7);
    O=struct('shapes',sh,'observed_ids',91,'verified_observation',true);
    A=TRIGGER_GENERIC_INTERVAL_SEPARATION.evaluate(T,O);
    assert(A.conditional_observed_geometry_clear && ~A.safe_to_commit, ...
        'CHECK_TRIGGER_PICARD_GENERIC_LINK:Far');
    R.far_clear=true;R.intervals=S.intervals;
    O.shapes.center0=[0;0;0];
    B=TRIGGER_GENERIC_INTERVAL_SEPARATION.evaluate(T,O);
    assert(~B.conditional_observed_geometry_clear && ~B.safe_to_commit, ...
        'CHECK_TRIGGER_PICARD_GENERIC_LINK:ContactUnknown');
    R.unknown_on_near=true;
    lo(19:21,:)=-1;hi(19:21,:)=1;
    [~,S2]=TRIGGER_PICARD_TO_GENERIC_TUBE.from_boxes( ...
        lo,hi,[0 1.4;1.4 4],[2 2],[2 2],0.35,0.02,0.25);
    assert(S2.full_sphere_intervals==2, ...
        'CHECK_TRIGGER_PICARD_GENERIC_LINK:UndefinedDirection');
    R.full_sphere_when_direction_unknown=true;
    R.noncommit=true;
    R.status='PASS_PICARD_24_TO_ALL_SHAPES_CONDITIONAL__NOT_FLIGHT_SAFETY';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    fid=fopen('TRIGGER_PICARD_GENERIC_LINK_RESULT.txt','w');
    if fid>0
        fprintf(fid,'status=%s\n',R.status);
        fprintf(fid,'intervals=%d\n',R.intervals);
        fprintf(fid,'far_clear=%d near_unknown=%d full_sphere=%d noncommit=%d\n', ...
            R.far_clear,R.unknown_on_near,R.full_sphere_when_direction_unknown,R.noncommit);
        fprintf(fid,'reason=%s\n',R.reason);fclose(fid);
    end
catch
end
end
