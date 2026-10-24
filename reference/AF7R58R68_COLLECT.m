function R68=AF7R58R68_COLLECT()
% Source-bound diagnostic ONLY. Run after SimHL, not in Flight.
root=fileparts(mfilename('fullpath'));
R68=struct();
R68.capture_available=false;
R68.hlc_unchanged=true;
R68.commit_logic_unchanged=true;
R68.closed_loop_4s_EKF_future_proven=false;
R68.natural_C7_return_committed=false;
R68.columns={'activation_t','prior_version','candidate_t0','seam_pass', ...
    'old_yaw0','old_yaw1','old_yaw2','old_yaw3','old_yaw4','old_yaw5','old_yaw6', ...
    'new_yaw0','new_yaw1','new_yaw2','new_yaw3','new_yaw4','new_yaw5','new_yaw6', ...
    'old_vx','old_vy','old_vz','new_vx','new_vy','new_vz', ...
    'old_true_tangent','new_true_tangent', ...
    'old_yaw_minus_tangent_wrapped','new_yaw_minus_tangent_wrapped', ...
    'old_horizontal_speed','new_horizontal_speed', ...
    'max_position_orders0to6','max_yaw_orders0to5','yaw_order6_abs', ...
    'first_mismatched_yaw_order','yaw0_difference_wrapped'};
R68.events=nan(0,35);
R68.total_attempts=0;
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
 D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
 if isstruct(D)&&isfield(D,'r68_seam_detail_ring')&&isfield(D,'r68_seam_detail_count')
  R68.capture_available=true;
  ring=double(D.r68_seam_detail_ring);
  R68.total_attempts=double(D.r68_seam_detail_count);
  n=min(size(ring,1),R68.total_attempts);
  if n>0
   ix=mod((R68.total_attempts-n):(R68.total_attempts-1),size(ring,1))+1;
   R68.events=ring(ix,:);
  end
 end
end
assert(size(R68.events,2)==35,'[R68] seam record width invalid.');
assert(numel(R68.columns)==35,'[R68] column labels invalid.');
R68.rejected_count=sum(R68.events(:,4)==0);
R68.yaw_low_order_mismatch_count=sum(R68.events(:,32)>1e-10);
R68.yaw6_only_count=sum(R68.events(:,32)<=1e-10 & ...
    R68.events(:,33)>1e-10);
R68.old_yaw_not_tangent_count=sum(abs(R68.events(:,27))>1e-6);
R68.new_yaw_not_tangent_count=sum(abs(R68.events(:,28))>1e-6);
R68.first_mismatch_orders=unique(R68.events(:,34));
R68.mat_file=fullfile(root,'AF7R58R68_OUT.mat');
R68.report_file=fullfile(root,'AF7R58R68_REPORT.txt');
save(R68.mat_file,'R68','-v7');
fid=fopen(R68.report_file,'wt');
assert(fid>0,'[R68] report creation failed');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R68 SOURCE-BOUND YAW SEAM ROOT CAUSE | DIAGNOSTIC ONLY\n');
fprintf(fid,'capture=%d total=%d retained=%d rejected=%d\n', ...
    R68.capture_available,R68.total_attempts,size(R68.events,1),R68.rejected_count);
fprintf(fid,'low_order_yaw_mismatch=%d yaw6_only=%d old_not_tangent=%d new_not_tangent=%d\n', ...
    R68.yaw_low_order_mismatch_count,R68.yaw6_only_count, ...
    R68.old_yaw_not_tangent_count,R68.new_yaw_not_tangent_count);
fprintf(fid,'COLS oldYaw0..6 newYaw0..6 in MAT, plus velocity and tangent angles\n');
for j=1:size(R68.events,1)
 z=R68.events(j,:);
 fprintf(fid,['SEAM t=%.9f version=%g t0=%.9f pass=%d firstYawOrder=%g ' ...
    'pos0to6=%.6g yaw0to5=%.6g yaw6=%.6g ' ...
    'old_tangent_err=%.6g new_tangent_err=%.6g ' ...
    'oldSpeed=%.6g newSpeed=%.6g oldYaw0=%.9g newYaw0=%.9g ' ...
    'oldYaw6=%.9g newYaw6=%.9g\n'], ...
    z(1),z(2),z(3),z(4),z(34),z(31),z(32),z(33), ...
    z(27),z(28),z(29),z(30),z(5),z(12),z(11),z(18));
end
fprintf(fid,'IMPORTANT: No proof of 4s HLC/plant future or natural C7 return.\n');
fprintf(fid,'HLC/EKF/plant/MEX/trigger/commit actions unchanged from R67.\n');
fprintf(fid,'MAT=%s\nTXT=%s\n',R68.mat_file,R68.report_file);
end
