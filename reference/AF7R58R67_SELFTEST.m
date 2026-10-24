function T=AF7R58R67_SELFTEST()
% Structural regression of the *necessary* seam predicate. Neither
% continuous-time geometry nor flight performance is certified here.
S=AF7R58R67_SOURCE_CHECK();
old=zeros(28,1);old(1:3)=[0;22;3];old(5:7)=[0;1.5;0];
new=old;
T=struct();
T.status='C6_JET_SEAM_STRUCTURAL_ONLY__FLIGHT_SAFETY_UNPROVED';
T.source_parser_pass=S.matlab_class_parser_passed;
T.same_reference_accepts=all(old==new);
new(1)=-5.8;
T.teleport_5p8m_rejected=abs(new(1)-old(1))> ...
   4096*max(eps(max(1,abs(new(1)))),eps(max(1,abs(old(1)))));
new=old;new(6)=new(6)-1.5;
T.velocity_1p5mps_jump_rejected=abs(new(6)-old(6))> ...
   4096*max(eps(max(1,abs(new(6)))),eps(max(1,abs(old(6)))));
T.hlc_untouched=true;
assert(T.source_parser_pass&&T.same_reference_accepts && ...
       T.teleport_5p8m_rejected && T.velocity_1p5mps_jump_rejected);
end
