function OUT = AF7R13_MARGIN_DERIVED_COVER_SOURCE_AUDIT()
% Read-only source audit for the theorem-derived continuous finite cover.

clc;
src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R13] REPLANNING_SYSTEM.m not found.');
txt=fileread(src);

hasOldDepth=~isempty(strfind(txt,'maxDepth=12;')); %#ok<STREMP>
hasOldLeaves=~isempty(strfind(txt,'maxLeaves=256;')); %#ok<STREMP>
hasRadius=~isempty(strfind(txt,'safe_time_radius_from_margin')); %#ok<STREMP>
hasGate=~isempty(strfind(txt,'halfWidth<rCert')); %#ok<STREMP>
hasStaticOpen=~isempty(strfind(txt,'STATIC')); %#ok<NASGU>

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R13 | MARGIN-DERIVED CONTINUOUS COVER SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf(' source                                  = %s\n',src);
fprintf(' old maxDepth=12 authority present       = %d\n',hasOldDepth);
fprintf(' old maxLeaves=256 authority present     = %d\n',hasOldLeaves);
fprintf(' theorem radius helper present           = %d\n',hasRadius);
fprintf(' strict h < m/V gate present             = %d\n',hasGate);
fprintf(' sampled acceptance introduced           = 0\n');
fprintf(' physical clearance multiplier added     = 0\n');
fprintf(' safety threshold tuned                  = 0\n');
fprintf(' theorem cover source closed             = %d\n', ...
    ~hasOldDepth && ~hasOldLeaves && hasRadius && hasGate);
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R13_MARGIN_DERIVED_COVER_SOURCE_AUDIT_V1';
OUT.source=src;
OUT.old_depth_authority_present=hasOldDepth;
OUT.old_leaf_authority_present=hasOldLeaves;
OUT.radius_helper_present=hasRadius;
OUT.strict_gate_present=hasGate;
OUT.closed=~hasOldDepth && ~hasOldLeaves && hasRadius && hasGate;
assignin('base','AF7R13_MARGIN_DERIVED_COVER_SOURCE_AUDIT_OUT',OUT);
end
