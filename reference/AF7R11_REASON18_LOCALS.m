function OUT = AF7R11_REASON18_LOCALS()
% AF7R11_REASON18_LOCALS
% Run ONLY while stopped inside builtin_bezier_d15_c7_proposal at the
% reason-18 breakpoint created by AF7R11_BACKEND1_REASON18_TRAP.
%
% Uses evalin('caller',...) so no production code modification is required.

names={ ...
    'kStart','kEnd','TdnSpeed','searchEnd','Tup','Tmin','TminFlat', ...
    'endpointSafeCount','tested','lastT1','bestFound','nw','w','dir', ...
    'lastGeom','lastSpeed','OUT'};

S=struct();
for k=1:numel(names)
    n=names{k};
    try
        S.(n)=evalin('caller',n);
    catch
        S.(n)=[];
    end
end

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R11 | REASON-18 LOCAL STATE\n');
fprintf('====================================================================================================\n');

print_scalar('kStart',S.kStart);
print_scalar('kEnd',S.kEnd);
print_scalar('TdnSpeed [s]',S.TdnSpeed);
print_scalar('searchEnd [s]',S.searchEnd);
print_scalar('Tup [s]',S.Tup);
print_scalar('Tmin [s]',S.Tmin);
print_scalar('TminFlat [s]',S.TminFlat);
print_scalar('endpointSafeCount',S.endpointSafeCount);
print_scalar('full certificate tests',S.tested);
print_scalar('last t1 [s]',S.lastT1);
print_scalar('bestFound',S.bestFound);
print_scalar('displacement norm [m]',S.nw);

if isnumeric(S.w)&&numel(S.w)==3
    q=double(S.w(:));
    fprintf(' displacement w [m]                    = [% .12g % .12g % .12g]\n',q);
end
if isnumeric(S.dir)&&numel(S.dir)==3
    q=double(S.dir(:));
    fprintf(' direction                              = [% .12g % .12g % .12g]\n',q);
end

fprintf('\n[A. LAST CONTINUOUS GEOMETRY CERTIFICATE]\n');
G=S.lastGeom;
if ~isstruct(G) || isempty(fieldnames(G))
    fprintf(' lastGeom                               = EMPTY\n');
    fprintf(' interpretation                         = no endpoint-safe candidate reached full certificate\n');
else
    print_field(G,'reason_code','reason code');
    print_field(G,'mathematical_pass','mathematical pass');
    print_field(G,'certified','certified');
    print_field(G,'min_margin_m','min total margin [m]');
    print_field(G,'min_payload_margin_m','min payload margin [m]');
    print_field(G,'min_uav_margin_m','min UAV margin [m]');
    print_field(G,'min_cable_margin_m','min cable margin [m]');
    print_field(G,'leaf_count','leaf count');
    print_field(G,'accepted_leaf_count','accepted leaf count');
    print_field(G,'split_count','split count');
    print_field(G,'max_depth_used','max depth used');
    print_field(G,'max_depth_cap','max depth cap');
    print_field(G,'resource_exhausted','resource exhausted');
    print_field(G,'tracking_inflation_m','tracking inflation [m]');
end

fprintf('\n[B. LAST REPAIRED-SPEED CERTIFICATE]\n');
SC=S.lastSpeed;
if ~isstruct(SC) || isempty(fieldnames(SC))
    fprintf(' lastSpeed                              = EMPTY\n');
else
    print_field(SC,'certified','certified');
    print_field(SC,'reason','reason');
    print_field(SC,'payload_speed_UB_m_s','payload speed UB [m/s]');
    print_field(SC,'uav_speed_UB_m_s','UAV speed UB [m/s]');
    print_field(SC,'specific_force_norm_LB_m_s2','specific force norm LB [m/s^2]');
    print_field(SC,'qdot_UB_rad_s','qdot UB [rad/s]');
end

fprintf('\n[C. PROPOSAL STATE BEFORE REASON 18]\n');
P=S.OUT;
if isstruct(P)
    print_field(P,'t0','t0 [s]');
    print_field(P,'tm','tm [s]');
    print_field(P,'t1','t1 [s]');
    print_field(P,'trigger_ttc','trigger TTC [s]');
    print_field(P,'trigger_collision_time','trigger collision time [s]');
    print_field(P,'trigger_metric_before','trigger metric before');
    print_field(P,'displacement_norm_m','displacement norm [m]');
    print_field(P,'c7_exact','C7 exact');
    if isfield(P,'correction_derivative_ub')
        q=double(P.correction_derivative_ub(:));
        fprintf(' correction derivative UB d1..d6       = [%s ]\n',sprintf(' %.6g',q));
    end
end

fprintf('\n[D. AUTOMATIC DIAGNOSIS]\n');
endpointCount=scalar_or_nan(S.endpointSafeCount);
tested=scalar_or_nan(S.tested);

if isfinite(endpointCount) && endpointCount==0
    classification='NO_REJOIN_ENDPOINT_PASSED_CONTINUOUS_TERMINAL_CELL';
elseif isfinite(tested) && tested>0 && isstruct(G) && ~isempty(fieldnames(G))
    if field_bool(G,'resource_exhausted')
        classification='ENDPOINT_SAFE_EXISTS__FULL_CERT_RESOURCE_EXHAUSTION';
    elseif ~field_bool(G,'mathematical_pass')
        classification='ENDPOINT_SAFE_EXISTS__FULL_INTERVAL_GEOMETRY_FAILS';
    else
        classification='UNEXPECTED__LAST_GEOMETRY_PASSES_BUT_NO_BEST_FOUND';
    end
else
    classification='REASON18_REQUIRES_MANUAL_SOURCE_STATE_REVIEW';
end

fprintf(' classification                         = %s\n',classification);

if strcmp(classification,'ENDPOINT_SAFE_EXISTS__FULL_INTERVAL_GEOMETRY_FAILS')
    fprintf([' likely interpretation                   = rejoin timing alone cannot repair the fixed rise/peak geometry;\n' ...
             '                                          inspect the peak displacement / strict support margin next.\n']);
end

fprintf('====================================================================================================\n');

OUT=S;
OUT.classification=classification;
assignin('base','AF7R11_REASON18_LOCALS_OUT',OUT);
end

function x=scalar_or_nan(v)
x=NaN;
try
    q=double(v);
    if isscalar(q)&&isfinite(q), x=q; end
catch
end
end

function print_scalar(name,v)
x=scalar_or_nan(v);
if isfinite(x)
    fprintf(' %-39s = %.12g\n',name,x);
else
    fprintf(' %-39s = NaN/UNAVAILABLE\n',name);
end
end

function print_field(S,name,label)
if ~isstruct(S)||~isfield(S,name)
    fprintf(' %-39s = MISSING\n',label);
    return;
end
v=S.(name);
if ischar(v) || (isstring(v)&&isscalar(v))
    fprintf(' %-39s = %s\n',label,char(string(v)));
elseif (islogical(v)||isnumeric(v))&&isscalar(v)
    fprintf(' %-39s = %.12g\n',label,double(v));
else
    fprintf(' %-39s = [%s]\n',label,class(v));
end
end

function tf=field_bool(S,name)
tf=false;
try
    tf=isstruct(S)&&isfield(S,name)&&logical(S.(name));
catch
end
end
