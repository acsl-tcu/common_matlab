function OUT = AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM(replanner,B49)
% AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM
%
% Read-only theorem-oriented audit of the current true-trigger rejoin domain.
%
% Inputs:
%   replanner : current REPLANNING_SYSTEM instance from the same run
%   B49       : output of AF7R49B_V2_REJOIN_DOMAIN_FORENSIC
%
% This script does NOT modify production source, runtime authority, or safety
% acceptance.  It uses B49 only to identify the topology of the diagnostic
% dt-lattice success set, then replays a small number of representative t1
% values with the unchanged AF7R36 continuous-time core.
%
% The dt-lattice is NOT declared production authority.  It is used only to
% expose the connected-component structure that a later validated continuous
% t1 solver must preserve.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R50A | TERMINAL FEASIBLE COMPONENT + FINITE-SENSING SPEED THEOREM AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | theoremAudit=1 | sampledSafetyAcceptance=0 | productionModified=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'), ...
    '[AF7R50A] replanner type mismatch.');
assert(isstruct(B49) && isfield(B49,'schema') && ...
    contains(char(B49.schema),'AF7R49B_REJOIN_DOMAIN_FORENSIC'), ...
    '[AF7R50A] pass B49 from AF7R49B_V2_REJOIN_DOMAIN_FORENSIC.');
assert(exist('af7r36_single_core_mex','file')==3, ...
    '[AF7R50A] AF7R36 native core missing.');
assert(exist('af7r49_replay_context','file')==2, ...
    '[AF7R50A] AF7R49 helper package must remain on path.');
assert(exist('af7r34_make_core_packet','file')==2, ...
    '[AF7R50A] af7r34_make_core_packet.m missing.');

t1 = double(B49.t1(:).');
pass = logical(B49.success(:).');
reason = double(B49.reason(:).');
assert(numel(t1)==numel(pass) && numel(pass)==numel(reason) && ~isempty(t1));
assert(all(diff(t1)>0),'[AF7R50A] B49 t1 lattice is not strictly increasing.');
assert(pass(end),'[AF7R50A] current source-domain upper bound is not certified.');

% -------------------------------------------------------------------------
% 1) Exact connected components of the diagnostic dt-lattice success set.
% -------------------------------------------------------------------------
d = diff([false pass false]);
beg = find(d==1);
fin = find(d==-1)-1;
assert(numel(beg)==numel(fin));

terminalIdx = find(fin==numel(pass),1,'last');
assert(~isempty(terminalIdx), ...
    '[AF7R50A] no certified component contains the current upper bound.');

kL = beg(terminalIdx);
kU = fin(terminalIdx);
terminalLeft = t1(kL);
terminalRight = t1(kU);

earliestIdx = find(pass,1,'first');
earliestGrid = t1(earliestIdx);

fprintf('\n[A. DIAGNOSTIC SUCCESS-SET TOPOLOGY]\n');
fprintf(' certified dt-lattice components          = %d\n',numel(beg));
for c=1:numel(beg)
    fprintf('  component %d                            = [%.15g, %.15g] s | points=%d%s\n', ...
        c,t1(beg(c)),t1(fin(c)),fin(c)-beg(c)+1, ...
        ternary(c==terminalIdx,' | CONTAINS CURRENT t_U',''));
end
fprintf(' earliest isolated/grid-certified t1 [s] = %.15g\n',earliestGrid);
fprintf(' terminal-component left lattice t1 [s]  = %.15g\n',terminalLeft);
fprintf(' current source-validity upper t_U [s]   = %.15g\n',terminalRight);

if kL>1
    fprintf(' point immediately below terminal comp   = %.15g s | pass=%d | reason=%g\n', ...
        t1(kL-1),pass(kL-1),reason(kL-1));
end

% -------------------------------------------------------------------------
% 2) Reconstruct exact true-trigger packet.
% -------------------------------------------------------------------------
E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid), ...
    '[AF7R50A] first true trigger event unavailable.');
assert(strcmpi(char(E.reference.mode),'NOMINAL'), ...
    '[AF7R50A] first trigger does not carry NOMINAL authority.');

CTX=af7r49_replay_context(replanner,E);
assert(replanner.nominal_batch_model_cache_valid && ...
    isstruct(replanner.nominal_batch_model_cache) && ...
    isfield(replanner.nominal_batch_model_cache,'certified') && ...
    logical(replanner.nominal_batch_model_cache.certified), ...
    '[AF7R50A] nominal batch model cache unavailable.');

pkt=af7r34_make_core_packet( ...
    replanner,CTX,[],replanner.nominal_batch_model_cache);

SC0=E.speed_certificate;
assert(isstruct(SC0)&&isfield(SC0,'certified')&&logical(SC0.certified));
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);

rhoBody=max([double(pkt.r_drone), ...
             double(pkt.cable_length)+double(pkt.r_load), ...
             double(pkt.cable_length)+double(pkt.r_cable)]);
VQlim=(double(pkt.sensor_range)-rhoBody)/(double(pkt.horizon)+double(pkt.dt));

fprintf('\n[B. FINITE-SENSING SPEED THEOREM]\n');
fprintf(' sensor range R_s [m]                    = %.15g\n',double(pkt.sensor_range));
fprintf(' whole-body radius rho [m]               = %.15g\n',rhoBody);
fprintf(' prediction horizon H [s]                = %.15g\n',double(pkt.horizon));
fprintf(' control period dt [s]                   = %.15g\n',double(pkt.dt));
fprintf(' theorem speed limit V_Q^lim [m/s]       = %.15g\n',VQlim);
fprintf(' formula                                  = (R_s-rho)/(H+dt)\n');
fprintf(' strict authority                         = V_Q^UB < V_Q^lim\n');

% Representative points: earliest grid pass, left terminal pass,
% predecessor of terminal component, and current t_U.
ids=unique([earliestIdx, max(1,kL-1), kL, kU],'stable');

fprintf('\n[C. REPRESENTATIVE TRUE-CERTIFICATE REPLAYS]\n');
fprintf(' #   t1[s] pass rsn speedPass stageA  VQ_UB      VQlim-VQ     sigmaLB     qdotUB   geomMargin\n');

rows=struct([]);
for q=1:numel(ids)
    k=ids(q);
    P=pkt; P.t1=t1(k);
    R=af7r36_single_core_mex(1,P);

    sp=double(R.speed_pass_count);
    if logical(R.success)
        stageA=double(logical(R.speed_stageA));
        vq=double(R.speed_uav);
        mar=VQlim-vq;
        sig=double(R.speed_sigma);
        qd=double(R.speed_qdot);
        gm=double(R.geometry_min_margin);
    else
        stageA=NaN; vq=NaN; mar=NaN; sig=NaN; qd=NaN; gm=NaN;
    end

    fprintf('%2d %8.3f %4d %3g %9.0f %6.0f %10.7f %12.7g %11.7g %10.7g %11.7g\n', ...
        q,t1(k),logical(R.success),double(R.reason_code),sp,stageA,vq,mar,sig,qd,gm);

    rows(q).t1=t1(k); %#ok<AGROW>
    rows(q).success=logical(R.success);
    rows(q).reason=double(R.reason_code);
    rows(q).speed_pass=sp;
    rows(q).stageA=stageA;
    rows(q).VQ=vq;
    rows(q).speed_margin=mar;
    rows(q).sigma=sig;
    rows(q).qdot=qd;
    rows(q).geom_margin=gm;
end

% -------------------------------------------------------------------------
% 3) Explain the failure code from the production source contract.
% -------------------------------------------------------------------------
n23=nnz(~pass & reason==23);
n24=nnz(~pass & reason==24);
nOther=nnz(~pass & reason~=23 & reason~=24);

fprintf('\n[D. FAILURE CLASSIFICATION]\n');
fprintf(' reason 23 count                          = %d | PIECEWISE_C7_TOWING_CORRIDOR_QP_FAILED\n',n23);
fprintf(' reason 24 count                          = %d | PIECEWISE_C7_FLATNESS_OR_FINITE_SENSING_FAILED\n',n24);
fprintf(' other failure count                      = %d\n',nOther);

preTerminal=1:max(1,kL-1);
n24pre=nnz(~pass(preTerminal) & reason(preTerminal)==24);
nFailPre=nnz(~pass(preTerminal));
fprintf(' pre-terminal failures due to reason 24  = %d / %d\n',n24pre,nFailPre);

% -------------------------------------------------------------------------
% 4) Formal production target.  Do NOT claim grid optimum as continuous
%    optimum.  The correct target is the left boundary of the connected
%    continuous feasible component containing the known-safe upper bound.
% -------------------------------------------------------------------------
fprintf('\n[E. FORMAL CONTINUOUS PRODUCTION TARGET]\n');
fprintf([' Define F = {t1 in (holdEnd,t_U] : the SAME C7 synthesis and all existing\n' ...
         ' continuous speed/rejoin/whole-body certificates close}.\\n']);
fprintf([' Let C_U be the connected component of F that contains the known-safe\n' ...
         ' source-domain upper bound t_U.\\n']);
fprintf(' The canonical target is                 t_* = inf C_U\n');
fprintf(' The dt-lattice value %.15g s is ONLY a diagnostic enclosure clue.\n',terminalLeft);
fprintf(' It is NOT yet a continuous-time production authority.\n');
fprintf(' Boolean feasibility is non-monotone globally = %d\n',numel(beg)>1);

% The existing grid itself has already demonstrated global non-monotonicity.
% Therefore ordinary binary search over pass/fail is mathematically invalid.
fprintf(' ordinary global bisection allowed        = 0\n');
fprintf(' required next solver                     = validated continuation / interval isolation of C_U boundary\n');
fprintf(' final candidate acceptance               = unchanged continuous AF7R36 certificates\n');

% -------------------------------------------------------------------------
% 5) Safety obligations that must remain invariant in the next implementation.
% -------------------------------------------------------------------------
fprintf('\n[F. SAFETY INVARIANTS FOR NEXT IMPLEMENTATION]\n');
fprintf(' C7 position/reference construction        = UNCHANGED\n');
fprintf(' R9/Stage-A/B speed certificate            = UNCHANGED\n');
fprintf(' strict finite-sensing inequality          = UNCHANGED\n');
fprintf(' continuous rejoin certificate             = UNCHANGED\n');
fprintf(' continuous whole-body certificate         = UNCHANGED\n');
fprintf(' sampled acceptance                        = 0\n');
fprintf(' unresolved numerical interval policy      = FAIL CLOSED\n');
fprintf(' IEEE754 outward boundary isolation needed = 1\n');
fprintf(' production source modified                = 0\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM_V1';
OUT.component_begin_index=beg;
OUT.component_end_index=fin;
OUT.component_begin_t1=t1(beg);
OUT.component_end_t1=t1(fin);
OUT.terminal_component_index=terminalIdx;
OUT.terminal_component_left_lattice_t1=terminalLeft;
OUT.current_upper_t1=terminalRight;
OUT.earliest_grid_certified_t1=earliestGrid;
OUT.VQ_limit=VQlim;
OUT.representative=rows;
OUT.global_boolean_feasibility_nonmonotone=(numel(beg)>1);
OUT.production_target='infimum of continuous feasible component containing current t_U';
OUT.production_modified=false;

assignin('base','AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM_OUT',OUT);
end


function s=ternary(tf,a,b)
if tf, s=a; else, s=b; end
end
