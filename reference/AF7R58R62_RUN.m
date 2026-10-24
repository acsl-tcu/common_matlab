function R62=AF7R58R62_RUN()
% No input variables, no modal dialogs, no timing inside Flight.
% Reads R61 actual C7 witness and historical R55 trigger snapshots, if present.
% Real HLC+plant 4s model prediction needs callbacks to be supplied separately.
root=fileparts(mfilename('fullpath'));
f61=fullfile(root,'AF7R58R61_OUT.mat');
assert(isfile(f61),'R62:MissingR61','Put AF7R58R61_OUT.mat next to this script');
S=load(f61,'R61'); C=S.R61.real_correction;
assert(C.candidate_found,'R62:R61','R61 did not produce a candidate');
R62=struct('status','R61_C7_SOURCE_WITNESS_CONFIRMED__HLC_FUTURE_UNBOUND', ...
    'return_start_jet_error',C.candidate.start_jet_error, ...
    'return_end_jet_error',C.candidate.end_jet_error, ...
    'return_duration_s',C.candidate.T_s, ...
    'return_phase_authority','NOT_BOUND_TO_REAL_NOMINAL_OR_ACTIVE_PHASE', ...
    'hlc_unchanged',true,'trigger_modified',false,'commit_modified',false, ...
    'wholebody_execution_certified',false,'tube_installed',false, ...
    'historical_now_gate_status','NOT_RUN_MISSING_R55_SNAPSHOT', ...
    'now_gate',struct([]),'hlc_forward_status','REAL_CLONED_HLC_PLANT_CALLBACKS_REQUIRED');
f55=fullfile(root,'AF7R58R55_OUT.mat');
if isfile(f55)
    D=load(f55,'R55');
    if isfield(D,'R55') && isfield(D.R55,'trigger_packets')
        packets=D.R55.trigger_packets;
        if iscell(packets),packets=[packets{:}];end
        if isstruct(packets)
            nh=numel(packets);
            A=struct([]);
            for i=1:nh
                Z=packets(i);
                Q=struct('pQ_EKF',Z.pQ_EKF,'pL_EKF',Z.pL_EKF, ...
                   'q_EKF',Z.q_EKF,'cable_length_m',Z.L, ...
                   'centers',Z.centers,'rotations',Z.rotations, ...
                   'axes',Z.axes,'ids',Z.ids,'known_mask',Z.active, ...
                   'radii_m',Z.radii);
                Gi=AF7R58R62_EKF_NOW_GATE(Q);
                if isempty(A), A=Gi; else, A(end+1)=Gi; end
            end
            R62.now_gate=A;
            R62.historical_now_gate_status='HISTORICAL_EKF_INSTANTS_CHECKED__NOT_FUTURE';
        end
    end
end
R62.mat_file=fullfile(root,'AF7R58R62_OUT.mat');
R62.report_file=fullfile(root,'AF7R58R62_REPORT.txt');
save(R62.mat_file,'R62','-v7');
f=fopen(R62.report_file,'w');assert(f>0,'R62:Save','Unable to write report');
closeFile=onCleanup(@()fclose(f));
fprintf(f,'R62 | ACTUAL C7 SOURCE + EKF NOW GATE | NON-COMMITTABLE\n');
fprintf(f,'source_return_duration=%.12g start_jet_error=%.12g end_jet_error=%.12g\n', ...
   R62.return_duration_s,R62.return_start_jet_error,R62.return_end_jet_error);
fprintf(f,'phase=%s\n',R62.return_phase_authority);
fprintf(f,'now_gate=%s\n',R62.historical_now_gate_status);
if ~isempty(R62.now_gate)
    for i=1:numel(R62.now_gate)
        g=R62.now_gate(i);
        fprintf(f,'NOW event=%d known=%d status=%s unsafe=%d obstacle=%g component=%g ekf_residual=%.9g\n', ...
            i,g.known_obstacle_count,g.status,logical(g.unsafe_or_unproved), ...
            g.offending_obstacle_id,g.offending_component_code,g.estimated_cable_residual_m);
    end
end
fprintf(f,'physical_future=%s\n',R62.hlc_forward_status);
fprintf(f,'NO production trigger change. NO HLC modification. NO safety commit. NO robust tube.\n');
end
