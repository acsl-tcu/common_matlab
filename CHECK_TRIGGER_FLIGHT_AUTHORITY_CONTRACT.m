function T=CHECK_TRIGGER_FLIGHT_AUTHORITY_CONTRACT()
% Synthetic contract test; never called by Flight or any actual controller.
% Uses the real HLC class solely to verify its constructor/binding layout.
% No prediction or proof of safety is claimed.
root=fileparts(mfilename('fullpath'));
module=fullfile(root,'reference','REPLANNING_SYSTEM','FUTURE_COLLISION_TRIGGER');
addpath(module,'-begin');
controller_dir=fullfile(root,'controller');
if isfolder(controller_dir),addpath(controller_dir,'-end');end
assert(exist('HLC_SUSPENDED_LOAD','class')==8, ...
    'The existing HLC_SUSPENDED_LOAD folder must be on the MATLAB path.');
order=["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity", ...
    "km1","km2","km3","km4","k1","k2","k3","k4", ...
    "rotor_r","Length","loadmass","cableL"];
vals=[0.75,0.2,0.2,0.1,0.1,0.02,0.025,0.04,9.8, ...
    0.03,0.03,0.03,0.03,8e-6,8e-6,8e-6,8e-6,0.04,0.08,0.14,2];
geo=struct('r_uav_m',0.13,'r_payload_m',0.08,'r_cable_m',0.005,'horizon_s',4);
e=struct('p',[0;0;2],'q',zeros(3,1),'v',zeros(3,1), ...
    'w',zeros(3,1),'pL',[0;0;0],'vL',zeros(3,1), ...
    'pT',[0;0;-1],'wL',zeros(3,1),'mL',0.2);
param=struct('parameter_order',order,'get',@read_parameter);
model_state=struct('list',["p","q","v","w","pL","vL","pT","wL"], ...
    'num_list',[3 3 3 3 3 3 3 3]);
model=struct('state',model_state,'method',@dummy_rhs,'solver',@dummy_solver, ...
    'dt',0.025,'param',vals);
gains=struct('F1',[1 2],'F2',[1 2 3 4 5 6], ...
    'F3',[1 2 3 4 5 6],'F4',[1 2],'dt',0.025);
shadow=HLC_SUSPENDED_LOAD(struct(),gains);
agent=struct('parameter',param,'plant',model, ...
    'controller',struct('hlc_suspended',shadow), ...
    'estimator',struct('result',struct('state',e)));
time=struct('t',5,'dt',0.025);
b=TRIGGER_FLIGHT_AUTHORITY(time,agent,@()geo,@reference_from_existing);
[ekf,a1]=b.current(time,agent);
assert(a1.payload_mass_kg==0.2 && ...
    a1.runtime.cable_length_m==2 && ...
    a1.runtime.r_uav_m==0.13 && ...
    a1.actual_hlc_mass_source_kg==0.14 && ...
    b.snapshot_count==1 && ...
    isequal(ekf.p,e.p));
% Between Flight ticks, ONLY estimated mass and current EKF state refresh.
vals(1)=0.9;vals(21)=2.8;
geo.r_uav_m=0.22;
agent.estimator.result.state.mL=0.33;
time.t=5.025;
[~,a2]=b.current(time,agent);
assert(a2.payload_mass_kg==0.33 && ...
    a2.runtime.cable_length_m==2 && ...
    a2.runtime.r_uav_m==0.13 && ...
    a2.actual_hlc_mass_source_kg==0.14 && ...
    b.snapshot_count==2);
% New Flight binding must read the changed external parameters anew.
agent.plant.param=vals;
b2=TRIGGER_FLIGHT_AUTHORITY(time,agent,@()geo,@reference_from_existing);
[~,a3]=b2.current(time,agent);
assert(a3.runtime.cable_length_m==2.8 && ...
    a3.runtime.r_uav_m==0.22 && ...
    a3.actual_hlc_mass_source_kg==0.14);
time.dt=0.01;
dt_rejected=false;
try
    b2.current(time,agent);
catch ME
    dt_rejected=strcmp(ME.identifier,'TRIGGER_FLIGHT_AUTHORITY:DTChanged');
end
assert(dt_rejected);
T=struct('status','PASS_FLIGHT_BINDING_AND_LATEST_EKF_MASS__NO_LIVE_PREDICTION', ...
    'checks',9,'nominal_predictor_executed',false, ...
    'existing_hlc_untouched',true,'no_future_sensing',true, ...
    'no_arbitrary_parameter_fallback',true);
disp(T.status);
    function v=read_parameter(keys)
        keys=string(keys);v=zeros(1,numel(keys));
        for i=1:numel(keys)
            j=find(order==keys(i),1);
            assert(~isempty(j));v(i)=vals(j);
        end
    end
    function dx=dummy_rhs(~,~,~)
        error('SYNTHETIC:NotAnActualModel','This model is never to be run.');
        dx=[];
    end
    function [t,x]=dummy_solver(~,~,~)
        error('SYNTHETIC:NotAnActualSolver','This solver is never to be run.');
        t=[];x=[];
    end
    function ref=reference_from_existing(~)
        error('SYNTHETIC:NoRealReference','This test does not execute a predictor.');
        ref=[];
    end
end
