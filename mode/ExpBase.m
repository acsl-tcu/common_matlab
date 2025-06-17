for i = 1:length(agent)
  agent(i).reference.takeoff = TAKEOFF_REFERENCE(agent(i),[]);
  agent(i).reference.landing = LANDING_REFERENCE(agent(i),dt,0.1);
  if isfield(agent,"cha_allocation")
     if  isfield(agent(i).cha_allocation,"a")
        agent(i).cha_allocation.t.reference = "takeoff";
     end   
     if  isfield(agent(i).cha_allocation,"t")
        agent(i).cha_allocation.t.reference = "takeoff";
     end   
     if  isfield(agent(i).cha_allocation,"l")
      agent(i).cha_allocation.l.reference = "landing";
     end
  else
      agent(i).cha_allocation = struct("a",struct("reference","takeoff"),"t",struct("reference","takeoff"),"l",struct("reference","landing"));
  end
end