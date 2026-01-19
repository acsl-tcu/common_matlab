function AL = set_cha_allocation(app,agent)
AL = struct();
for cha = ['a','t','f','l']
  AL.(cha) = struct();
  for i = ["sensor","estimator","controller","reference","input_transform","plant"]
    AL.(cha).(i) = [];
  end
end
if isprop(agent,"cha_allocation")
  al = agent.cha_allocation;
  for i = ["sensor","estimator","controller","reference","input_transform","plant"]
    % set prop for all cha
    % example: cha_allocation = struct('sensor', ['s1','s2','s3'])
    if isfield(al,i)
      for cha = ['a','t','f','l']
        AL.(cha).(i) = al.(i);
      end
    end
  end
  for cha = ['a','t','f','l']
    if isfield(al,cha)
      for i = ["sensor","estimator","controller","reference","input_transform","plant"]
          if isfield(al.(cha),i)
              AL.(cha).(i) = al.(cha).(i);
          end
      end
    end
  end
end
end
