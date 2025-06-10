classdef RIGIDBODY_STATE_CLASS < handle
  % state : [p q v w]
  % p: position
  % q: attitude (unit quaternion)
  % v: translational velocity
  % w: rotational velocity
  %
  % to get different attitude type
  % state.getq(3) : Euler angle
  % state.getq(9) : rotation matrix
  properties
    list = ["p","q","v","w"];
    num_list = [3,4,3,3];
    name = "RIGIDBODY_STATE_CLASS";
    p
    q
    v
    w
  end
  methods
    function obj=RIGIDBODY_STATE_CLASS(varargin)
        if ~isempty(varargin)
            param = varargin{1};
            obj.set(param);
        end
    end
  end

  methods
      function state= get(obj)
          state= [obj.p;obj.q;obj.v;obj.w];
      end
      function set(obj,param)
            if isstruct(param) || contains(class(param),"STATE")
                obj.p = param.p;
                obj.q = param.q;
                obj.v = param.v;
                obj.w = param.w;
            else
                obj.p = param(1:3);
                obj.q = param(4:7);
                obj.v = param(8:10);
                obj.w = param(11:13);
            end
      end
    function q = getq(obj,type)
      arguments
        obj
        type = 4;
      end
      switch type
        case 4
            q = obj.q;
        case 3
            q=Quat2Eul(obj.q);             
        case 9
            q = RodriguesQuaternion(obj.q);
      end
    end
  end
end

