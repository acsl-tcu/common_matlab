classdef FUNCTIONAL_MEC < handle
%ノミナルとMECをまとめるクラス
properties
    self
    result
    param
    parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    nominal%ノミナル用のコントローラー
    mec%MEC用のコントローラー
end
    

methods

    function obj = FUNCTIONAL_MEC(nominal_controller, mec_controller)
        % obj.nominal=class(nominal_controller);
        obj.nominal=nominal_controller;
        obj.mec=mec_controller;
        % obj.result.input = zeros(self.estimator.model.dim(2),1);
        % obj.result.input = [0;0;0;0];
    end

    function result = do(obj,varargin)
        obj.nominal.do(varargin{1},varargin{2});
        % obj.mec.do(obj.nominal.result.input);
        
        % obj.mec.do(varargin{1},varargin{2});
        obj.mec.do(varargin{1},varargin{2});%Δu計算にプラントとノミナルの状態必要
        % obj.result = obj.mec.result.input;
        obj.result = obj.nominal.result.input + obj.mec.result.input;%nominalコントローラーの入力とmecコントローラーの入力の和
        result = obj.result;
    end

    function show(obj)
        obj.result
    end

end

end