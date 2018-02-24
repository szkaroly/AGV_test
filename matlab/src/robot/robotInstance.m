%% UNFINISHED 
classdef robotInstance < handle
  properties
    hardwareAPI
  end

  methods
    %Constructor
    function self = robotInstance(hardwareAPI)
      self.hardwareAPI = hardwareAPI
    end

    function position = estimatePosition(self)
    end

    function [vr, vl] = calculateVelocityProfile(self)
    end




  end
end
