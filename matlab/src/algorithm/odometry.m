%% UNFINISHED 
classdef odometryCalculator < handle
  properties
    x, y, theta
    xT1m,yT1m, thetaT1m
  end

  methods
    function self = odometryCalculator(initialX, initialY, initialTheta)
      self.x = initialX;
      self.y = initialY;
      self.theta = initialTheta;
      self.xT1m = self.x;
      self.yT1m = self.y;
      self.thetaT1m = self.theta;
    end

    function [x,y,theta] = calculateNextPosition(vr, vl)
      % TODO
    end


  end
end
