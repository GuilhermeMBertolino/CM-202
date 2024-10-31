classdef FollowlineController < handle
    % Represents a controller for a followline robot.
    properties
        Kx; % gain for the x controller
        KyPrime; % gain for the y controller
        Kpsi; % gain for the psi controller
        vMin; % minimum speed used to compute Ky
        vMax; % maximum speed used to compute the forward velocity
        psiMax; % maximum reference psi
    end
    
    methods
        function self = FollowlineController(controllerParams)
            % self = FollowlineController(controllerParams) creates a 
            % followline controller. The input controllerParams contains 
            % parameters for the followline controller.
            self.Kx = controllerParams.Kx;
            self.KyPrime = controllerParams.KyPrime;
            self.Kpsi = controllerParams.Kpsi;
            self.vMin = controllerParams.vMin;
            self.vMax = controllerParams.vMax;
            self.psiMax = controllerParams.psiMax;
        end
        
        function [v, omega] = control(self, xr, yr, x, y, psi)
            % [v, omega] = control(self, xr, yr, x, y, psi) updates the
            % followline controller and computes the linear and angular
            % velocities to control the robot. The inputs are:
            % xr: reference x coordinate.
            % yr: reference y coordinate.
            % x: robot's x coordinate.
            % y: robot's y coordinate.
            % psi: robot's yaw angle.
            v = self.Kx * (xr - x);
            v = saturate(v, -self.vMax, self.vMax);
            if abs(v) < self.vMin
                Ky = self.KyPrime / self.vMin;
            else
                Ky = self.KyPrime / v;
            end
            psir = Ky * (yr - y);
            if psir > self.psiMax
                psir = self.psiMax;
            elseif psir < -self.psiMax
                psir = -self.psiMax;
            end
            omega = self.Kpsi * (psir - psi);
        end
    end
end