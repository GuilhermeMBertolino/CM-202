classdef DifferentialRobotKinematics
    % Represents an auxiliary class to compute forward and inverse 
    % kinematics for a differential robot.
    properties
        params; % the robot's physical parameters
    end
    
    methods
        function self = DifferentialRobotKinematics(params)
            % self = DifferentialRobotKinematics(params) creates an 
            % auxiliary class to compute forward and inverse kinematics 
            % for a differential robot. The input params contains the 
            % robot's physical parameters, as returned by the function 
            % getDifferentialRobotParams().
            self.params = params;
        end
        
        function [v, omega] = forwardKinematics(self, omegaR, omegaL)
            % [v, omega] = forwardKinematics(self, omegaR, omegaL)
            % computes the forward kinematics given the left (omegaR) and 
            % right (omegaL) wheel's velocities. The outputs v and omega
            % are the robot's linear and angular velocities, respectively.
            r = self.params.r;
            l = self.params.l;
            v = (omegaR + omegaL) * r / 2;
            omega = (omegaR - omegaL) * r / l;
        end
        
        function [omegaR, omegaL] = inverseKinematics(self, v, omega)
            % [omegaR, omegaL] = inverseKinematics(self, v, omega)
            % computes the inverse kinematics given the robot's linear (v) 
            % and angular (omega) velocities. The outputs omegaR and omegaL
            % are the left and right wheel's velocities, respectively.
            r = self.params.r;
            l = self.params.l;
            omegaR = (v + omega * l / 2) / r;
            omegaL = (v - omega * l / 2) / r;
        end
    end
end