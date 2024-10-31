classdef OmnidirectionalRobotKinematics < handle
    % Represents an auxiliary class to compute forward and inverse 
    % kinematics for an omnidirectional robot.
    properties
        M; % the matrix that transforms robot's velocities to wheels' velocities
        Mpinv; % the inverse of M
        params; % the robot's physical parameters
    end
    
    methods
        function self = OmnidirectionalRobotKinematics(params)
            % self = OmnidirectionalRobotKinematics(params) creates an 
            % auxiliary class to compute forward and inverse kinematics 
            % for an omnidirectional robot. The input params contains the 
            % robot's physical parameters, as returned by the function 
            % getOmnidirectionalParams().
            self.params = params;
            a = params.alpha;
            b = params.beta;
            l = params.l;
            self.M = zeros(4, 3);
            for i=1:size(self.M, 1)
                self.M(i, :) = [-sin(a(i) + b(i)), cos(a(i) + b(i)), l(i) * cos(b(i))];
            end
            self.Mpinv = inv((self.M') * self.M) * self.M';
        end
        
        function [v, vn, omega] = forwardKinematics(self, omegaw)
            % [v, vn, omega] = forwardKinematics(self, omegaw) computes 
            % the forward kinematics given the wheel angular velocities 
            % contained in omegaw (4x1 vector). The outputs v, vn and 
            % omega are the robot's forward, sideways, and angular 
            % velocities, respectively.
            r = self.params.r;
            vr = r * self.Mpinv * omegaw;
            v = vr(1);
            vn = vr(2);
            omega = vr(3);
        end
        
        function omegaw = inverseKinematics(self, v, vn, omega)
            % omegaw = inverseKinematics(self, v, omega) computes the 
            % inverse kinematics given the robot's linear (v), 
            % sideways (vn), and angular (omega) velocities. The output
            % omegaw is a 4x1 vector containing the wheel angular 
            % velocities, respectively.
            r = self.params.r;
            vr = [v; vn; omega];
            self.Mpinv;
            omegaw = (1 / r) * self.M * vr;
        end
    end
end