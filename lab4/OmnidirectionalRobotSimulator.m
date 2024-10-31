classdef OmnidirectionalRobotSimulator < handle
    % Represents an omnidirectional robot simulator.
    properties
        pose; % the robot's pose
        params; % the robot's physical parameters
        kinematics; % an auxiliary class to compute forward and inverse kinematics
    end
    
    methods
        function self = OmnidirectionalRobotSimulator(initialPose, params)
            % self = OmnidirectionalRobotSimulator(initialPose, params)
            % creates an omnidirectional robot simulator. The inputs
            % initialPose and params represent the robot's initial pose and
            % the robot's physical paramaters. The struct params may be
            % obtained through the function 
            % getOmnidirectionalRobotParams().
            self.pose = initialPose;
            self.params = params;
            self.kinematics = OmnidirectionalRobotKinematics(params);
        end
        
        function step(self, omegaw, dt)
            % step(self, omega, dt) executes a simulation step. 
            % The input omegaw represents a vector containing the wheel
            % angular velocities, respectively. The input dt is the 
            % simulation's time step.
            tolerance = 1e-3;
            omegaMax = self.params.omegaMax;
            omegaw = saturate(omegaw, -omegaMax, omegaMax);
            [v, vn, omega] = self.kinematics.forwardKinematics(omegaw);
            x = self.pose.x;
            y = self.pose.y;
            psi = self.pose.psi;
            if abs(omega) < tolerance
                self.pose.x = x + v * dt * cos(psi + omega * dt / 2) - vn * dt * sin(psi + omega * dt / 2);
                self.pose.y = y + v * dt * sin(psi + omega * dt / 2) + vn * dt * cos(psi + omega * dt / 2);
                self.pose.psi = psi + omega * dt;
            else
                self.pose.x = x + 2 * (v / omega) * sin(omega * dt / 2) * cos(psi + omega * dt / 2) - ...
                    2 * (vn / omega) * sin(omega * dt / 2) * sin(psi + omega * dt / 2);
                self.pose.y = y + 2 * (v / omega) * sin(omega * dt / 2) * sin(psi + omega * dt / 2) + ...
                    2 * (vn / omega) * sin(omega * dt / 2) * cos(psi + omega * dt / 2);
                self.pose.psi = psi + omega * dt;
            end
        end
    end
end