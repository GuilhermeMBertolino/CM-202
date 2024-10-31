classdef DifferentialRobotSimulator < handle
    % Represents a differential robot simulator.
    properties
        pose; % the robot's pose
        params; % the robot's physical parameters
        kinematics; % an auxiliary class to compute forward and inverse kinematics
    end
    
    methods
        function self = DifferentialRobotSimulator(initialPose, params)
            % self = DifferentialRobotSimulator(initialPose, params)
            % creates a differential robot simulator. The inputs
            % initialPose and params represent the robot's initial pose and
            % the robot's physical paramaters. The struct params may be
            % obtained through the function getDifferentialRobotParams().
            self.pose = initialPose;
            self.params = params;
            self.kinematics = DifferentialRobotKinematics(params);
        end
        
        function step(self, omegaR, omegaL, dt)
            % step(self, omegaR, omegaL, dt) executes a simulation step. 
            % The inputs omegaR and omegaL represent the right and left 
            % wheel's velocities, respectively. The input dt is the 
            % simulation's time step.
            
            % To change to the alternative equations if omega is close to
            % zero
            tolerance = 1e-3; 
            omegaMax = self.params.omegaMax;
            omegaR = saturate(omegaR, -omegaMax, omegaMax);
            omegaL = saturate(omegaL, -omegaMax, omegaMax);
            [v, omega] = self.kinematics.forwardKinematics(omegaR, omegaL);
            x = self.pose.x;
            y = self.pose.y;
            psi = self.pose.psi;
            if abs(omega) < tolerance
                self.pose.x = x + v * dt * cos(psi + omega * dt / 2);
                self.pose.y = y + v * dt * sin(psi + omega * dt / 2);
                self.pose.psi = psi + omega * dt;
            else
                self.pose.x = x + 2 * (v / omega) * sin(omega * dt / 2) * cos(psi + omega * dt / 2);
                self.pose.y = y + 2 * (v / omega) * sin(omega * dt / 2) * sin(psi + omega * dt / 2);
                self.pose.psi = psi + omega * dt;
            end
        end
    end
end