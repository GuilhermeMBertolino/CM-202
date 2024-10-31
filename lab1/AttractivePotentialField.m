classdef AttractivePotentialField < handle
    % Represents an attractive potential field.
    
    properties
        kAtt; % attractive constant
        d0; % distance threshold
    end
    
    methods
        function self = AttractivePotentialField(kAtt, d0)
            % self = AttractivePotentialField(kAtt, d0) constructs an
            % attractive field potential. The inputs are the attractive
            % constant kAtt and the distance threshold d0.
            self.kAtt = kAtt;
            self.d0 = d0;
        end
        
        function U = computePotential(self, position, goal)
            % U = computePotential(self, position, goal) computes the
            % potential given the robot's position and the goal. The output
            % is the potential at the given position.
            if norm(position - goal) <= self.d0
                U = (1/2) * self.kAtt * norm(position - goal)^2;
            else
                U = self.d0 * self.kAtt * norm(position - goal) - (1/2) * self.kAtt * self.d0^2;
            end
        end
        
        function dU = computeGradient(self, position, goal)
            % dU = computePotential(self, position, goal) computes the
            % potential gradient given the robot's position and the goal. 
            % The output is the potential gradient at the given position.
            if norm(position - goal) <= self.d0
                dU = self.kAtt * (position - goal);
            else
                dU = self.d0 * self.kAtt * (position - goal) / norm(position - goal);
            end
        end
    end
end