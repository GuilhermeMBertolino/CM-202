classdef RepulsivePotentialField < handle
    % Represents a repulsive potential field.
    
	properties
        kRep; % repulsive constant
        d0; % distance threshold
    end

    methods
        function self = RepulsivePotentialField(kRep, d0)
            % self = RepulsivePotentialField(kRep, d0) constructs a 
            % repulsive field potential. The inputs are the repulsive 
            % constant kRep and the distance threshold d0.
            self.kRep = kRep;
            self.d0 = d0;
        end
        
        function U = computePotential(self, position, obstacle)
            % U = computePotential(self, position, obstacle) computes the
            % potential given the robot's position and the position of a
            % single obstacle. The output is the potential at the given 
            % position.
            if norm(position - obstacle) <= self.d0
                U = (1/2) * self.kRep * (1/norm(position - obstacle) - 1/self.d0)^2;
            else
                U = 0;
            end
        end
        
        function dU = computeGradient(self, position, obstacle)
            % dU = computePotential(self, position, obstacle) computes the
            % potential gradient given the robot's position and the 
            % position of a single obstacle. The output is the potential 
            % gradient at the given position.
            if norm(position - obstacle) <= self.d0
                dU = self.kRep * (1/self.d0 - 1/norm(position - obstacle)) *...
                    (1/norm(position - obstacle)^2) * (position - obstacle) / norm(position - obstacle);
            else
                dU = 0;
            end 
        end
    end
end