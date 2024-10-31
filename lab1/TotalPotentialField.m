classdef TotalPotentialField < handle
    % Represents a total potential field, which combines attractive and
    % repulsive potential fields.
    
    properties
        attractiveField; % attractive potential field
        repulsiveField; % repulsive potential field
    end
    
    methods
        function self = TotalPotentialField(fieldParams)
            % self = TotalPotentialField(fieldParams) constructs a 
            % total field potential. The struct fieldParams is:
            % fieldParams.kAtt: attractive constant.
            % fieldParams.d0Att: distance threshold used in the attractive
            %                    field.
            % fieldParams.kRep: repulsive constant.
            % fieldParams.d0Rep: distance threshold used in the repulsive
            %                    field.
            self.attractiveField = AttractivePotentialField(fieldParams.kAtt,...
                fieldParams.d0Att);
            self.repulsiveField = RepulsivePotentialField(fieldParams.kRep,...
                fieldParams.d0Rep);
        end
        
        function U = computePotential(self, position, goal, obstacles)
            % U = computePotential(self, position, goal, obstacle) computes
            % the potential given the robot's position, the goal's 
            % position, and the obstacles' positions. The output is the 
            % potential at the given position.
            if ~isempty(goal)
                U = self.attractiveField.computePotential(position, goal);
            end
            if ~isempty(obstacles)
                for i=1:size(obstacles, 2)
                    U = U + self.repulsiveField.computePotential(position, obstacles(:, i));
                end
            end
        end
        
        function dU = computeGradient(self, position, goal, obstacles)
            % U = computePotential(self, position, goal, obstacle) computes
            % the potential gradient given the robot's position, the goal's 
            % position, and the obstacles' positions. The output is the 
            % potential gradient at the given position.
            if ~isempty(goal)
                dU = self.attractiveField.computeGradient(position, goal);
            end
            if ~isempty(obstacles)
                for i=1:size(obstacles, 2)
                    dU = dU + self.repulsiveField.computeGradient(position, obstacles(:, i));
                end
            end
        end
    end
end