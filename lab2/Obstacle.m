classdef Obstacle < handle
    % Represents a circular obstacle.
    properties
        center; % the center of the obstacle
        radius; % the radius of the obstacle
    end
    
    methods 
        function self = Obstacle(center, radius)
            % self = Obstacle(center, radius) constructs a circular
            % obstacle given its center and radius.
            self.center = center;
            self.radius = radius;
        end
        
        function collision = checkCollision(self, position)
            % collision = checkCollision(self, position) checks if a
            % position collides with the obstacle.
            collision = norm(position - self.center) <= self.radius;
        end
    end
end