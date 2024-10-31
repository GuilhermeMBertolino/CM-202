classdef Pose2D < handle
    % Represents a bidimensional pose, containing x and y coordinates, and
    % a yaw angle.
    properties
        x; % the x coordinate
        y; % the y coordinate
        psi; % the yaw angle
    end
    
    methods
        function self = Pose2D(x, y, psi)
            % self = Pose2D(x, y, psi) creates a bidimensional pose
            % containing x and y coordinates, and a yaw angle.
            self.x = x;
            self.y = y;
            self.psi = psi;
        end
    end
end