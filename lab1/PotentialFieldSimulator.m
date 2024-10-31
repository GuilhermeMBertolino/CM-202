classdef PotentialFieldSimulator < handle
    properties
        position;
        velocity;
        goal;
        obstacles;
        dt;
        time;
        mode;
        totalField;
        potential;
        command;
        history;
    end
    
    methods
        function self = PotentialFieldSimulator(fieldParams, position,...
                goal, obstacles, dt)
            self.position = position;
            self.velocity = [0; 0];
            self.goal = goal;
            self.obstacles = obstacles;
            self.dt = dt;
            self.time = 0;
            self.totalField = TotalPotentialField(fieldParams);
            self.history = struct;
        end
        
        function run(self, duration)
            numSteps = ceil(duration / self.dt);
            self.history.time = zeros(1, numSteps);
            self.history.positions = zeros(length(self.position), numSteps);
            self.history.velocities = zeros(length(self.velocity), numSteps);
            self.history.commands = zeros(length(self.position), numSteps);
            self.history.lastIndex = 0;
            for i=1:numSteps
                self.step();
            end
        end
        
        function logHistory(self)
            self.history.lastIndex = self.history.lastIndex + 1;
            i = self.history.lastIndex;
            self.history.time(i) = self.time;
            self.history.positions(:, i) = self.position;
            self.history.velocities(:, i) = self.velocity;
            self.history.potentials(:, i) = self.potential;
            self.history.commands(:, i) = self.command;
        end
        
        function step(self)
            U = self.totalField.computePotential(self.position,...
                self.goal, self.obstacles);
            dU = self.totalField.computeGradient(self.position,...
                self.goal, self.obstacles);
            self.potential = U;
            self.command = -dU;
            self.logHistory();
            self.position = self.position - dU * self.dt;
            self.time = self.time + self.dt;
        end
    end
end