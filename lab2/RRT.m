classdef RRT < handle
    % Represents the Rapidly-Exploring Random Tree (RRT) algorithm.
    properties
        tree; % the tree constructed during path planning
        obstacles; % the obstacles
        goalBias; % the goal bias probability
        delta; % the delta used in the extend operation
        limits; % the bounds of the search space
        epsilon = 1e-3 % value to check if we reached the goal
    end
    
    methods
        function self = RRT(obstacles, goalBias, delta, limits)
            % self = RRT(obstacles, goalBias, delta, limits) constructs the
            % Rapidly-Exploring Random Tree (RRT) algorithm. The inputs
            % are:
            % obstacles: the obstacles (array of Obstacle).
            % goalBias: goal bias probability.
            % delta: the delta used in the extend operation.
            % limits: the limits of the search space. It is a struct with 
            %         the fields xMin, xMax, yMin, and yMax representing
            %         the minimum x coordinate, maximum x coordinate,
            %         minimum y coordinate, and maximum y coordinate,
            %         respectively.
            self.tree = Tree(10000);
            self.obstacles = obstacles;
            self.goalBias = goalBias;
            self.delta = delta;
            self.limits = limits;
        end
        
        function collisionFree = checkCollisionFree(self, position)
            % collisionFree = checkCollisionFree(self, position) checks if
            % a given position is collision free. The output collisionFree
            % is a boolean.
            collisionFree = true;
            for obstacle = self.obstacles
                if obstacle.checkCollision(position)
                    collisionFree = false;
                    return;
                end
            end
        end
        
        function sample = sampleFreeSpace(self)
            % sample = sampleFreeSpace(self) samples from the free space
            % using reject sampling, i.e. the algorithm keeps on sampling
            % new points until the sampled point belongs to the free space.
            % The output sample is a 2x1 column vector.
            while true
                sample = [rand() * (self.limits.xMax - self.limits.xMin) + self.limits.xMin;
                          rand() * (self.limits.yMax - self.limits.yMin) + self.limits.yMin];
                if self.checkCollisionFree(sample)
                    break;
                end
            end
        end
        
        function [path, length, numIterations] = planPath(self, source, goal, maxIterations)
            % [path, length, numIterations] = planPath(self, source, goal, 
            % maxIterations) plans a path from the source to the goal. The 
            % variable maxIterations represents the maximum number of 
            %^iterations for the RRT algorithm. The outputs path and length
            % represent the planned path (as a 2 x p matrix representing 
            % a path of p positions) and the path length, respectively.
            % Finally, numItertations is the number of iterations the
            % algorithm ran until stopping.
            self.tree.reset(); % to clear the tree
            % 0 is used here to represent null, i.e. to represent that the 
            % root does not have a parent
            self.tree.insert(source, 0);
            for i=1:maxIterations
                r = rand();
                if(r < self.goalBias)
                    randomNode = goal;
                else
                    randomNode = self.sampleFreeSpace();
                end
                nearestNode = self.tree.findNearest(randomNode);
                newNode = nearestNode.extend(randomNode, self.delta);
                if self.checkCollisionFree(newNode)
                    self.tree.insert(newNode, nearestNode.index);
                    if norm(newNode - goal) < self.epsilon
                        [path, length] = self.reconstructPath();
                        numIterations = i;
                        return;
                    end
                end
            end
            % If the path was not found, return an empty path and an
            % infinite length (use MATLAB's Inf for infinite)
            path = [];
            length = Inf;
            numIterations = maxIterations;
        end
        
        function [path, length] = reconstructPath(self)
            % [path, length] = reconstructPath(self) reconstructs the path
            % planned by the RRT. The outputs path and length are the
            % planned path ()
            path = self.tree.reconstructPath(self.tree.numNodes);
            length = 0.0;
            for i=2:size(path, 2)
                length = length + norm(path(:, i) - path(:, i - 1));
            end
        end
    end
end