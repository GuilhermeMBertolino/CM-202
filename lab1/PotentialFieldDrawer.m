classdef PotentialFieldDrawer < handle
    % Auxiliary class to generate plots related to potential fields.
    
    methods(Static)
        function drawVectorField(field, x, y, goal, obstacles)
            % drawVectorField(field, x, y, goal, obstacles) plots the
            % vector field associated to the potential field called field.
            % The x and y vectors contain the coordinates x and y of the
            % positions where the vector field will be evaluated. The
            % inputs goal and obstacles represent the goal position and the
            % obstacles (in matrix form). If no obstacle is present, then
            % obstacles must be an empty matrix.
            ARROW_SIZE = 0.15;
            MAX_GRADIENT = 3.0;
            [X, Y] = meshgrid(x, y);
            hold on;
            for i=1:size(X, 1)
                for j=1:size(X, 2)
                    dU = field.computeGradient([X(i, j); Y(i, j)], goal, obstacles);
                    % Limit the gradient magnitude to avoid having arrows
                    % with very different sizes
                    if norm(dU) > MAX_GRADIENT
                        dU = MAX_GRADIENT * dU / norm(dU);
                    end
                    drawArrow([X(i, j); Y(i, j)], [X(i, j); Y(i, j)] - ARROW_SIZE * [dU(1); dU(2)], 'r');
                end
            end
            title('Vector Field', 'interpreter', 'latex');
            grid on;
            xlabel('X', 'FontSize', 12, 'interpreter', 'latex');
            ylabel('Y', 'FontSize', 12, 'interpreter', 'latex');
            set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
        end
        
        function drawPotentialColor(field, x, y, goal, obstacles)
            % drawPotentialColor(field, x, y, goal, obstacles) plots the
            % potential associated to the potential field called field.
            % The x and y vectors contain the coordinates x and y of the
            % positions where the vector field will be evaluated. The
            % inputs goal and obstacles represent the goal position and the
            % obstacles (in matrix form). If no obstacle is present, then
            % obstacles must be an empty matrix.
            % The plot is 2D with the color at each point representing the
            % potential value.
            MAX_POTENTIAL = 60.0;
            [X, Y] = meshgrid(x, y);
            U = zeros(size(X));
            for i=1:size(X, 1)
                for j=1:size(X, 2)
                    U(i, j) = field.computePotential([X(i, j); Y(i, j)], goal, obstacles);
                    % Limit the potential so the infinity potential close 
                    % to a obstacle's center does not harm the
                    % visualization
                    if U(i, j) > MAX_POTENTIAL
                        U(i, j) = MAX_POTENTIAL;
                    end
                end
            end
            pcolor(X, Y, U);
            c = colorbar;
            c.TickLabelInterpreter = 'latex';
            title('Potential Field -- Top View', 'interpreter', 'latex');
            xlabel('X', 'FontSize', 12, 'interpreter', 'latex');
            ylabel('Y', 'FontSize', 12, 'interpreter', 'latex');
            set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
        end
        
        function drawPotential3D(field, x, y, goal, obstacles)
            % drawPotential3D(field, x, y, goal, obstacles) plots the
            % potential associated to the potential field called field.
            % The x and y vectors contain the coordinates x and y of the
            % positions where the vector field will be evaluated. The
            % inputs goal and obstacles represent the goal position and the
            % obstacles (in matrix form). If no obstacle is present, then
            % obstacles must be an empty matrix.
            % The plot is 3D with the color and height at each point 
            % representing the potential value.
            SPACING = 8;
            [X, Y] = meshgrid(x, y);
            U = zeros(size(X));
            for i=1:size(X, 1)
                for j=1:size(X, 2)
                    U(i, j) = field.computePotential([X(i, j); Y(i, j)], goal, obstacles);
                    if U(i, j) > 60.0
                        U(i, j) = 60.0;
                    end
                end
            end
            s = surf(X, Y, U);
            s.EdgeColor = 'none';
            % We draw the lines with a coarser spacing for better
            % visualization
            for i=1:SPACING:length(X(:, 1))
                plot3(X(:, i), Y(:, i), U(:, i), '-k');
                plot3(X(i, :), Y(i, :), U(i, :), '-k');
            end
            view([-20 10]);
            title('Potential Field 3D', 'interpreter', 'latex');
            grid on;
            xlabel('X', 'FontSize', 12, 'interpreter', 'latex');
            ylabel('Y', 'FontSize', 12, 'interpreter', 'latex');
            zlabel('Z', 'FontSize', 12, 'interpreter', 'latex');
            set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
        end
    end
end