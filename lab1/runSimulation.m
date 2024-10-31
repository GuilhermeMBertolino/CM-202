function runSimulation(simulationCase)
% runSimulation(simulationCase) runs a potential field simulation. The
% simulation case is defined by the character simulationCase.

figFormat = 'png'; % for Word users
% figFormat = 'eps'; % for LaTeX users (better quality)

%% Potential field parameters
fieldParams.kAtt = 1.0;
fieldParams.d0Att = 3.0;
fieldParams.kRep = 1.0;
fieldParams.d0Rep = 3.0;

%% Configuring each simulation case
if simulationCase == 'a'
    position = [10; 10];
    goal = [5; 5];
    obstacles = [];
elseif simulationCase == 'b'
    position = [10; 10];
    goal = [0; 0];
    obstacles = [8.1, 8.4, 4.5, 2; 8.5, 6.5, 5, 1.1];
elseif simulationCase == 'c'
    position = [5; 10];
    goal = [5; 0];
    obstacles = [4.6, 5.4; 5, 5];
end

%% Creating and running the simulation

dt = 0.1; % Simulation timestep

simulator = PotentialFieldSimulator(fieldParams, position,...
                goal, obstacles, dt);

% Running the simulation
simulator.run(10.0);

%% Creating the potential field for plotting purposes
field = TotalPotentialField(fieldParams);

%% Plotting the vector field
x = -0:0.5:10;
y = -0:0.5:10;
figure;
PotentialFieldDrawer.drawVectorField(field, x, y, goal, obstacles);
xlim([x(1) x(end)]);
ylim([y(1) y(end)]);
axis square;
saveFig(sprintf('vector_%s.%s', simulationCase, figFormat), figFormat);

%% Plotting the potential field using color
x = -0:0.1:10;
y = -0:0.1:10;
figure;
PotentialFieldDrawer.drawPotentialColor(field, x, y, goal, obstacles);
shading interp;
hold on;
ARROW_SCALE = 0.15;
plot(simulator.history.positions(1, :), simulator.history.positions(2, :));
for i=1:2:simulator.history.lastIndex
    p0 = simulator.history.positions(:, i);
    p1 = p0 + ARROW_SCALE * simulator.history.commands(:, i);
    drawArrow(p0, p1, 'r');
end
xlim([x(1) x(end)]);
ylim([y(1) y(end)]);
axis square;
saveFig(sprintf('potential_color_%s.%s', simulationCase, figFormat), figFormat);

%% Plotting the potential field in 3D
figure;
hold on;
PotentialFieldDrawer.drawPotential3D(field, x, y, goal, obstacles);
plot3(simulator.history.positions(1, :), simulator.history.positions(2, :),...
    simulator.history.potentials, 'r', 'LineWidth', 2);
xlim([x(1) x(end)]);
ylim([y(1) y(end)]);
grid on;
axis square;
saveFig(sprintf('potential_3d_%s.%s', simulationCase, figFormat), figFormat);

end

function saveFig(filename, figFormat)
% saveFig(filename, figFormat) saves the current figure.

if strcmp(figFormat, 'png')
    print('-dpng', '-r400', filename);
else
    print('-depsc2', filename);
end

end