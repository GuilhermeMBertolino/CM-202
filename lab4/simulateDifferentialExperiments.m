function simulateDifferentialExperiments()
% simulateDifferentialExperiments() simulates a series of experiments to
% evaluate the implementation of the differential robot simulator.

figFormat = 'png'; % for Word users
% figFormat = 'eps'; % for LaTeX users

v = 1.0;
omegas = -5.0:2.5:5.0;

params = getDifferentialRobotParams();

time = 0:1/60:1;

for i=1:length(omegas)
    initialPose = Pose2D(0, 0, 0);
    omega = omegas(i);
    outputs{i} = simulateDifferential(params, initialPose, time,...
        v * ones(size(time)), omega * ones(size(time)));
    legs{i} = sprintf('$\\omega = %g \\ rad/s$', omegas(i));
end

figure;
hold on;
for i=1:length(omegas)
    plot(outputs{i}.x, outputs{i}.y, 'LineWidth', 2);
end
for i=1:length(omegas)
    plotOrientations(outputs{i}.x, outputs{i}.y, outputs{i}.psi, 10);
end
title(sprintf('Differential -- $v = %g \\ m/s$', v), 'FontSize', 14, 'interpreter', 'latex');
xlabel('X (m)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Y (m)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 12, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
axis equal;
grid on;
pause(1);
saveFig('differential_xy', figFormat);

numSteps = length(outputs{i}.time) - 1;

figure;
hold on;
for i=1:length(omegas)
    plot(outputs{i}.time(1:numSteps), outputs{i}.omegaR, 'LineWidth', 2);
end
plot([outputs{i}.time(1), outputs{i}.time(end)],...
    [params.omegaMax, params.omegaMax], '--k');
plot([outputs{i}.time(1), outputs{i}.time(end)],...
    [-params.omegaMax, -params.omegaMax], '--k');
title(sprintf('Differential -- $v = %g \\ m/s$', v), 'FontSize', 14, 'interpreter', 'latex');
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('$\omega_r$ ($rad/s$)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 12, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
pause(1);
saveFig('differential_omegar', figFormat);

figure;
hold on;
for i=1:length(omegas)
    plot(outputs{i}.time(1:numSteps), outputs{i}.omegaL, 'LineWidth', 2);
end
plot([outputs{i}.time(1), outputs{i}.time(end)],...
    [params.omegaMax, params.omegaMax], '--k');
plot([outputs{i}.time(1), outputs{i}.time(end)],...
    [-params.omegaMax, -params.omegaMax], '--k');
title(sprintf('Differential -- $v = %g \\ m/s$', v), 'FontSize', 14, 'interpreter', 'latex');
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('$\omega_l$ ($rad/s$)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 12, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
pause(1);
saveFig('differential_omegal', figFormat);

end