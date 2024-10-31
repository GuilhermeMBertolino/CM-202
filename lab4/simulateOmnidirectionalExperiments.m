function simulateOmnidirectionalExperiments()
% simulateOmnidirectionalExperiments() simulates a series of experiments to
% evaluate the implementation of the omnidirectional robot simulator.

params = getOmnidirectionalRobotParams();

time = 0:1/60:1;

v = 0.5;
vns = -1.0:0.5:1.0;
omega = 0;

for i=1:length(vns)
    initialPose = Pose2D(0, 0, 0);
    vn = vns(i);
    outputs{i} = simulateOmnidirectional(params, initialPose, time,...
        v * ones(size(time)), vn * ones(size(time)), omega * ones(size(time)));
    legs{i} = sprintf('$v_n = %g \\ m/s$', vns(i));
end

titleText = sprintf('Omnidirectional -- $v = %g \\ m/s, \\ \\omega = %g \\ rad/s$', v, omega);

plotExperiments(params, outputs, legs, titleText, 'diagonal');

vn = 0;
omegas = -5.0:2.5:5.0;

for i=1:length(omegas)
    initialPose = Pose2D(0, 0, 0);
    omega = omegas(i);
    outputs{i} = simulateOmnidirectional(params, initialPose, time,...
        v * ones(size(time)), vn * ones(size(time)), omega * ones(size(time)));
    legs{i} = sprintf('$\\omega = %g \\ rad/s$', omegas(i));
end

titleText = sprintf('Omnidirectional -- $v = %g \\ m/s, \\ v_n = %g \\ m/s$', v, vn);

plotExperiments(params, outputs, legs, titleText, 'curve');

end

function plotExperiments(params, outputs, legs, titleText, figSuffix)

figFormat = 'png'; % for Word users
% figFormat = 'eps'; % for LaTeX users

figure;
hold on;
for i=1:length(outputs)
    plot(outputs{i}.x, outputs{i}.y, 'LineWidth', 2);
end
for i=1:length(outputs)
    plotOrientations(outputs{i}.x, outputs{i}.y, outputs{i}.psi, 10);
end
title(titleText, 'FontSize', 14, 'interpreter', 'latex');
xlabel('X (m)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Y (m)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 12, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
axis equal;
grid on;
pause(1);
saveFig(['omnidirectional_xy_', figSuffix], figFormat);

numSteps = length(outputs{i}.time) - 1;

for j=1:4
    figure;
    hold on;
    for i=1:length(outputs)
        plot(outputs{i}.time(1:numSteps), outputs{i}.omegaw(j, :), 'LineWidth', 2);
    end
    plot([outputs{i}.time(1), outputs{i}.time(end)],...
        [params.omegaMax, params.omegaMax], '--k');
    plot([outputs{i}.time(1), outputs{i}.time(end)],...
        [-params.omegaMax, -params.omegaMax], '--k');
    title(titleText, 'FontSize', 14, 'interpreter', 'latex');
    xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
    ylabel(sprintf('$\\omega_%d$ ($rad/s$)', j), 'FontSize', 14, 'interpreter', 'latex');
    legend(legs, 'FontSize', 12, 'interpreter', 'latex');
    set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
    grid on;
    pause(1);
    saveFig([sprintf('omnidirectional_omega%d_', j), figSuffix], figFormat);
end

end