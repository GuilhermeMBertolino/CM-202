function simulateFollowlineExperiments()
% simulateFollowlineExperiments() simulates a series of experiments for a
% followline robot based on a differential robot dynamics.

figFormat = 'png'; % for Word users
% figFormat = 'eps'; % for LaTeX users

xr = 1.0;
yrs = -1.0:0.5:1.0;

duration = 4;

for i=1:length(yrs)
    yr = yrs(i);
    outputs{i} = simulateFollowline(xr, yr, duration);
    legs{i} = sprintf('$y_r = %g \\ m$', yr); 
end

figure;
hold on;
for i=1:length(yrs)
    plot(outputs{i}.x, outputs{i}.y, 'LineWidth', 2);
end
for i=1:length(outputs)
    plotOrientations(outputs{i}.x, outputs{i}.y, outputs{i}.psi, 10);
end
title(sprintf('Followline -- $x_r = %g \\ m$', xr), 'FontSize', 14, 'interpreter', 'latex');
xlabel('X (m)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Y (m)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 12, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
axis equal;
grid on;
pause(1);
saveFig('followline_xy', figFormat);

params = getDifferentialRobotParams();

figure;
hold on;
for i=1:length(yrs)
    plot(outputs{i}.time, outputs{i}.omegaR, 'LineWidth', 2);
end
plot([outputs{i}.time(1), outputs{i}.time(end)],...
    [params.omegaMax, params.omegaMax], '--k');
plot([outputs{i}.time(1), outputs{i}.time(end)],...
    [-params.omegaMax, -params.omegaMax], '--k');
title(sprintf('Followline -- $x_r = %g \\ m$', xr), 'FontSize', 14, 'interpreter', 'latex');
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('$\omega_r$ ($rad/s$)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 12, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
pause(1);
saveFig('followline_omegar', figFormat);

figure;
hold on;
for i=1:length(yrs)
    plot(outputs{i}.time, outputs{i}.omegaL, 'LineWidth', 2);
end
plot([outputs{i}.time(1), outputs{i}.time(end)],...
    [params.omegaMax, params.omegaMax], '--k');
plot([outputs{i}.time(1), outputs{i}.time(end)],...
    [-params.omegaMax, -params.omegaMax], '--k');
title(sprintf('Followline -- $x_r = %g \\ m$', xr), 'FontSize', 14, 'interpreter', 'latex');
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('$\omega_l$ ($rad/s$)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 12, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
pause(1);
saveFig('followline_omegal', figFormat);

end