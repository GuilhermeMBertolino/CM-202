function plotSimulationResults6DoF(simulation)
% tracarGraficosSimulacao(simulacao) traca graficos de uma simulacao de
% voo do multicoptero.

format = 'png'; % for Word users
% format = 'eps'; % for LaTeX users

% figure;
% hold on;
% plot3(simulation.Xg.signals.values(:, 1), simulation.Xg.signals.values(:, 2),...
%     simulation.Xg.signals.values(:, 3), 'LineWidth', 2);
% xlabel('X (m)', 'FontSize', 14);
% ylabel('Y (m)', 'FontSize', 14);
% zlabel('Z (m)', 'FontSize', 14);
% set(gca, 'FontSize', 14);
% grid on;
% axis equal;
% view(-45, 30);
% savePlot(sprintf('x_z_%c', simulation.experiment), format);

figure;
subplot(3, 1, 1);
plot(simulation.Xr.time, simulation.Xr.signals.values(:, 1), 'r', 'LineWidth', 2);
hold on;
plot(simulation.Xg.time, simulation.Xg.signals.values(:, 1), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12,'Interpreter','latex');
ylabel('$x$ (m)', 'FontSize', 12,'Interpreter','latex');
set(gca, 'FontSize', 12);
legend('$\bar{x}$', '$x$','Interpreter','latex');
grid on;

subplot(3, 1, 2);
plot(simulation.Xr.time, simulation.Xr.signals.values(:, 2), 'r', 'LineWidth', 2);
hold on;
plot(simulation.Xg.time, simulation.Xg.signals.values(:, 2), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12,'Interpreter','latex');
ylabel('$y$ (m)', 'FontSize', 12,'Interpreter','latex');
set(gca, 'FontSize', 12);
legend('$\bar{y}$', '$y$','Interpreter','latex');
grid on;

subplot(3, 1, 3);
plot(simulation.Xr.time, simulation.Xr.signals.values(:, 3), 'r', 'LineWidth', 2);
hold on;
plot(simulation.Xg.time, simulation.Xg.signals.values(:, 3), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12,'Interpreter','latex');
ylabel('$z$ (m)', 'FontSize', 12,'Interpreter','latex');
set(gca, 'FontSize', 12);
legend('$\bar{z}$', '$z$','Interpreter','latex');
grid on;

figure;
subplot(3, 1, 1);
plot(simulation.eulerr.time, simulation.eulerr.signals.values(:, 1), 'r', 'LineWidth', 2);
hold on;
plot(simulation.euler.time, simulation.euler.signals.values(:, 1), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12,'Interpreter','latex');
ylabel('$\phi$ (rad)', 'FontSize', 12,'Interpreter','latex');
set(gca, 'FontSize', 12);
legend('$\bar{\phi}$', '$\phi$','Interpreter','latex');
grid on;

subplot(3, 1, 2);
plot(simulation.eulerr.time, simulation.eulerr.signals.values(:, 2), 'r', 'LineWidth', 2);
hold on;
plot(simulation.euler.time, simulation.euler.signals.values(:, 2), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12,'Interpreter','latex');
ylabel('$\theta$ (rad)', 'FontSize', 12,'Interpreter','latex');
set(gca, 'FontSize', 12);
legend('$\bar{\theta}$', '$\theta$','Interpreter','latex');
grid on;

subplot(3, 1, 3);
plot(simulation.eulerr.time, simulation.eulerr.signals.values(:, 3), 'r', 'LineWidth', 2);
hold on;
plot(simulation.euler.time, simulation.euler.signals.values(:, 3), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12,'Interpreter','latex');
ylabel('$\psi$ (rad)', 'FontSize', 12,'Interpreter','latex');
set(gca, 'FontSize', 12);
legend('$\bar{\psi}$', '$\psi$','Interpreter','latex');
grid on;

figure;
subplot(2, 1, 1);
plot(simulation.Vg.time, simulation.Vg.signals.values, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 14,'Interpreter','latex');
ylabel('$V_g$ (m/s)', 'FontSize', 14,'Interpreter','latex');
set(gca, 'FontSize', 14);
legend('$v_x$', '$v_y$', '$v_z$','Interpreter','latex');
grid on;

subplot(2, 1, 2);
plot(simulation.wb.time, simulation.wb.signals.values, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 14,'Interpreter','latex');
ylabel('$\omega_b$ (rad/s)', 'FontSize', 14,'Interpreter','latex');
set(gca, 'FontSize', 14);
legend('$p$', '$q$', '$r$','Interpreter','latex');
grid on;

figure;
subplot(2, 1, 1);
plot(simulation.f.time, simulation.f.signals.values, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 14,'Interpreter','latex');
ylabel('$f_{z,B}$ (N)', 'FontSize', 14,'Interpreter','latex');
set(gca, 'FontSize', 14);
grid on;

subplot(2, 1, 2);
plot(simulation.Mxyz.time, simulation.Mxyz.signals.values, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 14,'Interpreter','latex');
ylabel('$\tau$ (N m)', 'FontSize', 14,'Interpreter','latex');
set(gca, 'FontSize', 14);
legend('$\tau_x$', '$\tau_y$', '$\tau_z$','Interpreter','latex');
grid on;


end