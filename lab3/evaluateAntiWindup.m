function evaluateAntiWindup(controller, servo)
% evaluateAntiWindup(controller, servo) evaluates the anti-windup
% implementation.

figFormat = 'png'; % for Word users
% figFormat = 'eps'; % for LaTeX users

reference.stepTime = 0;
reference.amplitude = 80;

% Configuring Simulink variables
assignin('base', 'reference', reference);
assignin('base', 'controller', controller);
assignin('base', 'servo', servo);
assignin('base', 'simulationTime', 0.5);

% Using a very high controller.Vmax to effectivelly disable
% the anti-windup strategy
controller.Vmax = 1000.0;
assignin('base', 'controller', controller);
outWindup = sim('servomotor');

% Returing to the correct controller.Vmax
controller.Vmax = servo.Vmax;
assignin('base', 'controller', controller);
out = sim('servomotor');

figure;
plot(out.wWheel.time, out.wWheel.signals.values, 'LineWidth', 2);
hold on;
plot(outWindup.wWheel.time, outWindup.wWheel.signals.values, 'r', 'LineWidth', 2);
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Wheel Speed ($rad/s$)', 'FontSize', 14, 'interpreter', 'latex');
legend({'With anti-windup', 'Without anti-windup'}, 'FontSize', 14, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
saveFig('speed_anti_windup', figFormat);

figure;
plot(out.V.time, out.V.signals.values, 'LineWidth', 2);
hold on;
plot(outWindup.V.time, outWindup.V.signals.values, 'r', 'LineWidth', 2);
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Voltage Command ($V$)', 'FontSize', 14, 'interpreter', 'latex');
legend({'With anti-windup', 'Without anti-windup'}, 'FontSize', 14, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
saveFig('voltage_anti_windup', figFormat);

end
