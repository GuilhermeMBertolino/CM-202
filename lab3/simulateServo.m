function simulateServo(controller, servo)

figFormat = 'png'; % for Word users
% figFormat = 'eps'; % for LaTeX users

% Configuring Simulink variables
assignin('base', 'controller', controller);
assignin('base', 'servo', servo);
assignin('base', 'simulationTime', 0.5);

refs = [40, 70, 100];
outs = {};

reference.stepTime = 0;
for i=1:length(refs)
    reference.amplitude = refs(i);
    assignin('base', 'reference', reference);
    legs{i} = sprintf('$r_l = %g \\ rad/s$', refs(i));
    outs{i} = sim('servomotor');
end

figure;
hold on;
for i=1:length(refs)
    plot(outs{i}.wWheel.time, outs{i}.wWheel.signals.values, 'LineWidth', 2);
end
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Wheel Speed ($rad/s$)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 14, 'interpreter', 'latex', 'location', 'southeast');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
saveFig('wheel_speed', figFormat);

figure;
hold on;
for i=1:length(refs)
    plot(outs{i}.wMotor.time, outs{i}.wMotor.signals.values, 'LineWidth', 2);
end
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Motor Speed ($rad/s$)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 14, 'interpreter', 'latex', 'location', 'southeast');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
saveFig('motor_speed', figFormat);

figure;
hold on;
for i=1:length(refs)
    plot(outs{i}.wEncoder.time, outs{i}.wEncoder.signals.values, 'LineWidth', 2);
end
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Speed Measured by Encoder ($rad/s$)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 14, 'interpreter', 'latex', 'location', 'southeast');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
grid on;
saveFig('encoder_speed', figFormat);

figure;
hold on;
for i=1:length(refs)
    plot(outs{i}.V.time, outs{i}.V.signals.values, 'LineWidth', 2);
end
xlabel('Time ($s$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('Voltage ($V$)', 'FontSize', 14, 'interpreter', 'latex');
legend(legs, 'FontSize', 14, 'interpreter', 'latex', 'location', 'southeast');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
ylim([0, controller.Vmax]);
grid on;
saveFig('voltage', figFormat);

end