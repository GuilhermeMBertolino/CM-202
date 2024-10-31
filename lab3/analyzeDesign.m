function analyzeDesign(controller, servo)
% analyzeDesign(controller, servo) analyzes the controller designed for the
% speed servo.

figFormat = 'png'; % for Word users
% figFormat = 'eps'; % for LaTeX users

% Obtaining the transfer function
dynamics = getSpeedDynamics(servo);

sampleTime = controller.T;
[numSpeedDelay, denSpeedDelay] = pade(sampleTime / 2.0, 2);
tfSpeedDelay = tf(numSpeedDelay, denSpeedDelay);
openLoopSpeed = tf([controller.Kp, controller.Ki], [1 0]) * dynamics;

% Analyzing gain and phase margins
figure;
margin(openLoopSpeed * tfSpeedDelay * tfSpeedDelay);
saveFig('margin_speed', figFormat);

closedLoopSpeed = feedback(openLoopSpeed * tfSpeedDelay, tfSpeedDelay);
figure;
% Analyzing step response
step(closedLoopSpeed);
grid on
saveFig('step_speed', figFormat);

figure;
% Analyzing frequency response
bode(closedLoopSpeed);
saveFig('bode_speed', figFormat);
fprintf('bandwidth: %g Hz\n', bandwidth(closedLoopSpeed) / 2 / pi);

end