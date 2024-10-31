function analyzeDesignNoDelay(controller, servo)
% analyzeDesignNoDelay(controller, servo) analyzes the controller designed 
% for the speed servo without taking delays into account.

figFormat = 'png'; % for Word users
% figFormat = 'eps'; % for LaTeX users

% Obtaining the transfer function
dynamics = getSpeedDynamics(servo);

openLoopSpeed = tf([controller.Kp, controller.Ki], [1 0]) * dynamics;

% Analyzing gain and phase margins
figure;
margin(openLoopSpeed);
saveFig('margin_speed_nodelay', figFormat);

closedLoopSpeed = feedback(openLoopSpeed, 1);
figure;
% Analyzing step response
step(closedLoopSpeed);
grid on
saveFig('step_speed_nodelay', figFormat);

figure;
% Analyzing frequency response
bode(closedLoopSpeed);
saveFig('bode_speed_nodelay', figFormat);
fprintf('bandwidth: %g Hz\n', bandwidth(closedLoopSpeed) / 2 / pi);

end