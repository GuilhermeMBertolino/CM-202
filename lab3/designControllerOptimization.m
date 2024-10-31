function controller = designControllerOptimization(requirements, servo)
% controller = designControllerOptimization(requirements, servo) designs 
% the speed compensator based on requirements and the servo's parameters. 
% This function uses an approach based on optimization for the design.
% The structs requirements and servo follow the format returned by the 
% functions getRequirements() and getServoParams(), respectively. The
% struct controller has the following fields:
% controller.Kp: proportional gain.
% controller.Ki: integrative gain.
% controller.T: sample time.
% controller.Vmax: maximum voltage (for the anti-windup).

% Do not delete these assignments, they are needed for the
% digital implementation
controller.T = 1.0 / requirements.fs; % do not delete!
controller.Vmax = servo.Vmax; % do not delete!

% Using analytic solution as initial guess
controllerAnalytic = designControllerAnalytic(requirements, servo);

% Initial guess
KpGuess = controllerAnalytic.Kp;
KiGuess = controllerAnalytic.Ki;

% Options to display the optimization progress
options = optimset('Display', 'iter');

% Optimization
gains = fminsearch(@(x) speedCompensatorCost(requirements, servo, x), [KpGuess; KiGuess], options);

controller.Kp = gains(1);
controller.Ki = gains(2);

end


function cost = speedCompensatorCost(requirements, servo, gains)

T = 1.0 / requirements.fs; % sample time

Kp = gains(1);
Ki = gains(2);

dynamics = getSpeedDynamics(servo);
PI = tf([Kp, Ki], [1, 0]);
[numDelay, denDelay] = pade(T, 2);
delay = tf(numDelay, denDelay);
openLoop = PI * delay * dynamics;
closedLoop = feedback(openLoop, delay);

wb = bandwidth(closedLoop);
[~, PM] = margin(openLoop);

cost = (requirements.wb - wb)^2 + (requirements.PM - PM)^2;

end