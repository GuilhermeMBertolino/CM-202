function controller = designControllerAnalytic(requirements, servo)
% controller = designCompensatorOptimization(requirements, servo) designs 
% the speed compensator based on requirements and the servo's parameters.
% This function uses an analytic approach for the design.
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

xi = requirements.PM / 100;

wn = requirements.wb / sqrt(1 - 2 * xi^2 + sqrt(4 * xi^4 - 4 * xi^2 + 2));

% Implement
Ki = wn^2 * servo.Jeq * servo.R / servo.Kt;
Kp = 2 * xi * wn * servo.Jeq * servo.R / servo.Kt - servo.Beq * servo.R / servo.Kt - servo.Kt;

controller.Kp = Kp;
controller.Ki = Ki;

end