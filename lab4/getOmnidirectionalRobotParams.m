function params = getOmnidirectionalRobotParams()
% params = getOmnidirectionalParams() returns the omnidirectional robot's
% parameters, which has 4 wheels. The following parameters are considered:
% params.alpha: the angles the wheels are distributed from the robot's
%               geometric center.
% params.beta: the angles the axes of the wheels make with the vectors that
%              connect the robot's geometric center with each wheel point 
%              of contact.
% params.l: the distances between the robot's geometric center and each
%           wheel.
% params.r: the wheel's radius.
% params.omegaMax: the maximum wheel's speed.

rpmToSI = 2.0 * pi / 60.0;

N = 3;

params.alpha = [pi / 4, 3 * pi / 4, 5 * pi / 4, 7 * pi / 4];
params.beta = [0, 0, 0, 0];
params.l = (171e-3 / 2.0) * ones(1, 4);
params.r = (64.5 + 8) * 1e-3 / 2.0;
params.omegaMax = 4380.0 * rpmToSI / N;

end