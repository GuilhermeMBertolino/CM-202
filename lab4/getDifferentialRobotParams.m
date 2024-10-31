function params = getDifferentialRobotParams()
% params = getDifferentialRobotParams() returns the differential robot's
% parameters. The following parameters are considered:
% params.r: the wheel's radius.
% params.l: the distance between the wheels.
% params.omegaMax: the maximum wheel's speed.

rpmToSI = 2 * pi / 60;

params.r = 0.06 / 2;
params.l = 0.075;
params.omegaMax = 590 * rpmToSI;

end