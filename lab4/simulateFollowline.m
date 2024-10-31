function simulation = simulateFollowline(xr, yr, duration)
% simulation = simulateFollowline(xr, yr, duration) simulates a 
% followline controller for a differential robot. The inputs are:
% xr: the reference x coordinate.
% yr: the reference y coordinate.
% duration: the simulation's duration.
% The simulation's output contain:
% simulation.x: the robot's x coordinate at each time step.
% simulation.y: the robot's y coordinate at each time step.
% simulation.psi: the robot's yaw angle at each time step.
% simulation.v: the robot's linear velocity at each time step.
% simulation.omega: the robot's angular velocity at each time step.
% simulation.omegaR: the right wheel's angular velocity at each time step.
% simulation.omegaL: the left wheel's angular velocity at each time step.

wbx = 2.0 * pi * 1.0;
omegan = 2.0 * pi * 3.0;
xi = 0.7;

controllerParams.Kx = wbx;
controllerParams.KyPrime = omegan / (2.0 * xi);
controllerParams.Kpsi = 2.0 * xi * omegan;
controllerParams.vMin = 0.1;
controllerParams.vMax = 0.8;
controllerParams.psiMax = 80.0 * pi / 180.0;

controller = FollowlineController(controllerParams);

dt = 1 / 60;

time = 0:dt:duration;

initialPose = Pose2D(0, 0, 0);
simulation.time = time;
simulation.x = zeros(length(time), 1);
simulation.y = zeros(length(time), 1);
simulation.psi = zeros(length(time), 1);
simulation.v = zeros(length(time), 1);
simulation.omega = zeros(length(time), 1);
simulation.omegaR = zeros(length(time), 1);
simulation.omegaL = zeros(length(time), 1);

params = getDifferentialRobotParams();

simulator = DifferentialRobotSimulator(initialPose, params);
kinematics = DifferentialRobotKinematics(params);

for i=1:length(time)
    pose = simulator.pose;
    simulation.x(i) = pose.x;
    simulation.y(i) = pose.y;
    simulation.psi(i) = pose.psi;
    [v, omega] = controller.control(xr, yr, pose.x, pose.y, pose.psi);
    simulation.v(i) = v;
    simulation.omega(i) = omega;
    [omegaR, omegaL] = kinematics.inverseKinematics(v, omega);
    omegaR = saturate(omegaR, -params.omegaMax, params.omegaMax);
    omegaL = saturate(omegaL, -params.omegaMax, params.omegaMax);
    simulation.omegaR(i) = omegaR;
    simulation.omegaL(i) = omegaL;
    simulator.step(omegaR, omegaL, dt);
end

end