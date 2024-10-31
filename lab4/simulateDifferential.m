function simulation = simulateDifferential(params, initialPose, time, v,...
    omega)
% simulation = simulateDifferential(params, initialPose, time, v, omega)
% simulates a differential robot. The inputs are:
% params: the robot's physical parameters, as returned by the function 
%         getDifferentialRobotParams().
% initialPose: the robot's initial pose.
% time: a vector containing the simulation's time instants. The
%       simulation's time step is expected to be fixed.
% v: the robot's commanded linear velocity at each time step.
% omega: the robot's commanded angular velocity at each time step.
% The simulation's output contain:
% simulation.x: the robot's x coordinate at each time step.
% simulation.y: the robot's y coordinate at each time step.
% simulation.psi: the robot's yaw angle at each time step.
% simulation.v: the robot's linear velocity at each time step.
% simulation.omega: the robot's angular velocity at each time step.
% simulation.omegaR: the right wheel's angular velocity at each time step.
% simulation.omegaL: the left wheel's angular velocity at each time step.

simulator = DifferentialRobotSimulator(initialPose, params);

simulation = struct;
simulation.time = time;
numSteps = length(simulation.time) - 1;

simulation.x = zeros(1, numSteps);
simulation.y = zeros(1, numSteps);
simulation.psi = zeros(1, numSteps);
simulation.v = v;
simulation.omega = omega;
simulation.omegaR = zeros(1, numSteps);
simulation.omegaL = zeros(1, numSteps);
kinematics = DifferentialRobotKinematics(params);

for i=1:numSteps
    simulation.x(i) = simulator.pose.x;
    simulation.y(i) = simulator.pose.y;
    simulation.psi(i) = simulator.pose.psi;
    dt = time(i + 1) - time(i);
    [omegaR, omegaL] = kinematics.inverseKinematics(v(i), omega(i));
    omegaR = saturate(omegaR, -params.omegaMax, params.omegaMax);
    omegaL = saturate(omegaL, -params.omegaMax, params.omegaMax);
    simulation.omegaR(i) = omegaR;
    simulation.omegaL(i) = omegaL;
    simulator.step(omegaR, omegaL, dt);
end

end