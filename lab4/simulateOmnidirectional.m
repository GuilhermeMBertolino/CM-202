function output = simulateOmnidirectional(params, initialPose, time,...
    v, vn, omega)
% simulation = simulateOmnidirectional(params, initialPose, time, v, vn,...
% omega) simulates an omnidirectional robot with 4 wheels. The inputs are:
% params: the robot's physical parameters, as returned by the function 
%         getDifferentialRobotParams().
% initialPose: the robot's initial pose.
% time: a vector containing the simulation's time instants. The
%       simulation's time step is expected to be fixed.
% v: the robot's commanded forward velocity at each time step.
% vn: the robot's commanded sideways velocity at each time step.
% omega: the robot's commanded angular velocity at each time step.
% The simulation's output contain:
% simulation.x: the robot's x coordinate at each time step.
% simulation.y: the robot's y coordinate at each time step.
% simulation.psi: the robot's yaw angle at each time step.
% simulation.v: the robot's forward velocity at each time step.
% simulation.vn: the robot's sideways velocity at each time step.
% simulation.omega: the robot's angular velocity at each time step.
% simulation.omegaw: the wheel angular velocities at each time step.

simulator = OmnidirectionalRobotSimulator(initialPose, params);

output = struct;
output.time = time;
numSteps = length(output.time) - 1;

output.x = zeros(1, numSteps);
output.y = zeros(1, numSteps);
output.psi = zeros(1, numSteps);
output.v = v;
output.vn = vn;
output.omega = omega;
kinematics = OmnidirectionalRobotKinematics(params);
output.omegaw = zeros(4, numSteps);

for i=1:numSteps
    output.x(i) = simulator.pose.x;
    output.y(i) = simulator.pose.y;
    output.psi(i) = simulator.pose.psi;
    dt = time(i + 1) - time(i);
    omegaw = kinematics.inverseKinematics(v(i), vn(i), omega(i));
    output.omegaw(:, i) = omegaw;
    simulator.step(omegaw, dt);
end

end