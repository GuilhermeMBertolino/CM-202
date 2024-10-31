function simulation = simulateDrone6DoF(controller, dynamics, tf, xr, yr, zr, psir)
% simulation = simulateDrone6DoF(controller, dynamics, tf, xr, zr) 
% simulates the flight of a 6DoF drone. The inputs controller and dynamics
% are obtained through the functions designController3DoF() and
% getDroneDynamics3DoF(). The inputs tf, xr, and zr are the simulation
% duration, the x trajectory reference, and the z trajectory reference,
% respectively. The output simulation contains trajectories generated by
% the simulation.

% Configuring the variables in Simulink
assignin('base', 'xr', xr);
assignin('base', 'yr', yr);
assignin('base', 'zr', zr);
assignin('base', 'psir', psir);
assignin('base', 'x0', 0);
assignin('base', 'y0', 0);
assignin('base', 'z0', 1);
assignin('base', 'phi0', 0);
assignin('base', 'theta0', 0);
assignin('base', 'psi0', 0);
assignin('base', 'controller', controller);
assignin('base', 'dynamics', dynamics);

% Loading the Simulink model
load_system('drone_6dof');

% Configuring the stop time of the simulation
set_param('drone_6dof', 'StopTime', sprintf('%g', tf));

% Running the simulation
simulation = sim('drone_6dof');

end