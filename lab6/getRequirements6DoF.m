function requirements = getRequirements6DoF()
% requirements = getRequirements6DoF() obtains the requirements for the
% flight control system of the 6DoF drone. The output requirements has the
% following fields (all units in SI):
% requirements.x.wn: natural frequency of the x channel.
% requirements.x.wn: damping factor of the x channel.
% requirements.y.wn: natural frequency of the y channel.
% requirements.y.wn: damping factor of the y channel.
% requirements.z.wn: natural frequency of the z channel.
% requirements.z.wn: damping factor of the z channel.
% requirements.roll.wn: natural frequency of the roll channel.
% requirements.roll.wn: damping factor of the roll channel.
% requirements.pitch.wn: natural frequency of the pitch channel.
% requirements.pitch.wn: damping factor of the pitch channel.
% requirements.yaw.wn: natural frequency of the yaw channel.
% requirements.yaw.wn: damping factor of the yaw channel.

requirements.x.wn = 2.0 * pi * 0.5;
requirements.x.xi = 0.7;

requirements.y.wn = 2.0 * pi * 0.5;
requirements.y.xi = 0.7;

requirements.z.wn = 2.0 * pi * 0.5;
requirements.z.xi = 0.7;

requirements.roll.wn = 2.0 * pi * 3.0;
requirements.roll.xi = 0.7;

requirements.pitch.wn = 2.0 * pi * 3.0;
requirements.pitch.xi = 0.7;

requirements.yaw.wn = 2.0 * pi * 3.0;
requirements.yaw.xi = 0.7;

end