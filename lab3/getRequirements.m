function requirements = getRequirements()
% requirements = getRequirements() obtains the requirements for the wheel
% servo controller.

requirements.wb = 2.0 * pi * 30.0; % bandwidth
requirements.PM = 50.0; % phase margin (in degrees)
requirements.fs = 200.0; % sampling frequency

end