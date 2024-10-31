function servo = getServoParams()
% servo = getServoParams() obtains parameters for the VSS wheel servo. 
% Please, refer to the VSS modeling report for more information.
% The parameters are:
% servo.R: resistance.
% servo.L: inductance (neglected).
% servo.Kt: torque constant.
% servo.Beq: equivalent damping (as seen from the motor).
% servo.Jeq: equivalent inertia (as seen from the motor).
% servo.eta: transmission efficiency.
% servo.encoderQuantization: quantization introduced by the encoder.

iFree = 0.12; % from Pololu
iStall = 1.6; % from Pololu
V = 6; % from Pololu
servo.N = 51.45; % from Pololu
omegaFree = servo.N * 625 * 2 * pi / 60; % from Pololu
a = 36.73; % from Okuyama's motor modeling paper (pole)
TStall = 0.1059; % from Pololu
servo.R = V / iStall; % resistance
servo.L = 0; % inductance (neglected)
servo.Kt = (V - servo.R * iFree) / omegaFree; % torque constant
servo.Beq = servo.Kt * iFree / omegaFree; % equivalent damping (from motor)
servo.Jeq = (servo.R * servo.Beq + servo.Kt^2) / (servo.R * a); % equivalent inertia (from motor)
servo.eta = TStall / (servo.Kt * iStall * servo.N);
servo.cpr = 12; % cpr = counts per revolution
servo.encoderQuantization = 2.0 * pi / servo.cpr;
servo.Vmax = 8.4; % maximum voltage provided by the battery

end