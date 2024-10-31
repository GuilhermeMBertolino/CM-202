function dynamics = getSpeedDynamics(servo)
% dynamics = getSpeedDynamics(servo) obtains the motor transfer function 
% given the servo parameters, stored in the struct servo.

dynamics = tf(servo.Kt, [servo.Jeq * servo.R, servo.Beq * servo.R + servo.Kt^2]);

end