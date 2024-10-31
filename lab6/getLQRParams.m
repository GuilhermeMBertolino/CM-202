function LQRparams = getLQRParams(addIntegrator)

switch addIntegrator
    
    case 1 % if theres is an integrator, one more weight value
        LQRparams.positionCtrl.Q = diag(1*[1 .1 1 .1 1 .1 1.5]); 
    case 0
        LQRparams.positionCtrl.Q = diag(1*[1 .1 1 .1 1 .1]);      
end

% Only the Q matrix changes depending on the integrator
LQRparams.positionCtrl.R = diag([10 10 10]);

% Rotation design remains the same
LQRparams.attitudeCtrl.Q = diag(5*[1 .05 1 .05 1 .05]);
LQRparams.attitudeCtrl.R = diag([10 10 10]);

end

