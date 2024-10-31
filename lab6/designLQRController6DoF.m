function controller = designLQRController6DoF(LQRparams,dynamics,addIntegrator)
% In this function the LQR controller is designed. There are two
% possibilities: with or without integrator. If addIntegrator = 0, Ki = 0,
% otherwise this gain must be calculated using the command lqr. The
% saturations limits are also defined here.

%% Position controller design
switch addIntegrator
    case 1 % Design with integrator
        
        % Pos. dyn
        Ape = [0 1 0 0 0 0 0
            0 0 0 0 0 0 0
            0 0 0 1 0 0 0
            0 0 0 0 0 0 0
            0 0 0 0 0 1 0
            0 0 0 0 0 0 0
            0 0 0 0 -1 0 0];
        
        Bpe = [0 0 0;
            1/dynamics.m 0 0;
            0 0 0;
            0 1/dynamics.m 0;
            0 0 0;
            0 0 1/dynamics.m
            0 0 0];
        
        [Kpos, ~, ~] = lqr(Ape, Bpe, LQRparams.positionCtrl.Q, LQRparams.positionCtrl.R);
        

    case 0 % Case without
        % Pos. dyn
        Ap = [0 1 0 0 0 0
            0 0 0 0 0 0
            0 0 0 1 0 0
            0 0 0 0 0 0
            0 0 0 0 0 1
            0 0 0 0 0 0];
        
        Bp = [0 0 0;
            1/dynamics.m 0 0;
            0 0 0;
            0 1/dynamics.m 0;
            0 0 0;
            0 0 1/dynamics.m];
        
        [Kpos, ~, ~] = lqr(Ap, Bp, LQRparams.positionCtrl.Q, LQRparams.positionCtrl.R);
end

%% Rotation controller design
% Att. dyn
Ar = [0 1 0 0 0 0
    0 0 0 0 0 0
    0 0 0 1 0 0
    0 0 0 0 0 0
    0 0 0 0 0 1
    0 0 0 0 0 0];

Br = [0 0 0;
    1/dynamics.J(1,1) 0 0;
    0 0 0;
    0 1/dynamics.J(2,2) 0;
    0 0 0;
    0 0 1/dynamics.J(3,3)];

[Krot, ~, ~] = lqr(Ar, Br, LQRparams.attitudeCtrl.Q, LQRparams.attitudeCtrl.R);


%% Assign values
controller.x.Kp = Kpos(1,1);
controller.x.Kv = Kpos(1,2);
controller.x.fMin = -0.5 * dynamics.m * dynamics.g;
controller.x.fMax = 0.5 * dynamics.m * dynamics.g;

controller.y.Kp = Kpos(2,3);
controller.y.Kv = Kpos(2,4);
controller.y.fMin = -0.5 * dynamics.m * dynamics.g;
controller.y.fMax = 0.5 * dynamics.m * dynamics.g;

controller.z.Kp = Kpos(3,5);
controller.z.Kv = Kpos(3,6);
controller.z.fMin = 0.1 * dynamics.m * dynamics.g;
controller.z.fMax = 2.0 * dynamics.m * dynamics.g;

controller.roll.Kp = Krot(1,1);
controller.roll.Kv = Krot(1,2);
controller.roll.phirMax = 30.0 * pi / 180.0;
controller.roll.tauMin = -dynamics.d * 4.0 * dynamics.m * dynamics.g;
controller.roll.tauMax = dynamics.d * 4.0 * dynamics.m * dynamics.g;

controller.pitch.Kp = Krot(2,3);
controller.pitch.Kv = Krot(2,4);
controller.pitch.thetarMax = 30.0 * pi / 180.0;
controller.pitch.tauMin = -dynamics.d * 4.0 * dynamics.m * dynamics.g;
controller.pitch.tauMax = dynamics.d * 4.0 * dynamics.m * dynamics.g;

controller.yaw.Kp = Krot(3,5);
controller.yaw.Kv = Krot(3,6);
controller.yaw.tauMin = -dynamics.k * 4.0 * dynamics.m * dynamics.g;
controller.yaw.tauMax = dynamics.k * 4.0 * dynamics.m * dynamics.g;

if addIntegrator
    controller.z.Ki = Kpos(3,end);
else
    controller.z.Ki = 0;
end

