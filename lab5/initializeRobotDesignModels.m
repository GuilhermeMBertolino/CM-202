function [continuousTimeModel,discreteTimeModel] = initializeRobotDesignModels(Ts,N)

% Continuous time robot + motor model
Ac = [-5.23, 0.47;
    0.47, -5.23];

Bc = [57.58, -5.19;
    -5.19, 57.58]*N; 

% Extended system (integrators added)
Ace = [Ac, zeros(2);
    eye(2), zeros(2)];

Bce = [Bc;zeros(2)];

Cce = [eye(2), zeros(2)];

Dce = zeros(2);

% Discrertization
[Ade,Bde,Cde,Dde] = c2dm(Ace,Bce,Cce,Dce,Ts);

% Outputs discrete and continuous model
discreteTimeModel.Ade = Ade;
discreteTimeModel.Bde = Bde;
discreteTimeModel.Cde = Cde;
discreteTimeModel.Dde = Dde;

continuousTimeModel.Ace = Ace;
continuousTimeModel.Bce = Bce;
continuousTimeModel.Cce = Cce;
continuousTimeModel.Dce = Dce;

end

