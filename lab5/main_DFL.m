%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Instituto Tecnológico de Aeronáutica (ITA)
% CM-202 - Planejamento e controle para robótica móvel - 2021-2
% Laboratório 3 - Controle de robôs diferenciais
% Professor: Angelo Caregnato Neto
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the main script which runs the dynamic feedback linearization
% technique
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars; close all; clc;

% Loads pre-computed trajectory
load('traj.mat');

% Some robot parameters
robotParams.L = 0.0370; % robot diameter/2
robotParams.R = 0.0300; % wheel radius
robotParams.N = 51.45; % reduction factor

% Sampling times (external and internal)
Ts_ext = 1/60; %(60Hz)
Ts_int = 1/(1*240); %(240Hz)

% Initialize state space models
[contModel,discModel] = initializeRobotDesignModels(Ts_int,robotParams.N);

% Dry friction model parameters
dryFrictionModelParams.Fsr = 0.75; dryFrictionModelParams.Fkr = 0.41;
dryFrictionModelParams.Fsl = 0.7;  dryFrictionModelParams.Fkl = 0.21;
dryFrictionModelParams.alpha_s = 5.19; dryFrictionModelParams.alpha_k = 0.39;

% Internal controller design
K = buildInternalController(contModel);

% Simulation parameters
simulationTime = traj.optHor-1;
kfinal = simulationTime/Ts_ext;
jfinal = Ts_ext/Ts_int;

% Pre-allocations
eta_e_motor = zeros(4,jfinal*kfinal+1); etaBar_motor = zeros(2,kfinal);
motorVoltages = zeros(2,jfinal*kfinal);
angVelError = zeros(2,jfinal*kfinal+1);
xUni = zeros(3,jfinal*kfinal+1); vUni = zeros(2,kfinal+1);
nuBar = zeros(1,kfinal+1); d_nu = zeros(1,kfinal+1); nuBar(1) = 0.1;
omegaBar = zeros(1,kfinal+1);
jindex = 1;

%% Simulation
for k=1:kfinal % External loop
    
    refs = assignReferences(traj,k);
    
    [nuBar(k+1),omegaBar(k),d_nu(k+1)] = DFL_compensator(refs,xUni(:,jindex+1),vUni(:,k),Ts_ext,nuBar(k),d_nu(k));
    
    etaBar_motor(:,k) = inverseKinematics(nuBar(k),omegaBar(k),robotParams);
    
    for j=1:jfinal % internal loop
        jindex = (k-1)*jfinal+j;
        
        % internal control law (full state feedback)
        motorVoltages(:,jindex) = -K*eta_e_motor(:,jindex);
        
        % motor ang vel tracking error
        angVelError(:,jindex+1) = (eta_e_motor(1:2,jindex)-etaBar_motor(:,k));
        
        % dry friction
        F = dryFrictionModel(dryFrictionModelParams,eta_e_motor(1:2,k));
        
        % Robot dynamics and kinematics
        [eta_e_motor(:,jindex+1),xUni(:,jindex+1)] = ...
            simulateRobot(discModel,motorVoltages(:,jindex),eta_e_motor(:,jindex),F,angVelError(:,jindex+1),...
            angVelError(:,jindex),Ts_int,xUni(:,jindex),robotParams.N);
        
        % Unicycle linear velocity components (x,y)
        vUni(1,k+1) = (xUni(1,jindex+1)-xUni(1,jindex))/Ts_int;
        vUni(2,k+1) = (xUni(2,jindex+1)-xUni(2,jindex))/Ts_int;
        
    end
end
%% Plots
plotFigures(jfinal,kfinal,Ts_int,Ts_ext,simulationTime,motorVoltages,traj,xUni,vUni);
