function [eta_e_motor_next,unicycleState_next] = ...
    simulateRobot(discModel,voltages,eta_e_motor_k,F,trackingError_next,trackingError_k,Ts,unicycleState_k,N)

        % Dynamics from motor + robot set and error integration with Tustin
        eta_e_motor_next(1:2) = discModel.Ade(1:2,1:4)*eta_e_motor_k + discModel.Bde(1:2,1:2)*voltages + F;
        eta_e_motor_next(3) = eta_e_motor_k(3) + (trackingError_next(1)+ trackingError_k(1))*(Ts/2); % Integrals
        eta_e_motor_next(4) = eta_e_motor_k(4) + (trackingError_next(2)+ trackingError_k(2))*(Ts/2); % Integrals
        
        % Integration of unicycle model 
        sol = ode45(@(t,y) unicycle(t,unicycleState_k(3),eta_e_motor_next)/N,[0,Ts],unicycleState_k);
        unicycleState_next = sol.y(:,end); % solução integração (ODE45)

end

