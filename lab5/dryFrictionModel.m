function F = dryFrictionModel(params,motorAxisVel)

    F = [params.Fsr*tanh(params.alpha_s*motorAxisVel(1)) - params.Fkr*tanh(params.alpha_k*motorAxisVel(1));
         params.Fsl*tanh(params.alpha_s*motorAxisVel(2)) - params.Fkl*tanh(params.alpha_k*motorAxisVel(2))];
end

