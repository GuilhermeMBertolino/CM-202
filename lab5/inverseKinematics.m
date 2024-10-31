function etaBar_motor = inverseKinematics(nu,omega,params)

    etaBar_motor = [ (nu + omega*params.L)/params.R;
        (nu - omega*params.L)/params.R]*params.N;
end

