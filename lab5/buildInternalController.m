function K = buildInternalController(model)
% Internal controller design with pole placement

% Desired poles
desPoles = -[24.9,25.0,25.1,25.2];

% Computes gain
K = place(model.Ace,model.Bce,desPoles);

end

