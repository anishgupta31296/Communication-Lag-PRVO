function controls = getControls(agent, obstacles, dt)
%getControls - Returns the controls that will avoid collision
%
% Syntax: controls = getControls(agent, obstacles, dt)
%

    cost = @(u) desiredVelocityCost(agent, u) + 0.5*sum((agent.velocity - u).^2);
    constraints = [];
    if length(obstacles) > 0
        constraints = @(u) getConstraints(agent, obstacles, u, dt);
    end

    init = agent.velocity ;
    lb = [-agent.vmax -agent.vmax];
    ub = [agent.vmax agent.vmax];
    options = optimoptions('fmincon','Display','final-detailed');
    controls = fmincon(cost, init, [], [], [], [], lb, ub, constraints, options);
    controls=0.5*(controls+agent.velocity);
    % Smoothen the controls
%     controls = smoothenControls(agent, controls);
end