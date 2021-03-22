function controls = getControlsBB(agent, obstacles, dt)
%getControls - Returns the controls that will avoid collision
%
% Syntax: controls = getControls(agent, obstacles, dt)
%
    agent.radius=agent.radiusbb;
    for i=1:length(obstacles)
        obstacles(i).radius=obstacles(i).radiusbb;
    end
    cost = @(u) desiredVelocityCost(agent, u) + 1*sum((agent.velocity - u).^2);
    constraints = [];
    if length(obstacles) > 0
        constraints = @(u) getConstraints(agent, obstacles, u, dt);
    end

    init = agent.velocity ;
    lb = [-agent.vmax -agent.vmax];
    ub = [agent.vmax agent.vmax];
    options = optimoptions('fmincon','Algorithm','sqp','Display','final-detailed');
    controls = fmincon(cost, init, [], [], [], [], lb, ub, constraints, options);
    controls=0.5*(controls+agent.velocity);
    % Smoothen the controls
    controls = smoothenControls(agent, controls);
end