function controls = getControlsRVO(agent, obstacles, dt)
%getControls - Returns the controls that will avoid collision
%
% Syntax: controls = getControls(agent, obstacles, dt)
%
    agent.radius=agent.radius;
    for i=1:length(obstacles)
        obstacles(i).radius=obstacles(i).radius;
    end
    cost = @(u) desiredVelocityCost(agent, u) ;
    constraints = [];
    if length(obstacles) > 0
        constraints = @(u) getConstraintsRVO(agent, obstacles, u, dt);
        constraints(agent.velocity)
    end
    init = agent.velocity ;
    lb = [-agent.vmax -agent.vmax];
    ub = [agent.vmax agent.vmax];
    options = optimoptions('fmincon','Display','final-detailed','Algorithm','active-set');
    controls = fmincon(cost, init, [], [], [], [], lb, ub, constraints, options);
end