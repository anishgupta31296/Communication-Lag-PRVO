function [controls ] = getControlsThroughSamples(agent, obstacles, dt)

    cost = @(u) desiredVelocityCost(agent, u) + 0.5*norm(u);
    constraints = [];

    if length(obstacles) > 0
        constraints = @(u) getConstraintThroughSamples(agent, obstacles, u, dt);
        init = agent.velocity ;
        init=getControlsRVO(agent, obstacles, dt);
        lb = [-agent.vmax -agent.vmax];
        ub = [agent.vmax agent.vmax];
        options = optimoptions('fmincon','Algorithm','sqp');
        controls = fmincon(cost, init, [], [], [], [], lb, ub, constraints, options);

    else
        init = agent.velocity ;
        lb = [-agent.vmax -agent.vmax];
        ub = [agent.vmax agent.vmax];
        options = optimoptions('fmincon','Display','final-detailed','Algorithm','sqp');
        controls = fmincon(cost, init, [], [], [], [], lb, ub, constraints, options);
    end
    controls = smoothenControls(agent, controls);
    control_cost=norm(agent.velocity-controls);

end