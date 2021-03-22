function [c ceq] = getConstraintsRVO(agent, obstacles, control, dt)
    c = [];
    ceq = [];

    for i = 1:length(obstacles)
        vRel = -agent.velocity+2*control - obstacles(i).velocity;
        pAb = (agent.position - obstacles(i).position) ;
        c(end+1)=sum(vRel.^2)*((agent.radius+obstacles(i).radius)^2-sum(pAb.^2))+(pAb*vRel')^2;
    end
    
end