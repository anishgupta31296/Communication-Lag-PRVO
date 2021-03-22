function agent = addAgent(name, position, velocity, goal)
% addAgent - Return a struct with agent details
%
% Syntax: agent = addAgent(name, position, velocity, goal)
%
    agent = struct( 'name', name, 'position', position, 'velocity', velocity );
    agent.goal = goal;
    agent.newControl=velocity;
    agent.path = position;
    agent.pos_samples=[];
    agent.vel_samples=[];    
    agent.vel_path = velocity;
    agent.communicated_position=position;
    agent.communicated_velocity=velocity;
    agent.communication_lag=2;
    agent.radius = 0.5;
    agent.sensorRange = 5;
    agent.vmax = 1.5;
    agent.radiusbb=agent.radius + agent.vmax*0.1*agent.communication_lag*1.5;

end