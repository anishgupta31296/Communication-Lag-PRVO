%% Clear workspace
clc;
clear all;
close all;
flag=0;
faceColor = [0 170 255] / 255;
min_lag=2;
max_lag=3;

goal_p = [7,-7];
agent_p = [-15,7];
obs_p = [-4,4;-2,2;0,1;-4,-4;-2,-2;];
obs_v = [-1,-0.6;-1,-0.6;-1,0;-0.8,0.8;-0.8,0.8;];
% 
% agent_p = [-15,7];
% obs_p = [-4,4;-2,2;0,0;2,-2;-4,-4;-2,-2;6,-6;8,-8];
% obs_v = [-1,0.4;-1,0.4;-1,0.4;-1,0.4;-1,0.4;-1,0.4;-1,0.4;-1,0.4];
% goal_p = [7,-7];
% % % 
% % 
%  agent_p = [-7,7];
%  obs_p = [5,-5;-7,-2;4,6;];
%  obs_v = [-0.6,0.8;1.0,0.9;-0.5,-0.5;];
%  goal_p = [7,-7];
% % %  
% %  
%  agent_p = [5,-5];
% obs_p = [17,-4;22,-4;19,-21;21,-19; 22.5, -17.5; 17.5, -22.5];
% obs_v = [-0.7,-0.7;-0.7,-0.7;-0.7,0.7;-0.7,0.7;-0.7,0.7;-0.7,0.7];
% goal_p = [20,-20];
% %  
%% Declare the agents
%Usage: addAgent(name, initialPosition, initialVelocity, goal)
agents = [
    addAgent('1', agent_p, [0 0], goal_p),
    addAgent('2', obs_p(1,:),   [0 0], obs_p(1,:)+20*obs_v(1,:)),
    addAgent('3',  obs_p(2,:),  [0 0], obs_p(2,:)+25*obs_v(2,:)),
    addAgent('4',  obs_p(3,:),  [0 0], obs_p(3,:)+13*obs_v(3,:))
    addAgent('5',  obs_p(4,:), [0 0], obs_p(4,:)+14*obs_v(4,:)),
    addAgent('6',  obs_p(5,:), [0 0], obs_p(5,:)+15*obs_v(5,:)),
%     addAgent('7',  obs_p(6,:), [0 0],  obs_p(6,:)+12*obs_v(6,:)),
];

x_min=min([obs_p(:,1); agent_p(1); goal_p(1)]);
x_max=max([obs_p(:,1); agent_p(1); goal_p(1)]);
y_min=min([obs_p(:,2); agent_p(2); goal_p(2)]);
y_max=max([obs_p(:,2); agent_p(2); goal_p(2)]);
range=max([(x_max-x_min) (y_max-y_min)]);
x_axis_min=(x_min + x_max)/2  - range/2 -2;
x_axis_max=(x_min + x_max)/2  + range/2 +2;
y_axis_min=(y_min + y_max)/2  - range/2 -2;
y_axis_max=(y_min + y_max)/2  + range/2 +2;
axisLimits = [x_min x_max y_min y_max]; % [xmin xmax ymin ymax] axis limits of the plot

agents = [
    addAgent('1', [-5 -5], [0 0], [5 5]),
    addAgent('2', [5 5],   [0 0], [-5 -5]),
%     addAgent('3', [-5 5],  [0 0], [5 -5]),
%     addAgent('4', [5 -5],  [0 0], [-5 5]),
%     addAgent('5', [-7.1,0], [0 0], [7.1 0]),
%     addAgent('6', [7.1,0], [0 0], [-7.1,0]),
%     addAgent('7', [0,7.1], [0 0],  [0,-7.1]),
%     addAgent('8', [0,-7.1], [0 0],  [0,7.1])
];

axisLimits = [-8 8 -8 8]; % [xmin xmax ymin ymax] axis limits of the plot
dt = 0.1;
no_agents=length(agents);
control_cost_list=zeros(length(agents),1);
path=norm(agents(1).position- agents(1).goal);
%% Simulation loop
% Runs till the distance to goal of all the agents is less than 0.32m
% Or for max iterations
maxIterations = 500;
counter = 0;
prev=randi(10,length(agents),length(agents))+2;
for i = 1:length(agents)
    agents(i).communication_lag=min_lag;
end
detected_count=zeros(length(agents),1);
while counter < maxIterations
    maxDistFromGoal = 0;
    flag=0;
    pos_array={};
    
    % Get the new velocity command for every agent but do not update it now
    if(counter>6)
     for i = 1:length(agents)
        agents(i).communication_lag=randi([min_lag,min(agents(i).communication_lag+1,max_lag)]);
        agents(i).communicated_position=agents(i).path(end+1-agents(i).communication_lag,:);
        agents(i).communicated_velocity=agents(i).vel_path(end+1-agents(i).communication_lag,:);
        agents(i).pos_samples=[];
     end
    end
    for i = 1:length(agents)
        % Get agents in the sensor range
        obstacles = [];
        flag=0;
        for j = 1:length(agents)
            if i ~= j
                if inSensorRange(agents(i), agents(j))
                    detected_count(j)=detected_count(j)+1;
                    obs=agents(j);
                    obs.position=obs.communicated_position;
                    obs.velocity=obs.communicated_velocity;
                    newControl = getControlsRVO(obs, agents(i), dt);
%                     obs.velocity=newControl;
                    agents(j).pos_samples=[pearsrnd(obs.position(1)+(min_lag+0.5*(-min_lag+max_lag))*obs.velocity(1)*dt,abs(obs.velocity(1)*0.4*(min_lag+max_lag)*dt),0,3,1000,1),pearsrnd(obs.position(2)+(min_lag+0.5*(-min_lag+max_lag))*obs.velocity(2)*dt,abs(obs.velocity(2)*0.4*(min_lag+max_lag)*dt),0,3,1000,1)];
                    obs.pos_samples=agents(j).pos_samples;
                    agents(j).vel_samples=[pearsrnd(obs.velocity(1),0.1,0,3,1000,1),pearsrnd(obs.velocity(2),0.1,0,3,1000,1)];
                    obs.vel_samples=agents(j).vel_samples;
                    obstacles = [obstacles; obs];
                    flag=1;
                end                
            end
        end
        % Get the new control for the agent
        [agents(i).newControl] = getControlsThroughSamples(agents(i), obstacles, dt);
        cost = @(u) desiredVelocityCost(agents(i), u);
        control_cost_list(i,end+1)=cost(agents(i).newControl);
end
%     if(flag==1)
%         pause
%     end

    % Update the positions of all the agents using the newly obtained controls
    % This is equivalent to running the same algorithm simultaneously on all agents
    for i = 1:length(agents)
        agents(i).path = [agents(i).path; agents(i).position];
        agents(i).vel_path = [agents(i).vel_path; agents(i).velocity];
        agents(i).position = futurePosition(agents(i), dt);
        agents(i).velocity = agents(i).newControl;
        maxDistFromGoal = max(maxDistFromGoal, sum((agents(i).position - agents(i).goal).^2));
    end

    % Plot the current simulation step
    plotSimulationWithSamples(agents, counter, dt, axisLimits, true);
 %   plotSimulationWithoutSamples(agents, counter, dt, axisLimits, false);
    counter = counter + 1;
    % Stop running of the goal is reached
    if maxDistFromGoal < 0.1
        break
    end

end
for j=1:length(agents)
path_length=0;
for i=1:length(agents(1).path)-1
    path_length=path_length+norm(agents( j).path(i+1,:)-agents( j).path(i,:));
end
    deviation(j)=path_length-norm(agents( j).path(end,:)-agents( j).path(1,:));
end
total_control_cost=sum(control_cost_list,2);
mean(deviation)
mean(total_control_cost)
max(deviation)
