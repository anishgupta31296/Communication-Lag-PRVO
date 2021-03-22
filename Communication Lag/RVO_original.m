%% Clear workspace
clc;
clear all;
close all;
flag=0;
faceColor = [0 170 255] / 255;


agent_p = [-15,7];
goal_p = [7,-7];
obs_p = [-4,4;-2,2;0,0;-4,-4;-2,-2;];
obs_v = [-1,-0.6;-1,-0.6;-1,0;-0.8,0.8;-0.8,0.8;];
% 
% agent_p = [-15,7];
% obs_p = [-4,4;-2,2;0,0;2,-2;-4,-4;-2,-2;6,-6;8,-8];
% obs_v = [-1,0.4;-1,0.4;-1,0.4;-1,0.4;-1,0.4;-1,0.4;-1,0.4;-1,0.4];
% goal_p = [7,-7];

 agent_p = [-7,7];
 obs_p = [5,-5;-7,-2;4,6;];
 obs_v = [-0.6,0.8;1.0,0.9;-0.5,-0.5;];
 goal_p = [10,-7];
  
%  agent_p = [5,-5];
% obs_p = [17,-4;22,-4;19,-21;21,-19; 22.5, -17.5; 17.5, -22.5];
% obs_v = [-0.7,-0.7;-0.7,-0.7;-0.7,0.7;-0.7,0.7;-0.7,0.7;-0.7,0.7];
% goal_p = [20,-20];
%% Declare the agents
% Usage: addAgent(name, initialPosition, initialVelocity, goal)
agents = [
    addAgent('1', agent_p, [0 0], goal_p),
    addAgent('2', obs_p(1,:),   [0 0], obs_p(1,:)+15*obs_v(1,:)),
    addAgent('3',  obs_p(2,:),  [0 0], obs_p(2,:)+14*obs_v(2,:)),
    addAgent('4',  obs_p(3,:),  [0 0], obs_p(3,:)+15*obs_v(3,:))
%     addAgent('5',  obs_p(4,:), [0 0], obs_p(4,:)+15*obs_v(4,:)),
%     addAgent('6',  obs_p(5,:), [0 0], obs_p(5,:)+15*obs_v(5,:)),
%     addAgent('7',  obs_p(6,:), [0 0],  obs_p(6,:)+15*obs_v(6,:)),
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
% 
agents = [
    addAgent('1', [-5 -5], [0 0], [5 5]),
    addAgent('2', [5 5],   [0 0], [-5 -6]),
    addAgent('3', [-5 5],  [0 0], [5 -4.9]),
    addAgent('4', [5 -5],  [0 0], [-4.9 5]),
%     addAgent('5', [-7.1,0], [0 0], [7.1 0]),
%     addAgent('6', [7.1,0], [0 0], [-7.1,0]),
%     addAgent('7', [0,7.1], [0 0],  [0,-7.1]),
%     addAgent('8', [0,-7.1], [0 0],  [0,7.1])
];
axisLimits = [-8 8 -8 8]; % [xmin xmax ymin ymax] axis limits of the plot

dt = 0.1;
no_agents=length(agents);

%% Simulation loop
% Runs till the distance to goal of all the agents is less than 0.32m
% Or for max iterations
maxIterations = 500;
counter = 0;


while counter < maxIterations
    maxDistFromGoal = 0;
    flag=0;
    pos_array={};
    % Get the new velocity command for every agent but do not update it now

    for i = 1:length(agents)
        % Get agents in the sensor range
        obstacles = [];
        for j = 1:length(agents)
            if i ~= j
                if inSensorRange(agents(i), agents(j))
                    flag=1;
                    obs=agents(j);
                    obstacles = [obstacles; obs];
                end                
            end
        end
        % Get the new control for the agent
        agents(i).newControl = getControlsRVO(agents(i), obstacles, dt);
    end
    
%     if(flag==1)
%         [a b c d]=tangent1(agents(1).position , agents(2).position,agents(1).radius, agents(2).radius);
%         hold on
%         plot([a(1) b(1)], [a(2) b(2)])
%         hold on
%         plot([c(1) d(1)], [c(2) d(2)])
%         quiver(agents(1).position(1), agents(1).position(2), agents(1).velocity(1)-agents(2).velocity(1), agents(1).velocity(2)-agents(2).velocity(2), 'Color', [0 0 0])
%         dot(agents(1).velocity,agents(2).velocity)
%         pause
%     end
%    
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
    % Usage: plotSimulation(agents, counter, dt, axisLimits, true) -> Save the outputs to disk
    %        plotSimulation(agents, counter, dt, axisLimits, false) -> Don't save the outputs to disk
    plotSimulation(agents, counter, dt, axisLimits, false,pos_array);
    counter = counter + 1;
    % Stop running of the goal is reached
    if maxDistFromGoal < 0.1
        break
    end
    agents(1).velocity

end