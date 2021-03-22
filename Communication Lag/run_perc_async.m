%% Clear workspace
clc;
clear all;
close all;
flag=0;
%% Declare the agents
% Usage: addAgent(name, initialPosition, initialVelocity, goal)
agents = [
    addAgent('1', [-5 -5], [0 0], [5 5]),
    addAgent('2', [5 5],   [0 0], [-5 -5]),
    addAgent('3', [-5 5],  [0 0], [5 -5]),
    addAgent('4', [5 -5],  [0 0], [-5 5])
%     addAgent('5', [-7.1,0], [0 0], [7.1 0]),
%     addAgent('6', [7.1,0], [0 0], [-7.1,0]),
%     addAgent('7', [0,7.1], [0 0],  [0,-7.1]),
%     addAgent('8', [0,-7.1], [0 0],  [0,7.1])
];
axisLimits = [-8 8 -8 8]; % [xmin xmax ymin ymax] axis limits of the plot
dt = 0.1;
no_agents=length(agents);
obstacles_list={[],[],[],[]};
%% Simulation loop
% Runs till the distance to goal of all the agents is less than 0.32m
% Or for max iterations
maxIterations = 500;
counter = 0;
while counter < maxIterations
    maxDistFromGoal = 0;
    flag=0;
    ag_list=randperm(no_agents);
    t_list=diff(sort(rand(1,no_agents+1)));
    t_list=t_list/sum(t_list);
    % Get the new velocity command for every agent but do not update it now
    freq_list=1+round(4*rand(4,1));
    for i = 1:length(agents)
        i=ag_list(i);
        
        % Get agents in the sensor range
        obstacles = [];
        for j = 1:length(agents)
            j=ag_list(j);
            if i ~= j  
                if inSensorRange(agents(i), agents(j))
                    obstacles = [obstacles; agents(j)];
                    flag=1;                   
                end
                agents(j).path = [agents(j).path; agents(j).position];
                agents(j).position = futurePosition(agents(j), dt*t_list(i));
                
                
            end
        end
        if(rem(counter,freq_list(i))==0)
         obstacles_list{i}=obstacles;
        end
        % Get the new control for the agent
        agents(i).newControl = getControls(agents(i), obstacles_list{i}, dt);
        agents(i).path = [agents(i).path; agents(i).position];
        agents(i).position = futurePosition(agents(i), dt*t_list(i));
        agents(i).velocity = agents(i).newControl;
        plotSimulation(agents, counter, dt, axisLimits, true);
        counter = counter + 1;
        

    end

    % Update the positions of all the agents using the newly obtained controls
    % This is equivalent to running the same algorithm simultaneously on all agents
    for i = 1:length(agents)
%         agents(i).path = [agents(i).path; agents(i).position];
%         agents(i).position = futurePosition(agents(i), dt);
%         agents(i).velocity = agents(i).newControl;
        maxDistFromGoal = max(maxDistFromGoal, sum((agents(i).position - agents(i).goal).^2));
    end

    % Plot the current simulation step
    % Usage: plotSimulation(agents, counter, dt, axisLimits, true) -> Save the outputs to disk
    %        plotSimulation(agents, counter, dt, axisLimits, false) -> Don't save the outputs to disk
%     plotSimulation(agents, counter, dt, axisLimits, true);
%     if(flag)
%         pause
%     end
    % Stop running of the goal is reached
    if maxDistFromGoal < 0.1
        break
    end
end