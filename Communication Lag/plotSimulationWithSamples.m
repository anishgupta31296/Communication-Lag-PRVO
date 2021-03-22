function plotSimulationWithSamples(agents, counter, dt, axisLimits, save)
% plotSimulation - Plots the positions of the agents and their paths
%
% Syntax: plotSimulation(agents, counter, dt, axisLimits, save)
%
    clf('reset')
    hold on

    
    for i = 1:length(agents)
        faceColor = [0 170 255] / 255;
        lineColor = [135 135 135] / 255;
        posn=agents(i).pos_samples;    
        if(posn)
         for k=1:50
           circ(posn(k,1),posn(k,2),agents(i).radius,faceColor);
         end
         filledCircle(agents(i).communicated_position, agents(i).radius, 1000, [0 1 0.4]);

        end
%         if(posn)
%          filledCircle(agents(i).position, agents(i).radius, 1000, [1 0.2 0.2]);
%          filledCircle( 2*[mean(posn(:,1)),mean(posn(:,2))]-agents(i).position,agents(i).radius, 1000,[0.1 0.1 1]);
%          filledCircle([mean(posn(:,1)),mean(posn(:,2))],agents(i).radius, 1000,faceColor);
% 
%         else
        filledCircle(agents(i).position, agents(i).radius, 1000, faceColor);
         
%     
%         end
        quiver(agents(i).position(1), agents(i).position(2), agents(i).velocity(1), agents(i).velocity(2), 'Color', [0 0 0]);
        plot(agents(i).path(:, 1), agents(i).path(:, 2), 'Color', lineColor);
        text(agents(i).position(1), agents(i).position(2), agents(i).name);
    end

    title([ 'Time: ' num2str(counter*dt) 's' ])
    set(get(gca, 'XLabel'), 'String', 'X [m]');
    set(get(gca, 'YLabel'), 'String', 'Y [m]');
    axis(axisLimits)
    axis equal
    hold off
    drawnow
 
    if save
        saveas(gcf, ['run/', num2str(counter,'%04.f'), '.png']);
    end
end