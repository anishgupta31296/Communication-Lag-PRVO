function [c,ceq] = getConstraintThroughSamples(agent, obstacles, control, dt)

    c = [];
    ceq = [];
    tau = 0.8;
    time=0;
    k=1.3;
%    Time horizon
%     tau = norm(pAb)/norm(agent.velocity+0.1);

    for i = 1:length(obstacles)
        % Refer the paper for explanation on these terms
        vRel = 2*control-agent.velocity- obstacles(i).vel_samples;
        
        pAb = (agent.position - obstacles(i).pos_samples) ;
        cone_list = ((vRel(:,1).^2+vRel(:,2).^2)*(agent.radius+obstacles(i).radius)^2+(sum(pAb.*vRel,2)).^2-(vRel(:,1).^2+vRel(:,2).^2).*(pAb(:,1).^2 + pAb(:,2).^2));
        c(end+1)=mean(cone_list)+k*std(cone_list);        
%         vRel = agent.velocity - obstacles(i).velocity;
%         vAb = agent.velocity + obstacles(i).velocity;
%         pAb = (agent.position - mean(obstacles(i).pos_samples)) / tau;
% 
%         % Finding pAb perpendicular
%         r = 2 / tau;
%         l = abs(sqrt(sum(pAb.^2) - r^2));
%         m = [
%             l -r;
%             r  l
%         ];
%         qL = (pAb * m') * (l / sum(pAb.^2));
%         qR = (pAb * m ) * (l / sum(pAb.^2));
%         pAbL = [qL(2) -qL(1)];
%         pAbR = [qR(2) -qR(1)];
%         if det([pAb; vRel]) < 0
%             c(end+1) = -(control(1)*(pAbR(1))^2*(agent.velocity(1)) + control(2)*(pAbR(1))*(pAbR(2))*(agent.velocity(1)) - 0.5*(pAbR(1))^2*(agent.velocity(1))^2 + control(1)*(pAbR(1))*(pAbR(2))*(agent.velocity(2)) + control(2)*(pAbR(2))^2*(agent.velocity(2)) - 1.*(pAbR(1))*(pAbR(2))*(agent.velocity(1))*(agent.velocity(2)) - 0.5*(pAbR(2))^2*(agent.velocity(2))^2 - control(1)*(pAbR(1))^2*(obstacles(i).velocity(1)) - control(2)*(pAbR(1))*(pAbR(2))*(obstacles(i).velocity(1)) + 0.5*(pAbR(1))^2*(obstacles(i).velocity(1))^2 - control(1)*(pAbR(1))*(pAbR(2))*(obstacles(i).velocity(2)) - control(2)*(pAbR(2))^2*(obstacles(i).velocity(2)) + 1.*(pAbR(1))*(pAbR(2))*(obstacles(i).velocity(1))*(obstacles(i).velocity(2)) + 0.5*(pAbR(2))^2*(obstacles(i).velocity(2))^2);
%         else
%             c(end+1) = -(control(1)*(pAbL(1))^2*(agent.velocity(1)) + control(2)*(pAbL(1))*(pAbL(2))*(agent.velocity(1)) - 0.5*(pAbL(1))^2*(agent.velocity(1))^2 + control(1)*(pAbL(1))*(pAbL(2))*(agent.velocity(2)) + control(2)*(pAbL(2))^2*(agent.velocity(2)) - 1.*(pAbL(1))*(pAbL(2))*(agent.velocity(1))*(agent.velocity(2)) - 0.5*(pAbL(2))^2*(agent.velocity(2))^2 - control(1)*(pAbL(1))^2*(obstacles(i).velocity(1)) - control(2)*(pAbL(1))*(pAbL(2))*(obstacles(i).velocity(1)) + 0.5*(pAbL(1))^2*(obstacles(i).velocity(1))^2 - control(1)*(pAbL(1))*(pAbL(2))*(obstacles(i).velocity(2)) - control(2)*(pAbL(2))^2*(obstacles(i).velocity(2)) + 1.*(pAbL(1))*(pAbL(2))*(obstacles(i).velocity(1))*(obstacles(i).velocity(2)) + 0.5*(pAbL(2))^2*(obstacles(i).velocity(2))^2);
%         end
    end

end