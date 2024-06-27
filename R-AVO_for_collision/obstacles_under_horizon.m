function [ tag, d_critical, dmin] = obstacles_under_horizon(agent, others, tH, n)

%INPUTS:
%agent and all the other agents(considering them as obstacles)
%tH: time horizon
%n: total number of agents given by user

%OUTPUTS:
%calculating d_critical: distance of the farthermost obstacle lying inside the time horizon to set the grid size 
%tag will provide the identities of obstacles that are under the time horizon tH
%dmin will give the minimum distance among all the obstacles

tag=[];

i = 1;
count=1;

tH_obstacle = [];
max_th = 0;
v_critical = 0;
d_critical = 10;
dmin=100000000;
R_s = 12;
% jCount = 1;
for i = 1:n
    if (agent.Identity ~= others(i).Identity)

%           r = agent.NeighbourDistance(1,jCount);  
        r = sqrt((others(i).Position(2) - agent.Position(2))^2 + (others(i).Position(1) - agent.Position(1))^2);
        if (dmin>r)
            dmin=r;
        end

        v = sqrt((others(i).NewVelocity(1) - agent.NewVelocity(1))^2 + (others(i).NewVelocity(2) - agent.NewVelocity(2))^2); %relative speed between agent and other(obstacle)

        tH_obstacle =(r/v); %time to collision

%         if ((tH_obstacle < tH))
        if ((r <= R_s))

            tag(count) = others(i).Identity;
            count =count+1;

            if tH_obstacle>max_th
                max_th=tH_obstacle; % its the time to reach the farthermost obstacle lying inside the time horizon
                d_critical=r; % its the distance of the farthermost obstacle lying inside the time horizon
            end

        end
%         jCount = jCount+1;
    end
     
end

end
