function Display(a,b,Identity,agent,ROW1)

if (a == 1 || a == 2 || a == 3 || a == 4) && b == 1
    disp(strcat('agent- ', int2str(Identity), ' is infront of agent- '," ", int2str(agent.Identity), ' and agent- ', int2str(agent.Identity), ' does not have RoW'));
elseif (a == 1 || a == 2 || a == 3 || a == 4) && b == 2
    disp(strcat('agent- ', int2str(Identity), ' is at the left-side of agent-'," ", int2str(agent.Identity), ' and agent- ', int2str(agent.Identity), ' has RoW'));
elseif (a == 1 || a == 2 || a == 3 || a == 4) && b == 3
    disp(strcat('agent-', int2str(Identity), ' is at the rear side of agent-'," ", int2str(agent.Identity), ' and agent- ', int2str(agent.Identity), ' has RoW'));
elseif a == 1 && b == 4 && ROW1 == 0
    disp(strcat('agent- ', int2str(Identity), ' is at the right-side of agent-'," ", int2str(agent.Identity), ' and agent- ', int2str(agent.Identity), ' does not have RoW'));
elseif a == 1 && b == 4 && ROW1 == 1
    disp(strcat('agent- ', int2str(Identity), ' is at the right-side of agent-'," ", int2str(agent.Identity), ' and agent- ', int2str(agent.Identity), ' has RoW'));
elseif a == 2 && b == 4
   disp(strcat('agent- ', int2str(Identity), ' is at the right-side of agent-'," ", int2str(agent.Identity), ' and agent- ', int2str(agent.Identity), ' does not have RoW'));
elseif (a == 3 || a == 4) && b == 4
    disp(strcat('agent- ', int2str(Identity), ' is at the right-side of agent-'," ", int2str(agent.Identity), ' and agent- ', int2str(agent.Identity), ' has RoW')); 

end