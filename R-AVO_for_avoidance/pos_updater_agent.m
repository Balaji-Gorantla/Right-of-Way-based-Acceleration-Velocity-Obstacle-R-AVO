function [Xagent,Yagent] = pos_updater_agent(delta,agent, desVagent_x, desVagent_y,t_iter)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% Xagent=Xagent+t_iter*Vagent_x+delta*(exp(-t_iter/delta)-1)*(Vagent_x-initVagent_x);
% Yagent=Yagent+t_iter*Vagent_y+delta*(exp(-t_iter/delta)-1)*(Vagent_y-initVagent_y);

Xagent=agent.Position(1) + t_iter*desVagent_x + delta*(exp(-t_iter/delta)-1)*(desVagent_x-agent.NewVelocity(1));
Yagent=agent.Position(2) + t_iter*desVagent_y + delta*(exp(-t_iter/delta)-1)*(desVagent_y-agent.NewVelocity(2));

end

