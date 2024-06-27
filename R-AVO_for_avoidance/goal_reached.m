function [bool, avoidance,t_comp,d_min, a, a_lo, a_la] = goal_reached(agents,t_iter,tol_d,n,t_run,R)

distance(agents);
bool = 0;
t_comp = [];
a = [];
a_lo = [];
a_la = [];
% avoidance = zeros(1,n);
for i = 1:length(agents)
   
    dist = [agents(i).Position; agents(i).Target]; %distance between agent and thier goal points
    t_compute = tic;
    if pdist(dist) >= tol_d
        [avoiding, a_net, dmin, a_long, a_lat] = agents(i).FindVelocity(agents,t_iter,n,t_run,R); %checks for collisions nd provides new velocity
        bool = 1; %agent did not reach the goal yet
%         continue
    else
%        agents(i).NewVelocity = [0,0]; %agent reached goal and speed equal zero
%        agents(i).NewVelocity = [0,0];
       desVagent_x = 0;
       desVagent_y = 0;
       delta = 5;
       agents(i).NewVelocity(1) = desVagent_x - exp(-t_iter/delta)*(desVagent_x-agents(i).NewVelocity(1)); %updating the new velocity of the agent
       agents(i).NewVelocity(2) = desVagent_y - exp(-t_iter/delta)*(desVagent_y-agents(i).NewVelocity(2));
       if pdist(dist) <= 0.2*tol_d
           agents(i).NewVelocity = [0,0];
       end
       avoiding = 0;
       dmin = inf;
       a_net = 0;
       a_long = 0;
       a_lat = 0;
    end
    
    if avoiding == 1
        avoidance(i) = 1;
    else
        avoidance(i) = 0;
    end
    
    t_comp(1,i) = toc(t_compute);
    if toc(t_compute) < 0.00009
        t_comp(1,i) = 0;
    end
    
    d_min(1,i) = dmin;
    a(1,i) = a_net;
    a_lo(1,i) = a_long;
    a_la(1,i) = a_lat;
end

%updating the positions of agents based on the velocity they have attained 
for j = 1:length(agents)
    
   agents(j).Position = agents(j).NewPosition;
    
end

end