%% 
% input required
% 1. robo vel x and y
% 2. robo pos x and y amd its corner coordinate points in terms of x and y
% 3. the obstacles velocity and its x and y coordinates of its corner
% points

function [valid] = avo(vel_opt_x,vel_opt_y,tag,agent,others,delta,plan_cycle,t_num,R_d)

% no of dynamic obstacles in the time horizon  
obstacle_matrix = [];
V_obs_hor = [];
Total_within = 0;
t=plan_cycle(1,t_num);
for iObs = 1:size(tag,2)
%     obstacle_matrix = [obstacle_matrix; others(tag(1,iObs)).Position(1) others(tag(1,iObs)).Position(2)];
%     V_obs_hor = [V_obs_hor; others(tag(1,iObs)).NewVelocity(1) others(tag(1,iObs)).NewVelocity(2)];  
    
    Vax = agent.NewVelocity(1) + agent.Position(1);
    Vay = agent.NewVelocity(2) + agent.Position(2);
    
    pbx = others(tag(1,iObs)).Position(1);
    pby = others(tag(1,iObs)).Position(2);
    vbx = others(tag(1,iObs)).NewVelocity(1) + pbx;
    vby = others(tag(1,iObs)).NewVelocity(2) + pby;
    ra = agent.Radius+R_d;
    rb = others(tag(1,iObs)).Radius+R_d;
    obs_vel = [vbx,vby];
    
    disk_center_x = (delta*(exp(-t/delta)-1)*(Vax - vbx)-((agent.Position(1)- pbx)))/((t+delta*(exp(-t/delta)-1)));
    disk_center_y = (delta*(exp(-t/delta)-1)*(Vay - vby)-((agent.Position(2)- pby)))/((t+delta*(exp(-t/delta)-1)));
    disk_radius = (ra+rb)/((t+delta*(exp(-t/delta)-1)));

    plan_vel = [vel_opt_x,vel_opt_y];
    disk_center = [disk_center_x, disk_center_y];
    check_dist = norm((plan_vel+agent.Position) - (disk_center+obs_vel));

%     disp(strcat('current planning velocity is-->', string(plan_vel(1)), ':', string(plan_vel(2))));
    
%     if agent.Identity == 2
%         viscircles(disk_center+obs_vel, disk_radius, 'EdgeColor', agent.Color, 'LineWidth', 2);
%         hold on;
% %         plot(vel_opt_x,vel_opt_y,'r*');
%     end
    
    if check_dist < disk_radius
        within = 0;
    else
        within = 1;
    end
    
    Total_within = Total_within + within;

%     disp(strcat('check distance is--> ', int2str(check_dist), ', within is -->', int2str(within)));
end

% disp(strcat('total within is ', int2str(Total_within)));
if Total_within == size(tag,2)
   valid = 1; 
else
   valid = 0;
end



end
