function [des_vel_x, des_vel_y] = safe_velocity(tag,agent,others,delta,t_iter,t_H,R_d)

plan_cycle = linspace(0,t_H,50);
time_interval = size(plan_cycle,2);

ad_vel = Admis_Velocity(agent,t_iter); % first two coloumns(Vx,Vy) are all the velocities right side to the current prefer velocity and third coloumn is a distance between NewVelocity and all the admissible velocities
vel_optx = ad_vel(:,1)';
vel_opty = ad_vel(:,2)';

ad_vel_count = size(ad_vel,1);% no. of possible velocities of the robot 
        
for iIndex = 1:ad_vel_count
    safe1 = 0;
    vel_opt_x = vel_optx(1,iIndex);
    vel_opt_y = vel_opty(1,iIndex);

    for t_num = 2:time_interval
        safe = avo(vel_opt_x,vel_opt_y,tag,agent,others,delta,plan_cycle,t_num,R_d);
        safe1 = safe1 + safe;
    end

    if safe1 == (time_interval-1)
        des_vel_x = vel_opt_x;
        des_vel_y = vel_opt_y;
        break;
    else 
        des_vel_x = 0;
        des_vel_y = 0;
    end
   
end

end