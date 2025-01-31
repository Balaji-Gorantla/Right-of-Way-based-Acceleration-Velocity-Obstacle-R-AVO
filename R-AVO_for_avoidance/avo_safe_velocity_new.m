function [des_vel_x, des_vel_y] = avo_safe_velocity_new(tag,agent,others,delta,t_iter,t_H,R_d)

global is_test_req;
plan_cycle = linspace(1,t_H,50);
time_interval = size(plan_cycle,2);

ad_vel = Admis_Velocity(agent,t_iter); 

for a=1:length(tag)
    if(tag(a) ~= agent.Identity)
        % compute the cone of collisions                        
        [AVO_cone_new] = avo_cone_new(tag(a),agent,others,delta,plan_cycle,R_d);

        if is_test_req 
            if (agent.Identity == 1)
                
                hold on;
                plot(AVO_cone_new(1,:)+agent.Position(1),AVO_cone_new(2,:)+agent.Position(2),'k-.');
                                                    
                hold off;
%             else
%                 hold on;
%                 plot(AVO_cone_new(1,:)+agent.Position(1),AVO_cone_new(2,:)+agent.Position(2),'g');
%                 %                                     plot(tag(i).NewVelocity(1),tag(i).NewVelocity(2),'g*');
%     %             hold off;  
            end
        end

        in=inpolygon(ad_vel(:,1)+agent.Position(1),ad_vel(:,2)+agent.Position(2),AVO_cone_new(1,:)+agent.Position(1),AVO_cone_new(2,:)+agent.Position(2));
        ad_vel=[ad_vel,in];
        
        for q = 1:length(ad_vel(:,end))
            if ad_vel(q,3) == 1
%                 disp(strcat('velocity '," ", int2str(ad_vel(q,1)),':', int2str(ad_vel(q,2))," ", 'is in'));
                ad_vel(q,:)=[0,0,0];
            end
        end
        
        ad_vel( all(~ad_vel,2), : ) = [];

        ad_vel=ad_vel(:,1:3);
    end
end

if numel(ad_vel) == 0
    des_vel_x = 0;
    des_vel_y = 0;
else
    des_vel_x = ad_vel(1,1);
    des_vel_y = ad_vel(1,2);
end
    
end