classdef Agent < handle
    
   properties
       
       Identity
       Position
       Target
       Velocity
       PrefVelocity
       Radius
       NewPosition
       NewVelocity
       NeighbourDistance
       NeighbourAngle
       Color
       OldVelocity
       Acceleration

   end
   
   
   methods (Access = public)
       
       function obj = Agent(id, pos, goal, s, p_s, r, n_p, n_p_s, n_d, n_a, c, o_v, ac, n) %It is a constructor, creates objects and assigns value to the properties and returns initialized object
           if nargin>0
               for i = 1:n
                   
                  obj(i).Identity = id;
                  obj(i).Position = pos;
                  obj(i).Target = goal;
                  obj(i).Velocity = s;
                  obj(i).PrefVelocity = p_s;
                  obj(i).Radius = r;
                  obj(i).NewPosition = n_p;
                  obj(i).NewVelocity = n_p_s;
                  obj(i).NeighbourDistance = n_d;
                  obj(i).NeighbourAngle = n_a;
                  obj(i).Color = c;
                  obj(i).OldVelocity = o_v;
                  obj(i).Acceleration = ac;
                   
               end          
           end
       end
      
       function setNeighbourDistance(obj,vector_dist)
          obj.NeighbourDistance = vector_dist; 
       end
       
       function setNeighbourAngle(obj,vector_angle)
          obj.NeighbourAngle = vector_angle; 
       end
       
%       function that sets the speed of a specific agent
%         function setVelocity(obj)
%             obj.Velocity(1) = 1;
%             obj.Velocity(2) = 1; %unit velocity vector in the direction of target from agent's position
%         end
 
%      function that sets the preferred velocity of a specific agent
       function setPrefVelocity(obj)
            obj.PrefVelocity = obj.Target - obj.Position;
            obj.PrefVelocity = (obj.Velocity)*(obj.PrefVelocity/norm(obj.PrefVelocity));
            
       end
       
        % function that sets the new preferred velocity of a specific agent
        function setNewPrefVelocity(obj)
            obj.PrefVelocity = obj.Target - obj.NewPosition;
            obj.PrefVelocity = (obj.Velocity)*(obj.PrefVelocity/norm(obj.PrefVelocity));  
        end
       
       function setColor(obj,color)
          obj.Color = color; 
       end
       
       
       
       function [avoiding, a_net, dmin, a_long, a_lat] = FindVelocity(obj,others,t_iter,n,t_run,R)
          
            
            tH = 2;%changing th affects the radius calculated for intline and thus can give error when assertion fails,
            global delta;
            R_d = 0.3;
            R_D = R+R_d;
%             t_iter = 0.3;                                        %%  time till which computer is closed eyed (in each iteration of the loop the position of each of the obstacles is updated in t_iter time with the current velocty
            counto = 0 ;                                          % i.e. its assumed that the obstacle will continue moving with the current velocity for t_iter time )
            num_dyn_obs_hor = 0;                                   % counts the number of dynamic obstacle under time horizon
            t_iter_temp = 0;
            brake_flag = 0;
            
            
            distance(others); %finds the distance between each object and update to Neighbour Distance
            scan_angles(others)
            
            los_angles = obj.get(1,n);
%             disp(strcat('los_angles w.r.t. agent-',int2str(obj.Identity),'-->' ,los_angles));

            [ tag, d_critical, dmin] = obstacles_under_horizon(obj, others, tH, n);
            tag;
            num_dyn_obs_hor = size(tag,2); % counts the number of obstacles under the time horizon
%             disp(strcat('number of obstacles found w.r.t. agent-',int2str(obj.Identity),'-->' ,int2str(num_dyn_obs_hor)));
%             if obj.Identity == 1
%                 hold on;
%                 viscircles(obj.Position, 10,'EdgeColor', obj.Color, 'LineWidth', 2);
%             end
            
            vel_agent = sqrt((obj.NewVelocity(1))^2 + (obj.NewVelocity(1))^2);
            
            for j = 1:n
                if(obj.Identity ~= others(j).Identity)
                    vel_obs = sqrt((others(j).NewVelocity(1))^2 + (others(j).NewVelocity(1))^2);
                end
            end
            
            [right_of_way, vel] = Encounter_new2(obj,others, tag, R_D);
%             disp(strcat('Encouonter Type of agent-',int2str(obj.Identity),'-->',int2str(Et)));

            
            
            if (num_dyn_obs_hor >= 1) 
                if right_of_way && vel %&& deconflict %
%                     disp('Overall - Right-of-Way');
%                     t_iter = 0.6;
                    obj.OldVelocity = obj.NewVelocity;
                    obj.NewVelocity(1) = obj.PrefVelocity(1) - exp(-t_iter/delta)*(obj.PrefVelocity(1)-obj.OldVelocity(1)); %updating the new velocity of the agent
                    obj.NewVelocity(2) = obj.PrefVelocity(2) - exp(-t_iter/delta)*(obj.PrefVelocity(2)-obj.OldVelocity(2));
                    [obj.NewPosition(1), obj.NewPosition(2)] = pos_updater_agent(delta,obj,obj.NewVelocity(1),obj.NewVelocity(2),t_iter);
                    setNewPrefVelocity(obj);
%                     disp(strcat('Identity-->',int2str(obj.Identity)));
                    avoiding = 0;
%                     a_net = 0;
%                     a_long = 0;
%                     a_lat = 0;

                    a_x = (obj.PrefVelocity(1) - obj.NewVelocity(1))/delta;
                    a_y = (obj.PrefVelocity(2) - obj.NewVelocity(2))/delta;
                    v = [obj.NewVelocity(1), obj.NewVelocity(2)];
                    a = [a_x, a_y];
                    ang = angle_finder(v,a);
%                     cos_ang = (dot(v,a)/(norm(a)*norm(v)));
%                     sin_ang = (cross(v,a)/(norm(a)*norm(v)));

                    a_mag = norm(a);
                    a_long = -a_mag*cos(ang);
                    a_lat = a_mag*sin(ang);
                    a_net = (a_long^2 + a_lat^2);
                    obj.Acceleration = [a_long, a_lat];
                else
%                     disp('Overall - Avoid from right-side');

%                     [myMap_occupancy,occ_index,grid_size,resol,robo_origin, obs_start_pos_occ_grid,dia_agent] = eye_grid(obj.NeighbourDistance, obj.NeighbourAngle, (R_d), others, num_dyn_obs_hor,tag, d_critical);%%this function will work in any scenario
%                       [myMap_occupancy,occ_index,grid_size,resol,robo_origin, obs_start_pos_occ_grid,dia_agent] = deal(0,0,0,1,0,0,2*R);
% 
                    %AVO ALGORITHM
                    desVagent_x = NaN;
                    desVagent_y = NaN;
                    pose = [obj.Position(1); obj.Position(2)];
                    
%                    [desVagent_x, desVagent_y] = avo_safe_velocity(tag,obj,others,delta,t_iter,tH,R_D);
                   [desVagent_x, desVagent_y] = avo_safe_velocity_new(tag,obj,others,delta,t_iter,tH,R_D);

                    % POSITION UPDATE of the agent
%                     disp(strcat('Agent-',int2str(obj.Identity),'desired velocity-->', string(desVagent_x),':', string(desVagent_y)));
                    obj.OldVelocity = obj.NewVelocity;
                    [obj.NewPosition(1), obj.NewPosition(2)] = pos_updater_agent(delta,obj,desVagent_x, desVagent_y, t_iter); %updating the new positions of the agent
                    obj.NewVelocity(1) = desVagent_x - exp(-t_iter/delta)*(desVagent_x-obj.OldVelocity(1)); %updating the new velocity of the agent
                    obj.NewVelocity(2) = desVagent_y - exp(-t_iter/delta)*(desVagent_y-obj.OldVelocity(2));
                   
                    setNewPrefVelocity(obj); %updating the new prefer velocity of the agent to give it to agent when it find zero obstacle to move back to goal point
%                     disp(strcat('Identity-->',int2str(obj.Identity)));
%                     hold on;
%                     plot(obj.NewPosition(1),obj.NewPosition(2),'k*','MarkerSize',10);
%                     hold off;
                    avoiding = 1;
%                     if obj.Identity == 3
%                         hold on;
%                         plot(desVagent_x+obj.Position(1),desVagent_y+obj.Position(2),'r+');
%                         
%                     end

                    a_x = (desVagent_x - obj.NewVelocity(1))/delta;
                    a_y = (desVagent_y - obj.NewVelocity(2))/delta;
                    v = [obj.NewVelocity(1), obj.NewVelocity(2)];
                    a = [a_x, a_y];
                    ang = angle_finder(v,a);
% %                     ang = abs(ang);
%                     sin_ang_sq = 1-(cos_ang^2);
%                     if(sin_ang_sq<1e-5)
%                         sin_ang_sq=0;
%                     end
%                     
%                     sin_ang = sqrt(sin_ang_sq);

                    a_mag = norm(a);
                    a_long = -a_mag*cos(ang);
                    a_lat = a_mag*sin(ang);
                    a_net = (a_long^2 + a_lat^2);
                    obj.Acceleration = [a_long, a_lat];

                    a_long_real = isreal(a_long);
                    if a_long_real == 0
                        1;
                    end
                    a_lat_real = isreal(a_lat);
                    if a_lat_real == 0
                        1;
                    end


                end
                
            else %num_dyn_obs_hor equals to zero condition
%                 disp('Continue with same velocity');
%                 t_iter = 0.6;
                obj.OldVelocity = obj.NewVelocity;
                obj.NewVelocity(1) = obj.PrefVelocity(1) - exp(-t_iter/delta)*(obj.PrefVelocity(1)-obj.OldVelocity(1)); %updating the new velocity of the agent
                obj.NewVelocity(2) = obj.PrefVelocity(2) - exp(-t_iter/delta)*(obj.PrefVelocity(2)-obj.OldVelocity(2));
                [obj.NewPosition(1), obj.NewPosition(2)] = pos_updater_agent(delta,obj,obj.NewVelocity(1),obj.NewVelocity(2),t_iter);
                setNewPrefVelocity(obj);
%                 disp(strcat('Identity-->',int2str(obj.Identity)));
                avoiding = 0;
%                 a_net = 0;
%                 a_long = 0;
%                 a_lat = 0;

                 a_x = (obj.PrefVelocity(1) - obj.NewVelocity(1))/delta;
                 a_y = (obj.PrefVelocity(2) - obj.NewVelocity(2))/delta;
                 v = [obj.NewVelocity(1), obj.NewVelocity(2)];
                 a = [a_x, a_y];
                 ang = angle_finder(v,a);
%                  cos_ang = (dot(v,a)/(norm(a)*norm(v)));
%                  sin_ang = (cross(v,a)/(norm(a)*norm(v)));

                 a_mag = norm(a);
                 a_long = -a_mag*cos(ang);
                 a_lat = a_mag*sin(ang);
                 a_net = (a_long^2 + a_lat^2);
                 obj.Acceleration = [a_long, a_lat];
            end  
            
       end
        
       
       %printing for debugging
       function output = get(obj,k,n)
           output = "";
           if k == 1
               for m = 1:n-1
                   output = strcat(output, string(obj.NeighbourAngle(1,m)*180/pi), ":"); 
               end
           end
           
       end
       
       
       
    end
end
        
    
    