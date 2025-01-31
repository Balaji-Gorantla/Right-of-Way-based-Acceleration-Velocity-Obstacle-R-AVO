function [right_of_way, vel] =  Encounter_new2(agent,others, tag, R_D)

    right_of_way = false;
    vel = true;
    ROW1 = [];
    ROW = [];
    count1 = 0;
    count2 = 0;
    Dist_count = 0;

    global B1 B2 B3 B4 RoW_Matrix;
    
    alphai = atan2(agent.NewVelocity(2),agent.NewVelocity(1)); % agent's heading
    if alphai<0
       alphai = alphai+2*pi;
    end
    
    for j = 1:length(tag)
    
        theta = atan2((others(tag(j)).Position(2) - agent.Position(2)), (others(tag(j)).Position(1) - agent.Position(1))); %los angles
        if theta<0
           theta = theta+2*pi;
        end
    
        alphaj = atan2(others(tag(j)).NewVelocity(2),others(tag(j)).NewVelocity(1)); % other agent's heading
        if alphaj<0
           alphaj = alphaj+2*pi;
        end
    
        beta = mod((theta - alphai + pi ),2*pi) + pi; %beta1
        if beta>2*pi
            beta = beta - 2*pi;
        end
    
        alpha = mod((alphaj - alphai + pi ),2*pi) + pi; 
        if alpha>2*pi
            alpha = alpha - 2*pi;
        end
        
        beta_region = get_conflict_region(B1,B2,B3,B4,beta); 
        alpha_region = get_conflict_region(B1,B2,B3,B4,alpha); 
        
        Dist_Agents = norm(others(tag(j)).Position - agent.Position);

        ROW1 = RoW_Matrix(alpha_region,beta_region);

        if (Dist_Agents < 2*R_D) && (beta_region == 4) && (alpha_region == 1)
            ROW1 = 0;
        end
        
        if (Dist_Agents >= 2*R_D) && (beta_region == 1 && alpha_region == 1) && (norm(agent.NewVelocity) <= norm(others(tag(j)).NewVelocity))
            ROW1 = 1;
        end
        
        if beta_region == 1 && (norm(agent.Position - agent.Target) < Dist_Agents)
            ROW1 = 1;
        end
        
        %new case -------- testing
%         if (Dist_Agents >= 0.6*2*R_D) && (beta_region == 1) && (alpha_region == 3)
%             ROW1 = 1;
%         end
%     
% %         if (Dist_Agents >=2*R_D) && (beta_region == 1) && (alpha_region == 4)
% %             ROW1 = 1;
% %         end
% 
%         if (Dist_Agents >= 2*R_D) && (beta_region == 4) && (alpha_region == 2)
%             ROW1 = 1;
%         end
        %-----------------------------------------------------

%         Display(alpha_region,beta_region,others(tag(j)).Identity,agent,ROW1);
        
        ROW = [ROW, ROW1];

        V_age_i = norm(agent.NewVelocity); % (sqrt((agent.NewVelocity(2))^2 + (agent.NewVelocity(1))^2));
        V_age_j = norm(others(tag(j)).NewVelocity); % (sqrt((others(tag(j)).NewVelocity(2))^2 + (others(tag(j)).NewVelocity(1))^2));

        %         disp(strcat('Velocity of Agent i --> ', int2str(V_age_i)));
        %         disp(strcat('Velocity of Agent j --> ', int2str(V_age_j)));
       
        

        if ((others(tag(j)).NewVelocity(1)) == 0) && ((others(tag(j)).NewVelocity(2)) == 0)
            
            if  (beta_region == 3) || (Dist_Agents >= 2*R_D) 
                vel = true;
%                 disp('Static obstcale but no problem');
            else
                vel = false;
%                 disp('Static obstcale');
            end
            
        end

    end

%     for k = 1:size(ROW1,2)
%        if ROW1(1,k) == 1
%            count1 = count1 +1;
% %            disp(int2str(count1));
%        elseif ROW1(1,k) == 0
%            count2 = count2 + 1;
%        end    
%     end

    if length(ROW) == sum(ROW)
%         disp(int2str(length(ROW)));
        right_of_way = true;  
%         disp('Right-of-Way');
    end

end