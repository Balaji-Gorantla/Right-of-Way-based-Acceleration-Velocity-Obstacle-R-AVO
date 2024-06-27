function scan_angles(agents)
%%% function that calculates the LOS angle between each agent
% input: agents
% output: row vector of LOS angles that will updated to the agents property
% of Neighbour Angles

    for i=1:length(agents) 
         for j=1:length(agents)
             %position between agent i and agent j
                Pos=[agents(i).Position; agents(j).Position];
                angle(i,j)=atan2((agents(j).Position(2) - agents(i).Position(2)), (agents(j).Position(1) - agents(i).Position(1)));
                
                if angle(i,j)< 0
                    angle(i,j) = angle(i,j) + 2*pi;
                end
         end
         
        vector_angle=angle(i,:); % extract the vector from the matrix
        agents(i).setNeighbourAngle(vector_angle); % send the LOS angle vector to the agent
        
%         data( ~any(data,2), : ) = [];  %rows
        agents(i).NeighbourAngle( :, agents(i).Identity ) = [];  %columns
    end
    
end