function distance(agents)
%%% function that calculates the a between each agent
% input: agents

    for i=1:length(agents) 
         for j=1:length(agents)
             %position between agent i and agent j
            Pos=[agents(i).Position; agents(j).Position];
            dist(i,j)=pdist(Pos,'euclidean');
        end
            vector_dist=dist(i,:); % I extract the vector from the matrix
            agents(i).setNeighbourDistance(vector_dist); % send the vector to the agent
            agents(i).NeighbourDistance( :, ~any(agents(i).NeighbourDistance,1) ) = [];  %columns
    end
    
end